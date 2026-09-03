// wifi_link.cpp -- see wifi_link.h. Host-portable: no pxt.h, no CODAL.
#include "wifi_link.h"

#include <cstdio>   // plain snprintf -- newlib-nano's <cstdio> never puts
                    // it in namespace std (same gotcha wire_handler.cpp
                    // and protocol.cpp document for their own copies)
#include <cstring>

namespace diffDrive {

namespace {

// (command, expect, timeout, tolerant) -- the wifi-link note, section 5.2,
// verbatim. AT+RST first because the RJ11-powered module keeps its AP
// join / server / socket state across an nRF reset; the teardown steps
// are tolerated because on a fresh module they answer ERROR ("nothing
// to close"), which is expected, not a fault.
struct AtStep {
  const char* command;
  const char* expect;
  uint32_t timeoutMs;
  bool tolerant;
};

const AtStep kConfigureSteps[] = {
    {"AT+RST", "ready", 6000, true},
    {"AT", "OK", 2000, true},  // absorbs boot-banner stragglers
    {"ATE0", "OK", WifiLink::kCommandTimeoutMs, true},
    {"AT+CIPMODE=0", "OK", WifiLink::kCommandTimeoutMs, true},
    {"AT+CIPSERVER=0", "OK", WifiLink::kCommandTimeoutMs, true},
    {"AT+CIPCLOSE=5", "OK", WifiLink::kCommandTimeoutMs, true},
    {"AT+CIPCLOSE", "OK", WifiLink::kCommandTimeoutMs, true},
    {"AT+CWMODE=1", "OK", WifiLink::kCommandTimeoutMs, false},
    {"AT+CIPMUX=1", "OK", WifiLink::kCommandTimeoutMs, false},
    {"AT+CIPDINFO=1", "OK", WifiLink::kCommandTimeoutMs, true},
};
constexpr uint8_t kConfigureStepCount =
    sizeof(kConfigureSteps) / sizeof(kConfigureSteps[0]);

const char kMdnsAddress[] = "224.0.0.251";
constexpr uint16_t kMdnsPort = 5353;
constexpr uint32_t kMdnsTtlSeconds = 120;

void copyBounded(char* dst, size_t cap, const char* src) {
  size_t i = 0;
  for (; src[i] != '\0' && i + 1 < cap; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

}  // namespace

// ---------------------------------------------------------------------------
// Matcher
// ---------------------------------------------------------------------------

void WifiLink::Matcher::reset(const char* token) {
  token_ = token;
  matched_ = 0;
}

bool WifiLink::Matcher::feed(char c) {
  if (token_ == nullptr || token_[0] == '\0') return false;
  if (c == token_[matched_]) {
    ++matched_;
    if (token_[matched_] == '\0') {
      matched_ = 0;  // re-arm: a token may legitimately arrive twice
      return true;
    }
    return false;
  }
  // Restart AT THIS BYTE rather than zeroing, so "OOK" still matches
  // "OK". The tokens here are short with no internal repetition, so a
  // one-byte retry is a complete answer (no KMP needed).
  matched_ = (c == token_[0]) ? 1 : 0;
  return false;
}

// ---------------------------------------------------------------------------
// IpdParser -- `+IPD,<link>,<len>:` and `+IPD,<link>,<len>,"<ip>",<port>:`
// ---------------------------------------------------------------------------

void WifiLink::IpdParser::reset() {
  stage_ = kTag;
  tag_.reset("+IPD,");
  link_ = -1;
  length_ = 0;
  sawDigit_ = false;
  ip_[0] = '\0';
  ipLen_ = 0;
  port_ = 0;
  portSawDigit_ = false;
}

bool WifiLink::IpdParser::feed(char c) {
  switch (stage_) {
    case kTag:
      if (tag_.feed(c)) {
        stage_ = kLink;
        link_ = 0;
        sawDigit_ = false;
      }
      return false;
    case kLink:
      if (c >= '0' && c <= '9') {
        link_ = link_ * 10 + (c - '0');
        sawDigit_ = true;
        return false;
      }
      if (c == ',' && sawDigit_) {
        stage_ = kLen;
        length_ = 0;
        sawDigit_ = false;
        return false;
      }
      reset();
      return false;
    case kLen:
      if (c >= '0' && c <= '9') {
        length_ = length_ * 10 + static_cast<size_t>(c - '0');
        sawDigit_ = true;
        return false;
      }
      if (c == ':' && sawDigit_) {
        stage_ = kDone;
        return true;
      }
      if (c == ',' && sawDigit_) {
        stage_ = kToQuote;
        return false;
      }
      reset();
      return false;
    case kToQuote:
      if (c == '"') {
        stage_ = kIp;
        ipLen_ = 0;
        return false;
      }
      reset();
      return false;
    case kIp:
      if (c == '"') {
        ip_[ipLen_] = '\0';
        stage_ = kToPort;
        return false;
      }
      if (ipLen_ < sizeof(ip_) - 1) {
        ip_[ipLen_++] = c;
        return false;
      }
      reset();  // not an address -- resynchronize
      return false;
    case kToPort:
      if (c == ',') {
        stage_ = kPort;
        port_ = 0;
        portSawDigit_ = false;
        return false;
      }
      reset();
      return false;
    case kPort:
      if (c >= '0' && c <= '9') {
        port_ = static_cast<uint16_t>(port_ * 10 + (c - '0'));
        portSawDigit_ = true;
        return false;
      }
      if (c == ':' && portSawDigit_) {
        stage_ = kDone;
        return true;
      }
      reset();
      return false;
    case kDone:
      return false;  // inert until the next reset()
  }
  return false;
}

// ---------------------------------------------------------------------------
// WifiLink
// ---------------------------------------------------------------------------

WifiLink::WifiLink(WifiUart& uart, NowMsFn nowMs)
    : uart_(uart), nowMs_(nowMs), config_(), state_(kDisabled), step_(0),
      joinQueryAttempt_(0), restartCount_(0), stateChanged_(false),
      awaiting_(false), deadline_(0), awaitMatched_(false),
      awaitRejected_(false), payloadRemaining_(0), payloadLink_(-1),
      statusLen_(0), ownIpCapturing_(false), ownIpLen_(0), peerPort_(0),
      peerKnown_(false), lastPeerHeardMs_(0), reportedPeerPort_(0),
      rxHead_(0), rxCount_(0), tcpOpenMask_(0), replyLink_(kProtocolLink),
      tcpConnectEdge_(false), tcpServerOpen_(false), telemetryMode_(false),
      txHead_(0), txCount_(0), sendPhase_(kIdle),
      lastTelemetryMs_(0), telemetryEverSent_(false), mdnsSocketOpen_(false),
      lastMdnsMs_(0), mdnsAnnounceCount_(0), dropCount_(0), sentCount_(0),
      receivedCount_(0), promptRetryDeadline_(0), lastReplyLen_(0),
      stageLen_(0), stagePos_(0) {
  expectBuf_[0] = '\0';
  payload_.len = 0;
  payload_.link = -1;
  for (int i = 0; i < kMaxTcpLinks; ++i) {
    tcpLineLen_[i] = 0;
    tcpLineOverflow_[i] = false;
  }
  ownIp_[0] = '\0';
  peerIp_[0] = '\0';
  reportedPeerIp_[0] = '\0';
  lastCommand_[0] = '\0';
  lastReply_[0] = '\0';
  commandBuf_[0] = '\0';
  inFlight_.link = -1;
  inFlight_.telemetry = false;
  inFlight_.slot.len = 0;
  ownIpTag_.reset("ip:\"");
}

void WifiLink::begin(const Config& config) {
  config_ = config;
  if (config_.ssid == nullptr || config_.ssid[0] == '\0') {
    enterState(kDisabled);
    return;
  }
  uart_.begin(115200);
  peerKnown_ = false;
  peerIp_[0] = '\0';
  peerPort_ = 0;
  ownIp_[0] = '\0';
  mdnsSocketOpen_ = false;
  tcpServerOpen_ = false;
  tcpOpenMask_ = 0;
  replyLink_ = kProtocolLink;
  txCount_ = 0;
  txHead_ = 0;
  sendPhase_ = kIdle;
  enterState(kConfigure);
}

void WifiLink::enterState(State next) {
  if (state_ != next) stateChanged_ = true;
  state_ = next;
  step_ = 0;
  awaiting_ = false;
  ipd_.reset();
  statusLen_ = 0;
  payloadRemaining_ = 0;
  payload_.len = 0;
}

void WifiLink::enterBackoff() {
  ++restartCount_;
  // Tear down peer/socket state: the next kConfigure pass's AT+RST wipes
  // every ESP-AT socket, so nothing queued can ever be delivered.
  peerKnown_ = false;
  peerIp_[0] = '\0';
  peerPort_ = 0;
  mdnsSocketOpen_ = false;
  tcpServerOpen_ = false;
  tcpOpenMask_ = 0;
  replyLink_ = kProtocolLink;
  txCount_ = 0;
  txHead_ = 0;
  sendPhase_ = kIdle;
  deadline_ = nowMs() + kBackoffDelayMs;
  enterState(kBackoff);
}

bool WifiLink::pollStateChanged() {
  const bool changed = stateChanged_;
  stateChanged_ = false;
  return changed;
}

// --- AT command / await ------------------------------------------------------

void WifiLink::traceReply(char c) {
  if (lastReplyLen_ + 1 >= kTraceReply) return;  // keep the FIRST bytes
  lastReply_[lastReplyLen_++] = (c >= 0x20 && c < 0x7F) ? c : '.';
  lastReply_[lastReplyLen_] = '\0';
}

void WifiLink::startAwait(const char* expect, uint32_t timeoutMs) {
  expect_.reset(expect);
  rejectError_.reset("ERROR");
  rejectFail_.reset("FAIL");
  rejectBusy_.reset("busy");
  awaitMatched_ = false;
  awaitRejected_ = false;
  deadline_ = nowMs() + timeoutMs;
  awaiting_ = true;
}

bool WifiLink::startCommand(const char* command, const char* expect,
                            uint32_t timeoutMs) {
  const int n = snprintf(commandBuf_, sizeof(commandBuf_), "%s\r\n", command);
  if (n <= 0 || static_cast<unsigned>(n) >= sizeof(commandBuf_)) return false;
  if (!uart_.write(reinterpret_cast<const uint8_t*>(commandBuf_),
                   static_cast<uint16_t>(n))) {
    return false;  // TX full -- caller retries next poll, state unchanged
  }
  copyBounded(lastCommand_, sizeof(lastCommand_), command);
  lastReply_[0] = '\0';
  lastReplyLen_ = 0;
  startAwait(expect, timeoutMs);
  return true;
}

WifiLink::Await WifiLink::pollAwait() {
  if (!awaiting_) return kTimedOut;
  if (awaitMatched_) {
    awaiting_ = false;
    return kMatched;
  }
  if (awaitRejected_) {
    awaiting_ = false;
    return kRejected;
  }
  if (static_cast<int32_t>(nowMs() - deadline_) >= 0) {
    awaiting_ = false;
    return kTimedOut;
  }
  return kPending;
}

// --- inbound demux -----------------------------------------------------------

void WifiLink::pumpIncoming() {
  // Bounded per call: at most a few stage buffers' worth, so a module
  // streaming garbage cannot pin the protocol fiber in this loop.
  for (int round = 0; round < 8; ++round) {
    stageLen_ = uart_.read(stage_, kStageBuffer);
    stagePos_ = 0;
    if (stageLen_ == 0) return;
    while (stagePos_ < stageLen_) feedByte(stage_[stagePos_++]);
  }
}

void WifiLink::feedByte(uint8_t c) {
  // 1. Payload capture: binary-safe, never reaches the matchers below.
  if (payloadRemaining_ > 0) {
    if (payload_.len < kSlotBytes) payload_.data[payload_.len++] = c;
    --payloadRemaining_;
    if (payloadRemaining_ == 0) finishPayload();
    return;
  }
  // 2. +IPD header parse. The header's own characters have also been
  // accumulating in the status-line buffer (step 3 sees every
  // non-payload byte); a completed header is not a status line, so
  // that buffer is cleared here -- otherwise a `0,CONNECT` following a
  // payload with no newline between them would be glued onto the
  // header text and never parse.
  if (ipd_.feed(static_cast<char>(c))) {
    statusLen_ = 0;
    payloadRemaining_ = ipd_.length();
    payloadLink_ = ipd_.link();
    payload_.len = 0;
    if (payloadLink_ == kProtocolLink) {
      // Peer learned/refreshed off the HEADER alone -- an empty datagram
      // still counts as heard-from (the wifi-link note, section 6.1).
      if (ipd_.ip()[0] != '\0') copyBounded(peerIp_, sizeof(peerIp_), ipd_.ip());
      if (ipd_.port() != 0) peerPort_ = ipd_.port();
      lastPeerHeardMs_ = nowMs();
      peerKnown_ = (peerIp_[0] != '\0' && peerPort_ != 0);
    }
    ipd_.reset();
    if (payloadRemaining_ == 0) finishPayload();
    return;
  }
  // 3. Status lines (trace only -- no TCP clients to track here).
  feedStatusByte(c);
  // 4. Own-IP capture from `+CIPSTA:ip:"<addr>"`.
  if (ownIpCapturing_) {
    if (c == '"' || ownIpLen_ >= sizeof(ownIp_) - 1) {
      ownIp_[ownIpLen_] = '\0';
      ownIpCapturing_ = false;
    } else {
      ownIp_[ownIpLen_++] = static_cast<char>(c);
    }
  } else if (ownIpTag_.feed(static_cast<char>(c))) {
    ownIpCapturing_ = true;
    ownIpLen_ = 0;
  }
  // 5. AT reply matchers.
  if (awaiting_) {
    traceReply(static_cast<char>(c));
    if (expect_.feed(static_cast<char>(c))) awaitMatched_ = true;
    if (rejectError_.feed(static_cast<char>(c)) ||
        rejectFail_.feed(static_cast<char>(c)) ||
        rejectBusy_.feed(static_cast<char>(c))) {
      awaitRejected_ = true;
    }
  }
}

void WifiLink::feedStatusByte(uint8_t c) {
  if (c == '\r') return;
  if (c == '\n') {
    handleStatusLine();
    statusLen_ = 0;
    return;
  }
  if (statusLen_ + 1 < kStatusLineMax) {
    statusLine_[statusLen_++] = c;
  } else {
    statusLen_ = 0;  // overlong -- discard and resync
  }
}

// `<link>,CONNECT` / `<link>,CLOSED` -- the module's TCP client
// lifecycle, one line each. Links 3/4 are our own sockets and report the
// same lines for their own lifecycle; those are ignored here.
void WifiLink::handleStatusLine() {
  if (statusLen_ < 3) return;
  const uint8_t* s = statusLine_;
  if (s[0] < '0' || s[0] > '9' || s[1] != ',') return;
  const int link = s[0] - '0';
  if (link < 0 || link >= kMaxTcpLinks) return;
  const size_t restLen = statusLen_ - 2;
  const bool connect = (restLen == 7 && memcmp(s + 2, "CONNECT", 7) == 0);
  const bool closed = (restLen == 6 && memcmp(s + 2, "CLOSED", 6) == 0);
  if (connect) {
    tcpOpenMask_ |= static_cast<uint8_t>(1u << link);
    tcpLineLen_[link] = 0;
    tcpLineOverflow_[link] = false;
    replyLink_ = static_cast<int8_t>(link);  // newest client wins
    tcpConnectEdge_ = true;
  } else if (closed) {
    tcpOpenMask_ &= static_cast<uint8_t>(~(1u << link));
    tcpLineLen_[link] = 0;
    if (replyLink_ == link) replyLink_ = kProtocolLink;  // back to the UDP host
  }
}

void WifiLink::pushRxLine(int link, const uint8_t* data, size_t len) {
  ++receivedCount_;
  if (rxCount_ >= kRxSlots) {
    ++dropCount_;  // host out-ran the poll -- drop newest, counted
    return;
  }
  Slot& slot = rx_[(rxHead_ + rxCount_) % kRxSlots];
  if (len > kSlotBytes) len = kSlotBytes;
  slot.len = static_cast<uint16_t>(len);
  slot.link = static_cast<int8_t>(link);
  memcpy(slot.data, data, len);
  ++rxCount_;
}

void WifiLink::finishPayload() {
  if (payloadLink_ == kProtocolLink) {
    // UDP: one datagram IS one line.
    pushRxLine(kProtocolLink, payload_.data, payload_.len);
  } else if (payloadLink_ >= 0 && payloadLink_ < kMaxTcpLinks &&
             (tcpOpenMask_ & (1u << payloadLink_))) {
    // TCP: a byte stream -- accumulate to '\n' per client, exactly as
    // SerialTransport::tryReadLine() does for USB. An overlong line is
    // discarded whole (never truncated into a shorter legal command).
    const int link = payloadLink_;
    for (size_t i = 0; i < payload_.len; ++i) {
      const uint8_t c = payload_.data[i];
      if (c == '\n') {
        if (!tcpLineOverflow_[link]) {
          pushRxLine(link, tcpLine_[link], tcpLineLen_[link]);
        } else {
          ++dropCount_;
        }
        tcpLineLen_[link] = 0;
        tcpLineOverflow_[link] = false;
      } else if (tcpLineLen_[link] < kMaxLineBytes) {
        tcpLine_[link][tcpLineLen_[link]++] = c;
      } else {
        tcpLineOverflow_[link] = true;
      }
    }
  }
  // any other link (the mDNS socket hears nothing useful) -- dropped
  payload_.len = 0;
  payloadLink_ = -1;
}

// --- peer ------------------------------------------------------------------

bool WifiLink::peerKnown() {
  if (!peerKnown_) return false;
  if (static_cast<int32_t>(nowMs() - (lastPeerHeardMs_ + kPeerSilenceMs)) >= 0) {
    peerKnown_ = false;
    peerIp_[0] = '\0';
    peerPort_ = 0;
    return false;
  }
  return true;
}

bool WifiLink::pollNewPeerEdge() {
  if (!peerKnown()) return false;
  if (strcmp(peerIp_, reportedPeerIp_) == 0 && peerPort_ == reportedPeerPort_) {
    return false;
  }
  copyBounded(reportedPeerIp_, sizeof(reportedPeerIp_), peerIp_);
  reportedPeerPort_ = peerPort_;
  return true;
}

bool WifiLink::pollNewClientEdge() {
  const bool edge = tcpConnectEdge_;
  tcpConnectEdge_ = false;
  return edge;
}

// --- bring-up states -----------------------------------------------------------

void WifiLink::serviceConfigure() {
  if (step_ >= kConfigureStepCount) {
    enterState(kJoin);
    return;
  }
  const AtStep& s = kConfigureSteps[step_];
  if (!awaiting_) {
    startCommand(s.command, s.expect, s.timeoutMs);
    return;
  }
  const Await outcome = pollAwait();
  if (outcome == kPending) return;
  if (outcome != kMatched && !s.tolerant) {
    enterBackoff();
    return;
  }
  if (++step_ >= kConfigureStepCount) enterState(kJoin);
}

void WifiLink::serviceJoin() {
  if (step_ == 0) {
    // LANDMINE (the wifi-link note, section 5.3): poll AT+CWJAP? first so the
    // module's own post-RST auto-rejoin can land. An explicit CWJAP
    // fired into an in-progress auto-join answers busy/ERROR and was
    // measured producing a join->backoff->RST near-livelock.
    if (!awaiting_) {
      // The expect token must outlive the await, and commandBuf_ is
      // reused by startCommand() -- so it gets its own member buffer.
      snprintf(expectBuf_, sizeof(expectBuf_), "+CWJAP:\"%s\"", config_.ssid);
      startCommand("AT+CWJAP?", expectBuf_, kJoinQueryMs);
      return;
    }
    const Await outcome = pollAwait();
    if (outcome == kMatched) {
      joinQueryAttempt_ = 0;
      enterState(kAddress);
      return;
    }
    if (outcome == kPending) return;
    if (++joinQueryAttempt_ < kJoinQueryAttempts) {
      awaiting_ = false;  // re-query; the auto-join takes a few seconds
      return;
    }
    joinQueryAttempt_ = 0;
    step_ = 1;
    awaiting_ = false;
    return;
  }
  if (!awaiting_) {
    char cmd[kCommandBuffer];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", config_.ssid,
             config_.password);
    startCommand(cmd, "OK", kJoinTimeoutMs);
    return;
  }
  const Await outcome = pollAwait();
  if (outcome == kMatched) {
    enterState(kAddress);
    return;
  }
  if (outcome == kPending) return;
  enterBackoff();  // +CWJAP:<code>/ERROR -- AP not up yet, wrong SSID, ...
}

void WifiLink::serviceAddress() {
  if (step_ == 0) {
    if (!awaiting_) {
      startCommand("AT+CWDHCP=1,1", "OK", kCommandTimeoutMs);
      return;
    }
    if (pollAwait() == kPending) return;
    step_ = 1;  // tolerant of match/reject/timeout alike
    awaiting_ = false;
    return;
  }
  // step 1: learn our own address for the mDNS A record. Tolerant: a
  // link with no known address still carries protocol traffic; it just
  // cannot announce itself.
  if (!awaiting_) {
    ownIp_[0] = '\0';
    ownIpLen_ = 0;
    ownIpCapturing_ = false;
    ownIpTag_.reset("ip:\"");
    startCommand("AT+CIPSTA?", "OK", kCommandTimeoutMs);
    return;
  }
  if (pollAwait() == kPending) return;
  enterState(kSocket);
}

void WifiLink::serviceSocket() {
  if (step_ == 0) {
    if (!awaiting_) {
      // Protocol plane: link 4, remote = the host's fixed port on the
      // broadcast address as a placeholder (no peer known yet), local
      // port = ours, UDP mode 2 (remote endpoint follows the last
      // sender). Strict.
      snprintf(commandBuf_, sizeof(commandBuf_),
               "AT+CIPSTART=%d,\"UDP\",\"255.255.255.255\",%u,%u,2",
               kProtocolLink, static_cast<unsigned>(config_.hostPort),
               static_cast<unsigned>(config_.port));
      char cmd[kCommandBuffer];
      copyBounded(cmd, sizeof(cmd), commandBuf_);
      startCommand(cmd, "OK", kCommandTimeoutMs);
      return;
    }
    const Await outcome = pollAwait();
    if (outcome == kPending) return;
    if (outcome != kMatched) {
      enterBackoff();
      return;
    }
    step_ = 1;
    awaiting_ = false;
    return;
  }
  // step 1: the mDNS announcer's multicast socket. Tolerant -- a module
  // that refuses it still carries the protocol; the robot is simply not
  // discoverable by name.
  if (step_ == 1 && !awaiting_) {
    snprintf(commandBuf_, sizeof(commandBuf_),
             "AT+CIPSTART=%d,\"UDP\",\"%s\",%u,%u,0", kMdnsLink, kMdnsAddress,
             static_cast<unsigned>(kMdnsPort), static_cast<unsigned>(kMdnsPort));
    char cmd[kCommandBuffer];
    copyBounded(cmd, sizeof(cmd), commandBuf_);
    startCommand(cmd, "OK", kCommandTimeoutMs);
    return;
  }
  if (step_ == 1) {
    const Await outcome = pollAwait();
    if (outcome == kPending) return;
    mdnsSocketOpen_ = (outcome == kMatched);
    if (!config_.tcpServer) {
      lastMdnsMs_ = nowMs() - kMdnsPeriodMs;  // announce on the first ready pass
      enterState(kReady);
      return;
    }
    step_ = 2;
    awaiting_ = false;
    return;
  }
  if (step_ == 2) {
    // The TCP server on the same port number: a client's socket is
    // then a line stream, like USB, with no keepalive or fixed host
    // port to remember. Tolerant -- a module that refuses still serves
    // the UDP plane.
    if (!awaiting_) {
      snprintf(commandBuf_, sizeof(commandBuf_), "AT+CIPSERVER=1,%u",
               static_cast<unsigned>(config_.port));
      char cmd[kCommandBuffer];
      copyBounded(cmd, sizeof(cmd), commandBuf_);
      startCommand(cmd, "OK", kCommandTimeoutMs);
      return;
    }
    const Await outcome = pollAwait();
    if (outcome == kPending) return;
    tcpServerOpen_ = (outcome == kMatched);
    step_ = 3;
    awaiting_ = false;
    return;
  }
  // step 3: never time an idle TCP client out (the ESP-AT default
  // closes one after 180 s of silence). Tolerant: an unsupported verb
  // just leaves the default in place.
  if (!awaiting_) {
    startCommand("AT+CIPSTO=0", "OK", kCommandTimeoutMs);
    return;
  }
  if (pollAwait() == kPending) return;
  lastMdnsMs_ = nowMs() - kMdnsPeriodMs;  // announce on the first ready pass
  enterState(kReady);
}

void WifiLink::serviceReady() {
  // The two-phase, non-blocking send exchange (the wifi-link note, section 7):
  // AT+CIPSEND=... -> '>' prompt -> payload -> SEND OK. Reject or timeout
  // at either phase DROPS the datagram and moves on.
  if (sendPhase_ == kAwaitPrompt) {
    const Await outcome = pollAwait();
    if (outcome == kMatched) {
      // The module is at its '>' prompt and now expects exactly
      // slot.len bytes. If the TX ring cannot take them right now, keep
      // the prompt "open" (re-arm the same await with a short deadline
      // -- the module will not speak again until it has the payload)
      // and retry the write next poll; give up at the deadline rather
      // than leaving the module waiting forever.
      if (!uart_.write(inFlight_.slot.data, inFlight_.slot.len)) {
        awaiting_ = true;
        awaitMatched_ = false;
        awaitRejected_ = false;
        if (promptRetryDeadline_ == 0) promptRetryDeadline_ = nowMs() + 200;
        if (static_cast<int32_t>(nowMs() - promptRetryDeadline_) >= 0) {
          ++dropCount_;
          sendPhase_ = kIdle;
          awaiting_ = false;
          promptRetryDeadline_ = 0;
        } else {
          awaitMatched_ = true;  // so the next pollAwait() reports the prompt again
        }
        return;
      }
      promptRetryDeadline_ = 0;
      sendPhase_ = kAwaitSendOk;
      lastReply_[0] = '\0';
      lastReplyLen_ = 0;
      startAwait("SEND OK", kCommandTimeoutMs);
    } else if (outcome != kPending) {
      ++dropCount_;
      sendPhase_ = kIdle;
    }
    return;
  }
  if (sendPhase_ == kAwaitSendOk) {
    const Await outcome = pollAwait();
    if (outcome == kPending) return;
    if (outcome == kMatched) ++sentCount_; else ++dropCount_;
    sendPhase_ = kIdle;
    return;
  }

  // Idle: periodic mDNS announcement, at lower priority than protocol
  // traffic (only when the queue is empty).
  if (mdnsSocketOpen_ && txCount_ == 0 && ownIp_[0] != '\0' &&
      static_cast<int32_t>(nowMs() - (lastMdnsMs_ + kMdnsPeriodMs)) >= 0) {
    queueMdnsAnnouncement();
    lastMdnsMs_ = nowMs();
  }
  if (txCount_ > 0) popNextSend();
}

void WifiLink::serviceBackoff() {
  if (static_cast<int32_t>(nowMs() - deadline_) < 0) return;
  enterState(kConfigure);
}

void WifiLink::service() {
  if (state_ == kDisabled) return;
  pumpIncoming();  // keep draining in every state so stray bytes never wedge the next one
  switch (state_) {
    case kDisabled: return;
    case kConfigure: serviceConfigure(); return;
    case kJoin: serviceJoin(); return;
    case kAddress: serviceAddress(); return;
    case kSocket: serviceSocket(); return;
    case kReady: serviceReady(); return;
    case kBackoff: serviceBackoff(); return;
  }
}

// --- the transport surface -----------------------------------------------------

bool WifiLink::tryReceiveLine(uint8_t* outBuf, size_t outCap, size_t* outLen) {
  if (rxCount_ == 0) return false;
  Slot& slot = rx_[rxHead_];
  rxHead_ = (rxHead_ + 1) % kRxSlots;
  --rxCount_;
  replyLink_ = slot.link;  // replies go back the way the line came
  size_t n = slot.len;
  while (n > 0 && (slot.data[n - 1] == '\n' || slot.data[n - 1] == '\r')) --n;
  if (n > outCap) n = outCap;
  if (outBuf != nullptr && n > 0) memcpy(outBuf, slot.data, n);
  if (outLen != nullptr) *outLen = n;
  return true;
}

bool WifiLink::enqueueSend(int link, const uint8_t* data, size_t len) {
  if (len > kSlotBytes) len = kSlotBytes;
  if (txCount_ >= kTxSlots) {
    ++dropCount_;
    return false;  // drop NEWEST -- stale data is not worth a stall
  }
  TxEntry& e = tx_[(txHead_ + txCount_) % kTxSlots];
  e.link = link;
  e.slot.len = static_cast<uint16_t>(len);
  memcpy(e.slot.data, data, len);
  ++txCount_;
  return true;
}

bool WifiLink::sendLine(const uint8_t* data, size_t len) {
  if (state_ != kReady) return false;
  int link = replyLink_;
  if (link >= 0 && link < kMaxTcpLinks) {
    if (!(tcpOpenMask_ & (1u << link))) {
      link = kProtocolLink;  // that client is gone; fall back to the UDP host
      replyLink_ = kProtocolLink;
    }
  }
  if (link == kProtocolLink && !peerKnown()) return false;
  if (len > kMaxLineBytes) len = kMaxLineBytes;
  // A reply never waits behind stale frames: discard whatever periodic
  // telemetry is still queued (see telemetryAllowed()'s comment).
  if (!telemetryMode_) purgeTelemetry();
  // One memcpy into the ring, never a stack copy (the protocol fiber's
  // stack is small -- see radio_transport.h).
  if (txCount_ >= kTxSlots) {
    ++dropCount_;
    return false;
  }
  TxEntry& e = tx_[(txHead_ + txCount_) % kTxSlots];
  e.link = link;
  e.telemetry = telemetryMode_;
  memcpy(e.slot.data, data, len);
  e.slot.data[len] = '\n';
  e.slot.len = static_cast<uint16_t>(len + 1);
  ++txCount_;
  return true;
}

void WifiLink::purgeTelemetry() {
  int kept = 0;
  for (int i = 0; i < txCount_; ++i) {
    TxEntry& e = tx_[(txHead_ + i) % kTxSlots];
    if (e.telemetry) continue;
    if (kept != i) tx_[(txHead_ + kept) % kTxSlots] = e;
    ++kept;
  }
  txCount_ = kept;
}

bool WifiLink::telemetryAllowed() {
  if (state_ != kReady) return false;
  const int link = replyLink_;
  const bool tcpTarget = link >= 0 && link < kMaxTcpLinks && (tcpOpenMask_ & (1u << link));
  if (!tcpTarget && !peerKnown()) return false;
  if (txCount_ != 0 || sendPhase_ != kIdle) return false;  // engine must be idle
  const uint32_t now = nowMs();
  if (telemetryEverSent_ &&
      static_cast<int32_t>(now - (lastTelemetryMs_ + kTelemetryMinIntervalMs)) < 0) {
    return false;
  }
  lastTelemetryMs_ = now;
  telemetryEverSent_ = true;
  return true;
}

void WifiLink::popNextSend() {
  inFlight_ = tx_[txHead_];
  txHead_ = (txHead_ + 1) % kTxSlots;
  --txCount_;
  if (inFlight_.link == kProtocolLink) {
    snprintf(commandBuf_, sizeof(commandBuf_), "AT+CIPSEND=%d,%u,\"%s\",%u",
             kProtocolLink, static_cast<unsigned>(inFlight_.slot.len), peerIp_,
             static_cast<unsigned>(peerPort_));
  } else {
    snprintf(commandBuf_, sizeof(commandBuf_), "AT+CIPSEND=%d,%u", inFlight_.link,
             static_cast<unsigned>(inFlight_.slot.len));
  }
  char cmd[kCommandBuffer];
  copyBounded(cmd, sizeof(cmd), commandBuf_);
  if (!startCommand(cmd, ">", kCommandTimeoutMs)) {
    ++dropCount_;  // TX ring full at this instant -- drop, never stall
    return;
  }
  sendPhase_ = kAwaitPrompt;
}

// --- mDNS / DNS-SD announcement ----------------------------------------------------
//
// The module has no mDNS of its own (neither the ESP-AT dialect it runs
// nor Ai-Thinker's newer Combo AT manual lists one), so the robot
// multicasts a complete, unsolicited DNS-SD response itself: one packet,
// five records, re-sent every kMdnsPeriodMs so browsers' caches (record
// TTL kMdnsTtlSeconds) never expire. It answers no queries -- a browser
// that starts listening sees the robot at the next announcement.
//
// Wire format (RFC 1035 / 6762 / 6763), names compressed with pointers
// so the whole thing fits one slot:
//   header: id 0, flags 0x8400 (response, authoritative), 0 questions,
//           5 answers
//   1. _services._dns-sd._udp.local  PTR  _robotlink._udp.local
//   2. _robotlink._udp.local         PTR  <host> robot link._robotlink._udp.local
//   3. <instance>                    SRV  0 0 <port> <host>.local     (cache-flush)
//   4. <instance>                    TXT  name=<host> role=robot link=v6-udp port=<port>
//   5. <host>.local                  A    <ownIp>                      (cache-flush)

namespace {

struct Packer {
  uint8_t* out;
  size_t cap;
  size_t len;
  bool overflow;

  Packer(uint8_t* o, size_t c) : out(o), cap(c), len(0), overflow(false) {}

  void u8(uint8_t v) {
    if (len < cap) out[len++] = v; else overflow = true;
  }
  void u16(uint16_t v) { u8(static_cast<uint8_t>(v >> 8)); u8(static_cast<uint8_t>(v)); }
  void u32(uint32_t v) { u16(static_cast<uint16_t>(v >> 16)); u16(static_cast<uint16_t>(v)); }
  void label(const char* s) {
    const size_t n = strlen(s);
    u8(static_cast<uint8_t>(n));
    for (size_t i = 0; i < n; ++i) u8(static_cast<uint8_t>(s[i]));
  }
  void pointer(size_t offset) { u16(static_cast<uint16_t>(0xC000 | offset)); }
  void rrHeader(uint16_t type, bool cacheFlush, uint32_t ttl) {
    u16(type);
    u16(static_cast<uint16_t>(0x0001 | (cacheFlush ? 0x8000 : 0)));
    u32(ttl);
  }
  // Writes the 2-byte RDLENGTH placeholder, returns its offset.
  size_t rdlenStart() { const size_t at = len; u16(0); return at; }
  void rdlenEnd(size_t at) {
    const uint16_t n = static_cast<uint16_t>(len - at - 2);
    if (at + 1 < cap) { out[at] = static_cast<uint8_t>(n >> 8); out[at + 1] = static_cast<uint8_t>(n); }
  }
};

bool parseIpv4(const char* text, uint8_t* octets) {
  int part = 0, value = 0, digits = 0;
  for (const char* p = text;; ++p) {
    if (*p >= '0' && *p <= '9') {
      value = value * 10 + (*p - '0');
      if (++digits > 3 || value > 255) return false;
    } else if (*p == '.' || *p == '\0') {
      if (digits == 0 || part > 3) return false;
      octets[part++] = static_cast<uint8_t>(value);
      value = 0; digits = 0;
      if (*p == '\0') break;
    } else {
      return false;
    }
  }
  return part == 4;
}

}  // namespace

size_t WifiLink::buildMdnsAnnouncement(uint8_t* out, size_t cap, const char* hostname,
                                       const char* ownIp, uint16_t port,
                                       uint32_t ttlSeconds, const char* proto) {
  uint8_t ip[4];
  if (hostname == nullptr || hostname[0] == '\0' || !parseIpv4(ownIp, ip)) return 0;

  char instance[64];
  snprintf(instance, sizeof(instance), "%s%s", hostname, instanceSuffix());
  char txtName[40], txtPort[24];
  snprintf(txtName, sizeof(txtName), "name=%s", hostname);
  snprintf(txtPort, sizeof(txtPort), "port=%u", static_cast<unsigned>(port));
  const char* txtRole = "role=robot";
  const char* txtLink = "link=v6";

  Packer p(out, cap);
  p.u16(0);        // id
  p.u16(0x8400);   // QR=1 (response), AA=1
  p.u16(0);        // QDCOUNT
  p.u16(5);        // ANCOUNT
  p.u16(0);        // NSCOUNT
  p.u16(0);        // ARCOUNT

  // 1. _services._dns-sd._udp.local PTR _robotlink._udp.local
  p.label("_services"); p.label("_dns-sd"); p.label("_udp");
  const size_t localOff = p.len;
  p.label("local"); p.u8(0);
  p.rrHeader(12, false, ttlSeconds);
  size_t rd = p.rdlenStart();
  const size_t serviceOff = p.len;
  p.label(serviceType()); p.label(proto); p.pointer(localOff);
  p.rdlenEnd(rd);

  // 2. _robotlink._udp.local PTR <instance>._robotlink._udp.local
  p.pointer(serviceOff);
  p.rrHeader(12, false, ttlSeconds);
  rd = p.rdlenStart();
  const size_t instanceOff = p.len;
  p.label(instance); p.pointer(serviceOff);
  p.rdlenEnd(rd);

  // 3. <instance> SRV 0 0 <port> <host>.local
  p.pointer(instanceOff);
  p.rrHeader(33, true, ttlSeconds);
  rd = p.rdlenStart();
  p.u16(0); p.u16(0); p.u16(port);
  const size_t hostOff = p.len;
  p.label(hostname); p.pointer(localOff);
  p.rdlenEnd(rd);

  // 4. <instance> TXT
  p.pointer(instanceOff);
  p.rrHeader(16, true, ttlSeconds);
  rd = p.rdlenStart();
  p.label(txtName); p.label(txtRole); p.label(txtLink); p.label(txtPort);
  p.rdlenEnd(rd);

  // 5. <host>.local A <ip>
  p.pointer(hostOff);
  p.rrHeader(1, true, ttlSeconds);
  rd = p.rdlenStart();
  for (int i = 0; i < 4; ++i) p.u8(ip[i]);
  p.rdlenEnd(rd);

  return p.overflow ? 0 : p.len;
}

void WifiLink::queueMdnsAnnouncement() {
  // Build straight into the ring slots: no stack copy of a ~200-byte
  // packet on the protocol fiber. One packet per service type.
  const char* protos[2] = {serviceProto(), serviceProtoTcp()};
  const int count = tcpServerOpen_ ? 2 : 1;
  for (int i = 0; i < count; ++i) {
    if (txCount_ >= kTxSlots) return;
    TxEntry& e = tx_[(txHead_ + txCount_) % kTxSlots];
    const size_t n = buildMdnsAnnouncement(e.slot.data, kSlotBytes, config_.hostname,
                                           ownIp_, config_.port, kMdnsTtlSeconds,
                                           protos[i]);
    if (n == 0) return;
    e.link = kMdnsLink;
    e.telemetry = true;  // purgeable: a reply outranks an announcement
    e.slot.len = static_cast<uint16_t>(n);
    ++txCount_;
  }
  ++mdnsAnnounceCount_;
}

}  // namespace diffDrive
