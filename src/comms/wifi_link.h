// wifi_link.h -- WifiLink: the ELECFREAKS Planet X WiFi module
// (Ai-Thinker Ai-WB2-12F, BL602, ESP-AT "Combo AT" 1.4.1 dialect) as a
// THIRD transport for the v6 wire, peer to USB serial and the radio.
//
// HOST-PORTABLE BY CONSTRUCTION: no pxt.h, no CODAL. Everything that
// touches a real UART lives behind the WifiUart seam below
// (comms/wifi_uart.{h,cpp} is the CODAL NRF_UARTE1 implementation), and
// the clock is a plain function pointer, so the whole AT state machine
// runs under tests/host/ against a scripted fake module
// (tests/host/wifi_link_shim.cpp, test_wifi_link.py). Same split as
// the kernel/motion engine vs their platform ports.
//
// == The design this ports ==
//
// radio-robot-lib docs/design/wifi-link (the design note) is the porting authority:
// the AT-mode, CIPMUX=1 design proven live on tovez 2026-08-21/22
// (nezha-upy `src/core/wifi_at.py`, its reference implementation, and
// the 2026-08-21 tovez bench log, sections 7-34, as the
// measured record). NOT the earlier transparent-passthrough C++
// exploration (radio-robot-elite `wifi-transport` branch): passthrough
// is one pipe, does not preserve datagram boundaries, and needs a
// `+++` escape dance every boot because the module -- powered from the
// RJ11 jack, not the nRF -- keeps its mode across a micro:bit reset.
// AT mode keeps ONE DATAGRAM == ONE WIRE LINE, learns the host from
// the `+IPD` header (`AT+CIPDINFO=1`), and can send to a multicast
// address on a second link, which is what the mDNS announcer below
// needs.
//
// One deliberate omission from that design: the TCP REPL mirror. A
// PXT program has no REPL, so only the UDP protocol plane (ESP-AT link
// id 4, port 7654) is carried; link id 3 is the mDNS announcer's own
// multicast socket.
//
// == Bring-up (all measured -- see the wifi-link note, section 5) ==
//
//   configure: AT+RST, AT, ATE0, CIPMODE=0, CIPSERVER=0, CIPCLOSE=5,
//              CIPCLOSE, CWMODE=1, CIPMUX=1, CIPDINFO=1
//   join:      poll `AT+CWJAP?` first (the module auto-rejoins after RST;
//              an explicit join fired into that answers busy/ERROR and
//              near-livelocks), then `AT+CWJAP="ssid","pw"`
//   address:   AT+CWDHCP=1,1 (tolerant), AT+CIPSTA? to learn our own IP
//              (for the mDNS A record)
//   socket:    AT+CIPSTART=4,"UDP","255.255.255.255",7655,7654,2
//              AT+CIPSTART=3,"UDP","224.0.0.251",5353,5353,0 (tolerant)
//   ready:     demux +IPD frames; one AT+CIPSEND per outbound line;
//              re-announce over mDNS every kMdnsPeriodMs
//
// Join time from a fresh RST is 6-170 s on this module (bench section
// 7) -- nothing above waits on it; the robot boots, drives, and answers
// USB/radio normally the whole time, and this link simply reports
// nothing until it is up.
//
// == Every method is non-blocking ==
//
// service() is called once per pass of Protocol::run()'s own loop (every
// kPollIntervalMs) and does one bounded step. No sleep, no spin, no
// yield anywhere in this class -- so no VFP-guard concern either (see
// the fiber-yield-safety rule); the CODAL side (wifi_uart.cpp)
// only ever uses ASYNC serial calls for the same reason.
#pragma once

#include <cstddef>
#include <cstdint>

namespace diffDrive {

// The byte-pipe seam. Every method non-blocking, mirroring the shape
// radio-robot-elite's Hal::UART and nezha-upy's wifiuart shim both
// converged on:
//   read()  -- copies what is ALREADY buffered, returns the count (0 = none)
//   write() -- WHOLE-BUFFER-OR-NOTHING: false and nothing written when
//              the TX buffer cannot take all of it right now. A partial
//              AT command is not a command that fails; it is a command
//              whose tail prefixes and corrupts the NEXT one.
class WifiUart {
 public:
  virtual ~WifiUart() {}
  virtual void begin(uint32_t baud) = 0;
  virtual uint16_t read(uint8_t* buf, uint16_t cap) = 0;
  virtual bool write(const uint8_t* data, uint16_t len) = 0;
  virtual void clearRx() = 0;
};

class WifiLink {
 public:
  typedef uint32_t (*NowMsFn)();

  enum State : uint8_t {
    kDisabled = 0,   // begin() never called, or empty SSID
    kConfigure = 1,  // the AT+RST ... CIPDINFO=1 sequence
    kJoin = 2,       // CWJAP? poll, then CWJAP
    kAddress = 3,    // CWDHCP + CIPSTA? (learn our IP)
    kSocket = 4,     // CIPSTART link 4 (protocol) and link 3 (mDNS)
    kReady = 5,      // the link is UP -- protocol traffic flows
    kBackoff = 6,    // a strict step failed; waiting before starting over
  };

  // Every pointer is borrowed for the life of the link (string literals
  // or Protocol-owned storage). An empty `ssid` leaves the link
  // kDisabled forever -- how a robot with no module / no secrets baked
  // behaves, at zero cost.
  struct Config {
    const char* ssid;
    const char* password;
    const char* hostname;  // mDNS host label (e.g. "tovez") -> <hostname>.local
    uint16_t port;         // our UDP protocol port (7654) -- and the TCP server's
    uint16_t hostPort;     // the host's fixed port (7655), CIPSTART's remote placeholder
    bool tcpServer;        // also accept TCP clients on `port` (AT+CIPSERVER)
    Config() : ssid(""), password(""), hostname("robot"), port(7654), hostPort(7655),
               tcpServer(true) {}
  };

  // Wire-line ceiling, matching Wire::WireHandler::kMaxLineBytes /
  // SerialTransport::kMaxLineBytes / RadioTransport::kMaxPayloadBytes
  // (all 240; tests/host/test_wire_constants_drift.py pins the others,
  // this one is sized to carry the same line). One datagram slot holds
  // one line plus its '\n'.
  static constexpr size_t kMaxLineBytes = 240;
  static constexpr size_t kSlotBytes = 256;  // >= kMaxLineBytes + 1, and >= one mDNS packet
  static constexpr int kRxSlots = 4;         // inbound datagrams parked for tryReceiveLine()
  static constexpr int kTxSlots = 8;         // outbound datagrams waiting for their CIPSEND

  // ESP-AT link ids. Pinned so the demux routes by constant. The module
  // hands INBOUND TCP clients the lowest free ids, 0..2, which is why
  // our own two sockets sit at the top.
  static constexpr int kProtocolLink = 4;
  static constexpr int kMdnsLink = 3;
  static constexpr int kMaxTcpLinks = 3;  // client ids 0, 1, 2

  // Timings (the wifi-link note, section 5.1 / 6.1 / 7; nezha-upy wifi_at.py).
  static constexpr uint32_t kCommandTimeoutMs = 4000;
  static constexpr uint32_t kJoinTimeoutMs = 15000;
  static constexpr uint32_t kJoinQueryMs = 1500;
  static constexpr int kJoinQueryAttempts = 6;  // ~9 s of CWJAP? polling
  static constexpr uint32_t kBackoffDelayMs = 5000;
  static constexpr uint32_t kPeerSilenceMs = 60000;  // forget a silent host
  static constexpr uint32_t kMdnsPeriodMs = 60000;   // re-announce cadence (record TTL is 120 s)
  static constexpr uint32_t kTelemetryMinIntervalMs = 50;  // the wifi-link note, section 7.1 floor

  WifiLink(WifiUart& uart, NowMsFn nowMs);

  // Records the config and arms the state machine; does no I/O itself.
  // Calling it again restarts bring-up from kConfigure.
  void begin(const Config& config);

  // One bounded step: drain the UART into the demux, advance the
  // bring-up state, service the outbound send engine. Call every poll.
  void service();

  // Next inbound protocol datagram as one wire line (trailing '\r'/'\n'
  // stripped, never NUL-included), or false. One datagram IS one line.
  bool tryReceiveLine(uint8_t* outBuf, size_t outCap, size_t* outLen);

  // Queue one wire line (WITHOUT its trailing '\n' -- this appends it)
  // to the CURRENT CLIENT: the TCP client the last inbound line came
  // from, or the learned UDP host (replyLink()). Returns false -- and
  // drops, never blocks -- when the link is not ready, no client is
  // known, or the bounded queue is full (drop-NEWEST, counted in
  // dropCount()).
  bool sendLine(const uint8_t* data, size_t len);

  // Where replies currently go: kProtocolLink (the UDP host) or a TCP
  // client id 0..2. Set by tryReceiveLine() to whichever carrier the
  // last inbound line arrived on, and by a TCP CONNECT (newest client
  // wins); falls back to the UDP host when that client closes.
  int replyLink() const { return replyLink_; }
  uint8_t tcpOpenMask() const { return tcpOpenMask_; }
  bool tcpServerOpen() const { return tcpServerOpen_; }
  int queuedSends() const { return txCount_; }

  // Telemetry gate (the wifi-link note, section 7.1, REQUIRED of every port):
  // periodic frames may only be queued when at least
  // kTelemetryMinIntervalMs has passed since the last one AND the send
  // engine is IDLE (nothing queued, nothing in flight). Replies/acks
  // never consult this. Consumes the interval when it returns true.
  //
  // "Idle", not merely "room for a frame": a TCP client's SEND OK waits
  // for the client's TCP acknowledgement, and a host that delays its
  // acks (macOS does, ~200 ms) turns every queued line into hundreds of
  // milliseconds. A queue of frames then holds a reply behind seconds
  // of stale telemetry. So frames are only ever queued into an empty
  // engine, and a reply purges any frames still waiting (sendLine()).
  bool telemetryAllowed();

  // Bracket the caller's telemetry emission: lines sent while `on` are
  // tagged as periodic frames, which a later reply may discard unsent
  // (a stale frame is worthless; a late ack is a stalled host).
  void markTelemetry(bool on) { telemetryMode_ = on; }

  // --- introspection, for the DBG:wifi line and bench tools ---
  State state() const { return state_; }
  bool ready() const { return state_ == kReady; }
  bool peerKnown();  // lazily applies the silence forget
  const char* peerIp() const { return peerIp_; }
  uint16_t peerPort() const { return peerPort_; }
  const char* ownIp() const { return ownIp_; }
  uint32_t restartCount() const { return restartCount_; }
  uint32_t dropCount() const { return dropCount_; }
  uint32_t sentCount() const { return sentCount_; }
  uint32_t receivedCount() const { return receivedCount_; }
  uint32_t mdnsAnnounceCount() const { return mdnsAnnounceCount_; }
  bool mdnsSocketOpen() const { return mdnsSocketOpen_; }
  const char* lastCommand() const { return lastCommand_; }
  const char* lastReply() const { return lastReply_; }

  // True exactly once per NEW (ip, port) the protocol plane starts
  // hearing from -- Protocol greets a fresh host with the boot banner on
  // this edge (the wifi-link note, section 6.1).
  bool pollNewPeerEdge();

  // True exactly once per TCP client CONNECT -- Protocol greets the
  // new client with the banner, exactly what a USB host sees at
  // connect.
  bool pollNewClientEdge();

  // True once per state transition since the last call -- lets the
  // caller emit one diagnostic line per change rather than per poll.
  bool pollStateChanged();

  // Builds the DNS-SD announcement (a complete mDNS response packet:
  // PTR for _services._dns-sd._udp, PTR/SRV/TXT for the service
  // instance, A for the host) into `out`. Returns the packet length, or
  // 0 if it does not fit or no IP is known. Public and pure so
  // tests/host/ can decode the exact bytes the robot multicasts.
  // `proto` is "_udp" or "_tcp": the same instance name is announced
  // under both service types when the TCP server is up.
  static size_t buildMdnsAnnouncement(uint8_t* out, size_t cap,
                                      const char* hostname,
                                      const char* ownIp, uint16_t port,
                                      uint32_t ttlSeconds,
                                      const char* proto);

  // The service this robot advertises: `<hostname> robot link` on
  // `_robotlink._udp.local` -- the name of the robot is the instance
  // name's first label so a browser listing reads "tovez robot link",
  // and the TXT record spells it out (name=tovez role=robot ...).
  static const char* serviceType() { return "_robotlink"; }
  static const char* serviceProto() { return "_udp"; }
  static const char* serviceProtoTcp() { return "_tcp"; }
  static const char* instanceSuffix() { return " robot link"; }

 private:
  // --- incremental literal matcher (byte at a time, no line buffer)
  class Matcher {
   public:
    Matcher() : token_(nullptr), matched_(0) {}
    void reset(const char* token);
    bool feed(char c);
   private:
    const char* token_;
    uint8_t matched_;
  };

  // --- `+IPD,<link>,<len>[,"<ip>",<port>]:` header parser (both forms)
  class IpdParser {
   public:
    IpdParser() { reset(); }
    void reset();
    bool feed(char c);  // true on the ':' that completes a header
    int link() const { return link_; }
    size_t length() const { return length_; }
    const char* ip() const { return ip_; }
    uint16_t port() const { return port_; }
   private:
    enum Stage : uint8_t { kTag, kLink, kLen, kToQuote, kIp, kToPort, kPort, kDone };
    Stage stage_;
    Matcher tag_;
    int link_;
    size_t length_;
    bool sawDigit_;
    char ip_[16];
    uint8_t ipLen_;
    uint16_t port_;
    bool portSawDigit_;
  };

  struct Slot {
    uint16_t len;
    int8_t link;  // which ESP-AT link this arrived on / goes out on
    uint8_t data[kSlotBytes];
  };

  uint32_t nowMs() const { return nowMs_(); }

  // AT command/await mechanics (one in flight at a time)
  enum Await : uint8_t { kPending, kMatched, kRejected, kTimedOut };
  bool startCommand(const char* command, const char* expect, uint32_t timeoutMs);
  void startAwait(const char* expect, uint32_t timeoutMs);
  Await pollAwait();

  // The ONE place UART bytes enter this class.
  void pumpIncoming();
  void feedByte(uint8_t c);
  void feedStatusByte(uint8_t c);
  void handleStatusLine();  // `<link>,CONNECT` / `<link>,CLOSED`
  void finishPayload();
  void pushRxLine(int link, const uint8_t* data, size_t len);
  void traceReply(char c);

  void enterState(State next);
  void enterBackoff();

  void serviceConfigure();
  void serviceJoin();
  void serviceAddress();
  void serviceSocket();
  void serviceReady();
  void serviceBackoff();

  // outbound send engine (one CIPSEND per datagram, drop-don't-stall)
  bool enqueueSend(int link, const uint8_t* data, size_t len);
  void popNextSend();
  void queueMdnsAnnouncement();
  int txCount() const { return txCount_; }  // (test introspection)

  WifiUart& uart_;
  NowMsFn nowMs_;
  Config config_;

  State state_;
  uint8_t step_;
  int joinQueryAttempt_;
  uint32_t restartCount_;
  bool stateChanged_;

  bool awaiting_;
  uint32_t deadline_;
  Matcher expect_;
  Matcher rejectError_;
  Matcher rejectFail_;
  Matcher rejectBusy_;
  bool awaitMatched_;
  bool awaitRejected_;

  // demux
  IpdParser ipd_;
  size_t payloadRemaining_;
  int payloadLink_;
  Slot payload_;  // the frame being captured
  static constexpr uint16_t kStatusLineMax = 96;
  uint8_t statusLine_[kStatusLineMax];
  uint16_t statusLen_;

  // CIPSTA? capture: after `ip:"` collect up to 15 chars to '"'
  Matcher ownIpTag_;
  bool ownIpCapturing_;
  char ownIp_[16];
  uint8_t ownIpLen_;

  // peer (the host), learned from +IPD headers on kProtocolLink
  char peerIp_[16];
  uint16_t peerPort_;
  bool peerKnown_;
  uint32_t lastPeerHeardMs_;
  char reportedPeerIp_[16];
  uint16_t reportedPeerPort_;

  // inbound line ring for tryReceiveLine() -- UDP datagrams and
  // completed TCP lines alike
  Slot rx_[kRxSlots];
  int rxHead_;
  int rxCount_;

  // TCP clients: which ids are open, one line accumulator each (a TCP
  // payload is a byte stream, not a line), the current reply target and
  // the connect edge.
  uint8_t tcpOpenMask_;
  uint8_t tcpLine_[kMaxTcpLinks][kMaxLineBytes + 1];
  uint16_t tcpLineLen_[kMaxTcpLinks];
  bool tcpLineOverflow_[kMaxTcpLinks];
  int8_t replyLink_;
  bool tcpConnectEdge_;
  bool tcpServerOpen_;

  // outbound datagram ring + the two-phase send in flight
  struct TxEntry {
    int link;
    bool telemetry;  // a periodic frame (or announcement): purgeable by a reply
    Slot slot;
  };
  bool telemetryMode_;
  void purgeTelemetry();  // drop every queued telemetry entry, keep order of the rest
  TxEntry tx_[kTxSlots];
  int txHead_;
  int txCount_;
  enum SendPhase : uint8_t { kIdle, kAwaitPrompt, kAwaitSendOk };
  SendPhase sendPhase_;
  TxEntry inFlight_;

  uint32_t lastTelemetryMs_;
  bool telemetryEverSent_;

  bool mdnsSocketOpen_;
  uint32_t lastMdnsMs_;
  uint32_t mdnsAnnounceCount_;

  uint32_t dropCount_;
  uint32_t sentCount_;
  uint32_t receivedCount_;
  uint32_t promptRetryDeadline_;  // [ms] 0 = no payload write pending at a '>' prompt

  static constexpr uint16_t kCommandBuffer = 128;
  char commandBuf_[kCommandBuffer];
  char expectBuf_[kCommandBuffer];  // the CWJAP? expect token, outlives commandBuf_'s reuse
  static constexpr uint16_t kTraceCommand = 48;
  static constexpr uint16_t kTraceReply = 72;
  char lastCommand_[kTraceCommand];
  char lastReply_[kTraceReply];
  uint16_t lastReplyLen_;

  static constexpr uint16_t kStageBuffer = 64;
  uint8_t stage_[kStageBuffer];
  uint16_t stageLen_;
  uint16_t stagePos_;
};

}  // namespace diffDrive
