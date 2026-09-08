// protocol.cpp -- see protocol.h.
#include "protocol.h"

#include "../core/fiber_identity.h"
#include "../platform/vfp_guard.h"

#include <cstdio>  // plain snprintf, not std::snprintf: newlib-nano's
                   // <cstdio> declares it globally but never puts it in
                   // namespace std (same gotcha wire_handler.cpp
                   // documents for its own copy).
#include <cstring>

namespace diffDrive {

// ---- shims.cpp entry points this file needs directly ---------------------
// tickDrive() runs one kernel.step() + serviceMove() -- the caller-
// driven replacement for the kernel's own unwired fiber. This fiber
// calls it directly while a wire motion obligation is live (see run()'s
// own comment below on why THIS file, not wireAdapter_, owns that
// call). Reached by same-package forward declaration -- shims.cpp has
// no header of its own; keep signatures compatible.
bool tickDrive();

// Runs the ONE registered RUN dispatch action (test.ts's onRun()/
// onRunCommand() handlers all share it), on the caller's own fiber --
// see handleRun()/dispatchJob() below. Returns false with nothing registered
// yet (no onRun() handler has ever been called), a silent no-op.
bool runDispatch();

// Registers a plain, no-capture function pointer to be called from
// inside tickDrive() itself, once per call, after its own kernel.step()/
// settle work is done and before its pacing sleep -- see
// serviceHookEntry()'s own comment (protocol.h) for why this fiber needs
// that. Kept OUT of shims.cpp's own `//%`-annotated, TS-facing surface:
// this is a plain C++-to-C++ seam, never called from a block or from
// TypeScript.
void registerTickServiceHook(void (*hook)());

namespace {

// ---- identity constants ----------------------------------------------
// Assembled into a Wire::Identity by Protocol::buildIdentity() below.
//   - name: microbit_friendly_name(), read from silicon at call time --
//     THE authoritative board identity, and what mbdeploy keys its
//     device registry off, so a fixed string here would stomp that
//     registry fleet-wide.
//   - serial: microbit_serial_number(), unique per device.
//   - kDrivetrain: this extension's kinematic type (matches the package
//     name).
//   - kProfile: which robot's config this hex was BUILT AGAINST
//     (radio-robot-lib's per-robot config filename stem, e.g. "vevov").
//     Build PROVENANCE, never board identity.
//   - kVersion: the PROJECT version, `1.YYYYMMDD.n`, kept identical
//     across pyproject.toml / package.json / config/dotconfig.yaml /
//     pxt.json by dotconfig. NOT pxt.json's old extension-only semver
//     story -- this IS pxt.json's version now, dotconfig's single
//     source of truth, so VER answers "which revision is this?" for
//     every build, not just release builds.
//
// kProfile is injected at DEPLOY time into the SCRATCH COPY only
// (tools/make_deploy.py's _inject_profile()), the same substitution
// _inject_radio_channel() performs on radio_transport.h's kChannel --
// so the checked-in "unbaked" literal below can never impersonate a
// fleet board. `unbaked` is the honest answer until a robot config is
// actually loaded, and nothing short of a real deploy loads one.
//
// kVersion is different: 2026-09-07 stakeholder direction moved it OFF
// the deploy-only path. config/hooks/version_bump now bakes it straight
// into this checked-in literal on every `dotconfig version bump`, so a
// plain `git clone` and the generated extension (assembled by
// tools/publish_extension.py and built in MakeCode's cloud compiler,
// which never runs make_deploy.py) both ship a real version instead of
// the old placeholder every consumer board used to answer with.
// tools/make_deploy.py's _inject_version() still re-injects the same
// value into the deploy scratch copy -- belt and braces, covering a
// tree whose pyproject.toml was hand-edited between bumps.
//
// `profile` and `name` can legitimately DISAGREE on one ID reply:
// profile says which robot's config the hex targeted, name says which
// physical board is answering. That disagreement IS the diagnostic --
// this board was flashed with the wrong robot's build -- so never
// "fix" it by forcing the two to match.
constexpr const char* kDrivetrain = "diffdrive";
constexpr const char* kProfile = "unbaked";
constexpr const char* kVersion = "1.20260907.5";  // baked by config/hooks/version_bump
                                              // at `dotconfig version bump`; see the
                                              // note above

// WiFi credentials, injected into the SCRATCH COPY ONLY by
// tools/make_deploy.py's _inject_wifi_secrets() from the gitignored
// config/wifi_secrets.json -- the same scratch-copy substitution kProfile
// and kVersion use. The checked-in literals are EMPTY on purpose: an
// empty SSID is WifiLink's own "disabled" sentinel, so a build made
// outside make_deploy.py (or with no secrets file) never opens UARTE1
// at all. Never put a real SSID or passphrase here.
//
// DO NOT reformat these two lines: _inject_wifi_secrets()'s regexes are
// `(constexpr const char\* kWifiSsid = ")[^"]*(";)` and the same for
// kWifiPassword, and the deploy raises if either stops matching.
constexpr const char* kWifiSsid = "";
constexpr const char* kWifiPassword = "";

// The robot's UDP protocol port and the host's fixed port
// (radio-robot-lib's wifi-link design note, section 2: the robot's planes
// share 7654 by design; the host binds 7655 so a host restart needs no
// re-discovery).
constexpr uint16_t kWifiPort = 7654;
constexpr uint16_t kWifiHostPort = 7655;

// How often the `DBG:wifi` line repeats while the link is NOT ready, on
// top of the one-per-state-change emission -- a bench operator watching
// USB during a slow join (6-170 s, measured) should see it still alive.
constexpr uint32_t kWifiDebugPeriod = 10000;  // [ms]

// Poll granularity between transport reads, and the telemetry frame
// emission cadence for a TLM-subscribed host (2026-08-26: this paces
// FRAMES only -- the reliability line that used to ride each frame is
// deleted, S8.5): small enough that a command arriving just after one
// poll is still picked up promptly; not so small it spins this fiber
// against an idle UART between bytes.
constexpr uint32_t kTelemetryEmitPeriod = 50;  // [ms]
constexpr uint32_t kPollInterval = 5;  // [ms]

}  // namespace

void Protocol::emitLine(const char* text) {
  if (text == nullptr) return;
  size_t len = 0;
  // Clip by NAME to RadioTransport::kMaxPayloadBytes, never to a
  // literal of this call's own: the two drifted apart silently once,
  // and a 201-239 byte line then truncated on EVERY transport.
  while (text[len] != '\0' && len < RadioTransport::kMaxPayloadBytes) ++len;
  if (len == 0) return;
  // No transport write here any more -- copy into the ring and return.
  // See protocol.h's own comment on emitLine()/emitLineNow() for why:
  // this call can run on any fiber, and only this object's own fiber
  // (drainEmitQueue(), below) is allowed to reach a transport write for
  // this path. A refusal (ring full) is counted via emitQueue_.dropped()
  // rather than silently overwriting a line still waiting to drain.
  emitQueue_.enqueue(text, len);
}

// The pre-restructuring emitLine() body, unchanged, now reachable only
// from drainEmitQueue() below -- see protocol.h's own comment on why
// this split makes this fiber the emit path's single producer.
void Protocol::emitLineNow(const char* text, size_t len) {
  transport_.writeLine(reinterpret_cast<const uint8_t*>(text), len);
  // WiFi mirror: a test's own result lines reach a WiFi host too.
  // sendLine() drops (never blocks) when the link is down or no host
  // is known, and this fiber is the only WifiLink caller, so no guard
  // or retry is needed here.
  if (wifiEnabled_) {
    (void)wifiLink_.sendLine(reinterpret_cast<const uint8_t*>(text), len);
  }
  // Radio mirror. No enable check here: RadioTransport owns that gate
  // itself and returns false, having touched nothing, while its link is
  // disabled (RadioTransport::enable()'s own doc comment,
  // radio_transport.h) -- which is what keeps a student's first debug
  // line from claiming the radio out from under MakeCode's own radio
  // blocks. Serial above is unconditional: serial is always v6.
  //
  // No retry either. This fiber is the only writer sendLine() has, so a
  // false return can no longer mean "another writer had the buffers" --
  // it means the link is off, and sending the same line twice to a
  // disabled radio achieves nothing.
  (void)radioTransport_.sendLine(reinterpret_cast<const uint8_t*>(text), len);
}

// Drains emitQueue_ into emitLineNow(), oldest line first. Copies each
// line out to a local, on-this-fiber's-stack buffer before calling
// emitLineNow() -- that call can yield (uBit.serial.send(SYNC_SLEEP)
// blocks once CODAL's TX ring fills, see serial_transport.cpp), and
// holding a pointer into the ring's own storage across a yield would
// let a concurrent enqueue() from another fiber overwrite it before
// this fiber finishes using it.
void Protocol::drainEmitQueue() {
  char text[kEmitTextBytes];
  size_t len;
  while ((len = emitQueue_.dequeue(text, sizeof(text))) > 0) {
    emitLineNow(text, len);
  }
}

// Free-function entry point for shims.cpp's emitLine shim. Lives here,
// on the protocol side of the boundary, so shims.cpp never has to
// include protocol.h (and with it radio_transport.h, which makes PXT's
// dependency scan demand the `radio` package for that file).
void protocolEmitLine(const char* text) { protocol().emitLine(text); }

// Free-function entry point for shims.cpp's setupWifi shim (ticket
// 002) -- same boundary reason as protocolEmitLine() above: shims.cpp
// forward-declares this the same way it forward-declares
// protocolEmitLine, rather than including protocol.h.
void protocolSetupWifi(const char* ssid, const char* password) {
  protocol().setupWifi(ssid, password);
}

int Protocol::serialDropCount() const {
  return static_cast<int>(transport_.dropCount());
}

// Free-function entry point for shims.cpp's diagValue(26) case (ticket
// 006) -- same boundary reason as protocolEmitLine() above.
int protocolSerialDropCount() { return protocol().serialDropCount(); }
int protocolRunDropCount() {
  return static_cast<int>(protocol().runDropCount());
}
int protocolEmitDropCount() {
  return static_cast<int>(protocol().emitDropCount());
}
int protocolRunMalformedCount() {
  return static_cast<int>(protocol().runMalformedCount());
}
int protocolRadioRxFrameCount() {
  return static_cast<int>(protocol().radioRxFrameCount());
}
int protocolRadioRxAcceptedCount() {
  return static_cast<int>(protocol().radioRxAcceptedCount());
}
int protocolRadioRxOverrunDropCount() {
  return static_cast<int>(protocol().radioRxOverrunDropCount());
}
int protocolRadioRxOversizeDropCount() {
  return static_cast<int>(protocol().radioRxOversizeDropCount());
}

void Protocol::setupRadio(uint8_t channel, uint8_t group) {
  // Order matters: configure, THEN enable. Both setters only store while
  // the radio is down, and ensureRadioReady() reads the stored values
  // when it later brings it up -- so doing it this way means the radio
  // comes up already on the requested channel and group, and the
  // mid-run re-apply path (unverified for the channel; see
  // RadioTransport::setChannel()) is never exercised by this call.
  radioTransport_.setChannel(channel);
  radioTransport_.setGroup(group);
  radioTransport_.enable();
}

void Protocol::enableRadio() { radioTransport_.enable(); }

void Protocol::enableWifi() { wifiEnabled_ = true; }

void Protocol::setupWifi(const char* ssid, const char* password) {
  if (ssid == nullptr) return;

  // Late-call guard: once wifiBegun_ is true, WifiLink::Config's
  // ssid/password already point into wifiSsid_/wifiPassword_ and
  // WifiLink::serviceJoin() re-reads them on every join attempt and
  // backoff retry (wifi_link.cpp) -- not a value snapshotted once at
  // begin(). Mutating this storage now would reach into a live join
  // attempt, so the write is refused outright rather than merely
  // skipped-but-harmless. UNVERIFIED on hardware: reasoned from
  // reading wifi_link.cpp's serviceJoin(), not measured against a real
  // in-progress join.
  if (wifiBegun_) {
    emitLine("DBG:wifi late setupWifi() ignored");
    return;
  }

  // Truncation is computed BEFORE the copy, from the plain C-string
  // source's own strlen() (no embedded NUL, so this is exact) against
  // the owned cell's usable length -- so wifiCredsTruncated_ reflects
  // whether the CALLER's string was too long, not an artifact of the
  // clipped copy itself. `password == nullptr` counts as length 0, not
  // truncated (default-argument call with no password supplied).
  wifiCredsTruncated_ = 0;
  if (strlen(ssid) >= sizeof(wifiSsid_)) wifiCredsTruncated_ |= 0x1;
  if (password != nullptr && strlen(password) >= sizeof(wifiPassword_)) {
    wifiCredsTruncated_ |= 0x2;
  }

  // snprintf clips safely regardless of the truncation check above --
  // that check exists to make an already-safe clip VISIBLE, not to
  // prevent an overflow snprintf would have prevented anyway.
  snprintf(wifiSsid_, sizeof(wifiSsid_), "%s", ssid);
  snprintf(wifiPassword_, sizeof(wifiPassword_), "%s",
           password != nullptr ? password : "");

  // Set unconditionally, even for an empty ssid: setupWifi("") is a
  // deliberate explicit disable (SUC-002), and this flag is what lets
  // serviceWifi()'s lazy-begin branch tell that apart from "never
  // called" -- both otherwise leave wifiSsid_[0] == '\0'.
  wifiCredsExplicit_ = true;

  // "Store AND enable" in one call, per the stakeholder decision: no
  // separate enable step, no ordering trap.
  enableWifi();
}

void Protocol::emitWifiDebug() {
  // One line, cleartext `DBG:` prefix (the same convention the TS
  // layer's debug output uses), through emitLine() so it reaches
  // serial, radio AND -- once up -- the WiFi host itself.
  snprintf(wifiDbgBuf_, sizeof(wifiDbgBuf_),
           "DBG:wifi state=%d ip=%s peer=%s:%u tcp=%u/%d to=%d restarts=%lu "
           "sent=%lu rx=%lu drop=%lu mdns=%lu/%d cmd=%s reply=%s "
           "credsrc=%d trunc=%u",
           static_cast<int>(wifiLink_.state()),
           wifiLink_.ownIp()[0] ? wifiLink_.ownIp() : "-",
           wifiLink_.peerIp()[0] ? wifiLink_.peerIp() : "-",
           static_cast<unsigned>(wifiLink_.peerPort()),
           static_cast<unsigned>(wifiLink_.tcpOpenMask()),
           wifiLink_.tcpServerOpen() ? 1 : 0, wifiLink_.replyLink(),
           static_cast<unsigned long>(wifiLink_.restartCount()),
           static_cast<unsigned long>(wifiLink_.sentCount()),
           static_cast<unsigned long>(wifiLink_.receivedCount()),
           static_cast<unsigned long>(wifiLink_.dropCount()),
           static_cast<unsigned long>(wifiLink_.mdnsAnnounceCount()),
           wifiLink_.mdnsSocketOpen() ? 1 : 0,
           wifiLink_.lastCommand(), wifiLink_.lastReply(),
           wifiCredsExplicit_ ? 1 : 0,
           static_cast<unsigned>(wifiCredsTruncated_));
  emitLine(wifiDbgBuf_);
}

void Protocol::serviceWifi() {
  if (!wifiBegun_) {
    wifiBegun_ = true;
    WifiLink::Config config;
    if (wifiCredsExplicit_) {
      // A program called setupWifi() before the link began -- use its
      // stored credentials (possibly an explicit "", which
      // WifiLink::begin() treats as a deliberate disable, same
      // zero-cost outcome as no module fitted; see setupWifi()'s own
      // comment).
      config.ssid = wifiSsid_;
      config.password = wifiPassword_;
    } else {
      config.ssid = kWifiSsid;
      config.password = kWifiPassword;
    }
    // The mDNS host label is the board's own silicon-derived name --
    // the same authoritative identity ID's `name` field reports -- so
    // `tovez.local` / "tovez robot link" can never be a stale bake.
    config.hostname = microbit_friendly_name();
    config.port = kWifiPort;
    config.hostPort = kWifiHostPort;
    wifiLink_.begin(config);
    lastWifiDbg_ = clockNow();
    emitWifiDebug();
  }

  wifiLink_.service();

  const uint32_t now = clockNow();  // [ms]
  if (wifiLink_.pollStateChanged() ||
      (!wifiLink_.ready() &&
       static_cast<int32_t>(now - (lastWifiDbg_ + kWifiDebugPeriod)) >= 0)) {
    lastWifiDbg_ = now;
    emitWifiDebug();
  }

  // A NEW host just spoke to us for the first time: greet it with the
  // boot banner, exactly what a USB host sees at connect
  // (the wifi-link note, section 6.1's READY-on-new-peer edge).
  if (wifiLink_.pollNewPeerEdge()) {
    wireHandlerWifi_.sendBanner();
    emitWifiDebug();
  }
  // Likewise a TCP client that just connected -- it is the current
  // reply target from its CONNECT line onward, so the banner reaches
  // it, exactly what a USB host sees at connect.
  if (wifiLink_.pollNewClientEdge()) {
    wireHandlerWifi_.sendBanner();
    emitWifiDebug();
  }

  // Inbound: one datagram is one line. Bounded per pass by the same
  // kRxDrainPerPass every transport uses, so a host blasting lines
  // cannot starve the rest of serviceOnce(). (WifiLink parks at most
  // kRxSlots datagrams, which is the same number today; the bound
  // spelled here is the servicing budget, not the parking capacity.)
  for (int i = 0; i < kRxDrainPerPass; ++i) {
    size_t len = 0;
    if (!wifiLink_.tryReceiveLine(wifiRxBuf_, WifiLink::kMaxLineBytes, &len)) break;
    routeLine(wireHandlerWifi_, wifiRxBuf_, len);
  }
}

// The ONE inbound path -- see this method's own doc comment
// (protocol.h). Serial, radio and WiFi all arrive here; the only thing
// that differs between them is which WireHandler the caller passes.
void Protocol::routeLine(Wire::WireHandler& handler, const uint8_t* data,
                         size_t len) {
  // EVERY line goes to the v6 wire stack -- no carve-out left. The
  // cleartext `RUN:<name>` prefix match deleted from here on 2026-09-07
  // is now the ordinary `RUN` verb, reaching the same TypeScript
  // dispatcher through WireAdapter::onRun -> protocolOfferRun (below).
  //
  // feed() reassembles regardless of chunking; the trailing '\n' it
  // needs to see the line as complete is a second, separate call.
  handler.feed(reinterpret_cast<const char*>(data), len);
  handler.feed("\n", 1);
}

// ---- the old-style cleartext RUN bridge ------------------------------

bool Protocol::handleRun(const uint8_t* data, size_t dataLen) {
  // Sanitizing and parking both live in runBridge_ (run_bridge.h/.cpp,
  // host-portable and host-tested there). This fiber makes the one call
  // the bridge cannot: the dispatch into TypeScript.
  const RunBridge::Offer outcome = runBridge_.offer(data, dataLen);
  if (outcome == RunBridge::Offer::kMalformed ||
      outcome == RunBridge::Offer::kDropped) {
    // Reported back through onRun()'s Result so the host gets an `err`
    // instead of an ack for a command that will never run. The
    // cleartext path had no way to say this -- it just returned.
    return false;
  }
  if (outcome != RunBridge::Offer::kBypass) return true;

  // "abort"/"clearestop" dispatch RIGHT NOW, on whatever fiber called
  // handleRun() -- run()'s own loop normally, but (crucially) also the
  // service hook nested inside a running job's own tick loop, which is
  // the ONLY way an abort sent while a job is mid-tour can ever be
  // noticed: dispatchJob() refuses to start a SECOND job while
  // motionOwner_ is not kNone, so if abort went through the same queued
  // path as everything else it would sit behind the very job it is
  // meant to stop. Ungated deliberately -- both handlers this
  // dispatches to (test.ts's "abort"/"clearestop") are trivial,
  // non-blocking, and safe to invoke reentrant from inside a job's own
  // call chain. The payload is already staged in runBridge_ for
  // runCommandText() to read back at the handler's own entry.
  runDispatch();
  return true;
}

// shims.cpp-style same-package seam, but pointing the other way: this
// is what WireAdapter::onRun() calls (wire_adapter.cpp forward-declares
// it) to turn a decoded `RUN` verb into a dispatched job. It exists
// because WireAdapter must stay free of pxt.h and cannot reach
// TypeScript, while Protocol can -- the same split
// protocolTryTakeMotionOwnership() already uses.
bool protocolOfferRun(const char* text) {
  if (text == nullptr) return false;
  size_t len = 0;
  while (text[len] != '\0') ++len;
  return protocol().handleRun(reinterpret_cast<const uint8_t*>(text), len);
}

void Protocol::dispatchJob() {
  if (motionOwner_ != MotionOwner::kNone) return;
  // Stages the oldest parked payload for currentRunText() and frees its
  // slot; false means nothing is queued.
  if (!runBridge_.dispatchOne()) return;

  motionOwner_ = MotionOwner::kJob;
  wireAdapter_.setExternalOwner(MotionOwner::kJob);
  runDispatch();
  wireAdapter_.setExternalOwner(MotionOwner::kNone);
  motionOwner_ = MotionOwner::kNone;
}

bool Protocol::tryTakeMotionOwnership() {
  // Same fiber-identity comparison serviceHookEntry() already makes
  // (diffDrive::shouldServiceHookRun(), core/fiber_identity.h) --
  // true iff THIS call is executing on Protocol's own fiber, i.e.
  // inside a dispatched RUN job's own call chain (dispatchJob() calls
  // the TS handler synchronously, on this fiber). See
  // diffDrive::tryTakeMotionOwnership()'s own doc comment
  // (core/motion_owner.h) for why that -- combined with motionOwner_
  // already being kJob -- means this is the job's OWN move, not a
  // competing claim.
  const bool isDispatchingFiber =
      protocolFiberId_ != nullptr && currentFiberFn_() == protocolFiberId_;
  if (!diffDrive::tryTakeMotionOwnership(&motionOwner_, isDispatchingFiber))
    return false;
  // Only a GENUINE kBlock take changes wireAdapter_'s externalOwner_
  // mirror -- the job's-own-fiber bypass above leaves motionOwner_ at
  // kJob unchanged, which dispatchJob() already told wireAdapter_ about
  // (setExternalOwner(kJob)) before ever calling into this job's
  // handler; re-asserting kBlock here would be wrong (and would leak,
  // since nothing on this path calls releaseBlockOwnership()).
  if (motionOwner_ == MotionOwner::kBlock) {
    wireAdapter_.setExternalOwner(MotionOwner::kBlock);
  }
  return true;
}

void Protocol::releaseBlockOwnership() {
  if (motionOwner_ != MotionOwner::kBlock) return;
  diffDrive::releaseBlockOwnership(&motionOwner_);
  wireAdapter_.setExternalOwner(MotionOwner::kNone);
}

// shims.cpp's own seam onto the two methods above -- same same-package
// forward-declaration convention as registerTickServiceHook()/
// runDispatch() (this file's own top-of-file forward declarations):
// only Protocol can see a wire request, a dispatched job, AND a block-
// motion call together, so a motion entry point reaches this
// singleton through a plain free function rather than holding a
// reference of its own.
bool protocolTryTakeMotionOwnership() {
  return protocol().tryTakeMotionOwnership();
}
void protocolReleaseBlockOwnership() {
  protocol().releaseBlockOwnership();
}

const char* Protocol::currentRunText() const {
  return runBridge_.currentText();
}

void Protocol::serviceHookEntry() {
  Protocol& p = protocol();
  if (!diffDrive::shouldServiceHookRun(p.protocolFiberId_,
                                       p.currentFiberFn_()))
    return;
  p.serviceOnce();
}

// The real "current fiber" reader: currentFiber is CODAL's own global
// scheduler pointer, reached unqualified via MicroBit.h's own
// "using namespace codal" -- the same path microbit_friendly_name()/
// microbit_serial_number() (buildIdentity(), above) and create_fiber()
// (platform_ports.h) already reach their own globals through. Compared
// only for pointer identity by shouldServiceHookRun(), never
// dereferenced.
const void* Protocol::defaultCurrentFiber() {
  return static_cast<const void*>(currentFiber);
}

Protocol::CurrentFiberFn Protocol::currentFiberFn_ = &Protocol::defaultCurrentFiber;

// ---- stack-canary fill: measurement scaffold, no production effect --
//
// Gated on the same macro nezha_port.cpp's own fault-forensics spin
// already uses: a debug build built for a bench pyOCD session, never a
// normal build. Painting happens exactly once, as literally the first
// thing run() does, so the "unused below here" boundary this function
// computes is the shallowest possible point in this fiber's own
// lifetime -- everything from there down through stack_bottom is
// guaranteed untouched so far, and everything from there up through
// this call's own frame is left alone.
//
// A local variable's own address stands in for "the current stack
// pointer": on this ABI it sits within a few words of the true SP,
// comfortably above anything this very call still needs, so the fill
// below can never overwrite a byte this function is using. currentFiber
// is the same CODAL global defaultCurrentFiber() above already reaches
// unqualified; its stack_bottom/stack_top bound the heap-allocated
// region a later offline memory read can scan.
#ifdef DIFFDRIVE_FAULT_SPIN
void Protocol::paintStackCanary() {
  constexpr uint8_t kFillByte = 0xA5;
  volatile uint8_t sentinel = 0;
  uintptr_t ceiling = reinterpret_cast<uintptr_t>(&sentinel);
  uintptr_t low = static_cast<uintptr_t>(currentFiber->stack_bottom);
  uintptr_t high = static_cast<uintptr_t>(currentFiber->stack_top);
  if (ceiling < high) high = ceiling;
  for (uintptr_t addr = low; addr < high; ++addr) {
    *reinterpret_cast<volatile uint8_t*>(addr) = kFillByte;
  }
}
#else
void Protocol::paintStackCanary() {}
#endif

uint32_t Protocol::runDropCount() const { return runBridge_.dropCount(); }
uint32_t Protocol::runMalformedCount() const {
  return runBridge_.malformedCount();
}
uint32_t Protocol::radioRxFrameCount() const {
  return radioTransport_.rxCounters().frames;
}
uint32_t Protocol::radioRxAcceptedCount() const {
  return radioTransport_.rxCounters().accepted;
}
uint32_t Protocol::radioRxOverrunDropCount() const {
  return radioTransport_.rxCounters().overrunDropped;
}
uint32_t Protocol::radioRxOversizeDropCount() const {
  return radioTransport_.rxCounters().oversizeDropped;
}
uint32_t Protocol::emitDropCount() const { return emitQueue_.dropped(); }

// Same boundary, opposite direction: shims.cpp's runCommandText shim
// reads back whichever RUN command is currently being dispatched.
const char* protocolCurrentRunText() { return protocol().currentRunText(); }

// ---- ticket 005: identity, assembled once the fiber actually runs ----

Wire::Identity Protocol::buildIdentity() {
  snprintf(serialBuf_, sizeof(serialBuf_), "%lu",
          static_cast<unsigned long>(microbit_serial_number()));
  Wire::Identity identity;
  identity.name = microbit_friendly_name();
  identity.serial = serialBuf_;
  identity.drivetrain = kDrivetrain;
  identity.profile = kProfile;
  identity.version = kVersion;
  return identity;
}

uint32_t Protocol::clockNow() {  // [ms]
  return static_cast<uint32_t>(clock_.nowMicros() / 1000ull);
}

uint32_t Protocol::wireNow() {  // [ms]
  // Reaches this Protocol instance's own clockNow() (above) through the
  // protocol() singleton accessor -- the only way a plain,
  // non-capturing function pointer (WireAdapter::NowMsFn) can reach
  // back into this specific instance's state. Safe: this is only ever
  // CALLED from inside wireAdapter_'s own methods, which only run once
  // the fiber is already executing commands, well after protocol()'s
  // singleton pointer is assigned (see protocol()'s own definition
  // below).
  return protocol().clockNow();
}

// ---- Protocol loop -----------------------------------------------------

void Protocol::start() {
  if (running_) return;  // idempotent, mirrors DifferentialDrive::start()
  running_ = true;
  transport_.begin();  // size serial rings before any traffic
  // No analogous radioTransport_.begin() call here: RadioTransport self-
  // enables on first use (see ensureRadioReady(), called from
  // tryReceiveLine() in run()'s own radio-poll loop below).
  launcher_.launch(&Protocol::fiberEntry, this);
}

void Protocol::fiberEntry(void* self) {
  static_cast<Protocol*>(self)->run();
}

void Protocol::serviceOnce() {
  // Drain every line any fiber queued via emitLine() since the last
  // pass, first -- before either transport's own RX poll below, so a
  // queued line does not wait behind a receive that happens to be
  // pending. This is also the ONLY place emitQueue_ is ever drained;
  // this method itself runs only on this object's own fiber (run()'s
  // own loop, or, nested, tickDrive()'s service hook -- see
  // serviceHookEntry()'s own comment, protocol.h, for why that nesting
  // is still this same fiber), which is what makes this fiber the emit
  // path's single producer (see protocol.h's own comment on
  // emitLine()/emitLineNow()).
  drainEmitQueue();

  // Dequeue and dispatch one queued RUN job, if the drivetrain is free
  // -- see dispatchJob()'s own comment (protocol.h) for the full
  // arbitration and why this can safely run nested, from inside a job
  // that is already dispatching.
  dispatchJob();

  // Serial: drain up to kRxDrainPerPass lines, not one. One per pass
  // meant one per 24 ms while a job ran (the tick hook is the only
  // caller then), and serial at 115200 delivers ~276 bytes into a
  // 255-byte ring in that window -- so a host writing two commands
  // back-to-back overflowed the ring, and CODAL drops those bytes with
  // no signal at all. Draining what has already arrived is what keeps
  // the ring from being the thing that fills.
  for (int n = 0; n < kRxDrainPerPass; ++n) {
    size_t len = 0;
    if (!transport_.tryReadLine(lineBuf_, sizeof(lineBuf_), &len)) break;
    routeLine(wireHandler_, lineBuf_, len);
  }

  // Radio command plane (single-fragment RX): the same one inbound path
  // the serial read above takes, differing only in which WireHandler it
  // is given. Radio speaks the full v6 grammar (ack/nack, TLM, STATUS,
  // the motion verbs, etc.) through its OWN WireHandler
  // (wireHandlerRadio_, its own expectedNext_ -- so a gap on this
  // transport can never nack serial's next command, or vice versa),
  // including `RUN` itself, which since 2026-09-07 is an ordinary
  // sequenced verb on this transport like any other -- the literal
  // "RUN:" prefix fallback that used to bypass the grammar here is
  // gone.
  //
  // No enable check here: this call is the one that would bring the
  // radio up (it calls ensureRadioReady() internally), and
  // RadioTransport now refuses it itself while its link is disabled --
  // NOT touching the radio at all is what leaves it free for MakeCode's
  // own radio blocks, and that refusal is now the transport's own
  // (RadioTransport::enable(), radio_transport.h). Radio's RX path
  // stays a single fragment slot with no multi-fragment reassembly: a
  // v6 line whose encoding does not fit one fragment is out of scope
  // here.
  //
  // Bounded by the same kRxDrainPerPass as serial. Radio holds ONE
  // inbound line at a time, so a second pass usually finds nothing --
  // but the datagram handler fires on its own event, so a line can
  // land between two iterations here, and consuming it now is a slot
  // freed before the next one arrives to find it busy (that drop is
  // counted, radioRxClassify()).
  for (int n = 0; n < kRxDrainPerPass; ++n) {
    size_t radioLen = 0;
    if (!radioTransport_.tryReceiveLine(rxLineBuf_, sizeof(rxLineBuf_),
                                        &radioLen)) {
      break;
    }
    routeLine(wireHandlerRadio_, rxLineBuf_, radioLen);
  }

  // The WiFi transport, gated exactly like the radio: nothing touches
  // UARTE1 until enableWifi() -- see serviceWifi() for what one pass
  // does.
  if (wifiEnabled_) serviceWifi();

  // The reliability layer's per-line ack/nack (WireHandler::dispatch(),
  // wire_handler.cpp) is the ENTIRE reliability plane, full stop: it
  // fires once per inbound line, driven from feed()/onLineComplete()
  // above, completely independent of this timing gate. This gate
  // exists ONLY to pace telemetry FRAMES for a subscribed host: when a
  // host has subscribed (wireAdapter_.telemetryEnabled()),
  // buildSnapshot() is called ONCE per tick and the SAME Snapshot
  // reference is handed to BOTH handlers' emitTelemetry() -- not once
  // per handler (buildSnapshot() mutates odometry and advances seq_, so
  // building it twice would double both for no benefit, and would
  // report different seq/now values to serial vs radio for what should
  // read as "the same instant").
  //
  // NO unsolicited ack/nack of any kind exists on any path (an ack or a
  // nack is only ever a response to a message, never a beacon). A lost
  // ack/nack heals via the host's own retransmit (tools/robotlink.py's
  // send_until()) or poll, one round trip later. A future reader must
  // NOT "restore" a periodic, rate-limited, gap-gated, or
  // telemetry-carried re-emission here -- that would reintroduce a
  // free-running beacon this design deliberately has none of.
  const uint32_t now = clockNow();  // [ms]
  if (static_cast<int32_t>(now - lastEmit_) >=
      static_cast<int32_t>(kTelemetryEmitPeriod)) {
    if (wireAdapter_.telemetryEnabled()) {
      const Wire::Snapshot& snapshot = wireAdapter_.buildSnapshot();
      wireHandler_.emitTelemetry(snapshot);
      // Asked of the transport, which owns the answer. sendLine() would
      // refuse a disabled link on its own, so this gate is not what
      // keeps the radio off the air -- it is what keeps
      // wireHandlerRadio_ from formatting and, more to the point,
      // advancing its own header state for frames that could never go
      // anywhere. Serial telemetry just above is unconditional.
      if (radioTransport_.enabled()) {
        wireHandlerRadio_.emitTelemetry(snapshot);
      }
      // WiFi frames are additionally gated by the link's own throttle
      // (the wifi-link note, section 7.1: >= 50 ms between periodic
      // pushes AND room in the bounded send queue) -- the measured
      // heap-exhaustion wedge of the unthrottled reference. Replies and
      // acks never pass through this gate.
      if (wifiEnabled_ && wifiLink_.telemetryAllowed()) {
        wifiLink_.markTelemetry(true);   // tag the frame's lines purgeable
        wireHandlerWifi_.emitTelemetry(snapshot);
        wifiLink_.markTelemetry(false);
      }
    }
    lastEmit_ = now;
  }

  // TLM NOW's one-shot frame: independent of the periodic timer just
  // above and of whether a subscription is even active -- see
  // WireAdapter::consumeOneShotTelemetry()'s own comment for why a
  // one-shot request must not wait for either. Reuses the exact same
  // buildSnapshot()/emitTelemetry() pair the periodic block above uses,
  // called one additional time here rather than through any new
  // emission path, and mirrors that block's own per-transport gating
  // exactly so a one-shot frame reaches every transport a periodic one
  // would have.
  if (wireAdapter_.consumeOneShotTelemetry()) {
    const Wire::Snapshot& snapshot = wireAdapter_.buildSnapshot();
    wireHandler_.emitTelemetry(snapshot);
    if (radioTransport_.enabled()) {
      wireHandlerRadio_.emitTelemetry(snapshot);
    }
    if (wifiEnabled_ && wifiLink_.telemetryAllowed()) {
      wifiLink_.markTelemetry(true);
      wireHandlerWifi_.emitTelemetry(snapshot);
      wifiLink_.markTelemetry(false);
    }
  }
}

void Protocol::run() {
  // Must come before anything else in this function: see
  // paintStackCanary()'s own comment above for why this fiber's frame
  // has to still be as shallow as possible at the moment it runs.
  paintStackCanary();

  // Captured once, the first (and only) time this fiber body executes
  // -- see serviceHookEntry()'s own comment (protocol.h) for the whole
  // reason this fiber's own identity has to be knowable at all. Same
  // "read now that this fiber is actually executing" timing buildIdentity()
  // below needs, for the same underlying reason: neither is safe or
  // meaningful before this fiber has actually started.
  protocolFiberId_ = currentFiberFn_();

  // Real identity, read now that this fiber is actually executing --
  // see buildIdentity()'s own comment (protocol.h) for why this is
  // deliberately NOT done at Protocol construction time. Must happen
  // before sendBanner() below, which reads identity() through
  // wireAdapter_.
  wireAdapter_.setIdentity(buildIdentity());

  // Boot banner: byte-identical to HELLO's own reply
  // (wire_handler.cpp's sendBanner()), sent here before this loop ever
  // blocks on a read, so it goes out unsolicited the moment this fiber
  // starts. HELLO re-sends the identical banner on request via
  // wireHandler_'s own dispatch.
  wireHandler_.sendBanner();

  lastEmit_ = clockNow();

  // Register this fiber's own servicing as tickDrive()'s service hook --
  // see serviceHookEntry()'s own comment (protocol.h) for what it does
  // and why it only ever runs on THIS fiber, regardless of which fiber
  // called tickDrive().
  registerTickServiceHook(&Protocol::serviceHookEntry);

  while (true) {
    serviceOnce();

    // Wire-issued motion tracking: with the kernel's own background
    // fiber removed (shims.cpp), a wire-issued WHEELS_V has no student
    // loop left to keep ticking it. This fiber still owns the actual
    // tickDrive() call -- a CODAL-fiber concern wireAdapter_ must never
    // touch -- driven by wireAdapter_'s own tracked deadline.
    // motionOwner_ tracks kWire for as long as the obligation stays
    // live (spanning many passes of this loop, not just one tick), and
    // drops back to kNone the first pass it clears -- see this field's
    // own comment (protocol.h).
    if (wireAdapter_.hasLiveMotionObligation()) {
      motionOwner_ = MotionOwner::kWire;
      tickDrive();
    } else {
      if (motionOwner_ == MotionOwner::kWire) {
        motionOwner_ = MotionOwner::kNone;
      }
      // Cooperative yield -- lets the kernel's own fiber (and any
      // other) run between polls; never spins.
      //
      // THIS IS THE MEASURED CRASH SITE. GCC keeps this function's live
      // pointers in the callee-saved FPU registers s16-s31, and CODAL's
      // context switch does not save them, so anything parked here is
      // destroyed by the next fiber that runs float code. MEASURED gopiv
      // 2026-09-01: `&radioTransport_` came back as float -25.0f -- a
      // wheel speed -- and the dereference took a precise bus error.
      // Every yield in this extension must go through the guarded
      // wrapper; see the yield-discipline invariant in the design notes.
      vfpSafeSleep(kPollInterval);
    }
  }
}

namespace {
Protocol* gProtocol = nullptr;
}  // namespace

Protocol& protocol() {
  if (gProtocol == nullptr) {
    gProtocol = new Protocol();
    gProtocol->start();
  }
  return *gProtocol;
}

// Boot-time auto-start wiring: called once from a top-level statement in
// `blocks/motion.ts`'s `diffDrive` namespace (see protocol()'s doc comment
// in protocol.h), so the protocol loop -- and its boot banner -- start as
// soon as this extension's compiled code loads, independent of whether
// any block is ever placed in a user's program. `protocol()`'s own
// lazy-singleton guard makes this call (and any other) idempotent.
//%
void startProtocol() { protocol(); }

// Free-function entry point for the "setup radio" block's shim
// (`_setupRadio`, `blocks/sim.ts`): same lazy-singleton Protocol&
// access pattern as startProtocol() just above -- protocol()'s own
// guard makes this call safe (and idempotent) regardless of whether
// the protocol fiber has started yet.
//
// This is the ONLY thing that turns the v6 radio link on. Until a
// program calls it, the radio is never enabled at all and MakeCode's
// own `radio.*` blocks own the air -- see RadioTransport::enable()'s
// own comment (radio_transport.h).
//%
void setupRadio(int channel, int group) {
  protocol().setupRadio(static_cast<uint8_t>(channel),
                        static_cast<uint8_t>(group));
}

// Free-function entry point for `_enableRadioLink` (blocks/sim.ts), the
// hidden block test/test.ts uses to bring the radio up on its
// deploy-injected channel. See Protocol::enableRadio() for why this is
// separate from setupRadio() rather than a defaulted argument.
//%
void enableRadioLink() { protocol().enableRadio(); }

// Free-function entry point for `_enableWifiLink` (blocks/sim.ts) --
// the WiFi twin of enableRadioLink() just above.
//%
void enableWifiLink() { protocol().enableWifi(); }

}  // namespace diffDrive
