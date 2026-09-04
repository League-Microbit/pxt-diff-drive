// protocol.cpp -- see protocol.h.
#include "protocol.h"

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
// see invokeRunDispatch() below. Returns false with nothing registered
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
//   - name: microbit_friendly_name() (silicon-derived, unique per
//     board). mbdeploy keys its device registry off this field, so a
//     fixed string here would stomp the registry across the fleet.
//   - serial: microbit_serial_number() -- genuinely unique per device.
//   - kDrivetrain = this extension's kinematic type (matches the
//     package name); kProfile = which robot's config this hex was
//     BUILT AGAINST -- deploy-time build-target selection, NOT board
//     identity (radio-robot-lib's per-robot config filename stem, e.g.
//     "vevov" or "tovez" -- matches the elite reference design's
//     Config::kRobotProfileName, "baked from the robot JSON's own ...
//     filename stem"). Injected per-robot at DEPLOY time, into the
//     SCRATCH COPY only, by tools/make_deploy.py's _inject_profile() --
//     the same scratch-copy-only substitution _inject_radio_channel()
//     performs for radio_transport.h's kChannel. This repo's own
//     checked-in literal below is therefore never a real fleet robot
//     name: it is the UN-BAKED default, so a build run with no
//     --robot (or built outside make_deploy.py entirely) cannot be
//     mistaken for, or impersonate, any board on the fleet. (Before
//     this injection existed, kProfile was a hand-written constant
//     frozen fleet-wide at "tovez" -- every board, including vevov,
//     reported "tovez" over ID. That was the defect this injection
//     fixes for BUILD PROVENANCE; it does not make kProfile board
//     identity -- `name` above (identity.name) is the wire's sole
//     authoritative board identity, because it is read from silicon at
//     call time and cannot be stale, whereas `profile` is only ever as
//     correct as the build that produced it. `profile` and `name` can
//     legitimately disagree on one ID reply: `profile` says which
//     robot's config the hex targeted, `name` says which physical
//     board is actually answering. That disagreement IS the
//     diagnostic -- it means this board was flashed with the wrong
//     robot's build -- so never "fix" it by forcing the two to match.
//     Note this also decouples kProfile from shims.cpp's Rig tuning
//     defaults -- which bake those constants are actually measured
//     from is a separate, currently-contradictory question, tracked as
//     its own issue, not settled by this constant.)
//   - kVersion: the BUILD version, `0.YYYYMMDD.n`, injected at deploy
//     time by tools/make_deploy.py's _inject_version() from
//     pyproject.toml (which config/dotconfig.yaml keeps in step via
//     `dotconfig version bump`). Same scratch-copy-only mechanism as
//     kProfile and kChannel, so the checked-in literal below is a
//     PLACEHOLDER, never a real build's version.
//
//     It used to mirror pxt.json's "1.0.10"-style EXTENSION semver
//     instead. That was changed 2026-08-27, by stakeholder direction,
//     because the extension version is a poor answer to the question
//     VER actually gets asked: "which build is on this robot?" It moves
//     only when the extension is released, so every firmware built
//     between two releases reports an identical VER -- and on
//     2026-08-27 that cost hours, with two robots on visibly different
//     firmware both answering `ver 1.0.10` and nothing on the wire able
//     to tell them apart. The build version moves on every commit, so
//     VER (and ID's third field) now identify the image.
//
//     pxt.json's semver is unaffected and still governs how MakeCode
//     resolves the extension -- a different question, deliberately not
//     answered by this constant any more.
constexpr const char* kDrivetrain = "diffdrive";
constexpr const char* kProfile = "unbaked";
constexpr const char* kVersion = "unbaked";  // injected by make_deploy.py;
                                              // see the note above

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

// The old-style cleartext RUN carve-out (see protocol.h's own top-of-file
// comment): detected directly by its literal prefix now that the v5
// verb registry that used to recognize it is gone.
constexpr char kOldRunPrefix[] = "RUN:";
constexpr size_t kOldRunPrefixLen = 4;

// Poll granularity between transport reads, and the telemetry frame
// emission cadence for a TLM-subscribed host (2026-08-26: this paces
// FRAMES only -- the reliability line that used to ride each frame is
// deleted, S8.5): small enough that a command arriving just after one
// poll is still picked up promptly; not so small it spins this fiber
// against an idle UART between bytes.
constexpr uint32_t kTelemetryEmitPeriod = 50;  // [ms]
constexpr uint32_t kPollInterval = 5;  // [ms]

// Bypass names for handleRun() below: these two skip runQueue_ entirely
// and dispatch immediately, regardless of what else is running -- see
// handleRun()'s own comment. `name` is the payload up to (not including)
// its first ':', or the whole payload if there is none, matching how
// the TS dispatcher itself splits a command into name + arguments.
bool isBypassRunName(const char* text) {
  size_t nameLen = 0;
  while (text[nameLen] != '\0' && text[nameLen] != ':') ++nameLen;
  auto matches = [&](const char* name) {
    return std::strlen(name) == nameLen &&
           std::memcmp(text, name, nameLen) == 0;
  };
  return matches("abort") || matches("clearestop");
}

}  // namespace

void Protocol::emitLine(const char* text) {
  if (text == nullptr) return;
  size_t len = 0;
  // Sprint 008 ticket 002 (WIRE-05/R-21): this clip now names
  // RadioTransport::kMaxPayloadBytes directly instead of re-declaring
  // its own bare 200 literal. The bare literal had drifted from a
  // parity claim into a silent defect: SerialTransport::kMaxLineBytes
  // was raised to 240 (sprint 004 ticket 005), radio_transport.h's own
  // doc comment kept claiming this cap "equals" that bound, and this
  // clip stayed at 200 regardless -- so a 201-239 byte result line
  // truncated silently on ANY transport, not only radio's. Naming the
  // shared constant closes that gap: kMaxPayloadBytes is deliberately
  // the TIGHTER of the two transports' caps (radio's real capacity, not
  // serial's 240 -- see radio_transport.h's own updated comment), so a
  // line this call clips never depends on which transport happens to
  // carry it, and the two can never drift apart silently again.
  // kMaxPayloadBytes moved from private to public on RadioTransport to
  // make this reference possible (one-line access-specifier change, no
  // encapsulation cost -- radio_transport.h).
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
  // RadioTransport::sendLine() now guards its shared scratch buffers
  // against the protocol fiber's own RadioSink::write() calls (ticket
  // 002); false means the guard fired and this line was dropped
  // untouched. This is the one caller whose loss is user-visible (a
  // test's own recorded result, e.g. an OCAL: corner fix), so it gets
  // exactly one fiber_sleep(2)-and-retry -- not a loop -- before giving
  // up silently, per sprint.md's Design Rationale.
  // Gate 2 of 3 (see radioEnabled_'s own comment, protocol.h): without
  // this, the first debug line a student emits would call sendLine(),
  // which lazily calls ensureRadioReady() and claims the radio out from
  // under MakeCode's own radio blocks. Serial above is unconditional --
  // serial is always v6.
  if (!radioEnabled_) return;
  if (!radioTransport_.sendLine(reinterpret_cast<const uint8_t*>(text),
                                len)) {
    vfpSafeSleep(2);
    (void)radioTransport_.sendLine(reinterpret_cast<const uint8_t*>(text),
                                   len);
  }
}

// Drains emitQueue_ into emitLineNow(), oldest line first. Copies each
// line out to a local, on-this-fiber's-stack buffer before calling
// emitLineNow() -- that call can yield (the radio retry above), and
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

void Protocol::setupRadio(uint8_t channel, uint8_t group) {
  // Order matters: configure, THEN enable. Both setters only store while
  // the radio is down, and ensureRadioReady() reads the stored values
  // when it later brings it up -- so doing it this way means the radio
  // comes up already on the requested channel and group, and the
  // mid-run re-apply path (unverified for the channel; see
  // RadioTransport::setChannel()) is never exercised by this call.
  radioTransport_.setChannel(channel);
  radioTransport_.setGroup(group);
  radioEnabled_ = true;
}

void Protocol::enableRadio() { radioEnabled_ = true; }

void Protocol::enableWifi() { wifiEnabled_ = true; }

void Protocol::emitWifiDebug() {
  // One line, cleartext `DBG:` prefix (the same convention the TS
  // layer's debug output uses), through emitLine() so it reaches
  // serial, radio AND -- once up -- the WiFi host itself.
  snprintf(wifiDbgBuf_, sizeof(wifiDbgBuf_),
           "DBG:wifi state=%d ip=%s peer=%s:%u tcp=%u/%d to=%d restarts=%lu "
           "sent=%lu rx=%lu drop=%lu mdns=%lu/%d cmd=%s reply=%s",
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
           wifiLink_.lastCommand(), wifiLink_.lastReply());
  emitLine(wifiDbgBuf_);
}

void Protocol::serviceWifi() {
  if (!wifiBegun_) {
    wifiBegun_ = true;
    WifiLink::Config config;
    config.ssid = kWifiSsid;
    config.password = kWifiPassword;
    // The mDNS host label is the board's own silicon-derived name --
    // the same authoritative identity ID's `name` field reports -- so
    // `tovez.local` / "tovez robot link" can never be a stale bake.
    config.hostname = microbit_friendly_name();
    config.port = kWifiPort;
    config.hostPort = kWifiHostPort;
    wifiLink_.begin(config);
    lastWifiDbg_ = static_cast<uint32_t>(clock_.nowMicros() / 1000ull);  // [ms]
    emitWifiDebug();
  }

  wifiLink_.service();

  const uint32_t now = static_cast<uint32_t>(clock_.nowMicros() / 1000ull);  // [ms]
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

  // Inbound: one datagram is one line. Bounded per pass so a host
  // blasting lines cannot starve the rest of serviceOnce().
  for (int i = 0; i < WifiLink::kRxSlots; ++i) {
    size_t len = 0;
    if (!wifiLink_.tryReceiveLine(wifiRxBuf_, WifiLink::kMaxLineBytes, &len)) break;
    if (len >= kOldRunPrefixLen &&
        std::memcmp(wifiRxBuf_, kOldRunPrefix, kOldRunPrefixLen) == 0) {
      handleRun(wifiRxBuf_ + kOldRunPrefixLen, len - kOldRunPrefixLen);
    } else {
      wireHandlerWifi_.feed(reinterpret_cast<const char*>(wifiRxBuf_), len);
      wireHandlerWifi_.feed("\n", 1);
    }
  }
}

// ---- the old-style cleartext RUN bridge ------------------------------

void Protocol::handleRun(const uint8_t* data, size_t dataLen) {
  if (data == nullptr || dataLen == 0) return;
  // Strip one trailing '\r' (raw-terminal artifact, same tolerance the
  // old cleartext line parser gave colon-less lines), then copy the payload
  // verbatim. Anything outside printable ASCII -- or too long for a
  // slot -- is malformed: drop silently. The name/argument split is NOT
  // done here: this layer stays a transport for the text, and the TS
  // layer owns the vocabulary.
  if (data[dataLen - 1] == '\r') --dataLen;
  if (dataLen == 0 || dataLen >= kRunTextBytes) return;
  char text[kRunTextBytes];
  for (size_t i = 0; i < dataLen; ++i) {
    const uint8_t c = data[i];
    if (c < 0x20 || c > 0x7E) return;
    text[i] = static_cast<char>(c);
  }
  text[dataLen] = '\0';
  if (text[0] == ':') return;   // empty name -- nothing to dispatch on

  // Dedupe repeats of the SAME command. The robot's inbound wireless
  // path is a single-slot buffer, so hosts repeat commands to survive
  // loss -- a repeated RUN does not hit the test programs' own re-entry
  // guard (which has already cleared by the time a retransmit lands):
  // measured on vevov, one 3x-repeated RUN:4 ran three consecutive
  // 180 deg pivots.
  //
  // Suppression is by (text, arrival time) here at the point of
  // arrival, NOT at handling time, which is what makes it immune to
  // that queueing. Two commands that differ only in their arguments
  // are different text, so they are not each other's repeats. A
  // deliberate re-run of the same command just needs to be spaced past
  // the window.
  const uint32_t now = static_cast<uint32_t>(clock_.nowMicros() / 1000ull);  // [ms]
  if (std::strcmp(lastRunText_, text) == 0 &&
      static_cast<int32_t>(now - lastRun_) < kRunDedupe) {
    lastRun_ = now;   // extend across a burst of repeats
    return;
  }
  std::memcpy(lastRunText_, text, dataLen + 1);
  lastRun_ = now;

  // Abort/clearestop bypass runQueue_ entirely and dispatch RIGHT NOW,
  // on whatever fiber called handleRun() -- run()'s own loop normally,
  // but (crucially) also the service hook nested inside a running job's
  // own tick loop, which is the ONLY way an abort sent while a job is
  // mid-tour can ever be noticed: dispatchJob() refuses to start a
  // SECOND job while motionOwner_ is not kNone, so if abort went through
  // the same queued path as everything else it would sit behind the
  // very job it is meant to stop. Ungated on motionOwner_ deliberately
  // -- both handlers this dispatches to (test.ts's "abort"/"clearestop")
  // are trivial, non-blocking, and safe to invoke reentrant from inside
  // a job's own call chain.
  if (isBypassRunName(text)) {
    invokeRunDispatch(text);
    return;
  }

  if (runQueue_.enqueue(text, static_cast<int>(dataLen)) < 0) {
    // Every slot is still in flight. Refusing is the point: the old
    // cursor would have overwritten one, and the handler holding it
    // would then have run a command nobody sent. The refusal is
    // counted and readable rather than silent, so a host that
    // out-runs the robot can find out.
    return;
  }
  // dispatchJob() drains runQueue_ itself, in arrival order -- nothing
  // else to do here once the enqueue above has accepted the command.
}

void Protocol::dispatchJob() {
  if (motionOwner_ != MotionOwner::kNone) return;
  const int slot = runQueue_.peek();
  if (slot < 0) return;  // nothing queued

  // Copy the text out and release the slot BEFORE dispatching -- the
  // call below can run for as long as the job itself does (a whole
  // tour), and releasing first frees runQueue_'s capacity for a burst
  // arriving during that whole span instead of holding one slot hostage
  // for it.
  setCurrentRunText(runQueue_.at(slot));
  runQueue_.release(slot);

  motionOwner_ = MotionOwner::kJob;
  wireAdapter_.setJobOwnsMotion(true);
  runDispatch();
  wireAdapter_.setJobOwnsMotion(false);
  motionOwner_ = MotionOwner::kNone;
}

void Protocol::invokeRunDispatch(const char* text) {
  setCurrentRunText(text);
  runDispatch();
}

void Protocol::setCurrentRunText(const char* text) {
  std::strncpy(currentRunText_, text, kRunTextBytes - 1);
  currentRunText_[kRunTextBytes - 1] = '\0';
}

const char* Protocol::currentRunText() const { return currentRunText_; }

void Protocol::serviceHookEntry() {
  if (protocol().motionOwner_ != MotionOwner::kJob) return;
  protocol().serviceOnce();
}

uint32_t Protocol::runDropCount() const { return runQueue_.dropped(); }
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

uint32_t Protocol::wireNow() {  // [ms]
  // Reuses this Protocol instance's own clock_ (unchanged member, still
  // backing handleRun()'s dedupe timing too) via the protocol()
  // singleton accessor -- the only way a plain, non-capturing function
  // pointer (WireAdapter::NowMsFn) can reach back into this specific
  // instance's state. Safe: this is only ever CALLED from inside
  // wireAdapter_'s own methods, which only run once the fiber is
  // already executing commands, well after protocol()'s singleton
  // pointer is assigned (see protocol()'s own definition below).
  return static_cast<uint32_t>(protocol().clock_.nowMicros() / 1000ull);
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

  size_t len = 0;
  if (transport_.tryReadLine(lineBuf_, sizeof(lineBuf_), &len)) {
    if (len >= kOldRunPrefixLen &&
        std::memcmp(lineBuf_, kOldRunPrefix, kOldRunPrefixLen) == 0) {
      // Old-style cleartext RUN, preserved unchanged -- see this
      // file's own top-of-file comment (protocol.h) for why it's
      // detected here by literal prefix rather than through a verb
      // registry (the v5 registry that used to do this is gone).
      handleRun(lineBuf_ + kOldRunPrefixLen, len - kOldRunPrefixLen);
    } else {
      // Every other line -- including the v6 grammar's own
      // space-separated "RUN <name> ... #<id>" verb -- goes to the
      // v6 wire stack. feed() reassembles regardless of chunking; the
      // trailing '\n' it needs to recognize the line as complete is
      // fed as a second, separate call.
      wireHandler_.feed(reinterpret_cast<const char*>(lineBuf_), len);
      wireHandler_.feed("\n", 1);
    }
  }

  // Radio command plane (single-fragment RX): mirrors the serial
  // branch's own dual-path logic above exactly. Radio speaks the full
  // v6 grammar (ack/nack, TLM, STATUS, the motion verbs, etc.) through
  // its OWN WireHandler (wireHandlerRadio_), with the old-style literal
  // "RUN:" prefix preserved as a fallback, unchanged, exactly as
  // protocol.h's own top-of-file comment describes. This call also
  // lazily brings the radio up on its first invocation
  // (RadioTransport::tryReceiveLine() calls ensureRadioReady()
  // internally) -- the same "unconditionally from this fiber's boot
  // path" cost the old banner/TLM radio mirror used to pay, just via
  // the receive side now that there is no v6 reply mirror to pay it
  // instead. Radio's RX path stays a single 64-byte fragment slot with
  // no multi-fragment reassembly: a v6 line whose encoding does not fit
  // one fragment is out of scope here.
  // Gate 1 of 3 (see radioEnabled_'s own comment, protocol.h). This
  // poll is what used to bring the radio up at boot, unconditionally,
  // for every program -- which is exactly why a student's joystick
  // program could never work. tryReceiveLine() calls
  // ensureRadioReady() internally, so NOT calling it at all is what
  // leaves the radio free for MakeCode's own radio blocks.
  size_t radioLen = 0;
  if (radioEnabled_ &&
      radioTransport_.tryReceiveLine(rxLineBuf_, sizeof(rxLineBuf_),
                                     &radioLen)) {
    if (radioLen >= kOldRunPrefixLen &&
        std::memcmp(rxLineBuf_, kOldRunPrefix, kOldRunPrefixLen) == 0) {
      // Old-style cleartext RUN, preserved unchanged as a fallback --
      // see protocol.h's own top-of-file comment for why it's
      // detected here by literal prefix rather than through the v6
      // grammar's own verb lookup.
      handleRun(rxLineBuf_ + kOldRunPrefixLen,
               radioLen - kOldRunPrefixLen);
    } else {
      // Every other line -- including the v6 grammar's own
      // space-separated "RUN <name> ... #<id>" verb -- goes to
      // radio's own v6 wire stack. wireHandlerRadio_ keeps its own
      // independent expectedNext_ (wifi-link.md:373),
      // so a gap on this transport can never nack wireHandler_'s
      // (serial's) next command, or vice versa.
      wireHandlerRadio_.feed(reinterpret_cast<const char*>(rxLineBuf_),
                             radioLen);
      wireHandlerRadio_.feed("\n", 1);
    }
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
  const uint32_t now = static_cast<uint32_t>(clock_.nowMicros() / 1000ull);  // [ms]
  if (static_cast<int32_t>(now - lastEmit_) >=
      static_cast<int32_t>(kTelemetryEmitPeriod)) {
    if (wireAdapter_.telemetryEnabled()) {
      const Wire::Snapshot& snapshot = wireAdapter_.buildSnapshot();
      wireHandler_.emitTelemetry(snapshot);
      // Gate 3 of 3 (see radioEnabled_'s own comment, protocol.h):
      // wireHandlerRadio_ sinks through radioSink_ into
      // RadioTransport::sendLine(), which lazily enables the radio.
      // Serial telemetry just above is unconditional.
      if (radioEnabled_) {
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
}

void Protocol::run() {
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

  lastEmit_ = static_cast<uint32_t>(clock_.nowMicros() / 1000ull);

  // Register this fiber's own servicing as tickDrive()'s service hook --
  // see serviceHookEntry()'s own comment (protocol.h) for what it does
  // and why it is a deliberate no-op outside a dispatched job's own
  // call span.
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
// own `radio.*` blocks own the air -- see radioEnabled_'s comment in
// protocol.h.
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
