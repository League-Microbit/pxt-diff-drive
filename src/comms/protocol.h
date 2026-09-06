// protocol.h -- Protocol: the CODAL protocol fiber and byte plumbing
// between SerialTransport/RadioTransport and the v6 wire stack
// (wire_handler.h/.cpp, wire_adapter.h/.cpp). Knows nothing of the v6
// grammar, the reliability layer, or any verb's own behavior -- all of
// that lives behind wireHandler_/wireHandlerRadio_/wireAdapter_.
//
// One exception, preserved deliberately: the OLD cleartext
// "RUN:<name>[:<arg>...]" bridge (handleRun()/dispatchJob()/the
// runBridge_ object below) coexists with v6 on the same wire -- detected
// directly by its literal "RUN:" prefix before a line ever reaches the
// v6 stack (no verb registry involved -- see run()'s own comment). It
// is the ONLY path that feeds the by-name test-trigger dispatch
// test.ts actually uses: v6's own RUN verb (wire_adapter.cpp's
// WireAdapter::onRun()) is kUnknown.
//
// The radio transport speaks the full v6 grammar too, through a SECOND
// WireHandler (wireHandlerRadio_, below) composed over the SAME
// wireAdapter_ instance the serial handler uses -- not a second
// adapter: two adapters would let a sequence gap on one transport nack
// the OTHER transport's next command, which is exactly the corruption
// the second-handler structure exists to prevent. The old-style
// cleartext RUN: carve-out above is preserved on radio too, as a
// fallback, unchanged -- see run()'s own radio-polling block.
// `emitLine()` below -- the free function shims.cpp's test-result
// reporting already uses -- queues onto a Protocol-owned ring rather
// than writing to either transport directly; this fiber's own loop
// drains it every pass. An untethered bench run's results still reach
// a listening host, just through one more level of indirection than
// before, and only ever written by this fiber.
//
// This project's own telemetry is real and shipped on the v6 wire
// stack: WireHandler::emitTelemetry(Snapshot) (see its own doc comment
// for the thdr/t frame format) replaces the old v5 cleartext
// "TLM:<ms>:..." line entirely. tools/tour_run.py, tools/tour_capture.py,
// tools/tour_watch.py, and this repo's other bench scripts read the
// thdr/t stream directly -- retrofitted onto it in full; no TLM:
// parsing remains anywhere in this tree.
#pragma once

#include <cstddef>
#include <cstdint>

#include "../core/motion_owner.h"  // MotionOwner: shared with wire_adapter.h's own mirror
#include "../platform/platform_ports.h"  // CodalFiberLauncher, CodalClock (reused, not reimplemented)
#include "radio_transport.h"  // radio transport -- now a full v6 sink too
#include "serial_transport.h"
#include "wifi_link.h"        // WiFi transport (host-portable AT state machine)
#include "wifi_uart.h"        // ...over NRF_UARTE1 (CODAL-free header)
#include "wire_adapter.h"
#include "run_bridge.h"
#include "emit_queue.h"
#include "transport_sink.h"  // the ONE Sink all three transports use
#include "wire_handler.h"

namespace diffDrive {

// ---- Protocol loop -----------------------------------------------------
class Protocol {
 public:
  // Starts the protocol loop on its own CODAL fiber via
  // CodalFiberLauncher. Idempotent, mirroring
  // DifferentialDrive::start()'s own idempotent guard.
  void start();

  // Queue one caller-supplied text line for emission on BOTH transports
  // -- this consolidates what used to be two separate single-transport
  // emitters into the one path anything wanting both wires mirrored now
  // uses. Exists because the test programs' result lines (tour fixes,
  // calibration data, timings) were written with TypeScript's
  // `serial.writeLine`, which reaches the USB cable only -- and the USB
  // cable only reaches the bench stand, where the wheels are off the
  // ground. Every test that needs the robot to actually move therefore
  // runs untethered, and its results have to come back over the radio.
  //
  // Called from the TS layer (shims.cpp's emitLine), on whatever fiber
  // that call happens to run on -- NOT this object's own fiber. This
  // clips the line and copies it into emitQueue_, then returns: it no
  // longer touches transport_/radioTransport_ itself. Only this
  // object's own fiber (Protocol::run(), via drainEmitQueue()) ever
  // writes either transport, so two fibers can never race the same
  // underlying serial write again. The tradeoff: a caller can no longer
  // assume the line is physically on the wire by the time this call
  // returns, only that it is queued for the next drain pass (at most
  // one poll interval later); and a full ring drops the newest line
  // rather than blocking the caller, counted rather than silent (see
  // emitLineNow()'s own comment for where the actual writes happen).
  void emitLine(const char* text);

  // The text of whichever RUN command is CURRENTLY being dispatched --
  // the whole payload after `RUN:`, e.g. "pivot:180". Valid only while
  // dispatchJob()'s (or the abort/clearestop bypass's) own call into the
  // registered RUN dispatch callback is executing, on THIS fiber -- see
  // RunBridge::currentText()'s own comment for why a nested reentrant
  // dispatch (abort arriving mid-job) can never corrupt an outer job's
  // already-consumed text. Called from the TS layer (shims.cpp's
  // now-zero-argument runCommandText() -- the old
  // MessageBus-event-carries-a-slot-number indirection this used to
  // read through is gone).
  const char* currentRunText() const;

  // The take/release pair every MOTION entry point in shims.cpp
  // (startMove()/driveTwist()/engineGoToRArmed()) brackets its own call
  // span with -- tryTakeMotionOwnership() up front, releaseBlockOwnership()
  // (below) once tickDrive()/the starvation watchdog/the explicit stop
  // paths (endMove()/stopAll()/estopAll()) next find the drivetrain idle.
  //
  // This used to be a plain kBlock take (tryTakeBlockOwnership()) --
  // refused unconditionally whenever motionOwner_ was anything but
  // kNone, INCLUDING kJob. That refused a dispatched RUN job's own
  // move: dispatchJob() (below) sets motionOwner_ = kJob and then calls
  // the TS handler SYNCHRONOUSLY, on THIS fiber, so the handler's own
  // startMove()/driveTwist() call was indistinguishable from a genuine
  // block-program caller arriving mid-job -- both saw motionOwner_ ==
  // kJob and were refused, silently (the caller still emitted its
  // normal completion receipts). tryTakeMotionOwnership() (below)
  // resolves that: it computes whether THIS call is running on
  // Protocol's own fiber (the same fiber-identity comparison
  // serviceHookEntry() already makes, core/fiber_identity.h) and, if
  // so AND motionOwner_ is already kJob, lets the call through
  // unchanged -- that is the job's OWN move, not a competitor for the
  // drivetrain. A genuine block caller (a different fiber -- a button
  // handler, a student script) still applies the UNCHANGED kBlock
  // take/refuse rule: refused, never silently superseding, whenever
  // motionOwner_ is kWire, kJob, or an already-taken kBlock. Both rules
  // are the SAME pure decision core/motion_owner.h defines
  // (tryTakeMotionOwnership(), host-tested there directly) -- a release
  // is a no-op unless currently kBlock, exactly as before (the job-own-
  // fiber bypass never sets kBlock, so it has nothing here to release;
  // dispatchJob() itself owns clearing kJob once runDispatch() returns).
  // Public (not private, unlike motionOwner_ itself) so the free-
  // function seam beside these methods' own definitions (protocol.cpp)
  // can reach them from shims.cpp, the same "public method, free-
  // function forward-declaration wrapper" shape currentRunText() above
  // already uses for protocolCurrentRunText().
  bool tryTakeMotionOwnership();
  void releaseBlockOwnership();

  // Cleartext RUN payloads refused because every slot was still
  // in flight. Saturates rather than wrapping -- a drop count
  // that rolls to zero reads as "nothing was lost".
  uint32_t runDropCount() const;

  // Cleartext RUN payloads refused by RunBridge's own sanitizer --
  // empty, overlong, non-printable, or an empty name -- surfaced for
  // shims.cpp's diagValue(30)/probe(30). Kept apart from
  // runDropCount() above on purpose: a malformed line and a full ring
  // are different failures, and a bench operator seeing one climb
  // needs to know which. From the relay, an uncounted malformed refusal
  // is indistinguishable from radio loss.
  uint32_t runMalformedCount() const;

  // The radio RX path's four diagnostics, surfaced for
  // diagValue(31..34)/probe(31..34) in that order: datagrams that
  // arrived as a complete single-fragment line, lines actually
  // delivered into the RX slot, lines dropped because the previous one
  // was still unconsumed, and lines dropped because they were longer
  // than the RX buffer.
  //
  // frames - accepted is the whole story of what the radio heard and
  // could not keep. The first two used to exist as members nothing ever
  // incremented, which answered that question with a permanent,
  // confident zero.
  uint32_t radioRxFrameCount() const;
  uint32_t radioRxAcceptedCount() const;
  uint32_t radioRxOverrunDropCount() const;
  uint32_t radioRxOversizeDropCount() const;

  // emitLine() calls refused because emitQueue_ was already full,
  // surfaced for shims.cpp's diagValue(29)/probe(29). Same saturating
  // convention as runDropCount() above: should stay 0 across a normal
  // session.
  uint32_t emitDropCount() const;

  // SerialTransport::writeLine()'s drop counter (ticket 006), surfaced
  // for shims.cpp's diagValue(26)/probe(26). Same same-package
  // forward-declaration boundary as emitLine()/currentRunText() above:
  // shims.cpp reaches this via a free-function wrapper
  // (protocolSerialDropCount(), protocol.cpp) rather than including
  // this header directly, so it never pulls in radio_transport.h (see
  // protocol.h's own top-of-file comment on why that matters to PXT's
  // dependency scan).
  int serialDropCount() const;

  // Configure the radio AND bring the v6 radio link up -- the ONE write
  // path student blocks gain into RadioTransport's own configuration.
  // Exists because radioTransport_ (below) is private to this class; the
  // free-function shim beside startProtocol() (protocol.cpp) is this
  // method's only caller.
  //
  // Channel and group are applied BEFORE the link is enabled, so the
  // radio comes up already on the requested channel/group the first time
  // anything touches it -- the supported ordering (see
  // RadioTransport::setChannel()'s own doc comment for why the
  // already-up path is the unverified one).
  void setupRadio(uint8_t channel, uint8_t group);

  // Bring the v6 radio link up on whatever channel/group are already
  // configured -- i.e. the per-robot channel tools/make_deploy.py
  // injected into kChannel at deploy time, and group 10.
  //
  // This is what the on-robot test program (test/test.ts) calls. It
  // deliberately does NOT take a channel: hardcoding one there would
  // override the deploy injection and put every `--robot tovez` build on
  // vevov's channel. Students get setupRadio() instead, where naming the
  // channel is the point.
  void enableRadio();

  // Bring the v6 WiFi link up (Planet X Ai-WB2-12F on RJ11 J1 -- see
  // wifi_link.h). OPT-IN exactly like the radio: nothing touches
  // UARTE1 until a program calls this, and a build with no SSID baked
  // (tools/make_deploy.py's _inject_wifi_secrets()) stays disabled even
  // then. The actual begin() happens on this object's own fiber, in
  // serviceWifi(), because the mDNS hostname is the board's friendly
  // name, which is only safe to read there (see buildIdentity()).
  void enableWifi();

 private:
  static void fiberEntry(void* self);
  void run();

  // Debug-build-only scaffold: fills this fiber's own currently-unused
  // stack region with a fixed byte pattern, once, before run()'s loop
  // ever starts -- so an offline memory scan can later find the
  // deepest point any call this fiber makes actually reached. A no-op
  // body outside a debug forensics build (see the .cpp for the gating
  // macro): a normal build never runs the fill loop at all.
  static void paintStackCanary();

  // One pass of the WiFi transport's own servicing, called from
  // serviceOnce() when wifiEnabled_: lazily begin()s the link, pumps
  // its AT state machine, greets a newly-learned host with the banner,
  // polls its inbound lines into wireHandlerWifi_ (with the same
  // cleartext RUN: carve-out serial and radio get), and emits one
  // `DBG:wifi ...` diagnostic line per state change.
  void serviceWifi();
  void emitWifiDebug();

  // ---- single executor -- motionOwner_ arbitration -------------------
  // Exactly one execution model remains for engine-facing motion: this
  // fiber. motionOwner_ (MotionOwner, core/motion_owner.h) arbitrates
  // which caller currently holds the drivetrain -- kNone (idle), kWire
  // (a live wire motion obligation, set/cleared by run()'s own loop
  // around its tickDrive() call), kJob (a dispatched RUN job, set/
  // cleared by dispatchJob() around its call into the TS handler), or
  // kBlock (the block program's own fiber -- a student's own move()/
  // driveTwist()/startDrive() call, or a MessageBus button handler
  // calling one of those directly -- taken/released via
  // tryTakeMotionOwnership()/releaseBlockOwnership() below, reached from
  // shims.cpp through the free-function seam beside those methods; a
  // dispatched job's OWN call into those same shims.cpp entry points
  // takes the OTHER branch of tryTakeMotionOwnership() instead, per that
  // method's own doc comment above, and never touches kBlock at all).
  // Lives HERE, not on WireAdapter or the RUN queue, because this class
  // is the only one that can see a wire request, a dispatched job, AND
  // a block-motion call, all three; wireAdapter_.setExternalOwner()
  // (wire_adapter.h) is the one seam this class uses to mirror the same
  // value into that host-portable class, so the two never drift apart
  // as separately-maintained fields.
  MotionOwner motionOwner_ = MotionOwner::kNone;

  // Dequeues and dispatches ONE queued RUN job, if one is waiting and
  // motionOwner_ is kNone (nothing else owns the drivetrain right now --
  // covers both a live wire motion and, defensively, a job already
  // dispatching via a reentrant call, though the latter cannot actually
  // happen: see the note below). Sets motionOwner_ = kJob and tells
  // wireAdapter_ so a wire motion verb arriving while this job runs is
  // refused (kBusy) rather than silently overwriting or racing its move
  // -- both cleared again once the dispatched call returns.
  //
  // Called once per pass of run()'s own loop, after drainEmitQueue() and
  // before the wire/radio poll (serviceOnce(), below) -- but the
  // dispatched call itself can run for a long time (a whole tour), during
  // which THIS SAME fiber re-enters serviceOnce() repeatedly through
  // tickDrive()'s own service hook (see serviceHookEntry() below), which
  // calls dispatchJob() again on every re-entry. That nested call always
  // finds motionOwner_ == kJob already and returns immediately -- a job
  // is dispatched exactly once per queued command, never re-entered.
  void dispatchJob();

  // One pass of this fiber's OWN servicing: drains emitQueue_, dispatches
  // one queued RUN job if the drivetrain is free, polls serial and radio
  // for new lines (the old-style cleartext RUN: bridge or the v6
  // grammar), and emits a telemetry frame if one is due. This is run()'s
  // own former per-pass loop body (minus the final tick-or-sleep step),
  // extracted so it can ALSO run as tickDrive()'s service hook
  // (serviceHookEntry(), below): a dispatched job's own
  // `while (driveTick())` tick loop nests back into this same servicing
  // once per tick, which is what lets an abort, a new queued command, or
  // ordinary telemetry keep flowing without waiting for that job to
  // return -- inverting the pump (this fiber's own servicing rides
  // inside the job's tick loop) rather than adding a second fiber to
  // drive the job. Never calls tickDrive() itself and never
  // sleeps -- run()'s own loop (below) does both of those, once per pass,
  // strictly AFTER this returns; the ONE nested call site (tickDrive()
  // itself) is already mid-tick when this fires, so doing either here
  // would be reentrant and wrong.
  void serviceOnce();

  // How many complete inbound lines serviceOnce() drains from ONE
  // transport before moving on. Four, for three reasons that agree:
  //
  //  - One was too few. While a dispatched job runs, the tick hook is
  //    the only caller, so a pass happens once per ~24 ms tick; serial
  //    at 115200 delivers ~276 bytes into a 255-byte ring in that
  //    window, so a host that writes two commands back-to-back
  //    overflowed CODAL's ring, which drops the overflow silently.
  //  - Four is what the WiFi path already bounded itself at, so all
  //    three transports now answer to one number instead of three
  //    independent ones.
  //  - Unbounded would be wrong. Each routed line can emit several
  //    reply lines, and the emit ring holds kEmitSlots (8) of them; a
  //    host blasting commands would starve drainEmitQueue() and the
  //    telemetry cadence, both of which run only between passes.
  //
  // Not a per-transport tuning knob: if one transport ever needs its
  // own budget, that is a reason to name a second constant here, not to
  // spell a bare number at its call site.
  static constexpr int kRxDrainPerPass = 4;

  // The ONE path an inbound line takes, whichever transport produced
  // it: `data`/`len` is one complete line, delimiter already stripped by
  // the transport that framed it. A line whose first bytes are the
  // literal "RUN:" prefix goes to the old-style cleartext bridge
  // (handleRun(), below); everything else -- including the v6 grammar's
  // own space-separated "RUN <name> ... #<id>" verb -- is fed to
  // `handler`, followed by the separate "\n" feed() needs to see the
  // line as complete.
  //
  // `handler` is the caller's own WireHandler, never a fixed one: each
  // transport has its own (wireHandler_/wireHandlerRadio_/
  // wireHandlerWifi_), each with its own expectedNext_, so a sequence
  // gap on one transport can never nack another's next command. That
  // per-transport handler is the ONLY thing that ever differed between
  // the three poll branches this replaces -- they were otherwise
  // identical, which is exactly why the "RUN:" carve-out had to be
  // written out three times to stay true on all three wires.
  void routeLine(Wire::WireHandler& handler, const uint8_t* data, size_t len);

  // This fiber's own clock reading -- the ONE place clock_'s microsecond
  // counter is reduced to the millisecond scale everything above it
  // (the telemetry cadence, the WiFi debug period, RunBridge's dedupe
  // window) actually works in. Was four separate `nowMicros() / 1000`
  // conversions, one per caller.
  uint32_t clockNow();  // [ms]

  // tickDrive()'s (shims.cpp) service hook, registered once via
  // registerTickServiceHook() when run() starts -- a plain
  // no-capture function pointer (same reason wireNow() below is a
  // plain static member function, not a lambda: a bare C function
  // pointer cannot capture `this`), so it reaches this specific
  // Protocol instance through the protocol() singleton accessor, safe
  // for the same reason wireNow() is. Gates on WHICH FIBER is calling,
  // via diffDrive::shouldServiceHookRun() (core/fiber_identity.h) --
  // NOT on motionOwner_'s value, which used to be this method's whole
  // check and is exactly what let a second fiber slip through: a
  // button-handler fiber calling tickDrive() while a job ran on this
  // one satisfied `motionOwner_ == kJob` and ran serviceOnce() a SECOND
  // time, concurrently, corrupting the wire dispatcher's own shared
  // line buffer mid-yield. Comparing fiber identity instead makes "no
  // fiber but this object's own ever runs serviceOnce()" true by
  // construction, independent of what any state variable says: tickDrive()
  // is also called (a) from run()'s own loop for a live wire motion
  // obligation, which already gets its own servicing once per pass via
  // run()'s own loop calling serviceOnce() directly, and (b) from
  // anything else's own fiber -- a student's continuous-mode drive loop,
  // or a button handler -- neither of which needs or may ever trigger
  // this hook's extra work.
  static void serviceHookEntry();

  // The identity this fiber captured as its own, the first (and only)
  // time run() executes -- null before that. shouldServiceHookRun()
  // above compares currentFiberFn_()'s reading against this on every
  // tickDrive() call.
  const void* protocolFiberId_ = nullptr;

  // The injectable "current fiber" reader serviceHookEntry() calls --
  // defaults to a real CODAL global read (defaultCurrentFiber(),
  // protocol.cpp), overridable so a future on-target test can pin both
  // sides of the comparison without a real second fiber. Both ids this
  // class ever compares are opaque pointers, by identity only, never
  // dereferenced.
  using CurrentFiberFn = const void* (*)();
  static const void* defaultCurrentFiber();
  static CurrentFiberFn currentFiberFn_;

  // ---- the outbound emit path: single producer, one caller each ------
  // emitLine() (public, above) no longer writes a transport itself -- it
  // clips and enqueues onto emitQueue_ below and returns. These two
  // private methods are the split: emitLineNow() is the actual write
  // (the old emitLine() body, unchanged), and drainEmitQueue() is its
  // only caller, itself called once per pass of run()'s own loop, on
  // this object's own fiber. That makes this fiber the only caller that
  // can ever reach either transport's underlying write for this path,
  // regardless of which fiber called emitLine().
  //
  // Copies `len` bytes from `text` to serial, then (if the radio link
  // is up) mirrors the same bytes to radio with one retry -- see the
  // definition (protocol.cpp) for the retry's own reasoning. Only ever
  // called from drainEmitQueue(), so `text` always points at a local
  // buffer that outlives any yield this performs.
  void emitLineNow(const char* text, size_t len);

  // Drains every currently-queued line out of emitQueue_, in FIFO
  // order, into emitLineNow() -- called once at the top of run()'s loop,
  // before either transport's own RX poll, so a line any fiber queued
  // reaches the wire within one poll interval (kPollInterval).
  void drainEmitQueue();

  // emitQueue_'s slot text bytes: RadioTransport::kMaxPayloadBytes (the
  // cap emitLine() already clips to) plus one for the NUL this ring
  // adds itself -- a clipped line always fits. Slot count matches
  // RunBridge's own ring: generous enough for a burst of result lines
  // between drain passes without becoming a large static allocation.
  static constexpr size_t kEmitTextBytes = RadioTransport::kMaxPayloadBytes + 1;
  static constexpr int kEmitSlots = 8;
  EmitQueue<kEmitSlots, static_cast<int>(kEmitTextBytes)> emitQueue_;

  // ---- the old-style cleartext RUN bridge --------------------------
  // RUN:<name>[:<arg>...] (cleartext, e.g. "RUN:pivot:180") goes to
  // runBridge_ below, which sanitizes it, suppresses a host's own
  // retransmits, and either parks it for dispatchJob() to drain in
  // arrival order or -- for "abort"/"clearestop" -- stages it straight
  // back for immediate dispatch. This method is the thin seam between
  // that object and the transports: read the clock, offer the payload,
  // and make the one TypeScript call a bypass asks for.
  void handleRun(const uint8_t* data, size_t dataLen);

  // Parking, dedupe and hand-off for the cleartext RUN bridge, with the
  // run_queue.h ring inside it -- host-portable and host-tested on its
  // own (run_bridge.h). Its overflow count is readable as diagValue
  // ordinal 28. Ordinal 28 is diagValue()'s own numbering, which is a
  // SEPARATE namespace from the config ordinals SET/GET use -- 28 there
  // is `jerk`, and diagValue() has no ordinal 30 at all.
  RunBridge runBridge_;

  // ---- identity, assembled once the fiber actually runs ---------------
  // WireAdapter must stay CODAL-free (host-testable), so this CODAL-
  // facing side calls microbit_friendly_name()/microbit_serial_number()
  // and hands the result to wireAdapter_ via setIdentity() -- from
  // run() (the fiber body), never at construction: neither function is
  // proven safe before uBit.init(). name/drivetrain/profile/version are
  // program-lifetime-stable pointers; serial is formatted once into
  // serialBuf_, a member because WireAdapter borrows a pointer into it.
  static constexpr size_t kSerialBufBytes = 16;  // 10 decimal digits + NUL, with margin
  char serialBuf_[kSerialBufBytes] = {};
  Wire::Identity buildIdentity();

  // Real clock for WireAdapter::now()/its motion-obligation tracking
  // (see wire_adapter.h's own comment on both) -- a plain static member
  // function so it matches WireAdapter::NowMsFn's C-function-pointer
  // type exactly (no captured state possible, so it reaches this
  // Protocol instance through the existing protocol() singleton
  // accessor instead, safe here because it is only ever CALLED from the
  // running fiber, well after that singleton is assigned).
  static uint32_t wireNow();  // [ms]

  // ---- the v6 wire transport seam --------------------------------------
  // All three transports are reached through ONE Sink class
  // (TransportSink, transport_sink.h) rather than one hand-copied Sink
  // apiece. Everything a Sink does here -- decide how much of the
  // written line is content, hand those bytes to the transport, which
  // appends its own single delimiter -- is identical for all three; the
  // only difference is the write call itself, which is what these three
  // one-line adapters supply. The content decision (and, in particular,
  // that the terminator is CHECKED before it is dropped) lives in
  // transport_sink.h, host-portable and host-tested by
  // tests/host/test_transport_sink.py.
  //
  // A dropped line is accepted silently in all three cases: radio
  // refuses while its link is disabled, and WiFi drops while the link
  // is down, no host is known, or its bounded send queue is full. Both
  // self-heal -- a lost telemetry frame through the next frame's seq
  // gap, a lost reply through the host's own retransmit.
  static void writeSerial(SerialTransport& transport, const uint8_t* data,
                          size_t length) {
    transport.writeLine(data, length);
  }
  static void writeRadio(RadioTransport& transport, const uint8_t* data,
                         size_t length) {
    (void)transport.sendLine(data, length);
  }
  static void writeWifi(WifiLink& link, const uint8_t* data, size_t length) {
    (void)link.sendLine(data, length);
  }

  RadioTransport radioTransport_;
  SerialTransport transport_;
  WifiUartCodal wifiUart_;
  CodalFiberLauncher launcher_;
  CodalClock clock_;  // read through clockNow() (above), never directly:
                      // the RUN dedupe's timing, the telemetry cadence,
                      // the WiFi debug period and wireNow() all take
                      // their reading from that one method.
  bool running_ = false;

  // The v6 radio link's own OPT-IN flag lives on RadioTransport
  // (enable()/enabled(), radio_transport.h), not here. This class
  // asks radioTransport_.enabled() where it needs the answer and
  // otherwise leaves the refusal to the transport, which returns false
  // from sendLine()/tryReceiveLine() while disabled -- one object owns
  // "may the radio transmit or receive right now", instead of a bool
  // here that every new call site had to remember to check.
  //
  // The WiFi link is OPT-IN the same way (enableWifi()); wifiBegun_
  // records that serviceWifi() has already handed wifiLink_ its config
  // on this fiber.
  bool wifiEnabled_ = false;
  bool wifiBegun_ = false;
  uint32_t lastWifiDbg_ = 0;  // [ms]

  // NSDMI, not a hand-written constructor: each member depends only on
  // members declared textually above it (transport_/radioTransport_ for
  // the sinks; wireNow() for wireAdapter_; wireAdapter_ + the sinks
  // for wireHandler_/wireHandlerRadio_), so declaration-order in-class
  // initializers are enough. wireAdapter_ starts with a placeholder
  // Wire::Identity(); run() supplies the real one via setIdentity()
  // once it is safe to read (see buildIdentity()'s own comment above).
  //
  // wireHandlerRadio_ shares this SAME wireAdapter_ instance with
  // wireHandler_ -- NOT a second WireAdapter (see this file's own
  // top-of-file comment for why) -- but each keeps its own
  // expectedNext_.
  TransportSink<SerialTransport> serialSink_{transport_, &Protocol::writeSerial};
  TransportSink<RadioTransport> radioSink_{radioTransport_,
                                           &Protocol::writeRadio};
  WireAdapter wireAdapter_{Wire::Identity(), &Protocol::wireNow};
  Wire::WireHandler wireHandler_{wireAdapter_, serialSink_};
  Wire::WireHandler wireHandlerRadio_{wireAdapter_, radioSink_};

  // The WiFi transport and ITS OWN WireHandler over the same shared
  // wireAdapter_ -- a third handler, not a third adapter, for the same
  // reason the radio got a second one (see this file's top comment):
  // each transport keeps its own expectedNext_, so a sequence gap on
  // WiFi can never nack serial's or radio's next command.
  WifiLink wifiLink_{wifiUart_, &Protocol::wireNow};
  TransportSink<WifiLink> wifiSink_{wifiLink_, &Protocol::writeWifi};
  Wire::WireHandler wireHandlerWifi_{wireAdapter_, wifiSink_};
  uint8_t wifiRxBuf_[WifiLink::kMaxLineBytes + 1];
  // Sized for the worst-case `DBG:wifi ...` line: fixed text plus two
  // 15-char addresses, six counters, a 47-char command and a 71-char
  // reply trace (emitLine() clips to the wire cap anyway).
  char wifiDbgBuf_[320];

  // Radio RX scratch -- every line the radio's own poll receives lands
  // here first, whether it turns out to be the old-style cleartext RUN
  // carve-out or a v6 line handed to wireHandlerRadio_ (see run()'s own
  // radio-polling block); reused every poll, serial branch is done with
  // its own buffer by the time this runs each iteration.
  uint8_t rxLineBuf_[64];

  // Serial RX scratch, the serial-side twin of rxLineBuf_ above --
  // moved from a run()-local to a member so serviceOnce() (below) can
  // read into it from ANY nesting depth (run()'s own top-level pass, or
  // a re-entrant call arriving through tickDrive()'s service hook while
  // a job's own tick loop runs) without needing to thread a buffer
  // pointer down through that reentrant call chain. Safe to share: each
  // level fully reads and dispatches whatever landed here before any
  // deeper call could touch it again, and a shallower level never reads
  // it again once it has already handed off to handleRun()/feed().
  uint8_t lineBuf_[kMaxLineBytes];

  // serviceOnce()'s own telemetry-cadence clock, the same reason
  // lineBuf_ above became a member -- shared safely across reentrant
  // calls for the same reason.
  uint32_t lastEmit_ = 0;  // [ms]
};

// Lazy singleton, mirroring shims.cpp's Rig/ensure() pattern:
// constructed and started on first access, never from a global
// constructor (which would run before uBit.init() brings up the CODAL
// fiber scheduler -- see buildIdentity()). Called from `blocks/motion.ts`'s
// top-level `_startProtocol()` statement, so the boot banner
// (Protocol::run()'s own) goes out without any host request.
Protocol& protocol();

}  // namespace diffDrive
