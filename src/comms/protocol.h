// protocol.h -- Protocol: the CODAL protocol fiber and byte plumbing
// between SerialTransport/RadioTransport/WifiLink and the v6 wire stack
// (wire_handler.h/.cpp, wire_adapter.h/.cpp). Knows nothing of the v6
// grammar, the reliability layer, or any verb's own behavior -- all of
// that lives behind wireHandler_/wireHandlerRadio_/wireHandlerWifi_/
// wireAdapter_.
//
// There is NO carve-out: every inbound line goes to the v6 stack.
// `RUN <name> [arg...] #<id>` reaches the by-name dispatch test.ts uses
// via WireAdapter::onRun() -> protocolOfferRun() -> handleRun() below,
// with the reliability layer. routeLine()'s cleartext "RUN:" prefix
// match, and the 400 ms window standing in for it, went 2026-09-07.
//
// Each transport gets its OWN WireHandler over the SAME wireAdapter_ --
// never a second adapter: two adapters would let a sequence gap on one
// transport nack the OTHER transport's next command, which is exactly
// the corruption this structure prevents. Each handler keeps its own
// expectedNext_.
//
// emitLine() below queues onto a Protocol-owned ring rather than
// writing a transport directly, and this fiber's loop drains it every
// pass -- so an untethered bench run's result lines still reach a
// listening host, and only this fiber ever writes a wire.
//
// Telemetry is the v6 thdr/t frame stream
// (WireHandler::emitTelemetry(Snapshot), see its own doc comment for
// the format); the bench scripts read it directly and no cleartext
// "TLM:" parsing remains anywhere in this tree.
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

  // Queue one caller-supplied text line for emission on BOTH transports.
  // Callable from ANY fiber (shims.cpp's emitLine is the TS-facing
  // caller): it clips to RadioTransport::kMaxPayloadBytes, copies into
  // emitQueue_ and returns, touching no transport. Only this object's
  // own fiber (run(), via drainEmitQueue()) ever writes a transport for
  // this path, so two fibers can never race one underlying serial write.
  //
  // The tradeoff: a caller cannot assume the line is physically on the
  // wire when this returns, only that it is queued for the next drain
  // (at most one poll interval later); a full ring drops the NEWEST
  // line, counted (emitDropCount()) rather than silent.
  //
  // It exists because TypeScript's `serial.writeLine` reaches the USB
  // cable only, and the cable only reaches the bench stand: anything
  // that has to actually move runs untethered, so its results have to
  // come back over the radio.
  void emitLine(const char* text);

  // The text of whichever RUN command is CURRENTLY being dispatched --
  // name and arguments colon-joined ("pivot:180"), as
  // WireAdapter::onRun() rebuilt it from the verb's own space-separated
  // tokens. Valid only while
  // the registered RUN dispatch callback is executing, on THIS fiber;
  // see RunBridge::currentText() for why a nested reentrant dispatch
  // (an abort arriving mid-job) cannot corrupt an outer job's
  // already-consumed text. Read from TS via shims.cpp's
  // runCommandText().
  const char* currentRunText() const;

  // The take/release pair every MOTION entry point in shims.cpp
  // (startMove()/driveTwist()/engineGoToRArmed()) brackets its own call
  // span with: take up front, release once tickDrive()/the starvation
  // watchdog/an explicit stop path (endMove()/stopAll()/estopAll())
  // next finds the drivetrain idle.
  //
  // Two rules, one pure decision core (core/motion_owner.h, host-tested
  // there directly). A call arriving on Protocol's OWN fiber while
  // motionOwner_ is already kJob is the dispatched job's own move and
  // passes through unchanged -- dispatchJob() sets kJob and then calls
  // the TS handler SYNCHRONOUSLY on this fiber, so without that branch
  // the handler's own startMove() is indistinguishable from a
  // competitor arriving mid-job and is refused silently. Any other
  // fiber (a student script, a button handler) gets the UNCHANGED
  // kBlock rule: refused, never silently superseding, whenever
  // motionOwner_ is kWire, kJob or an already-taken kBlock. A release
  // is a no-op unless currently kBlock -- the job-own-fiber branch
  // never sets kBlock, and dispatchJob() clears kJob itself.
  //
  // Public (unlike motionOwner_) so the free-function seam beside these
  // definitions (protocol.cpp) can reach them from shims.cpp, the same
  // shape currentRunText() above uses.
  bool tryTakeMotionOwnership();

  void releaseBlockOwnership();

  // ---- the RUN bridge ----------------------------------------------
  // A decoded `RUN <name> [arg...] #<id>` arrives as the colon-joined
  // text WireAdapter::onRun() built, via protocolOfferRun(). runBridge_
  // parks it for dispatchJob(), or -- for "abort"/"clearestop" --
  // stages it back for immediate dispatch. Public for
  // tryTakeMotionOwnership()'s reason: a free function outside this
  // class calls it. False means the bridge REFUSED the payload
  // (malformed, or every slot in flight), which onRun() makes an `err`
  // rather than an ack for a command that never runs.
  bool handleRun(const uint8_t* data, size_t dataLen);

  // RUN payloads refused because every slot was still
  // in flight. Saturates rather than wrapping -- a drop count
  // that rolls to zero reads as "nothing was lost".
  uint32_t runDropCount() const;

  // RUN payloads refused by RunBridge's own sanitizer --
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
  // could not keep.
  uint32_t radioRxFrameCount() const;
  uint32_t radioRxAcceptedCount() const;
  uint32_t radioRxOverrunDropCount() const;
  uint32_t radioRxOversizeDropCount() const;

  // emitLine() calls refused because emitQueue_ was already full,
  // surfaced for shims.cpp's diagValue(29)/probe(29). Same saturating
  // convention as runDropCount() above: should stay 0 across a normal
  // session.
  uint32_t emitDropCount() const;

  // SerialTransport::writeLine()'s drop counter, surfaced for
  // shims.cpp's diagValue(26)/probe(26). Same free-function boundary as
  // emitLine()/currentRunText() above: shims.cpp reaches this through
  // protocolSerialDropCount() (protocol.cpp) rather than including this
  // header, so it never pulls radio_transport.h into its own include
  // graph.
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
  // configured -- i.e. the per-robot pair tools/make_deploy.py injected
  // into kChannel/kGroup at deploy time (radio_transport.h).
  //
  // This is what the on-robot test program (test/test.ts) calls. It
  // deliberately does NOT take a channel: hardcoding one there would
  // override the deploy injection and put every robot's build on one
  // board's channel. Students get setupRadio() instead, where naming
  // the channel is the point.
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
  // polls its inbound lines into wireHandlerWifi_ (through the same
  // carve-out-free routeLine() serial and radio use), and emits one
  // `DBG:wifi ...` diagnostic line per state change.
  void serviceWifi();
  void emitWifiDebug();

  // ---- single executor -- motionOwner_ arbitration -------------------
  // Which caller currently holds the drivetrain (MotionOwner,
  // core/motion_owner.h): kNone (idle), kWire (a live wire motion
  // obligation, set/cleared by run()'s loop around its tickDrive()
  // call), kJob (a dispatched RUN job, set/cleared by dispatchJob()
  // around its call into the TS handler), or kBlock (a student's own
  // move()/driveTwist()/startDrive() on some other fiber, taken and
  // released through tryTakeMotionOwnership()/releaseBlockOwnership()
  // above).
  //
  // Lives HERE, not on WireAdapter or the RUN queue, because this class
  // is the only one that can see a wire request, a dispatched job AND a
  // block-motion call. wireAdapter_.setExternalOwner() (wire_adapter.h)
  // is the one seam that mirrors the value into that host-portable
  // class, so the two cannot drift as separately-maintained fields.
  MotionOwner motionOwner_ = MotionOwner::kNone;

  // Dequeues and dispatches ONE queued RUN job, if one is waiting and
  // motionOwner_ is kNone. Sets motionOwner_ = kJob and tells
  // wireAdapter_, so a wire motion verb arriving mid-job is refused
  // (kBusy) rather than silently racing its move; both are cleared once
  // the dispatched call returns.
  //
  // The dispatched call can run for a whole tour, during which THIS
  // SAME fiber re-enters serviceOnce() through tickDrive()'s service
  // hook (serviceHookEntry(), below), which calls this again on every
  // re-entry. Those nested calls find motionOwner_ == kJob and return
  // immediately: a job is dispatched exactly once per queued command,
  // never re-entered.
  void dispatchJob();

  // One pass of this fiber's OWN servicing: drains emitQueue_,
  // dispatches one queued RUN job if the drivetrain is free, polls each
  // transport for new lines, and emits a telemetry frame if one is due.
  // It ALSO runs as tickDrive()'s service hook (serviceHookEntry(),
  // below), so a dispatched job's own `while (driveTick())` loop nests
  // back into this servicing once per tick -- which is what lets an
  // abort, a new queued command or ordinary telemetry keep flowing
  // without waiting for that job to return, without a second fiber.
  //
  // Never calls tickDrive() itself and never sleeps: run()'s loop does
  // both, once per pass, strictly AFTER this returns. The one nested
  // call site is already mid-tick when this fires, so doing either here
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
  // Not a per-transport knob: a transport needing its own budget is a
  // reason to name a second constant here, not to spell a bare number
  // at its call site.
  static constexpr int kRxDrainPerPass = 4;

  // The ONE path an inbound line takes, whichever transport produced
  // it: `data`/`len` is one complete line, delimiter already stripped
  // by the transport that framed it. Every line is fed to `handler`,
  // followed by the separate "\n" feed() needs to see it as complete --
  // no prefix fork here any more (see this file's own top comment).
  //
  // `handler` is the CALLER's own WireHandler, never a fixed one: each
  // transport has its own, each with its own expectedNext_, so a
  // sequence gap on one can never nack another's next command. That is
  // the only thing that differs between the three poll branches.
  void routeLine(Wire::WireHandler& handler, const uint8_t* data, size_t len);

  // This fiber's own clock reading -- the ONE place clock_'s microsecond
  // counter is reduced to the millisecond scale everything above it
  // (the telemetry cadence, the WiFi debug period, RunBridge's dedupe
  // window) actually works in. Was four separate `nowMicros() / 1000`
  // conversions, one per caller.
  uint32_t clockNow();  // [ms]

  // tickDrive()'s (shims.cpp) service hook, registered once by run() --
  // a plain no-capture function pointer (a bare C function pointer
  // cannot capture `this`, the same reason wireNow() below is a static
  // member function), so it reaches this instance through the
  // protocol() singleton accessor.
  //
  // Gates on WHICH FIBER is calling, via
  // diffDrive::shouldServiceHookRun() (core/fiber_identity.h), NOT on
  // motionOwner_: a button-handler fiber calling tickDrive() while a
  // job ran satisfied `motionOwner_ == kJob` and ran serviceOnce() a
  // SECOND time, concurrently, corrupting the wire dispatcher's shared
  // line buffer mid-yield. Comparing fiber identity makes "no fiber but
  // this object's own ever runs serviceOnce()" true by construction,
  // whatever any state variable says.
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
  // Copies `len` bytes from `text` to serial, then (if the radio link
  // is up) mirrors the same bytes to radio with one retry -- see the
  // definition (protocol.cpp) for the retry's own reasoning. Its ONLY
  // caller is drainEmitQueue() below, itself called once per pass of
  // run()'s loop on this object's own fiber, which is what makes this
  // fiber the only writer of either transport for the emit path
  // whatever fiber called emitLine(). `text` therefore always points at
  // a local buffer that outlives any yield this performs.
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

  // Parking and hand-off for the RUN bridge, with the
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
  // (TransportSink, transport_sink.h); the only thing that differs
  // between them is the write call itself, which is what these three
  // one-line adapters supply.
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

  // NSDMI, not a hand-written constructor: every member below depends
  // only on members declared textually above it, so declaration-order
  // in-class initializers suffice. wireAdapter_ starts with a
  // placeholder Wire::Identity(); run() supplies the real one via
  // setIdentity() once it is safe to read (buildIdentity(), above).
  // wireHandlerRadio_ shares this SAME wireAdapter_ -- never a second
  // one, see this file's top comment -- but keeps its own expectedNext_.
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

  // Radio RX scratch -- every line the radio poll receives lands here
  // first, cleartext RUN carve-out or v6 line alike; reused every poll,
  // and the serial branch is done with its own buffer by then.
  uint8_t rxLineBuf_[64];

  // Serial RX scratch, the serial-side twin of rxLineBuf_ above. A
  // member, not a run() local, so serviceOnce() can read into it from
  // ANY nesting depth without threading a pointer down the reentrant
  // call chain. Safe to share: each level fully reads and dispatches
  // whatever landed here before any deeper call can touch it, and never
  // reads it again once it has handed off to handleRun()/feed().
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
