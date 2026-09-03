// protocol.h -- Protocol: the CODAL protocol fiber and byte plumbing
// between SerialTransport/RadioTransport and the v6 wire stack
// (wire_handler.h/.cpp, wire_adapter.h/.cpp). Knows nothing of the v6
// grammar, the reliability layer, or any verb's own behavior -- all of
// that lives behind wireHandler_/wireHandlerRadio_/wireAdapter_.
//
// One exception, preserved deliberately: the OLD cleartext
// "RUN:<name>[:<arg>...]" bridge (handleRun()/dispatchJob()/the
// runQueue_ ring below) coexists with v6 on the same wire -- detected
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

#include "../platform/platform_ports.h"  // CodalFiberLauncher, CodalClock (reused, not reimplemented)
#include "radio_transport.h"  // radio transport -- now a full v6 sink too
#include "serial_transport.h"
#include "wifi_link.h"        // WiFi transport (host-portable AT state machine)
#include "wifi_uart.h"        // ...over NRF_UARTE1 (CODAL-free header)
#include "wire_adapter.h"
#include "run_queue.h"
#include "emit_queue.h"
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
  // invokeRunDispatch()'s own comment for why a nested reentrant
  // dispatch (abort arriving mid-job) can never corrupt an outer job's
  // already-consumed text. Called from the TS layer (shims.cpp's
  // now-zero-argument runCommandText() -- the old
  // MessageBus-event-carries-a-slot-number indirection this used to
  // read through is gone).
  const char* currentRunText() const;

  // Cleartext RUN payloads refused because every slot was still
  // in flight. Saturates rather than wrapping -- a drop count
  // that rolls to zero reads as "nothing was lost".
  uint32_t runDropCount() const;

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
  // Channel and group are applied BEFORE radioEnabled_ flips, so the
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
  // fiber. motionOwner_ arbitrates which caller currently holds the
  // drivetrain -- kNone (idle), kWire (a live wire motion obligation,
  // set/cleared by run()'s own loop around its tickDrive() call), or
  // kJob (a dispatched RUN job, set/cleared by dispatchJob() around its
  // call into the TS handler). Lives HERE, not on WireAdapter or the RUN
  // queue, because this class is the only one that can see both a wire
  // request and a dispatched job.
  enum class MotionOwner : uint8_t { kNone, kWire, kJob };
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

  // Copies `text` into currentRunText_ and invokes the one registered
  // RUN dispatch callback (shims.cpp's runDispatch(), which runs
  // whichever `onRun()`/`onRunCommand()` handler test.ts bound to this
  // command's name) -- the single path both dispatchJob() (a queued job,
  // gated on motionOwner_) and handleRun()'s abort/clearestop bypass
  // (ungated, see that method's own comment) funnel through. Safe to
  // call reentrantly (an abort dispatched from inside a running job's own
  // tick loop): the callback reads currentRunText_ back via
  // runCommandText() at its OWN entry, before doing anything else, so a
  // nested call's overwrite can never corrupt an outer, still-running
  // job's own already-consumed text -- every onRun() handler in this
  // package reads its arguments only at entry (test/test.ts), never
  // later during a long-running tick loop.
  void invokeRunDispatch(const char* text);

  // Copies `text` into currentRunText_, bounded to kRunTextBytes and
  // always NUL-terminated. The one place that buffer is written.
  void setCurrentRunText(const char* text);

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

  // tickDrive()'s (shims.cpp) service hook, registered once via
  // registerTickServiceHook() when run() starts -- a plain
  // no-capture function pointer (same reason wireNowMs() below is a
  // plain static member function, not a lambda: a bare C function
  // pointer cannot capture `this`), so it reaches this specific
  // Protocol instance through the protocol() singleton accessor, safe
  // for the same reason wireNowMs() is. Deliberately a NO-OP unless
  // motionOwner_ == kJob: tickDrive() is also called (a) from run()'s
  // own loop for a live WIRE motion obligation, which already gets its
  // own servicing once per pass via run()'s own loop calling
  // serviceOnce() directly, and (b) from a student's own program driving
  // continuous-mode motion on THAT program's own fiber (the third,
  // deliberately unchanged execution model) -- neither call site needs
  // or wants this hook's extra work, and (b)
  // must never run serviceOnce() on a fiber other than this one.
  static void serviceHookEntry();

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
  // reaches the wire within one poll interval (kPollIntervalMs).
  void drainEmitQueue();

  // emitQueue_'s slot text bytes: RadioTransport::kMaxPayloadBytes (the
  // cap emitLine() already clips to) plus one for the NUL this ring
  // adds itself -- a clipped line always fits. Slot count matches
  // runQueue_'s own: generous enough for a burst of result lines
  // between drain passes without becoming a large static allocation.
  static constexpr size_t kEmitTextBytes = RadioTransport::kMaxPayloadBytes + 1;
  static constexpr int kEmitSlots = 8;
  EmitQueue<kEmitSlots, static_cast<int>(kEmitTextBytes)> emitQueue_;

  // ---- the old-style cleartext RUN bridge, preserved unchanged from
  // before the v5 retirement (see this file's own top-of-file comment)
  // except for HOW a dequeued command reaches TypeScript: not a
  // MessageBus event to a second, forked fiber (deleted) but
  // dispatchJob() dequeuing and calling invokeRunDispatch() directly, on
  // THIS fiber. -----------------------
  // RUN:<name>[:<arg>...] (cleartext, e.g. "RUN:pivot:180") normally
  // parks the payload text in runQueue_ below for dispatchJob() to drain
  // in arrival order. "abort"/"clearestop" bypass that queue entirely --
  // see this method's own definition (protocol.cpp) for why: a queued
  // abort would sit behind the very job it is meant to stop.
  void handleRun(const uint8_t* data, size_t dataLen);

  // RUN payload storage: a real ring with occupancy (run_queue.h), not a
  // bare write cursor -- a slot stays in flight from enqueue() until
  // dispatchJob() reads and releases it, so a burst arriving during a
  // long job's dispatch can no longer overwrite payload not yet
  // consumed. Overflow is counted and readable (diagValue ordinal 30)
  // instead of silent.
  static constexpr size_t kRunTextBytes = 48;  // name + args + NUL
  static constexpr int kRunSlots = 8;
  RunQueue<kRunSlots, static_cast<int>(kRunTextBytes)> runQueue_;

  // The text of whichever RUN command dispatchJob()/invokeRunDispatch()
  // most recently copied in, for currentRunText() (public, above) to
  // return. The one buffer both call sites write, always through
  // setCurrentRunText().
  char currentRunText_[kRunTextBytes] = {};

  // RUN repeat suppression -- see handleRun's own comment. Hosts repeat
  // commands to survive the single-slot inbound buffer, and without
  // this a repeated RUN runs the test once per copy. Compared on the
  // whole payload, so RUN:pivot:180 does not suppress RUN:pivot:-180.
  // Suppress a host's own RETRANSMITS, not deliberate repeats. The
  // queue below fixes loss; this fixes duplicate EXECUTION, which is a
  // different failure -- a host repeating a command over a lossy radio
  // would otherwise run the tour once per copy. 3000 ms was far wider
  // than any retransmit burst and made sending one command twice in a
  // row impossible, which is exactly the shape a parameter sweep
  // sends. 400 ms still swallows a burst and gives deliberate repeats
  // back.
  static constexpr int32_t kRunDedupeMs = 400;
  char lastRunText_[kRunTextBytes] = {};
  uint32_t lastRunMs_ = 0;   // [ms] arrival time of the last accepted RUN

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
  static uint32_t wireNowMs();

  // ---- ticket 005: the v6 wire transport seam --------------------------
  // The one Sink WireHandler writes every reply line through (Sink's own
  // contract, wire_handler.h). `data`/`length` always include a
  // trailing '\n' -- WireHandler::writeLine() supplies it on every
  // call -- so this strips that one byte before handing off to
  // SerialTransport::writeLine(), which appends its OWN trailing
  // delimiter; passing both through would double the newline.
  class SerialSink : public Wire::Sink {
   public:
    explicit SerialSink(SerialTransport& transport) : transport_(transport) {}
    void write(const char* data, size_t length) override {
      const size_t contentLen = length > 0 ? length - 1 : 0;
      transport_.writeLine(reinterpret_cast<const uint8_t*>(data),
                           contentLen);
    }

   private:
    SerialTransport& transport_;
  };

  // The Sink wireHandlerRadio_ writes every v6 reply line through --
  // mirrors SerialSink exactly, including WHY the trailing '\n' is
  // stripped here: RadioTransport::sendLine() appends its own trailing
  // delimiter, same convention SerialTransport::writeLine() follows,
  // so passing both through would double it.
  class RadioSink : public Wire::Sink {
   public:
    explicit RadioSink(RadioTransport& transport) : transport_(transport) {}
    void write(const char* data, size_t length) override {
      const size_t contentLen = length > 0 ? length - 1 : 0;
      // sendLine()'s bool return (ticket 002's re-entrancy guard) is
      // deliberately ignored here, not an oversight: a telemetry/ack
      // line dropped under contention with Protocol::emitLine() (the
      // TS fiber's own sendLine() caller) self-heals for free via the
      // next frame's seq gap, and retrying here would just reintroduce
      // the contention the guard exists to avoid (see sprint.md's
      // Design Rationale -- do not "fix" this into matching
      // emitLine()'s retry).
      (void)transport_.sendLine(reinterpret_cast<const uint8_t*>(data),
                                contentLen);
    }

   private:
    RadioTransport& transport_;
  };

  // The Sink wireHandlerWifi_ writes every v6 reply line through --
  // mirrors RadioSink, including the trailing-'\n' strip (WifiLink::
  // sendLine() frames the datagram with its own '\n'). A line dropped
  // because the link is down, no host is known, or the bounded send
  // queue is full is dropped silently here, by the same reasoning
  // RadioSink gives: it self-heals through the host's own retransmit.
  class WifiSink : public Wire::Sink {
   public:
    explicit WifiSink(WifiLink& link) : link_(link) {}
    void write(const char* data, size_t length) override {
      const size_t contentLen = length > 0 ? length - 1 : 0;
      (void)link_.sendLine(reinterpret_cast<const uint8_t*>(data), contentLen);
    }

   private:
    WifiLink& link_;
  };

  RadioTransport radioTransport_;
  SerialTransport transport_;
  WifiUartCodal wifiUart_;
  CodalFiberLauncher launcher_;
  CodalClock clock_;  // PING's t=<ms> equivalent is now wireNowMs(); this
                      // instance now backs only handleRun()'s own dedupe
                      // timing and wireNowMs() itself (via protocol()).
  bool running_ = false;

  // The v6 radio link is OPT-IN: false until setupRadio() flips it.
  //
  // This is what lets a student's program use MakeCode's own `radio.*`
  // blocks (a joystick controller, say). RadioTransport frames raw
  // RadioRelay fragments with NO PXT radio packet header, on a fixed
  // band -- see radio_transport.h's top comment -- so the two cannot
  // share the air. Whichever one comes up first owns the radio.
  //
  // Every path that would reach RadioTransport must be gated on this,
  // not just the RX poll: RadioTransport lazily calls ensureRadioReady()
  // from BOTH tryReceiveLine() and sendLine(), so an ungated emitLine()
  // or telemetry emission would claim the radio just as surely as the
  // poll does. run()'s radio poll, emitLine()'s radio write, and run()'s
  // wireHandlerRadio_.emitTelemetry() are the three sites.
  bool radioEnabled_ = false;

  // The WiFi link is OPT-IN the same way (enableWifi()); wifiBegun_
  // records that serviceWifi() has already handed wifiLink_ its config
  // on this fiber.
  bool wifiEnabled_ = false;
  bool wifiBegun_ = false;
  uint32_t lastWifiDbgMs_ = 0;

  // NSDMI, not a hand-written constructor: each member depends only on
  // members declared textually above it (transport_/radioTransport_ for
  // the sinks; wireNowMs() for wireAdapter_; wireAdapter_ + the sinks
  // for wireHandler_/wireHandlerRadio_), so declaration-order in-class
  // initializers are enough. wireAdapter_ starts with a placeholder
  // Wire::Identity(); run() supplies the real one via setIdentity()
  // once it is safe to read (see buildIdentity()'s own comment above).
  //
  // wireHandlerRadio_ shares this SAME wireAdapter_ instance with
  // wireHandler_ -- NOT a second WireAdapter (see this file's own
  // top-of-file comment for why) -- but each keeps its own
  // expectedNext_.
  SerialSink serialSink_{transport_};
  RadioSink radioSink_{radioTransport_};
  WireAdapter wireAdapter_{Wire::Identity(), &Protocol::wireNowMs};
  Wire::WireHandler wireHandler_{wireAdapter_, serialSink_};
  Wire::WireHandler wireHandlerRadio_{wireAdapter_, radioSink_};

  // The WiFi transport and ITS OWN WireHandler over the same shared
  // wireAdapter_ -- a third handler, not a third adapter, for the same
  // reason the radio got a second one (see this file's top comment):
  // each transport keeps its own expectedNext_, so a sequence gap on
  // WiFi can never nack serial's or radio's next command.
  WifiLink wifiLink_{wifiUart_, &Protocol::wireNowMs};
  WifiSink wifiSink_{wifiLink_};
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
  uint32_t lastEmitMs_ = 0;
};

// Lazy singleton, mirroring shims.cpp's Rig/ensure() pattern:
// constructed and started on first access, never from a global
// constructor (which would run before uBit.init() brings up the CODAL
// fiber scheduler -- see buildIdentity()). Called from `blocks/motion.ts`'s
// top-level `_startProtocol()` statement, so the boot banner
// (Protocol::run()'s own) goes out without any host request.
Protocol& protocol();

}  // namespace diffDrive
