// protocol.h -- Protocol: the CODAL protocol fiber and byte plumbing
// between SerialTransport/RadioTransport and the v6 wire stack
// (wire_handler.h/.cpp, wire_adapter.h/.cpp). Knows nothing of the v6
// grammar, the reliability layer, or any verb's own behavior -- all of
// that lives behind wireHandler_/wireHandlerRadio_/wireAdapter_.
//
// One exception, preserved deliberately: the OLD cleartext
// "RUN:<name>[:<arg>...]" MessageBus bridge (handleRun()/runText()/the
// runSlots_ ring below) coexists with v6 on the same wire -- detected
// directly by its literal "RUN:" prefix before a line ever reaches the
// v6 stack (no verb registry involved -- see run()'s own comment). It
// is the ONLY path that feeds the MessageBus test-trigger bridge
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
// reporting already uses -- still writes to both transports unchanged,
// so an untethered bench run's results still reach a listening host.
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
#include "wire_adapter.h"
#include "wire_handler.h"

namespace diffDrive {

// ---- Protocol loop -----------------------------------------------------
class Protocol {
 public:
  // Starts the protocol loop on its own CODAL fiber via
  // CodalFiberLauncher. Idempotent, mirroring
  // DifferentialDrive::start()'s own idempotent guard.
  void start();

  // Emit one caller-supplied text line on BOTH transports -- this
  // consolidates what used to be two separate single-transport emitters
  // into the one path anything wanting both wires mirrored now uses.
  // Exists because the test programs' result lines (tour fixes,
  // calibration data, timings) were written with TypeScript's
  // `serial.writeLine`, which reaches the USB cable only -- and the USB
  // cable only reaches the bench stand, where the wheels are off the
  // ground. Every test that needs the robot to actually move therefore
  // runs untethered, and its results have to come back over the radio.
  //
  // Called from the TS layer (shims.cpp's emitLine), NOT from this
  // object's own fiber; SerialTransport::writeLine blocks the caller
  // until the bytes are out, and RadioTransport::sendLine is a single
  // datagram, so a caller between moves pays a bounded cost -- plus,
  // as of ticket 002, at most one extra fiber_sleep(2) if
  // sendLine()'s re-entrancy guard fires against the protocol fiber's
  // own concurrent RadioSink::write() and this call retries once.
  void emitLine(const char* text);

  // Text of the RUN command that raised MessageBus event value `slot`
  // (1..kRunSlots) -- the whole payload after `RUN:`, e.g.
  // "pivot:180". Returns "" for an out-of-range or never-written slot.
  // Called from the TS layer (shims.cpp's runCommandText), on the event
  // handler's fiber rather than this object's own.
  const char* runText(int slot) const;

  // SerialTransport::writeLine()'s drop counter (ticket 006), surfaced
  // for shims.cpp's diagValue(26)/probe(26). Same same-package
  // forward-declaration boundary as emitLine()/runText() above:
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

 private:
  static void fiberEntry(void* self);
  void run();

  // ---- the old-style cleartext RUN MessageBus bridge, preserved
  // unchanged from before this cutover (see this file's own top-of-file
  // comment for why it survives the v5 retirement) -----------------------
  // RUN:<name>[:<arg>...] (cleartext, e.g. "RUN:pivot:180") parks the
  // payload text in a slot and raises a MessageBus event carrying that
  // slot as the event value. `blocks/run.ts`'s run dispatcher registers a
  // TS handler against the same source id, reads the text back through the
  // runCommandText shim, and calls whichever handler test.ts bound to
  // that NAME -- so a wire command reads as the test it runs, not as a
  // magic number, and its arguments ride along as text instead of being
  // encoded into numeric offsets. The event fires handlers on their own
  // fiber (MessageBus default), so a long-running test (a full square
  // tour ticking the kernel) does not block this protocol fiber.
  void handleRun(const uint8_t* data, size_t dataLen);

  // RUN payload storage. The event value is a uint16 and cannot carry
  // text, so the payload is parked here and the event carries only the
  // slot it landed in (1-based; 0 is MICROBIT_EVT_ANY). A RING of slots,
  // not one buffer: MessageBus events queue and a test handler can run
  // for a minute, so a second RUN arriving mid-test would otherwise
  // overwrite the text the queued handler has not read yet. Four slots
  // covers any burst a host can plausibly send inside one handler.
  static constexpr size_t kRunTextBytes = 48;  // name + args + NUL
  static constexpr int kRunSlots = 4;
  char runSlots_[kRunSlots][kRunTextBytes] = {};
  int nextRunSlot_ = 0;      // round-robin write cursor, 0-based

  // RUN repeat suppression -- see handleRun's own comment. Hosts repeat
  // commands to survive the single-slot inbound buffer, and without
  // this a repeated RUN runs the test once per copy. Compared on the
  // whole payload, so RUN:pivot:180 does not suppress RUN:pivot:-180.
  static constexpr int32_t kRunDedupeMs = 3000;
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

  RadioTransport radioTransport_;
  SerialTransport transport_;
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

  // Radio RX scratch -- every line the radio's own poll receives lands
  // here first, whether it turns out to be the old-style cleartext RUN
  // carve-out or a v6 line handed to wireHandlerRadio_ (see run()'s own
  // radio-polling block); reused every poll, serial branch is done with
  // its own buffer by the time this runs each iteration.
  uint8_t rxLineBuf_[64];
};

// Lazy singleton, mirroring shims.cpp's Rig/ensure() pattern:
// constructed and started on first access, never from a global
// constructor (which would run before uBit.init() brings up the CODAL
// fiber scheduler -- see buildIdentity()). Called from `blocks/motion.ts`'s
// top-level `_startProtocol()` statement, so the boot banner
// (Protocol::run()'s own) goes out without any host request.
Protocol& protocol();

}  // namespace diffDrive
