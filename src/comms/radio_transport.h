// radio_transport.h -- RadioTransport: gets a formatted wire line onto
// the micro:bit radio, framed for the fleet's RADIOBRIDGE relay. Thin
// CODAL-facing leaf beneath Protocol (mirrors SerialTransport's role
// for uBit.serial -- see serial_transport.h's top comment): knows
// uBit.radio and the RadioRelay on-air fragment framing, nothing about
// pose, verb names, or command semantics.
//
// On-air framing is a port of radio-robot's
// Platform::MicroBitRadioLink (RadioRelay wire spec section 5 -- the
// fleet's own radio driver the RADIOBRIDGE relay hardware is built
// against; src/DESIGN.md §2 has the authoritative upstream repo/path
// statement): every packet is a fragment
//     [SEQ:1][FLAGS:1][LEN:1][payload:LEN]
// carried as the raw CODAL datagram payload (no MakeCode/PXT radio
// package header). FLAGS: START=0x01, MORE=0x02, END=0x04. A message
// longer than one fragment's payload capacity is split START..END; a
// single-fragment message is flagged START|END.
//
// TX (sendLine(), below) and RX (tryReceiveLine()/onDatagram(), below)
// both use this framing. RX accepts only single-fragment messages (see
// onDatagram()'s own doc for the current capacity limit and
// radioRxLineFits(), just below, for the accept/reject predicate
// itself); no ACK protocol either direction -- FLAG_ACK (0x10) is never
// set or interpreted.
#pragma once

#include <cstddef>
#include <cstdint>

namespace diffDrive {

// True iff an inbound fragment declaring `declaredLen` payload bytes
// (after onDatagram() has stripped the trailing 0x0A) fits WHOLE into a
// `bufferCapacity`-byte receive buffer. False means drop the frame
// ENTIRE: never truncate it to a shorter, still-parseable prefix.
// WireHandler::feed() cannot tell a truncated line from a genuinely
// short one the host sent, so a truncated over-length command decodes
// and EXECUTES as a different, legal, shorter command -- a dropped line
// is merely invisible, a truncated-and-accepted one is dangerous.
//
// CODAL-free (this header includes only <cstddef>/<cstdint>) so a host
// test can #include it directly; radio_transport.cpp needs pxt.h and
// cannot be host-compiled at all. See
// tests/host/test_radio_transport_rx_capacity.py.
inline bool radioRxLineFits(size_t declaredLen, size_t bufferCapacity) {
  return declaredLen <= bufferCapacity;
}

// What the RX path did with one complete single-fragment line.
enum class RadioRxDisposition : uint8_t {
  kAccept,    // copy it into the RX slot and raise the ready flag
  kOversize,  // longer than the RX buffer: dropped whole, never truncated
  kOverrun,   // the previous line is still unconsumed: dropped
};

// Everything the radio RX path knows about what it heard. Saturating,
// like every other diagnostic counter in this package: a count that
// wrapped to zero would read as "nothing happened". The two drop
// reasons are counted separately because they call for different fixes
// -- an oversize drop means a line that cannot fit at all, an overrun
// drop means the drain is not keeping up.
//
// The default member initializers make this a non-aggregate under
// C++11 (the target's standard) though not under the host suite's
// C++20 -- the same trap Wire::Column already documents. Nothing
// brace-initializes one with values, and nothing should: default-
// construct it and let radioRxClassify() do the writing.
struct RadioRxCounters {
  uint32_t frames = 0;
  uint32_t accepted = 0;
  uint32_t oversizeDropped = 0;
  uint32_t overrunDropped = 0;
};

// Classify one complete single-fragment inbound line and record it.
// `declaredLen` is the line length after the trailing delimiter has
// been stripped; `slotBusy` is whether the previous line is still
// waiting to be consumed.
//
// Pure decision plus counter bookkeeping, deliberately OUTSIDE
// RadioTransport: onDatagram() needs pxt.h's uBit.radio/PacketBuffer,
// while the part worth testing -- which disposition each case gets and
// which counter moves -- has no CODAL in it. Same reason
// radioRxLineFits() above lives here.
inline RadioRxDisposition radioRxClassify(size_t declaredLen,
                                          size_t bufferCapacity, bool slotBusy,
                                          RadioRxCounters& counters) {
  // Counted first, and unconditionally: `frames` is "what arrived",
  // which is only useful as the denominator the three outcomes below
  // are read against.
  if (counters.frames != UINT32_MAX) ++counters.frames;
  if (!radioRxLineFits(declaredLen, bufferCapacity)) {
    if (counters.oversizeDropped != UINT32_MAX) ++counters.oversizeDropped;
    return RadioRxDisposition::kOversize;
  }
  // Capacity is checked BEFORE occupancy so an over-length line is
  // reported as over-length even when it also happened to arrive on a
  // busy slot -- the two are not equally fixable, and the length is the
  // property of the line itself.
  if (slotBusy) {
    if (counters.overrunDropped != UINT32_MAX) ++counters.overrunDropped;
    return RadioRxDisposition::kOverrun;
  }
  if (counters.accepted != UINT32_MAX) ++counters.accepted;
  return RadioRxDisposition::kAccept;
}

class RadioTransport {
 public:
  // The v6 radio link is OPT-IN, and this class owns that decision --
  // sendLine() and tryReceiveLine() below both refuse (return false,
  // touching no hardware) until enable() has been called. That is what
  // leaves the air to a student program's own MakeCode `radio.*`
  // blocks: this class frames raw RadioRelay fragments with NO PXT
  // radio packet header, on a fixed band (see this file's top comment),
  // so the two cannot share the air -- whichever comes up first owns
  // the radio.
  //
  // The gate lives on this ONE object rather than as a bool on Protocol
  // checked per call site, because every path into this class must be
  // gated, not just the RX poll: sendLine() and tryReceiveLine() both
  // lazily call ensureRadioReady(), so an ungated emit claims the radio
  // just as surely as a poll does.
  //
  // enable() does NOT bring the radio up -- the lazy first-use bring-up
  // is unchanged, so a program that enables the link and never sends or
  // polls still never pays uBit.radio.enable()'s RAM/softdevice cost.
  // Idempotent; there is deliberately no disable().
  void enable() { enabled_ = true; }
  bool enabled() const { return enabled_; }

  // Fragments `data` (len bytes) into RadioRelay-framed radio packets
  // and transmits each one via uBit.radio.datagram.send(), appending a
  // trailing 0x0A ('\n') as the final payload byte -- the same
  // one-terminator-per-line convention SerialTransport::writeLine()
  // uses. TRUNCATES rather than overflows a `len` beyond this module's
  // internal line-buffer capacity, mirroring SerialTransport's own
  // defensive truncation.
  //
  // Lazily brings the radio up and configures it (uBit.radio.enable(),
  // then channel_/group_/kTransmitPower, matching the reference
  // driver's own begin()) on the FIRST call PAST the enable gate above
  // -- never at construction, so a bench-only serial user who never
  // calls sendLine() never pays uBit.radio.enable()'s RAM/softdevice
  // cost.
  //
  // Returns false, having touched nothing, while the link is disabled;
  // true once the fragments are on air. That is the ONLY thing the
  // return value means -- a failed send is not retried here. Single
  // writer: the protocol fiber. Every caller -- a v6 reply, a telemetry
  // frame, a queued emitLine() -- arrives from Protocol::serviceOnce()
  // on that one fiber, so the shared payloadBuf_/frameBuf_ scratch
  // below cannot be interleaved and no caller needs to retry.
  bool sendLine(const uint8_t* data, size_t len);

  // RX (radio command plane, single-fragment only): returns false,
  // having touched nothing, while the link is disabled -- this is the
  // call that would otherwise bring the radio up out from under
  // MakeCode's own radio blocks. Once enabled it polls one queued
  // datagram, accepts frames whose flags carry START|END together (a
  // complete message in one fragment -- with the 250-byte fleet packet
  // size every relay-forwarded command line qualifies), strips the
  // trailing 0x0A, and copies the line into outBuf. Returns true when a
  // line was produced. MORE-flagged fragments are dropped: multi-
  // fragment inbound reassembly is deliberately out of scope.
  bool tryReceiveLine(uint8_t* outBuf, size_t outCap, size_t* outLen);

  // Set the radio group this robot listens/transmits on. Always stores
  // `group` into group_ unconditionally.
  //
  // Supported path: called from `on start`, before the radio has come
  // up (radioReady_ == false), which is all the student-facing block
  // does -- ensureRadioReady() reads group_, not a constant, the first
  // time it runs, and brings the radio up already on that group.
  //
  // If the radio is ALREADY up this re-applies uBit.radio.setGroup()
  // immediately rather than silently no-opping, but whether that
  // changes what an armed radio actually receives on is UNVERIFIED --
  // see setChannel() just below for the source reading behind the doubt
  // and what would settle it.
  void setGroup(uint8_t group);

  // Set the radio channel (CODAL frequency band). Same store-then-apply
  // contract as setGroup() above: always stores into channel_, and
  // ensureRadioReady() reads that field -- not the kChannel constant --
  // when it lazily brings the radio up, so a call made BEFORE the radio
  // is up is the supported path, and the only one
  // Protocol::setupRadio() uses.
  //
  // UNVERIFIED (2026-08-29): the already-up path, where this re-applies
  // uBit.radio.setFrequencyBand(channel_) against live hardware, has
  // never been observed either way (open issue
  // clasi/issues/low/changing-the-radio-group-mid-run-is-unverified.md).
  // A SOURCE READING of the vendored MicroBitRadio.cpp -- not a
  // measurement -- shows setFrequencyBand() performing an explicit
  // NVIC_DisableIRQ / TASKS_DISABLE / write / TASKS_RXEN restart cycle,
  // commented "We need to restart the radio for the frequency change to
  // take effect", where setGroup() only writes PREFIX0 and returns.
  // What that restart does to an in-flight link is unknown. What would
  // settle it: bring the radio up, call this with a different channel,
  // and check from a second board on the new channel whether traffic
  // resumes -- capturing the result to a file this comment can name.
  void setChannel(uint8_t channel);

  // Event-driven RX internals (public only for the static MessageBus
  // trampoline): mirrors the reference driver's design -- datagram.recv()
  // is ONLY called inside the MICROBIT_RADIO_EVT_DATAGRAM handler, where
  // the queue is guaranteed non-empty. Bench-measured: polling recv() on
  // an EMPTY queue kills the program within two polls (codal's shared
  // EmptyPacket refcounting), which is exactly why the reference never
  // polls. The handler copies a complete single-fragment line into
  // rxLine_ and sets rxReady_; tryReceiveLine() just consumes the flag.
  // A single-fragment datagram whose declared LEN (after stripping the
  // trailing 0x0A) exceeds rxLine_'s capacity is REJECTED whole --
  // rxOversizeDropped_ counts it, rxReady_ is left untouched, and no
  // prefix is copied -- see radioRxLineFits()'s own doc comment, above,
  // for why this must never truncate-and-accept instead.
  void onDatagram();

  // Truncation bound for sendLine()'s `len`, and this module's real
  // radio-capacity ceiling: 240 bytes, EQUAL to
  // SerialTransport::kMaxLineBytes (serial_transport.h),
  // Wire::WireHandler::kMaxLineBytes (wire_handler.h) and this class's
  // own private RX capacity just below -- all four are the same number.
  // tests/host/test_wire_constants_drift.py pins that four-way equality
  // by reading the headers as text, so changing one of the four fails a
  // test instead of silently reintroducing an inequality.
  //
  // 241 B (a full payload plus sendLine()'s trailing '\n') still fits
  // ONE physical fragment: the MTU is MICROBIT_RADIO_MAX_PACKET_SIZE
  // (250, pxt.json) - kFrameHeaderBytes (3) = 247 (kMtu,
  // sendFragmented(), radio_transport.cpp), so that loop still runs its
  // single-iteration path for every real payload.
  //
  // PUBLIC so protocol.cpp's Protocol::emitLine() clips to this
  // constant by name rather than a bare literal of its own -- the two
  // drifted apart silently once. Still a compile-time constant, still
  // sizing payloadBuf_ below.
  static constexpr size_t kMaxPayloadBytes = 240;

 private:
  void ensureRadioReady();

  // Fragments `payload[0..payloadLen)` -- which already carries its own
  // trailing '\n' as the last byte, appended by sendLine() -- into
  // on-air frames of up to one packet's payload capacity each. That
  // capacity is derived from MICROBIT_RADIO_MAX_PACKET_SIZE, whatever
  // this build's CODAL target actually resolves it to (this module
  // never hardcodes a value). Always emits at least one fragment, even
  // for a zero-length payload, so a degenerate empty line still gets a
  // valid START|END frame.
  void sendFragmented(const uint8_t* payload, size_t payloadLen);

  // RadioRelay wire spec section 5 fragment framing -- see this
  // header's top comment for the reference file this mirrors.
  static constexpr uint8_t kFlagStart = 0x01;
  static constexpr uint8_t kFlagMore = 0x02;
  static constexpr uint8_t kFlagEnd = 0x04;
  // FLAG_ACK (0x10) deliberately not declared -- TX-only, see top
  // comment: nothing in this module ever sets or interprets it.

  static constexpr int kFrameHeaderBytes = 3;  // [SEQ][FLAGS][LEN]

  // kChannel/kGroup are an UN-BAKED placeholder (4/10), not any board's
  // address: tools/make_deploy.py's _inject_radio_channel() rewrites
  // BOTH in the deploy SCRATCH COPY from the robot's own
  // radio-robot-lib config (connection.radio_channel /
  // connection.radio_group), so every build overwrites them. They are
  // the DEFAULTS for channel_/group_ below, which setChannel()/
  // setGroup() can still move; kTransmitPower matches the reference
  // driver's own setTransmitPower(7) and has no settable surface at
  // all.
  //
  // The config wins over the name-derived default scheme
  // (make_deploy.py's derive_radio_from_name()) because a five-letter
  // micro:bit name is DEVICEID[1] reduced to 3125 values: two boards
  // CAN share one, derive the same address, and then talk over each
  // other invisibly. A hand-set config pair is the only escape from
  // that. Who is on what is mirrored, human-readably, in
  // .claude/rules/playfield-testing.md -- read numbers out of the
  // per-robot JSON, never out of that table or out of here.
  //
  // DO NOT reformat the kChannel or kGroup lines. tools/make_deploy.py's
  // _K_CHANNEL_RE / _K_GROUP_RE are
  // `(static constexpr int kChannel = )\d+(;)` and
  // `(static constexpr int kGroup = )\d+(;)`, and the deploy raises if
  // either stops matching -- loudly, but every per-robot build breaks
  // until it is fixed.
  static constexpr int kChannel = 4;
  static constexpr int kGroup = 10;
  static constexpr int kTransmitPower = 7;

  // Radio channel actually used -- MUTABLE, defaulting to the
  // deploy-injected kChannel just above, so a build with no setChannel()
  // call behaves exactly as before. setChannel() (above) is the only way
  // to change it, and ensureRadioReady() (radio_transport.cpp) reads
  // this field, not the constant, when it lazily brings the radio up.
  uint8_t channel_ = static_cast<uint8_t>(kChannel);

  // Radio group this robot listens/transmits on -- MUTABLE, defaulting
  // to the deploy-injected kGroup above exactly as channel_ defaults to
  // kChannel, so a build whose config names no group is byte-identical
  // to before (kGroup is 10, the value this field used to hold
  // literally). setGroup() (above) is the only way to change it, and
  // ensureRadioReady() (radio_transport.cpp) reads this field, not the
  // constant, when it lazily brings the radio up.
  uint8_t group_ = static_cast<uint8_t>(kGroup);

  // Send-path scratch buffers, deliberately MEMBERS not stack locals:
  // the protocol fiber's 2 KB stack cannot afford ~450 B of line+frame
  // buffers at the bottom of the deepest call chain (bench-measured:
  // run()+the (since-retired) DIAG-surface formatter+sendLine+
  // sendFragmented overflowed the fiber stack and hard-faulted ~1 s
  // after boot). Single-fiber use: the protocol fiber is sendLine()'s
  // only writer (see its own doc comment), so these are shared scratch
  // in the sense of "reused every call", not "reached concurrently".
  uint8_t payloadBuf_[kMaxPayloadBytes + 1];
  uint8_t frameBuf_[256];

  // Set by enable(); the whole of this class's opt-in gate -- see
  // enable()'s own doc comment above.
  bool enabled_ = false;
  bool radioReady_ = false;
  volatile bool rxReady_ = false;
  size_t rxLen_ = 0;

  // RX line-buffer capacity, in bytes: the wire grammar's own 240-byte
  // line ceiling (Wire::WireHandler::kMaxLineBytes, wire_handler.h),
  // duplicated here as this header's OWN independent constant rather
  // than included by name -- src/DESIGN.md §1's layering table places
  // Transports below the Wire grammar, so this header must not
  // #include "wire_handler.h" (the same reason SerialTransport carries
  // its own independent 240). MUST stay == both of those, and the drift
  // test pins it. Sized off the wire grammar's line cap, NOT off the
  // physical single-fragment MTU (~247 B, see sendFragmented()) -- the
  // MTU is comfortably larger, so this buffer's job is to carry one
  // whole v6 line, not to reach radio's own physical ceiling.
  static constexpr size_t kMaxLineBytes = 240;
  uint8_t rxLine_[kMaxLineBytes];

 public:
  // What the RX path has heard and what it did with it, all four
  // counters in one place (RadioRxCounters, above). Read through
  // Protocol's radioRx*Count() accessors, which shims.cpp surfaces at
  // diag ordinals 31-34. Read-only by design: onDatagram() and
  // radioRxClassify() are the only writers.
  const RadioRxCounters& rxCounters() const { return rxCounters_; }

 private:
  RadioRxCounters rxCounters_;

  uint8_t txSeq_ = 0;  // rolling RadioRelay §5 sequence number
};

}  // namespace diffDrive
