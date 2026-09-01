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

// Pure accept/reject decision for an inbound RX fragment (sprint 010
// ticket 001, radio-rx-capacity-fragmentation.md): true iff a fragment
// whose payload declares `declaredLen` bytes (after
// RadioTransport::onDatagram() has already stripped the trailing 0x0A
// delimiter) fits WHOLE into a receive buffer of `bufferCapacity` bytes.
// False means REJECT the frame in its entirety -- the caller must drop
// it outright, exactly like an already-dropped MORE-flagged fragment,
// and must NEVER truncate it to a shorter, still-parseable prefix and
// deliver that prefix as if it were the complete line.
//
// Truncate-and-accept (the pre-fix behavior) was the actual hazard this
// function exists to close: WireHandler::feed() cannot tell a truncated
// line from a genuinely short one the host sent, so a truncated
// over-length command could silently decode and EXECUTE as a different,
// shorter, legal command. A dropped line is merely invisible; a
// truncated-and-accepted one is dangerous -- see
// radio-rx-capacity-fragmentation.md for the full defect writeup.
//
// No CODAL dependency (this header includes only <cstddef>/<cstdint>),
// so this function is host-testable directly by #include-ing this
// header -- no link against radio_transport.cpp, which requires pxt.h
// and cannot be host-compiled at all. See
// tests/host/test_radio_transport_rx_capacity.py.
inline bool radioRxLineFits(size_t declaredLen, size_t bufferCapacity) {
  return declaredLen <= bufferCapacity;
}

class RadioTransport {
 public:
  // Fragments `data` (len bytes) into RadioRelay-framed radio packets
  // and transmits each one via uBit.radio.datagram.send(), appending a
  // trailing 0x0A ('\n') as the final payload byte -- the same
  // one-terminator-per-line convention SerialTransport::writeLine()
  // uses. Truncates -- rather than overflows -- a `len` beyond this
  // module's internal line-buffer capacity, mirroring SerialTransport's
  // own defensive truncation.
  //
  // Lazily enables and configures the radio (uBit.radio.enable(),
  // group/channel/power -- see group_/kChannel/kTransmitPower below;
  // group is student-settable via setGroup(), channel and power are
  // fixed -- matching the reference driver's own begin()) on the FIRST
  // call, never at construction and never via a separate begin() step:
  // uBit.radio.enable() has its own RAM/softdevice cost, so a
  // bench-only serial user who never calls sendLine() never pays it.
  //
  // Re-entrancy guard: TWO fibers can call this -- the TS fiber via
  // Protocol::emitLine(), and the protocol fiber via RadioSink::write()
  // (replies and emitTelemetry()'s frames) -- and
  // uBit.radio.datagram.send() can block and yield, giving the two a
  // real chance to interleave mid-format into payloadBuf_/frameBuf_.
  // Returns false, WITHOUT TOUCHING payloadBuf_/frameBuf_ at all, if a
  // call is already in progress on the other fiber; true after a
  // normal completion. The dropped caller decides for itself whether
  // that matters: Protocol::emitLine() retries once after
  // fiber_sleep(2); RadioSink::write() ignores the return value and
  // accepts the drop silently -- deliberately different from
  // SerialTransport::writeLine()'s own guard, which retries internally
  // instead of dropping (see that method's own doc comment).
  bool sendLine(const uint8_t* data, size_t len);

  // RX (radio command plane, single-fragment only): polls one queued
  // datagram, accepts frames whose flags carry START|END together (a
  // complete message in one fragment -- with the 250-byte fleet packet
  // size every relay-forwarded command line qualifies), strips the
  // trailing 0x0A, and copies the line into outBuf. Returns true when a
  // line was produced. MORE-flagged fragments are dropped (multi-
  // fragment inbound reassembly is deliberately out of scope; see
  // clasi/issues/radio-rx-command-plane-run-over-bridge.md).
  bool tryReceiveLine(uint8_t* outBuf, size_t outCap, size_t* outLen);

  // Set the radio group this robot listens/transmits on -- the ONE
  // write path the blocks layer gains into this class's configuration;
  // channel and transmit power stay fixed constexpr values, below, and
  // are NOT settable this way. Always stores `group` into group_
  // unconditionally.
  //
  // Supported path: called from `on start`, before the radio has come
  // up (radioReady_ == false). Nothing else happens here in that case --
  // ensureRadioReady() reads group_ (not a hardcoded constant) the first
  // time it actually runs, and brings the radio up already on the
  // requested group. This is the student-facing path the block targets.
  //
  // If the radio has ALREADY come up (radioReady_ == true, e.g. a prior
  // sendLine()/tryReceiveLine() already lazily called
  // ensureRadioReady()), this re-applies the group immediately via
  // uBit.radio.setGroup(group_) so the call does not silently no-op.
  // Whether that re-apply actually changes what the already-armed radio
  // receives on is UNVERIFIED on this hardware -- no test of this path
  // has been run. As a source observation only (not a measurement): the
  // vendored MicroBitRadio.cpp's setFrequencyBand() performs an explicit
  // TASKS_DISABLE/TASKS_RXEN restart with the comment "We need to
  // restart the radio for the frequency change to take effect", while
  // its setGroup() only writes NRF_RADIO->PREFIX0 and returns, with no
  // such restart. What that difference means for reception on
  // already-armed hardware has not been observed either way.
  void setGroup(uint8_t group);

  // Set the radio channel (CODAL frequency band). Same store-then-apply
  // contract as setGroup() just above: always stores into channel_, and
  // ensureRadioReady() reads that field -- not the kChannel constant --
  // when it lazily brings the radio up, so a call made BEFORE the radio
  // is up brings it up already on the requested channel. That is the
  // supported path, and the only one Protocol::setupRadio() uses.
  //
  // UNVERIFIED (2026-08-29): the already-up path, where this re-applies
  // uBit.radio.setFrequencyBand(channel_) against live hardware, has
  // never been observed either way -- exactly the open question
  // clasi/issues/changing-the-radio-group-mid-run-is-unverified.md
  // raises for setGroup(). The concern is stronger here, not weaker: a
  // SOURCE READING of the vendored MicroBitRadio.cpp (not a
  // measurement) shows setFrequencyBand() performing an explicit
  // NVIC_DisableIRQ / TASKS_DISABLE / write / TASKS_RXEN restart cycle,
  // commented "We need to restart the radio for the frequency change to
  // take effect", where setGroup() only writes PREFIX0 and returns.
  // What that restart does to an in-flight link is unknown.
  //
  // What would settle it: bring the radio up (any sendLine()), call
  // this with a different channel, and check from a second board on the
  // new channel whether traffic resumes -- capturing the result to a
  // file this comment can then name.
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

  // Truncation bound for sendLine()'s `len` parameter, and this
  // module's real radio-capacity ceiling. PUBLIC as of sprint 008
  // ticket 002 (WIRE-05/R-21) -- was private, moved here (not simply
  // relabeled in place) so no other private member below picks up
  // public access as a side effect. Made public so protocol.cpp's
  // Protocol::emitLine() can clip to this SAME constant by name instead
  // of re-declaring its own bare literal, which had silently drifted
  // out of sync with what this constant actually means: once sprint
  // 004 ticket 005 raised SerialTransport::kMaxLineBytes to 240, this
  // constant -- and radio's real capacity -- stayed at its old, smaller
  // value, and emitLine()'s own separate literal was numerically right
  // but disconnected from that fact, which is what let it read as
  // merely stale rather than load-bearing.
  //
  // RAISED to 240 by sprint 010 ticket 002
  // (radio-rx-capacity-fragmentation.md): this constant now EQUALS
  // SerialTransport::kMaxLineBytes (serial_transport.h) and
  // Wire::WireHandler::kMaxLineBytes (wire_handler.h) -- and this
  // class's own private RX-capacity constant just below (kMaxLineBytes,
  // sprint 010 ticket 001) -- all four are the SAME number, 240. Sprint
  // 008's version of this comment described the relationship as
  // deliberately the smaller of the two transports' bounds, not equal
  // to SerialTransport's own; that was true at the old value and is no
  // longer true now. tests/host/test_wire_constants_drift.py pins this
  // four-way equality by reading all the relevant headers as text, so a
  // future edit to any one of the four numbers fails a test instead of
  // silently reintroducing an inequality. The widened value still fits
  // one physical radio fragment: the MTU (kMtu = MICROBIT_RADIO_MAX_
  // PACKET_SIZE(250, pxt.json) - kFrameHeaderBytes(3) = 247, see
  // sendFragmented() in radio_transport.cpp) has 6 bytes of margin above
  // the 241-byte payload+delimiter this constant now allows, so
  // sendFragmented()'s multi-fragment loop still runs its
  // single-iteration path for every real payload. No encapsulation
  // cost: it stays a compile-time constant, still used in-class below
  // to size payloadBuf_ -- the class still owns every byte of storage
  // it sizes.
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

  // Radio convention matching the fleet's RADIOBRIDGE relay: channel 4
  // (vevov's fleet-assigned channel; the zavaz relay matches: !CG 4 10);
  // transmit power 7 (matches the reference driver's own
  // setTransmitPower(7)). kChannel is injected per-robot at DEPLOY time
  // into the SCRATCH COPY only (tools/make_deploy.py's
  // _inject_radio_channel()); it is now the DEFAULT for channel_ below
  // rather than the value the radio uses directly, because the "setup
  // radio" block gives students a surface for it. Transmit power still
  // has no settable surface, student-facing or per-robot.
  //
  // DO NOT reformat the kChannel or kGroup lines. tools/make_deploy.py's
  // _K_CHANNEL_RE / _K_GROUP_RE are
  // `(static constexpr int kChannel = )\d+(;)` and
  // `(static constexpr int kGroup = )\d+(;)`, and the deploy raises if
  // either stops matching -- loudly, but every per-robot build breaks
  // until it is fixed.
  static constexpr int kChannel = 4;

  // WHERE A ROBOT'S (channel, group) COMES FROM -- read this before
  // changing either constant or the config they are injected from.
  //
  // BY DEFAULT BOTH ARE DERIVED FROM THE BOARD'S NAME. A micro:bit's
  // five-letter friendly name is NRF_FICR->DEVICEID[1] written in base
  // 5, so the pair is CALCULABLE from the name alone, by anyone,
  // offline, with no registry:
  //
  //     n = base5(name)                     // name[0] most significant
  //     channel = 25 + 2 * (n % 25)         // odd, 25..73
  //     group   = 1 + n / 25, +1 if >= 10   // 1..9, 11..126
  //
  // (`tools/make_deploy.py`'s `derive_radio_from_name()` is the
  // implementation, and cites the normative spec.)
  //
  // BUT THE CONFIG IS AUTHORITATIVE, NOT THE DERIVATION. The robot does
  // NOT compute this at boot: both values are read from the robot's
  // radio-robot-lib config (`connection.radio_channel` /
  // `connection.radio_group`) and baked in here at DEPLOY time. The
  // derivation is how a pair is ASSIGNED to a name so the fleet stays
  // collision-free and hand-picked numbers stop drifting -- it is not a
  // runtime behaviour, and nothing on the robot re-derives it.
  //
  // The reason config wins is that a name is not guaranteed unique.
  // The name is a base-5 view of DEVICEID[1], which is 32 bits reduced
  // to 3125 values, so TWO BOARDS CAN SHARE A NAME. It is rare, but
  // when it happens the two boards derive the SAME (channel, group) and
  // would talk over each other with nothing to see. Config is the
  // escape hatch: give one of them a different pair by hand, and the
  // build honours it. A derivation with no override has no answer for
  // that case at all.
  //
  // kGroup is currently 10 fleet-wide -- the RADIOBRIDGE relay's listen
  // group, and the value radio-robot-elite's `robot_config.proto` still
  // documents as fixed. Note the derived scheme above can NEVER emit
  // group 10 (it is skipped, being the relay's `!C` button space), so
  // the fleet's present group and the derived groups are disjoint by
  // construction: migrating is a coordinated reflash of robots AND
  // relay, never a per-robot edit. Until that happens, a config with no
  // `radio_group` key keeps 10.
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

  // kMaxPayloadBytes itself is declared PUBLIC, above (sprint 008
  // ticket 002) -- moved out of this section rather than merely
  // relabeled in place, so nothing below silently became public with
  // it. payloadBuf_'s bound below resolves against that earlier, public
  // declaration; C++ does not require a data member's array bound to be
  // declared textually adjacent to it, only earlier in the class.

  // Send-path scratch buffers, deliberately MEMBERS not stack locals:
  // the protocol fiber's 2 KB stack cannot afford ~450 B of line+frame
  // buffers at the bottom of the deepest call chain (bench-measured:
  // run()+the (since-retired) DIAG-surface formatter+sendLine+
  // sendFragmented overflowed the fiber stack and hard-faulted ~1 s
  // after boot). No longer single-fiber
  // use only as of sprint 004 ticket 002: two fibers now call
  // sendLine() (see its header comment), and sending_ below is what
  // keeps only one of them touching these buffers at a time.
  uint8_t payloadBuf_[kMaxPayloadBytes + 1];
  uint8_t frameBuf_[256];

  bool radioReady_ = false;
  // Re-entrancy guard for sendLine()'s payloadBuf_/frameBuf_-touching
  // body (sprint 004 ticket 002): true from the moment a caller enters
  // that body until it returns. A second caller arriving while this is
  // already true returns false immediately, touching neither buffer;
  // only the caller that actually set this clears it, on its own way
  // out -- the dropped caller never touches it.
  bool sending_ = false;
  volatile bool rxReady_ = false;
  size_t rxLen_ = 0;

  // RX line-buffer capacity, in bytes: the wire grammar's own 240-byte
  // line ceiling (Wire::WireHandler::kMaxLineBytes, wire_handler.h),
  // duplicated here as radio_transport.h's OWN independent constant
  // rather than included by name -- src/DESIGN.md §1's layering table
  // places Transports below the Wire grammar, so this header must not
  // #include "wire_handler.h" (the same layering reason
  // SerialTransport::kMaxLineBytes, serial_transport.h, already exists
  // as ITS OWN independent 240 rather than including wire_handler.h
  // either). MUST stay == both of those (see ticket 002's drift test).
  // Sized off the wire grammar's line cap, NOT off the physical
  // single-fragment MTU (~247 B: MICROBIT_RADIO_MAX_PACKET_SIZE (250,
  // pxt.json) - kFrameHeaderBytes (3), see sendFragmented()'s kMtu in
  // radio_transport.cpp) -- the MTU is comfortably larger, so this
  // buffer's job is to carry one whole v6 line, not to reach radio's
  // own physical ceiling. Was a bare `64` with no name of its own
  // before sprint 010 ticket 001 (radio-rx-capacity-fragmentation.md);
  // naming it lets radioRxLineFits() (above) and this ticket's own host
  // test pin the real capacity by value instead of by an anonymous
  // array bound.
  static constexpr size_t kMaxLineBytes = 240;
  uint8_t rxLine_[kMaxLineBytes];

 public:
  // RX diagnostics (bench): datagrams polled with nonzero length, and
  // frames accepted as complete single-fragment lines. Bench-only
  // counters; the cleartext DIAG verb that used to read them
  // (Protocol::formatDiag()) was retired, and nothing in the current
  // tree consumes these.
  uint32_t rxFrames_ = 0;
  uint32_t rxAccepted_ = 0;
  // Count of single-fragment datagrams REJECTED because their declared
  // LEN exceeded rxLine_'s capacity (sprint 010 ticket 001,
  // radio-rx-capacity-fragmentation.md) -- dropped whole, never
  // truncated-and-accepted; see radioRxLineFits()'s own doc comment for
  // why. Same bench-diagnostics convention as rxFrames_/rxAccepted_
  // above.
  uint32_t rxOversizeDropped_ = 0;

 private:
  uint8_t txSeq_ = 0;  // rolling RadioRelay §5 sequence number
};

}  // namespace diffDrive
