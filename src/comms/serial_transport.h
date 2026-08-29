// serial_transport.h -- SerialTransport: owns the raw USB-serial byte
// stream and 0x0A line delimiting. Thin CODAL-facing leaf (mirrors how
// nezha_port.{h,cpp} is the thin I2C-facing port beneath the DiffDrive
// kernel): knows uBit.serial and the 0x0A byte, nothing about verb
// names or command semantics -- see protocol.h for that layer.
//
// Byte buffers, not ManagedString: line content may legally contain an
// embedded 0x00 byte, so this module and everything layered on it
// carry explicit (buffer, length) pairs end to end rather than
// NUL-terminated strings.
#pragma once

#include <cstddef>
#include <cstdint>

namespace diffDrive {

// One wire line's content, excluding the trailing 0x0A this module
// owns and strips/appends. MUST stay == Wire::WireHandler::kMaxLineBytes
// (wire_handler.h, 240): if this layer were the tighter cap, it would
// truncate an overlong line into a still-parseable prefix one layer
// below WireHandler's own tested discard-whole-line guarantee.
constexpr size_t kMaxLineBytes = 240;

// RX/TX serial ring capacity used by begin() (sprint 004 ticket 006,
// code review R-19/WIRE-03; corrected by ticket 007 -- remediating
// ticket 005's thrown exception). v5 sized these at a flat 128 B, tuned
// for a ~27-byte binary WHEELS frame -- that sizing driver is gone under
// v6, where a single line can legally be kMaxLineBytes (240 B). The
// protocol fiber only drains the ring once per ~24 ms motion-tick
// window (shims.cpp's self-pacing tick), so a near-max-length line plus
// anything else arriving in the same window (a keepalive ack, a
// reliability-layer resend) can exceed one line's worth of bytes before
// the next drain -- 128 B overflows on exactly that pattern, silently
// (codal's ring drops the overflow with no signal).
//
// CONFIRMED (ticket 007; was UNVERIFIED under ticket 006): codal-core's
// real setRxBufferSize()/setTxBufferSize() (inc/driver-models/Serial.h)
// take `uint8_t size`, capping at 255. Ticket 006's original
// `2 * kMaxLineBytes` (480) silently truncated to 224 on assignment --
// BELOW kMaxLineBytes (240) itself, defeating the resize entirely with
// nothing but an easy-to-miss `-Woverflow` build warning as the signal;
// this is what ticket 005's bench checkpoint caught. This is the hard
// ceiling, not the 2x-kMaxLineBytes margin ticket 006 intended: 255
// leaves only ~15 bytes of headroom above one full 240-byte line --
// enough for one maximal line plus a little slack, NOT enough to hold
// two full lines concurrently. If a second maximal-length line arrives
// before the first is drained, overflow is still possible -- a known,
// documented residual limitation of the codal-core uint8_t API ceiling,
// not something engineered further around here (see ticket 007's own
// notes). Brace-initialized (not `=`) so any future edit that pushes
// this past 255 is a HARD COMPILE ERROR (narrowing conversion in a
// constant expression) instead of a repeat of this exact silent
// truncation.
constexpr uint8_t kRingBytes{255};

class SerialTransport {
 public:
  // One-time setup: grows codal's default ~20-byte serial rings to
  // kRingBytes (ticket 006; previously a flat 128 B tuned for v5's
  // binary frames -- see kRingBytes' own comment) so a full line
  // arriving as one burst, or two lines arriving in the same
  // motion-tick window, can't overflow them between protocol-fiber
  // polls. Call before the first read.
  void begin();


  // Writes `len` bytes from `buf`, then a single 0x0A delimiter.
  // Callers (Protocol) never include the delimiter themselves.
  //
  // Two-writer guard (sprint 004 ticket 006, code review R-19/R-20 aka
  // WIRE-03/WIRE-04): two fibers call this today -- the TS fiber via
  // Protocol::emitLine(), and the protocol fiber via the serial
  // WireHandler's own replies/keepalives (Protocol::SerialSink::write())
  // -- and this predates ticket 006 itself; the review is what caught
  // it, unfixed, already present. Each call issues two back-to-back
  // uBit.serial.send(..., SYNC_SLEEP) calls that block and yield the
  // caller, exactly the window the other caller could interleave bytes
  // into. Unlike RadioTransport::sendLine()'s guard (ticket 002), where
  // a second caller drops immediately and only Protocol::emitLine()
  // retries once, BOTH callers here get a bounded retry: a caller that
  // finds the guard already held sleeps 2 ms and checks again, up to a
  // small fixed attempt cap, because serial has no caller whose loss is
  // "fine" the way telemetry's self-healing seq gap makes radio's drop
  // acceptable (see sprint.md's Design Rationale). If the retry cap is
  // exhausted, or either uBit.serial.send() call itself reports a
  // failure, the attempt is counted in a drop counter (read via
  // diagValue(26)/probe(26), shims.cpp) and this function returns
  // having given up silently -- callers do not check a return value or
  // retry themselves; the bounded retry and the drop accounting are
  // both fully internal to this call, unlike RadioTransport's
  // caller-driven retry-once.
  void writeLine(const uint8_t* buf, size_t len);

  // Non-blocking: lets a caller interleave cadence-driven work --
  // telemetry -- with command reads on one fiber. Never sleeps: drains
  // only whatever bytes uBit.serial already has buffered (ASYNC reads),
  // accumulating them across calls in a small internal partial-line
  // buffer.
  //
  // Returns true iff that drain completed one full line (a 0x0A delimiter
  // was seen this call or an earlier one): `outBuf`/`*outLen` are filled
  // with the line content, truncated to `outCap` if the line is longer --
  // bytes beyond outCap are dropped, not overrun. Returns false --
  // `*outLen` left untouched -- whenever no delimiter has arrived yet,
  // including when nothing at all was available; any bytes read this call
  // are retained internally as a head start on the next call.
  bool tryReadLine(uint8_t* outBuf, size_t outCap, size_t* outLen);

  // Count of writeLine() calls dropped since boot (ticket 006): either
  // the two-writer guard's retry cap was exhausted before this call got
  // a turn, or one of writeLine()'s own uBit.serial.send() calls itself
  // reported a failure. Exposed to the wire protocol's numeric DIAG
  // surface via diagValue(26) (shims.cpp) / probe(26) (bench) -- a
  // bench operator watches this stay at 0 during a normal run the same
  // way the existing counters (i2cFaultCount, cycleOverrunCount, etc.)
  // are already read.
  uint32_t dropCount() const { return dropCount_; }

 private:
  // Accumulation is fixed at kMaxLineBytes regardless of a given call's
  // `outCap`; outCap only bounds the final copy-out (see tryReadLine()).
  uint8_t partial_[kMaxLineBytes] = {0};
  size_t partialLen_ = 0;

  // Two-writer guard for writeLine() (ticket 006): true from the moment
  // a caller enters the guarded body until it returns. A second caller
  // arriving while this is already true does NOT drop immediately the
  // way RadioTransport::sending_ does -- it sleeps and retries, bounded
  // (see writeLine()'s own doc comment and serial_transport.cpp's
  // kMaxSendAttempts). Only the caller that actually set this clears
  // it, on its own way out.
  bool sending_ = false;

  // Backing counter for dropCount() above.
  uint32_t dropCount_ = 0;
};

}  // namespace diffDrive
