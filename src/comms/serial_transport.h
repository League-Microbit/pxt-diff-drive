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

// RX/TX serial ring capacity used by begin(). 255 is a HARD CEILING,
// not a chosen size: codal-core's setRxBufferSize()/setTxBufferSize()
// (inc/driver-models/Serial.h) take a `uint8_t`, so a naive 480
// silently truncates to 224 -- BELOW one maximal line -- with nothing
// but an easy-to-miss `-Woverflow` warning to show for it.
//
// That leaves ~15 bytes of slack above one full 240-byte line: enough
// for one maximal line and a little more, NOT enough for two. A second
// maximal line arriving before the first is drained can still overflow,
// silently (codal's ring drops the overflow with no signal) -- a
// residual limitation of the uint8_t API, not engineered around here.
//
// Brace-initialized (not `=`) so a future edit past 255 is a HARD
// COMPILE ERROR (narrowing conversion in a constant expression) rather
// than a repeat of that silent truncation.
constexpr uint8_t kRingBytes{255};

class SerialTransport {
 public:
  // One-time setup: grows codal's default ~20-byte serial rings to
  // kRingBytes (see its own comment) so a full line arriving as one
  // burst can't overflow them between protocol-fiber polls -- those
  // happen once per ~24 ms motion-tick window. Call before the first
  // read.
  void begin();


  // Writes `len` bytes from `buf`, then a single 0x0A delimiter.
  // Callers (Protocol) never include the delimiter themselves.
  //
  // Single writer: the protocol fiber. That matters because each call
  // issues two back-to-back uBit.serial.send(..., SYNC_SLEEP) calls
  // that block and YIELD (serial_transport.cpp's own note): with one
  // writer a yield mid-line simply resumes; with two it would
  // interleave their bytes. Nothing here serializes writers, because
  // there are none to serialize.
  //
  // No return value: a caller cannot do anything useful with a failure
  // this deep. If either uBit.serial.send() reports one, the line is
  // counted in a drop counter (read via diagValue(26)/probe(26),
  // shims.cpp) and this returns having given up silently.
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

  // Count of writeLine() calls dropped since boot: one of that call's
  // own uBit.serial.send() calls reported a failure. Exposed on the
  // numeric DIAG surface as diagValue(26) (shims.cpp) / probe(26)
  // (bench); a bench operator watches it stay at 0 across a normal run,
  // the same way i2cFaultCount and the rest are read.
  uint32_t dropCount() const { return dropCount_; }

 private:
  // Accumulation is fixed at kMaxLineBytes regardless of a given call's
  // `outCap`; outCap only bounds the final copy-out (see tryReadLine()).
  uint8_t partial_[kMaxLineBytes] = {0};
  size_t partialLen_ = 0;

  // Backing counter for dropCount() above.
  uint32_t dropCount_ = 0;
};

}  // namespace diffDrive
