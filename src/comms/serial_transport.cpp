// serial_transport.cpp -- see serial_transport.h. Talks to uBit.serial
// directly (the microbit's default USB-CDC serial link), the same way
// nezha_port.cpp talks to uBit.i2c directly: one small CODAL-facing
// leaf, no shaping/porting layers of its own.
#include "serial_transport.h"

#include "pxt.h"

using namespace pxt;

namespace diffDrive {

namespace {
constexpr uint8_t kLineDelimiter = 0x0A;

// Bounded retry cap for writeLine()'s two-writer guard (ticket 006):
// small enough that a fiber stuck waiting cannot meaningfully stall the
// 50 ms telemetry emission cadence (kMaxSendAttempts * 2 ms sleep =
// 10 ms worst case), large enough that ordinary contention between the
// TS fiber's emitLine() and the protocol fiber's own replies/keepalives
// clears well within it. See writeLine()'s own doc comment
// (serial_transport.h) for the policy this implements.
constexpr int kMaxSendAttempts = 5;
}  // namespace

void SerialTransport::begin() {
  // codal's serial rx ring defaults to ~20 bytes -- smaller than a full
  // v6 line, so a line sent as a single burst at 115200 (or a burst
  // plus other traffic in the same motion-tick window) overflows the
  // ring between the protocol fiber's polls and drops bytes (measured
  // on bench, pre-v6: mangled frames, eaten delimiters, merged lines).
  // Size both rings to kRingBytes -- see its own doc comment
  // (serial_transport.h) for why 255, not a larger value, is the real
  // ceiling here.
  uBit.serial.setRxBufferSize(kRingBytes);
  uBit.serial.setTxBufferSize(kRingBytes);
}

void SerialTransport::writeLine(const uint8_t* buf, size_t len) {
  // Two-writer guard, bounded retry on the caller side (ticket 006):
  // unlike RadioTransport::sendLine(), a caller that finds the guard
  // already held does not drop immediately -- it sleeps 2 ms and checks
  // again, up to kMaxSendAttempts times, because serial has no caller
  // whose loss is "fine" (see this function's own doc comment in
  // serial_transport.h). Exhausting the cap without ever acquiring the
  // guard counts as a drop and gives up without sending anything.
  int attempts = 0;
  while (sending_) {
    if (++attempts >= kMaxSendAttempts) {
      ++dropCount_;
      return;
    }
    fiber_sleep(2);
  }
  sending_ = true;

  // Both uBit.serial.send() calls' return values are checked (ticket
  // 006; previously ignored) -- a negative return indicates the send
  // itself failed (mirrors this file's own tryReadLine(), which already
  // treats a negative uBit.serial.read() result as an error/no-data
  // signal). Either call failing counts as one dropped line, not two.
  bool ok = true;
  if (len > 0) {
    if (uBit.serial.send(const_cast<uint8_t*>(buf), static_cast<int>(len),
                         SYNC_SLEEP) < 0) {
      ok = false;
    }
  }
  uint8_t delimiter = kLineDelimiter;
  if (uBit.serial.send(&delimiter, 1, SYNC_SLEEP) < 0) {
    ok = false;
  }

  sending_ = false;
  if (!ok) ++dropCount_;
}

bool SerialTransport::tryReadLine(uint8_t* outBuf, size_t outCap,
                                  size_t* outLen) {
  // ASYNC: never blocks -- returns a buffered byte immediately, or
  // DEVICE_NO_DATA (a negative value) once the rx buffer is drained. The
  // isReadable() guard is belt-and-suspenders (uBit.serial.read(ASYNC)
  // already returns immediately either way); it just avoids a call that
  // would only ever return "no data" once the buffer is empty.
  while (uBit.serial.isReadable()) {
    const int c = uBit.serial.read(ASYNC);
    if (c < 0) break;  // drained (or a transient read error) -- stop here
    if (static_cast<uint8_t>(c) == kLineDelimiter) {
      const size_t len = partialLen_ < outCap ? partialLen_ : outCap;
      if (outBuf != nullptr && len > 0) {
        for (size_t i = 0; i < len; ++i) outBuf[i] = partial_[i];
      }
      *outLen = len;
      partialLen_ = 0;
      return true;
    }
    if (partialLen_ < sizeof(partial_)) {
      partial_[partialLen_++] = static_cast<uint8_t>(c);
    }
    // else: past kMaxLineBytes -- keep consuming so the stream stays
    // framed, just stop copying into partial_.
  }
  return false;  // no complete line yet -- partial_ retained for next call
}

}  // namespace diffDrive
