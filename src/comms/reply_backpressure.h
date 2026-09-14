// reply_backpressure.h -- a reply line waits for room in a bounded
// transmit ring instead of being dropped.
//
// WifiLink queues every outbound line into an 8-slot ring (kTxSlots)
// that drains one AT+CIPSEND round-trip at a time, and drops the newest
// line when the ring is full. A reply the wire handler writes all at
// once and longer than the ring -- FUNCS writes one `funcs <name>` line
// per registered function -- therefore lost everything past the eighth
// line over WiFi, while USB serial (which writes directly) delivered the
// whole list. MEASURED on a calibration image built with
// pxt-nezha-diffdrive v1.20260912.8: the function list over WiFi always
// ended at `calx`, the seventh name, though `cala`, `spin` and `diag`
// are present in the image and answer RUN.
//
// The fix keeps WifiLink's own drop-newest contract (telemetry must never
// stall) and moves the waiting to the one caller that may block: the
// protocol fiber writing a REPLY. This header is that wait, extracted as
// a pure function so tests/host/test_reply_backpressure.py can drive it
// with a fake link and a fake clock -- comms/protocol.cpp includes pxt.h
// and cannot be compiled host-side.
#pragma once

#include <cstdint>

namespace diffDrive {

// Waits until `link` has room for one more line, driving it forward
// while it waits. Returns true when there is room.
//
//   link   anything with `int queuedSends() const` and `bool ready() const`
//   slots  the ring's capacity (WifiLink::kTxSlots)
//   pump   advances the in-flight send (the WiFi join sequencer's service())
//   yield  lets other fibers and the UART interrupt run between pumps
//   now    a millisecond clock
//
// Gives up and returns false once `budget` has elapsed, or at once if the
// link stops being ready: a dropped link will never drain, and the caller
// then falls back to the transport's normal drop behaviour.
template <typename Link, typename Pump, typename Yield, typename Now>
inline bool waitForTxRoom(Link& link, int slots, Pump pump, Yield yield,
                          Now now, uint32_t budget) {  // [ms]
  if (link.queuedSends() < slots) return true;
  const uint32_t start = now();  // [ms]
  while (link.ready() && link.queuedSends() >= slots) {
    if (static_cast<uint32_t>(now() - start) >= budget) return false;
    pump();
    yield();
  }
  return link.queuedSends() < slots;
}

}  // namespace diffDrive
