// emit_queue.h -- the ring behind the outbound emit path.
//
// WHAT THIS REPLACES. The line a caller handed to the protocol's emit
// path used to go straight onto the wire from whatever fiber called
// it: a direct serial write followed by a direct radio mirror. That
// made the calling fiber a second producer into the serial driver,
// racing the protocol fiber's own writes -- see this module's caller
// (Protocol::emitLine(), comms/protocol.cpp) for the full mechanism.
// This ring turns that direct write into a copy-and-return: the text
// is copied into a slot here, and only the fiber that owns this ring
// ever reads a slot back out and puts it on the wire. That gives the
// underlying transport writes exactly one caller, by construction,
// regardless of how many fibers call enqueue().
//
// WHY DRAIN, NOT RANDOM ACCESS. Unlike a ring whose consumer reads a
// specific slot back by index (see run_queue.h's own RunQueue, which
// hands a MessageBus listener a slot number to read later), this
// ring's only consumer is a drain loop that wants every line, in the
// order it arrived, with nothing left keyed by index once it has been
// read. dequeue() therefore both returns and releases the oldest slot
// in one call, copying its text OUT to a caller-supplied buffer rather
// than handing back an internal pointer -- the drain loop's own write
// to the wire can block or yield, and a pointer into this ring's
// storage would not survive a concurrent enqueue() landing in the same
// slot across that yield.
//
// A refusal (ring full) counts rather than silently overwriting a
// queued line, and the counter SATURATES rather than wrapping, same
// convention as RunQueue -- a drop count that rolls over to zero reads
// as "nothing was lost".
//
// Host-portable on purpose -- no pxt.h, no CODAL types, nothing but
// <cstdint>/<cstring> -- same split run_queue.h/heading_wrap.h/
// encoder_glitch_armor.h already use: the logic worth testing lives
// where a host test can reach it. Exercised host-side by the FIFO-order
// and drop-counting behavior a dedicated pytest module drives through a
// small ctypes shim (see tests/host/'s own test for this file).
#pragma once

#include <cstdint>
#include <cstring>

namespace diffDrive {

template <int Slots = 8, int Bytes = 48>
class EmitQueue {
 public:
  static constexpr int kSlots = Slots;
  static constexpr int kBytes = Bytes;

  // Copy `len` bytes into the next free slot (plus a NUL this class
  // adds itself -- `len` describes the text only). Refuses -- counting
  // a drop -- when the ring is already full or `len` does not fit a
  // slot with room left for that NUL. An empty line is refused too,
  // uncounted: there is nothing to emit, so it is not a capacity
  // problem.
  bool enqueue(const char* text, size_t len) {
    if (text == nullptr || len == 0 || len >= static_cast<size_t>(Bytes)) {
      return false;
    }
    if (count_ >= Slots) {
      if (dropped_ < UINT32_MAX) ++dropped_;
      return false;
    }
    char* slot = slots_[tail_];
    std::memcpy(slot, text, len);
    slot[len] = '\0';
    lens_[tail_] = len;
    tail_ = (tail_ + 1) % Slots;
    ++count_;
    return true;
  }

  // Copy the oldest queued line into `dest` (capacity `destCap`,
  // NUL-terminated on return) and release its slot. Returns the copied
  // length, or 0 when the ring is empty or `dest`/`destCap` cannot
  // hold anything -- 0 is never a valid enqueued length (enqueue()
  // refuses an empty line), so it doubles as an unambiguous empty
  // signal without a separate "is there anything" call.
  size_t dequeue(char* dest, size_t destCap) {
    if (count_ == 0 || dest == nullptr || destCap == 0) return 0;
    const int slot = head_;
    size_t len = lens_[slot];
    if (len >= destCap) len = destCap - 1;  // defensive; slots_ never
                                             // holds more than Bytes-1
    std::memcpy(dest, slots_[slot], len);
    dest[len] = '\0';
    head_ = (head_ + 1) % Slots;
    --count_;
    return len;
  }

  int count() const { return count_; }
  uint32_t dropped() const { return dropped_; }
  bool empty() const { return count_ == 0; }
  bool full() const { return count_ >= Slots; }

 private:
  char slots_[Slots][Bytes] = {};
  size_t lens_[Slots] = {};
  int head_ = 0;
  int tail_ = 0;
  int count_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace diffDrive
