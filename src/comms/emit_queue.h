// emit_queue.h -- the ring behind the outbound emit path.
//
// Protocol::emitLine() can be called from ANY fiber. Writing the wire
// there directly made the calling fiber a second producer into the
// serial driver, racing the protocol fiber's own writes; this ring
// turns that into a copy-and-return, so the transport writes have
// exactly one caller by construction however many fibers enqueue().
//
// WHY DRAIN, NOT RANDOM ACCESS. The only consumer is the protocol
// fiber's own drain loop, which wants every line in the order it
// arrived with nothing left keyed by index. dequeue() therefore both
// returns AND releases the oldest slot, copying its text OUT to a
// caller-supplied buffer rather than handing back an internal pointer
// -- that write to the wire can block or yield, and a pointer into this
// ring's storage would not survive an enqueue() landing in the same
// slot across the yield.
//
// A refusal (ring full) counts rather than silently overwriting a
// queued line, and the counter SATURATES rather than wrapping, same
// convention as RunQueue -- a drop count that rolled over to zero would
// read as "nothing was lost".
//
// Host-portable on purpose -- no pxt.h, no CODAL types, nothing but
// <cstdint>/<cstring> -- same split run_queue.h/heading_wrap.h/
// encoder_glitch_armor.h already use, and exercised host-side through a
// small ctypes shim (tests/host/'s own test for this file).
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
