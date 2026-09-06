// run_queue.h -- the ring behind cleartext RUN: dispatch.
//
// A fixed ring with OCCUPANCY, not a bare write cursor: a slot is in
// flight from enqueue() until release(), and enqueue() refuses --
// counting a drop, saturating -- rather than trampling a slot still in
// flight. Without that, a burst arriving while a long handler runs
// overwrites payload that handler has not read yet, silently: no
// counter moves, no reply changes, and the handler runs the wrong
// command.
//
// Occupancy is closed by RunBridge (run_bridge.h), which parks a
// payload here and stages it back out from Protocol::dispatchJob(), on
// the protocol fiber -- release() happens BEFORE the TypeScript handler
// is called, so a job as long as a whole tour holds no slot. Allocation
// walks the ring in order, so what a consumer sees is FIFO by
// construction, and dequeue() exposes that order directly for a caller
// that wants to drain rather than be pushed. The 400 ms same-text
// suppression in front of the ring (RunBridge::kDedupe) solves a
// DIFFERENT problem -- duplicate EXECUTION of a host's retransmit --
// and is not a substitute for occupancy.
//
// Host-portable on purpose -- no pxt.h, no CODAL types, nothing but
// <cstdint>/<cstring> -- the same split heading_wrap.h and
// encoder_glitch_armor.h use.
#pragma once

#include <cstdint>
#include <cstring>

namespace diffDrive {

template <int Slots = 8, int Bytes = 48>
class RunQueue {
 public:
  static constexpr int kSlots = Slots;
  static constexpr int kBytes = Bytes;

  // Copy `len` bytes (plus a NUL) into a free slot and return its index,
  // or -1 when every slot is still in flight. A refusal counts, and the
  // counter SATURATES rather than wrapping -- a drop count that rolls
  // over to zero is worse than one pinned at the top, because it reads
  // as "nothing was lost".
  int enqueue(const char* text, int len) {
    if (len < 0 || len >= Bytes) return -1;
    if (count_ >= Slots) {
      if (dropped_ < UINT32_MAX) ++dropped_;
      return -1;
    }
    const int slot = tail_;
    std::memcpy(slots_[slot], text, static_cast<size_t>(len));
    slots_[slot][len] = '\0';
    live_[slot] = true;
    tail_ = (tail_ + 1) % Slots;
    ++count_;
    return slot;
  }

  // The text in `slot`, or "" if that slot holds nothing. Never null:
  // the caller hands this straight to a string API.
  const char* at(int slot) const {
    if (slot < 0 || slot >= Slots || !live_[slot]) return "";
    return slots_[slot];
  }

  // Mark `slot` consumed. Idempotent -- releasing a slot twice is a
  // no-op, not a double-decrement, because a lossy transport can
  // genuinely deliver the same read twice.
  void release(int slot) {
    if (slot < 0 || slot >= Slots || !live_[slot]) return;
    live_[slot] = false;
    slots_[slot][0] = '\0';
    if (count_ > 0) --count_;
    while (count_ > 0 && !live_[head_]) head_ = (head_ + 1) % Slots;
    if (count_ == 0) head_ = tail_;
  }

  // Oldest in-flight slot index, or -1 when empty. Does not release.
  int peek() const {
    if (count_ == 0) return -1;
    int i = head_;
    for (int n = 0; n < Slots; ++n) {
      if (live_[i]) return i;
      i = (i + 1) % Slots;
    }
    return -1;
  }

  // Oldest in-flight slot index, released. -1 when empty.
  int dequeue() {
    const int s = peek();
    if (s >= 0) release(s);
    return s;
  }

  int count() const { return count_; }
  uint32_t dropped() const { return dropped_; }
  bool full() const { return count_ >= Slots; }

 private:
  char slots_[Slots][Bytes] = {};
  bool live_[Slots] = {};
  int head_ = 0;
  int tail_ = 0;
  int count_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace diffDrive
