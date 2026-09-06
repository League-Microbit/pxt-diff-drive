// run_bridge.h -- RunBridge: everything that happens to a cleartext
// "RUN:<name>[:<arg>...]" payload between arriving on a transport and
// being handed to the TypeScript dispatcher.
//
// Three jobs, and only these three: sanitize the payload, suppress a
// host's own retransmits, and park what survives in the run_queue.h
// ring until a consumer is ready for it. It does NOT call TypeScript
// and it does NOT arbitrate the drivetrain -- Protocol keeps both of
// those, because only Protocol can see a wire request, a dispatched
// job and a block-program move at the same time. RunBridge just says
// which text is next and whether it must be dispatched right now.
//
// Host-portable on purpose -- no pxt.h, no CODAL types, nothing but
// <cstddef>/<cstdint> and run_queue.h -- so the parking/dedupe/bypass
// rules can be exercised with no Protocol, no fiber and no radio. The
// clock is a parameter (`now`) for the same reason: a test steps time
// exactly across the dedupe window's edges.
#pragma once

#include <cstddef>
#include <cstdint>

#include "run_queue.h"

namespace diffDrive {

class RunBridge {
 public:
  // Payload sizing: name + args + NUL, and enough slots for a burst
  // arriving while a long job (a whole tour) is still dispatching.
  static constexpr size_t kTextBytes = 48;
  static constexpr int kSlots = 8;

  // Repeat-suppression window. Hosts repeat commands to survive the
  // robot's single-slot inbound wireless buffer, and without this a
  // repeated RUN executes once per copy -- the test programs' re-entry
  // guard has already cleared by the time a retransmit lands. The ring
  // fixes LOSS; this fixes duplicate EXECUTION, a different failure.
  // 400 ms swallows a retransmit burst and still lets a deliberate
  // repeat through, which a parameter sweep sends constantly.
  static constexpr int32_t kDedupe = 400;  // [ms]

  // What offer() did with the payload. The caller only has to act on
  // kBypass (dispatch immediately); every other outcome is already
  // fully handled here.
  enum class Offer : uint8_t {
    kMalformed,   // empty, oversized, non-printable, or empty name
    kSuppressed,  // same text as the last accepted one, inside kDedupe
    kBypass,      // staged into currentText() -- dispatch it NOW
    kQueued,      // parked for dispatchOne()
    kDropped,     // every slot still in flight; counted by dropCount()
  };

  // Take one raw payload (the bytes AFTER "RUN:", not NUL-terminated)
  // that arrived at `now`, and decide its fate.
  //
  // Suppression is by (text, arrival time) HERE, at the point of
  // arrival, not at handling time -- which is what makes it immune to
  // however long the payload then sits in the ring. Two commands that
  // differ only in their arguments are different text, so they are not
  // each other's repeats.
  //
  // A kBypass payload has already been copied into currentText(); the
  // caller dispatches it without consulting any drivetrain owner. See
  // isBypassName() for why those two names cannot wait in line.
  Offer offer(const uint8_t* data, size_t len, uint32_t now);  // [ms]

  // Stage the oldest parked payload into currentText() and release its
  // slot, returning false when nothing is waiting. The slot is released
  // BEFORE the caller dispatches -- a dispatch can run for as long as
  // the job itself does, and holding a slot hostage for that whole span
  // would cost capacity a burst arriving during it needs.
  bool dispatchOne();

  // The payload most recently staged by dispatchOne() or by a kBypass
  // offer(). Never null: the caller hands this straight to a string
  // API. Valid only while that dispatch is executing -- a nested
  // bypass arriving mid-job overwrites it, which is safe precisely
  // because every handler reads it back at its OWN entry, before doing
  // anything else.
  const char* currentText() const { return currentText_; }

  // Payloads refused because every slot was still in flight. Saturating
  // (run_queue.h's own counter) -- a drop count that wrapped to zero
  // would read as "nothing was lost".
  uint32_t dropCount() const { return queue_.dropped(); }

  // Payloads refused by offer()'s sanitizer: empty, overlong,
  // non-printable, or an empty name. Deliberately SEPARATE from
  // dropCount() above -- a malformed line is a parse problem and a
  // dropped one is a capacity problem, and folding them together would
  // make either number a lie about the other. Saturating, same reason.
  uint32_t malformedCount() const { return malformedCount_; }

  // Payloads parked and not yet staged.
  int queued() const { return queue_.count(); }

  // True for the two names that skip the queue entirely -- and the
  // dedupe window with it: a repeated `abort` inside kDedupe still
  // executes.
  //
  // A QUEUED abort would sit behind the very job it is meant to stop,
  // because a consumer refuses to start a second job while one owns the
  // drivetrain -- it would dispatch only after the tour it was sent to
  // end. Both handlers these names reach are trivial, non-blocking and
  // safe to invoke reentrantly inside a running job's call chain, which
  // is what makes the bypass sound.
  //
  // `text` is matched up to (not including) its first ':', or the whole
  // payload if there is none, mirroring how the TypeScript dispatcher
  // splits a command into name + arguments.
  static bool isBypassName(const char* text);

 private:
  // The one buffer both staging paths write.
  void stage(const char* text);

  // Counts one sanitizer refusal and returns the outcome to hand back,
  // so every `return Offer::kMalformed` site is a single call that
  // cannot forget the counter.
  Offer malformed();

  RunQueue<kSlots, static_cast<int>(kTextBytes)> queue_;
  char currentText_[kTextBytes] = {};
  char lastText_[kTextBytes] = {};
  uint32_t lastAccepted_ = 0;  // [ms] arrival time of the last accepted payload
  uint32_t malformedCount_ = 0;
};

}  // namespace diffDrive
