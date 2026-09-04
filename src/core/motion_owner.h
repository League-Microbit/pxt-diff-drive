// motion_owner.h -- MotionOwner: the single value that arbitrates which
// caller currently owns the drivetrain.
//
// Four owners share one drivetrain: kNone (idle), kWire (a live wire
// motion obligation), kJob (a dispatched RUN job), and kBlock (the
// block program's own fiber -- a student's move()/goTo()/driveTwist()/
// startDrive(), or a MessageBus button handler calling one of those
// directly). Before kBlock existed, a block-motion call reached the
// engine unconditionally, with no arbitration at all -- it could
// supersede a live wire move, and the wire's own completion channel
// then resolved that superseded move as an ordinary stop, indistinct
// from one the host itself caused.
//
// This header holds only the type and the pure take/release rule for
// kBlock -- host-portable (no pxt.h, no CODAL type) so
// tests/host/test_motion_owner.py can pin every case directly. The
// owning value itself lives on whichever CODAL-facing object can see
// every caller (comms/protocol.cpp's own Protocol::motionOwner_); this
// header is what keeps that field, and comms/wire_adapter.h's own
// mirror of it (needed there because that class must stay CODAL-free),
// expressed as the SAME enum instead of two independently-maintained
// booleans answering overlapping questions.
#pragma once

#include <cstdint>

namespace diffDrive {

enum class MotionOwner : uint8_t { kNone, kWire, kJob, kBlock };

// The one arbitration rule a block-motion entry point applies before it
// ever touches the engine: take kBlock and return true iff `*owner` is
// currently kNone, otherwise leave `*owner` untouched and return false
// -- refuse, never silently supersede a live kWire/kJob move.
inline bool tryTakeBlockOwnership(MotionOwner* owner) {
  if (*owner != MotionOwner::kNone) return false;
  *owner = MotionOwner::kBlock;
  return true;
}

// Releases kBlock ownership -- a no-op unless `*owner` is currently
// kBlock, so a caller that never actually held it (or a caller racing
// another owner's already-completed take/release pair) can never
// clobber someone else's claim.
inline void releaseBlockOwnership(MotionOwner* owner) {
  if (*owner == MotionOwner::kBlock) *owner = MotionOwner::kNone;
}

}  // namespace diffDrive
