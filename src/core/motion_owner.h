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

// The rule a MOTION entry point (src/shims.cpp's startMove()/
// driveTwist()/engineGoToRArmed()) actually applies before it ever
// touches the engine -- tryTakeBlockOwnership() above, EXTENDED to
// recognize a dispatched RUN job's own call.
//
// Every one of those entry points is reached from TWO kinds of caller
// that look identical from inside shims.cpp: the TS RUN handler a
// dispatched job invoked (comms/protocol.cpp's own dispatchJob(),
// running SYNCHRONOUSLY on Protocol's own fiber, having already set
// `*owner` to kJob around the whole call span) -- or a genuine
// block-program caller (a MessageBus button-handler's own fiber, or a
// student script's main fiber, neither of which is Protocol's fiber).
// `isDispatchingFiber` is that distinction: true iff the CURRENT call
// is executing on Protocol's own fiber, computed by the caller
// (Protocol::tryTakeMotionOwnership(), comms/protocol.cpp) the SAME
// way core/fiber_identity.h's shouldServiceHookRun() already compares
// fiber identity for the tick service hook, and passed in here as a
// plain bool so this header stays free of any fiber/pointer type.
//
// If the caller IS the dispatching fiber AND `*owner` is already kJob,
// this is that job's own move: let it through UNCHANGED -- take
// nothing, touch nothing. dispatchJob() itself already owns the kJob
// take/release pair bracketing the whole call (protocol.cpp), so there
// is no new ownership here to release later. The `*owner == kJob`
// conjunct is defensive, not load-bearing: structurally, the ONLY way
// a motion entry point runs on Protocol's own fiber at all is via a
// dispatched job's call chain, which already set kJob before invoking
// it -- but requiring it explicitly means a caller that broke that
// invariant would fall through to the ordinary rule below instead of
// bypassing it unconditionally.
//
// Otherwise, fall through to the ordinary kBlock take/refuse rule
// above: refused, never silently superseding a live kWire/kJob move,
// EXACTLY as before. This is the case that preserves the correct
// refusal a genuine block call (a button, a script) arriving while a
// job or wire motion holds the drivetrain -- verified on hardware.
inline bool tryTakeMotionOwnership(MotionOwner* owner,
                                    bool isDispatchingFiber) {
  if (isDispatchingFiber && *owner == MotionOwner::kJob) return true;
  return tryTakeBlockOwnership(owner);
}

}  // namespace diffDrive
