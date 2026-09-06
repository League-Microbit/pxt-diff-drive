// odometry.h -- Odometry: the robot's dead-reckoned pose, as one object.
// Turns the kernel's published wheel positions (DiffDrive::
// DifferentialDrive::Output) into (x, y, heading) by midpoint-heading
// integration, and IS the PoseSource (motion_engine.h) that
// MotionEngine::goToW() reads when no OTOS is fitted -- most of the
// fleet (the OTOS is on vevov only).
//
// Replaces four things that used to be spread across shims.cpp: Rig's
// x/y/heading + odomPos*/odomPrimed/odomPositionEpoch* fields, the free
// function odomUpdate() over them, a read-only PoseSource adapter that
// held const float& into those fields (platform/encoder_pose_source.h,
// deleted -- see src/DESIGN.md S7), and the rebase-epoch guard. The
// integration math below is odomUpdate()'s, moved unchanged.
//
// READS MUTATE ODOMETRY -- the contract, stated here because it is
// otherwise invisible at the call sites. This class does not advance
// itself: x()/y()/heading() are pure reads of the current frame. The
// CALLER advances it, and shims.cpp's poseX()/poseY()/poseHeading()
// deliberately call update() immediately before each read, so a pose
// read DOES advance odometry as a side effect. That is load-bearing,
// not incidental (src/DESIGN.md S9): between moves nothing else calls
// update(), so a host's 50 ms telemetry poll is the only thing keeping
// pose current -- make the reads pure and pose silently freezes
// whenever the tick loop is idle. tickDrive() calls update()
// unconditionally every tick; updateMove() calls it only while a move
// was active. Both gates are unchanged from the free function this
// class absorbed.
//
// GEOMETRY comes from the MotionEngine reference, read fresh on every
// update() -- countsPerMm() and effectiveTrackWidth() are the single
// source of truth for both, and a SET travel_calib/track_width between
// two updates must take effect on the next one. The reference is bound
// once at construction, so an Odometry must be declared AFTER the
// engine it reads (members initialize in DECLARATION order) and must
// not outlive it; in production both are members of shims.cpp's
// process-lifetime Rig.
//
// HEADING WRAP: deliberately UNWRAPPED -- heading() accumulates without
// normalizing, one of the two conventions PoseSource's own comment
// (motion_engine.h) declares valid (OtosPort wraps to (-pi, pi] because
// its int16 register does). Valid because goToR()/goToW() consume
// heading() only through cos()/sin(). A caller that DIFFERENCES two
// heading() reads must not assume a shared convention across
// implementations.
//
// Host-portable: motion_engine.h only (no pxt.h/CODAL anywhere), so a
// host test drives the real class -- tests/host/test_odometry.py over
// tests/host/odometry_shim.cpp.
#pragma once

#include <cmath>
#include <cstdint>

#include "motion_engine.h"

namespace diffDrive {

class Odometry : public PoseSource {
 public:
  explicit Odometry(const MotionEngine& engine) : engine_(engine) {}

  // Folds one kernel Output into the frame. Diffs against the last
  // Output consumed and immediately re-stamps it, so calling this on a
  // tick with no new encoder movement is a no-op -- safe to call
  // unconditionally, which is what tickDrive() does.
  void update(const DiffDrive::DifferentialDrive::Output& out) {
    // A rebase request (SET rebase -> kernel.rebasePosition()) re-anchors
    // positionLeft/positionRight to a new software zero at the kernel's
    // own NEXT step() -- an intentional discontinuity, not drift.
    // positionEpochLeft/Right (diffdrive.h) change only alongside that
    // re-anchor, so a change on either wheel since the last call means
    // this sample cannot be diffed against the last one -- the same
    // handling the very first call (`!primed_`) already gets, for the
    // same reason (there is no prior sample this one can continue).
    // This is the codebase's ONLY reader of positionEpochLeft/Right.
    const bool rebased =
        primed_ && (out.positionEpochLeft != positionEpochLeft_ ||
                    out.positionEpochRight != positionEpochRight_);
    if (!primed_ || rebased) {
      consume(out);
      primed_ = true;
      return;
    }
    const float cpm = engine_.countsPerMm();          // [counts/mm]
    const float dLeft = (out.positionLeft - posLeft_) / cpm;    // [mm]
    const float dRight = (out.positionRight - posRight_) / cpm;  // [mm]
    consume(out);
    const float dCenter = 0.5f * (dLeft + dRight);              // [mm]
    const float dHeading =
        (dRight - dLeft) / engine_.effectiveTrackWidth();       // [rad]
    const float midHeading = heading_ + 0.5f * dHeading;        // [rad]
    x_ += dCenter * std::cos(midHeading);
    y_ += dCenter * std::sin(midHeading);
    heading_ += dHeading;
  }

  // Zeroes the frame (resetPose(), SET rebase). The wheel baseline is
  // deliberately untouched: callers consume pending deltas by calling
  // update() first, so motion since the last update is folded in and
  // then discarded with the frame, rather than reappearing as a jump on
  // the next update.
  void reset() { seed(0.0f, 0.0f, 0.0f); }

  // Declares the frame from an external fix (seedPose(), v6 SEED) --
  // same "call update() first" idiom as reset().
  void seed(float x, float y, float heading) {  // [mm] [mm] [rad]
    x_ = x;
    y_ = y;
    heading_ = heading;
  }

  float x() const override { return x_; }  // [mm] world frame
  float y() const override { return y_; }  // [mm] world frame

  // [rad] world frame, CCW+, UNWRAPPED -- see this file's header comment.
  float heading() const override { return heading_; }

 private:
  void consume(const DiffDrive::DifferentialDrive::Output& out) {
    posLeft_ = out.positionLeft;
    posRight_ = out.positionRight;
    positionEpochLeft_ = out.positionEpochLeft;
    positionEpochRight_ = out.positionEpochRight;
  }

  const MotionEngine& engine_;
  float x_ = 0.0f, y_ = 0.0f;  // [mm] world frame
  float heading_ = 0.0f;       // [rad] world frame, CCW+, unwrapped
  float posLeft_ = 0.0f, posRight_ = 0.0f;  // [counts] last consumed
  bool primed_ = false;
  uint32_t positionEpochLeft_ = 0, positionEpochRight_ = 0;
};

// The one-line selection rule engineGoToW() (shims.cpp) applies:
// `primary` (OtosPort in production) when `primaryConnected`,
// `fallback` (the Rig's Odometry in production) otherwise. Extracted
// here, rather than left inline at the one call site, purely so a host
// test can exercise the RULE directly against two fakes -- OtosPort::
// connected() itself has no host-testable seam (otos_port.h includes
// pxt.h unconditionally).
inline PoseSource& selectPoseSource(bool primaryConnected, PoseSource& primary,
                                    PoseSource& fallback) {
  return primaryConnected ? primary : fallback;
}

}  // namespace diffDrive
