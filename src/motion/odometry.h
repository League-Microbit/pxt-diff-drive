// diffDrive::Odometry -- the robot's dead-reckoned pose, and the
// PoseSource MotionEngine::goToW() reads when no OTOS is fitted.
//
// READS MUTATE ODOMETRY: this class never advances itself, and shims.cpp's
// pose reads call update() first, so a pose read DOES advance odometry.
// That is load-bearing, not incidental -- see DESIGN.md before changing it.
#pragma once

#include <cmath>
#include <cstdint>

#include "motion_engine.h"

namespace diffDrive {

class Odometry : public PoseSource {
 public:
  // Geometry is read from `engine` fresh on every update(), so `engine`
  // must outlive this object and be declared before it.
  explicit Odometry(const MotionEngine& engine) : engine_(engine) {}

  // Folds one kernel Output into the frame. Diffs against the last Output
  // consumed and re-stamps it, so a tick with no encoder movement is a
  // no-op -- safe to call unconditionally.
  void update(const DiffDrive::DifferentialDrive::Output& out) {
    // A rebase re-anchors the wheel positions to a new software zero: an
    // intentional discontinuity, so this sample cannot be diffed against
    // the last one. Same handling as the very first call. This is the
    // codebase's only reader of positionEpochLeft/Right.
    const bool rebased =
        primed_ && (out.positionEpochLeft != positionEpochLeft_ ||
                    out.positionEpochRight != positionEpochRight_);
    if (!primed_ || rebased) {
      consume(out);
      primed_ = true;
      return;
    }
    const float cpm = engine_.countsPerMm();                     // [counts/mm]
    const float dLeft = (out.positionLeft - posLeft_) / cpm;      // [mm]
    const float dRight = (out.positionRight - posRight_) / cpm;   // [mm]
    consume(out);
    const float dCenter = 0.5f * (dLeft + dRight);                // [mm]
    const float dHeading =
        (dRight - dLeft) / engine_.effectiveTrackWidth();         // [rad]
    const float midHeading = heading_ + 0.5f * dHeading;          // [rad]
    x_ += dCenter * std::cos(midHeading);
    y_ += dCenter * std::sin(midHeading);
    heading_ += dHeading;
  }

  // Zeroes the frame. The wheel baseline is deliberately untouched:
  // callers call update() first, so pending motion is folded in and then
  // discarded with the frame rather than reappearing as a later jump.
  void reset() { seed(0.0f, 0.0f, 0.0f); }

  // Declares the frame from an external fix -- same "update() first" idiom.
  void seed(float x, float y, float heading) {  // [mm] [mm] [rad]
    x_ = x;
    y_ = y;
    heading_ = heading;
  }

  float x() const override { return x_; }  // [mm] world frame
  float y() const override { return y_; }  // [mm] world frame

  // [rad] world frame, CCW+, UNWRAPPED -- one of the two conventions
  // PoseSource permits. Do not difference two reads across implementations.
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

// The selection rule engineGoToW() (shims.cpp) applies. Extracted from its
// one call site only so a host test can exercise it against two fakes.
inline PoseSource& selectPoseSource(bool primaryConnected, PoseSource& primary,
                                    PoseSource& fallback) {
  return primaryConnected ? primary : fallback;
}

}  // namespace diffDrive
