// diffDrive::Segment -- one constant-ratio motion-engine plan and its
// progress. Owns what a segment IS and how far along it is; never decides
// speeds (that is VelocityShaper). See DESIGN.md.
#pragma once

#include <cmath>
#include <cstdint>

#include "../core/diffdrive.h"

namespace diffDrive {

struct Segment {
  enum class Axis : uint8_t { kDistance, kYaw };

  float distTarget = 0.0f;  // [counts] signed mean-axis target
  float yawTarget = 0.0f;   // [counts] signed half-differential target
  float cruise = 0.0f;      // [mm/s] caller's ceiling before limits
  Axis dominantAxis = Axis::kDistance;
  float dominant = 0.0f;    // [counts] |target| on the dominant wheel

  // Captured on the first service() tick after start(), never at start().
  bool originPending = true;
  float posLeft0 = 0.0f, posRight0 = 0.0f;  // [counts]

  // Phase 2 of a pivot-then-straight split. Its cruise may differ from
  // phase 1's `cruise` above.
  bool hasPending = false;
  float pendingDistance = 0.0f;  // [mm]
  float pendingCruise = 0.0f;    // [mm/s]

  uint32_t deadline = 0;  // [ms] the caller's timeout backstop
  bool active = false;
  bool settling = false;
  uint8_t restSamples = 0;
  uint32_t restSampleLeft = 0;
  uint32_t restSampleRight = 0;

  // Rotation only, no translation. A blended arc is NOT a pure turn.
  bool pureTurn() const { return yawTarget != 0.0f && distTarget == 0.0f; }

  // [counts] dominant-axis distance still to travel. Signed "toward the
  // target" on the yaw axis, so a pivot briefly rotating the wrong way is
  // never credited as progress; unsigned magnitude on the distance axis.
  float remaining(const DiffDrive::DifferentialDrive::Output& out) const {
    const float dLeft = out.positionLeft - posLeft0;     // [counts]
    const float dRight = out.positionRight - posRight0;  // [counts]
    if (dominantAxis == Axis::kYaw) {
      const float diffProgress = 0.5f * (dRight - dLeft);
      const float toward = yawTarget > 0.0f ? diffProgress : -diffProgress;
      return std::fabs(yawTarget) - toward;
    }
    const float meanProgress = 0.5f * (dLeft + dRight);
    return std::fabs(distTarget) - std::fabs(meanProgress);
  }

  // [counts] signed progress TOWARD the commanded yaw direction. Exposed
  // separately from wrongWay() so a caller can gate when that verdict is
  // trustworthy at all without recomputing it.
  float yawProgress(const DiffDrive::DifferentialDrive::Output& out) const {
    const float dLeft = out.positionLeft - posLeft0;
    const float dRight = out.positionRight - posRight0;
    const float diffProgress = 0.5f * (dRight - dLeft);
    return yawTarget > 0.0f ? diffProgress : -diffProgress;
  }

  // Rotating AWAY from the commanded direction. Always a yaw question: a
  // segment with no rotation component cannot go the wrong way by this
  // measure. Margin scales with the yaw target -- see DESIGN.md.
  bool wrongWay(const DiffDrive::DifferentialDrive::Output& out) const {
    if (yawTarget == 0.0f) return false;
    const float toward = yawProgress(out);
    const float margin = 0.25f * std::fabs(yawTarget);
    return toward < -(margin > kWrongWayMargin ? margin : kWrongWayMargin);
  }

  // [0..1] fraction of whichever axis has a nonzero target; the min across
  // both when both do, so neither axis reports done before the other.
  float progress(const DiffDrive::DifferentialDrive::Output& out) const {
    const float dLeft = out.positionLeft - posLeft0;
    const float dRight = out.positionRight - posRight0;
    float fraction = 1.0f;
    if (distTarget != 0.0f) {
      const float f =
          std::fabs(0.5f * (dLeft + dRight)) / std::fabs(distTarget);
      if (f < fraction) fraction = f;
    }
    if (yawTarget != 0.0f) {
      const float f =
          std::fabs(0.5f * (dRight - dLeft)) / std::fabs(yawTarget);
      if (f < fraction) fraction = f;
    }
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;
    return fraction;
  }

 private:
  static constexpr float kWrongWayMargin = 12.0f;  // [counts] noise floor
};

}  // namespace diffDrive
