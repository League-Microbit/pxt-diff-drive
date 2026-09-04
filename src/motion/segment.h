// segment.h -- diffDrive::Segment: one constant-ratio motion-engine plan
// and its progress (design docs/design/motion-profile-unification.md
// S4.3). Replaces MotionEngine's old MoveState (motion_engine.h,
// pre-sprint-029): owns what a segment IS and how far along it is;
// never decides speeds itself -- that is VelocityShaper's job
// (velocity_shaper.h), driven by MotionEngine::service() (design S5).
//
// Host-portable by construction: <cstdint>/<cmath> plus
// "../core/diffdrive.h" only (for DiffDrive::DifferentialDrive::Output)
// -- no pxt.h, no CODAL type, anywhere.
//
// LAZY ORIGIN CAPTURE (design S6.5): a Segment is built with
// `originPending = true` and no drive() is ever issued at construction
// time. The first MotionEngine::service() call after start() -- which
// always follows that tick's own kernel_.step() -- captures
// posLeft0/posRight0 from THAT tick's Output and clears originPending.
// Between construction and that first step(), the wheels cannot have
// moved (nothing was commanded yet), and any rebasePosition() deferred
// before the segment started has, by construction, already been applied
// by that same step() -- so the segment's own origin is never stale.
// This retires the old MoveState's positionEpochLeft0/Right0 pair
// entirely: there is no rebase race left to guard against (design S6.5,
// S4.8).
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

  // Origin: captured LAZILY on the first service() call after start(),
  // never at start() -- see this file's header comment.
  bool originPending = true;
  float posLeft0 = 0.0f, posRight0 = 0.0f;  // [counts]

  // Pending second phase (pivot-then-straight, moveX()'s own split
  // rule -- motion-api.md S3.3). `pendingCruise` is the phase 2
  // straight leg's own cruise, which may differ from phase 1's own
  // `cruise` field (moveX()'s dual-rate reconciliation, shims.cpp's
  // startMove()).
  bool hasPending = false;
  float pendingDistance = 0.0f;  // [mm]
  float pendingCruise = 0.0f;    // [mm/s]

  uint32_t deadline = 0;  // [ms] the caller's timeout backstop
  bool active = false;

  // A pure pivot: rotation only, no translation (motion-api.md S2.1's
  // degenerate case). The single predicate this struct's own
  // dominantAxis mirrors at construction (see startSegment()'s own
  // builder in motion_engine.cpp): a blended arc (both distTarget and
  // yawTarget nonzero) is NOT a pure turn -- its dominant axis is
  // distance (an arc only exists below the pivot-first split threshold,
  // where distance dominates).
  bool pureTurn() const { return yawTarget != 0.0f && distTarget == 0.0f; }

  // [counts] dominant-axis distance still to travel, signed positive
  // while approaching the target (see this method's own two branches
  // for what "toward" means per axis). Callers divide by countsPerMm()
  // to get the [mm] VelocityShaper::advance() wants (design S5's
  // `remain = seg_.remaining(out) / cpm`).
  //
  // dominantAxis == kYaw (a pure pivot): the differential wheel-position
  // delta, signed TOWARD the commanded rotation direction -- mirrors the
  // old MoveState's own signed "toward" yaw progress (motion_engine.cpp,
  // pre-sprint-029), which existed so a pivot that is briefly rotating
  // the WRONG way is never credited as progress.
  //
  // dominantAxis == kDistance (a straight leg or a blended arc): the
  // mean wheel-position delta's MAGNITUDE, unsigned -- the distance axis
  // has no wrongWay() check of its own (see that method below), so the
  // old code's plain |meanProgress| carries forward unchanged.
  float remaining(const DiffDrive::DifferentialDrive::Output& out) const {
    const float dLeft = out.positionLeft - posLeft0;    // [counts]
    const float dRight = out.positionRight - posRight0;  // [counts]
    if (dominantAxis == Axis::kYaw) {
      const float diffProgress = 0.5f * (dRight - dLeft);
      const float toward = yawTarget > 0.0f ? diffProgress : -diffProgress;
      return std::fabs(yawTarget) - toward;
    }
    const float meanProgress = 0.5f * (dLeft + dRight);
    return std::fabs(distTarget) - std::fabs(meanProgress);
  }

  // True when the robot is rotating AWAY from the commanded direction --
  // always a YAW question (motion_engine.cpp's pre-sprint-029
  // serviceMove() only ever checked this for a nonzero yawTarget,
  // whether the segment was a pure pivot or a blended arc); a segment
  // with no rotation component at all (yawTarget == 0, a plain straight
  // leg) can never go the wrong way by this measure. `kWrongWayMargin`
  // mirrors the old code's own `3 * yawMargin` threshold (yawMargin was
  // 4 counts on a pure pivot, matching the old 0.16 deg completion
  // window) -- a fixed constant, not derived from MotionLimits, since
  // this struct holds no reference to one (by design -- see this
  // struct's own field list, S4.3).
  bool wrongWay(const DiffDrive::DifferentialDrive::Output& out) const {
    if (yawTarget == 0.0f) return false;
    const float dLeft = out.positionLeft - posLeft0;
    const float dRight = out.positionRight - posRight0;
    const float diffProgress = 0.5f * (dRight - dLeft);
    const float toward = yawTarget > 0.0f ? diffProgress : -diffProgress;
    // The margin scales with the yaw target: on a blended arc the yaw
    // axis is small next to the travel axis, and the wheels do not
    // start together -- MEASURED tovez 2026-09-04,
    // captures/bench-acceptance-029-20260904d/ (lag-measure.log: left
    // lag 0.115 s vs right 0.146 s; g2-run.log: two of three forward
    // LEFT 45 deg arcs, MOVE_X 300 785, ended on their first tick with
    // no motion while every right arc ran; probe-arc.log: the same
    // command ran when the wheels happened to start together). A fixed
    // 12-count margin read that start-up skew as a wrong-way turn. A
    // quarter of the yaw target still catches a genuinely inverted
    // wheel within the first few degrees of any real pivot.
    const float margin = 0.25f * std::fabs(yawTarget);
    return toward < -(margin > kWrongWayMargin ? margin : kWrongWayMargin);
  }

  // [0..1] fraction of whichever axis (or axes, for a blended arc) has
  // a nonzero target -- the min across both when both are nonzero, same
  // "neither axis may report done before the other" rule the old
  // MoveState-based progress() used. No rebase-race clamp is needed
  // here (unlike the old code's own progress()) -- see this file's
  // header comment on why lazy origin capture retires that guard
  // entirely.
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
  // [counts] see wrongWay()'s own comment. 4-count pivot margin * 3,
  // the old code's own constant (motion_engine.cpp, pre-sprint-029).
  static constexpr float kWrongWayMargin = 12.0f;
};

}  // namespace diffDrive
