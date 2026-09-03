// motion_engine.cpp -- see motion_engine.h for the class contract, the
// two-primitive design (motion-api.md S2), the geometry rationale
// (S2.1), and the move engine (S3.3-S3.5). Host-portable: this file
// includes nothing but <cmath> and its own header -- no pxt.h, no CODAL
// type.
#include "motion_engine.h"

#include <cmath>

namespace diffDrive {

namespace {

// [rad] -- literal, matching this project's existing convention (see
// e.g. otos_port.h/shims.cpp) rather than relying on <cmath>'s
// non-standard M_PI.
constexpr float kPi = 3.14159265f;

// Wrap an angle to the short arc, (-pi, pi] -- sprint 006, KERN-03.
// goToR()'s arc-angle formula (`theta = 2*atan2(y, x)`) doubles atan2's
// own principal value, so it can land up to just under +-2*pi even
// though atan2(y,x) itself never exceeds +-pi -- that "long way around"
// value reaches the same (x, y) on the same constant-curvature circle
// as the short, wrapped angle (the circle is periodic), but only the
// short one is a sane distance to actually drive. The input here is
// always bounded to (-2*pi, 2*pi] (twice an atan2 result), so a single
// conditional wrap suffices -- no loop, no fmod, needed.
float wrapToPi(float angleRad) {
  if (angleRad > kPi) return angleRad - 2.0f * kPi;
  if (angleRad <= -kPi) return angleRad + 2.0f * kPi;
  return angleRad;
}

// Constant-a braking-speed axis scale (SUC-001): axisScale = v_allow /
// axisCruiseMmS, where v_allow =
// sqrt(2*aDecelMmS2*remainMm) is the speed permissible with `remainMm`
// [mm] left to travel on this axis at deceleration `aDecelMmS2`
// [mm/s^2], and axisCruiseMmS is THIS AXIS's own full-rate (scale==1.0)
// commanded speed, recovered from the counts/s command startSegment()
// computed for it (`axisCmdCountsPerS`) -- NOT the wheels_x-level
// `cruise` argument directly, since a blended arc's dist/yaw axes each
// run at their own fraction of it (see startSegment()'s velCmd/twistCmd
// derivation). `remainCounts` may be negative (past the target); clamped
// to 0 mm so this always returns a finite, non-negative scale rather
// than NaN. The caller is responsible for its own window gate -- see
// serviceMove()'s own call sites: the dist axis gates on
// constantDecelWindowMm() below (the kinematics themselves) -- as, since
// 2026-09-02, does the yaw axis. This function itself has no notion of
// "outside the window" at all, for either axis; the caller gates it.
float constantDecelAxisScale(float aDecelMmS2, float remainCounts,
                              float axisCmdCountsPerS, float cpm) {
  const float remainMm = remainCounts > 0.0f ? remainCounts / cpm : 0.0f;
  const float vAllow = std::sqrt(2.0f * aDecelMmS2 * remainMm);
  const float axisCruiseMmS = std::fabs(axisCmdCountsPerS) / cpm;
  return axisCruiseMmS > 0.0f ? (vAllow / axisCruiseMmS) : 0.0f;
}

// Braking window before a constant-a stop, in [mm]: the kinematic
// v^2/(2a) an axis genuinely needs to come to rest from its own
// full-rate commanded speed (`axisCmdCountsPerS`, recovered the same
// way constantDecelAxisScale() above does -- not the raw wheels_x-level
// `cruise` argument). serviceMove()'s dist axis gates its shaped-mode
// branch on this instead of on distTaper_: a fixed-counts window is
// smaller than this kinematic one at any meaningful cruise, which left
// the constant-a solve unreachable above roughly 200 mm/s. MEASURED on
// the compiled engine, captures/gopiv-profile-sweep-20260901/
// sweep_gopiv_wide.json: raising the old fixed window from its
// 400-count default to 5000 counts took the cruise-400 ramp-down from
// 1 control tick to 14, and the cruise-300 window (153.8 mm) matched
// this formula's own prediction (150.0 mm) to 2.5%. `aDecelMmS2` is
// the caller's own already-validated (`> 0`) field.
float constantDecelWindowMm(float aDecelMmS2, float axisCmdCountsPerS,
                             float cpm) {
  const float axisCruiseMmS = std::fabs(axisCmdCountsPerS) / cpm;
  return (axisCruiseMmS * axisCruiseMmS) / (2.0f * aDecelMmS2);
}

// Small multiplicative safety margin applied to constantDecelWindowMm()
// above before it gates the dist axis's shaped-mode branch: at the
// EXACT boundary the two branches already agree (v_allow there equals
// the axis's own full-rate speed, so the derived scale is 1.0, same as
// the outside-the-window branch), so this margin is not needed for
// continuity -- it exists so a caller ticking slower than the braking
// curve's own timescale, and so sampling `remain` only once per
// control period, engages the solve slightly before the exact
// threshold rather than exactly at it, absorbing up to one period's
// worth of travel without changing what the solve computes once
// engaged.
constexpr float kBrakingWindowMargin = 1.10f;

}  // namespace

MotionEngine::MotionEngine(DiffDrive::DifferentialDrive& kernel,
                           const DiffDrive::Clock& clock)
    : kernel_(kernel), clock_(clock) {}

uint32_t MotionEngine::nowMs() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000ull);
}

// See motion_engine.h's own comment on this method for the contract.
// `d` is clamped to >= 0 first: `distanceMm` can arrive negative (a
// caller passing a signed wire-level distance without taking its own
// magnitude first), and a negative product under sqrt() would be NaN.
float MotionEngine::defaultCruiseForDistance(float distanceMm) const {
  const float d = distanceMm > 0.0f ? distanceMm : 0.0f;
  const float vAllow = std::sqrt(2.0f * aDecelMmS2_ * brakeFrac_ * d);
  return vAllow < vMaxMmS_ ? vAllow : vMaxMmS_;
}

// See motion_engine.h's own comment on this method for the contract.
//
// A trapezoid over distance D holding v for T seconds satisfies
//   D = v^2/(2*aAccel) + v*T + v^2/(2*aDecel)
// which rearranges to the quadratic
//   (1/2)(1/aAccel + 1/aDecel) v^2 + T*v - D = 0
// whose positive root is the largest cruise that still leaves a plateau
// of T. With T == 0 this degenerates to the familiar triangular limit
// sqrt(2*D*aAccel*aDecel/(aAccel+aDecel)) -- a plateau of exactly zero,
// i.e. still a corner -- which is why plateauMinS_ is the useful knob
// rather than the triangular speed itself.
float MotionEngine::plateauCruiseMmS(float distanceMm) const {
  if (aAccelMmS2_ <= 0.0f || aDecelMmS2_ <= 0.0f) return 0.0f;
  if (plateauMinS_ <= 0.0f) return 0.0f;
  const float d = distanceMm > 0.0f ? distanceMm : 0.0f;
  const float k = 0.5f * (1.0f / aAccelMmS2_ + 1.0f / aDecelMmS2_);
  const float t = plateauMinS_;
  const float disc = t * t + 4.0f * k * d;
  if (disc <= 0.0f) return 0.0f;
  const float root = (-t + std::sqrt(disc)) / (2.0f * k);
  return root > 0.0f ? root : 0.0f;
}

// See motion_engine.h's own comment on this method for the contract.
float MotionEngine::yawRateCapMmS() const {
  if (maxYawRateDegS_ <= 0.0f) return 0.0f;
  const float omega = maxYawRateDegS_ * 3.14159265f / 180.0f;  // [rad/s]
  return omega * effectiveTrackWidth() * 0.5f;                 // [mm/s]
}

// See motion_engine.h's own comment on this method for the contract.
float MotionEngine::dominantAxisTravelMm(float distanceMm,
                                         float rotationRad) const {
  const float distTravel = std::fabs(distanceMm);
  const float yawTravel = std::fabs(rotationRad) * effectiveTrackWidth() * 0.5f;
  return yawTravel > distTravel ? yawTravel : distTravel;
}

void MotionEngine::wheelsV(float left, float right, uint32_t durationMs) {
  cancelMove();  // motion-api.md S6: wheels_* clears the planner
  const float cpm = countsPerMm();
  const float velocity = 0.5f * (left + right) * cpm;  // [counts/s]
  const float twist = 0.5f * (right - left) * cpm;     // [counts/s] CCW+
  kernel_.drive(velocity, twist, durationMs);
}

void MotionEngine::wheelsX(float left, float right, float cruise,
                           uint32_t timeoutMs) {
  cancelMove();  // motion-api.md S6: wheels_* clears the planner
  const float absLeft = std::fabs(left);
  const float absRight = std::fabs(right);
  const float dominant = absLeft > absRight ? absLeft : absRight;
  if (dominant <= 0.0f || cruise <= 0.0f) {
    // Nothing NEW to command -- but a previous wheelsV()/wheelsX() hold
    // may still be driving on its own lease (cancelMove(), above, only
    // clears the move engine's own bookkeeping and never touches the
    // kernel). Stop it unconditionally, not gated on move_.active:
    // wheelsV() never sets that flag, so a gated stop here would miss
    // exactly the case this exists for.
    kernel_.neutral();
    return;
  }

  // Normalize so the dominant wheel's own ratio is exactly +-1
  // (motion-api.md S4's control block: "uLeft, uRight normalized so
  // max(|uLeft|, |uRight|) == 1"), then scale by cruise -- the DOMINANT
  // wheel's own ceiling (S3.1) -- to get each wheel's commanded speed.
  const float uLeft = left / dominant;
  const float uRight = right / dominant;
  const float leftSpeed = uLeft * cruise;    // [mm/s]
  const float rightSpeed = uRight * cruise;  // [mm/s]

  const float cpm = countsPerMm();
  const float velocity = 0.5f * (leftSpeed + rightSpeed) * cpm;  // [counts/s]
  const float twist = 0.5f * (rightSpeed - leftSpeed) * cpm;     // [counts/s]

  // Dead-reckoned lease: how long the dominant wheel takes to cover its
  // own commanded distance at the ratio-locked cruise ceiling, capped by
  // the required timeout backstop (motion-api.md S3.1: "timeout is a
  // required backstop, not the stop condition" -- the live
  // encoder-progress check that makes this genuinely closed-loop is
  // moveX()'s own shaping layer, not this primitive).
  const float computedMs = (dominant / cruise) * 1000.0f;
  uint32_t lease = static_cast<uint32_t>(std::lround(computedMs));
  if (timeoutMs > 0 && timeoutMs < lease) lease = timeoutMs;

  kernel_.drive(velocity, twist, lease);
}

// ---- move engine (motion-api.md S3.3-S3.5) -------

void MotionEngine::cancelMove() {
  move_.active = false;
  move_.hasPending = false;
  // A caller (wheelsX()/wheelsV()) can cancel a split move mid-handoff --
  // after phase 1 finished and staged kernel_.neutral() but before the
  // NEXT serviceMove() call actually starts phase 2 (see
  // move_.awaitingHandoffNeutral's own comment). active becomes false
  // here, so a stale true would sit inert until some LATER moveX() sets
  // active back to true for an unrelated new move -- at which point
  // serviceMove()'s handoff check would fire on stale pendingDistance/
  // pendingCruise and silently skip that new move's own phase 1. Clearing
  // it here, alongside hasPending, is what keeps that from happening.
  move_.awaitingHandoffNeutral = false;
}

void MotionEngine::endMove() {
  if (move_.active) kernel_.neutral();
  cancelMove();
}

void MotionEngine::startSegment(float distance, float rotation,
                                float cruise) {
  const float cpm = countsPerMm();
  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  move_.posLeft0 = out.positionLeft;
  move_.posRight0 = out.positionRight;
  move_.epochLeft0 = out.positionEpochLeft;
  move_.epochRight0 = out.positionEpochRight;
  move_.distTarget = distance * cpm;                              // [counts]
  move_.yawTarget = rotation * 0.5f * effectiveTrackWidth() * cpm;  // [counts]
  // Subtract the measured per-wheel end-of-move overrun from the
  // rotation target (pivotOverrunMm_, motion_engine.h): a constant the
  // controller lands PAST every rotation, whatever its size, so it is
  // taken off the target's magnitude, never scaled. Clamped at zero: a
  // rotation smaller than the overrun itself becomes no rotation rather
  // than one in the wrong direction.
  // PROFILE-COMPLETION EXIT: aim past by exactly the distance the exit
  // gives up. serviceMove() ends the move once the commanded speed
  // decays to profileExitMmS_, which by the same kinematics the profile
  // is built on leaves v_exit^2/(2*aDecel) still to travel -- a fixed,
  // known shortfall, not an error. Extending each axis's target by it
  // means the move still lands where it was asked to; without this the
  // shortfall is charged to every single move, so a figure made of many
  // moves accumulates it (MEASURED vevov 2026-09-02: a 4-leg square
  // barely noticed, while an 8-arc circle lost 12 deg of rotation and a
  // 16-arc figure-8's closure went 85.6 -> 128.9 mm).
  if (profileExitMmS_ > 0.0f && aDecelMmS2_ > 0.0f) {
    const float stopCounts =
        (profileExitMmS_ * profileExitMmS_) / (2.0f * aDecelMmS2_) * cpm;
    if (move_.distTarget != 0.0f) {
      move_.distTarget += move_.distTarget > 0.0f ? stopCounts : -stopCounts;
    }
    if (move_.yawTarget != 0.0f) {
      move_.yawTarget += move_.yawTarget > 0.0f ? stopCounts : -stopCounts;
    }
  }

  if (move_.yawTarget != 0.0f && pivotOverrunMm_ > 0.0f) {
    const float overrun = pivotOverrunMm_ * cpm;  // [counts]
    const float mag = std::fabs(move_.yawTarget) - overrun;
    move_.yawTarget = mag > 0.0f
        ? (move_.yawTarget > 0.0f ? mag : -mag)
        : 0.0f;
  }

  // wheels_x's own reduction (motion-api.md S2): left = distance -
  // rotation*b/2, right = distance + rotation*b/2 -- restated here in
  // counts as mean +- half-differential, which is exactly distTarget/
  // yawTarget above (mean(left,right) == distance, halfDiff(right,left)
  // == rotation*b/2).
  const float left = move_.distTarget - move_.yawTarget;
  const float right = move_.distTarget + move_.yawTarget;
  const float absLeft = std::fabs(left);
  const float absRight = std::fabs(right);
  const float dominant = absLeft > absRight ? absLeft : absRight;
  if (dominant <= 0.0f || cruise <= 0.0f) {
    // Nothing NEW to command -- same contract as wheelsX()'s own
    // degenerate branch, including the stop: moveX()'s single-segment
    // path reaches here WITHOUT ever calling cancelMove() first, so a
    // prior wheelsV()/wheelsX() drive still holding its own lease would
    // otherwise survive untouched. kernel_.neutral() runs unconditionally
    // here, not gated on move_.active, for the same reason wheelsX()'s
    // own stop is unconditional.
    move_.active = false;
    kernel_.neutral();
    return;
  }

  // Shape the requested cruise before it becomes wheel commands. Two
  // independent clamps, both lowering-only so an explicit request is
  // never raised:
  //
  //  1. Turn-rate cap. The wire's cruise is linear mm/s, so a pure turn
  //     silently inherits the straight-line speed. MEASURED gopiv
  //     2026-09-01, captures/gopiv-profile-sweep-20260901/square120.json:
  //     cruise 300 mm/s turned at 254-285 deg/s.
  //  2. Plateau derate. A move commanded above its own triangular limit
  //     peaks for a single tick with a corner at the apex (same capture:
  //     every pivot held its peak for exactly one telemetry sample).
  //     plateauCruiseMmS() solves for the largest cruise that still
  //     leaves plateauMinS_ of flat top, so there is something for the
  //     jerk limiter to round without the two roundings colliding.
  float shapedCruise = cruise;
  const bool pureTurnSeg =
      (move_.yawTarget != 0.0f && move_.distTarget == 0.0f);
  if (pureTurnSeg) {
    const float cap = yawRateCapMmS();
    if (cap > 0.0f && cap < shapedCruise) shapedCruise = cap;
  }
  const float dominantMm = dominant / cpm;  // [mm] this segment's own
                                            // dominant-wheel travel
  const float plateau = plateauCruiseMmS(dominantMm);
  if (plateau > 0.0f && plateau < shapedCruise) shapedCruise = plateau;
  cruise = shapedCruise;

  const float cruiseCounts = cruise * cpm;  // [counts/s]
  move_.velCmd = move_.distTarget / dominant * cruiseCounts;
  move_.twistCmd = move_.yawTarget / dominant * cruiseCounts;
  move_.cruiseMmS = cruise;  // the accel integrator's own reference
                             // speed (serviceMove()'s ramp block);
                             // unused in legacy mode.

  // Acceleration ramp (stakeholder, 2026-08-20): start at the floor
  // rate, not a full-rate step -- serviceMove() raises the scale over
  // rampMs_. Mirrors the end-of-move taper; effective accel ~= full
  // rate / 0.4 s (~375 mm/s^2 at 15 cm/s), reference-shaper-like.
  move_.startMs = nowMs();
  move_.lastTickMs = move_.startMs;  // dt anchor for the accel
                                     // integrator.
  // In LEGACY mode (aAccelMmS2_ == 0.0f), the first tick starts at the
  // original, undocumented 0.25f literal -- unchanged. In SHAPED mode,
  // that literal is removed: the first tick starts at the same
  // distFloor_/turnFloor_ floor serviceMove()'s own taper uses (a pure
  // turn's own floor is turnFloor_, everything else is distFloor_), so
  // the velocity-slew integrator ramps up from a floor already proven
  // safe rather than from an unrelated hardcoded fraction.
  const bool pureTurnAtStart =
      (move_.yawTarget != 0.0f && move_.distTarget == 0.0f);
  const float initialScale = aAccelMmS2_ > 0.0f
      ? (pureTurnAtStart ? turnFloor_ : distFloor_)
      : 0.25f;
  move_.cmdScale = initialScale;
  move_.accelScalePerS = 0.0f;  // jerk limiter starts from rest
  const uint32_t now = move_.startMs;
  const uint32_t remainingMs =
      static_cast<int32_t>(move_.deadline - now) > 0
          ? (move_.deadline - now)
          : 0u;
  // A refused drive() (kRefusedUnconfigured/kRefusedNotBegun/
  // kRefusedEstopped/kRefusedNonFinite) must not arm move_.active --
  // otherwise this move reports progress, spins to its own deadline, and
  // resolves as kStop on the wire, indistinguishable from a move that
  // actually ran.
  const DiffDrive::DifferentialDrive::Status driveStatus = kernel_.drive(
      move_.velCmd * initialScale, move_.twistCmd * initialScale,
      remainingMs);
  move_.active = (driveStatus == DiffDrive::DifferentialDrive::Status::kOk);
}

void MotionEngine::queuePivotThenStraight(float pivotRotation,
                                          float straightDistance,
                                          float cruise) {
  move_.hasPending = true;
  move_.pendingDistance = straightDistance;
  move_.pendingCruise = cruise;
  startSegment(0.0f, pivotRotation, cruise);
}

void MotionEngine::moveX(float distance, float rotation, float cruise,
                         uint32_t timeoutMs) {
  // One deadline for the WHOLE call (both phases, if the pivot-first
  // split below fires) -- the wire's own single `timeout` field, a REAL
  // backstop distinct from any internally-computed lease (see
  // startSegment()'s own comment on the initial kernel_.drive() lease).
  move_.deadline = nowMs() + timeoutMs;
  move_.hasPending = false;
  // Starting a fresh move must not inherit a stale mid-handoff wait left
  // over from a PREVIOUS split move that was cancelled between its own
  // phase 1 and phase 2 (see cancelMove()'s own comment) -- otherwise
  // serviceMove()'s handoff check would fire immediately on this move's
  // very first tick, using leftover pendingDistance/pendingCruise, and
  // skip this move's own phase 1 entirely.
  move_.awaitingHandoffNeutral = false;

  // motion-api.md S3.3's measured table: a rotation this large combined
  // with an actual translation is NOT one blended segment -- pivot to
  // the new heading first (distance == 0 here), then travel the
  // remainder straight (rotation == 0 there). A pure pivot (distance ==
  // 0 already) or a rotation under the threshold stays one segment --
  // the degenerate cases motion-api.md S2.1 calls out (move_x(d,0)
  // straight, move_x(0,theta) pivot) are both single-segment already.
  if (distance != 0.0f && std::fabs(rotation) >= kTurnFirstAngleRad) {
    queuePivotThenStraight(rotation, distance, cruise);
  } else {
    startSegment(distance, rotation, cruise);
  }
}

void MotionEngine::moveV(float vx, float omega, uint32_t durationMs) {
  // motion-api.md S2: move_v(v_x, omega) == wheels_v(v_x - omega*b/2,
  // v_x + omega*b/2). wheelsV() itself clears any in-flight moveX().
  const float twist = omega * 0.5f * effectiveTrackWidth();  // [mm/s] CCW+
  wheelsV(vx - twist, vx + twist, durationMs);
}

void MotionEngine::goToR(float x, float y, float speed, float arrive,
                         uint32_t timeoutMs) {
  // KERN-04: `arrive` is now a radial no-op gate, checked ahead of any
  // split decision -- being within `arrive` [mm] of the target
  // (including exactly at it, since hypot(0,0) == 0 for any arrive >=
  // 0) issues no segment at all. This replaces the old exact-float-
  // equality guard (`x == 0.0f && y == 0.0f`), which a measured pose
  // (goToW() subtracts two live reads) could essentially never satisfy
  // -- still single-shot, no supervisory re-solve: a caller wanting
  // repeat-until-arrival re-issues goToR() itself (see header comment).
  if (std::hypot(x, y) <= arrive) return;

  // motion-api.md S3.5's arc-angle formula: bearingRaw = atan2(y,x) is
  // the line-of-sight direction to the target, always already bounded
  // to (-pi, pi]; thetaRaw = 2*bearingRaw is the constant-curvature
  // arc's own turn angle, which is NOT similarly bounded -- doubling can
  // land up to just under +-2*pi. Wrap it to the short arc, (-pi, pi],
  // BEFORE deciding anything else (KERN-03): the wrapped and unwrapped
  // values reach the same (x, y) on the same circle (it's periodic),
  // but only the short one is a sane distance to drive, and only the
  // short one may correctly decide the split below.
  const float bearingRaw = std::atan2(y, x);  // [rad] signed, |.| <= pi
  const float thetaRaw = 2.0f * bearingRaw;   // [rad] signed, |.| < 2*pi
  const float theta = wrapToPi(thetaRaw);     // [rad] signed, |.| <= pi

  if (std::fabs(theta) >= kTurnFirstAngleRad) {
    // KERN-02: goToR owns this split instead of inheriting moveX()'s
    // generic one. moveX()'s own pivot-first split would reissue
    // theta/arc-length as pivot-then-straight, which lands at a
    // DIFFERENT endpoint than the blended arc (arc length != chord
    // length except in the limit) -- e.g. goToR(100, 100) would pivot
    // 90 deg then drive the 157.1 mm ARC length straight, landing at
    // (0, 157.1) instead of (100, 100), a 115 mm miss on a 141 mm hop.
    // Pivoting to the line-of-sight bearing (already short-arc by
    // construction -- atan2's own principal value) then driving the
    // straight-line chord instead reaches (x, y) exactly, no matter how
    // large the bearing is. One deadline for the WHOLE call (both
    // phases), set directly here -- exactly what moveX() itself would
    // do, but this call bypasses the public moveX() entirely so it can
    // force this split regardless of whether |bearingRaw| alone would
    // have crossed moveX()'s own threshold (see header comment).
    move_.deadline = nowMs() + timeoutMs;
    move_.hasPending = false;
    // See moveX()'s identical reset for why: a fresh move must not
    // inherit a stale mid-handoff wait from a cancelled previous one.
    move_.awaitingHandoffNeutral = false;
    const float chord = std::hypot(x, y);  // [mm] >= 0
    queuePivotThenStraight(bearingRaw, chord, speed);
  } else {
    // Plain arc reduction (motion-api.md S3.5), unchanged below
    // threshold except that `theta` here is the SHORT-ARC-normalized
    // value (see above), not the raw 2*atan2(y, x) -- restated (matching
    // this project's prior TypeScript-side startGoTo() implementation)
    // via the signed circle
    // radius R = (x^2+y^2)/(2y), s = R*theta, which is the same formula
    // algebraically as arc length = radius*angle and avoids a sin() near
    // theta == 0.
    float s;                          // [mm] signed arc length
    if (std::fabs(y) < 0.1f) {        // ~0.01 cm: call it straight
      s = x;
    } else {
      const float radius = (x * x + y * y) / (2.0f * y);  // [mm] signed
      s = radius * theta;
    }
    moveX(s, theta, speed, timeoutMs);
  }
}

void MotionEngine::goToW(const PoseSource& pose, float x, float y,
                         float speed, float arrive, uint32_t timeoutMs) {
  // motion-api.md S2/S3.6: "go_to_w(x, y) == read pose -> world-to-body
  // -> go_to_r". Read the pose ONCE, here, at call time -- goToW() takes
  // no supervisory re-solve any more than goToR() does (see that
  // method's own comment).
  const float dx = x - pose.x();
  const float dy = y - pose.y();
  const float heading = pose.heading();
  const float cosH = std::cos(heading);
  const float sinH = std::sin(heading);

  // World-to-body rotation by -heading, matching this file's CCW-
  // positive convention (header comment): body x (forward) is the world
  // delta projected onto the heading direction; body y (left) is the
  // world delta projected onto the direction 90 deg CCW from heading.
  // Sign/rotation-direction errors hide exactly here when heading is
  // both nonzero AND the position offset is nonzero -- see this file's
  // own test coverage (tests/host/test_motion_engine_gotow.py).
  const float bodyX = dx * cosH + dy * sinH;
  const float bodyY = -dx * sinH + dy * cosH;

  goToR(bodyX, bodyY, speed, arrive, timeoutMs);
}

bool MotionEngine::serviceMove() {
  if (!move_.active) return false;

  // Second half of the phase 1 -> phase 2 handoff (see the branch below
  // that sets this flag for the full mechanism). The PREVIOUS
  // serviceMove() call staged kernel_.neutral() and returned without
  // touching move_.distTarget/yawTarget -- the caller (tickDrive()/
  // updateMove()) then ran exactly one real kernel_.step(), which is what
  // actually delivers that neutral and disarms the kernel's twist-hold
  // reference. Only NOW, on this call, is it safe to stage phase 2's
  // kernel_.drive() (via startSegment()) -- staging it any earlier, in
  // the same tick as the neutral, would silently overwrite the staged
  // neutral before any step() ever consumed it. Return immediately,
  // before the taper/progress math below (which still reads phase 1's
  // now-stale distTarget/yawTarget and would otherwise fall through to
  // the "move complete" branch and end the whole call one phase early).
  if (move_.awaitingHandoffNeutral) {
    move_.awaitingHandoffNeutral = false;
    const float pendingDistance = move_.pendingDistance;
    const float pendingCruise = move_.pendingCruise;
    startSegment(pendingDistance, 0.0f, pendingCruise);
    return move_.active;
  }

  const DiffDrive::DifferentialDrive::Output out = kernel_.output();

  // REBASE RACE (sprint 028 ticket 002, MEASURED gopiv 2026-09-02,
  // captures/gopiv-acceptance-028-20260902/step_e_transcript.txt): a
  // `SET rebase 1` defers kernel.rebasePosition() to the kernel's own
  // NEXT step() (diffdrive.cpp's rebaseReq_/seenRebaseReq_ check runs
  // before that step()'s own command processing). When a MOVE_X is
  // issued immediately after a SUCCESSFUL rebase (no motion active in
  // between, so the busy gate never fires), this move's own
  // startSegment() captures posLeft0/posRight0 from the kernel's
  // STILL-OLD, pre-rebase Output -- and the very step() that first
  // drives this move (tickDrive(), triggered by this move going active)
  // is also the step() that finally honours the deferred rebase,
  // zeroing positionLeft/positionRight (and sampleLeft_/sampleRight_
  // wholesale) out from under that snapshot. Diffing the fresh,
  // near-zero post-rebase position against the stale pre-rebase
  // baseline below produces a huge signed delta that satisfies the
  // completion margin on literally this move's first serviced tick --
  // measured: `MOVE_X 0 -900 60 3000` delivered 11 of ~5160 commanded
  // centidegrees and reported itself complete.
  //
  // positionEpochLeft/Right (diffdrive.h) change ONLY alongside that
  // re-anchor -- the same signal shims.cpp's odomUpdate() already reads
  // to catch the equivalent discontinuity in the Rig-level x/y/heading
  // odometry it owns. A change since startSegment()'s own snapshot means
  // the wheel has not actually moved relative to the new zero (the
  // rebase and this move's own first tick land in the same real
  // instant, never further apart -- rebase is refused with err 10 while
  // any motion obligation is live, so it can only ever land at a move's
  // very start, never mid-flight) -- so re-anchoring the baseline to the
  // fresh position is the correct fix, not a workaround: distTarget/
  // yawTarget stay untouched RELATIVE displacements, only the reference
  // point they are measured from moves.
  if (out.positionEpochLeft != move_.epochLeft0 ||
      out.positionEpochRight != move_.epochRight0) {
    move_.posLeft0 = out.positionLeft;
    move_.posRight0 = out.positionRight;
    move_.epochLeft0 = out.positionEpochLeft;
    move_.epochRight0 = out.positionEpochRight;
  }

  const float dLeft = out.positionLeft - move_.posLeft0;    // [counts]
  const float dRight = out.positionRight - move_.posRight0;  // [counts]
  const float meanProgress = 0.5f * (dLeft + dRight);
  const float diffProgress = 0.5f * (dRight - dLeft);

  // End-of-move taper (2026-08-20, exact-turns work): approach the
  // target at a decreasing rate so the post-stop coast shrinks to noise.
  // One shared scale keeps an arc's velocity/twist ratio (curvature)
  // intact; the floor keeps the binding axis above the kernel's own
  // speed floor so the crawl still moves.
  const bool pureTurn =
      (move_.yawTarget != 0.0f && move_.distTarget == 0.0f);
  const float distMargin = 10.0f;                     // [counts]
  const float yawMargin = pureTurn ? 4.0f : 10.0f;     // [counts]
  const float kTaperFloor = pureTurn ? turnFloor_ : distFloor_;
  // Only read by the shaped (aDecelMmS2_ > 0) branches below; a dead
  // load in legacy mode, same as every other new field this ticket adds.
  const float cpm = countsPerMm();

  float scale = 1.0f;
  bool distDone = true;
  if (move_.distTarget != 0.0f) {
    const float remain =
        std::fabs(move_.distTarget) - std::fabs(meanProgress);
    distDone = remain <= distMargin;
    // SUC-001: aDecelMmS2_ > 0 selects the constant-a braking-speed
    // solve, gated by the kinematics themselves
    // (constantDecelWindowMm() above, `v_cmd^2/(2*aDecelMmS2_)`) rather
    // than by distTaper_ -- a fixed-counts window is smaller than that
    // kinematic one at any meaningful cruise, which left this solve
    // unreachable above roughly 200 mm/s (see that function's own
    // comment for the measured before/after). distTaper_ stays
    // authoritative only in legacy mode. aDecelMmS2_ == 0 is untouched:
    // same `remain/distTaper_` expression as before, importing none of
    // the new math.
    float axisScale;
    if (aDecelMmS2_ > 0.0f) {
      const float remainMm = remain > 0.0f ? remain / cpm : 0.0f;
      const float windowMm =
          constantDecelWindowMm(aDecelMmS2_, move_.velCmd, cpm);
      axisScale = remainMm <= windowMm * kBrakingWindowMargin
          ? constantDecelAxisScale(aDecelMmS2_, remain, move_.velCmd, cpm)
          : 1.0f;
    } else {
      axisScale = remain / distTaper_;
    }
    if (axisScale < scale) scale = axisScale;
  }
  bool yawDone = true;
  bool wrongWay = false;
  if (move_.yawTarget != 0.0f) {
    // SIGNED progress toward the target, not |progress| -- comparing
    // magnitudes credits rotation in the WRONG DIRECTION as progress.
    const float toward =
        move_.yawTarget > 0.0f ? diffProgress : -diffProgress;
    const float remain = std::fabs(move_.yawTarget) - toward;
    yawDone = remain <= yawMargin;
    if (toward < -3.0f * yawMargin) wrongWay = true;
    // Only a PURE TURN tapers on yaw -- in an arc the twist and the
    // velocity are locked by curvature, so the distance taper already
    // scales yaw by the same factor; a second, independent yaw taper
    // double-counts (see motion_engine.h/shims.cpp history: measured
    // vevov 2026-08-22, three legs pinned at the 25% distFloor by this
    // exact double-count while the one leg under goToWorld's straight-
    // line threshold skipped this branch and ran full speed).
    if (pureTurn) {
      // Same constant-a/legacy split as the distance axis above, and
      // now the same kinematics-derived gate. This axis used to brake
      // on the fixed yawTaper_ counts window even in shaped mode, which
      // made yawTaper_ -- not the kinematics -- decide when a pivot
      // starts slowing. At the tuned 800 counts that is ~70% of a 90 deg
      // pivot's ~1141 counts of differential, so the pivot spent most of
      // its life tapering and then held the turnFloor_ crawl until the
      // 4-count completion margin closed.
      //
      // MEASURED vevov 2026-09-02, four 90 deg pivots per setting with
      // constant-a shaping on: the crawl tracked yawTaper_ and nothing
      // else -- 800 -> 375 ms per pivot, 400 -> 235, 100 -> 146 -- while
      // accuracy fell apart as the window shrank (excess +0.36 deg
      // sd 0.16 at 800, +4.97 deg sd 1.14 at 100). That trade only
      // existed because the brake point was a fixed count rather than
      // v^2/(2a): the kinematic window is short at low twist AND still
      // reaches zero exactly at the target, so it buys back the crawl
      // without paying in scatter. yawTaper_ stays authoritative in
      // legacy mode only, exactly as distTaper_ does.
      float axisScale;
      if (aDecelMmS2_ > 0.0f) {
        const float remainMm = remain > 0.0f ? remain / cpm : 0.0f;
        const float windowMm =
            constantDecelWindowMm(aDecelMmS2_, move_.twistCmd, cpm);
        axisScale = remainMm <= windowMm * kBrakingWindowMargin
            ? constantDecelAxisScale(aDecelMmS2_, remain, move_.twistCmd,
                                     cpm)
            : 1.0f;
      } else {
        axisScale = remain / yawTaper_;
      }
      if (axisScale < scale) scale = axisScale;
    }
  }
  // The taper alone, before any floor or ramp min-combine: this is the
  // profile's own opinion of how fast to be going, and it decays to
  // zero at the target. The floor below is what stops it landing there.
  const float taperScale = scale;

  // PROFILE-COMPLETION EXIT (profileExitMmS_ > 0, shaped mode only).
  // The floor exists to keep the command above stiction so a positional
  // exit can still close its margin; when the move ends on the profile
  // instead, that floor is exactly what prevents the glide. Bypass it
  // and let the command decay. Legacy behaviour is untouched: with
  // profileExitMmS_ == 0 this is the same single line it always was.
  const bool profileExitArmed =
      profileExitMmS_ > 0.0f && aDecelMmS2_ > 0.0f;
  if (!profileExitArmed && scale < kTaperFloor) scale = kTaperFloor;

  // Acceleration ramp, min-combined with the end taper (a very short
  // move may go straight from ramp to taper without ever reaching
  // full) -- exactly as before, unless shaped mode is selected. SUC-002:
  // aAccelMmS2_ > 0 replaces the time-based `elapsed/rampMs_` fraction
  // with a velocity-slew integrator, `v_cmd <= v_prev + aAccelMmS2_*dt`,
  // expressed as a scale of move_.cruiseMmS_ so it min-combines with the
  // taper scale the same way the legacy fraction did. `v_prev` is
  // move_.cmdScale from the PREVIOUS tick -- the ACTUAL last commanded
  // scale (already taper/floor-limited by that tick's own min-combine
  // below), not a re-derivation from elapsed time -- so a taper-limited
  // segment's accel ramp picks up from where the taper actually left it,
  // and so that changing aDecelMmS2_ alone never perturbs this
  // integrator's own math (it never reads distTaper_/yawTaper_/rampMs_
  // at all). aAccelMmS2_ == 0 is untouched: same `elapsed/rampMs_`
  // expression as before, importing none of the new math.
  const uint32_t nowMsRamp = nowMs();
  float ramp;
  if (aAccelMmS2_ > 0.0f) {
    const float dtS =
        static_cast<float>(nowMsRamp - move_.lastTickMs) / 1000.0f;
    const float cruiseMmS = move_.cruiseMmS > 0.0f ? move_.cruiseMmS : 1.0f;
    ramp = move_.cmdScale + (aAccelMmS2_ * dtS) / cruiseMmS;
  } else {
    ramp = static_cast<float>(nowMsRamp - move_.startMs) / rampMs_;
  }
  if (ramp < kTaperFloor) ramp = kTaperFloor;
  if (ramp < scale) scale = ramp;
  if (scale > 1.0f) scale = 1.0f;

  // Jerk limiter (second-order shaper). The first-order limiter above
  // bounds acceleration but still STEPS it -- at the accel->decel
  // handover the commanded acceleration jumps straight from +aAccel to
  // -aDecel in one tick, which is the corner at the apex of every
  // short move. Bounding da/dt rounds every such corner without having
  // to locate it.
  //
  // The subtlety is that a jerk-limited controller cannot stop
  // accelerating instantly, so it overshoots a speed cap unless it
  // anticipates: while ramping `a` down to zero at rate `j`, velocity
  // still gains a^2/(2j). Easing off once cmdScale + that gain reaches
  // the target holds the cap exactly (simulated overshoot without it:
  // 113 deg/s against a 90 deg/s cap --
  // captures/gopiv-profile-sweep-20260901/).
  if (jerkMmS3_ > 0.0f && aAccelMmS2_ > 0.0f) {
    const float cruiseRef = move_.cruiseMmS > 0.0f ? move_.cruiseMmS : 1.0f;
    const float dtJ =
        static_cast<float>(nowMsRamp - move_.lastTickMs) / 1000.0f;
    if (dtJ > 0.0f) {
      const float jScale = jerkMmS3_ / cruiseRef;    // [scale/s^2]
      const float aMax = aAccelMmS2_ / cruiseRef;    // [scale/s]
      const float decelRef =
          aDecelMmS2_ > 0.0f ? aDecelMmS2_ : aAccelMmS2_;
      const float aMin = -decelRef / cruiseRef;      // [scale/s]
      float a = move_.accelScalePerS;
      const float gain = a > 0.0f ? (a * a) / (2.0f * jScale) : 0.0f;
      float aWant;
      if (move_.cmdScale + gain >= scale) {
        aWant = move_.cmdScale > scale ? aMin : 0.0f;
      } else {
        aWant = aMax;
      }
      const float dA = jScale * dtJ;
      const float err = aWant - a;
      if (err > dA) {
        a += dA;
      } else if (err < -dA) {
        a -= dA;
      } else {
        a = aWant;
      }
      if (a > aMax) a = aMax;
      if (a < aMin) a = aMin;
      float shaped = move_.cmdScale + a * dtJ;
      if (shaped < 0.0f) shaped = 0.0f;
      if (shaped > scale) shaped = scale;  // never exceed the taper /
                                           // cap the min-combine set
      move_.accelScalePerS = a;
      scale = shaped;
      if (scale < kTaperFloor) scale = kTaperFloor;
    }
  }

  move_.lastTickMs = nowMsRamp;  // dt anchor for the NEXT tick; a dead
                                 // store in legacy mode.

  // Reissue EVERY tick while the move is active, at the current scale
  // with a rolling 500 ms lease -- the only form that is lease-safe:
  // gating reissues on scale CHANGE would let the lease expire during
  // any steady phase (floor crawl, or full rate after the ramp
  // completes). The kernel's lease backstop still covers an abandoned
  // move within 500 ms; the REAL timeout backstop is move_.deadline,
  // checked below, independent of this rolling lease.
  // Has the profile run out? The dominant axis's commanded speed is the
  // taper's own scale applied to that axis's full-rate command; once it
  // falls to profileExitMmS_ the planned deceleration has landed and
  // anything further is chasing an encoder-frame residual.
  bool profileDone = false;
  if (profileExitArmed) {
    const float axisCountsPerS =
        std::fabs(move_.velCmd) > std::fabs(move_.twistCmd)
            ? std::fabs(move_.velCmd)
            : std::fabs(move_.twistCmd);
    const float cmdMmS = (taperScale * axisCountsPerS) / cpm;
    profileDone = cmdMmS <= profileExitMmS_;
  }

  // One notion of "arrived", shared by the drive gate, the phase-1 ->
  // phase-2 handoff and the terminator below -- they must never
  // disagree about whether the move is over.
  const bool reached = (distDone && yawDone) || profileDone;

  if (!reached) {
    move_.cmdScale = scale;
    kernel_.drive(move_.velCmd * scale, move_.twistCmd * scale, 500u);
  }

  const uint32_t now = nowMs();
  const bool expired = static_cast<int32_t>(now - move_.deadline) >= 0;

  // Phase 1 (pivot) finished CLEANLY -- motion-api.md S3.3's pivot-then-
  // travel split (moveX()) queued a second (straight) phase. Timeout/
  // stall/wrong-way abort the WHOLE moveX() call instead of advancing --
  // a blocked or misbehaving pivot should not be followed by a straight
  // leg run blind.
  if (reached && !expired && !out.stallHalted && !wrongWay &&
      move_.hasPending) {
    // DifferentialDrive's twist-hold servo (diffdrive.cpp, vendored)
    // keeps an integrated reference of commanded differential and trims
    // the wheels toward it every velocity-mode step; that reference is
    // disarmed in exactly two kernel modes -- neutral and raw-duty -- and
    // NOT by a velocity-mode drive() call, no matter how much the
    // commanded twist changes. Going straight from this pivot's
    // startSegment() call into phase 2's startSegment() (both stage
    // kernel_.drive()) would therefore leave the reference armed with
    // phase 1's PRE-PIVOT origin and its fully-accumulated pivot value;
    // phase 2 commands twist = 0, so the reference stops growing but the
    // measured twist position still carries the whole pivot, and the
    // servo actively drives the wheels to unwind it -- fast, right at
    // this transition, with the robot essentially in place (measured on
    // hardware: a pivot that peaked past its commanded angle then lost
    // ~17 degrees of it before the straight leg ever moved). Calling
    // kernel_.neutral() here reproduces the state a real gap between two
    // separate commands already proves is correct -- it disarms the
    // reference so phase 2 re-arms fresh, at the post-pivot origin, with
    // zero accumulated error.
    //
    // kernel_.neutral() only STAGES the neutral command, though --
    // delivery (and the disarm above) happens on the kernel's NEXT
    // step(), which this class never calls itself (see class header
    // comment: step() is always the caller's, once per tick, before
    // serviceMove()). Calling startSegment() -- which stages
    // kernel_.drive() -- in this SAME call would overwrite the staged
    // neutral before any step() ever saw it, silently reproducing the
    // exact bug this exists to fix. So phase 2 does not start here:
    // move_.awaitingHandoffNeutral defers startSegment() to the NEXT
    // serviceMove() call, by which point the caller's own step() has run
    // and the disarm has actually happened.
    move_.hasPending = false;
    kernel_.neutral();
    move_.awaitingHandoffNeutral = true;
    return move_.active;
  }

  // out.estopped joins stallHalted/expired/wrongWay here: it is the same
  // kind of published, latched refusal (the kernel forces neutral under
  // the e-stop latch, same as it does under the stall latch) -- without
  // it, isMoveActive() stays true and every `while (driveTick())` loop
  // spins to the deadline even though the wheels are already safe
  // (measured: 1230 further ticks / 29.5 s after the latch, on a 30 s
  // move). This was previously masked only by shims.cpp's estopAll()
  // calling engine.endMove() BEFORE kernel.estop() -- an ordering this
  // class must not depend on, since kernel.emergencyStopMotors() latches
  // the e-stop as a side effect that bypasses that ordering entirely.
  if (reached || expired || out.stallHalted || wrongWay ||
      out.estopped) {
    if (wrongWay) ++wrongWayCount_;
    kernel_.neutral();
    move_.active = false;
    move_.hasPending = false;
    return false;
  }
  return true;
}

void MotionEngine::settleToRest() {
  // Sprint 008 ticket 004: extracted verbatim from shims.cpp::
  // tickDrive()'s former inline loop -- see motion_engine.h's own
  // comment on this method for the full contract and the bench history
  // (commit 3e919e5) it guards. Behavior is identical to the loop it
  // replaces, not merely similar: same bound, same threshold, same
  // break condition, no new command ever issued.
  for (int i = 0; i < kSettleMaxSteps; ++i) {
    kernel_.step();
    const DiffDrive::DifferentialDrive::Output o = kernel_.output();
    if (o.velocityLeft < kSettleRestCountsPerS &&
        o.velocityLeft > -kSettleRestCountsPerS &&
        o.velocityRight < kSettleRestCountsPerS &&
        o.velocityRight > -kSettleRestCountsPerS) {
      break;
    }
  }
}

int MotionEngine::progress() const {
  if (!move_.active) return 1000;
  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  // Same rebase-race guard as serviceMove()'s own comment at its
  // equivalent read site: a query landing in the single-tick window
  // between a move's startSegment() snapshot and the deferred rebase's
  // step()-time re-anchor would otherwise diff a fresh position against
  // a stale pre-rebase baseline. This method is const -- it cannot
  // perform the real re-anchor serviceMove() does -- so it reports "no
  // progress yet" for that one tick instead of a bogus huge delta;
  // serviceMove() itself re-anchors move_ on its own next call.
  const bool rebasedSinceStart = out.positionEpochLeft != move_.epochLeft0 ||
      out.positionEpochRight != move_.epochRight0;
  const float dLeft =
      rebasedSinceStart ? 0.0f : out.positionLeft - move_.posLeft0;
  const float dRight =
      rebasedSinceStart ? 0.0f : out.positionRight - move_.posRight0;
  float fraction = 1.0f;
  if (move_.distTarget != 0.0f) {
    const float f = std::fabs(0.5f * (dLeft + dRight)) /
                    std::fabs(move_.distTarget);
    if (f < fraction) fraction = f;
  }
  if (move_.yawTarget != 0.0f) {
    const float f = std::fabs(0.5f * (dRight - dLeft)) /
                    std::fabs(move_.yawTarget);
    if (f < fraction) fraction = f;
  }
  if (fraction < 0.0f) fraction = 0.0f;
  if (fraction > 1.0f) fraction = 1.0f;
  return static_cast<int>(fraction * 1000.0f);
}

}  // namespace diffDrive
