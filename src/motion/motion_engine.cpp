// motion_engine.cpp -- see motion_engine.h for the class contract, the
// two-primitive design (motion-api.md S2), the geometry rationale
// (S2.1), and the move engine (S3.3-S3.5). Host-portable: this file
// includes nothing but <cmath>/<limits> and its own header -- no pxt.h,
// no CODAL type.
#include "motion_engine.h"

#include <cmath>
#include <limits>

namespace diffDrive {

namespace {

// [rad] -- literal, matching this project's existing convention (see
// e.g. otos_port.h/shims.cpp) rather than relying on <cmath>'s
// non-standard M_PI.
constexpr float kPi = 3.14159265f;

// Wrap an angle to the short arc, (-pi, pi].
// goToR()'s arc-angle formula (`theta = 2*atan2(y, x)`) doubles atan2's
// own principal value, so it can land up to just under +-2*pi even
// though atan2(y,x) itself never exceeds +-pi -- that "long way around"
// value reaches the same (x, y) on the same constant-curvature circle
// as the short, wrapped angle (the circle is periodic), but only the
// short one is a sane distance to actually drive. The input here is
// always bounded to (-2*pi, 2*pi] (twice an atan2 result), so a single
// conditional wrap suffices -- no loop, no fmod, needed.
float wrapToPi(float angle) {  // [rad]
  if (angle > kPi) return angle - 2.0f * kPi;
  if (angle <= -kPi) return angle + 2.0f * kPi;
  return angle;
}

}  // namespace

MotionEngine::MotionEngine(DiffDrive::DifferentialDrive& kernel,
                           const DiffDrive::Clock& clock)
    : kernel_(kernel), clock_(clock) {}

uint32_t MotionEngine::now() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000ull);
}

// See motion_engine.h's own comment on this method for the contract and
// design motion-profile-unification.md S8's formula.
float MotionEngine::defaultCruiseForDistance(float distance) const {
  const float d = distance > 0.0f ? distance : 0.0f;
  const float vAllow = std::sqrt(limits_.decel * d);
  return vAllow < limits_.vMax ? vAllow : limits_.vMax;
}

// See motion_engine.h's own comment on this method for the contract.
float MotionEngine::dominantAxisTravel(float distance,
                                       float rotation) const {
  const float distTravel = std::fabs(distance);
  const float yawTravel = std::fabs(rotation) * effectiveTrackWidth() * 0.5f;
  return yawTravel > distTravel ? yawTravel : distTravel;
}

// design S6.2: converts this segment's own axis into the dominant-wheel
// [mm/s] floor/cap. See motion_engine.h's own comment on this method.
MotionEngine::AxisLimits MotionEngine::axisLimits(const Segment& seg) const {
  const float b = effectiveTrackWidth();
  const float kInfinity = std::numeric_limits<float>::infinity();
  if (seg.pureTurn()) {
    const float floor = limits_.omegaFloorAsWheelSpeed(b);
    const float cap =
        limits_.omegaMax > 0.0f ? limits_.omegaMaxAsWheelSpeed(b) : kInfinity;
    return AxisLimits{floor, cap};
  }
  return AxisLimits{limits_.vFloor, kInfinity};
}

void MotionEngine::cancelMove() {
  seg_ = Segment();
  hold_.active = false;
}

void MotionEngine::wheelsV(float left, float right, uint32_t duration) {
  const bool wasSegActive = seg_.active;
  const bool wasHoldActive = hold_.active;
  cancelMove();  // motion-api.md S6: wheels_* clears the planner

  const float v = 0.5f * (left + right);      // [mm/s] target mean
  const float twist = 0.5f * (right - left);  // [mm/s] target half-diff
  const float leftSpeed = v - twist;
  const float rightSpeed = v + twist;
  const float absLeft = std::fabs(leftSpeed);
  const float absRight = std::fabs(rightSpeed);
  hold_.v = v;
  hold_.twist = twist;
  hold_.dominant = absLeft > absRight ? absLeft : absRight;
  hold_.until = now() + duration;
  hold_.active = true;

  // A genuinely NEW continuous-drive session (previously idle, or
  // superseding a position-mode Segment) restarts the shaper from the
  // floor, same as any fresh Segment (design S4.2: "at every segment
  // start"). A wheelsV() call that only UPDATES an already-live hold's
  // own target (e.g. a closed-loop steering correction re-issuing
  // setWheels() every cycle) does NOT reset -- the shaper keeps slewing
  // from whatever it is currently commanding toward the new target
  // instead of re-floor-starting on every call. Design left this exact
  // choice open (S4.2 only says "at every segment start"); see this
  // ticket's own report for the alternative considered (always reset)
  // and why it was rejected.
  if (wasSegActive || !wasHoldActive) {
    shaper_.reset();
    lastTick_ = now();
  }
}

// Builds seg_ from (distTarget, yawTarget, cruise,
// deadline) -- the shared tail of wheelsX()'s per-wheel reduction and
// moveX()'s distance/rotation reduction. See motion_engine.h's own
// comment on this method.
void MotionEngine::beginSegment(float distTarget, float yawTarget,
                                float cruise, uint32_t deadline) {
  cancelMove();  // motion-api.md S6: a new command supersedes any prior
                 // Segment/Hold, degenerate or not (see wheelsX()'s own
                 // doc comment: "not purely inert" -- it must stop a
                 // still-live wheelsV() hold too).

  const float left = distTarget - yawTarget;
  const float right = distTarget + yawTarget;
  const float absLeft = std::fabs(left);
  const float absRight = std::fabs(right);
  const float dominant = absLeft > absRight ? absLeft : absRight;
  if (dominant <= 0.0f || cruise <= 0.0f) {
    // Nothing NEW to command -- cancelMove() above already cleared any
    // prior state; unconditionally neutral the kernel too, same
    // degenerate contract wheelsX()/the old startSegment() always had.
    kernel_.neutral();
    return;
  }

  seg_.distTarget = distTarget;
  seg_.yawTarget = yawTarget;
  seg_.cruise = cruise;
  seg_.dominant = dominant;
  seg_.dominantAxis =
      seg_.pureTurn() ? Segment::Axis::kYaw : Segment::Axis::kDistance;
  seg_.originPending = true;
  seg_.deadline = deadline;
  seg_.active = true;

  shaper_.reset();  // design S4.2: v = 0, a = 0 at every segment start
  lastTick_ = now();
}

void MotionEngine::queuePivotThenStraight(float pivotRotation,
                                          float straightDistance,
                                          float cruise, uint32_t deadline) {
  const float cpm = countsPerMm();
  const float yawTarget =
      pivotRotation * 0.5f * effectiveTrackWidth() * cpm;
  beginSegment(0.0f, yawTarget, cruise, deadline);
  if (!seg_.active) return;  // degenerate pivot -- nothing to queue
  seg_.hasPending = true;
  seg_.pendingDistance = straightDistance;
  seg_.pendingCruise = cruise;
}

void MotionEngine::beginPendingStraightPhase() {
  const float distance = seg_.pendingDistance;
  const float cruise = seg_.pendingCruise;
  const uint32_t deadline = seg_.deadline;
  const float cpm = countsPerMm();
  beginSegment(distance * cpm, 0.0f, cruise, deadline);
}

void MotionEngine::wheelsX(float left, float right, float cruise,
                           uint32_t timeout) {
  const uint32_t deadline = now() + timeout;
  const float cpm = countsPerMm();
  const float distTarget = 0.5f * (left + right) * cpm;
  const float yawTarget = 0.5f * (right - left) * cpm;
  beginSegment(distTarget, yawTarget, cruise, deadline);
}

void MotionEngine::moveX(float distance, float rotation, float cruise,
                         uint32_t timeout) {
  // One deadline for the WHOLE call (both phases, if the pivot-first
  // split below fires) -- the wire's own single `timeout` field, a REAL
  // backstop.
  const uint32_t deadline = now() + timeout;

  // motion-api.md S3.3's measured table: a rotation this large combined
  // with an actual translation is NOT one blended segment -- pivot to
  // the new heading first (distance == 0 here), then travel the
  // remainder straight (rotation == 0 there). A pure pivot (distance ==
  // 0 already) or a rotation under the threshold stays one segment --
  // the degenerate cases motion-api.md S2.1 calls out (move_x(d,0)
  // straight, move_x(0,theta) pivot) are both single-segment already.
  if (distance != 0.0f && std::fabs(rotation) >= kTurnFirstAngle) {
    queuePivotThenStraight(rotation, distance, cruise, deadline);
  } else {
    const float cpm = countsPerMm();
    const float distTarget = distance * cpm;
    const float yawTarget = rotation * 0.5f * effectiveTrackWidth() * cpm;
    beginSegment(distTarget, yawTarget, cruise, deadline);
  }
}

void MotionEngine::moveV(float vx, float omega, uint32_t duration) {
  // motion-api.md S2: move_v(v_x, omega) == wheels_v(v_x - omega*b/2,
  // v_x + omega*b/2). wheelsV() itself clears any in-flight moveX().
  const float twist = omega * 0.5f * effectiveTrackWidth();  // [mm/s] CCW+
  wheelsV(vx - twist, vx + twist, duration);
}

void MotionEngine::goToR(float x, float y, float speed, float arrive,
                         uint32_t timeout) {
  // `arrive` is a radial no-op gate, checked ahead of any
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

  if (std::fabs(theta) >= kTurnFirstAngle) {
    // goToR owns this split instead of inheriting moveX()'s
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
    // phases). This call bypasses the public moveX() entirely so it can
    // force this split regardless of whether |bearingRaw| alone would
    // have crossed moveX()'s own threshold (see header comment).
    const uint32_t deadline = now() + timeout;
    const float chord = std::hypot(x, y);  // [mm] >= 0
    queuePivotThenStraight(bearingRaw, chord, speed, deadline);
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
    moveX(s, theta, speed, timeout);
  }
}

void MotionEngine::goToW(const PoseSource& pose, float x, float y,
                         float speed, float arrive, uint32_t timeout) {
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

  goToR(bodyX, bodyY, speed, arrive, timeout);
}

// The whole per-tick behaviour (design S5): segment-or-hold dispatch,
// no mode forks. See motion_engine.h's own comment on this method for
// what changed from the old serviceMove().
bool MotionEngine::service() {
  if (!seg_.active && !hold_.active) return false;

  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  const uint32_t nowVal = now();
  const float dt = static_cast<float>(nowVal - lastTick_) / 1000.0f;
  lastTick_ = nowVal;
  const float cpm = countsPerMm();

  if (seg_.active) {
    // 6.5: first tick after start() -- capture the origin from THIS
    // tick's already-published Output (the caller's own step() has
    // already run, and applied any deferred rebase, before service() is
    // ever called).
    if (seg_.originPending) {
      seg_.posLeft0 = out.positionLeft;
      seg_.posRight0 = out.positionRight;
      seg_.originPending = false;
    }

    // Clamp at 0: Segment::remaining() is a SIGNED "toward the target"
    // quantity and can land a hair negative on the exact tick the
    // target is reached or overshot (float rounding in the pivot/
    // distance arithmetic, or a real overshoot past the target by less
    // than one tick's travel) -- VelocityShaper::advance()'s own
    // `remain < 0` branch is reserved for the continuous-hold caller's
    // "no displacement bound" sentinel (design S6.1: "remain < 0 means
    // no displacement bound"), never for a Segment, which always has a
    // real, bounded target. Passing a barely-negative remain through
    // unclamped would silently flip advance() into that unbounded
    // branch -- skipping the floor and, more importantly, the
    // predictive-arrival test itself (`remain >= 0.0f && ...`) -- so a
    // segment landing exactly on (or a hair past) its target would
    // never be detected as arrived at all. MEASURED against this
    // engine (tests/host/test_motion_engine_reductions.py's
    // behind-robot goToR split cases): an exact-target arm computed
    // remain = -0.00001, which read arriving=false and kept driving
    // indefinitely until this clamp was added.
    float remain = seg_.remaining(out) / cpm;
    if (remain < 0.0f) remain = 0.0f;
    const AxisLimits al = axisLimits(seg_);
    float target = seg_.cruise;
    if (al.cap < target) target = al.cap;
    if (limits_.vMax < target) target = limits_.vMax;

    // design S6.1 step 0: the kernel's own last-measured
    // dominant-axis speed, on THIS segment's own dominant axis -- a
    // pure turn's dominant axis is the half-differential (mirrors
    // axisLimits()'s own pureTurn() branch and Segment::remaining()'s
    // own kYaw branch, both already scaled in "one wheel's own linear
    // speed" units); anything else (straight or blended arc) is the
    // mean of the two wheels. Sign-normalized toward the target via
    // fabs() -- a wheel briefly moving the WRONG way is already caught
    // by wrongWay() below, not by this measurement.
    const float vAct = seg_.dominantAxis == Segment::Axis::kYaw  // [mm/s]
        ? std::fabs(0.5f * (out.velocityRight - out.velocityLeft) / cpm)
        : std::fabs(0.5f * (out.velocityLeft + out.velocityRight) / cpm);
    const VelocityShaper::Step step =
        shaper_.advance(target, remain, al.floor, al.cap, dt, limits_, vAct);

    const bool wrongWay = seg_.wrongWay(out);
    const bool expired = static_cast<int32_t>(nowVal - seg_.deadline) >= 0;
    if (wrongWay || out.stallHalted || out.estopped || expired) {
      if (wrongWay) ++wrongWayCount_;
      kernel_.neutral();
      seg_ = Segment();
      return false;
    }

    if (step.arriving) {  // 6.3: the plan says "this is the last tick"
      kernel_.neutral();
      if (seg_.hasPending) {
        // 6.4: the pivot->straight handoff goes through rest. neutral()
        // above only STAGES the stop -- delivery (and the kernel's own
        // reference disarm) happens on the caller's NEXT step(). K4's
        // rearmReferences() disarms the twist-hold/position references
        // too, at the START of that same next step(), so phase 2
        // re-anchors fresh instead of carrying phase 1's accumulated
        // reference. beginPendingStraightPhase() builds phase 2 as a
        // brand-new Segment (originPending = true, shaper_ reset) but
        // issues no drive() of its own (S6.5's lazy start) -- the
        // FOLLOWING service() tick captures its origin and issues its
        // first command, by which point the neutral+rearm above has
        // actually landed.
        kernel_.rearmReferences();
        beginPendingStraightPhase();
        return seg_.active;
      }
      seg_ = Segment();
      return false;
    }

    // The (uLeft, uRight) ratio (motion-api.md S4) is implied by
    // (distTarget, yawTarget): recomputed from step.vCmd EVERY tick,
    // not stored as a full-rate pair scaled afterwards (design S4.3).
    const float velocity = (seg_.distTarget / seg_.dominant) * step.vCmd;
    const float twist = (seg_.yawTarget / seg_.dominant) * step.vCmd;
    const DiffDrive::DifferentialDrive::Status driveStatus =
        kernel_.drive(velocity * cpm, twist * cpm, 500u);
    // this ticket: beginSegment() issues NO drive() of its own
    // (design S6.5's lazy start), so a refused command (maxDuty == 0,
    // e-stopped, non-finite target, ...) can only ever be discovered
    // HERE, on the segment's own first (or any later) service() tick --
    // never at moveX()/wheelsX() call time the way the old synchronous
    // startSegment() could. Without this check a permanently-refused
    // drive() (e.g. an unconfigured kernel) would re-issue every tick,
    // never land, and spin the segment all the way to its own deadline
    // exactly like a move that actually ran -- the same defect this
    // ticket's own test_refused_drive_does_not_arm_move_active guards,
    // now detected one tick later than before instead of not at all.
    if (driveStatus != DiffDrive::DifferentialDrive::Status::kOk) {
      kernel_.neutral();
      seg_ = Segment();
      return false;
    }
    return true;
  }

  // continuous hold (wheelsV()/moveV()).
  const bool holdExpired = static_cast<int32_t>(nowVal - hold_.until) >= 0;
  if (holdExpired) {
    kernel_.neutral();
    hold_.active = false;
    return false;
  }
  // No single dominant axis to measure against a slewing
  // continuous-hold target -- the sentinel case (design S5/S6.1's own
  // "remain < 0 means no displacement bound" branch never uses vAct
  // for anything but the arrival test, which is unreachable here).
  const VelocityShaper::Step step = shaper_.advance(
      hold_.dominant, -1.0f, 0.0f, limits_.vMax, dt, limits_, -1.0f);
  const float scale = hold_.dominant > 0.0f ? (step.vCmd / hold_.dominant)
                                            : 0.0f;
  const float velocity = hold_.v * scale;
  const float twist = hold_.twist * scale;
  const DiffDrive::DifferentialDrive::Status holdDriveStatus =
      kernel_.drive(velocity * cpm, twist * cpm, 500u);
  if (holdDriveStatus != DiffDrive::DifferentialDrive::Status::kOk) {
    kernel_.neutral();
    hold_.active = false;
    return false;
  }
  return true;
}

void MotionEngine::endMove() {
  if (seg_.active || hold_.active) kernel_.neutral();
  seg_ = Segment();
  hold_ = Hold();
  shaper_.reset();
}

void MotionEngine::settleToRest() {
  // Extracted verbatim from shims.cpp::tickDrive()'s former inline
  // loop -- see motion_engine.h's own comment on this method for the
  // full contract and the bench history
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
  if (!seg_.active) return 1000;
  // Lazy origin capture (S6.5): before the first service() tick has
  // ever run for this segment, posLeft0/posRight0 have not been
  // captured yet -- reporting a fraction computed against that
  // uninitialized (0, 0) baseline would be nonsense, not merely stale.
  // Nothing has happened yet, so 0 is the honest answer.
  if (seg_.originPending) return 0;
  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  return static_cast<int>(seg_.progress(out) * 1000.0f);
}

}  // namespace diffDrive
