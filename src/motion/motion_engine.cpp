// See motion_engine.h for the class contract and DESIGN.md for rationale.
#include "motion_engine.h"

#include <cmath>
#include <limits>

namespace diffDrive {

namespace {

// [rad] literal, matching this project's convention rather than <cmath>'s
// non-standard M_PI.
constexpr float kPi = 3.14159265f;

// Wrap to the short arc, (-pi, pi]. The input is always twice an atan2
// result, so one conditional suffices -- no loop, no fmod.
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

float MotionEngine::defaultCruiseForDistance(float distance) const {
  const float d = distance > 0.0f ? distance : 0.0f;  // no NaN from sqrt
  const float vAllow = std::sqrt(limits_.decel * d);
  return vAllow < limits_.vMax ? vAllow : limits_.vMax;
}

float MotionEngine::dominantAxisTravel(float distance,
                                       float rotation) const {
  const float distTravel = std::fabs(distance);
  const float yawTravel = std::fabs(rotation) * effectiveTrackWidth() * 0.5f;
  return yawTravel > distTravel ? yawTravel : distTravel;
}

MotionEngine::DualRateReconciliation MotionEngine::reconcileDualRateCruise(
    float distance, float rotation, float speed, float yawRate) const {
  const float cpm = countsPerMm();
  const float b = effectiveTrackWidth();
  const float distTarget = distance * cpm;            // [counts]
  const float yawTarget = rotation * 0.5f * b * cpm;  // [counts]
  const float distSpeed = speed * cpm;                // [counts/s]
  const float twistSpeed = yawRate * 0.5f * b * cpm;  // [counts/s]

  float distDuration = 0.0f;  // [s]
  if (distTarget != 0.0f) distDuration = std::fabs(distTarget) / distSpeed;
  float yawDuration = 0.0f;  // [s]
  if (yawTarget != 0.0f) yawDuration = std::fabs(yawTarget) / twistSpeed;
  const float duration =
      distDuration > yawDuration ? distDuration : yawDuration;

  if (duration <= 0.0f) return DualRateReconciliation{0.0f, 0.0f, 0.0f};

  const float left = distTarget - yawTarget;
  const float right = distTarget + yawTarget;
  const float absLeft = std::fabs(left);
  const float absRight = std::fabs(right);
  const float dominant = absLeft > absRight ? absLeft : absRight;
  const float cruise = (dominant / duration) / cpm;  // [mm/s]

  return DualRateReconciliation{cruise, distDuration, yawDuration};
}

MotionEngine::GoToRPlan MotionEngine::decomposeGoToR(float x, float y) {
  // thetaRaw doubles atan2's principal value, so it can reach nearly
  // +-2*pi. Wrap it BEFORE deciding the split: both values reach the same
  // point on the same circle, but only the short one is sane to drive.
  const float bearingRaw = std::atan2(y, x);  // [rad] |.| <= pi
  const float thetaRaw = 2.0f * bearingRaw;   // [rad] |.| < 2*pi
  const float theta = wrapToPi(thetaRaw);     // [rad] |.| <= pi
  const float chord = std::hypot(x, y);       // [mm] >= 0
  const bool willSplit = std::fabs(theta) >= kTurnFirstAngle;

  // Blended-arc reduction via the signed radius R = (x^2+y^2)/(2y),
  // s = R*theta -- algebraically arc = radius*angle, but with no sin()
  // near theta == 0. Computed unconditionally so this one function is the
  // single source of truth for both outcomes.
  float arcLength;  // [mm] signed
  if (std::fabs(y) < 0.1f) {  // ~0.01 cm: call it straight
    arcLength = x;
  } else {
    const float radius = (x * x + y * y) / (2.0f * y);  // [mm] signed
    arcLength = radius * theta;
  }

  return GoToRPlan{bearingRaw, theta, chord, arcLength, willSplit};
}

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
  cancelMove();

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

  // A genuinely new hold restarts the shaper from the floor. A call that
  // only RETARGETS a live hold (closed-loop steering re-issuing every
  // cycle) does not -- it keeps slewing from what it is commanding now.
  if (wasSegActive || !wasHoldActive) {
    shaper_.reset();
    lastTick_ = now();
  }
}

void MotionEngine::beginSegment(float distTarget, float yawTarget,
                                float cruise, uint32_t deadline) {
  cancelMove();

  const float left = distTarget - yawTarget;
  const float right = distTarget + yawTarget;
  const float absLeft = std::fabs(left);
  const float absRight = std::fabs(right);
  const float dominant = absLeft > absRight ? absLeft : absRight;
  if (dominant <= 0.0f || cruise <= 0.0f) {
    // Nothing new to command, but still stop anything already moving.
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
  lastSegmentEndedByDeadline_ = false;

  shaper_.reset();
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
  // Captured before beginSegment() below resets seg_.
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
  const uint32_t deadline = now() + timeout;  // spans both phases

  // A large rotation combined with actual translation is not one blended
  // segment. A pure pivot, or a rotation under the threshold, stays one.
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
  const float twist = omega * 0.5f * effectiveTrackWidth();  // [mm/s] CCW+
  wheelsV(vx - twist, vx + twist, duration);
}

void MotionEngine::goToR(float x, float y, float speed, float arrive,
                         uint32_t timeout) {
  // Radial no-op gate, ahead of any split decision.
  if (std::hypot(x, y) <= arrive) return;

  const GoToRPlan plan = decomposeGoToR(x, y);

  if (plan.willSplit) {
    // Pivot to the line-of-sight bearing, then drive the straight-line
    // CHORD. Bypasses moveX() so this split fires regardless of whether
    // the bearing alone would cross moveX()'s threshold -- moveX()'s own
    // split would drive the ARC length straight and land elsewhere.
    const uint32_t deadline = now() + timeout;
    queuePivotThenStraight(plan.bearingRaw, plan.chord, speed, deadline);
  } else {
    moveX(plan.arcLength, plan.theta, speed, timeout);
  }
}

void MotionEngine::goToW(const PoseSource& pose, float x, float y,
                         float speed, float arrive, uint32_t timeout) {
  const float dx = x - pose.x();
  const float dy = y - pose.y();
  const float heading = pose.heading();
  const float cosH = std::cos(heading);
  const float sinH = std::sin(heading);

  // World-to-body rotation by -heading (CCW-positive): body x is the delta
  // projected onto the heading, body y onto 90 deg CCW of it.
  const float bodyX = dx * cosH + dy * sinH;
  const float bodyY = -dx * sinH + dy * cosH;

  goToR(bodyX, bodyY, speed, arrive, timeout);
}

bool MotionEngine::service() {
  if (!seg_.active && !hold_.active) return false;

  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  const uint32_t nowVal = now();
  const float dt = static_cast<float>(nowVal - lastTick_) / 1000.0f;
  lastTick_ = nowVal;
  const float cpm = countsPerMm();

  if (seg_.active) {
    // First tick after start: capture the origin from this tick's already
    // published Output, which the caller's step() has produced and any
    // deferred rebase has already landed in.
    if (seg_.originPending) {
      seg_.posLeft0 = out.positionLeft;
      seg_.posRight0 = out.positionRight;
      seg_.originPending = false;
    }

    // Clamp at 0: remaining() is signed and can land a hair negative on
    // the tick the target is reached. A negative remain means "no
    // displacement bound" to the shaper, which would skip the arrival test
    // entirely and drive forever.
    float remain = seg_.remaining(out) / cpm;
    if (remain < 0.0f) remain = 0.0f;
    const AxisLimits al = axisLimits(seg_);
    float target = seg_.cruise;
    if (al.cap < target) target = al.cap;
    if (limits_.vMax < target) target = limits_.vMax;

    // The kernel's last-measured speed on THIS segment's dominant axis.
    // Sign-normalized: a wheel briefly moving the wrong way is wrongWay()'s
    // problem, not this measurement's.
    const float vAct = seg_.dominantAxis == Segment::Axis::kYaw  // [mm/s]
        ? std::fabs(0.5f * (out.velocityRight - out.velocityLeft) / cpm)
        : std::fabs(0.5f * (out.velocityLeft + out.velocityRight) / cpm);
    const VelocityShaper::Step step =
        shaper_.advance(target, remain, al.floor, al.cap, dt, limits_, vAct);

    // Trust wrongWay() only once the yaw axis has genuinely moved.
    const bool wrongWay =
        seg_.wrongWay(out) &&
        std::fabs(seg_.yawProgress(out)) >= kMinYawProgressBeforeWrongWay;
    const bool expired = static_cast<int32_t>(nowVal - seg_.deadline) >= 0;
    if (wrongWay || out.stallHalted || out.estopped || expired) {
      if (wrongWay) ++wrongWayCount_;
      // The deadline had first refusal above only for WHETHER to end the
      // segment; an abort reason still wins on WHY.
      lastSegmentEndedByDeadline_ = expired && !wrongWay && !out.stallHalted &&
                                    !out.estopped;
      kernel_.neutral();
      seg_ = Segment();
      return false;
    }

    if (step.arriving) {
      kernel_.neutral();
      if (seg_.hasPending) {
        // The pivot -> straight handoff goes through rest: neutral() above
        // only stages the stop, and it lands, along with rearmReferences()
        // disarming the kernel's references, on the caller's next step().
        // Phase 2 then re-anchors fresh instead of carrying phase 1's
        // accumulated reference.
        kernel_.rearmReferences();
        beginPendingStraightPhase();
        return seg_.active;
      }
      lastSegmentEndedByDeadline_ = false;
      seg_ = Segment();
      return false;
    }

    // The (uLeft, uRight) ratio is implied by (distTarget, yawTarget),
    // recomputed from step.vCmd every tick rather than stored.
    const float velocity = (seg_.distTarget / seg_.dominant) * step.vCmd;
    const float twist = (seg_.yawTarget / seg_.dominant) * step.vCmd;
    const DiffDrive::DifferentialDrive::Status driveStatus =
        kernel_.drive(velocity * cpm, twist * cpm, 500u);
    // Because no entry point drives synchronously, a refused command can
    // only be discovered here. Without this check a permanently-refused
    // drive would re-issue every tick and spin out the whole deadline
    // looking exactly like a move that ran.
    if (driveStatus != DiffDrive::DifferentialDrive::Status::kOk) {
      lastSegmentEndedByDeadline_ = false;
      kernel_.neutral();
      seg_ = Segment();
      return false;
    }
    return true;
  }

  // Continuous hold (wheelsV()/moveV()).
  const bool holdExpired = static_cast<int32_t>(nowVal - hold_.until) >= 0;
  if (holdExpired) {
    kernel_.neutral();
    hold_.active = false;
    return false;
  }
  // remain < 0: no displacement bound, and no dominant axis worth measuring
  // against a slewing target.
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
  // An explicit external end is never a timeout, but a no-op call must not
  // overwrite a still-meaningful earlier verdict.
  if (seg_.active) lastSegmentEndedByDeadline_ = false;
  if (seg_.active || hold_.active) kernel_.neutral();
  seg_ = Segment();
  hold_ = Hold();
  shaper_.reset();
}

void MotionEngine::settleToRest() {
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
  // Before the first service() tick the origin is uncaptured, so a fraction
  // against it would be nonsense rather than merely stale.
  if (seg_.originPending) return 0;
  const DiffDrive::DifferentialDrive::Output out = kernel_.output();
  return static_cast<int>(seg_.progress(out) * 1000.0f);
}

}  // namespace diffDrive
