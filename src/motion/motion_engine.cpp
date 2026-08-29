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

}  // namespace

MotionEngine::MotionEngine(DiffDrive::DifferentialDrive& kernel,
                           const DiffDrive::Clock& clock)
    : kernel_(kernel), clock_(clock) {}

uint32_t MotionEngine::nowMs() const {
  return static_cast<uint32_t>(clock_.nowMicros() / 1000ull);
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
  move_.distTarget = distance * cpm;                              // [counts]
  move_.yawTarget = rotation * 0.5f * effectiveTrackWidth() * cpm;  // [counts]
  // Subtract the measured per-wheel end-of-move overrun from the
  // rotation target (pivotOverrunMm_, motion_engine.h): a constant the
  // controller lands PAST every rotation, whatever its size, so it is
  // taken off the target's magnitude, never scaled. Clamped at zero: a
  // rotation smaller than the overrun itself becomes no rotation rather
  // than one in the wrong direction.
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

  const float cruiseCounts = cruise * cpm;  // [counts/s]
  move_.velCmd = move_.distTarget / dominant * cruiseCounts;
  move_.twistCmd = move_.yawTarget / dominant * cruiseCounts;

  // Acceleration ramp (stakeholder, 2026-08-20): start at the floor
  // rate, not a full-rate step -- serviceMove() raises the scale over
  // rampMs_. Mirrors the end-of-move taper; effective accel ~= full
  // rate / 0.4 s (~375 mm/s^2 at 15 cm/s), reference-shaper-like.
  move_.startMs = nowMs();
  move_.cmdScale = 0.25f;
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
      move_.velCmd * 0.25f, move_.twistCmd * 0.25f, remainingMs);
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

  float scale = 1.0f;
  bool distDone = true;
  if (move_.distTarget != 0.0f) {
    const float remain =
        std::fabs(move_.distTarget) - std::fabs(meanProgress);
    distDone = remain <= distMargin;
    const float axisScale = remain / distTaper_;
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
      const float axisScale = remain / yawTaper_;
      if (axisScale < scale) scale = axisScale;
    }
  }
  if (scale < kTaperFloor) scale = kTaperFloor;

  // Acceleration ramp: time-based rise from the floor to full rate over
  // rampMs_, min-combined with the end taper (a very short move may go
  // straight from ramp to taper without ever reaching full).
  const uint32_t nowMsRamp = nowMs();
  float ramp =
      static_cast<float>(nowMsRamp - move_.startMs) / rampMs_;
  if (ramp < kTaperFloor) ramp = kTaperFloor;
  if (ramp < scale) scale = ramp;
  if (scale > 1.0f) scale = 1.0f;

  // Reissue EVERY tick while the move is active, at the current scale
  // with a rolling 500 ms lease -- the only form that is lease-safe:
  // gating reissues on scale CHANGE would let the lease expire during
  // any steady phase (floor crawl, or full rate after the ramp
  // completes). The kernel's lease backstop still covers an abandoned
  // move within 500 ms; the REAL timeout backstop is move_.deadline,
  // checked below, independent of this rolling lease.
  if (!(distDone && yawDone)) {
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
  if (distDone && yawDone && !expired && !out.stallHalted && !wrongWay &&
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
  if ((distDone && yawDone) || expired || out.stallHalted || wrongWay ||
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
  const float dLeft = out.positionLeft - move_.posLeft0;
  const float dRight = out.positionRight - move_.posRight0;
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
