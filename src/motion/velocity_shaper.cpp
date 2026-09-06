// VelocityShaper::advance(). See DESIGN.md for the five steps, the
// lag-aware amendment, and why the lag term is added rather than folded.
#include "velocity_shaper.h"

#include <cmath>

namespace diffDrive {

void VelocityShaper::reset() {
  v_ = 0.0f;
  a_ = 0.0f;
}

VelocityShaper::Step VelocityShaper::advance(float target, float remain,
                                              float floor, float cap,
                                              float dt,
                                              const MotionLimits& lim,
                                              float measured) {
  const float vPrev = v_;  // [mm/s]
  const float aPrev = a_;  // [mm/s^2]

  // 0. What the wheel is ACTUALLY doing, which the command leads by lim.lag.
  const float vAct = measured >= 0.0f ? measured : vPrev;  // [mm/s]

  // 1. Budget coast, one command-pipeline tick, and jerk rounding.
  //
  // The pipeline term is driven by the COMMANDED speed (vPrev), not by
  // vAct, and the lag is credited SEPARATELY as vAct*lim.lag. Design
  // S6.1 writes this as a single vAct*(dt + lag); that literal form is
  // deliberately not used, and the deviation is measured, not stylistic.
  //
  // MEASURED tovez 2026-09-06, reports/square-hw-vs-sim-20260906/
  // pivot-cruise-ab.json vs pivot-cruise-ab-PREMERGE.json: with vAct on
  // the dt term (guarded on lag > 0), 90 deg pivots at lag 0.13 landed
  // -8.75 deg short at cruise 188 against -6.91 deg for the same board,
  // same config, on the pre-merge build 1.20260906.900 -- 1.8 deg per
  // pivot of extra shortfall, four corners of which walked a 60 cm
  // square into the west rail. vAct leads the falling command while
  // decelerating, so putting it on the dt term inflates the braking
  // budget and the arrival threshold together, and the move ends before
  // the wheels have covered the commanded arc.
  //
  // The lag > 0 guard is what made this invisible: every host test runs
  // at MotionLimits' default lag = 0 and was bit-identical either way.
  // Real drivetrains also stop far faster than a first-order lag near
  // the speed floor, so the extra coast this form assumes is never
  // delivered -- and the host sim cannot show that, because its plant IS
  // a first-order lag with tau equal to the configured lag.
  float vGoal;
  if (remain >= 0.0f) {
    const float usable0 =
      remain - lim.stopDistance - vPrev * dt - vAct * lim.lag;
    const float usable = usable0 < 0.0f ? 0.0f : usable0;
    const float rounding = lim.jerk > 0.0f
      ? 0.5f * lim.decel * lim.decel / lim.jerk : 0.0f;
    const float vBrake = std::sqrt(rounding * rounding +
                    2.0f * lim.decel * usable) - rounding;
    vGoal = target < vBrake ? target : vBrake;
    if (cap < vGoal) vGoal = cap;
  } else {
    vGoal = target < cap ? target : cap;
  }

  // 2. Rate limit toward vGoal.
  float vNext = vGoal;
  const float vUp = vPrev + lim.accel * dt;
  const float vDown = vPrev - lim.decel * dt;
  if (vNext > vUp) vNext = vUp;
  if (vNext < vDown) vNext = vDown;

  if (lim.jerk > 0.0f && dt > 0.0f) {
    const float difference = vGoal - vPrev;
    const float jerkStep = lim.jerk * dt;
    float lower = 0.0f;
    float upper = difference >= 0.0f ? lim.accel : lim.decel;
    for (int iteration = 0; iteration < 20; ++iteration) {
      const float candidate = 0.5f * (lower + upper);
      const float ticks = std::ceil(candidate / jerkStep);
      const float change = dt *
          (ticks * candidate - 0.5f * jerkStep * ticks * (ticks - 1.0f));
      if (change <= std::fabs(difference)) lower = candidate;
      else upper = candidate;
    }
    float acceleration = std::copysign(lower, difference);
    if (acceleration > aPrev + jerkStep) acceleration = aPrev + jerkStep;
    if (acceleration < aPrev - jerkStep) acceleration = aPrev - jerkStep;
    const float landing = difference / dt;
    if (std::fabs(landing) <= jerkStep &&
        std::fabs(landing - aPrev) <= jerkStep &&
        landing <= lim.accel && landing >= -lim.decel) {
      acceleration = landing;
    }
    vNext = vPrev + acceleration * dt;
    if (vNext < 0.0f) vNext = 0.0f;
  }

  // 4. Retain the legacy floor only without jerk limiting. A jerk-
  // limited reference must ramp through it instead of stepping to it.
  if (lim.jerk <= 0.0f && remain >= 0.0f && vNext < floor) vNext = floor;

  // 5. Predict coast-to-target. The engine separately confirms rest
  // before completing a lagged segment.
  //
  // Same basis rule as step 1, and for the same measured reason: the
  // one-tick term uses the COMMANDED vNext, with the lag credited
  // separately as vAct*lim.lag. Substituting vAct here declares arrival
  // a tick or more early on a real, lagged drivetrain -- see the
  // measurement cited in step 1. Pinned by
  // test_lagged_arrival_is_not_advanced_by_the_pipeline_term.
  //
  // Lag coast is credited only for speed ABOVE the floor. At the floor a
  // real wheel is stiction-bound and stops within a millimetre or two;
  // crediting the full vAct*lag there ends a pivot before the wheels
  // have covered the arc. MEASURED tovez 2026-09-06, wheels up, 12
  // interleaved 90 deg pivots, reports/square-hw-vs-sim-20260906/
  // bench-demo/lag-sweep-bench.json: with the full-credit form the
  // odometry shortfall was +0.14 / -0.56 / -2.80 / -8.05 deg at lag
  // 0.00 / 0.04 / 0.08 / 0.13 -- linear in lag at ~63 mm/s, i.e. the
  // credit was being taken AT the 70 mm/s floor. This is the
  // "floor-speed coast credited separately" reconciliation vevov's
  // 2026-09-04 config note asked sprint 031 for.
  //
  // MEASURED on this form, same bench, same sweep
  // (bench-demo/lag-sweep-bench-FLOORCREDIT.json): +0.60 / +0.12 /
  // -0.10 / -2.38 deg at lag 0.00 / 0.04 / 0.08 / 0.13. The clean
  // 4-leg 600 mm square at tovez's baked lag 0.13 closed at 11.0 mm
  // (bench-demo/square_clean-floorcredit.json) against 387.4 mm on
  // the full-credit form forty minutes earlier -- and 5.0 mm on the
  // pre-029 engine with its MEASURED 2 mm pivot_overrun (gopiv,
  // reports/tours-20260901.md). The residual at 0.13 is credit taken
  // during the last braking ticks where vAct is still ~20 mm/s above
  // the floor.
  //
  // The host plants (LaggedRig in test_profile_probe.py, SimWheel in
  // sim_nezha_bus.h) are pure first-order lags that coast the full
  // vAct*lag at every speed, so on THEM this form lands a lag-0.13
  // pivot ~3 deg LONG; their tests carry that offset as a documented
  // model deviation. Real wheels arbitrate; see docs/design/
  // motion-profile-unification.md S6.3, "floor-relative credit".
  const float coastCredit =                                  // [mm]
      vAct > floor ? (vAct - floor) * lim.lag : 0.0f;
  const bool arriving = remain >= 0.0f &&
      remain <= vNext * dt + coastCredit + lim.stopDistance;

  v_ = vNext;
  a_ = dt > 0.0f ? (vNext - vPrev) / dt : 0.0f;

  return Step{v_, arriving};
}

float VelocityShaper::velocity() const { return v_; }
float VelocityShaper::acceleration() const { return a_; }

}  // namespace diffDrive
