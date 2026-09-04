// velocity_shaper.cpp -- diffDrive::VelocityShaper::advance(), design
// docs/design/motion-profile-unification.md S6.1. See velocity_shaper.h's
// header comment for scope and portability, and this file's own step 1/
// step 5 comments below for the lag-aware amendment and its one
// deliberate deviation from S6.1's literal pseudocode.
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
  const float vPrev = v_;  // [mm/s] this tick's starting speed
  const float aPrev = a_;  // [mm/s^2] this tick's starting acceleration

  // 0. vAct (design S6.1 step 0): what the wheel is ACTUALLY doing --
  // the kernel's own last-measured dominant-axis speed, when the
  // caller has one, else this shaper's own last commanded speed (this
  // parameter's own default assumption). A real drivetrain lags the
  // command by lim.lag, so the wheel keeps covering ground at this
  // (possibly higher) speed for that long after a new, lower command
  // is issued.
  const float vAct = measured >= 0.0f ? measured : vPrev;  // [mm/s]

  // 1. Braking plan (design S6.1 step 1): the highest speed from which
  // decel can still stop inside what remains, less the coast the
  // hardware adds after the last command lands (stopDistance), less
  // what the wheel travels before it can respond at all: one tick of
  // pipeline (the command in flight this tick has not landed yet, so it
  // still covers vPrev*dt before decel can begin -- the original,
  // pre-lag term) PLUS the lag (vAct*lag, credited separately, added on
  // top).
  //
  // DEVIATION from design S6.1's literal, single-term `vAct*(dt + lag)`
  // (both the dt- and lag-portions driven by vAct): MEASURED (host
  // testing, see this repo's own report for the exact numbers) that
  // formula, applied unconditionally, changes behavior even when NO lag
  // is configured -- a real kernel-measured vAct is only APPROXIMATELY
  // equal to vPrev, even for ideal, unlagged wheels (float noise in the
  // encoder-derived velocity, ~1e-4 mm/s -- utterly negligible on its
  // own). But this system makes a discrete arrival-boundary decision
  // every tick, and that noise compounding across dozens of ticks was
  // enough to shift WHICH tick a 90 deg pivot arrives on: a cruise-200
  // ideal pivot moved from 90.15 deg to 89.26 deg (0.89 deg regression)
  // purely from vAct replacing vPrev in this term while lag stayed 0 --
  // breaking the existing "ideal-wheel (lag=0) results are unchanged"
  // guarantee a sibling host test already locks in. Keeping the
  // dt-portion on vPrev/vNext (their original basis) and ADDING the
  // lag-portion on vAct means that added term is multiplied by lim.lag,
  // which is EXACTLY 0.0f -- not merely close -- whenever lag is
  // unconfigured, so this whole expression is bit-identical to the
  // original formula at lag=0, with no float-noise sensitivity at all.
  // Once lag IS configured, this additive vAct*lag term is what closes
  // most of the gap design S6.3 describes (MEASURED against a lagged-
  // wheel host model ported from a first-order-lag/breakaway-stiction
  // probe: unfixed errors of +3.4..+26.1 deg fall to -1.6..+2.2 deg
  // with lag set to the model's own time constant -- an 85-98%
  // reduction, though not uniformly under a 1.0 deg stretch goal at
  // every speed/lag combination tested; the residual traces to a
  // separate, unrelated startup-ramp effect this formula does not
  // reach -- see this repo's own report).
  float vGoal;
  if (remain >= 0.0f) {
    const float usable0 =
        remain - lim.stopDistance - vPrev * dt - vAct * lim.lag;
    const float usable = usable0 < 0.0f ? 0.0f : usable0;
    const float vBrake = std::sqrt(2.0f * lim.decel * usable);
    vGoal = target < vBrake ? target : vBrake;
    if (cap < vGoal) vGoal = cap;
  } else {
    vGoal = target < cap ? target : cap;
  }

  // 2. Rate limit toward vGoal (first-order shaper, design S6.1 step 2).
  float vNext = vGoal;
  const float vUp = vPrev + lim.accel * dt;
  const float vDown = vPrev - lim.decel * dt;
  if (vNext > vUp) vNext = vUp;
  if (vNext < vDown) vNext = vDown;

  // 3. Optional jerk rounding (second-order, design S6.1 step 3): bound
  // da/dt, with the a^2/(2j) anticipation so a jerk-limited ramp does
  // not overshoot vGoal.
  if (lim.jerk > 0.0f && dt > 0.0f) {
    float aWant = (vGoal - vPrev) / dt;
    if (aWant > lim.accel) aWant = lim.accel;
    if (aWant < -lim.decel) aWant = -lim.decel;
    const float anticipated = vPrev + (aPrev * aPrev) / (2.0f * lim.jerk);
    if (anticipated >= vGoal && aPrev > 0.0f) aWant = 0.0f;

    float a = aWant;
    if (a > aPrev + lim.jerk * dt) a = aPrev + lim.jerk * dt;
    if (a < aPrev - lim.jerk * dt) a = aPrev - lim.jerk * dt;

    vNext = vPrev + a * dt;
    if (vNext < 0.0f) vNext = 0.0f;
    if (vNext > vGoal) vNext = vGoal;
  }

  // 4. Floor (design S6.1 step 4): while not arrived the drivetrain
  // cannot move below it, so never command less. This is the ONLY
  // floor in the system.
  if (remain >= 0.0f && vNext < floor) vNext = floor;

  // 5. Arrival (design S6.1 step 5 / S6.3): "the tick I am about to
  // command will carry me to the target" -- predicted, not discovered
  // after the fact. Same additive-term deviation as step 1 above, and
  // the same reason: bit-identical to the original `vNext*dt +
  // stopDistance` formula (a sibling host test locks that formula
  // bit-exactly) whenever lag is unconfigured -- the added `vAct*lag`
  // term is exactly zero -- and adds exactly the "wheel keeps coasting
  // at its old, actual speed for `lag` seconds" credit design S6.3
  // asks for once a real lag is configured.
  const bool arriving = remain >= 0.0f &&
      remain <= vNext * dt + vAct * lim.lag + lim.stopDistance;

  v_ = vNext;
  a_ = dt > 0.0f ? (vNext - vPrev) / dt : 0.0f;

  return Step{v_, arriving};
}

float VelocityShaper::velocity() const { return v_; }
float VelocityShaper::acceleration() const { return a_; }

}  // namespace diffDrive
