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

  // 1. Braking plan: the highest speed decel can still stop inside what
  // remains, less the hardware's coast (stopDistance), less one tick of
  // pipeline (vPrev*dt) and the lag the wheel spends still at vAct.
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

  // 2. Rate limit toward vGoal.
  float vNext = vGoal;
  const float vUp = vPrev + lim.accel * dt;
  const float vDown = vPrev - lim.decel * dt;
  if (vNext > vUp) vNext = vUp;
  if (vNext < vDown) vNext = vDown;

  // 3. Optional jerk rounding: bound da/dt, with the a^2/(2j) anticipation
  // so a jerk-limited ramp does not overshoot vGoal.
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

  // 4. Floor. The only floor in the system.
  if (remain >= 0.0f && vNext < floor) vNext = floor;

  // 5. Arrival, predicted rather than discovered: "the tick I am about to
  // command will carry me to the target".
  const bool arriving = remain >= 0.0f &&
      remain <= vNext * dt + vAct * lim.lag + lim.stopDistance;

  v_ = vNext;
  a_ = dt > 0.0f ? (vNext - vPrev) / dt : 0.0f;

  return Step{v_, arriving};
}

float VelocityShaper::velocity() const { return v_; }
float VelocityShaper::acceleration() const { return a_; }

}  // namespace diffDrive
