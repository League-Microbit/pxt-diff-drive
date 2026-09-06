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
  // Lagged motors use measured speed for the pipeline distance;
  // zero-lag motors retain the original commanded-speed basis.
  float vGoal;
  if (remain >= 0.0f) {
    const float usable0 =
      remain - lim.stopDistance -
      (lim.lag > 0.0f ? vAct : vPrev) * dt - vAct * lim.lag;
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
  const bool arriving = remain >= 0.0f &&
      remain <= (lim.lag > 0.0f ? vAct : vNext) * dt +
            vAct * lim.lag + lim.stopDistance;

  v_ = vNext;
  a_ = dt > 0.0f ? (vNext - vPrev) / dt : 0.0f;

  return Step{v_, arriving};
}

float VelocityShaper::velocity() const { return v_; }
float VelocityShaper::acceleration() const { return a_; }

}  // namespace diffDrive
