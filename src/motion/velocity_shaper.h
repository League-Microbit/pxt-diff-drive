// velocity_shaper.h -- diffDrive::VelocityShaper: the ONE per-tick
// scalar (design docs/design/motion-profile-unification.md S4.2).
// Answers "given where I am and where I want to be, how fast should
// the dominant wheel be commanded this tick" -- braking plan, rate
// limit, optional jerk rounding, floor, and the predictive arrival
// decision, all in one place (design S6.1). Wired into MotionEngine::
// service() as its own single scalar shaper.
//
// Host-portable by construction: <cstdint>/<cmath> only (std::sqrt for
// the braking plan) -- no pxt.h, no CODAL type, no counts, no wheels,
// no kernel knowledge anywhere in this file or velocity_shaper.cpp.
// Stateful only in the two values a rate limiter needs: the last
// commanded speed and the last commanded acceleration (design S4.2's
// exact class shape) -- nothing else survives between advance() calls.
#pragma once

#include <cstdint>

#include "motion_limits.h"

namespace diffDrive {

class VelocityShaper {
 public:
  struct Step {
    float vCmd;      // [mm/s] what to command the dominant wheel this tick
    bool arriving;   // true when this is the LAST nonzero tick (design S6.3)
  };

  // At every segment start: v = 0, a = 0 (design S4.2).
  void reset();

  // design S6.1's five-step algorithm, with its lag-aware amendment
  // (design S4.1/S6.1/S6.3): braking plan, first-order rate limit,
  // optional jerk rounding, floor, predictive arrival.
  //
  // `target`   [mm/s] the ceiling this segment may run at.
  // `remain`   [mm]   signed distance still to travel on the dominant
  //                   axis, or < 0 for "no displacement bound" (continuous
  //                   drive -- design S6.1's remain < 0 branch).
  // `floor`    [mm/s] already converted to dominant-wheel speed for THIS
  //                   axis (design S6.2's omegaFloorAsWheelSpeed()/vFloor).
  // `cap`      [mm/s] likewise, this axis's ceiling.
  // `dt`       [s]    this tick's elapsed time.
  // `lim`      the shaping limits (accel/decel/jerk/lag/stopDistance)
  //            this tick's plan is built from.
  // `measured` [mm/s] the kernel's own last-measured dominant-axis
  //            speed (mean(vl, vr) for a straight/arc, the half-
  //            differential for a pure turn -- design S6.1 step 0),
  //            NOT this shaper's own last commanded speed -- a real
  //            drivetrain's wheel lags the command by `lim.lag`, so
  //            planning/predicting arrival against the command alone
  //            over-brakes late and lands long (design S6.3, MEASURED
  //            tovez 2026-09-04). `< 0` (the default) means "unknown,
  //            use this shaper's own last commanded speed instead" --
  //            a regression guard: with `measured < 0` AND `lim.lag ==
  //            0` (MotionLimits' own default) every branch below is
  //            bit-identical to the formula this parameter's addition
  //            replaced, so a caller that does not yet supply a
  //            measured speed sees no behavior change at all.
  Step advance(float target, float remain, float floor, float cap, float dt,
               const MotionLimits& lim, float measured = -1.0f);

  float velocity() const;      // [mm/s] last commanded
  float acceleration() const;  // [mm/s^2] last commanded

 private:
  float v_ = 0.0f;  // [mm/s]
  float a_ = 0.0f;  // [mm/s^2]
};

}  // namespace diffDrive
