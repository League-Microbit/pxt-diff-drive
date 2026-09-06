// diffDrive::VelocityShaper -- the one per-tick commanded-speed scalar:
// braking plan, rate limit, jerk rounding, floor, predictive arrival.
// Algorithm and its lag amendment: DESIGN.md.
#pragma once

#include <cstdint>

#include "motion_limits.h"

namespace diffDrive {

class VelocityShaper {
 public:
  struct Step {
    float vCmd;     // [mm/s] what to command the dominant wheel this tick
    bool arriving;  // true when this is the LAST nonzero tick
  };

  // v = 0, a = 0. Called at every segment start.
  void reset();

  // `target`   [mm/s] this segment's ceiling.
  // `remain`   [mm]   dominant-axis distance still to travel, or < 0 for
  //                   "no displacement bound" (a continuous hold).
  // `floor`    [mm/s] already converted to dominant-wheel speed for this axis.
  // `cap`      [mm/s] likewise, this axis's ceiling.
  // `dt`       [s]    this tick's elapsed time.
  // `lim`      the shaping limits this tick's plan is built from.
  // `measured` [mm/s] the kernel's own last-measured dominant-axis speed,
  //                   NOT this shaper's last command; < 0 means "unknown".
  Step advance(float target, float remain, float floor, float cap, float dt,
               const MotionLimits& lim, float measured = -1.0f);

  float velocity() const;      // [mm/s] last commanded
  float acceleration() const;  // [mm/s^2] last commanded

 private:
  float v_ = 0.0f;  // [mm/s]
  float a_ = 0.0f;  // [mm/s^2]
};

}  // namespace diffDrive
