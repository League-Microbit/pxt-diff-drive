// diffDrive::VelocityShaper -- the one per-tick commanded-speed scalar:
// braking plan, rate limit, jerk rounding, floor, predictive arrival.
// Algorithm and its lag amendment: DESIGN.md.
#pragma once

#include <cstdint>

#include "motion_limits.h"

namespace diffDrive {

class VelocityShaper {
 public:
  // WheelCommandTap's own phase signal (the Cutebot Pro support design
  // doc's "where the velocity setpoint comes from" section), derived
  // here rather than handed a second channel of its own -- see
  // advance()'s own comment for exactly where each value is decided.
  // Does not feed back into v_/a_/vCmd/arriving: adding it changes no
  // shaping behaviour, only what a caller can additionally read off a
  // Step.
  enum class Phase : uint8_t {
    kAccel,   // still ramping toward this tick's goal
    kCruise,  // holding at the goal; not bounded by the stop-distance
              // brake budget below
    kBrake,   // this tick's goal is bounded by the brake-to-stop budget
              // (remain-driven), not by the segment's target/cap -- a
              // short move whose target is never reachable is kBrake
              // from its very first tick, never kCruise
  };

  struct Step {
    float vCmd;     // [mm/s] what to command the dominant wheel this tick
    bool arriving;  // true when this is the LAST nonzero tick
    Phase phase;    // this tick's accel/cruise/brake classification
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
