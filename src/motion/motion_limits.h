// motion_limits.h -- diffDrive::MotionLimits: the ONE value object that
// holds every shaping number VelocityShaper::advance() reads (design
// docs/design/motion-profile-unification.md S4.1). Now wired into
// MotionEngine via limits() (motion_engine.h) -- the class's own
// settable shaping surface.
//
// Host-portable by construction: <cstdint> only, no <cmath> (the two
// unit-conversion helpers below need only a fixed multiplication by a
// literal pi, never a libm call) -- no pxt.h, no CODAL type, anywhere.
// Covered by tests/host/test_cxx11_syntax_gate.py via a dedicated
// syntax-check translation unit (tests/host/motion_limits_syntax_check.cpp
// -- this header has no natural .cpp of its own, the same reason
// heading_wrap.h/encoder_pose_source.h each have one).
//
// Field names follow design S4.7's wire-name mapping exactly (`accel`,
// `decel`, `jerk`, `vMax`, `omegaMax`, `vFloor`, `omegaFloor`, `lag`,
// `stopDistance`, `arriveDist`, `arriveYaw`) so the wire descriptor
// table (a later ticket's own job) does not need to rename anything
// here. `lag` (design S4.1/S6.1/S10.2) is the
// drivetrain's own first-order response lag -- the wheel follows a
// command change about `lag` seconds late, so it keeps covering ground
// at its old, higher, ACTUAL speed for that long after the shaper
// thinks it has already started slowing down. 0 until bench-measured
// (design S10.2's own first of the three S10.2 measurements, landing
// before `stopDistance`, which must be measured with `lag` already
// set). Naming follows .claude/rules/no-units-in-identifiers.md and the
// kernel's own style: an identifier names the quantity, the unit is a
// trailing `// [unit]` comment on its declaration -- never in the name
// itself (design S4's own naming note, first paragraph).
//
// Fields are PUBLIC, per design S4.1's struct listing -- VelocityShaper::
// advance() (velocity_shaper.h) reads them directly (`lim.accel`,
// `lim.decel`, ...), the same way the design's own pseudocode does. The
// "positive, else keep" setters below (matching MotionEngine::
// setRotationalSlip()'s validation style, motion_engine.h) are what
// limits()'s own callers (the wire `SET`/block `set config` surface)
// use for a validated write path; direct field access stays available
// for tests and for VelocityShaper itself.
#pragma once

#include <cstdint>

namespace diffDrive {

struct MotionLimits {
  // ---- rates: per robot, bench-measured; defaults are the fleet bake
  // (design S4.1). ----
  float accel = 400.0f;      // [mm/s^2] dominant-wheel accel ceiling
  float decel = 400.0f;      // [mm/s^2] dominant-wheel decel ceiling
                              //   (braking plan, design S6.1 step 1)
  float jerk = 0.0f;         // [mm/s^3] 0 = no jerk rounding (first-order
                              //   shaper only, design S6.1 step 3)
  float vMax = 250.0f;       // [mm/s] dominant-wheel cruise ceiling
  float omegaMax = 0.0f;     // [deg/s] pure-turn rate ceiling; 0 = none

  // ---- floors: below these the drivetrain does not move, so the
  // profile never commands less while not yet arrived (design S6.1
  // step 4 -- "the ONLY floor in the system"). ----
  float vFloor = 70.0f;      // [mm/s] MEASURED tovez/gopiv 2026-08-29
                              //   (the old kernel vMin; see the
                              //   playfield-testing rules doc's
                              //   "v_floor is already measured" note)
  float omegaFloor = 20.0f;  // [deg/s] UNVERIFIED; sized so one tick is
                              //   ~0.5 deg (design S6.2, pending the
                              //   S10.2 bench sweep)

  // ---- arrival (design S6.3: "predict, don't discover"). ----
  float lag = 0.0f;          // [s] drivetrain response lag: the wheel
                              //   follows a command change about this
                              //   much later (design S6.1 step 0's
                              //   `vAct`, S10.2). 0 until measured --
                              //   the FIRST of the three S10.2 bench
                              //   measurements, before stopDistance.
  float stopDistance = 0.0f; // [mm] per-wheel coast after the last
                              //   nonzero command lands; replaces
                              //   pivot_overrun. 0 until measured
                              //   (design S10.2, measured WITH lag
                              //   already set)
  float arriveDist = 1.0f;   // [mm] distance-axis arrival window
  float arriveYaw = 0.3f;    // [deg] pure-turn arrival window

  // ---- "positive, else keep" setters (MotionEngine::setRotationalSlip()'s
  // style, motion_engine.h) -- jerk and omegaMax accept ZERO too, since
  // 0 is each field's own documented "off"/"none" value above; every
  // other field requires a strictly positive value to take effect. An
  // invalid input is silently ignored, same as today's setters. ----
  void setAccel(float v) {
    if (v > 0.0f) accel = v;
  }
  void setDecel(float v) {
    if (v > 0.0f) decel = v;
  }
  void setJerk(float v) {
    if (v >= 0.0f) jerk = v;
  }
  void setVMax(float v) {
    if (v > 0.0f) vMax = v;
  }
  void setOmegaMax(float v) {
    if (v >= 0.0f) omegaMax = v;
  }
  void setVFloor(float v) {
    if (v >= 0.0f) vFloor = v;
  }
  void setOmegaFloor(float v) {
    if (v >= 0.0f) omegaFloor = v;
  }
  void setLag(float v) {
    if (v >= 0.0f) lag = v;
  }
  void setStopDistance(float v) {
    if (v >= 0.0f) stopDistance = v;
  }
  void setArriveDist(float v) {
    if (v > 0.0f) arriveDist = v;
  }
  void setArriveYaw(float v) {
    if (v > 0.0f) arriveYaw = v;
  }

  // ---- axis-unit conversions (design S6.2): a pure turn's dominant
  // wheel speed is omega * (pi/180) * trackWidth/2 -- these convert
  // THIS object's own deg/s floor/ceiling into the dominant-wheel
  // mm/s a caller (VelocityShaper::advance(), later MotionEngine)
  // already works in. `trackWidth` is the caller's EFFECTIVE track
  // width `b` (motion-api.md S2.1: trackWidth / rotationalSlip), not
  // the caliper-measured one -- this object has no opinion on which;
  // it just converts whatever [mm] it is given. ----
  float omegaFloorAsWheelSpeed(float trackWidth) const {  // [mm] -> [mm/s]
    const float kDegToRad = 3.14159265358979323846f / 180.0f;
    return omegaFloor * kDegToRad * trackWidth * 0.5f;
  }
  float omegaMaxAsWheelSpeed(float trackWidth) const {  // [mm] -> [mm/s]
    const float kDegToRad = 3.14159265358979323846f / 180.0f;
    return omegaMax * kDegToRad * trackWidth * 0.5f;
  }
};

}  // namespace diffDrive
