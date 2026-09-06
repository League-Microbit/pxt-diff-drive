// diffDrive::MotionLimits -- every shaping number VelocityShaper reads.
// Rationale, defaults and their measurements: DESIGN.md.
#pragma once

#include <cstdint>

namespace diffDrive {

struct MotionLimits {
  float accel = 400.0f;       // [mm/s^2] dominant-wheel accel ceiling
  float decel = 400.0f;       // [mm/s^2] dominant-wheel decel ceiling
  float jerk = 0.0f;          // [mm/s^3] 0 = no jerk rounding
  float vMax = 250.0f;        // [mm/s] dominant-wheel cruise ceiling
  float omegaMax = 0.0f;      // [deg/s] pure-turn rate ceiling; 0 = none
  float vFloor = 70.0f;       // [mm/s] below this the drivetrain does not move
  float omegaFloor = 20.0f;   // [deg/s] the same floor, for a pure turn
  float lag = 0.0f;           // [s] drivetrain response lag
  float stopDistance = 0.0f;  // [mm] per-wheel coast after the last command
  float arriveDist = 1.0f;    // [mm] distance-axis arrival window
  float arriveYaw = 0.3f;     // [deg] pure-turn arrival window

  // Validated writes: out-of-range is silently ignored. The fields whose
  // documented "off" value is 0 accept 0; the rest require positive.
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

  // A pure turn's dominant wheel runs at omega * (pi/180) * b/2.
  // `trackWidth` is the caller's EFFECTIVE track width b, not the
  // caliper-measured one.
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
