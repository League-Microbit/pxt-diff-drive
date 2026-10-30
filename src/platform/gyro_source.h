// gyro_source.h -- the narrow, host-portable interfaces the pose estimator
// reads heading and optical position through. No pxt.h, no CODAL: OtosPort
// (otos_port.h, target-only) and tests/host's SimGyro both implement them.
#pragma once

#include <cstdint>

namespace diffDrive {

enum class GyroReadStatus : uint8_t {
  kOk,      // a fresh sample was latched
  kFailed,  // the transaction failed; the latched values did not change
  kStale,   // the transaction succeeded but delivered no new sample
};

class GyroSource {
 public:
  virtual ~GyroSource() {}

  // True once the part identified itself; false means "not fitted".
  virtual bool present() const = 0;

  virtual GyroReadStatus readGyro() = 0;

  virtual bool providesHeading() const = 0;
  virtual bool providesRate() const = 0;

  // Latched by the last readGyro() that returned kOk.
  // Integrated by the part; the wrap convention is implementation-defined
  // (OtosPort wraps to (-pi, pi]), so consume via cos()/sin() or difference
  // with a wrap-aware helper.
  virtual float gyroHeading() const = 0;  // [rad]
  virtual float gyroRate() const = 0;     // [rad/s]
};

class OpticalPositionSource {
 public:
  virtual ~OpticalPositionSource() {}

  virtual bool opticalPresent() const = 0;

  // True when the last read produced a usable position fix.
  virtual bool opticalValid() const = 0;

  // Position change since the previous successful take, in the source's
  // own world frame. (dx, dy) is robot-centre referenced; (rawDx, rawDy)
  // is the sensor's own track with the lever arm NOT applied, which is
  // what stays still when the wheels do. Returns false (outputs 0) when
  // there is no new valid fix, or when this take only established a new
  // baseline (first take, or after the source was re-seeded).
  virtual bool takeOpticalIncrement(float& dx, float& dy, float& rawDx,
                                    float& rawDy) = 0;  // [mm] x4
};

}  // namespace diffDrive
