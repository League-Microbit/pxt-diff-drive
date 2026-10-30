// otos_port.h -- OtosPort: the SparkFun OTOS optical tracking odometry
// sensor (I2C 0x17) for the MakeCode target, ported from radio-robot's
// battle-tested Hardware::RealOtos (src/firm/hardware/generic/
// real_otos.{h,cpp}; src/DESIGN.md §2 has the authoritative upstream
// repo/path statement).
//
// Ported verbatim: the register map, the LSB scales (the velocity
// registers have their OWN full-scale ranges -- decoding them with the
// position constants reads linear velocity 2x high and angular 11.1x
// low, measured on tovez 2026-08-13), the init sequence, the
// boot-time zeroing of the chip's offset AND scalar registers (the
// chip is never power-cycled by an nRF reset, so it silently inherits
// whatever an earlier session wrote -- measured 2026-08-05: a stale
// 47.8 mm arm made a pure pivot trace a 42.7 mm circle), and the 4 ms
// shared-bus clearance between transactions (the Nezha brick shares
// this bus and needs the spacing).
//
// Bus discipline: every method that touches I2C is synchronous and
// must be called from the SAME fiber that ticks the drive kernel --
// an OTOS transaction interposed in the Nezha encoder's select->read
// settle window destroys the encoder sample (Phase F,
// radio-robot docs/design/encoder-refresh-characterization.md).
#pragma once

#include "pxt.h"

#include "../motion/motion_engine.h"
#include "gyro_source.h"

namespace diffDrive {

class OtosPort : public PoseSource,
                 public GyroSource,
                 public OpticalPositionSource {
 public:
  // Probe the product id and, on a match, run the full init sequence
  // (signal-process config, tracking reset, IMU bias calibration
  // ~612 ms of required stillness, offset/scalar/position zeroing).
  // Returns true iff the product id matched. Safe to call again to
  // re-init.
  bool begin();

  uint8_t productId() const { return lastProbeId_; }  // last probed id
  bool present() const override { return initialized_; }
  bool connected() const { return initialized_ && connected_; }

  // One 12-byte position+velocity burst read into the cache. Returns
  // false (and leaves the cache untouched) on a bus error or if
  // begin() never matched. The cached pose is the ROBOT CENTRE's, with
  // the lever arm applied (setOffset()).
  bool read();

  // GyroSource: the chip's heading channel, from the same burst read().
  GyroReadStatus readGyro() override {
    return read() ? GyroReadStatus::kOk : GyroReadStatus::kFailed;
  }
  bool providesHeading() const override { return true; }
  bool providesRate() const override { return true; }
  float gyroHeading() const override { return heading_; }  // [rad]
  float gyroRate() const override { return omega_; }       // [rad/s]

  // OpticalPositionSource: increments of the cached centre position.
  bool opticalPresent() const override { return initialized_; }
  bool opticalValid() const override { return initialized_ && connected_; }
  bool takeOpticalIncrement(float& dx, float& dy, float& rawDx,
                            float& rawDy) override;  // [mm] x4

  // PoseSource overrides (motion_engine.h) -- world/centre pose, with
  // the lever arm already applied (setOffset()). "no mount offset" below
  // refers to the sensor's own YAW mounting rotation only, not position.
  float x() const override { return x_; }              // [mm] centre
  float y() const override { return y_; }              // [mm] centre
  // [rad] (no mount offset). WRAPPED to (-pi, pi] -- the chip's int16
  // heading register has full scale +/-pi (kHdgRadPerLsb below), so
  // this is wrapped by hardware construction, not a choice this class
  // makes (resolves code review KERN-08: PoseSource::heading()'s
  // contract, motion_engine.h, is implementation-defined on wrap
  // convention for exactly this reason -- OtosPort wraps, Odometry
  // (motion/odometry.h), motion-api.md S3.6's encoder fallback, does
  // not). Consume via cos()/sin() only; do not difference two
  // heading() reads and assume a shared wrap convention.
  float heading() const override { return heading_; }
  float vx() const { return vx_; }            // [mm/s]
  float vy() const { return vy_; }            // [mm/s]
  float omega() const { return omega_; }      // [rad/s]

  // Lever arm: where the SENSOR sits relative to the robot's centre of
  // rotation, in the robot's body frame (x forward, y left), plus the
  // sensor's own yaw mounting rotation. Applied in SOFTWARE on every
  // read (sensorToCentre) and every seed (centreToSensor); the chip's
  // own offset register is deliberately held at ZERO -- applying it in
  // both places double-corrects (reference measured a pure pivot
  // tracing a 42.7 mm circle instead of holding the centre still).
  //
  // The arm is the robot's, not the program's: a robot whose config bakes
  // a non-zero arm (firmware_bake otos_arm_x / otos_arm_y) keeps it and
  // setOffset() changes only the yaw. A robot with no baked arm takes the
  // program's, as before.
  void setOffset(float x, float y, float yaw);  // [mm] [mm] [rad]
  float offsetX() const { return offsetX_; }
  float offsetY() const { return offsetY_; }
  float offsetYaw() const { return offsetYaw_; }

  // Seed the CENTRE pose: the chip's position registers are written
  // with the corresponding sensor pose.
  void setPose(float x, float y, float heading);  // [mm] [mm] [rad]

  void zeroPose() { setPose(0.0f, 0.0f, 0.0f); }
  void resetTracking();               // Kalman reset; calibration kept
  void calibrateImu(uint8_t samples); // 0 = boot default (255 samples)
  uint8_t imuCalibrationSamplesRemaining();

 private:
  static constexpr uint8_t kAddr = 0x17;  // 7-bit I2C address

  static constexpr uint8_t kRegProductId        = 0x00;
  static constexpr uint8_t kRegLinearScalar     = 0x04;
  static constexpr uint8_t kRegAngularScalar    = 0x05;
  static constexpr uint8_t kRegImuCalibration   = 0x06;
  static constexpr uint8_t kRegReset            = 0x07;
  static constexpr uint8_t kRegSignalProcessCfg = 0x0E;
  static constexpr uint8_t kRegOffsetXl         = 0x10;
  static constexpr uint8_t kRegPositionXl       = 0x20;

  static constexpr uint8_t kExpectedProductId = 0x5F;
  static constexpr uint8_t kImuCalibSamples = 255;

  static constexpr float kPosMmPerLsb = 0.305f;  // [mm/LSB] (10 m FSR)
  static constexpr float kHdgRadPerLsb =
      0.00549f * (3.14159265f / 180.0f);         // [rad/LSB] (pi FSR)
  static constexpr float kVelocityPerLsb = 5000.0f / 32768.0f;  // [mm/s/LSB]
  static constexpr float kOmegaPerLsb = 34.9f / 32768.0f;       // [rad/s/LSB]

  static constexpr int kBusClearance = 4;  // [ms] between transactions

  void busGap();  // fiber-sleep the shared-bus clearance
  bool writeReg8(uint8_t reg, uint8_t val);
  bool readReg8(uint8_t reg, uint8_t* val);
  bool writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
  void writePoseMm(uint8_t startReg, float xF, float yF, float hF);

  static void sensorToCentre(float sensorX, float sensorY, float heading,
                             float offsetX, float offsetY,
                             float& centreXOut, float& centreYOut);
  static void centreToSensor(float centreX, float centreY, float heading,
                             float offsetX, float offsetY,
                             float& sensorXOut, float& sensorYOut);

  float linearScale_ = 1.0f;   // [1] chip distance gain, baked per robot
  float angularScale_ = 1.0f;  // [1] chip heading gain, baked per robot

  float bakedArmX_ = 0.0f;  // [mm] body frame, baked per robot
  float bakedArmY_ = 0.0f;  // [mm] body frame, baked per robot

  float offsetX_ = bakedArmX_;  // [mm] body frame
  float offsetY_ = bakedArmY_;  // [mm] body frame
  float offsetYaw_ = 0.0f;  // [rad] sensor mounting rotation

  bool initialized_ = false;
  bool connected_ = false;
  uint8_t lastProbeId_ = 0;

  float x_ = 0.0f;        // [mm]
  float y_ = 0.0f;        // [mm]
  float heading_ = 0.0f;  // [rad]
  float vx_ = 0.0f;       // [mm/s]
  float vy_ = 0.0f;       // [mm/s]
  float omega_ = 0.0f;    // [rad/s]

  float rawX_ = 0.0f;     // [mm] sensor track, arm not applied
  float rawY_ = 0.0f;     // [mm]

  bool opticalFresh_ = false;     // read() succeeded since the last take
  bool opticalBaselined_ = false;
  float opticalTakenX_ = 0.0f;    // [mm]
  float opticalTakenY_ = 0.0f;    // [mm]
  float opticalTakenRawX_ = 0.0f; // [mm]
  float opticalTakenRawY_ = 0.0f; // [mm]
};

}  // namespace diffDrive
