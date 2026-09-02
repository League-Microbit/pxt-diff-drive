// otos_port.cpp -- see otos_port.h. Faithful port of radio-robot's
// Hardware::RealOtos onto uBit.i2c, minus the lever-arm transform.
#include "otos_port.h"

#include "vfp_guard.h"

#include <cmath>

#include "../core/heading_wrap.h"

namespace diffDrive {

void OtosPort::busGap() { vfpSafeSleep(kBusClearanceMs); }

namespace {
// codal-microbit-v2 (V2) I2C takes uint8_t*; classic DAL (V1) takes
// char* -- same guard nezha_port.cpp uses.
int i2cWrite(uint8_t addr8, uint8_t* buf, int len) {
#if MICROBIT_CODAL
  return uBit.i2c.write(addr8, buf, len);
#else
  return uBit.i2c.write(addr8, reinterpret_cast<char*>(buf), len);
#endif
}

int i2cRead(uint8_t addr8, uint8_t* buf, int len) {
#if MICROBIT_CODAL
  return uBit.i2c.read(addr8, buf, len);
#else
  return uBit.i2c.read(addr8, reinterpret_cast<char*>(buf), len);
#endif
}
}  // namespace

bool OtosPort::writeReg8(uint8_t reg, uint8_t val) {
  busGap();
  uint8_t buf[2] = {reg, val};
  return i2cWrite(kAddr << 1, buf, 2) == MICROBIT_OK;
}

bool OtosPort::readReg8(uint8_t reg, uint8_t* val) {
  busGap();
  if (i2cWrite(kAddr << 1, &reg, 1) != MICROBIT_OK) return false;
  busGap();
  return i2cRead(kAddr << 1, val, 1) == MICROBIT_OK;
}

bool OtosPort::writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h) {
  busGap();
  uint8_t buf[7];
  buf[0] = startReg;
  buf[1] = static_cast<uint8_t>(x & 0xFF);
  buf[2] = static_cast<uint8_t>((x >> 8) & 0xFF);
  buf[3] = static_cast<uint8_t>(y & 0xFF);
  buf[4] = static_cast<uint8_t>((y >> 8) & 0xFF);
  buf[5] = static_cast<uint8_t>(h & 0xFF);
  buf[6] = static_cast<uint8_t>((h >> 8) & 0xFF);
  return i2cWrite(kAddr << 1, buf, 7) == MICROBIT_OK;
}

void OtosPort::writePoseMm(uint8_t startReg, float xF, float yF, float hF) {
  long rx = lroundf(xF / kPosMmPerLsb);
  long ry = lroundf(yF / kPosMmPerLsb);
  long rh = lroundf(hF / kHdgRadPerLsb);
  if (rx > 32767) rx = 32767;
  if (rx < -32767) rx = -32767;
  if (ry > 32767) ry = 32767;
  if (ry < -32767) ry = -32767;
  if (rh > 32767) rh = 32767;
  if (rh < -32767) rh = -32767;
  writeXYH(startReg, static_cast<int16_t>(rx), static_cast<int16_t>(ry),
           static_cast<int16_t>(rh));
}

bool OtosPort::begin() {
  uint8_t id = 0;
  bool ok = readReg8(kRegProductId, &id);
  lastProbeId_ = id;
  initialized_ = ok && (id == kExpectedProductId);
  connected_ = initialized_;
  if (!initialized_) return false;

  writeReg8(kRegSignalProcessCfg, 0x0F);
  writeReg8(kRegReset, 0x01);
  writeReg8(kRegImuCalibration, kImuCalibSamples);

  // Zero the chip's scalar AND offset registers on every boot: the
  // chip is never power-cycled by an nRF reset, so it inherits
  // whatever an earlier session wrote (reference driver's measured
  // 42.7 mm phantom-circle failure). Scalar 0 == scale 1.000.
  writeReg8(kRegLinearScalar, 0);
  writeReg8(kRegAngularScalar, 0);
  writePoseMm(kRegOffsetXl, 0.0f, 0.0f, 0.0f);
  writePoseMm(kRegPositionXl, 0.0f, 0.0f, 0.0f);

  // WAIT for the IMU bias calibration to finish before returning.
  // The chip silently DISCARDS position writes while it is calibrating
  // (255 samples, ~612 ms), and then reports the origin -- so a seed
  // issued straight after begin() vanishes and the sensor confidently
  // claims the robot is at (0,0). Measured on vevov 2026-08-21: seeding
  // (50, 30, 180) immediately read back as the bare lever arm, while
  // the same seed a few seconds later read back 49.97, 29.97, 179.89.
  //
  // Silent discard is the dangerous part: a lost seed is invisible
  // unless something reads it back, and every world-frame move planned
  // afterwards would be referenced to the wrong origin.
  for (int i = 0; i < 150; ++i) {          // ~1.5 s cap
    uint8_t remaining = 0;
    if (!readReg8(kRegImuCalibration, &remaining)) break;
    if (remaining == 0) break;
    vfpSafeSleep(10);
  }
  return true;
}

bool OtosPort::read() {
  if (!initialized_) return false;

  busGap();
  uint8_t reg = kRegPositionXl;
  uint8_t raw[12] = {0};
  int ws = i2cWrite(kAddr << 1, &reg, 1);
  busGap();
  int rs = i2cRead(kAddr << 1, raw, 12);

  connected_ = (ws == MICROBIT_OK && rs == MICROBIT_OK);
  if (!connected_) return false;

  int16_t rx = static_cast<int16_t>(raw[0] | (static_cast<uint16_t>(raw[1]) << 8));
  int16_t ry = static_cast<int16_t>(raw[2] | (static_cast<uint16_t>(raw[3]) << 8));
  int16_t rh = static_cast<int16_t>(raw[4] | (static_cast<uint16_t>(raw[5]) << 8));
  int16_t rvx = static_cast<int16_t>(raw[6] | (static_cast<uint16_t>(raw[7]) << 8));
  int16_t rvy = static_cast<int16_t>(raw[8] | (static_cast<uint16_t>(raw[9]) << 8));
  int16_t rvh = static_cast<int16_t>(raw[10] | (static_cast<uint16_t>(raw[11]) << 8));

  float xF = static_cast<float>(rx) * kPosMmPerLsb;
  float yF = static_cast<float>(ry) * kPosMmPerLsb;
  const float hF = static_cast<float>(rh) * kHdgRadPerLsb;
  float vxF = static_cast<float>(rvx) * kVelocityPerLsb;
  float vyF = static_cast<float>(rvy) * kVelocityPerLsb;

  // Undo the sensor's own yaw mounting rotation, then the lever arm.
  const float ang = -offsetYaw_;
  const float c = cosf(ang);
  const float s = sinf(ang);
  const float rotX = c * xF - s * yF;
  const float rotY = s * xF + c * yF;
  const float rotVx = c * vxF - s * vyF;
  const float rotVy = s * vxF + c * vyF;

  sensorToCentre(rotX, rotY, hF, offsetX_, offsetY_, x_, y_);
  heading_ = hF;   // heading takes no mounting offset
  vx_ = rotVx;
  vy_ = rotVy;
  omega_ = static_cast<float>(rvh) * kOmegaPerLsb;
  return true;
}

// The lever arm rotates with the robot: at heading h the sensor sits
// at centre + R(h) * offset, so recovering the centre subtracts that
// same rotated offset.
void OtosPort::sensorToCentre(float sensorX, float sensorY, float heading,
                              float offsetX, float offsetY,
                              float& centreXOut, float& centreYOut) {
  const float c = cosf(heading);
  const float s = sinf(heading);
  centreXOut = sensorX - (c * offsetX - s * offsetY);
  centreYOut = sensorY - (s * offsetX + c * offsetY);
}

void OtosPort::centreToSensor(float centreX, float centreY, float heading,
                              float offsetX, float offsetY,
                              float& sensorXOut, float& sensorYOut) {
  const float c = cosf(heading);
  const float s = sinf(heading);
  sensorXOut = centreX + (c * offsetX - s * offsetY);
  sensorYOut = centreY + (s * offsetX + c * offsetY);
}

void OtosPort::setOffset(float x, float y, float yaw) {
  offsetX_ = x;
  offsetY_ = y;
  offsetYaw_ = yaw;
  if (!initialized_) return;
  // Keep the CHIP's own offset register at zero -- the arm is applied
  // in software above.
  writePoseMm(kRegOffsetXl, 0.0f, 0.0f, 0.0f);
}

// Sprint 006 ticket 004 (code review KERN-05): wraps `heading` into
// (-pi, pi] via heading_wrap.h before it reaches writePoseMm()'s LSB
// quantizer -- x/y are UNAFFECTED, still clamped (a length), not
// wrapped (a periodic angle). Without this, a heading outside +/-180
// deg (a 0-360 deg camera-yaw convention, or this project's own
// deliberately-unwrapped odometry heading echoed back through
// poseHeading()) silently clamped instead of wrapping, up to ~170 deg
// of seed error. centreToSensor()'s own use of `heading` above is
// UNCHANGED (unwrapped) -- it consumes heading only through cos/sin,
// which is wrap-invariant, so wrapping there would be a no-op; only
// the value handed to writePoseMm() needs it.
//
// This wiring is REVIEW-VERIFIED ONLY: otos_port.h includes pxt.h
// unconditionally, so OtosPort cannot be host-compiled at all (no
// existing seam exercises its I2C-bound methods host-side). The wrap
// math itself is host-tested directly and thoroughly against
// heading_wrap.h (tests/host/test_heading_wrap.py), which is the only
// host-testable proxy for this method's fix.
void OtosPort::setPose(float x, float y, float heading) {
  if (!initialized_) return;
  float sensorX = 0.0f, sensorY = 0.0f;
  centreToSensor(x, y, heading, offsetX_, offsetY_, sensorX, sensorY);
  // Re-apply the sensor's own yaw mounting rotation (inverse of read()).
  const float c = cosf(-offsetYaw_);
  const float s = sinf(-offsetYaw_);
  const float xF = c * sensorX + s * sensorY;
  const float yF = -s * sensorX + c * sensorY;
  writePoseMm(kRegPositionXl, xF, yF, wrapRadians(heading));
}

void OtosPort::resetTracking() {
  if (!initialized_) return;
  writeReg8(kRegReset, 0x01);
}

void OtosPort::calibrateImu(uint8_t samples) {
  if (!initialized_) return;
  writeReg8(kRegImuCalibration, samples == 0 ? kImuCalibSamples : samples);
}

uint8_t OtosPort::imuCalibrationSamplesRemaining() {
  if (!initialized_) return 0;
  uint8_t v = 0;
  readReg8(kRegImuCalibration, &v);
  return v;
}

}  // namespace diffDrive
