// heading_wrap.h -- wrapRadians() [rad]: normalizes any radian angle
// into (-pi, pi] before it reaches OtosPort::writePoseMm()'s LSB
// quantizer. Its own header rather than an OtosPort member because
// otos_port.h includes pxt.h unconditionally (src/DESIGN.md S1's
// layering table) and so cannot be compiled host-side at all; <cmath>
// and nothing else here is what lets tests/host/test_heading_wrap.py
// exercise the wrap directly and heading_wrap_syntax_check.cpp keep it
// in the C++11 syntax gate. The call site in OtosPort::setPose() is
// review-verified only -- see its own comment in otos_port.cpp.
#pragma once

#include <cmath>

namespace diffDrive {

// Normalizes any radian value into (-pi, pi] -- the wrap-mandatory
// range a periodic angle needs, as opposed to x/y (a bounded length,
// correctly CLAMPED rather than wrapped by OtosPort::writePoseMm(),
// otos_port.cpp -- unaffected by this function). Without this, a
// heading outside +/-180 deg -- a 0-360 deg camera-yaw convention
// value, or this project's own deliberately-unwrapped odometry heading
// (Rig::heading, echoed back through poseHeading()) -- is silently
// CLAMPED instead of wrapping: a 350 deg seed lands at +179.89 deg
// instead of the correct -10 deg, up to ~170 deg of error.
//
// Exactly +/-180 deg (+/-pi) is a boundary case worth calling out
// explicitly: this function wraps any input congruent to an odd
// multiple of pi to the canonical +pi representative (it never returns
// -pi), but the chip's int16 heading register has full scale +/-pi
// with a granularity of one LSB (~0.00549 deg) -- so +pi itself lands
// one LSB outside the representable range, and the register write
// still clamps to +179.89 deg even after this fix. That residual clamp
// is NOT a bug this function needs to fix -- it is
// OtosPort::writePoseMm()'s pre-existing, correct clamp, still
// exercised at this one exact boundary because a wrap-mandatory value
// and a length-style clamp happen to agree there. See
// tests/host/test_heading_wrap.py's own boundary-case coverage.
inline float wrapRadians(float rad) {
  const float kPi = 3.14159265358979323846f;
  const float kTwoPi = 2.0f * kPi;
  float m = fmodf(kPi - rad, kTwoPi);
  if (m < 0.0f) m += kTwoPi;
  return kPi - m;
}

}  // namespace diffDrive
