// cutebot_actuation_policy.cpp -- see cutebot_actuation_policy.h. Pure;
// no I2C, no board, no kernel reference; host-compilable in its
// entirety.
#include "cutebot_actuation_policy.h"

#include <cmath>

namespace diffDrive {

namespace {

// A wheel's own tapped setpoint is eligible for the onboard loop iff it
// is exactly 0 (the `0x80` frame's own "stop" value, design doc S1.2)
// or its magnitude is at or above the floor -- a nonzero value strictly
// between 0 and the floor can never be represented on the onboard loop
// without being clamped up to something the caller did not ask for
// (design doc S1.5/S3.D's own "both-wheels eligibility gate").
bool wheelEligible(float setpoint, float floor) {
  const float mag = std::fabs(setpoint);
  return mag == 0.0f || mag >= floor;
}

// Mode 1's hysteresis band, expressed as a fraction of the configured
// floor -- see this header's own "UNVERIFIED" note: these ratios
// reproduce the design doc's own illustrative 220/180 numbers around a
// 200 mm/s floor, not a bench-fitted constant.
constexpr float kEngageFactor = 1.1f;   // engage at/above floor * 1.1
constexpr float kReleaseFactor = 0.9f;  // release at/below floor * 0.9

}  // namespace

CutebotActuationPolicy::Decision CutebotActuationPolicy::decide(
    PolicyState prevState, const WheelStaged& kernelDuty,
    const WheelSetpoint& tapSetpoint, int mode, float floor) {
  // See this file's own header comment: kernelDuty is part of the
  // signature for the caller's convenience and for a future policy
  // revision, not consulted by either mode below.
  (void)kernelDuty;

  Decision result;
  result.state = prevState;

  // Mode 0 (the wire's always-available escape hatch, SUC-004) and a
  // neutral tick both force PWM unconditionally, ahead of every other
  // check -- neither depends on the floor, the setpoint magnitude, or
  // the previous engagement state.
  if (mode == 0 || tapSetpoint.neutral) {
    result.frame = Frame::kPwm;
    result.state.engaged = false;
    return result;
  }

  if (mode == 1) {
    // Both wheels must clear the upper threshold to ENGAGE (the
    // smaller of the two magnitudes is what limits that); either wheel
    // dropping to/below the lower threshold RELEASES -- the same
    // "smaller of the two" number answers both questions.
    //
    // NO SEPARATE ELIGIBILITY CHECK HERE, DELIBERATELY. The design
    // doc's own illustrative numbers (engage 220, release 180 around a
    // 200 mm/s floor) put the RELEASE threshold BELOW the floor on
    // purpose: while already engaged, a tick whose magnitude briefly
    // sits in [lower, floor) -- the documented hysteresis "gap" --
    // must stay engaged, even though that same magnitude would fail
    // the hard "0 or >= floor" gate mode 2 uses below. Sending that
    // in-gap value over `0x80` is not a bug: the receiving end's own
    // 200 mm/s clamp (sim_cutebot_bus.h) absorbs it, which is exactly
    // this sprint's whole reason that clamp lives on the receiving end
    // and not the sender's. For the NOT-yet-engaged branch this is
    // moot: `minMag >= upper` (upper > floor) already implies both
    // wheels clear the floor, so a separate eligibility check there
    // would only ever agree with what the threshold already decided.
    const float minMag =
        std::fabs(tapSetpoint.left) < std::fabs(tapSetpoint.right)
            ? std::fabs(tapSetpoint.left)
            : std::fabs(tapSetpoint.right);
    const float upper = floor * kEngageFactor;
    const float lower = floor * kReleaseFactor;
    result.state.engaged =
        prevState.engaged ? (minMag > lower) : (minMag >= upper);
  } else if (mode == 2) {
    // Mode 2 has no magnitude threshold of its own to fall back on
    // (engagement is decided by shaper phase alone), so it DOES need
    // the explicit both-wheels gate: without it, a cruise-phase tick
    // with one wheel asymmetrically under the floor (an arc) would
    // engage just because the shaper says "at goal".
    const bool eligible = wheelEligible(tapSetpoint.left, floor) &&
                         wheelEligible(tapSetpoint.right, floor);
    if (!eligible) {
      result.state.engaged = false;
      result.frame = Frame::kPwm;
      return result;
    }
    // Plateau-only: engage the moment the shaper reports cruise,
    // release at the first sign of braking. A tick reporting kAccel
    // neither newly engages nor releases an already-engaged run (the
    // shaper does not revisit kAccel once it reaches kCruise on a
    // normal profile, but a target change mid-hold could -- staying
    // engaged through that is a deliberate, documented choice, not an
    // oversight).
    if (tapSetpoint.phase == VelocityShaper::Phase::kBrake) {
      result.state.engaged = false;
    } else if (tapSetpoint.phase == VelocityShaper::Phase::kCruise) {
      result.state.engaged = true;
    } else {
      result.state.engaged = prevState.engaged;
    }
  } else {
    // An out-of-range mode falls back to PWM, the same "unrecognized
    // field is silently inert" precedent shims.cpp's own
    // setKernelValue() uses for an ordinal neither table claims.
    result.frame = Frame::kPwm;
    result.state.engaged = false;
    return result;
  }

  result.frame = result.state.engaged ? Frame::kOnboard : Frame::kPwm;
  return result;
}

}  // namespace diffDrive
