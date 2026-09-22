// cutebot_actuation_policy.h -- diffDrive::CutebotActuationPolicy: the
// pure decision that picks, once per kernel cycle, which frame a
// Cutebot Pro ships -- the kernel's own `0x10` PWM duty, or the
// onboard speed loop's `0x80` setpoint.
//
// The design doc's own hybrid-actuation section ("our kernel below the
// handoff, their loop above") and its Design Rationale
// ("CutebotActuationPolicy as a pure function, not a stateful method on
// CutebotDevice"): hysteresis mode needs memory of the previous
// engaged/disengaged state, and both the sim bus and this file's own
// host tests need the decision to be fully deterministic and
// independent of I2C timing. This class knows NOTHING about I2C,
// `CutebotDevice`, `MotionEngine` or the kernel -- it is exercised by
// `test_cutebot_actuation_policy.py` with zero bus in the link, the
// same isolation `MotionLimits`/`VelocityShaper` already get.
//
// THE TWO INPUTS, AND WHY BOTH ARE HANDED IN. `kernelDuty` is the
// kernel's own staged PWM percent (caller-space, [-1, 1]) for this
// cycle -- what `CutebotDevice::shipFrame()` would send if this policy
// picks PWM. `tapSetpoint` is `WheelCommandTap`'s own shaped mm/s for
// the SAME cycle -- what would go out as an `0x80` setpoint if this
// policy picks onboard. Eligibility and the two modes below decide
// entirely from `tapSetpoint`, because that is the SAME mm/s quantity
// the Cutebot's own 200 mm/s floor (design doc S1.5) applies to --
// `kernelDuty` is a percent, not a speed, and comparing it against a
// mm/s floor would need a duty->mm/s conversion this policy has no
// business owning. `kernelDuty` is still part of the signature so
// `CutebotDevice` can hand this function its ENTIRE per-cycle state in
// one call rather than two, and so a later policy revision that wants
// to sanity-check duty against setpoint agreement has somewhere to
// read it from; NEITHER mode below consults it today, and it is
// explicitly unused in `decide()`'s own body.
//
// UNVERIFIED, all of it (see this repo's own measurement-citations
// rule): the 200 mm/s floor and the 200..500 clamp are SOURCE READINGS
// of ELECFREAKS' own extension (design doc S1.2/S1.5), not
// measurements off a real Cutebot Pro. The hysteresis band below
// (+-10% of the configured floor) is a POLICY CHOICE modelled on the
// design doc's own illustrative numbers (engage 220, release 180,
// around a 200 mm/s floor), not a bench-tuned constant -- a later
// bring-up measures whether either policy is even worth keeping.
#pragma once

#include "../motion/wheel_command_tap.h"

namespace diffDrive {

class CutebotActuationPolicy {
 public:
  // Which frame CutebotDevice ships this cycle.
  enum class Frame : uint8_t { kPwm, kOnboard };

  // The kernel's own staged PWM duty for both wheels this cycle,
  // caller-space (matches CutebotMotorPort::appliedDuty()'s own
  // convention: [-1, 1], sign BEFORE fwdSign_). See this header's own
  // comment above for why decide() does not consult it.
  struct WheelStaged {
    float left;   // [-1,1]
    float right;  // [-1,1]
  };

  // WheelCommandTap's own shaped command for the SAME cycle, already
  // converted to whatever sign convention the wire's `0x80` dirbits
  // use (the caller's job -- see cutebot_port.h's CutebotTapAdapter).
  // `neutral` is true exactly when the most recent tap notification was
  // onNeutral() rather than onDrive() -- mode 0 and a neutral tick both
  // force PWM unconditionally, ahead of every other check below.
  struct WheelSetpoint {
    float left;                // [mm/s]
    float right;               // [mm/s]
    VelocityShaper::Phase phase;
    bool neutral;
  };

  // Persistent hysteresis/plateau memory, threaded through by the
  // caller (CutebotDevice) one tick at a time -- see this file's own
  // header comment on why this is not a member of the policy itself.
  struct PolicyState {
    bool engaged = false;
  };

  struct Decision {
    Frame frame;
    PolicyState state;
  };

  // mode: 0 = off (always PWM), 1 = threshold with hysteresis, 2 =
  // plateau-only. floor: `onboard_floor`'s current value -- the
  // both-wheels eligibility gate's own boundary AND (scaled +-10%) the
  // hysteresis band's two thresholds in mode 1.
  static Decision decide(PolicyState prevState, const WheelStaged& kernelDuty,
                         const WheelSetpoint& tapSetpoint, int mode,
                         float floor);
};

}  // namespace diffDrive
