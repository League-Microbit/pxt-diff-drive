// nezha_port.h -- NezhaMotorPort: the DiffDrive::Motor port implemented
// against the ElecFreaks Nezha brick over I2C, for the MakeCode target.
//
// Ported from radio-robot nezha_motor.cpp + motor_armor.h's wedge
// detector. The write-shaping pipeline is not optional styling -- each
// stage guards a measured hardware failure:
//
//   - exact-zero short-circuit: stop is NEVER shaped or throttled; the
//     brick physically latches its last commanded speed across MCU
//     resets, so one lost zero write is permanent.
//   - stopNotTaken: a commanded zero re-writes while the wheel still
//     reads motion, regardless of the write-dedupe cache.
//   - reversal dwell: an instantaneous H-bridge sign flip latches the
//     0x46 encoder readback (the "encoder wedge"); write 0, hold, then
//     ship the new sign.
//   - sigma-delta duty quantizer: the brick takes integer percent; the
//     carry preserves sub-percent commands (~8 mm/s per count) and is
//     DISCARDED on zero so a stopped wheel cannot creep.
//   - min-write throttle + slew: pacing that keeps the brick's own
//     controller stable; stop bypasses both.
//
// Encoder: split-phase. requestSample() writes the 0x46 select; the
// kernel spends the ~4 ms settle in Sleeper::sleepMillis(); tick()
// executes the staged duty write AND collects the 4-byte little-endian
// counter. 1 count = 0.1 deg of shaft rotation. Encoders are never
// device-reset -- software offset rebaseline only.
#pragma once

#include "pxt.h"
#include "../core/diffdrive.h"
#include "../core/encoder_glitch_armor.h"

namespace diffDrive {

class NezhaMotorPort final : public DiffDrive::Motor {
 public:
  // port: 1-based Nezha motor port (M1..M4). fwdSign: +1/-1 so that
  // positive duty is robot-forward for a mirror-mounted wheel pair.
  NezhaMotorPort(uint8_t port, int8_t fwdSign)
      : port_(port), fwdSign_(fwdSign) {}

  // ---- DiffDrive::Motor ----
  void begin() override;
  void requestSample() override;
  void setDuty(float duty) override;      // [-1, 1] staged
  void emergencyStop() override;          // zero NOW, unstaged
  void tick(uint64_t nowUs) override;     // [us] execute staged + collect
  float position() const override;        // [counts]
  float velocity() const override;        // [counts/s]
  float appliedDuty() const override;     // [-1, 1]
  bool connected() const override { return connected_; }
  uint64_t sampleTime() const override { return sampleTimeUs_; }
  void rebaseline() override;             // software re-anchor, no bus
  bool wedged() const override { return wedgeLatched_; }
  bool wedgeSuspect() const override { return wedgeSuspect_; }

  // Shaping parameters (defaults are the firmware's shipped values).
  void configureShaping(float outputDeadband, float reversalDwell,
                        float slewRate, float writeThrottle);
                        // [-1,1] [ms] [pct/tick] [us]

 private:
  static constexpr uint8_t kAddress = 0x10;      // 7-bit I2C address
  static constexpr uint8_t kRegMotorRun = 0x60;
  static constexpr uint8_t kRegEncoder = 0x46;
  static constexpr uint8_t kDirCw = 1;
  static constexpr uint8_t kDirCcw = 2;
  static constexpr int8_t kNeverWritten = -128;  // first-write sentinel
  static constexpr float kStopConfirmVelocity = 102.0f;  // [counts/s]
  static constexpr int kWedgeThreshold = 10;     // identical reads
  static constexpr float kMotionThreshold = 0.03f;  // [-1,1] duty

  bool writeFrame(uint8_t arg, uint8_t reg, uint8_t val);
  bool readEncoderRaw(int32_t* raw);
  void writeShapedDuty(float duty, uint32_t nowMs);
  void writeRawDuty(float duty, uint64_t nowUs, bool stopping);
  void collect(uint64_t nowUs);

  uint8_t port_;
  int8_t fwdSign_;

  // shaping config [defaults = firmware shipped values]
  float outputDeadband_ = 0.03f;   // [-1,1]
  float reversalDwell_ = 100.0f;   // [ms]
  float slewRate_ = 25.0f;         // [pct per tick]
  float writeThrottle_ = 19000.0f; // [us]

  // staged command + write pipeline state
  float stagedDuty_ = 0.0f;        // [-1,1]
  int lastNonzeroSign_ = 0;        // sign of the last NONZERO command --
                                   // survives commanded zeros, so a
                                   // reversal through a brief zero still
                                   // triggers the dwell (wedgelab: the
                                   // through-zero corner reversal is the
                                   // latch trigger)
  bool atZero_ = false;            // commanded zero is currently held
  uint32_t zeroSinceMs_ = 0;       // [ms] when the zero hold began --
                                   // credited toward the reversal dwell
  bool dwelling_ = false;
  uint32_t dwellStart_ = 0;        // [ms]
 public:
  // Peak consecutive-identical-encoder-read streak observed while the
  // wheel was driven (cumulative since boot) -- direct latch evidence:
  // streaks are ticks (~24 ms each), so 13 means ~300 ms frozen.
  uint32_t maxDrivenStreak_ = 0;
  uint32_t glitchCount_ = 0;       // rejected implausible encoder reads
  // Rebaseline-on-discontinuity events (sprint 006 ticket 005,
  // encoder_glitch_armor.h's kAcceptAsRebaseline outcome): a two-strike
  // implausible-then-consistent jump treated as a counter restart
  // (e.g. a brick MCU reset) rather than integrated as a ~4 m
  // teleport. Should read 0 across a normal session with no
  // discontinuities. Exposed via diagValue() ordinal 27.
  uint32_t rebaselineCount_ = 0;
 private:
  // Two-strike raw-counts plausibility gate, extracted to
  // encoder_glitch_armor.h (host-portable, host-tested directly --
  // see that header and tests/host/test_encoder_glitch_armor.py). Owns
  // the lastGoodRaw_/lastRejectedRaw_/rejectPending_/primed_ state this
  // member used to hold inline.
  EncoderGlitchArmor glitchArmor_;

  float dutyCarry_ = 0.0f;         // [-1,1] sigma-delta remainder
  int8_t lastWrittenPct_ = kNeverWritten;
  uint64_t lastWriteTimeUs_ = 0;   // [us]

  // encoder state
  int32_t encOffset_ = 0;          // [counts] software rebaseline
  float lastPosition_ = 0.0f;      // [counts]
  float velocity_ = 0.0f;          // [counts/s]
  uint64_t sampleTimeUs_ = 0;      // [us] last SUCCESSFUL collect
  uint64_t lastTickUs_ = 0;        // [us]
  bool hasLastTick_ = false;
  bool connected_ = false;

  // wedge detector (motor_armor.h's counter, folded in)
  float lastWedgeCheckPosition_ = 0.0f;  // [counts]
  int identicalReads_ = 0;
  int identicalReadsDriven_ = 0;
  bool wedgeLatched_ = false;
  bool wedgeSuspect_ = false;
};

}  // namespace diffDrive
