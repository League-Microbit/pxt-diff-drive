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

#include <cstdint>

#include "i2c_bus.h"
#include "../core/diffdrive.h"
#include "../core/encoder_glitch_armor.h"
#include "../core/position_motor.h"

namespace diffDrive {

class NezhaMotorPort final : public DiffDrive::Motor,
                             public DiffDrive::PositionMotor {
 public:
  // port: 1-based Nezha motor port (M1..M4). fwdSign: +1/-1 so that
  // positive duty is robot-forward for a mirror-mounted wheel pair.
  //
  // The default bus preserves the two-argument motor-bake literals in
  // shims.cpp; host tests inject a simulated bus.
  NezhaMotorPort(uint8_t port, int8_t fwdSign,
                 I2CBus& bus = defaultI2CBus())
      : port_(port), fwdSign_(fwdSign), bus_(bus) {}

  // ---- DiffDrive::Motor ----
  void begin() override;
  void requestSample() override;
  void setDuty(float duty) override;      // [-1, 1] staged
  void emergencyStop() override;          // zero NOW, unstaged
  void tick(uint64_t now) override;     // [us] execute staged + collect
  float position() const override;        // [counts]
  float velocity() const override;        // [counts/s]
  float appliedDuty() const override;     // [-1, 1]
  bool connected() const override { return connected_; }
  uint64_t sampleTime() const override { return sampleTime_; }
  void rebaseline() override;             // software re-anchor, no bus
  bool wedged() const override { return wedgeLatched_; }
  bool wedgeSuspect() const override { return wedgeSuspect_; }

  // ---- DiffDrive::PositionMotor ----
  // The brick's own position move. Frame bytes: the "three frames"
  // table in docs/knowledge/2026-10-27-nezha-brick-opcodes (section 4).
  // While a move is live (the latch) setDuty() is a no-op, no 0x60 is
  // written, and driven() is true. The latch clears when the wheel is at
  // rest near its target, on hardStop(), or on emergencyStop().
  bool supportsPositionMove() const override { return true; }
  float positionQuantum() const override { return kPositionQuantum; }
  bool beginMove(float delta, float degPerSec) override;
  bool hardStop() override;
  bool moveLive() const override { return moveLive_; }
  bool driven() const override;
  float lastMoveCounts() const override { return lastMove_; }

  // speed_path selector (config ordinal 53). 0 (the
  // default) is today's 0x60 duty path, byte-identical. 1 drives the
  // wheel through the motor's own speed loop: each changed shaped duty
  // ships `0x77 round(|pct| x 11.5)` (deg/s) then `0x70` mode 3 with
  // n = 1 (a timed run of (10n - 3) x 100 ms = 0.7 s, so a starved
  // firmware self-stops), and a stop from speed is the hard-stop
  // opcode. Frame bytes: the brick opcode reference under docs/knowledge/.
  // The speed_path config ordinal calls this setter.
  void setSpeedPath(int path) { speedPath_ = path == 1 ? 1 : 0; }
  int speedPath() const { return speedPath_; }

  // Shaping parameters (defaults are the firmware's shipped values).
  void configureShaping(float outputDeadband, float reversalDwell,
                        float slewRate, float writeThrottle);
                        // [-1,1] [ms] [pct/tick] [us]

  // Re-point this side at a different port and/or reverse it at RUN
  // time. Out-of-range ignored, no-change free. Zeroes the port being
  // left, re-anchors on the new one: caller holds the bus guard.
  void configureWiring(uint8_t port, int8_t fwdSign);  // [M1..M4] [+1/-1]

  uint8_t wiredPort() const { return port_; }
  int8_t wiredSign() const { return fwdSign_; }

  // The brick's OWN counter, before encOffset_ and fwdSign_: position()
  // applies both and cannot witness a direction change; this can.
  // MEASURED tovez 2026-09-12, captures/configmotor-hardware-20260912/.
  int32_t rawCount() const { return glitchArmor_.lastGoodRaw(); }

  // PUBLIC so the fault-context emergency stop
  // (diffdrive_emergency_motor_stop(), board_nezha.cpp) can build the
  // same wire frame without an instance -- see its own comment for why
  // that path cannot use any object state. Values unchanged.
  static constexpr uint8_t kAddress = 0x10;      // 7-bit I2C address
  static constexpr uint8_t kRegMotorRun = 0x60;
  static constexpr uint8_t kDirCw = 1;

 private:
  static constexpr uint8_t kRegEncoder = 0x46;
  static constexpr uint8_t kDirCcw = 2;
  static constexpr uint8_t kRegSpeedStore = 0x77;  // per-port deg/s
  static constexpr uint8_t kRegMove = 0x70;        // move, mode in byte 6
  static constexpr uint8_t kRegHardStop = 0x5F;
  static constexpr uint8_t kMoveModeDegrees = 2;   // integer degrees
  static constexpr uint8_t kMoveModeTimed = 3;     // timed run, speed_path 1
  static constexpr uint8_t kTimedRunUnits = 1;     // n: (10n-3) x 100 ms
  // 11.5 deg/s per percent: ESP32 rig speed-step runs, 2026-10-27
  // (ESP32 rig motor characterization, brick opcode reference section 1).
  // UNVERIFIED through the brick: that rig drove the motor over UART.
  static constexpr float kDegPerSecPerPercent = 11.5f;  // [deg/s per %]
  // Re-send 0x70 before the 0.7 s timed run lapses when the speed has
  // not changed (a steady command would otherwise coast to a stop).
  static constexpr uint64_t kTimedRefresh = 400000;  // [us]
  // A stop that starts at or below the kernel floor keeps 0x60 0: the hard stop
  // and 0x60 0 travel the same 0.6 mm from the 8% floor
  // (docs/code-review/2026-10-27/data/mb_probe_stops1.csv).
  static constexpr int kStopFromSpeedFloor = 8;    // [pct]
  static constexpr float kPositionQuantum = 10.0f; // [counts] = 1 deg
  static constexpr int kMaxMoveDegrees = 65535;
  static constexpr int kMaxSpeedStore = 65535;     // [deg/s]
  // Move-done test: within this many counts of the target (capped at
  // half the commanded move so a 1-degree move cannot start "done"), and
  // the position unchanged for kMoveRestTicks ticks. The 5-degree figure
  // is the hard-stop hold accuracy (brick opcode reference, section 3).
  static constexpr float kMoveDoneBand = 50.0f;    // [counts]
  static constexpr int kMoveRestTicks = 3;
  static constexpr int8_t kNeverWritten = -128;  // first-write sentinel
  static constexpr float kStopConfirmVelocity = 102.0f;  // [counts/s]
  static constexpr int kWedgeThreshold = 10;     // identical reads
  static constexpr float kMotionThreshold = 0.03f;  // [-1,1] duty

  bool writeFrame(uint8_t arg, uint8_t reg, uint8_t val);
  bool writeBrickFrame(uint8_t arg, uint8_t reg, uint8_t hi, uint8_t mode,
                       uint8_t lo);
  void trackMove();  // tick(): completion test for a live move
  bool readEncoderRaw(int32_t* raw);
  void writeShapedDuty(float duty, uint32_t now);  // [-1,1] [ms]
  void writeRawDuty(float duty, uint64_t now, bool stopping);  // [-1,1] [us]
  void collect(uint64_t now);  // [us]

  uint8_t port_;
  int8_t fwdSign_;
  I2CBus& bus_;  // the wire; never owned

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
  uint32_t zeroSince_ = 0;       // [ms] when the zero hold began --
                                   // credited toward the reversal dwell
  bool dwelling_ = false;
  uint32_t dwellStart_ = 0;        // [ms]

  // position-move state
  bool moveLive_ = false;          // the move-live latch
  int speedPath_ = 0;              // 0 = 0x60 duty, 1 = 0x77/0x70 mode 3
  bool speedRunning_ = false;      // a mode 3 run was issued, no stop since
  uint64_t lastTimedWrite_ = 0;    // [us] last mode 3 0x70
  bool stopHold_ = false;          // brick is stopping itself: do not
                                   // re-write 0x60 0 for stopNotTaken
  int32_t lastSpeedStore_ = -1;    // [deg/s] last 0x77 value ACKed; -1 none
  float lastMove_ = 0.0f;         // [counts] rounded delta last commanded
  float moveTarget_ = 0.0f;        // [counts] position() value to reach
  int moveRestTicks_ = 0;
  float moveLastPosition_ = 0.0f;  // [counts]
 public:
  // Peak consecutive-identical-encoder-read streak observed while the
  // wheel was driven (cumulative since boot) -- direct latch evidence:
  // streaks are ticks (~24 ms each), so 13 means ~300 ms frozen.
  // Exposed via diagValue() ordinals 21 (left) and 22 (right).
  uint32_t maxDrivenStreak_ = 0;
  uint32_t glitchCount_ = 0;       // rejected implausible encoder reads
  // Two-strike counter-restart events, not integrated as motion.
  // Exposed via diagValue() ordinal 27; normally zero.
  uint32_t rebaselineCount_ = 0;
 private:
  // Two-strike raw-counts plausibility gate; owns the
  // lastGoodRaw_/lastRejectedRaw_/rejectPending_/primed_ state. Lives in
  // encoder_glitch_armor.h -- host-portable and host-tested directly.
  EncoderGlitchArmor glitchArmor_;

  float dutyCarry_ = 0.0f;         // [-1,1] sigma-delta remainder
  int8_t lastWritten_ = kNeverWritten;  // [pct]
  uint64_t lastWriteTime_ = 0;   // [us]

  // encoder state
  int32_t encOffset_ = 0;          // [counts] software rebaseline
  float lastPosition_ = 0.0f;      // [counts]
  float velocity_ = 0.0f;          // [counts/s]
  uint64_t sampleTime_ = 0;      // [us] last SUCCESSFUL collect
  uint64_t lastTick_ = 0;        // [us]
  bool hasLastTick_ = false;
  bool connected_ = false;

  // wedge detector (motor_armor.h's counter, folded in)
  float lastWedgeCheckPosition_ = 0.0f;  // [counts]
  int identicalReads_ = 0;
  int identicalReadsDriven_ = 0;
  bool wedgeLatched_ = false;
  bool wedgeSuspect_ = false;
};

// diagValue() ordinals 21/22/23/24/27/35-40's field mapping, callable
// with no I2C and no singleton -- board_nezha.cpp's boardDiagValue().
int nezhaBoardDiagValue(const NezhaMotorPort& left, const NezhaMotorPort& right,
                        int ordinal);

}  // namespace diffDrive
