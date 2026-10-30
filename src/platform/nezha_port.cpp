// nezha_port.cpp -- see nezha_port.h. Ported from the firmware's
// nezha_motor.cpp; the shaping-stage ORDER is load-bearing.
#include "nezha_port.h"

// Declaration only (`void vfpSafeSleep(uint32_t)`); the pxt-bound
// definition is vfp_guard.cpp, which the host harness replaces with its
// own no-op -- there are no fibers to yield to on the host.
#include "vfp_guard.h"

#include <cmath>

// Host tests exclude CODAL and ARM fault handlers, not motor shaping.
// Keep DIFFDRIVE_FAULT_SPIN here: make_deploy.py validates it by source.
#ifndef DIFFDRIVE_HOST_BUILD
#include "pxt.h"
#endif

namespace diffDrive {

#ifndef DIFFDRIVE_HOST_BUILD

// ---- fault-context emergency stop -----------------------------------
//
// Writes "run at 0" to BOTH motor ports over I2C with no dependency on
// any object, fiber, scheduler or kernel state -- so it is callable
// from a fault handler, where none of those can be trusted.
//
// WHY THIS EXISTS. Left to codal-nrf52's DEFAULT weak
// HardFault_Handler -- an infinite loop in gcc_startup_nrf52833.S:303
// -- a fault panics nothing, reboots nothing, and THE BRICK KEEPS ITS
// LAST MOTOR COMMAND: the wheels run until someone reflashes the board
// (MEASURED tigez 2026-08-30, pyOCD halt on the wedged chip, CFSR
// 0x8200 / BFAR 0x474E4988, forensics in
// captures/tigez-cal-20260830/notes.md).
//
// The root cause was the VFP register clobber vfp_guard.h now closes:
// CODAL's context switch saves no VFP registers, GCC parks pointers in
// the callee-saved bank s16-s31, and a fiber switch destroys them --
// which is why that BFAR decoded as the ASCII bytes "PING" with no heap
// corruption anywhere. Decode a future BFAR as ASCII and as a float
// before calling it garbage. The fault no longer reproduces (MEASURED
// tigez 2026-09-02, 28 radio-hammer trials over two builds plus 6
// radio-silent controls, 0 reset signatures,
// captures/tigez-radio-retest-20260902/). This function and the
// handlers below stay regardless: a fault handler that fails safe is
// correct even against a fault that no longer fires.
//
// A plain reboot is NOT sufficient on its own: the Rig (and with it
// DifferentialDrive::begin()'s boot zero-write) is created LAZILY on
// the first motion command, so a rebooted board that nobody commands
// would leave the brick driving.
//
// DEFINED per-board in board_nezha.cpp now (a different board's frame
// at this same address would be wrong); the fault handlers below stay
// board-generic and reach it through this declaration either way.
extern "C" void diffdrive_emergency_motor_stop();

// ---- fail-safe fault handlers ---------------------------------------
//
// Override CODAL's weak infinite-loop handlers: stop the motors first,
// then reset (or retain the frame for a debugger in a fault-spin build).
// Do not allocate, lock, print, or yield: memory may already be corrupt.
extern "C" {

// `frame` is the hardware-stacked exception frame:
// {r0, r1, r2, r3, r12, LR, PC, xPSR}.
void diffdriveFaultReport(uint32_t* frame) {
  (void)frame;  // see DIFFDRIVE_FAULT_SPIN below for how to read it
  diffdrive_emergency_motor_stop();

  // NOTE: do NOT print here. uBit.serial.printf() is not fault-safe --
  // it blocked forever inside the handler when tried (MEASURED tigez
  // 2026-08-30, captures/tigez-cal-20260830/notes.md: pyOCD showed
  // IPSR=3, PC parked in the handler). The
  // motors were already stopped by then, so it was safe, but the board
  // no longer recovered. Forensics go through DIFFDRIVE_FAULT_SPIN.
#ifdef DIFFDRIVE_FAULT_SPIN
  // DEBUG BUILD: hold the fault state so pyOCD can halt and read the
  // stacked frame (`b diffdriveFaultReport` keeps `frame` in r0).
  // Safe to sit here: the motors are already stopped, above.
  while (true) {
  }
#else
  NVIC_SystemReset();  // never returns
#endif
}

__attribute__((naked)) void HardFault_Handler() {
  // Pick the stack the fault frame is actually on (EXC_RETURN bit 2).
  __asm volatile(
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "b diffdriveFaultReport\n");
}
__attribute__((naked)) void MemoryManagement_Handler() {
  __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n"
                 "b diffdriveFaultReport\n");
}
__attribute__((naked)) void BusFault_Handler() {
  __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n"
                 "b diffdriveFaultReport\n");
}
__attribute__((naked)) void UsageFault_Handler() {
  __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n"
                 "b diffdriveFaultReport\n");
}

}  // extern "C"

#endif  // DIFFDRIVE_HOST_BUILD -- fault handlers are ARM-only


namespace {
float clampf(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}
}  // namespace

// ---- bus primitives -------------------------------------------------

bool NezhaMotorPort::writeFrame(uint8_t arg, uint8_t reg, uint8_t val) {
  uint8_t frame[8] = {0xFF, 0xF9, port_, arg, reg, val, 0xF5, 0x00};
  // The V1/V2 signature split this used to switch on lives in the bus
  // implementation now (platform/microbit_i2c_bus.cpp); 0 is CODAL's
  // MICROBIT_OK, which `I2CBus` adopts as its own success convention.
  return bus_.write(kAddress << 1, frame, 8) == 0;
}

bool NezhaMotorPort::writeBrickFrame(uint8_t arg, uint8_t reg, uint8_t hi,
                                     uint8_t mode, uint8_t lo) {
  uint8_t frame[8] = {0xFF, 0xF9, port_, arg, reg, hi, mode, lo};
  return bus_.write(kAddress << 1, frame, 8) == 0;
}

bool NezhaMotorPort::readEncoderRaw(int32_t* raw) {
  uint8_t data[4] = {0, 0, 0, 0};
  if (bus_.read(kAddress << 1, data, 4) != 0) return false;
  *raw = static_cast<int32_t>(
      static_cast<uint32_t>(data[0]) |
      (static_cast<uint32_t>(data[1]) << 8) |
      (static_cast<uint32_t>(data[2]) << 16) |
      (static_cast<uint32_t>(data[3]) << 24));
  return true;
}

// ---- lifecycle ------------------------------------------------------

void NezhaMotorPort::begin() {
  // The 0x46 register sits frozen at 0 until its first select+read.
  // Median-of-3 atomic reads -> software offset, so position() starts
  // at zero without ever device-resetting the counter.
  //
  // Bus-hang guard. codal-nrf52 commit 1fbb724 -- what this project's
  // codal-microbit-v2 v0.3.5 actually resolves to -- bounds one stuck
  // I2C transaction at ~11s rather than forever:
  // NRF52I2C::waitForStop() times out after ~10s and then waits up to
  // ~1s for its recovery STOP. Break at the FIRST hard failure so a
  // dead brick costs one such call per motor instead of three (~33s per
  // motor, ~66s across both wheels in DifferentialDrive::begin()). The
  // cost: a single transient NACK mid-sequence now reports
  // connected() == false where a median-of-2 boot used to survive it.
  int32_t samples[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < 3; ++i) {
    if (!writeFrame(0x00, kRegEncoder, 0x00)) break;  // hard failure --
                                                        // don't multiply a
                                                        // wedged-bus delay
                                                        // by trying more
    vfpSafeSleep(4);  // [ms] select -> read settle
    int32_t raw = 0;
    if (!readEncoderRaw(&raw)) break;  // same reasoning
    samples[good++] = raw;
  }
  if (good > 0) {
    // median of what we got
    for (int i = 0; i < good; ++i)
      for (int j = i + 1; j < good; ++j)
        if (samples[j] < samples[i]) {
          int32_t t = samples[i]; samples[i] = samples[j]; samples[j] = t;
        }
    encOffset_ = samples[good / 2];
    glitchArmor_.seedLastGoodRaw(encOffset_);
    connected_ = true;
  }
  // Arms the plausibility gate unconditionally, even when good == 0 --
  // reproducing the pre-extraction code's own asymmetry (lastGoodRaw_
  // stays at its default 0 in that case, but the gate still arms).
  glitchArmor_.markPrimed();
}

// ---- command staging + shaping --------------------------------------

void NezhaMotorPort::setDuty(float duty) {
  // Move-live latch: the brick's own loop owns the wheel, the kernel's
  // per-tick duty (including its setDuty(0)) must not reach the wire or
  // advance any shaping state. Review F3.
  if (moveLive_) return;
  stagedDuty_ = clampf(duty, -1.0f, 1.0f);
}

void NezhaMotorPort::emergencyStop() {
  // The one call that must not depend on a healthy tick(): zero the
  // stage AND write zero through the never-shaped stop path now.
  stagedDuty_ = 0.0f;
  if (moveLive_) {
    // Stop the brick's own move first (the hard-stop opcode holds the
    // count), then the usual 0x60 0. The latch drops either
    // way: e-stop owns the wheel now. lastWritten_ is forgotten so the
    // 0x60 0 is not deduped against a pre-move value.
    writeBrickFrame(0, kRegHardStop, 0, 0xF5, 0);
    moveLive_ = false;
    lastWritten_ = kNeverWritten;
  } else if (lastWritten_ != kNeverWritten &&
             (lastWritten_ > kStopFromSpeedFloor ||
              lastWritten_ < -kStopFromSpeedFloor)) {
    // No live move, but the wheel was last driven above the kernel
    // floor: stop it with the hard stop, as the speed path does. Travel
    // after a stop is the same at 8 % and 5 to 12 times shorter above
    // it (docs/code-review/2026-10-27/data/mb_probe_stops1.csv). At or
    // below the floor, or with nothing written, the plain 0x60 0 below
    // is all that is sent.
    writeBrickFrame(0, kRegHardStop, 0, 0xF5, 0);
    speedRunning_ = false;
    stopHold_ = true;  // stopNotTaken must not follow with more 0x60 0
    lastWritten_ = kNeverWritten;
  }
  writeShapedDuty(0.0f, static_cast<uint32_t>(lastTick_ / 1000));
}

// ---- position moves (PositionMotor) ----------------------------------

bool NezhaMotorPort::beginMove(float delta, float degPerSec) {
  lastMove_ = 0.0f;
  const long deg = std::lround(std::fabs(delta) / kPositionQuantum);
  const long speed = std::lround(degPerSec);
  if (deg < 1 || deg > kMaxMoveDegrees) return false;
  if (speed < 1) return false;
  const int store = speed > kMaxSpeedStore ? kMaxSpeedStore
                                           : static_cast<int>(speed);
  // 0x77 only when this port's stored speed differs (the brick keeps it
  // per port across moves, opcode doc section 3 rule 5).
  if (store != lastSpeedStore_) {
    if (!writeBrickFrame(0, kRegSpeedStore,
                         static_cast<uint8_t>(store >> 8), 0,
                         static_cast<uint8_t>(store & 0xFF))) {
      connected_ = false;
      lastSpeedStore_ = -1;
      return false;
    }
    lastSpeedStore_ = store;
  }
  // The brick's counter runs with the raw sign; position() applies
  // fwdSign_, so a robot-forward delta maps to direction fwdSign_ * sign.
  const bool forward = (delta >= 0.0f) == (fwdSign_ > 0);
  if (!writeBrickFrame(forward ? kDirCw : kDirCcw, kRegMove,
                       static_cast<uint8_t>(deg >> 8), kMoveModeDegrees,
                       static_cast<uint8_t>(deg & 0xFF))) {
    connected_ = false;
    return false;
  }
  lastMove_ = (delta >= 0.0f ? 1.0f : -1.0f) *
                    static_cast<float>(deg) * kPositionQuantum;
  moveTarget_ = lastPosition_ + lastMove_;
  moveLive_ = true;
  stopHold_ = false;
  speedRunning_ = false;  // the mode 2 move supersedes any timed run
  moveRestTicks_ = 0;
  moveLastPosition_ = lastPosition_;
  // The move supersedes whatever 0x60 the wheel was running: the
  // shaping pipeline restarts from "brick at rest, last duty 0".
  stagedDuty_ = 0.0f;
  dutyCarry_ = 0.0f;
  lastWritten_ = 0;
  lastNonzeroSign_ = 0;
  atZero_ = true;
  dwelling_ = false;
  return true;
}

bool NezhaMotorPort::hardStop() {
  if (!writeBrickFrame(0, kRegHardStop, 0, 0xF5, 0)) {
    connected_ = false;
    return false;  // latch stays: the move may still be running
  }
  moveLive_ = false;
  speedRunning_ = false;
  // The brick brings the wheel to rest itself; a 0x60 0 while it is
  // still above rest would turn the hard stop into a ramp and drop the
  // hold (doc section 3 rule 1), so stopNotTaken stands down until the
  // wheel reads at rest.
  stopHold_ = true;
  stagedDuty_ = 0.0f;
  dutyCarry_ = 0.0f;
  lastWritten_ = 0;
  return true;
}

bool NezhaMotorPort::driven() const {
  return moveLive_ || std::fabs(appliedDuty()) > kMotionThreshold;
}

void NezhaMotorPort::trackMove() {
  const float remaining = std::fabs(moveTarget_ - lastPosition_);
  float band = std::fabs(lastMove_) * 0.5f;
  if (band > kMoveDoneBand) band = kMoveDoneBand;
  if (lastPosition_ == moveLastPosition_) ++moveRestTicks_;
  else moveRestTicks_ = 0;
  moveLastPosition_ = lastPosition_;
  if (remaining <= band && moveRestTicks_ >= kMoveRestTicks) {
    moveLive_ = false;  // completed: wheel at rest at/near the target
    stopHold_ = true;
  }
}

void NezhaMotorPort::writeShapedDuty(float duty, uint32_t now) {  // [-1,1] [ms]
  // 1. Exact zero short-circuits ALL shaping. Stop is stop. The zero
  //    entry TIME is recorded and the last nonzero SIGN is kept: a
  //    reversal that passes through a brief commanded zero (a move
  //    ending, then the next move starting opposite -- every square
  //    corner) is still a reversal, and the brick still needs its full
  //    zero dwell. The old code cleared the sign history here, which
  //    shipped corner reversals ~20-30 ms after the zero -- inside the
  //    (20, 50] ms window the wedgelab campaign measured as 12/12
  //    latching (radio-robot docs/knowledge/2026-07-04-encoder-
  //    wedge.md). Bench signature this fixes: intermittent tour-corner
  //    encoder freezes -> leg overshoot / heading corruption.
  if (duty == 0.0f) {
    if (!atZero_) {
      atZero_ = true;
      zeroSince_ = now;
    }
    dwelling_ = false;  // an explicit stop supersedes a pending dwell
    writeRawDuty(0.0f, lastTick_, /*stopping=*/true);
    return;
  }
  // 2. Deadband BOOST: a genuine sub-deadband command is raised to the
  //    floor, never zeroed (zero has its own meaning above).
  if (std::fabs(duty) < outputDeadband_) {
    duty = duty < 0.0f ? -outputDeadband_ : outputDeadband_;
  }
  const int sign = duty > 0.0f ? 1 : -1;
  // 3. Reversal dwell: on ANY sign change versus the last NONZERO
  //    command -- direct flip or through an intervening zero -- hold
  //    commanded zero until a full reversalDwell_ of zero time has
  //    elapsed, crediting time already spent at commanded zero.
  if (lastNonzeroSign_ != 0 && sign != lastNonzeroSign_) {
    if (!dwelling_) {
      dwelling_ = true;
      dwellStart_ = atZero_ ? zeroSince_ : now;
    }
    if (static_cast<float>(now - dwellStart_) < reversalDwell_) {
      writeRawDuty(0.0f, lastTick_, /*stopping=*/true);
      return;  // still holding; the new duty ships on a later tick
    }
    dwelling_ = false;
  }
  atZero_ = false;
  lastNonzeroSign_ = sign;
  writeRawDuty(duty, lastTick_, /*stopping=*/false);
}

void NezhaMotorPort::writeRawDuty(float duty, uint64_t now,  // [-1,1] [us]
                                  bool stopping) {
  duty = clampf(duty, -1.0f, 1.0f);

  // Sigma-delta quantizer to integer percent. The carry preserves
  // sub-percent resolution; it is DISCARDED on a commanded zero so a
  // stopped wheel cannot creep from accumulated remainder.
  int pct;
  if (stopping) {
    dutyCarry_ = 0.0f;
    pct = 0;
  } else if (speedPath_ == 1) {
    // speed_path 1 bypasses the dither: the 0x77 field is 1 deg/s wide
    // (0.087 %), so the sigma-delta carry has nothing to add.
    dutyCarry_ = 0.0f;
    pct = static_cast<int>(std::lround(duty * 100.0f));
  } else {
    float wanted = clampf(duty * 100.0f + dutyCarry_, -100.0f, 100.0f);
    pct = static_cast<int>(std::lround(wanted));
    dutyCarry_ = clampf(wanted - static_cast<float>(pct), -1.0f, 1.0f);
  }

  // stopNotTaken: a commanded zero re-writes while the wheel still
  // reads motion, regardless of the dedupe cache -- the brick latches
  // its last speed and one lost zero write is permanent.
  const bool stopNotTaken =
      pct == 0 && !stopHold_ && std::fabs(velocity_) > kStopConfirmVelocity;
  bool refreshOnly = false;  // speed_path 1: keep the timed run alive
  if (pct == lastWritten_ && !stopNotTaken) {
    if (speedPath_ == 1 && !stopping && pct != 0 && speedRunning_ &&
        now - lastTimedWrite_ >= kTimedRefresh) {
      refreshOnly = true;
    } else {
      return;
    }
  }

  // Min-write throttle -- stop writes always bypass it.
  if (!stopping && writeThrottle_ > 0.0f &&
      static_cast<float>(now - lastWriteTime_) < writeThrottle_) {
    return;
  }

  // Slew -- skipped for a stop and for the very first write (the
  // kNeverWritten sentinel through a clamp once produced a
  // wrong-direction first command, a wedge trigger).
  if (!stopping && lastWritten_ != kNeverWritten) {
    const int step = pct - lastWritten_;
    const int maxStep = static_cast<int>(slewRate_);
    if (step > maxStep) pct = lastWritten_ + maxStep;
    else if (step < -maxStep) pct = lastWritten_ - maxStep;
  }

  if (speedPath_ == 1 && (stopping || pct == 0)) {
    // Stop from speed is the hard stop; from at/below the kernel floor (or with
    // no timed run outstanding) it stays 0x60 0, falling through below.
    const int last = lastWritten_ == kNeverWritten ? 0 : lastWritten_;
    if (speedRunning_ && (last > kStopFromSpeedFloor ||
                          last < -kStopFromSpeedFloor)) {
      if (writeBrickFrame(0, kRegHardStop, 0, 0xF5, 0)) {
        lastWritten_ = 0;
        lastWriteTime_ = now;
        speedRunning_ = false;
        stopHold_ = true;  // brick stops itself; no stopNotTaken 0x60 0
      } else {
        connected_ = false;
      }
      return;
    }
    speedRunning_ = false;
    pct = 0;
  }

  if (speedPath_ == 1 && pct != 0) {
    stopHold_ = false;
    const int sp = pct * fwdSign_;
    const uint8_t dir = sp >= 0 ? kDirCw : kDirCcw;
    const int mag = sp >= 0 ? sp : -sp;
    const int store = static_cast<int>(
        std::lround(static_cast<float>(mag) * kDegPerSecPerPercent));
    // 0x77 only when this port's stored speed changes; a 0x77 alone
    // does not retarget (opcode doc section 3 rule 5), so 0x70 follows.
    if (!refreshOnly && store != lastSpeedStore_) {
      if (!writeBrickFrame(0, kRegSpeedStore,
                           static_cast<uint8_t>(store >> 8), 0,
                           static_cast<uint8_t>(store & 0xFF))) {
        connected_ = false;
        lastSpeedStore_ = -1;
        return;
      }
      lastSpeedStore_ = store;
    }
    if (writeBrickFrame(dir, kRegMove, 0, kMoveModeTimed, kTimedRunUnits)) {
      lastWritten_ = static_cast<int8_t>(pct);
      lastWriteTime_ = now;
      lastTimedWrite_ = now;
      speedRunning_ = true;
    } else {
      connected_ = false;
    }
    return;
  }

  if (!stopping) stopHold_ = false;  // a real run command: the 0x60 path
                                     // owns the wheel again
  const int signed_pct = pct * fwdSign_;
  const uint8_t direction = signed_pct >= 0 ? kDirCw : kDirCcw;
  const uint8_t magnitude =
      static_cast<uint8_t>(signed_pct >= 0 ? signed_pct : -signed_pct);
  if (writeFrame(direction, kRegMotorRun, magnitude)) {
    // Commit only on ACK: a NAK'd write retries next tick instead of
    // latching "already written".
    lastWritten_ = static_cast<int8_t>(pct);
    lastWriteTime_ = now;
  } else {
    connected_ = false;
  }
}

// ---- split-phase encoder -------------------------------------------

void NezhaMotorPort::requestSample() {
  // Select 0x46. The kernel spends the settle in sleepMillis(4); this
  // port is the brick's only client, so ordering holds structurally.
  if (!writeFrame(0x00, kRegEncoder, 0x00)) ++busFailCount_;
}

void NezhaMotorPort::collect(uint64_t now) {  // [us]
  int32_t raw = 0;
  if (readEncoderRaw(&raw)) {
    // Glitch/discontinuity plausibility decision, extracted to
    // encoder_glitch_armor.h -- see that header for the
    // kMaxDeltaCounts derivation. Ported from the reference driver
    // (see radio-robot docs/design/encoder-refresh-characterization.md
    // Phase F: a sample destroyed by interposed bus traffic reads as
    // raw 0; other corruption shows as a physically impossible jump).
    // Bench capture 2026-08-20: one such read produced a phantom -22 m
    // odometry teleport and a 3.5 M counts/s velocity spike that
    // instantly mis-terminated the tour's last leg.
    //
    // Captured BEFORE evaluate() mutates it -- the raw value this
    // armor last accepted as truth, i.e. what THIS tick's raw is about
    // to be compared against below.
    const int32_t previousGoodRaw = glitchArmor_.lastGoodRaw();
    const EncoderGlitchArmor::Decision decision = glitchArmor_.evaluate(raw);
    if (decision == EncoderGlitchArmor::Decision::kRejectPending) {
      // First implausible reading: held, awaiting a second consistent
      // one to disambiguate real motion from a counter discontinuity.
      ++glitchCount_;
      return;  // discarded: position/velocity/sampleTime all HOLD
    }
    if (decision == EncoderGlitchArmor::Decision::kAcceptAsRebaseline) {
      // Second consecutive self-consistent implausible reading: treat
      // it as the counter having restarted (e.g. a brick MCU reset),
      // not the wheel having moved. Re-anchor encOffset_ so THIS raw
      // value maps to the position already held (continuity) rather
      // than to zero -- the same offset technique rebaseline() below
      // uses, generalized from "map to 0" to "map to the current
      // position."
      //
      // Hold sampleTime_ here instead of falling through to the shared
      // accept-path advance, exactly as the raw-unchanged guard below
      // and the read-failure branch already do. This tick's true
      // velocity is UNKNOWN -- the armor just rejected the raw delta as
      // implausible -- and a confidently-fresh zero sample is what
      // DifferentialDrive::refreshSample() (the vendored kernel) feeds
      // straight to the velocity PID, which cannot tell "the wheel
      // stopped" from "we discarded an untrustworthy delta" and steps
      // the duty toward the rail. MEASURED gopiv 2026-09-02,
      // captures/gopiv-frozen-encoder-fix-20260902/notes.md: i2cf
      // incrementing alongside a wire-reported velocity of exactly 0,
      // five tours in six, every one traced to THIS branch rather than
      // the raw-unchanged one. The encOffset_ re-anchor above still
      // prevents the multi-m/s reintegration spike a genuine counter
      // restart would cause on the FOLLOWING tick, and
      // i2cFaultCount_ (core/diffdrive.cpp) still increments via the
      // same sampleTime-unchanged condition, so the event stays
      // visible in telemetry.
      encOffset_ = raw - static_cast<int32_t>(lastPosition_) * fwdSign_;
      ++rebaselineCount_;
      connected_ = true;
      lastPosition_ = static_cast<float>(raw - encOffset_) *
                       static_cast<float>(fwdSign_);  // == prior lastPosition_
      hasLastTick_ = true;
      return;  // sampleTime_ HOLDS -- velocity_ holds its prior value
    }
    connected_ = true;
    const float pos =
        static_cast<float>(raw - encOffset_) * static_cast<float>(fwdSign_);

    // A *successful* read whose raw counts are byte-identical to the
    // previous accepted sample while the wheel is under active drive
    // (appliedDuty() nonzero) is stale-but-acked data, not a real
    // stop -- one documented cause is an instant H-bridge sign flip
    // latching the 0x46 readback (the "encoder wedge" this class'
    // wedge detector below also watches for, at a slower multi-tick
    // threshold). MEASURED gopiv 2026-09-01,
    // captures/gopiv-profile-sweep-20260901/tour_tight.json frames
    // 185-191: posr frozen for one tick while driven, i2cf unmoved,
    // vr reported 0, duty stepped 3300->4500, wheel overshot to
    // 420 mm/s. Withhold the fresh sampleTime_ stamp exactly as the
    // read-failure branch below already does, so
    // DifferentialDrive::refreshSample()'s existing
    // sampleTime-unchanged gate holds the previously-derived velocity
    // instead of computing an honestly-derived-but-wrong zero from
    // stale data -- and, as a side effect, i2cFaultCount_
    // (core/diffdrive.cpp) now counts this tick too, since it
    // increments on that same sampleTime-unchanged condition. Gated on
    // the RAW value (not the offset-adjusted pos) and hasLastTick_ so
    // it can never fire on the very first sample this port ever takes.
    if (decision == EncoderGlitchArmor::Decision::kAccept && hasLastTick_ &&
        raw == previousGoodRaw && driven()) {
      lastPosition_ = pos;  // already equal to lastPosition_ by construction
      ++staleCount_;
      return;  // sampleTime_ HOLDS -- velocity_ holds its prior value
    }

    if (hasLastTick_) {
      const float dt =
          static_cast<float>(now - sampleTime_) / 1e6f;  // [s]
      if (dt > 0.0f) velocity_ = (pos - lastPosition_) / dt;
    }
    lastPosition_ = pos;
    sampleTime_ = now;  // stamped at collect SUCCESS only
    hasLastTick_ = true;
  } else {
    connected_ = false;  // sampleTime_ HOLDS -- age grows honestly
    ++busFailCount_;
  }
}

void NezhaMotorPort::tick(uint64_t now) {  // [us]
  lastTick_ = now;
  collect(now);
  if (moveLive_) {
    trackMove();  // the brick's loop owns the wheel: no 0x60 at all
  } else {
    writeShapedDuty(stagedDuty_, static_cast<uint32_t>(now / 1000));
    if (stopHold_ && std::fabs(velocity_) <= kStopConfirmVelocity)
      stopHold_ = false;
  }

  // Wedge detector: consecutive IDENTICAL position reads, raw and
  // unconditional; the suspect flavor additionally requires drive.
  if (lastPosition_ == lastWedgeCheckPosition_ && connected_) {
    ++identicalReads_;
    if (driven()) {
      ++identicalReadsDriven_;
      if (identicalReadsDriven_ > maxDrivenStreak_)
        maxDrivenStreak_ = identicalReadsDriven_;
    } else {
      identicalReadsDriven_ = 0;
    }
  } else {
    identicalReads_ = 0;
    identicalReadsDriven_ = 0;
  }
  lastWedgeCheckPosition_ = lastPosition_;
  wedgeLatched_ = identicalReads_ >= kWedgeThreshold;
  wedgeSuspect_ = identicalReadsDriven_ >= kWedgeThreshold;
}

// ---- readbacks ------------------------------------------------------

float NezhaMotorPort::position() const { return lastPosition_; }
float NezhaMotorPort::velocity() const { return velocity_; }

float NezhaMotorPort::appliedDuty() const {
  if (lastWritten_ == kNeverWritten) return 0.0f;
  return static_cast<float>(lastWritten_) / 100.0f;
}

void NezhaMotorPort::rebaseline() {
  // Software-only re-anchor: position() reads 0 from here, no bus
  // traffic, the device counter is untouched.
  encOffset_ = glitchArmor_.lastGoodRaw();
  lastPosition_ = 0.0f;
}

void NezhaMotorPort::configureWiring(uint8_t port, int8_t fwdSign) {
  if (port < 1 || port > 4) return;
  if (fwdSign != 1 && fwdSign != -1) return;
  if (port == port_ && fwdSign == fwdSign_) return;

  // Zero the port we are LEAVING, while we can still address it. The
  // brick latches its last commanded speed across even an MCU reset
  // (see this file's header), so a rewire that skipped this could
  // orphan a running wheel that nothing is able to command any more.
  emergencyStop();

  port_ = port;
  fwdSign_ = fwdSign;

  // The whole write pipeline describes the OLD motor: the dedupe cache
  // would suppress the first write to the new one, the sigma-delta
  // carry belongs to a different wheel's sub-percent history, and the
  // reversal dwell is mid-flight against a sign that no longer applies.
  // None of it transfers.
  stagedDuty_ = 0.0f;
  dutyCarry_ = 0.0f;
  lastWritten_ = kNeverWritten;
  lastWriteTime_ = 0;
  lastNonzeroSign_ = 0;
  atZero_ = false;
  zeroSince_ = 0;
  dwelling_ = false;
  dwellStart_ = 0;
  moveLive_ = false;  // emergencyStop() above already sent the hard stop
  stopHold_ = false;
  speedRunning_ = false;
  lastTimedWrite_ = 0;
  lastSpeedStore_ = -1;  // the 0x77 store is per port: unknown on the new one
  lastMove_ = 0.0f;
  moveRestTicks_ = 0;

  // Encoder state likewise. The new port has its own free-running
  // counter, so drop the offset measured against the old one and let
  // begin() re-anchor; keeping it would teleport odometry by whatever
  // the two counters happen to differ by. Also clears the wedge
  // detector, whose streak is evidence about a motor we no longer read.
  lastPosition_ = 0.0f;
  velocity_ = 0.0f;
  sampleTime_ = 0;
  lastTick_ = 0;
  hasLastTick_ = false;
  connected_ = false;
  lastWedgeCheckPosition_ = 0.0f;
  identicalReads_ = 0;
  identicalReadsDriven_ = 0;
  wedgeLatched_ = false;
  wedgeSuspect_ = false;
  glitchArmor_ = EncoderGlitchArmor();

  begin();
}

void NezhaMotorPort::configureShaping(float outputDeadband,
                                      float reversalDwell, float slewRate,
                                      float writeThrottle) {
  outputDeadband_ = outputDeadband;
  reversalDwell_ = reversalDwell;
  slewRate_ = slewRate;
  writeThrottle_ = writeThrottle;
}

// Board-generic diag hook: ordinal -> field, host-portable (no I2C, no
// pxt.h). See maxDrivenStreak_/glitchCount_/rebaselineCount_ above and
// wiredPort()/wiredSign()/rawCount() in nezha_port.h for what each
// ordinal reads.
int nezhaBoardDiagValue(const NezhaMotorPort& left, const NezhaMotorPort& right,
                        int ordinal) {
  switch (ordinal) {
    case 21: return static_cast<int>(left.maxDrivenStreak_);
    case 22: return static_cast<int>(right.maxDrivenStreak_);
    case 23: return static_cast<int>(left.glitchCount_);
    case 24: return static_cast<int>(right.glitchCount_);
    case 27:
      return static_cast<int>(left.rebaselineCount_ + right.rebaselineCount_);
    case 35: return static_cast<int>(left.wiredPort());
    case 36: return static_cast<int>(left.wiredSign());
    case 37: return static_cast<int>(right.wiredPort());
    case 38: return static_cast<int>(right.wiredSign());
    case 39: return static_cast<int>(left.rawCount());
    case 40: return static_cast<int>(right.rawCount());
    case 52: return static_cast<int>(left.busFailCount_ + right.busFailCount_);
    case 53: return static_cast<int>(left.staleCount_ + right.staleCount_);
    case 54:
      return static_cast<int>(left.glitchCount_ + right.glitchCount_ +
                              left.rebaselineCount_ + right.rebaselineCount_);
    default: return 0;
  }
}

}  // namespace diffDrive
