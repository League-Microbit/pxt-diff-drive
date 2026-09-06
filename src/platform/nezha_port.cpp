// nezha_port.cpp -- see nezha_port.h. Ported from the firmware's
// nezha_motor.cpp; the shaping-stage ORDER is load-bearing.
#include "nezha_port.h"

#include "vfp_guard.h"

#include <cmath>

namespace diffDrive {

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
extern "C" void diffdrive_emergency_motor_stop() {
  // Same frame shape as NezhaMotorPort::writeFrame(), inlined so this
  // needs no instance: {0xFF, 0xF9, port, arg, reg, val, 0xF5, 0x00}.
  for (uint8_t port = 1; port <= 2; ++port) {
    uint8_t frame[8] = {0xFF, 0xF9, port, NezhaMotorPort::kDirCw,
                        NezhaMotorPort::kRegMotorRun, 0x00, 0xF5, 0x00};
#if MICROBIT_CODAL
    uBit.i2c.write(NezhaMotorPort::kAddress << 1, frame, 8);
#else
    uBit.i2c.write(NezhaMotorPort::kAddress << 1,
                   reinterpret_cast<char*>(frame), 8);
#endif
  }
}

// ---- fail-safe fault handlers ---------------------------------------
//
// These OVERRIDE the weak defaults in codal-nrf52's
// gcc_startup_nrf52833.S, every one of which is an infinite loop. That
// default is what turned a fault into a runaway robot: the CPU stopped,
// the display stayed blank, every fiber died, and the brick held its
// last motor command indefinitely (MEASURED tigez 2026-08-30 -- see
// diffdrive_emergency_motor_stop() above for the full forensics).
//
// Order matters:
//   1. STOP THE MOTORS -- before anything else can go wrong. A reset
//      alone would leave the brick driving, because the Rig (and its
//      boot zero-write) is only created on the first motion command.
//   2. REPORT -- print the fault site so a wedge is diagnosable from a
//      serial log instead of needing a debugger on the wedged chip.
//   3. RESET -- the board comes back on its own.
//
// Nothing here allocates, takes a lock, or yields: a fault handler must
// assume memory is already corrupt.
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


namespace {
float clampf(float value, float lo, float hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}
}  // namespace

// ---- bus primitives -------------------------------------------------

bool NezhaMotorPort::writeFrame(uint8_t arg, uint8_t reg, uint8_t val) {
  uint8_t frame[8] = {0xFF, 0xF9, port_, arg, reg, val, 0xF5, 0x00};
  // codal-microbit-v2 (V2) I2C takes uint8_t*; classic DAL (V1) takes char*.
#if MICROBIT_CODAL
  int status = uBit.i2c.write(kAddress << 1, frame, 8);
#else
  int status = uBit.i2c.write(kAddress << 1,
                              reinterpret_cast<char*>(frame), 8);
#endif
  return status == MICROBIT_OK;
}

bool NezhaMotorPort::readEncoderRaw(int32_t* raw) {
  uint8_t data[4] = {0, 0, 0, 0};
#if MICROBIT_CODAL
  int status = uBit.i2c.read(kAddress << 1, data, 4);
#else
  int status = uBit.i2c.read(kAddress << 1,
                             reinterpret_cast<char*>(data), 4);
#endif
  if (status != MICROBIT_OK) return false;
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
  stagedDuty_ = clampf(duty, -1.0f, 1.0f);
}

void NezhaMotorPort::emergencyStop() {
  // The one call that must not depend on a healthy tick(): zero the
  // stage AND write zero through the never-shaped stop path now.
  stagedDuty_ = 0.0f;
  writeShapedDuty(0.0f, static_cast<uint32_t>(lastTick_ / 1000));
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
  } else {
    float wanted = clampf(duty * 100.0f + dutyCarry_, -100.0f, 100.0f);
    pct = static_cast<int>(std::lround(wanted));
    dutyCarry_ = clampf(wanted - static_cast<float>(pct), -1.0f, 1.0f);
  }

  // stopNotTaken: a commanded zero re-writes while the wheel still
  // reads motion, regardless of the dedupe cache -- the brick latches
  // its last speed and one lost zero write is permanent.
  const bool stopNotTaken =
      pct == 0 && std::fabs(velocity_) > kStopConfirmVelocity;
  if (pct == lastWritten_ && !stopNotTaken) return;

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
  writeFrame(0x00, kRegEncoder, 0x00);
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
        raw == previousGoodRaw &&
        std::fabs(appliedDuty()) > kMotionThreshold) {
      lastPosition_ = pos;  // already equal to lastPosition_ by construction
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
  }
}

void NezhaMotorPort::tick(uint64_t now) {  // [us]
  lastTick_ = now;
  collect(now);
  writeShapedDuty(stagedDuty_, static_cast<uint32_t>(now / 1000));

  // Wedge detector: consecutive IDENTICAL position reads, raw and
  // unconditional; the suspect flavor additionally requires drive.
  if (lastPosition_ == lastWedgeCheckPosition_ && connected_) {
    ++identicalReads_;
    if (std::fabs(appliedDuty()) > kMotionThreshold) {
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

void NezhaMotorPort::configureShaping(float outputDeadband,
                                      float reversalDwell, float slewRate,
                                      float writeThrottle) {
  outputDeadband_ = outputDeadband;
  reversalDwell_ = reversalDwell;
  slewRate_ = slewRate;
  writeThrottle_ = writeThrottle;
}

}  // namespace diffDrive
