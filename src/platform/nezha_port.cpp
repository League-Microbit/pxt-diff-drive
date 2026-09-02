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
// WHY THIS EXISTS. MEASURED tigez 2026-08-30 (pyOCD halt on the wedged
// chip; full forensics in captures/tigez-cal-20260830/): a radio-path
// memory corruption makes controlStep()
// dereference a pointer holding the ASCII bytes "PING", taking a
// precise bus error (CFSR 0x8200, BFAR 0x474E4988). The board then sits
// in the DEFAULT weak HardFault_Handler -- an infinite loop in
// codal-nrf52's gcc_startup_nrf52833.S:303 -- so nothing panics,
// nothing reboots, and THE BRICK KEEPS ITS LAST MOTOR COMMAND. The
// wheels run until someone reflashes the board. This function is what
// makes that impossible.
//
// UPDATE 2026-09-01 -- the "memory corruption" above is probably not
// heap corruption. A second fault with the IDENTICAL CFSR 0x8200 was
// root-caused on gopiv that day: CODAL's context switch saves no VFP
// registers, GCC parks pointers in the callee-saved bank s16-s31, and a
// fiber switch destroys them. A pointer restored from a clobbered FPU
// register explains a dereference of "PING"-looking bytes with no heap
// corruption at all. Anyone reading a fault here should suspect that
// first: decode BFAR as a float and as ASCII before calling it garbage.
// See the yield-discipline invariant in this package's design notes.
//
// UPDATE 2026-09-02 -- RESOLVED. The VFP-register-clobber theory above
// was confirmed by retest: the radio-during-motion fault this whole
// comment describes no longer reproduces. MEASURED tigez 2026-09-02
// (28 radio-hammer trials -- PING hammered continuously over the radio
// relay while MOVE_X pivots ran over USB, 14 trials on each of two
// builds -- plus 6 radio-silent negative-control trials, 0 reset
// signatures across all of them, full per-trial transcripts in
// captures/tigez-radio-retest-20260902/): the guarded yield fix that
// closes the register-clobber window (this file's vfp_guard.h) holds
// even on the build that predates the emit-queue work, i.e. the guard
// alone is what stops the fault, not anything downstream of it. This
// function and the handlers below stay regardless -- a fault handler
// that fails safe is correct even against a fault that no longer fires.
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
  // 2026-08-30: pyOCD showed IPSR=3, PC parked in the handler). The
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
  // Bus-hang guard (sprint 010 ticket 004 investigation -- see
  // clasi/sprints/010-.../tickets/004-...md for the full written
  // finding). This project's actual resolved build (codal-microbit-v2
  // v0.3.5, confirmed via .tmp/deploy-head/built/codal.json after a
  // real `pxt build`) pins codal-nrf52 commit 1fbb724, which is a
  // confirmed descendant of BOTH upstream fixes the issue's research
  // flagged: "NRF52I2C: Introduce transaction timeout" (2021-06-30) and
  // "NRF52I2C::waitForStop: recover from hang" (2022-01/04). So
  // writeFrame()/readEncoderRaw() below can no longer hang forever --
  // codal-nrf52's own NRF52I2C::waitForStop() bounds one stuck
  // transaction to ~NRF52I2C_TIMEOUT10US (1,000,000 x 10us = ~10s)
  // before it force-recovers the bus, plus up to
  // ~NRF52I2C_TIMEOUT10US_STOP (~1s) waiting for that recovery's own
  // STOP to land -- roughly 11s worst case PER CALL, not infinite. That
  // is a real, confirmed platform-level bound; it is NOT confirmed to
  // be the exact path a genuinely unpowered (vs. mid-transaction-wedged)
  // brick hits in practice -- an unpowered device more plausibly NACKs
  // fast (NRF_TWIM_EVENT_ERROR fires immediately, checked every spin),
  // which is a separate, much cheaper failure path through the same
  // function. Only a bench check with a real dead brick (ticket 005)
  // can settle which path fires.
  //
  // Given that ~11s-per-call ceiling is real either way, the original
  // loop's "try all 3 samples regardless of an earlier failure" shape
  // multiplies a bad worst case: up to 3 sequential hard failures per
  // motor (~33s), up to 6 across both wheels in
  // DifferentialDrive::begin() (~66s) before ever reporting
  // connected()==false. Stopping at the FIRST hard failure (write or
  // read) caps this motor's own worst case to one attempt (~11-22s)
  // instead of three -- a real, bounded, honest delay, not the
  // "silent, unbounded hang" the issue described, though still not
  // fast. Trade-off, stated plainly: this also removes the old loop's
  // tolerance for a single transient blip mid-sequence (e.g. sample 1
  // NACKs on a cold-boot brownout but samples 2-3 would have been
  // fine) -- previously that still produced a good median-of-2 boot;
  // now it reports connected()==false on that motor. No bench evidence
  // either way yet; ticket 005 should watch for it.
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
  writeShapedDuty(0.0f, static_cast<uint32_t>(lastTickUs_ / 1000));
}

void NezhaMotorPort::writeShapedDuty(float duty, uint32_t nowMs) {
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
      zeroSinceMs_ = nowMs;
    }
    dwelling_ = false;  // an explicit stop supersedes a pending dwell
    writeRawDuty(0.0f, lastTickUs_, /*stopping=*/true);
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
      dwellStart_ = atZero_ ? zeroSinceMs_ : nowMs;
    }
    if (static_cast<float>(nowMs - dwellStart_) < reversalDwell_) {
      writeRawDuty(0.0f, lastTickUs_, /*stopping=*/true);
      return;  // still holding; the new duty ships on a later tick
    }
    dwelling_ = false;
  }
  atZero_ = false;
  lastNonzeroSign_ = sign;
  writeRawDuty(duty, lastTickUs_, /*stopping=*/false);
}

void NezhaMotorPort::writeRawDuty(float duty, uint64_t nowUs,
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
  if (pct == lastWrittenPct_ && !stopNotTaken) return;

  // Min-write throttle -- stop writes always bypass it.
  if (!stopping && writeThrottle_ > 0.0f &&
      static_cast<float>(nowUs - lastWriteTimeUs_) < writeThrottle_) {
    return;
  }

  // Slew -- skipped for a stop and for the very first write (the
  // kNeverWritten sentinel through a clamp once produced a
  // wrong-direction first command, a wedge trigger).
  if (!stopping && lastWrittenPct_ != kNeverWritten) {
    const int step = pct - lastWrittenPct_;
    const int maxStep = static_cast<int>(slewRate_);
    if (step > maxStep) pct = lastWrittenPct_ + maxStep;
    else if (step < -maxStep) pct = lastWrittenPct_ - maxStep;
  }

  const int signed_pct = pct * fwdSign_;
  const uint8_t direction = signed_pct >= 0 ? kDirCw : kDirCcw;
  const uint8_t magnitude =
      static_cast<uint8_t>(signed_pct >= 0 ? signed_pct : -signed_pct);
  if (writeFrame(direction, kRegMotorRun, magnitude)) {
    // Commit only on ACK: a NAK'd write retries next tick instead of
    // latching "already written".
    lastWrittenPct_ = static_cast<int8_t>(pct);
    lastWriteTimeUs_ = nowUs;
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

void NezhaMotorPort::collect(uint64_t nowUs) {
  int32_t raw = 0;
  if (readEncoderRaw(&raw)) {
    // Glitch/discontinuity plausibility decision, extracted to
    // encoder_glitch_armor.h (sprint 006 ticket 005 -- see that header
    // for the kMaxDeltaCounts derivation and code review R-07/KERN-07).
    // Ported from the reference driver (see radio-robot
    // docs/design/encoder-refresh-characterization.md Phase F: a sample
    // destroyed by interposed bus traffic reads as raw 0; other
    // corruption shows as a physically impossible jump). Bench capture
    // 2026-08-20: one such read produced a phantom -22 m odometry
    // teleport and a 3.5 M counts/s velocity spike that instantly
    // mis-terminated the tour's last leg.
    const EncoderGlitchArmor::Decision decision = glitchArmor_.evaluate(raw);
    if (decision == EncoderGlitchArmor::Decision::kRejectPending) {
      // First implausible reading: held, awaiting a second consistent
      // one to disambiguate real motion from a counter discontinuity.
      ++glitchCount_;
      return;  // discarded: position/velocity/sampleTime all HOLD
    }
    if (decision == EncoderGlitchArmor::Decision::kAcceptAsRebaseline) {
      // Second consecutive self-consistent implausible reading: R-07's
      // fix. The pre-extraction code accepted this as a real ~4 m jump
      // (the "hand-rotation re-sync" path); this is now a THIRD outcome
      // instead -- treat it as the counter having restarted (e.g. a
      // brick MCU reset), not the wheel having moved. Re-anchor
      // encOffset_ so THIS raw value maps to the position already held
      // (continuity) rather than to zero -- the same offset technique
      // rebaseline() below uses, generalized from "map to 0" to "map to
      // the current position." Falls through to the shared accept-path
      // computation below, using the freshly re-anchored offset; since
      // pos comes out equal to lastPosition_ by construction, velocity_
      // for this tick correctly reads ~0 rather than the multi-m/s
      // spike the old behavior produced.
      encOffset_ = raw - static_cast<int32_t>(lastPosition_) * fwdSign_;
      ++rebaselineCount_;
    }
    connected_ = true;
    const float pos =
        static_cast<float>(raw - encOffset_) * static_cast<float>(fwdSign_);
    if (hasLastTick_) {
      const float dt =
          static_cast<float>(nowUs - sampleTimeUs_) / 1e6f;  // [s]
      if (dt > 0.0f) velocity_ = (pos - lastPosition_) / dt;
    }
    lastPosition_ = pos;
    sampleTimeUs_ = nowUs;  // stamped at collect SUCCESS only
    hasLastTick_ = true;
  } else {
    connected_ = false;  // sampleTimeUs_ HOLDS -- age grows honestly
  }
}

void NezhaMotorPort::tick(uint64_t nowUs) {
  lastTickUs_ = nowUs;
  collect(nowUs);
  writeShapedDuty(stagedDuty_, static_cast<uint32_t>(nowUs / 1000));

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
  if (lastWrittenPct_ == kNeverWritten) return 0.0f;
  return static_cast<float>(lastWrittenPct_) / 100.0f;
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
