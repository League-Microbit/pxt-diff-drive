// differential_drive.cpp — DiffDrive::DifferentialDrive implementation.
// Vendored from League-Robotics/radio-robot
// src/firm/diffdrive/differential_drive.cpp (namespace/include changes
// only) — see src/DESIGN.md §2 for the full provenance statement,
// current path, and the one known local divergence (cycleGapCount).
// Fix bugs in both trees until the firmware consumes this package
// directly.
#include "diffdrive.h"

#include <algorithm>
#include <cmath>

namespace DiffDrive {

namespace {

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool isFinite(float v) { return std::isfinite(v); }

bool allFinite(const DiffDrive::DifferentialDrive::Config& c) {
  const float scalars[] = {
      c.maxDuty, c.fullDutyVelocity, c.kp, c.ki, c.iMax, c.kaff, c.pidMax,
      c.twistHoldGain, c.vMin, c.posErrMax, c.biasMax, c.tauAdapt, c.aSteady,
      c.deficitThreshold, c.deficitWindow, c.stallSpeed, c.stallDemand,
      c.stallWindow, c.crawlPulse,
  };
  for (float v : scalars) {
    if (!std::isfinite(v)) return false;
  }
  for (int i = 0; i < 2; ++i) {
    for (int j = 0; j < 2; ++j) {
      if (!std::isfinite(c.wheelGain[i][j])) return false;
      if (!std::isfinite(c.wheelIntercept[i][j])) return false;
    }
  }
  return true;
}

}  // namespace

DifferentialDrive::DifferentialDrive(Motor& left, Motor& right,
                                     const Clock& clock,
                                     Sleeper& sleeper,
                                     FiberLauncher& launcher)
    : left_(left), right_(right), clock_(clock), sleeper_(sleeper),
      launcher_(launcher) {}

DifferentialDrive& DifferentialDrive::setMaxDuty(float maxDuty) {
  if (!isFinite(maxDuty)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.maxDuty = maxDuty;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setFullDutyVelocity(float velocity) {
  if (!isFinite(velocity)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.fullDutyVelocity = velocity;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKp(float kp) {
  if (!isFinite(kp)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.kp = kp;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKi(float ki) {
  if (!isFinite(ki)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.ki = ki;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setIMax(float iMax) {
  if (!isFinite(iMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.iMax = iMax;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setKaff(float kaff) {
  if (!isFinite(kaff)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.kaff = kaff;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setPidMax(float pidMax) {
  if (!isFinite(pidMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.pidMax = pidMax;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setTwistHoldGain(float gain) {
  if (!isFinite(gain)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.twistHoldGain = gain;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setWheelCorrection(
    float gainLeftAccel, float interceptLeftAccel, float gainLeftDecel,
    float interceptLeftDecel, float gainRightAccel, float interceptRightAccel,
    float gainRightDecel, float interceptRightDecel) {
  if (!isFinite(gainLeftAccel) || !isFinite(interceptLeftAccel) ||
      !isFinite(gainLeftDecel) || !isFinite(interceptLeftDecel) ||
      !isFinite(gainRightAccel) || !isFinite(interceptRightAccel) ||
      !isFinite(gainRightDecel) || !isFinite(interceptRightDecel)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.wheelGain[0][0] = gainLeftAccel;
  staged_.wheelIntercept[0][0] = interceptLeftAccel;
  staged_.wheelGain[0][1] = gainLeftDecel;
  staged_.wheelIntercept[0][1] = interceptLeftDecel;
  staged_.wheelGain[1][0] = gainRightAccel;
  staged_.wheelIntercept[1][0] = interceptRightAccel;
  staged_.wheelGain[1][1] = gainRightDecel;
  staged_.wheelIntercept[1][1] = interceptRightDecel;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setSpeedFloor(float vMin) {
  if (!isFinite(vMin)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.vMin = (vMin > 0.0f) ? vMin : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setPositionErrorMax(float posErrMax) {
  if (!isFinite(posErrMax)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.posErrMax = (posErrMax > 0.0f) ? posErrMax : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setAdaptation(float biasMax,
                                                    float tauAdapt,
                                                    float aSteady) {
  if (!isFinite(biasMax) || !isFinite(tauAdapt) || !isFinite(aSteady)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.biasMax = biasMax;
  staged_.tauAdapt = tauAdapt;
  staged_.aSteady = (aSteady > 0.0f) ? aSteady : 0.0f;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setDeficit(float threshold,
                                                 float window) {
  if (!isFinite(threshold) || !isFinite(window)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.deficitThreshold = threshold;
  staged_.deficitWindow = window;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setStall(float speed, float demand,
                                               float window) {
  if (!isFinite(speed) || !isFinite(demand) || !isFinite(window)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.stallSpeed = speed;
  staged_.stallDemand = demand;
  staged_.stallWindow = window;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setLambdaEnabled(bool enabled) {
  staged_.lambdaEnabled = enabled;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setCrawlPulse(float crawlPulse) {
  if (!isFinite(crawlPulse)) {
    noteRefusal(Status::kRefusedNonFinite);
    return *this;
  }
  staged_.crawlPulse = crawlPulse;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive& DifferentialDrive::setCyclePeriod(uint32_t period) {
  if (begun_) {
    noteRefusal(Status::kCadencePreserved);
    return *this;
  }
  staged_.cyclePeriod = period;
  ++cfgSeq_;
  return *this;
}

DifferentialDrive::Status DifferentialDrive::setConfig(const Config& config) {
  if (!allFinite(config)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  const bool cadenceDiffers =
      begun_ && config.cyclePeriod != staged_.cyclePeriod;
  const uint32_t frozen = staged_.cyclePeriod;
  staged_ = config;
  if (cadenceDiffers) {
    staged_.cyclePeriod = frozen;
    ++cfgSeq_;
    noteRefusal(Status::kCadencePreserved);
    return Status::kCadencePreserved;
  }
  ++cfgSeq_;
  return Status::kOk;
}

DifferentialDrive::Config DifferentialDrive::config() const {
  return staged_;
}

DifferentialDrive::Status DifferentialDrive::begin() {
  left_.begin();
  right_.begin();
  stopEnforceCountdown_ = kStopEnforceTicks;
  begun_ = true;
  if (staged_.maxDuty <= 0.0f) {
    noteRefusal(Status::kRefusedUnconfigured);
    return Status::kRefusedUnconfigured;
  }
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::start() {
  if (!begun_) {
    noteRefusal(Status::kRefusedNotBegun);
    return Status::kRefusedNotBegun;
  }
  if (running_) return Status::kOk;  // idempotent
  running_ = true;
  launcher_.launch(&DifferentialDrive::fiberEntry, this);
  return Status::kOk;
}

void DifferentialDrive::fiberEntry(void* self) {
  static_cast<DifferentialDrive*>(self)->run();
}

void DifferentialDrive::run() {
  while (true) {
    const uint64_t cycleStartUs = clock_.nowMicros();
    step();
    const uint64_t deadlineUs =
        cycleStartUs + static_cast<uint64_t>(active_.cyclePeriod) * 1000ull;
    const uint64_t nowUs = clock_.nowMicros();
    if (nowUs < deadlineUs) {
      const uint32_t shortfall =
          static_cast<uint32_t>((deadlineUs - nowUs + 999) / 1000);  // [ms]
      sleeper_.sleepMillis(shortfall);
    } else {
      ++cycleOverrunCount_;
      sleeper_.yield();
    }
  }
}

DifferentialDrive::Status DifferentialDrive::checkCommandable(
    bool needsVelocityCalibration) const {
  if (!begun_) return Status::kRefusedNotBegun;
  if (estopLatch_) return Status::kRefusedEstopped;
  if (staged_.maxDuty <= 0.0f) return Status::kRefusedUnconfigured;
  if (needsVelocityCalibration && staged_.fullDutyVelocity <= 0.0f) {
    return Status::kRefusedUnconfigured;
  }
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::drive(float velocity, float twist,
                                                   uint32_t lease) {
  if (!isFinite(velocity) || !isFinite(twist)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  const Status gate = checkCommandable(/*needsVelocityCalibration=*/true);
  if (gate != Status::kOk) {
    noteRefusal(gate);
    return gate;
  }
  Command c;
  c.mode = kModeVelocity;
  c.velocity = velocity;
  c.twist = twist;
  c.validUntil = static_cast<uint32_t>(clock_.nowMicros() / 1000) +
                 (lease > kLeaseMax ? kLeaseMax : lease);
  command_ = c;
  ++cmdSeq_;
  return Status::kOk;
}

DifferentialDrive::Status DifferentialDrive::driveDuty(float dutyLeft,
                                                       float dutyRight,
                                                       uint32_t lease) {
  if (!isFinite(dutyLeft) || !isFinite(dutyRight)) {
    noteRefusal(Status::kRefusedNonFinite);
    return Status::kRefusedNonFinite;
  }
  const Status gate = checkCommandable(/*needsVelocityCalibration=*/false);
  if (gate != Status::kOk) {
    noteRefusal(gate);
    return gate;
  }
  Command c;
  c.mode = kModeRawDuty;
  c.dutyLeft = dutyLeft;
  c.dutyRight = dutyRight;
  c.validUntil = static_cast<uint32_t>(clock_.nowMicros() / 1000) +
                 (lease > kLeaseMax ? kLeaseMax : lease);
  command_ = c;
  ++cmdSeq_;
  return Status::kOk;
}

void DifferentialDrive::neutral() {
  Command c;  // mode defaults to kModeNeutral; no lease needed to be stopped
  command_ = c;
  ++cmdSeq_;
}

void DifferentialDrive::estop() {
  estopLatch_ = true;
}

void DifferentialDrive::estopClear() {
  estopLatch_ = false;
}

void DifferentialDrive::emergencyStopMotors() {
  estopLatch_ = 1;
  left_.emergencyStop();
  right_.emergencyStop();
}

void DifferentialDrive::clearStallLatch() {
  ++clearStallReq_;
}

void DifferentialDrive::rebasePosition() {
  ++rebaseReq_;
}

void DifferentialDrive::rearmReferences() {
  ++rearmReq_;
}

DifferentialDrive::Output DifferentialDrive::output() const {
  Output copy;
  uint32_t s1, s2;
  do {
    s1 = outSeq_;
    copy = out_;
    s2 = outSeq_;
  } while (s1 != s2 || (s1 & 1u));
  return copy;
}

void DifferentialDrive::snapshotConfig() {
  const uint32_t seq = cfgSeq_;
  if (seq == activeCfgSeq_) return;
  active_ = staged_;
  activeCfgSeq_ = seq;
}

DifferentialDrive::Command DifferentialDrive::snapshotCommand() const {
  Command copy;
  uint32_t s1, s2;
  do {
    s1 = cmdSeq_;
    copy = command_;
    s2 = cmdSeq_;
  } while (s1 != s2);
  return copy;
}

void DifferentialDrive::step() {
  const uint64_t cycleStartUs = clock_.nowMicros();
  const uint32_t nowMs = static_cast<uint32_t>(cycleStartUs / 1000);

  // A gap far longer than the control period means this kernel was NOT
  // being stepped -- the caller was idle between moves -- not that one
  // enormous control cycle elapsed. Integrating across such a gap is a
  // measured runaway: positionError() advances its reference by
  // speed*dt, so the first step after a 70 s idle injected roughly
  // 40,000 counts of phantom position error in a single tick and
  // pinned duty at 100%. Observed on vevov 2026-08-21 as a violent
  // high-speed lurch on the first pivot after any pause -- the robot
  // turned at ~440 deg/s against a commanded 45, and the move
  // "completed" in 17 ticks because it blew through its encoder target.
  //
  // Treating the gap as a fresh start is exactly what the very first
  // cycle already does (measuredPeriodUs == 0 -> dt == 0), and
  // positionError()/adaptBias() both re-anchor rather than integrate
  // when dt <= 0. So the correct handling is to reuse that path.
  static constexpr uint32_t kMaxCycleGapUs = 250000;  // [us] ~10 cycles
  uint32_t measuredPeriodUs =
      everCycled_ ? static_cast<uint32_t>(cycleStartUs - previousCycleStartUs_)
                  : 0u;
  if (measuredPeriodUs > kMaxCycleGapUs) {
    measuredPeriodUs = 0u;  // re-anchor the integrators, do not integrate
    ++cycleGapCount_;
  }
  previousCycleStartUs_ = cycleStartUs;
  everCycled_ = true;
  const float dt = static_cast<float>(measuredPeriodUs) * 1e-6f;  // [s]
  ++cycleCount_;

  snapshotConfig();

  if (clearStallReq_ != seenClearStallReq_) {
    seenClearStallReq_ = clearStallReq_;
    stallHalted_ = false;
    stallLatched_ = false;
    stallSince_ = 0;
  }
  if (rebaseReq_ != seenRebaseReq_) {
    seenRebaseReq_ = rebaseReq_;
    left_.rebaseline();
    right_.rebaseline();
    ++epoch_;  // every integrator re-anchors; sample caches restart
    ++positionEpochLeft_;
    ++positionEpochRight_;
    sampleLeft_ = WheelSample{};
    sampleRight_ = WheelSample{};
  }
  if (rearmReq_ != seenRearmReq_) {
    // K4 (design §4.5): disarm both position references and the twist
    // reference -- controlStep() below re-anchors whichever of them is
    // still active THIS SAME step(), at the wheels' current measured
    // position, with no wasted neutral tick and no epoch bump (the
    // wheel samples and epoch_ are untouched; only the "armed" latches
    // that gate re-anchoring are cleared).
    seenRearmReq_ = rearmReq_;
    posRefLeft_.armed = false;
    posRefRight_.armed = false;
    twistRef_.armed = false;
  }

  const Command cmd = snapshotCommand();

  const bool needsLease = cmd.mode != kModeNeutral;
  const bool leaseLive =
      needsLease && static_cast<int32_t>(cmd.validUntil - nowMs) > 0;
  const bool leaseExpired = needsLease && !leaseLive;
  if (leaseWasLive_ && leaseExpired) ++leaseExpiryCount_;
  leaseWasLive_ = leaseLive;

  uint8_t effective = cmd.mode;
  if (leaseExpired) effective = kModeNeutral;
  if (stallHalted_) effective = kModeNeutral;
  if (estopLatch_) effective = kModeNeutral;
  if (effective == kModeVelocity && active_.fullDutyVelocity <= 0.0f) {
    effective = kModeNeutral;
  }

  const bool halted = estopLatch_ || stallHalted_;
  if (halted && !wasForcedStop_) resetAdaptiveState();
  wasForcedStop_ = halted;

  controlStep(cmd, effective, dt, nowMs);

  left_.requestSample();
  sleeper_.sleepMillis(kSettle);
  left_.tick(clock_.nowMicros());

  right_.requestSample();
  sleeper_.sleepMillis(kSettle);
  right_.tick(clock_.nowMicros());

  const uint64_t stampLeftBefore = sampleLeft_.sampleTime;
  const uint64_t stampRightBefore = sampleRight_.sampleTime;
  refreshSample(left_, sampleLeft_);
  refreshSample(right_, sampleRight_);
  if (sampleLeft_.sampleTime == stampLeftBefore ||
      sampleRight_.sampleTime == stampRightBefore) {
    ++i2cFaultCount_;
  }
  // K2 (design §4.5): remember, per wheel, whether THIS cycle's own
  // collect actually advanced the cached sample. Consumed at the TOP
  // of the NEXT step()'s controlStep() call -- the earliest point that
  // step's own positionError() can see the sample this collect just
  // produced, since controlStep() always runs before this cycle's own
  // requestSample()/tick() pair.
  sampleAdvancedLeft_ = sampleLeft_.sampleTime != stampLeftBefore;
  sampleAdvancedRight_ = sampleRight_.sampleTime != stampRightBefore;

  const uint64_t busyEndUs = clock_.nowMicros();
  publishOutput(nowMs, cycleStartUs, busyEndUs, measuredPeriodUs,
                leaseExpired);
}

void DifferentialDrive::controlStep(const Command& cmd, uint8_t effectiveMode,
                                    float dt, uint32_t nowMs) {
  const float rail = active_.maxDuty * 0.01f;  // [-1,1] duty fraction

  if (effectiveMode == kModeNeutral) {
    dutyDemandLeft_ = 0.0f;
    dutyDemandRight_ = 0.0f;
    satLeft_ = false;
    satRight_ = false;
    lambda_ = 1.0f;
    twistRef_.armed = false;
    lastSpeedLeft_ = 0.0f;
    lastSpeedRight_ = 0.0f;
    previousTargetLeft_ = 0.0f;
    previousTargetRight_ = 0.0f;
    cmdAccelLeft_ = 0.0f;
    cmdAccelRight_ = 0.0f;
    lastPidLeft_ = 0.0f;
    lastPidRight_ = 0.0f;
    updateLatch(false, active_.stallWindow, nowMs, stallSince_, stallLatched_);
    updateLatch(false, active_.deficitWindow, nowMs, deficitSinceLeft_,
                deficitLeft_);
    updateLatch(false, active_.deficitWindow, nowMs, deficitSinceRight_,
                deficitRight_);
    posRefLeft_.armed = false;
    posRefRight_.armed = false;
    stageStop();
    return;
  }

  if (effectiveMode == kModeRawDuty) {
    const float demandL = cmd.dutyLeft * 0.01f;
    const float demandR = cmd.dutyRight * 0.01f;
    dutyDemandLeft_ = demandL;
    dutyDemandRight_ = demandR;
    satLeft_ = std::fabs(demandL) > rail;
    satRight_ = std::fabs(demandR) > rail;
    lambda_ = 1.0f;
    twistRef_.armed = false;
    stageDuty(clampf(demandL, -rail, rail), clampf(demandR, -rail, rail));
    return;
  }

  const float dutyPerSpeed = 1.0f / active_.fullDutyVelocity;  // [1/(counts/s)]

  const float rawLeft = cmd.velocity - cmd.twist;
  const float rawRight = cmd.velocity + cmd.twist;

  const float demandMagLeft = std::fabs(dutyDemandLeft_);
  const float demandMagRight = std::fabs(dutyDemandRight_);
  satLeft_ = demandMagLeft > rail;
  satRight_ = demandMagRight > rail;
  if (!active_.lambdaEnabled) {
    lambda_ = 1.0f;
  } else {
    float lambdaInstant = 1.0f;
    if (satLeft_) lambdaInstant = std::min(lambdaInstant, rail / demandMagLeft);
    if (satRight_) lambdaInstant = std::min(lambdaInstant, rail / demandMagRight);
    if (lambdaInstant < lambda_) {
      lambda_ = lambdaInstant;  // fast attack: shed authority immediately
    } else if (dt > 0.0f) {
      lambda_ += (lambdaInstant - lambda_) *
                 std::min(1.0f, dt / kLambdaReleaseTau);
    }
    lambda_ = clampf(lambda_, 0.0f, 1.0f);
  }

  float scaledLeft = lambda_ * rawLeft;
  float scaledRight = lambda_ * rawRight;

  // K1 (design §4.5, sprint 029 ticket 001): decided up front so the
  // SAME condition governs both the trim computed below and whether
  // the reference is allowed to integrate once the floor has run.
  const bool twistHoldActive = active_.twistHoldGain > 0.0f &&
                               sampleLeft_.connected && sampleRight_.connected;

  float trim = 0.0f;
  if (twistHoldActive) {
    if (!twistRef_.armed || twistRef_.epoch != epoch_) {
      twistRef_.reference = 0.0f;
      twistRef_.originLeft = sampleLeft_.position;
      twistRef_.originRight = sampleRight_.position;
      twistRef_.epoch = epoch_;
      twistRef_.armed = true;
    }
    const float measuredTwistPosition =
        0.5f * ((sampleRight_.position - twistRef_.originRight) -
                (sampleLeft_.position - twistRef_.originLeft));  // [counts]
    const float twistError = twistRef_.reference - measuredTwistPosition;
    const float authority = rail * active_.fullDutyVelocity;  // [counts/s]
    // Headroom against the last FLOORED command (lastSpeedLeft_/Right_
    // are only overwritten below, after applySpeedFloor() runs, so
    // here they still hold the PREVIOUS cycle's post-floor values) --
    // not this cycle's pre-floor scaledLeft/Right, which understates
    // how little authority remains once the floor rescales up.
    const float headroom = std::max(
        0.0f, authority - std::max(std::fabs(lastSpeedLeft_),
                                   std::fabs(lastSpeedRight_)));
    trim = clampf(active_.twistHoldGain * twistError, -headroom, headroom);
  } else {
    twistRef_.armed = false;
  }

  float targetLeft = scaledLeft - trim;
  float targetRight = scaledRight + trim;

  float speedLeft, speedRight, floorScale;
  applySpeedFloor(targetLeft, targetRight, speedLeft, speedRight, floorScale);
  lastFloorScale_ = floorScale;  // [1] host-test diagnostic (lastFloorScale())

  // K1, corrected 2026-09-04 (design §4.5): integrate the FLOORED
  // COMMANDED twist -- scaledTwist * floorScale, computed from
  // scaledLeft/scaledRight BEFORE trim is folded in -- never the
  // post-floor targets (speedLeft/speedRight), which already contain
  // +/-trim. The first landing (ticket 001) integrated
  // 0.5*(speedRight - speedLeft) of those trimmed targets: the servo's
  // own trim output fed straight back into the reference it is judged
  // against next tick, a positive-feedback loop ideal (matched) wheels
  // never excite because their trim stays near zero. MEASURED tovez
  // 2026-09-04, captures/bench-acceptance-029-20260904c/: WHEELS_V 200
  // 200 drove the left wheel negative (~-76 mm/s) and the right to
  // 492 mm/s under this defect. floorScale (1.0 when the floor does
  // not bind) still applies the floor's rescale to the reference, so
  // K1's original fix (integrate the FLOORED value, not the pre-floor
  // one) is preserved; only the trim contribution is now excluded.
  // With vMin == 0, floorScale == 1.0 always and this reduces to the
  // pre-K1-patch line, scaledTwist * dt.
  if (twistHoldActive && dt > 0.0f) {
    const float scaledTwist = 0.5f * (scaledRight - scaledLeft);
    twistRef_.reference += scaledTwist * floorScale * dt;
  }

  const float correctedLeft =
      correctedCommand(speedLeft, lastSpeedLeft_, true, biasLeft_);
  const float correctedRight =
      correctedCommand(speedRight, lastSpeedRight_, false, biasRight_);
  lastSpeedLeft_ = speedLeft;
  lastSpeedRight_ = speedRight;

  if (dt > 0.0f) {
    const float rawAccelLeft = (speedLeft - previousTargetLeft_) / dt;
    const float rawAccelRight = (speedRight - previousTargetRight_) / dt;
    cmdAccelLeft_ += kAccelSmoothing * (rawAccelLeft - cmdAccelLeft_);
    cmdAccelRight_ += kAccelSmoothing * (rawAccelRight - cmdAccelRight_);
  }
  previousTargetLeft_ = speedLeft;
  previousTargetRight_ = speedRight;

  const uint64_t nowUs = previousCycleStartUs_;  // this cycle's start stamp
  const bool freshLeft =
      sampleLeft_.connected && !left_.wedgeSuspect() &&
      static_cast<float>(nowUs - sampleLeft_.sampleTime) <= kMaxSampleAge;
  const bool freshRight =
      sampleRight_.connected && !right_.wedgeSuspect() &&
      static_cast<float>(nowUs - sampleRight_.sampleTime) <= kMaxSampleAge;

  const float errLeft = speedLeft - sampleLeft_.velocity;
  const float errRight = speedRight - sampleRight_.velocity;

  const float posErrorLeft = positionError(speedLeft, sampleLeft_, posRefLeft_,
                                           dt, sampleAdvancedLeft_);
  const float posErrorRight = positionError(
      speedRight, sampleRight_, posRefRight_, dt, sampleAdvancedRight_);
  const float pidLeft =
      (speedLeft == 0.0f) ? 0.0f
                          : fastPid(posErrorLeft, errLeft, cmdAccelLeft_);
  const float pidRight =
      (speedRight == 0.0f) ? 0.0f
                           : fastPid(posErrorRight, errRight, cmdAccelRight_);
  lastPidLeft_ = pidLeft;
  lastPidRight_ = pidRight;

  const float demandLeft = (correctedLeft + pidLeft) * dutyPerSpeed;
  const float demandRight = (correctedRight + pidRight) * dutyPerSpeed;
  dutyDemandLeft_ = demandLeft;
  dutyDemandRight_ = demandRight;

  const float dutyLeft =
      crawlDuty(clampf(demandLeft, -rail, rail), crawlCarryLeft_);
  const float dutyRight =
      crawlDuty(clampf(demandRight, -rail, rail), crawlCarryRight_);

  const bool adaptAllowed = lambda_ >= kLambdaAdaptFloor;
  adaptBias(biasLeft_, errLeft, cmdAccelLeft_, std::fabs(speedLeft),
            freshLeft && adaptAllowed, dt);
  adaptBias(biasRight_, errRight, cmdAccelRight_, std::fabs(speedRight),
            freshRight && adaptAllowed, dt);

  const bool biasSaturatedLeft =
      active_.biasMax > 0.0f && std::fabs(biasLeft_) >= active_.biasMax;
  const bool biasSaturatedRight =
      active_.biasMax > 0.0f && std::fabs(biasRight_) >= active_.biasMax;
  const bool pidSaturatedLeft =
      active_.pidMax > 0.0f && std::fabs(pidLeft) >= active_.pidMax;
  const bool pidSaturatedRight =
      active_.pidMax > 0.0f && std::fabs(pidRight) >= active_.pidMax;
  const bool deficitCondLeft = active_.deficitThreshold > 0.0f &&
                               std::fabs(errLeft) > active_.deficitThreshold &&
                               biasSaturatedLeft && pidSaturatedLeft;
  const bool deficitCondRight =
      active_.deficitThreshold > 0.0f &&
      std::fabs(errRight) > active_.deficitThreshold && biasSaturatedRight &&
      pidSaturatedRight;
  updateLatch(deficitCondLeft, active_.deficitWindow, nowMs, deficitSinceLeft_,
              deficitLeft_);
  updateLatch(deficitCondRight, active_.deficitWindow, nowMs,
              deficitSinceRight_, deficitRight_);

  const bool demanding =
      active_.stallDemand > 0.0f &&
      (std::fabs(rawLeft) > active_.stallDemand ||
       std::fabs(rawRight) > active_.stallDemand);
  const bool encoderStill =
      std::fabs(sampleLeft_.velocity) <= active_.stallSpeed &&
      std::fabs(sampleRight_.velocity) <= active_.stallSpeed &&
      sampleLeft_.connected && sampleRight_.connected;
  updateLatch(demanding && encoderStill, active_.stallWindow, nowMs,
              stallSince_, stallLatched_);
  if (stallLatched_ && !stallHalted_) {
    stallHalted_ = true;
  }

  stageDuty(dutyLeft, dutyRight);
}

void DifferentialDrive::stageStop() { stageDuty(0.0f, 0.0f); }

void DifferentialDrive::stageDuty(float dutyLeft, float dutyRight) {
  const bool wheelsMoving = std::fabs(left_.velocity()) > kRestVelocity ||
                            std::fabs(right_.velocity()) > kRestVelocity;
  const bool enforceStop = stopEnforceCountdown_ > 0 || wheelsMoving;
  if (stopEnforceCountdown_ > 0) --stopEnforceCountdown_;

  const bool commandedStop = dutyLeft == 0.0f && dutyRight == 0.0f;
  const bool alreadyQuiet =
      commandedStop && writtenLeft_ == 0.0f && writtenRight_ == 0.0f;
  if (commandedStop && !alreadyQuiet) stopEnforceCountdown_ = kStopEnforceTicks;

  if (alreadyQuiet && !enforceStop) return;
  left_.setDuty(dutyLeft);
  right_.setDuty(dutyRight);
  writtenLeft_ = dutyLeft;
  writtenRight_ = dutyRight;
}

void DifferentialDrive::refreshSample(Motor& motor, WheelSample& sample) {
  sample.connected = motor.connected();
  const uint64_t sampleTime = motor.sampleTime();
  const float position = motor.position();
  if (!sample.everSampled) {
    if (sampleTime != 0) {
      sample.everSampled = true;
      sample.sampleTime = sampleTime;
      sample.position = position;
    }
    return;  // velocity stays 0 until a second genuine sample exists
  }
  if (sampleTime != sample.sampleTime) {
    const float interval =
        static_cast<float>(sampleTime - sample.sampleTime) * 1e-6f;  // [s]
    if (interval > 0.0f) {
      sample.velocity = (position - sample.position) / interval;
    }
    sample.sampleTime = sampleTime;
    sample.position = position;
  }
}

void DifferentialDrive::resetAdaptiveState() {
  posRefLeft_ = PositionRef{};
  posRefRight_ = PositionRef{};
  twistRef_ = TwistRef{};
  biasLeft_ = 0.0f;
  biasRight_ = 0.0f;
  deficitSinceLeft_ = 0;
  deficitSinceRight_ = 0;
  deficitLeft_ = false;
  deficitRight_ = false;
  stallSince_ = 0;
  stallLatched_ = false;
  crawlCarryLeft_ = 0.0f;
  crawlCarryRight_ = 0.0f;
  lastPidLeft_ = 0.0f;
  lastPidRight_ = 0.0f;
  stopEnforceCountdown_ = kStopEnforceTicks;
}

void DifferentialDrive::publishOutput(uint32_t nowMs, uint64_t cycleStartUs,
                                      uint64_t busyEndUs,
                                      uint32_t measuredPeriod,
                                      bool leaseExpired) {
  ++outSeq_;  // odd: write in progress
  out_.cyclePeriodMeasured = measuredPeriod;
  out_.leaseExpired = leaseExpired;
  out_.now = nowMs;
  out_.nowFine = static_cast<uint32_t>(busyEndUs);
  out_.cycleCount = cycleCount_;
  out_.cycleOverrunCount = cycleOverrunCount_;
  out_.cycleGapCount = cycleGapCount_;
  out_.cycleBusy = static_cast<uint32_t>(busyEndUs - cycleStartUs);
  out_.sampleTimeLeft = static_cast<uint32_t>(sampleLeft_.sampleTime);
  out_.sampleTimeRight = static_cast<uint32_t>(sampleRight_.sampleTime);
  out_.positionLeft = sampleLeft_.position;
  out_.positionRight = sampleRight_.position;
  out_.velocityLeft = sampleLeft_.velocity;
  out_.velocityRight = sampleRight_.velocity;
  out_.velocity = 0.5f * (sampleLeft_.velocity + sampleRight_.velocity);
  out_.twist = 0.5f * (sampleRight_.velocity - sampleLeft_.velocity);
  out_.appliedDutyLeft = left_.appliedDuty() * 100.0f;
  out_.appliedDutyRight = right_.appliedDuty() * 100.0f;
  out_.lambda = lambda_;
  out_.biasLeft = biasLeft_;
  out_.biasRight = biasRight_;
  out_.ready = begun_ && active_.fullDutyVelocity > 0.0f;
  out_.estopped = estopLatch_;
  out_.stallHalted = stallHalted_;
  out_.satLeft = satLeft_;
  out_.satRight = satRight_;
  out_.stallLeft = stallLatched_;
  out_.stallRight = stallLatched_;
  out_.wedgeLeft = left_.wedged();
  out_.wedgeRight = right_.wedged();
  out_.wedgeSuspectLeft = left_.wedgeSuspect();
  out_.wedgeSuspectRight = right_.wedgeSuspect();
  out_.deficitLeft = deficitLeft_;
  out_.deficitRight = deficitRight_;
  out_.connectedLeft = sampleLeft_.connected;
  out_.connectedRight = sampleRight_.connected;
  out_.leaseExpiryCount = leaseExpiryCount_;
  out_.i2cFaultCount = i2cFaultCount_;
  out_.positionEpochLeft = positionEpochLeft_;
  out_.positionEpochRight = positionEpochRight_;
  ++outSeq_;  // even: committed
}

float DifferentialDrive::correctedCommand(float desired, float previous,
                                          bool leftWheel, float bias) const {
  if (desired == 0.0f) return 0.0f;  // stop is stop; never offset it
  const int w = leftWheel ? 0 : 1;
  const int d = (std::fabs(desired) > std::fabs(previous)) ? 0 : 1;
  const float magnitude =
      (std::fabs(desired) - active_.wheelIntercept[w][d]) /
      active_.wheelGain[w][d];
  if (magnitude <= 0.0f) return 0.0f;  // below the intercept: unreachable
  const float correctedMagnitude = magnitude + bias;
  if (correctedMagnitude <= 0.0f) return 0.0f;  // never flip direction
  return std::copysign(correctedMagnitude, desired);
}

float DifferentialDrive::fastPid(float posError, float err, float aCmd) const {
  const float proportional = active_.kp * err;
  const float feed = active_.kaff * aCmd;

  float integral = 0.0f;  // [counts/s]
  if (active_.iMax > 0.0f) {
    integral = active_.ki * posError;
    if (integral > active_.iMax) integral = active_.iMax;
    if (integral < -active_.iMax) integral = -active_.iMax;
  }

  float pid = proportional + feed + integral;
  if (active_.pidMax > 0.0f) {
    if (pid > active_.pidMax) pid = active_.pidMax;
    if (pid < -active_.pidMax) pid = -active_.pidMax;
  }
  if (!std::isfinite(pid)) return 0.0f;  // fail closed, never inject NaN
  return pid;
}

float DifferentialDrive::positionError(float speed, const WheelSample& wheel,
                                       PositionRef& ref, float dt,
                                       bool advanced) {
  if (speed == 0.0f || dt <= 0.0f || !wheel.connected ||
      ref.epoch != epoch_ || !ref.armed) {
    ref.armed = (speed != 0.0f) && wheel.connected;
    ref.epoch = epoch_;
    ref.origin = wheel.position;
    ref.reference = 0.0f;
    return 0.0f;
  }
  // K2 (design §4.5): `advanced` reflects whether the PREVIOUS step()'s
  // own collect actually moved this wheel's cached sample (see step()'s
  // stampBefore/After comparison). A tick whose sample did NOT advance
  // must not integrate the reference against a wheel.position that
  // never moved -- doing so injects a full tick's worth of phantom
  // position error the instant the sample resumes (MEASURED: +6 duty
  // points off one frozen tick, review MK-03 / profile_probe.cpp E5).
  // wheel.position is unchanged from the last call by construction
  // here, so recomputing from the UNCHANGED reference against the
  // UNCHANGED position reproduces the previous call's error exactly --
  // "return the last error" needs no separate stored field.
  if (advanced) {
    ref.reference += speed * dt;                                   // [counts]
    if (active_.posErrMax > 0.0f) {
      // K3 anti-windup (design §4.5): clamp the STORED reference, not
      // just the returned error, so a wheel that fell behind for a
      // long stretch cannot carry an unbounded backlog into the taper
      // and discharge it there (the "end bump" memory). Previously
      // only the error below was clamped; the reference itself grew
      // without bound.
      const float measured = wheel.position - ref.origin;  // [counts]
      ref.reference = clampf(ref.reference, measured - active_.posErrMax,
                             measured + active_.posErrMax);
    }
  }
  float error = ref.reference - (wheel.position - ref.origin);     // [counts]
  if (active_.posErrMax > 0.0f) {
    if (error > active_.posErrMax) error = active_.posErrMax;
    if (error < -active_.posErrMax) error = -active_.posErrMax;
  }
  return error;
}

void DifferentialDrive::adaptBias(float& bias, float err, float aCmd,
                                  float vCmdMagnitude, bool fresh,
                                  float dt) const {
  if (active_.tauAdapt <= 0.0f || dt <= 0.0f || !fresh) return;
  if (std::fabs(aCmd) >= active_.aSteady) return;  // ramping, not steady
  if (vCmdMagnitude < active_.vMin) return;        // below the speed floor
  bias += err * dt / active_.tauAdapt;
  if (active_.biasMax > 0.0f) {
    if (bias > active_.biasMax) bias = active_.biasMax;
    if (bias < -active_.biasMax) bias = -active_.biasMax;
  } else {
    bias = 0.0f;
  }
}

float DifferentialDrive::crawlDuty(float duty, float& carry) const {
  const float magnitude = std::fabs(duty);
  if (active_.crawlPulse == 0.0f || magnitude >= active_.crawlPulse) {
    return duty;
  }
  if (magnitude == 0.0f) {
    carry = 0.0f;
    return 0.0f;
  }
  carry += magnitude / active_.crawlPulse;
  if (carry < 1.0f) return 0.0f;
  carry -= 1.0f;
  return std::copysign(active_.crawlPulse, duty);
}

void DifferentialDrive::applySpeedFloor(float rawLeft, float rawRight,
                                        float& speedLeft,
                                        float& speedRight,
                                        float& floorScale) const {
  speedLeft = rawLeft;
  speedRight = rawRight;
  floorScale = 1.0f;  // [1] no rescale unless the floor binds below
  if (active_.vMin <= 0.0f) return;
  const float dominantMag =
      std::max(std::fabs(rawLeft), std::fabs(rawRight));
  if (dominantMag <= 0.0f || dominantMag >= active_.vMin) return;
  const float scale = active_.vMin / dominantMag;
  speedLeft = rawLeft * scale;
  speedRight = rawRight * scale;
  floorScale = scale;  // [1]
}

void DifferentialDrive::updateLatch(bool conditionNow, float window,
                                    uint32_t now, uint32_t& since,
                                    bool& latched) const {
  if (window <= 0.0f || !conditionNow) {
    since = 0;
    latched = false;
    return;
  }
  if (since == 0) since = now;
  latched = (now - since) >= static_cast<uint32_t>(window);
}

}  // namespace DiffDrive
