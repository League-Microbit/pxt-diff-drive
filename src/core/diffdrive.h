// differential_drive.h — DiffDrive::DifferentialDrive: a self-contained
// differential-drive wheel kernel. ONE class, TWO files (this header +
// differential_drive.cpp), and NOTHING else: the only include is
// <cstdint>, and the four small interfaces below are the package's OWN
// ports — the complete surface a host platform implements to run it.
//
// Vendored from League-Robotics/radio-robot, where the kernel currently
// lives at src/firm/diffdrive/ (src/DESIGN.md §2 is the one
// authoritative statement of this path and the maintenance boundary).
// Control law byte-identical to upstream except cycleGapCount /
// cycleGapCount_ below, a local fix not yet ported back — fix bugs in
// both trees.
//
// Ports the host implements: Motor (staged duty writes, split-phase
// encoder sampling, an immediate emergency stop), Clock (monotonic
// microseconds), Sleeper (settle/pace sleeps + a cooperative yield),
// FiberLauncher (optional — a host that owns its own loop never calls
// start() and drives step() directly instead).
#pragma once

#include <cstdint>

namespace DiffDrive {

class Motor {
 public:
  virtual ~Motor() = default;

  virtual void begin() = 0;
  virtual void requestSample() = 0;
  virtual void setDuty(float duty) = 0;   // [-1, 1] staged raw duty
  virtual void emergencyStop() = 0;
  virtual void tick(uint64_t nowUs) = 0;  // [us] execute staged + collect

  virtual float position() const = 0;     // [counts] accumulated
  virtual float velocity() const = 0;     // [counts/s] signed
  virtual float appliedDuty() const = 0;  // [-1, 1] last landed write
  virtual bool connected() const = 0;
  virtual uint64_t sampleTime() const = 0;  // [us] last SUCCESSFUL collect
  virtual void rebaseline() = 0;  // software re-anchor; no bus traffic

  virtual bool wedged() const = 0;
  virtual bool wedgeSuspect() const = 0;
};

class Clock {
 public:
  virtual ~Clock() = default;
  virtual uint64_t nowMicros() const = 0;  // [us] monotonic
};

class Sleeper {
 public:
  virtual ~Sleeper() = default;
  virtual void sleepMillis(uint32_t duration) = 0;  // [ms]
  virtual void yield() = 0;  // hand the processor to another task/fiber
};

class FiberLauncher {
 public:
  virtual ~FiberLauncher() = default;
  virtual void launch(void (*entry)(void*), void* context) = 0;
};

class DifferentialDrive {
 public:
  static constexpr uint8_t kModeNeutral = 0;
  static constexpr uint8_t kModeVelocity = 1;
  static constexpr uint8_t kModeRawDuty = 2;

  enum class Status : uint8_t {
    kOk = 0,
    kRefusedUnconfigured,  // maxDuty == 0; or VELOCITY with fullDutyVelocity == 0
    kRefusedNotBegun,      // command before begin(). NOT before start(): the
                           //   host harness commands and step()s WITHOUT ever
                           //   launching the fiber, so readiness is begin()'s
                           //   to grant, not start()'s
    kRefusedEstopped,
    kRefusedNonFinite,
    kCadencePreserved,     // post-begin setConfig with a differing cyclePeriod:
                           //   block applied, frozen cadence kept
  };

  static constexpr uint32_t kLeaseMax = 3600000u;  // [ms]

  struct Config {
    float maxDuty = 0.0f;            // [%] authority rail (lambda scales to
                                     //   this); 0 = ALL modes refused
    float fullDutyVelocity = 0.0f;   // [counts/s] wheel rate at 100% duty;
                                     //   0 = uncalibrated → VELOCITY refused
    float kp = 0.0f;                 // [1]
    float ki = 0.0f;                 // [1/s] on clamped position error
    float iMax = 0.0f;               // [counts/s] I-term clamp; 0 disables I
    float kaff = 0.0f;               // [s] accel feedforward
    float pidMax = 0.0f;             // [counts/s] whole-PID output clamp
    float twistHoldGain = 0.0f;      // [1/s] twist-integral ratio hold; 0 = off
    float wheelGain[2][2] = {{1.0f, 1.0f}, {1.0f, 1.0f}};       // [1]
    float wheelIntercept[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};  // [counts/s]
    float vMin = 0.0f;               // [counts/s] speed floor; 0 = off
    float posErrMax = 0.0f;          // [counts] position-error clamp; 0 = unclamped
    float biasMax = 0.0f;            // [counts/s] Stage C trim clamp; 0 disables
    float tauAdapt = 0.0f;           // [s] Stage C time constant; <=0 disables
    float aSteady = 0.0f;            // [counts/s^2] |aCmd| below this is steady
    float deficitThreshold = 0.0f;   // [counts/s] 0 = detector off
    float deficitWindow = 0.0f;      // [ms]
    float stallSpeed = 0.0f;         // [counts/s]
    float stallDemand = 0.0f;        // [counts/s] 0 = detector off
    float stallWindow = 0.0f;        // [ms]
    bool lambdaEnabled = false;
    float crawlPulse = 0.0f;         // [-1, 1] sub-breakaway pulse amplitude; 0 = off
    uint32_t cyclePeriod = 24;       // [ms] fiber cadence (>= 2*kSettle + margin)
  };

  struct Output {
    uint32_t now = 0;               // [ms] kernel clock at publish
    uint32_t nowFine = 0;           // [us] same instant — age-math base.
    uint32_t cycleCount = 0;        // heartbeat — the RobotLoop sentinel
    uint32_t cyclePeriodMeasured = 0;  // [us] measured (feeds all dt terms).
    uint32_t cycleBusy = 0;         // [us]
    uint32_t cycleGapCount = 0;      // cycles whose measured period was
                                     // too long to be a control cycle
                                     // (caller idle); re-anchored, not
                                     // integrated across
    uint32_t cycleOverrunCount = 0;  // cycles that missed their absolute
                                    //   deadline
    uint32_t sampleTimeLeft = 0;    // [us]
    uint32_t sampleTimeRight = 0;   // [us]
    uint32_t positionEpochLeft = 0;
    uint32_t positionEpochRight = 0;
    float positionLeft = 0.0f;      // [counts] accumulated, never device-reset
    float positionRight = 0.0f;     // [counts]
    float velocityLeft = 0.0f;      // [counts/s]
    float velocityRight = 0.0f;     // [counts/s]
    float velocity = 0.0f;          // [counts/s] measured mean
    float twist = 0.0f;             // [counts/s] measured half-differential, CCW+
    float appliedDutyLeft = 0.0f;   // [%]
    float appliedDutyRight = 0.0f;  // [%]
    float lambda = 1.0f;            // [1] authority scale currently applied
    float biasLeft = 0.0f;          // [counts/s]
    float biasRight = 0.0f;         // [counts/s]
    bool ready = false;             // begun + calibrated (velocity mode usable)
    bool estopped = false;
    bool leaseExpired = false;
    bool stallHalted = false;       // kernel self-halted on the stall latch
    bool satLeft = false, satRight = false;      // duty demand beyond the rail
    bool stallLeft = false, stallRight = false;
    bool wedgeLeft = false, wedgeRight = false;
    bool wedgeSuspectLeft = false, wedgeSuspectRight = false;
    bool deficitLeft = false, deficitRight = false;
    bool connectedLeft = false, connectedRight = false;
    uint32_t leaseExpiryCount = 0;  // sticky diagnostics
    uint32_t i2cFaultCount = 0;
  };

  DifferentialDrive(Motor& left, Motor& right,
                    const Clock& clock, Sleeper& sleeper,
                    FiberLauncher& launcher);

  DifferentialDrive& setMaxDuty(float maxDuty);            // [%]
  DifferentialDrive& setFullDutyVelocity(float velocity);  // [counts/s]
  DifferentialDrive& setKp(float kp);                      // [1]
  DifferentialDrive& setKi(float ki);                      // [1/s]
  DifferentialDrive& setIMax(float iMax);                  // [counts/s]
  DifferentialDrive& setKaff(float kaff);                  // [s]
  DifferentialDrive& setPidMax(float pidMax);              // [counts/s]
  DifferentialDrive& setTwistHoldGain(float gain);         // [1/s]
  DifferentialDrive& setWheelCorrection(
      float gainLeftAccel, float interceptLeftAccel,
      float gainLeftDecel, float interceptLeftDecel,
      float gainRightAccel, float interceptRightAccel,
      float gainRightDecel, float interceptRightDecel);    // [1]/[counts/s] x4
  DifferentialDrive& setSpeedFloor(float vMin);            // [counts/s]
  DifferentialDrive& setPositionErrorMax(float posErrMax); // [counts]
  DifferentialDrive& setAdaptation(float biasMax, float tauAdapt,
                                   float aSteady);  // [counts/s] [s] [counts/s^2]
  DifferentialDrive& setDeficit(float threshold, float window);  // [counts/s] [ms]
  DifferentialDrive& setStall(float speed, float demand,
                              float window);  // [counts/s] [counts/s] [ms]
  DifferentialDrive& setLambdaEnabled(bool enabled);
  DifferentialDrive& setCrawlPulse(float crawlPulse);      // [-1, 1]
  DifferentialDrive& setCyclePeriod(uint32_t period);      // [ms]

  Status setConfig(const Config& config);
  Config config() const;

  Status lastError() const { return lastError_; }
  void clearLastError() { lastError_ = Status::kOk; }

  Status begin();
  Status start();
  bool running() const { return running_; }

  Status drive(float velocity, float twist,
               uint32_t lease);       // [counts/s] [counts/s] [ms]
  Status driveDuty(float dutyLeft, float dutyRight,
                   uint32_t lease);   // [%] [%] [ms]
  void neutral();        // commanded stop through the full stop path
  void estop();          // latch: zero NOW; holds until estopClear()
  void estopClear();
  void emergencyStopMotors();
  void clearStallLatch();
  void rebasePosition();
  // K4 (sprint 029 ticket 001, design §4.5): deferred request, same
  // shape as rebasePosition() -- disarms posRefLeft_/Right_ and
  // twistRef_ at the START of the next step(), before controlStep()
  // runs, so that SAME step() re-anchors all three references at the
  // new origin instead of carrying a stale one across a segment
  // boundary. Unlike rebasePosition() this does NOT touch the wheel
  // samples or bump epoch_ -- it only clears the "armed" latches that
  // control re-anchoring, exactly what a caller wants when the
  // COMMAND is changing (e.g. a pivot-to-straight handoff) but the
  // measured position itself is still valid.
  void rearmReferences();

  Output output() const;

  void step();

  // ---- Diagnostic accessors (sprint 029 ticket 001; host-test support
  // for K1/K3/K4) -- read-only windows into the reference integrators
  // those patches govern. Not part of the control law, not written by
  // any production caller, and safe to call at any time (return 0 /
  // false before the kernel has ever armed the corresponding
  // reference). Added because a host test verifying the post-floor
  // twist integration or the anti-windup clamp has no other way to see
  // these private integrators without re-deriving them from duty
  // output, which is indirect and fragile for an exact-value assertion.
  float twistReferenceCounts() const { return twistRef_.reference; }  // [counts]
  bool twistReferenceArmed() const { return twistRef_.armed; }
  float positionReferenceCounts(bool leftWheel) const {  // [counts]
    return (leftWheel ? posRefLeft_ : posRefRight_).reference;
  }
  // K1, corrected (sprint 029 ticket 010, design §4.5): the scale
  // applySpeedFloor() applied on the MOST RECENT controlStep() call
  // (1.0 when the floor did not bind). Exposed so a host test can
  // reconstruct twistRef_'s expected increment (scaledTwist *
  // floorScale * dt) independently, without re-deriving floorScale
  // from observed duty -- which would be indirect and, once trim is
  // folded into duty, no longer equal to floorScale at all.
  float lastFloorScale() const { return lastFloorScale_; }  // [1]

 private:
  struct Command {
    uint8_t mode = kModeNeutral;
    float velocity = 0.0f;     // [counts/s]
    float twist = 0.0f;        // [counts/s]
    float dutyLeft = 0.0f;     // [%]
    float dutyRight = 0.0f;    // [%]
    uint32_t validUntil = 0;   // [ms] absolute kernel clock; computed in drive()
  };

  struct WheelSample {
    float position = 0.0f;       // [counts]
    float velocity = 0.0f;       // [counts/s] successful-collect quotient
    uint64_t sampleTime = 0;     // [us] last SUCCESSFUL collect
    bool connected = false;
    bool everSampled = false;
  };

  struct PositionRef {
    float reference = 0.0f;  // [counts] integral of commanded speed since anchor
    float origin = 0.0f;     // [counts] wheel position when anchored
    uint8_t epoch = 0;       // kernel epoch when anchored (bumped on rebase)
    bool armed = false;
  };

  struct TwistRef {
    float reference = 0.0f;   // [counts] integral of commanded twist since anchor
    float originLeft = 0.0f;  // [counts]
    float originRight = 0.0f; // [counts]
    uint8_t epoch = 0;
    bool armed = false;
  };

  static void fiberEntry(void* self);
  void run();  // the kernel fiber body: step() + absolute-deadline pace

  Status checkCommandable(bool needsVelocityCalibration) const;

  void snapshotConfig();
  Command snapshotCommand() const;
  void controlStep(const Command& cmd, uint8_t effectiveMode, float dt,
                   uint32_t nowMs);  // [s] [ms]
  void stageStop();
  void stageDuty(float dutyLeft, float dutyRight);  // [-1,1] x2, write-gated
  void refreshSample(Motor& motor, WheelSample& sample);
  void resetAdaptiveState();
  void publishOutput(uint32_t nowMs, uint64_t cycleStartUs, uint64_t busyEndUs,
                     uint32_t measuredPeriod, bool leaseExpired);  // [us] x2 [us]

  float correctedCommand(float desired, float previous, bool leftWheel,
                         float bias) const;
  float fastPid(float posError, float err, float aCmd) const;  // [counts] [counts/s] [counts/s^2]
  float positionError(float speed, const WheelSample& wheel, PositionRef& ref,
                      float dt,
                      bool advanced);  // [counts/s] [s] bool -> [counts]
  void adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                 bool fresh, float dt) const;
  float crawlDuty(float duty, float& carry) const;
  void applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                       float& speedRight, float& floorScale) const;  // [1]
  void updateLatch(bool conditionNow, float window, uint32_t now,
                   uint32_t& since, bool& latched) const;  // [ms]

  Motor& left_;
  Motor& right_;
  const Clock& clock_;
  Sleeper& sleeper_;
  FiberLauncher& launcher_;

  Config staged_;
  Config active_;
  volatile uint32_t cfgSeq_ = 0;
  uint32_t activeCfgSeq_ = 0;

  Command command_;
  volatile uint32_t cmdSeq_ = 0;
  uint32_t seenCmdSeq_ = 0;

  volatile bool estopLatch_ = false;

  volatile uint32_t clearStallReq_ = 0;
  volatile uint32_t rebaseReq_ = 0;
  volatile uint32_t rearmReq_ = 0;   // K4: rearmReferences()'s own counter
  uint32_t seenClearStallReq_ = 0;
  uint32_t seenRebaseReq_ = 0;
  uint32_t seenRearmReq_ = 0;

  Status lastError_ = Status::kOk;
  void noteRefusal(Status status) {
    if (lastError_ == Status::kOk) lastError_ = status;
  }

  bool begun_ = false;
  volatile bool running_ = false;
  uint8_t epoch_ = 0;              // bumped on rebasePosition
  bool stallHalted_ = false;
  bool wasForcedStop_ = false;     // edge detector for adaptive reset
  bool leaseWasLive_ = false;      // edge detector for leaseExpiryCount

  WheelSample sampleLeft_;
  WheelSample sampleRight_;
  // K2 (design §4.5): did the PREVIOUS step()'s own collect actually
  // advance this wheel's cached sample -- set at the end of step()
  // from the same stampBefore/After comparison i2cFaultCount_ already
  // makes, and consumed at the TOP of the NEXT step()'s controlStep()
  // call (the earliest point that step's own positionError() can see
  // it). Defaults true so the very first step() -- before any collect
  // has run -- behaves like an ordinary fresh tick, not a frozen one.
  bool sampleAdvancedLeft_ = true;
  bool sampleAdvancedRight_ = true;

  PositionRef posRefLeft_;
  PositionRef posRefRight_;
  TwistRef twistRef_;

  uint32_t positionEpochLeft_ = 0;
  uint32_t positionEpochRight_ = 0;
  uint32_t i2cFaultCount_ = 0;     // failed-collect cycles, sticky
  uint32_t cycleOverrunCount_ = 0;  // missed absolute deadlines, sticky
  // Cycles whose measured period was too long to be a real control
  // cycle -- the caller had stopped stepping. Counted, and treated as
  // a fresh start rather than integrated across (see step()).
  uint32_t cycleGapCount_ = 0;

  float biasLeft_ = 0.0f;          // [counts/s] Stage C's adapted parameter
  float biasRight_ = 0.0f;         // [counts/s]
  float lastSpeedLeft_ = 0.0f;     // [counts/s] Stage A direction-of-change memory
  float lastSpeedRight_ = 0.0f;    // [counts/s]
  float lastFloorScale_ = 1.0f;    // [1] applySpeedFloor()'s most recent scale
  float lastPidLeft_ = 0.0f;       // [counts/s]
  float lastPidRight_ = 0.0f;      // [counts/s]
  float crawlCarryLeft_ = 0.0f;    // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;
  float previousTargetLeft_ = 0.0f;   // [counts/s]
  float previousTargetRight_ = 0.0f;  // [counts/s]
  float cmdAccelLeft_ = 0.0f;         // [counts/s^2] smoothed
  float cmdAccelRight_ = 0.0f;        // [counts/s^2]
  bool satLeft_ = false;              // duty demand beyond the rail
  bool satRight_ = false;
  uint32_t leaseExpiryCount_ = 0;     // sticky diagnostics

  float dutyDemandLeft_ = 0.0f;    // [-1,1] fraction, unclamped magnitude kept
  float dutyDemandRight_ = 0.0f;   // [-1,1]
  float lambda_ = 1.0f;            // [1] filtered authority scale

  uint32_t deficitSinceLeft_ = 0;  // [ms]
  uint32_t deficitSinceRight_ = 0; // [ms]
  bool deficitLeft_ = false;
  bool deficitRight_ = false;
  uint32_t stallSince_ = 0;        // [ms] one condition, both wheels latch
  bool stallLatched_ = false;

  float writtenLeft_ = 0.0f;       // [-1, 1]
  float writtenRight_ = 0.0f;      // [-1, 1]
  uint8_t stopEnforceCountdown_ = 0;

  Output out_;
  volatile uint32_t outSeq_ = 0;

  uint64_t previousCycleStartUs_ = 0;  // [us]
  bool everCycled_ = false;
  uint32_t cycleCount_ = 0;

  static constexpr uint32_t kSettle = 4;          // [ms]
  static constexpr uint8_t kStopEnforceTicks = 30;
  static constexpr float kRestVelocity = 100.0f;  // [counts/s]
  static constexpr float kMaxSampleAge = 200000.0f;  // [us] freshness gate
  static constexpr float kAccelSmoothing = 0.35f;    // [1] cmdAccel EMA weight
  static constexpr float kLambdaReleaseTau = 0.3f;   // [s]
  static constexpr float kLambdaAdaptFloor = 0.95f;  // [1]
};

}  // namespace DiffDrive
