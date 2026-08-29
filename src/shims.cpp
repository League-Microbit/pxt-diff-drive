// shims.cpp -- the MakeCode-facing C++ surface. Composes the DiffDrive
// kernel (self-contained control law) with two NezhaMotorPorts and the
// CODAL platform ports, and adds the application-layer pieces the
// kernel deliberately does not contain:
//
//   - ODOMETRY: differential dead-reckoning from the kernel's Output
//     positions (the kernel is counts-native and has no chassis
//     geometry; track width and travel calibration live HERE).
//   - MOVE ENGINE: position-mode moves (distance+yaw, and goto via the
//     TS layer's arc math) as a start/update/end state machine over the
//     kernel's velocity interface. The TypeScript layer polls
//     updateMove() -- blocking and loop-style forms are both built on
//     that poll. Both updateMove() and the tick engine below share one
//     implementation, serviceMove().
//   - TICK ENGINE: tickDrive() runs one kernel.step() + serviceMove()
//     on the CALLER's own fiber, then self-paces to the next 24 ms
//     deadline. The kernel's own background fiber pacer
//     (start()/run()/fiberEntry()) is deliberately left unwired -- see
//     ensure()'s own comment -- so every control cycle now runs on
//     whichever fiber calls tickDrive() instead.
//   - STARVATION WATCHDOG: the one background fiber this file still
//     launches -- a safety net, not a control path; see its own
//     clearly delineated section below.
//
// Boundary convention: integers only. mm, mm/s, centidegrees,
// centidegrees/s; config values scaled x1000. The TS layer owns the
// cm/deg student units.
//
// Also called directly by protocol.cpp and wire_adapter.cpp via
// same-package forward declarations (this file has no header):
// tickDrive/stopAll/estopAll/setWheelsTimed/setKernelValue/
// getConfigValue/diagValue and the engine* wire forwards. Keep
// signatures compatible with their forward-declaration blocks.
#include "pxt.h"
#include "core/diffdrive.h"
#include "platform/encoder_pose_source.h"
#include "motion/motion_engine.h"
#include "platform/nezha_port.h"
#include "platform/otos_port.h"
#include "platform/platform_ports.h"

#include <cmath>

using namespace pxt;

namespace diffDrive {

// Boundary convention's cdeg<->rad conversion (see this file's own
// header comment above: "mm, mm/s, centidegrees, centidegrees/s"). The
// wire/TS-facing shim surface is centidegree-scaled; the kernel/motion-
// engine math beneath it is radian-scaled. Named once here so every
// crossing site defers to the same constant instead of open-coding the
// formula independently.
constexpr float kCdegToRad = 0.01f * 3.14159265f / 180.0f;
constexpr float kRadToCdeg = 1.0f / kCdegToRad;

// Forward declaration: the starvation watchdog fiber entry point is
// defined in its own clearly delineated section further down (see
// "starvation watchdog"); ensure() launches it via the same
// CodalFiberLauncher mechanism the kernel used for its own now-unwired
// fiber.
static void watchdogEntry(void* context);

// ---- composition ----------------------------------------------------

struct Rig {
  // vevov wiring. History: the tovez defaults left{2,-1}/right{1,+1}
  // drove vevov backward, so on 2026-08-19 both fwdSigns were flipped to
  // left{2,+1}/right{1,-1}. That fixed forward and silently mirrored
  // rotation: NezhaMotorPort applies fwdSign to the duty AND to the
  // encoder position (nezha_port.cpp), so flipping both signs reverses
  // the robot's physical rotation while odometry, reading through the
  // same flipped signs, stays self-consistent and never notices.
  //
  // Camera-measured 2026-08-19 on a single commanded +360 CCW pivot
  // ("P"): odometry believed +360.83 deg, AprilCam measured -342.58 deg
  // -- same move, opposite direction. Legs in the square tour tracked
  // their heading to within 1-8 deg, so translation was never wrong;
  // only rotation was.
  //
  // Flipping signs cannot fix this: forward and rotation flip together,
  // so no sign pair gives both. The free variable is which port is
  // called "left". Each motor KEEPS its own sign (M1 -> -1, M2 -> +1, so
  // forward is untouched) and the side labels swap, which negates the
  // (right - left) differential that sets heading. Equivalent statement:
  // vevov's motors are plugged in mirror-swapped relative to the old
  // config -- M1 is the physical LEFT wheel, M2 the physical RIGHT.
  //
  // VERIFIED 2026-08-20 under AprilCam: commanded +180 swept the tag
  // yaw counter-clockwise (17.9 deg -> 169.7 deg mid-turn samples) --
  // physical direction now matches the commanded sign.
  NezhaMotorPort left{1, -1};    // left = M1, mirrored
  NezhaMotorPort right{2, +1};   // right = M2
  CodalClock clock;
  CodalSleeper sleeper;
  CodalFiberLauncher launcher;
  DiffDrive::DifferentialDrive kernel{left, right, clock, sleeper,
                                      launcher};
  // Geometry, the two wheel primitives (sprint 003 ticket 006), and the
  // move engine (ticket 007), constructed over `kernel`/`clock` above --
  // must stay declared after both, since members initialize in
  // DECLARATION order regardless of this struct's own (implicit)
  // member-initializer order.
  MotionEngine engine{kernel, clock};

  // odometry [mm, rad], updated lazily from kernel Output
  float x = 0.0f, y = 0.0f, heading = 0.0f;
  float odomPosLeft = 0.0f, odomPosRight = 0.0f;  // [counts]
  bool odomPrimed = false;

  // GO_TO_W's encoder-odometry fallback PoseSource (sprint 006 ticket
  // 007, encoder_pose_source.h): binds to x/y/heading ABOVE by const
  // reference, so it MUST be declared after them -- member references
  // bind once, at construction, and members initialize in DECLARATION
  // order regardless of this struct's own (implicit) member-initializer
  // order, the same rule this file's header comment already states for
  // `engine` above. Lives exactly as long as this Rig (a process-
  // lifetime lazy singleton, see `rig`/`ensure()` below) -- see
  // encoder_pose_source.h's own header comment for why a shorter-lived
  // instance would dangle. engineGoToW() (below) is this project's one
  // selection point between this and `otosRef()`'s OtosPort.
  EncoderPoseSource encoderPose{x, y, heading};

  // tick engine (sprint 002): caller-driven stepping replaces the
  // kernel's own now-unwired fiber pacer -- see ensure(), tickDrive(),
  // and the starvation watchdog section below.
  uint64_t lastTickUs = 0;       // [us] clock.nowMicros() at the start
                                  // of the most recent tickDrive() call
                                  // -- the watchdog's only freshness
                                  // signal. 0 = no tick has run yet.
  uint64_t tickDeadlineUs = 0;   // [us] tickDrive()'s own absolute-
                                  // deadline pacing anchor. 0 = no tick
                                  // has run yet -- re-anchor to now.
  bool stepBusy = false;         // concurrency guard around
                                  // kernel.step() inside tickDrive();
                                  // see that function's own comment.
  uint32_t tickOverrunCount = 0; // Rig-level: tickDrive() calls that ran
                                  // past their own paced deadline.
                                  // Distinct from the kernel's own
                                  // cycleOverrunCount_, which only its
                                  // unused run() ever increments.

  // The wire's OWN "0 = use the configured default" convenience field,
  // deliberately SEPARATE from kernel.config().fullDutyVelocity. Those
  // are two unrelated meanings of zero that used to be collapsed onto
  // one field: the wire layer's sentinel (this) vs. the kernel's own
  // "0 = uncalibrated, refuse VELOCITY" gate
  // (DifferentialDrive::checkCommandable()). See engineDefaultCruiseMmS()
  // below, the wire-layer section, for the consumer. Seeded to 150.0f
  // -- NOT derived from any kernel constant, and NOT the duty ceiling --
  // chosen AT IMPLEMENTATION TIME to numerically match the block
  // layer's own `defaultSpeed` (15 cm/s, `blocks/motion.ts`), which was
  // 15 at the time.
  //
  // That match is NOT an enforced invariant and never has been: this
  // field is independently settable over the wire (`default_cruise`,
  // ordinal 15, setKernelValue()/getConfigValue() below), and
  // `defaultSpeed` is independently settable from a block
  // (`setDefaultSpeed()`, `blocks/motion.ts`) -- nothing keeps the two
  // in sync, and either one changing alone silently breaks the match.
  // A caller that needs the two to agree must set both explicitly; a
  // future change that wants a real coupling would need the TS layer
  // to read this field back over the wire before choosing its own
  // default, not a comment asserting they match.
  float defaultCruiseMmS_ = 150.0f;  // [mm/s]

  // Sprint 015 ticket 006 (build checkpoint): one-shot handoff from
  // engineSetGoToDeadline() to engineGoToRArmed() (both below), the
  // block layer's own go-to entry point split across two `//%` shims.
  // A real PXT build of the ORIGINAL single five-parameter
  // engineGoToR() shim reproduced "TS9200: Assertion failed"
  // deterministically -- twice, non-benign, surviving make_deploy.py's
  // one retry for the shape tools/DESIGN.md documents under the same
  // error code -- confirming the risk setTaperWindows()'s own comment
  // already recorded from an earlier incident. NOT a sticky
  // MotionEngine config field like setRampMs()/setTaperWindows()/
  // setTaperFloors() above (those intentionally persist across many
  // moves); this is read-once, for the VERY NEXT engineGoToRArmed()
  // call only, and both halves have exactly one caller between them
  // (sim.ts's _setGoToDeadline()/_goToR() pair, called back-to-back by
  // motion.ts's startGoTo()) so there is nowhere for a stale value to
  // leak in from.
  uint32_t pendingGoToDeadlineMs_ = 0;  // [ms]
};

static Rig* rig = nullptr;

static Rig& ensure() {
  if (rig == nullptr) {
    rig = new Rig();
    // Kernel defaults: the tovez bake (boot_calibration.cpp) with
    // NEUTRAL wheel gains -- a generic kit starts uncorrected.
    DiffDrive::DifferentialDrive::Config cfg;
    cfg.maxDuty = 100.0f;            // [%]
    cfg.fullDutyVelocity = 10795.0f; // [counts/s]
    cfg.kp = 0.0f;
    cfg.ki = 6.0f;                   // [1/s]
    cfg.iMax = 765.6f;               // [counts/s]
    cfg.pidMax = 1276.0f;            // [counts/s]
    // 70 mm/s (2026-08-29), was 20 mm/s (255.2f), a bare-motor figure.
    // With the floor under breakaway, MOVE_X's end taper let the
    // position I-term brake the wheel to a near-stop 6-11 mm short and
    // the 25%-of-cruise crawl feedforward (3-5% duty) could not restart
    // it, so every leg ended with a stiction jump. MEASURED tovez
    // 2026-08-29 wheels-up, captures/tovez-taper-20260829/variants.json
    // (SET speed_floor 893: 0 stalls at 100/150/200 mm/s, legs within
    // +-1.4 mm; baseline.json: 6-9 mm stalls) and gopiv 2026-08-29,
    // captures/gopiv-floor70-20260829/ (this default in source: 0/6
    // restart bumps vs 6/6 stock, legs end ~0.2 s sooner). UNVERIFIED
    // loaded/on the floor and on vevov; 100 mm/s overshot 2-4 mm.
    cfg.vMin = 893.2f;               // [counts/s] = 70 mm/s at 12.76 c/mm
    cfg.posErrMax = 127.6f;          // [counts]
    cfg.biasMax = 303.7f;            // [counts/s]
    cfg.tauAdapt = 30.0f;            // [s]
    cfg.aSteady = 382.8f;            // [counts/s^2]
    cfg.stallSpeed = 191.4f;         // [counts/s]
    cfg.stallDemand = 510.4f;        // [counts/s]
    cfg.stallWindow = 500.0f;        // [ms]
    // Twist-hold trim ON (2026-08-20): the tovez bake ships it disabled,
    // and bench charts show straight legs tilting ~1-2 deg each (wheel
    // imbalance integrating into heading, rotating the whole square).
    // This is the kernel's own servo for exactly that -- it trims the
    // measured differential toward the commanded one.
    cfg.twistHoldGain = 2.0f;        // [1/s]
    cfg.cyclePeriod = 24;            // [ms]
    rig->kernel.setConfig(cfg);
    rig->kernel.begin();   // primes encoders, arms boot zero-write

    // TICK MODEL (sprint 002): the kernel's own background fiber pacer
    // is intentionally left unwired here -- every control cycle now
    // runs on whichever fiber calls tickDrive() (below), not a fiber
    // this file starts. kernel.start()/run()/fiberEntry() stay compiled
    // and available (diffdrive.h/.cpp are byte-unmodified); restoring
    // the single call below re-enables the old free-running fiber-paced
    // mode. See sprint.md's Design Rationale ("pure tick model, fiber
    // pacer entirely unwired -- no dual mode").
    // rig->kernel.start();

    // The starvation watchdog is the only background fiber this file
    // launches -- see the "starvation watchdog" section below.
    rig->launcher.launch(&watchdogEntry, rig);
  }
  return *rig;
}

// ---- odometry -------------------------------------------------------

static void odomUpdate(Rig& r) {
  const DiffDrive::DifferentialDrive::Output out = r.kernel.output();
  if (!r.odomPrimed) {
    r.odomPosLeft = out.positionLeft;
    r.odomPosRight = out.positionRight;
    r.odomPrimed = true;
    return;
  }
  const float cpm = r.engine.countsPerMm();
  const float dLeft = (out.positionLeft - r.odomPosLeft) / cpm;    // [mm]
  const float dRight = (out.positionRight - r.odomPosRight) / cpm; // [mm]
  r.odomPosLeft = out.positionLeft;
  r.odomPosRight = out.positionRight;
  const float dCenter = 0.5f * (dLeft + dRight);          // [mm]
  const float dHeading =
      (dRight - dLeft) / r.engine.effectiveTrackWidth();  // [rad]
  const float midHeading = r.heading + 0.5f * dHeading;
  r.x += dCenter * std::cos(midHeading);
  r.y += dCenter * std::sin(midHeading);
  r.heading += dHeading;
}

// ---- cross-fiber stop delivery (sprint 006 ticket 002) -----------------
// Closes R-08/BLK-01 (code review 2026-08-23, independently re-derived in
// verify-blocks.md): kernel.neutral() only STAGES a zero command
// (diffdrive.cpp) -- delivery to the motors happens solely on a LATER
// kernel.step(), and step()'s own duty write happens BEFORE its two
// ~4 ms-per-wheel encoder settle sleeps. A stop or move-completion issued
// from a fiber other than the one currently inside step()'s settle
// window therefore stages a neutral that is not delivered until that
// step() returns AND another step() runs -- which, if the very call that
// staged it is what ended a `while (tickDrive())` loop (the common
// case), never happens until the ~100-150 ms starvation watchdog fires:
// the same class of bug commit 3e919e5 fixed for the in-fiber
// (move-completion) case, reopened here for the cross-fiber case.
//
// This helper pushes an immediate, PORT-LEVEL zero write to both
// motors -- the exact primitive the starvation watchdog below already
// uses (NezhaMotorPort::emergencyStop(), proven tick-independent by its
// exact-zero short-circuit in writeShapedDuty()) -- alongside the
// pre-existing staged kernel.neutral()/engine.endMove() at each call
// site. That delivers the stop within the SAME tick regardless of where
// in the settle window the race lands, adds no new fiber/ticker (a
// synchronous call on whichever fiber is already running -- the
// one-ticker-per-move invariant is unaffected), and never touches the
// vendored kernel (diffdrive.{h,cpp} stay byte-unchanged).
//
// Deliberately calls the MOTOR ports directly, exactly as the watchdog
// does, and never kernel.emergencyStopMotors() -- that kernel-level
// method also latches estopLatch_ as an (undocumented) side effect
// (diffdrive.cpp), which would turn this resumable soft stop into a
// hard e-stop requiring clearEmergencyStop(). Calling the ports directly
// stays in the same resumable "soft stop" family stopAll()/the watchdog
// already established: a fresh drive()/tickDrive() call resumes motion
// with no clear step needed.
static void deliverStopNow(Rig& r) {
  r.left.emergencyStop();
  r.right.emergencyStop();
}

// ---- velocity commands ----------------------------------------------

//%
void setWheels(int left, int right) {  // [mm/s] [mm/s]
  Rig& r = ensure();
  r.engine.wheelsV(static_cast<float>(left), static_cast<float>(right),
                   DiffDrive::DifferentialDrive::kLeaseMax);
}

//%
void driveTwist(int speed, int yawRate) {  // [mm/s] [cdeg/s]
  Rig& r = ensure();
  const float yawRad = static_cast<float>(yawRate) * kCdegToRad;
  const float twistMmS = yawRad * 0.5f * r.engine.effectiveTrackWidth();
  const float speedMmS = static_cast<float>(speed);
  r.engine.wheelsV(speedMmS - twistMmS, speedMmS + twistMmS,
                   DiffDrive::DifferentialDrive::kLeaseMax);
}

// ---- duration-bound direct drive (Protocol's WHEELS and
// MOVE-with-TIME-stop verb handlers) ------------------------------------
// Two additive primitives, identical to setWheels()/driveTwist() above
// except the lease is the caller's own duration instead of kLeaseMax --
// an expired lease auto-neutralizes on the kernel's next step()
// (diffdrive.cpp), so no separate timer is needed. Deliberately NOT
// `//%`-annotated: not block-facing (protocol.cpp is their only caller,
// via same-package forward declarations).
void setWheelsTimed(int left, int right,
                    uint32_t durationMs) {  // [mm/s] [mm/s] [ms]
  Rig& r = ensure();
  // WHEELS supersedes any in-flight move-engine move -- wheelsV() itself
  // clears it (motion-api.md S6, motion_engine.h).
  r.engine.wheelsV(static_cast<float>(left), static_cast<float>(right),
                   durationMs);
}

void driveTwistTimed(int speed, int yawRate,
                     uint32_t durationMs) {  // [mm/s] [cdeg/s] [ms]
  Rig& r = ensure();
  const float yawRad = static_cast<float>(yawRate) * kCdegToRad;
  const float twistMmS = yawRad * 0.5f * r.engine.effectiveTrackWidth();
  const float speedMmS = static_cast<float>(speed);
  r.engine.wheelsV(speedMmS - twistMmS, speedMmS + twistMmS, durationMs);
}

// ---- wire motion-engine primitives (WireAdapter's WHEELS_X/MOVE_X
// handlers) --------------------------------------------------------------
// Same same-package forward-declaration convention as setWheelsTimed()/
// driveTwistTimed() above -- WireAdapter has no reference of its own to
// this Rig's `engine`, so it forwards through these thin, wire-shaped
// calls instead. Wire-shaped units throughout (mm, mm/s, ms);
// `rotationRad` arrives at engineMoveX() ALREADY converted from the
// wire's milliradian integer (wire_adapter.cpp's mradToRad()). `cruise`
// <= 0 here is MotionEngine's own existing no-op -- the wire's "0 means
// the configured default" substitution (engineDefaultCruiseMmS() below)
// is resolved BEFORE calling these; neither of these two ever sees the
// sentinel itself. Deliberately NOT `//%`-annotated: the block API's
// own startMove() already has a call shape of its own.
void engineWheelsX(float left, float right, float cruise,
                   uint32_t timeoutMs) {  // [mm] [mm] [mm/s] [ms]
  Rig& r = ensure();
  r.engine.wheelsX(left, right, cruise, timeoutMs);
}

void engineMoveX(float distance, float rotationRad, float cruise,
                 uint32_t timeoutMs) {  // [mm] [rad] [mm/s] [ms]
  Rig& r = ensure();
  r.engine.moveX(distance, rotationRad, cruise, timeoutMs);
}

// The wire's "cruise == 0 means the configured default" substitution
// (motion-api.md S1.1: "an X-form's commanded value is a displacement
// ... pass 0 for the configured default"). Sprint 007 ticket 003
// (closing R-11/BLK-03/API-03): this used to derive the substituted
// value from kernel.config().fullDutyVelocity -- this robot's 100%-duty
// ceiling, ~875 mm/s -- so a spec-following host sending `cruise 0`
// got the fastest, least-controlled move the robot can make, ~1.5x the
// speed the project's own bench notes call unusable. fullDutyVelocity
// is the wrong field for this: at the kernel layer, `0` there means
// "uncalibrated, refuse VELOCITY commands entirely"
// (DifferentialDrive::checkCommandable()) -- an unrelated meaning of
// zero that happened to share a variable with this substitution. Now
// returns the Rig's own, independently configured defaultCruiseMmS_
// (seeded 150 mm/s above, settable/gettable via the `default_cruise`
// wire field, ordinal 15 -- setKernelValue()/getConfigValue() below).
// fullDutyVelocity remains the duty CEILING elsewhere in this file and
// the kernel; it is no longer read here. Returns 0 if defaultCruiseMmS_
// itself is non-positive (an operator can still force "no default
// available" via `SET default_cruise 0`) -- wire_adapter.cpp's four
// verb handlers already treat that as a range refusal, not a
// silently-accepted zero-speed command; that refusal logic is
// unchanged by this ticket.
float engineDefaultCruiseMmS() {
  return ensure().defaultCruiseMmS_;
}

// Sprint 005 ticket 004 (closing wire-motion-completion-signal.md/R-23):
// the ONE genuinely new read WireAdapter's own motion-completion
// resolution needs (wire_adapter.cpp's forward declaration, its own
// header comment there) -- true iff MotionEngine's move-engine state
// (MOVE_X/GO_TO_R/GO_TO_W's own tracked segment) is currently active.
// Mirrors moving()'s exact body (further down, this file's `//%`
// block-API surface) but is deliberately its OWN function, not a call
// to moving(): this is a wire-shaped bridge (no `//%`, same
// same-package forward-declaration convention as engineWheelsX() et
// al. above), and must not depend on a block-API function's own
// continued existence or shape -- same rationale setWheelsTimed() gives
// for staying separate from the block API's own setWheels(). `rig ==
// nullptr` (no kernel ever composed -- e.g. a wire session with no
// prior motion verb at all) answers false, same honest "nothing is
// active" default moving() gives.
bool engineMoveActive() {
  return rig != nullptr && rig->engine.isMoveActive();
}

// ---- move engine ----------------------------------------------------

//%
void startMove(int distance, int yaw, int speed, int yawRate) {
  // [mm] [cdeg] [mm/s] [cdeg/s]
  Rig& r = ensure();
  odomUpdate(r);
  const float distanceMm = static_cast<float>(distance);
  const float rotationRad = static_cast<float>(yaw) * kCdegToRad;

  // This shim predates MotionEngine::moveX()'s single-`cruise` wire-
  // shaped signature (motion-api.md S2: move_x(distance,rot) ==
  // wheels_x(distance-rot*b/2, distance+rot*b/2)) -- `blocks/motion.ts`'s
  // block API still passes two INDEPENDENT rate ceilings (speed for the distance
  // axis, yawRate for the yaw axis), picking whichever axis takes
  // LONGER at its own ceiling as the move's shared duration. Reconciled
  // here, not by favoring one of the two legacy rates: derive the
  // single cruise that reproduces the EXACT SAME commanded
  // velocity/twist this dual-rate math has always produced, so
  // move()/whileMoving()'s observable behavior is unchanged.
  //
  // Algebra: moveX()'s own wheels_x-style reduction commands
  // velocity = distTarget/dominant*cruiseCounts (dominant =
  // max(|left|,|right|), in counts). Setting cruiseCounts =
  // dominant/duration -- `duration` computed the OLD way below --
  // makes that velocity equal distTarget/duration exactly, the legacy
  // formula, for ANY distance/yaw/speed/yawRate combination, not only
  // the degenerate straight/pivot cases.
  const float cpm = r.engine.countsPerMm();
  const float b = r.engine.effectiveTrackWidth();
  const float distTargetCounts = distanceMm * cpm;              // [counts]
  const float yawTargetCounts = rotationRad * 0.5f * b * cpm;   // [counts]
  const float speedCounts =
      static_cast<float>(speed > 0 ? speed : 1) * cpm;    // [counts/s]
  const float yawRadPerS =
      static_cast<float>(yawRate > 0 ? yawRate : 1) * kCdegToRad;
  const float twistCounts = yawRadPerS * 0.5f * b * cpm;  // [counts/s]

  // One duration covers both axes -> simultaneous arc completion. This
  // max()-based `duration` is also what derives `cruiseMmS` below
  // (unaffected by the split-aware budget fix further down) -- it is
  // the legacy dual-rate reconciliation the header comment above
  // describes, correct regardless of whether moveX() ends up splitting.
  float distDuration = 0.0f;  // [s]
  if (distTargetCounts != 0.0f)
    distDuration = std::fabs(distTargetCounts) / speedCounts;
  float yawDuration = 0.0f;  // [s]
  if (yawTargetCounts != 0.0f)
    yawDuration = std::fabs(yawTargetCounts) / twistCounts;
  const float duration = distDuration > yawDuration ? distDuration
                                                      : yawDuration;
  if (duration <= 0.0f) return;  // nothing to do

  const float leftCounts = distTargetCounts - yawTargetCounts;
  const float rightCounts = distTargetCounts + yawTargetCounts;
  const float absLeft = std::fabs(leftCounts);
  const float absRight = std::fabs(rightCounts);
  const float dominantCounts = absLeft > absRight ? absLeft : absRight;
  const float cruiseMmS = (dominantCounts / duration) / cpm;  // [mm/s]

  // moveX() (motion_engine.cpp) splits a nonzero distance combined with
  // a large enough rotation into pivot-then-straight -- two SEQUENTIAL
  // segments sharing the one deadline this call sets (motion_engine.h:
  // "NOT reset across a pivot-to-straight phase transition") -- rather
  // than one blended segment where both axes finish together. Budget
  // the SUM of both axes' durations for that case; max() only covers
  // the genuinely simultaneous (non-split) move. Read the split
  // threshold from MotionEngine itself (turnFirstAngleRad(), the public
  // accessor for its own private kTurnFirstAngleRad) rather than
  // retyping the 50 deg constant here, so this decision can never drift
  // from moveX()'s own.
  const bool willSplit =
      distanceMm != 0.0f &&
      std::fabs(rotationRad) >= MotionEngine::turnFirstAngleRad();
  const float budgetDuration =
      willSplit ? (distDuration + yawDuration) : duration;

  // Backstop: for a single segment, this covers the end-of-move taper
  // (serviceMove) -- the last ~15 deg / ~40 mm run at reduced rate,
  // adding up to ~1 s. When the split above fires, it is ALSO the only
  // thing paying for the SECOND phase's own ramp/taper overhead, since
  // one deadline spans both phases. This is moveX()'s own `timeout` --
  // a REAL backstop the wire's own MOVE_X carries as a required field,
  // not an internally re-derived one.
  const uint32_t timeoutMs =
      static_cast<uint32_t>(budgetDuration * 1000.0f) + 1500u;

  r.engine.moveX(distanceMm, rotationRad, cruiseMmS, timeoutMs);
}

//%
bool updateMove() {
  if (rig == nullptr) return false;
  Rig& r = *rig;
  // odomUpdate() only while a move was actually active -- pose stays
  // lazily updated (poseX()/Y()/heading() on demand) otherwise.
  const bool wasActive = r.engine.isMoveActive();
  if (wasActive) odomUpdate(r);
  const bool moveActive = r.engine.serviceMove();
  // Cross-fiber stop delivery (sprint 006 ticket 002, BLK-01(b)): this
  // poller's own call path -- isMoving() (moveProgress() is read-only;
  // see verify-blocks.md's BLK-12 spot check, which confirmed
  // isMoving()'s "checks state only" doc is false but REFUTED that same
  // claim for moveProgress()) -- can end a move at its deadline backstop
  // without tickDrive() ever running. Mirrors tickDrive()'s own
  // wasActive && !moveActive gate, but delivers the port write HERE
  // instead of relying on a settle-loop re-step this call path never
  // runs. See deliverStopNow()'s own comment above for the full
  // write-up.
  if (wasActive && !moveActive) deliverStopNow(r);
  return moveActive;
}

// Forward declaration: commandLooksActive() is defined further down, in
// its own clearly delineated section right before the starvation
// watchdog (it was written there first, for the watchdog's own use) --
// tickDrive() below needs it too now, for its return value (sprint 007
// ticket 002, closes R-10/API-01: see that function's own comment for
// what it checks, and tickDrive()'s own comment below for why).
static bool commandLooksActive(const Rig& r);

// ---- tick engine --------------------------------------------------------
// tickDrive(): the caller-driven replacement for the kernel's own
// now-unwired fiber (see ensure()'s comment). Runs exactly one
// kernel.step() + serviceMove() on the CALLER's fiber, then self-paces
// to the next absolute 24 ms deadline -- the same absolute-deadline
// pacing DifferentialDrive::run() uses, lifted here since run() itself
// is no longer wired to anything. The deadline anchors to the previous
// tick's own deadline while calls stay consecutive (no drift
// accumulates); a gap since the last deadline re-anchors to now rather
// than catching up a burst of overdue ticks.
//
// Always executes the step, even with no move active and no continuous
// command in force: a `while (tickDrive())` loop driving
// setWheels()/driveTwist() must step the kernel every call, or
// continuous-mode driving never progresses.
//
// Returns commandLooksActive(r) -- a move-engine move still in flight,
// OR nonzero applied duty -- computed AFTER serviceMove() runs. NOT raw
// post-serviceMove() moveActive: wheelsV()/wheelsX() clear the move
// planner before tickDrive() is ever called, so a continuous-mode
// `while (tickDrive())` loop reading raw moveActive exited on its very
// first iteration (the starvation watchdog then stopped the robot
// ~150 ms later); commandLooksActive()'s "or nonzero applied duty"
// clause is what keeps the loop running. A position-mode move's final
// tick is unaffected: the settle loop just below already drives
// applied duty to zero before this function returns, so that tick still
// returns false as before. See
// tests/host/test_continuous_drive_command_looks_active.py (pins this
// return value's condition) and
// tests/host/test_regression_post_move_neutral.py (pins the settle
// loop).
//%
bool tickDrive() {
  Rig& r = ensure();
  const uint64_t cycleStartUs = r.clock.nowMicros();
  r.lastTickUs = cycleStartUs;  // the watchdog's only freshness signal

  // Concurrency guard: check-and-set with no intervening yield is
  // atomic on CODAL's cooperative fibers, so this is safe against a
  // second fiber also calling tickDrive() -- it just waits (a short
  // timed poll, since the busy fiber may itself be parked in step()'s
  // settle sleeps) until the flag clears rather than racing
  // kernel.step().
  while (r.stepBusy) {
    r.sleeper.sleepMillis(1);
  }
  r.stepBusy = true;
  r.kernel.step();

  const bool wasActive = r.engine.isMoveActive();
  // odomUpdate() now runs UNCONDITIONALLY, every tick (sprint 006 ticket
  // 003, closes R-09/BLK-05, continuous-mode-odometry-chord-error.md):
  // this used to read `if (wasActive) odomUpdate(r);`, matching
  // updateMove()'s own gate just below -- so continuous-mode driving
  // (setWheels()/driveTwist() under a `while (tickDrive())` loop, no
  // move-engine move ever active) never called this at all, and the
  // next pose read integrated the ENTIRE driven interval as one
  // straight chord at one midpoint heading: wrong by the difference
  // between an arc and its chord, exactly the whole path length for a
  // closed loop (drive a full circle, pose reports ~the path length
  // instead of ~0). odomUpdate() diffs against the last kernel Output
  // it consumed and immediately re-stamps that value, so it is a no-op
  // on a tick with no new encoder movement -- safe to call
  // unconditionally. `wasActive` is still computed here and kept for
  // the settle-loop gate below (`if (wasActive && !moveActive)`), which
  // is a different concern (folding post-move coast counts into pose)
  // and is unaffected by this change. updateMove()'s OWN odometry gate
  // -- a different caller, serving the TypeScript layer's blocking-move
  // poll -- is untouched; see its own comment.
  odomUpdate(r);
  const bool moveActive = r.engine.serviceMove();

  // Move-completion stop delivery (bench root-cause, 2026-08-20): when
  // serviceMove() ends the move it posts kernel.neutral(), but the
  // neutral only reaches the MOTORS on the NEXT kernel.step() -- and a
  // `while (tickDrive())` caller exits the moment we return false, so
  // that step never ran. The wheels then coasted at the last commanded
  // duty until the starvation watchdog's port-level stop (~100-150 ms
  // = +9-13 deg per turn, +15-22 mm per leg): the intermittent tour
  // corruption. (It was intermittent only because the protocol fiber's
  // former co-ticking sometimes delivered this step by accident.) Run
  // one extra step here so the stop lands before we report "done".
  if (wasActive && !moveActive) {
    // Settling before reporting "done": kernel.neutral() only STAGES a
    // zero command, and one extra step's own encoder read can land
    // mid-spin-down, freezing Output -- and every post-move DIAG -- at
    // a nonzero velocity forever (bench chart artifact: wheels "ending"
    // at +4/-2.5 cm/s). settleToRest() (MotionEngine, host-tested) keeps
    // stepping, bounded, until both wheels witness the actual stop --
    // see its own comment (motion_engine.h) for the decision logic.
    // KNOWN GAP: the decision logic is host-tested, but this call and
    // odomUpdate(r) below are only ever exercised against real hardware
    // (flashing) -- both need a real kernel.step() over real encoders.
    // Odometry ownership is unchanged by the extraction: settleToRest()
    // never touches Rig-local x/y/heading -- odomUpdate(r) is still
    // this file's own call, immediately after.
    r.engine.settleToRest();
    odomUpdate(r);  // coast counts -> pose before the final TLM
  }
  r.stepBusy = false;

  // Absolute-deadline self-pacing, lifted from DifferentialDrive::run()
  // (diffdrive.cpp:290-306): read the cadence from the kernel's own
  // config (still 24 ms per sprint.md's Design Rationale) rather than
  // duplicating the constant here.
  const uint64_t periodUs =
      static_cast<uint64_t>(r.kernel.config().cyclePeriod) * 1000ull;
  const bool consecutive =
      r.tickDeadlineUs != 0 && cycleStartUs < r.tickDeadlineUs + periodUs;
  const uint64_t deadlineUs =
      consecutive ? r.tickDeadlineUs + periodUs : cycleStartUs + periodUs;
  r.tickDeadlineUs = deadlineUs;

  const uint64_t nowUs = r.clock.nowMicros();
  if (nowUs < deadlineUs) {
    const uint32_t shortfallMs =
        static_cast<uint32_t>((deadlineUs - nowUs + 999) / 1000);
    r.sleeper.sleepMillis(shortfallMs);
  } else {
    ++r.tickOverrunCount;
    r.sleeper.yield();
  }

  return commandLooksActive(r);
}

// cycleStat(): read-only tick/cycle diagnostics for desk verification
// (and future wire-protocol reporting). 0 = measured cycle period [us],
// 1 = measured busy time [us] (both straight off the kernel's own
// Output, unaffected by who calls step()); 2 = the Rig-level
// tick-overrun counter above (NOT the kernel's own cycleOverrunCount_,
// which only its unused run() increments); 3 = cycleCount (existing
// kernel Output field, likewise unaffected by who calls step()).
// Deliberately NOT exposing more than these four fields -- diagValue()
// above already covers the rest of Output for the wire protocol's DIAG
// verb.
//%
int cycleStat(int which) {
  Rig& r = ensure();
  const DiffDrive::DifferentialDrive::Output out = r.kernel.output();
  switch (which) {
    case 0: return static_cast<int>(out.cyclePeriodMeasured);
    case 1: return static_cast<int>(out.cycleBusy);
    case 2: return static_cast<int>(r.tickOverrunCount);
    case 3: return static_cast<int>(out.cycleCount);
    default: return 0;
  }
}

// ---- starvation watchdog ------------------------------------------------
// The ONLY background fiber left running (every other control cycle
// now runs on whichever caller's fiber invokes tickDrive() above).
// Purely a safety net -- it never drives, only
// stops -- guaranteeing "the robot only moves while something ticks" is
// actually true even when a tick caller (a student's loop, a wire
// session) disappears mid-move. Launched from ensure() via the same
// CodalFiberLauncher the kernel used for its own now-unwired fiber.
//
// Every ~50 ms: if something looks like it is actively commanding the
// wheels AND it has been more than ~100 ms (about 4 tick periods) since
// the last tickDrive() call, force a stop DIRECTLY at the motor-port
// level -- NOT through kernel.neutral() alone, which only takes effect
// on the next step() and may never run again if the caller has truly
// abandoned its loop. This reuses NezhaMotorPort::emergencyStop()
// (nezha_port.cpp:80-85), already proven tick-independent by its
// exact-zero short-circuit in writeShapedDuty(). kernel.neutral() is
// still called too, so whichever fiber resumes ticking finds the
// kernel's own commanded mode already neutral instead of stale.
//
// This is a resumable SOFT stop, a third flavor distinct from both the
// block API's stop() (kernel.neutral(), takes effect on the next step()
// only) and emergencyStop() (kernel.estop() latch + port zero): it
// never touches kernel.estop()/estopLatch_, so a fresh tickDrive() call
// (a new move, or a resumed driveTick() loop) resumes motion
// immediately, with no clearEmergencyStop() needed.
//
// Note: while abandonment persists, this fires on every ~50 ms poll,
// not just once -- kernel.neutral()/moveActive=false/the port zero
// write are all idempotent, and re-asserting zero is the conservative
// choice given commandLooksActive() below can only see stale state
// (nothing refreshes Output without a step()) until ticking resumes.

static constexpr uint32_t kWatchdogPeriodMs = 50;          // [ms]
static constexpr uint64_t kWatchdogTimeoutUs = 100000ull;  // [us] ~4 periods

// The kernel exposes no direct "is the commanded mode non-neutral"
// accessor (Command::mode is private, read only inside step()).
// appliedDutyLeft/Right -- the last duty actually WRITTEN to a motor
// port -- is the most honest available proxy for "is something
// currently driving the wheels": writeShapedDuty()'s exact-zero
// short-circuit (nezha_port.cpp) means a genuinely neutral commanded
// mode reads back as exactly zero here, with no separate Rig-level
// "driving" flag needed. Combined with moveActive, this covers both
// continuous-drive (setWheels/driveTwist and their timed variants) and
// move-engine abandonment.
static bool commandLooksActive(const Rig& r) {
  if (r.engine.isMoveActive()) return true;
  const DiffDrive::DifferentialDrive::Output out = r.kernel.output();
  return out.appliedDutyLeft != 0.0f || out.appliedDutyRight != 0.0f;
}

static void watchdogEntry(void* context) {
  Rig& r = *static_cast<Rig*>(context);
  while (true) {
    r.sleeper.sleepMillis(kWatchdogPeriodMs);
    const uint64_t nowUs = r.clock.nowMicros();
    const uint64_t sinceLastTickUs = nowUs - r.lastTickUs;
    if (sinceLastTickUs <= kWatchdogTimeoutUs) continue;
    if (!commandLooksActive(r)) continue;
    r.kernel.neutral();      // commands neutral for whenever step() next runs
    r.engine.endMove();      // clears the move-engine's own in-flight state
    r.left.emergencyStop();  // port-level zero write, NOW, tick-independent
    r.right.emergencyStop();
  }
}

//%
bool moving() { return rig != nullptr && rig->engine.isMoveActive(); }

//%
int progress() {  // [0..1000]
  if (rig == nullptr) return 1000;
  return rig->engine.progress();
}

//%
void endMove() {
  if (rig == nullptr) return;
  // "stop move" is a full stop, not move-engine bookkeeping alone
  // (stakeholder decision, 2026-08-26): engine.endMove() only stages
  // kernel.neutral() when a move-engine move (startMove/startGoTo) is
  // active. After a continuous-drive command (setWheelSpeeds/
  // driveTwist) no move-engine move is active, so without the explicit
  // kernel.neutral() below nothing disarms the kernel's held commanded
  // velocity mode (up to kLeaseMax, one hour) -- deliverStopNow()'s
  // port-level zero is momentary and the very next step() re-commands
  // the duty. Same three-call shape stopAll() already uses.
  rig->engine.endMove();
  rig->kernel.neutral();
  // Cross-fiber stop delivery (sprint 006 ticket 002): the "stop move"
  // block's own entry point -- see deliverStopNow()'s comment above.
  deliverStopNow(*rig);
}

// ---- stopping -------------------------------------------------------

//%
void stopAll() {
  Rig& r = ensure();
  r.engine.endMove();
  r.kernel.neutral();
  // Cross-fiber stop delivery (sprint 006 ticket 002): the "stop" block
  // and the wire's STOP verb both land here -- see deliverStopNow()'s
  // comment above.
  deliverStopNow(r);
}

//%
void estopAll() {
  Rig& r = ensure();
  r.engine.endMove();
  r.kernel.estop();
  r.kernel.emergencyStopMotors();
}

//%
void estopClear() { ensure().kernel.estopClear(); }

// ---- stall latch: clear path + readback (sprint 007 ticket 001,
// closing R-01/KERN-01) ------------------------------------------------
// The kernel's clearStallLatch()/Output.stallHalted (diffdrive.h/.cpp)
// already existed and were already correct -- this was a MISSING-CALLER
// problem, not missing kernel logic (see the ticket/issue for the full
// review trail). Two thin forwards, exactly like estopClear() above,
// except deliberately NOT routed through estopClear()/estopAll() or any
// new top-level wire verb: the stall latch and the e-stop latch are
// separate fault classes (same principle deliverStopNow() above
// established for a different pair -- a stop must never silently
// become a latch, and clearing one latch must never silently clear the
// other). clearStall() is reachable from a dedicated `blocks/stop.ts` block
// AND (ticket 001) the wire's `stall_clear` SET-action ConfigField
// (setKernelValue() case 17, below); isStalled() backs the matching
// `blocks/stop.ts` readback block, the STATUS `flags` bit 2, and the
// pre-existing diagValue(2) -- three independent ways to read the same
// bit, all sourced from this one Output field.
//%
void clearStall() { ensure().kernel.clearStallLatch(); }

//%
bool isStalled() { return ensure().kernel.output().stallHalted; }

// SerialTransport's writeLine() drop counter (sprint 004 ticket 006),
// read back by case 26 below. Reached by same-package forward
// declaration rather than by including protocol.h: that header pulls
// in radio_transport.h, and PXT's per-file dependency scan then decides
// this file needs the `radio` package and fails the build -- same
// convention protocolEmitLine/protocolRunText already use further down
// this file.
int protocolSerialDropCount();

// Kernel Output accessor, one int per field: booleans 0/1, duty percent
// x100 (10000 == full duty -- Output.appliedDutyLeft/Right already
// arrives here as percent, per diffdrive.h; this multiplies by 100 a
// SECOND time), positions/velocities raw counts. Callers:
// wire_adapter.cpp status(), probe() (TS).
int diagValue(int what) {
  const DiffDrive::DifferentialDrive::Output out = ensure().kernel.output();
  switch (what) {
    case 0: return out.ready ? 1 : 0;
    case 1: return out.estopped ? 1 : 0;
    case 2: return out.stallHalted ? 1 : 0;
    case 3: return out.leaseExpired ? 1 : 0;
    case 4: return out.connectedLeft ? 1 : 0;
    case 5: return out.connectedRight ? 1 : 0;
    case 6: return out.wedgeSuspectLeft ? 1 : 0;
    case 7: return out.wedgeSuspectRight ? 1 : 0;
    case 8: return static_cast<int>(out.i2cFaultCount);
    case 9: return static_cast<int>(out.leaseExpiryCount);
    case 10: return static_cast<int>(out.positionLeft);
    case 11: return static_cast<int>(out.positionRight);
    case 12: return static_cast<int>(out.appliedDutyLeft * 100.0f);
    case 13: return static_cast<int>(out.appliedDutyRight * 100.0f);
    case 14: return static_cast<int>(out.velocityLeft);
    case 15: return static_cast<int>(out.velocityRight);
    case 16: return static_cast<int>(out.cycleCount);
    case 17: return (out.satLeft ? 1 : 0) | (out.satRight ? 2 : 0);
    case 18: return (out.deficitLeft ? 1 : 0) | (out.deficitRight ? 2 : 0);
    case 19: return static_cast<int>(out.cycleOverrunCount);
    // 20: latched first refusal (Status enum: 0 ok, 1 unconfigured,
    // 2 not-begun, 3 estopped, 4 non-finite, 5 cadence-preserved).
    case 20:
      return static_cast<int>(ensure().kernel.lastError());
    // 21/22: peak driven identical-encoder-read streaks (latch evidence)
    case 21: return static_cast<int>(ensure().left.maxDrivenStreak_);
    case 22: return static_cast<int>(ensure().right.maxDrivenStreak_);
    // 23/24: rejected implausible encoder reads (glitch armor)
    case 23: return static_cast<int>(ensure().left.glitchCount_);
    case 24: return static_cast<int>(ensure().right.glitchCount_);
    case 25: return static_cast<int>(ensure().engine.wrongWayCount());
    // 26: SerialTransport::writeLine() drop count (ticket 006) -- the
    // two-writer guard's retry cap exhausted, or a uBit.serial.send()
    // call itself failed. Bench operators read this via probe(26); it
    // should stay 0 during a normal run.
    case 26: return protocolSerialDropCount();
    // 27: sum of both wheels' encoder rebaseline-on-discontinuity
    // events (sprint 006 ticket 005, EncoderGlitchArmor's
    // kAcceptAsRebaseline outcome -- see encoder_glitch_armor.h). A
    // two-strike implausible-then-consistent jump treated as a counter
    // restart (e.g. a brick MCU reset) instead of integrated as a
    // multi-meter teleport. Should read 0 across a normal session with
    // no discontinuities.
    case 27:
      return static_cast<int>(ensure().left.rebaselineCount_ +
                              ensure().right.rebaselineCount_);
    default: return 0;
  }
}

// ---- pose -----------------------------------------------------------

//%
int poseX() {  // [mm]
  Rig& r = ensure();
  odomUpdate(r);
  return static_cast<int>(std::lround(r.x));
}

//%
int poseY() {  // [mm]
  Rig& r = ensure();
  odomUpdate(r);
  return static_cast<int>(std::lround(r.y));
}

//%
int poseHeading() {  // [cdeg]
  Rig& r = ensure();
  odomUpdate(r);
  return static_cast<int>(std::lround(r.heading * kRadToCdeg));
}

//%
void resetPose() {
  Rig& r = ensure();
  odomUpdate(r);  // consume any pending deltas first
  r.x = 0.0f;
  r.y = 0.0f;
  r.heading = 0.0f;
}

// ---- configuration --------------------------------------------------

//%
void setGeometry(int trackWidth, int calib) {  // [0.1 mm] [1e-4 mm/deg]
  Rig& r = ensure();
  if (trackWidth > 0)
    r.engine.setTrackWidth(static_cast<float>(trackWidth) * 0.1f);
  if (calib > 0) r.engine.setTravelCalib(static_cast<float>(calib) * 1e-4f);
}

//%
void setKernelValue(int field, int value) {  // [x1000 scaled]
  Rig& r = ensure();
  const float v = static_cast<float>(value) * 0.001f;
  DiffDrive::DifferentialDrive& k = r.kernel;
  switch (field) {
    case 0: k.setMaxDuty(v); break;
    case 1: k.setFullDutyVelocity(v); break;
    case 2: k.setKp(v); break;
    case 3: k.setKi(v); break;
    case 4: k.setIMax(v); break;
    case 5: k.setKaff(v); break;
    case 6: k.setPidMax(v); break;
    case 7: k.setTwistHoldGain(v); break;
    case 8: k.setSpeedFloor(v); break;
    case 9: k.setPositionErrorMax(v); break;
    case 10: k.setStall(v, k.config().stallDemand,
                        k.config().stallWindow); break;
    case 11: k.setStall(k.config().stallSpeed, v,
                        k.config().stallWindow); break;
    case 12: k.setStall(k.config().stallSpeed, k.config().stallDemand,
                        v); break;
    case 13: k.setLambdaEnabled(v != 0.0f); break;
    case 14: k.setCrawlPulse(v); break;
    // 15 (sprint 007 ticket 003, closing R-11/BLK-03/API-03):
    // default_cruise -- the wire layer's OWN configured-default cruise
    // field (Rig::defaultCruiseMmS_, NOT kernel.config()), see
    // engineDefaultCruiseMmS()'s own comment above. Same ">0" silent-
    // ignore validation style as setGeometry() -- a `SET default_cruise
    // 0` line over the wire is accepted (kOk) but does not clear the
    // field to 0; that is only reachable via the test double's own
    // direct setter (there is no wire-level way to force "no default
    // available" at ordinal 15, deliberately -- unlike stall_clear's
    // ordinal 17 below, this is a real stored value, not an action).
    case 15: if (v > 0.0f) r.defaultCruiseMmS_ = v; break;
    // 16 (ticket 005, closing R-14/API-06): rotational_slip -- a thin
    // forward to the now-tested MotionEngine::setRotationalSlip(),
    // which already applies the ">0, else keep the prior value"
    // validation itself (motion_engine.h); no duplicate check needed
    // here, unlike case 15's own inline check above (defaultCruiseMmS_
    // has no dedicated setter to own that validation).
    case 16: r.engine.setRotationalSlip(v); break;
    // 17 (ticket 001): stall_clear -- a write-triggered ACTION wearing a
    // config-field's clothes, not a stored value. Only nonzero-vs-zero
    // matters (mirrors the x1000 scaling convention: a wire
    // `SET stall_clear 1` arrives here as value=1000, v=1.0f); the
    // magnitude is otherwise ignored. Deliberately does not touch
    // estopLatch_ -- see clearStall()'s own comment above.
    case 17: if (v != 0.0f) k.clearStallLatch(); break;
    default: break;
  }
}

// ---- config read-back -------------------------------------------------
// The read-back counterpart to setKernelValue() above: same ordinals,
// same x1000 scaling. Reads config() (the kernel's `staged_` Config,
// written synchronously by every setter, so this always reflects the
// true current value). Deliberately NOT `//%`-annotated -- C++-internal;
// wire_adapter.cpp is the only caller, via its own same-package forward
// declaration. An out-of-range field returns 0.
int getConfigValue(int field) {  // -> [x1000 scaled]
  Rig& r = ensure();
  const DiffDrive::DifferentialDrive::Config c = r.kernel.config();
  float v = 0.0f;
  switch (field) {
    case 0: v = c.maxDuty; break;
    case 1: v = c.fullDutyVelocity; break;
    case 2: v = c.kp; break;
    case 3: v = c.ki; break;
    case 4: v = c.iMax; break;
    case 5: v = c.kaff; break;
    case 6: v = c.pidMax; break;
    case 7: v = c.twistHoldGain; break;
    case 8: v = c.vMin; break;
    case 9: v = c.posErrMax; break;
    case 10: v = c.stallSpeed; break;
    case 11: v = c.stallDemand; break;
    case 12: v = c.stallWindow; break;
    case 13: v = c.lambdaEnabled ? 1.0f : 0.0f; break;
    case 14: v = c.crawlPulse; break;
    // 15 (sprint 007 ticket 003): default_cruise's GET side --
    // deliberately NOT read from `c` (this ordinal has no stored
    // kernel Config field at all; it lives on Rig, see
    // defaultCruiseMmS_'s own field comment above).
    case 15: v = r.defaultCruiseMmS_; break;
    // 16 (ticket 005): rotational_slip's GET side -- a thin forward to
    // MotionEngine::rotationalSlip(), deliberately NOT read from `c`
    // (this ordinal has no kernel Config field at all; it lives on
    // MotionEngine, see setKernelValue() case 16's own comment above).
    case 16: v = r.engine.rotationalSlip(); break;
    // 17 (ticket 001): stall_clear's GET side -- a convenience readback
    // of Output.stallHalted, deliberately NOT read from `c` (this
    // ordinal has no stored Config field at all; see clearStall()'s own
    // comment above and sprint 007's design/DESIGN.md §5 field table).
    case 17: v = r.kernel.output().stallHalted ? 1.0f : 0.0f; break;
    default: return 0;
  }
  return static_cast<int>(std::lround(v * 1000.0));
}

// ---- OTOS (zeguz bench bring-up, 2026-08-20) ------------------------
// Thin shim surface over OtosPort (otos_port.h). Same integer boundary
// convention as the rest of this file. BUS DISCIPLINE: call these only
// from the same fiber that calls tickDrive() -- an OTOS transaction
// interposed in the Nezha encoder's select->read window destroys the
// encoder sample (Phase F). Lazy singleton, separate from Rig: the
// sensor is usable without ever starting the drive kernel.

static OtosPort* gOtos = nullptr;

static OtosPort& otosRef() {
  if (gOtos == nullptr) gOtos = new OtosPort();
  return *gOtos;
}

// ---- wire motion-engine primitives, part 2 (WireAdapter's MOVE_V/
// GO_TO_R/GO_TO_W handlers) -----------------------------------------------
// Same forward-declaration convention as engineWheelsX()/engineMoveX()/
// engineDefaultCruiseMmS() above. `omegaRad` arrives at engineMoveV()
// ALREADY converted from the wire's milliradian integer
// (wire_adapter.cpp's mradToRad()); `speed`'s <0/==0 "configured
// default" substitution is resolved by onGoToR()/onGoToW() BEFORE
// either of these is ever called, via engineDefaultCruiseMmS() above.
// Placed after otosRef(), not with engineWheelsX()/engineMoveX() above,
// because engineGoToW() below needs it.
void engineMoveV(float vx, float omegaRad, uint32_t durationMs) {
  Rig& r = ensure();
  r.engine.moveV(vx, omegaRad, durationMs);
}

// Deliberately NOT `//%`-annotated any more (sprint 015 ticket 006) --
// this is now the WIRE layer's own private forward only, called
// exclusively from wire_adapter.cpp's onGoToR() via that file's
// same-package forward declaration (kept 5-parameter, matching this
// definition exactly, per this file's own header comment). The block
// layer reaches the identical `r.engine.goToR()` call through
// engineGoToRArmed()/engineSetGoToDeadline() below instead -- see
// those functions' comments for why the split exists. Wire-shaped
// units (mm, mm/s, ms); cm-to-mm conversion stays the TS caller's job,
// exactly as startMove() already does for _startMove().
void engineGoToR(float x, float y, float speed, float arrive, uint32_t timeoutMs) {
  Rig& r = ensure();
  r.engine.goToR(x, y, speed, arrive, timeoutMs);
}

// `//%`-annotated -- pre-arms the NEXT engineGoToRArmed() call's
// deadline. Split out of what used to be a single five-parameter
// engineGoToR() shim (sprint 015 ticket 006): a real PXT build of that
// version reproduced "TS9200: Assertion failed" deterministically --
// twice, including after make_deploy.py's one automatic retry for the
// benign packaging-abort shape tools/DESIGN.md documents under the
// same error code, so this was the ARITY, not a nondeterministic
// abort. setTaperWindows()'s own comment already recorded an earlier
// incident with the identical symptom; this build is what confirmed
// it. Every `//%` shim in this file now stays at <=4 params. See
// Rig::pendingGoToDeadlineMs_ (above, in the struct) for the handoff
// contract -- one caller only (sim.ts's _setGoToDeadline(), called by
// motion.ts's startGoTo() immediately before _goToR()), so there is no
// path for a stale deadline to reach an unrelated move.
//%
void engineSetGoToDeadline(uint32_t timeoutMs) {
  ensure().pendingGoToDeadlineMs_ = timeoutMs;
}

// `//%`-annotated -- the block layer's own entry point onto the SAME
// goToR() the wire's GO_TO_R verb reaches via engineGoToR() above,
// just split to FOUR parameters (engineSetGoToDeadline() immediately
// above supplies the fifth, `timeoutMs`, via
// Rig::pendingGoToDeadlineMs_) -- see that function's comment for why.
// Deliberately delegates to engineGoToR() above rather than calling
// r.engine.goToR() directly a second time, so the actual move-engine
// call site stays in exactly one place.
//%
void engineGoToRArmed(float x, float y, float speed, float arrive) {
  Rig& r = ensure();
  engineGoToR(x, y, speed, arrive, r.pendingGoToDeadlineMs_);
}

// GO_TO_W's own PoseSource selection: the ONE place this project
// decides which PoseSource serves a GO_TO_W call, via
// selectPoseSource() (encoder_pose_source.h) -- this file's
// `gOtos`/otosRef() lazy singleton when `connected()` (initialized AND
// actually talking to the chip), the Rig-owned `encoderPose`
// (dead-reckoned, drifting, but always available) otherwise. A robot
// with no OTOS fitted, or one whose OTOS was never begun/matched, now
// drives on encoder odometry instead of refusing the call outright --
// GO_TO_W is no longer a no-op on the fleet's OTOS-less robots (tovez,
// gopiv, zeguz).
//
// This always dispatches onto MotionEngine::goToW() now, so the bool
// return is unconditionally true; kept (rather than void) only because
// wire_adapter.cpp's contract for this entry point ("was a live pose
// actually available") is otherwise unchanged. The return value still
// does NOT distinguish "served by OTOS" (accurate) from "served by
// encoder odometry" (drifts, no correction) -- a caller that needs to
// know reads STATUS's `otos=` flag.
//
// Mid-move OTOS disconnection is not a race to guard against: goToW()
// reads its PoseSource exactly ONCE, at call time, before ever
// delegating to goToR() -- nothing re-reads or re-selects a pose source
// while a move is in flight.
bool engineGoToW(float x, float y, float speed, float arrive,
                uint32_t timeoutMs) {
  OtosPort& otos = otosRef();
  Rig& r = ensure();
  PoseSource& pose = selectPoseSource(otos.connected(), otos, r.encoderPose);
  r.engine.goToW(pose, x, y, speed, arrive, timeoutMs);
  return true;
}

// Set end-of-move shaping. Larger tapers and lower floors buy accuracy
// with time; a closed-loop caller that re-fixes between moves should
// spend far less of it. Zero or negative leaves a field unchanged.
// NOTE: kept to TWO arguments each. A single five-argument shim made
// the PXT compiler fail with "TS9200: Assertion failed" -- reported
// against this project's single pre-sprint-012-split top-level file at
// (1,1), nowhere near the real cause. The `//%` marker
// must also sit IMMEDIATELY above the signature; a comment between
// them makes the scanner miss the function entirely.
//%
void setTaperWindows(int distCounts, int yawCounts) {
  Rig& r = ensure();
  if (distCounts > 0) r.engine.setDistTaper(static_cast<float>(distCounts));
  if (yawCounts > 0) r.engine.setYawTaper(static_cast<float>(yawCounts));
}

//%
void setTaperFloors(int distPct, int turnPct) {
  Rig& r = ensure();
  if (distPct > 0)
    r.engine.setDistFloor(static_cast<float>(distPct) * 0.01f);
  if (turnPct > 0)
    r.engine.setTurnFloor(static_cast<float>(turnPct) * 0.01f);
}

//%
void setRampMs(int ms) {
  Rig& r = ensure();
  if (ms > 0) r.engine.setRampMs(static_cast<float>(ms));
}

// Measured wheel speed [mm/s] straight from the kernel's per-tick
// encoder measurement. 1 count = 0.1 deg of shaft, so counts/s * the
// travel calibration / 10 gives mm/s.
int wheelSpeed(int which) {
  Rig& r = ensure();
  const DiffDrive::DifferentialDrive::Output out = r.kernel.output();
  const float counts = (which == 0) ? out.velocityLeft : out.velocityRight;
  return static_cast<int>(std::lround(counts * r.engine.travelCalib() * 0.1f));
}

// Expose diagValue() to the TS layer for on-device instrumentation.
// The values it carries (applied duty, encoder positions, wedge
// suspicion) are exactly what a failing move needs recorded PER TICK,
// and they cannot be polled from the host during a move: a
// request/reply round-trip inside a move over the relay is measured to
// be actively dangerous (a 197.5 mm leg collapsed to 0.3 mm). A test
// program samples into arrays and dumps afterwards instead.
//%
int probe(int what) { return diagValue(what); }

//%
int otosBegin() {  // -> raw product id, for diagnostics only; readiness
                    // is OtosPort::connected(), gated on otos_port.h's
                    // kExpectedProductId -- not this return value
  OtosPort& o = otosRef();
  o.begin();
  return o.productId();
}

//%
bool otosRead() { return otosRef().read(); }

//%
int otosGet(int what) {
  OtosPort& o = otosRef();
  switch (what) {
    case 0: return static_cast<int>(std::lround(o.x() * 10.0f));  // [0.1 mm]
    case 1: return static_cast<int>(std::lround(o.y() * 10.0f));  // [0.1 mm]
    case 2: return static_cast<int>(std::lround(o.heading() * kRadToCdeg));
    case 3: return static_cast<int>(std::lround(o.vx()));   // [mm/s]
    case 4: return static_cast<int>(std::lround(o.vy()));   // [mm/s]
    case 5: return static_cast<int>(std::lround(o.omega() * kRadToCdeg));
    case 6: return o.productId();
    case 7: return o.connected() ? 1 : 0;
    case 8: return o.imuCalibrationSamplesRemaining();
    default: return 0;
  }
}

//%
void otosZero() { otosRef().zeroPose(); }

//%
void otosCalibrate(int samples) {
  otosRef().calibrateImu(static_cast<uint8_t>(samples));
}

// Emit a test-result line on BOTH transports. TypeScript's
// serial.writeLine reaches the USB cable only, and the USB cable only
// reaches the bench stand -- where the wheels are off the ground, so
// nothing that needs real motion can be measured there. Test programs
// use this instead, and their results come back over the radio when the
// robot is on the playfield.
//
// Reached by same-package forward declaration rather than by including
// protocol.h: that header pulls in radio_transport.h, and PXT's
// per-file dependency scan then decides this file needs the `radio`
// package and fails the build. Same convention protocol.cpp already
// uses to reach into this file.
void protocolEmitLine(const char* text);

//%
void emitLine(String text) {
  if (text == nullptr) return;
  ManagedString ms = MSTR(text);
  protocolEmitLine(ms.toCharArray());
}

// Read back the text of the RUN command a run event refers to (`slot`
// is the event value; protocol.cpp parks the payload and sends only the
// slot, because an event value is a uint16 and cannot carry a name).
// Same forward-declaration convention as protocolEmitLine above.
const char* protocolRunText(int slot);

//%
String runCommandText(int slot) {
  const char* text = protocolRunText(slot);
  size_t len = 0;
  while (text[len] != '\0') ++len;
  return mkString(text, static_cast<int>(len));
}

//%
void otosSetOffset(int x, int y, int yaw) {  // [0.1 mm] [0.1 mm] [cdeg]
  otosRef().setOffset(static_cast<float>(x) * 0.1f,
                      static_cast<float>(y) * 0.1f,
                      static_cast<float>(yaw) * kCdegToRad);
}

// V6 SEED (protocol-v6-spec.md 5.5): declare the world pose from an
// external fix. Writes BOTH pose sources -- the OTOS position register
// (lever arm applied) and this file's encoder odometry -- so the two
// start agreed and their later divergence IS the drift being measured.
//%
void seedPose(int x, int y, int heading) {  // [mm] [mm] [cdeg]
  Rig& r = ensure();
  odomUpdate(r);  // consume pending deltas before overwriting
  const float h = static_cast<float>(heading) * kCdegToRad;
  r.x = static_cast<float>(x);
  r.y = static_cast<float>(y);
  r.heading = h;
  otosRef().setPose(static_cast<float>(x), static_cast<float>(y), h);
}

}  // namespace diffDrive
