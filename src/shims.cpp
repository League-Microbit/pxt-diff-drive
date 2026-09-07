// shims.cpp -- the MakeCode-facing C++ surface. Composes the DiffDrive
// kernel (self-contained control law) with two NezhaMotorPorts and the
// CODAL platform ports, and adds the application-layer pieces the
// kernel deliberately does not contain:
//
//   - ODOMETRY: differential dead-reckoning from the kernel's Output
//     positions (the kernel is counts-native and has no chassis
//     geometry; track width and travel calibration live HERE).
//   - MOVE ENGINE: position-mode moves (distance+yaw, and goto via the
//     TS layer's arc math) live in motion/motion_engine.{h,cpp}; what
//     this file adds is the MotionEngine member composed onto the
//     kernel and the forwards onto it. The TypeScript layer polls
//     updateMove() -- blocking and loop-style forms are both built on
//     that poll -- and updateMove() and the tick engine below both
//     drive the engine's one service().
//   - TICK ENGINE: tickDrive() runs one kernel.step() + service()
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
#include "core/bus_guard.h"
#include "core/diffdrive.h"
#include "motion/motion_engine.h"
#include "motion/odometry.h"
#include "platform/nezha_port.h"
#include "platform/otos_port.h"
#include "platform/platform_ports.h"
#include "comms/run_registry.h"

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

// Forward declarations: motion-owner arbitration lives on the Protocol
// singleton (comms/protocol.cpp) -- it is the one object that can see a
// wire request, a dispatched job, AND a block program's own call, all
// three. Same same-package forward-declaration convention as
// protocolEmitLine()/protocolCurrentRunText() elsewhere in this file.
// protocolTryTakeMotionOwnership() returns true either because this
// call is the CURRENTLY-DISPATCHING RUN job's own move (recognized by
// fiber identity -- see core/motion_owner.h's tryTakeMotionOwnership()
// and comms/protocol.cpp's Protocol::tryTakeMotionOwnership()) or
// because nothing else currently holds the drivetrain, in which case
// it takes kBlock; otherwise it leaves motionOwner_ alone and returns
// false -- refused, not silently superseded.
// protocolReleaseBlockOwnership() is a no-op unless this fiber's own
// call actually holds kBlock (a dispatched job's own move never does
// -- dispatchJob() owns clearing kJob itself).
bool protocolTryTakeMotionOwnership();
void protocolReleaseBlockOwnership();

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

  // The dead-reckoned pose (motion/odometry.h): the kernel-Output
  // integration, its wheel baseline, the rebase-epoch guard, and the
  // PoseSource GO_TO_W falls back to when no OTOS is fitted -- one
  // object, where this struct used to carry five loose fields, a free
  // odomUpdate() over them, and a separate read-only adapter bound to
  // them by const reference. Reads the geometry it integrates with
  // (countsPerMm/effectiveTrackWidth) from `engine` above by reference,
  // so it MUST be declared after it: members initialize in DECLARATION
  // order regardless of this struct's own (implicit) member-initializer
  // order, the same rule this file's header comment already states for
  // `engine` itself. Lives exactly as long as this Rig (a process-
  // lifetime lazy singleton, see `rig`/`ensure()` below).
  // engineGoToW() (below) is this project's one selection point between
  // this and `otosRef()`'s OtosPort.
  Odometry odometry{engine};

  // tick engine (sprint 002): caller-driven stepping replaces the
  // kernel's own now-unwired fiber pacer -- see ensure(), tickDrive(),
  // and the starvation watchdog section below.
  uint64_t lastTick = 0;       // [us] clock.nowMicros() at the start
                                  // of the most recent tickDrive() call
                                  // -- the watchdog's only freshness
                                  // signal. 0 = no tick has run yet.
  uint64_t tickDeadline = 0;   // [us] tickDrive()'s own absolute-
                                  // deadline pacing anchor. 0 = no tick
                                  // has run yet -- re-anchor to now.
  // Bus-ownership guard (core/bus_guard.h): serializes kernel.step()
  // inside tickDrive() against a second fiber also calling tickDrive()
  // -- see that function's own comment -- AND, as of this change,
  // against every OTOS shim entry point below (otosBegin/Read/Zero/
  // Calibrate/SetOffset, seedPose), which now acquire/release the SAME
  // guard around their own I2C body. Formerly a bare `bool stepBusy`
  // known only to tickDrive(); promoted to BusGuard so "the bus has
  // exactly one owner at a time" covers every I2C-touching entry
  // point, not just the kernel step.
  BusGuard busGuard;
  // Deferred OTOS zero: SET rebase
  // (cfgSetRebase()) used to call otosRef().setPose(0,0,0)
  // SYNCHRONOUSLY on whichever fiber issued it (typically the protocol
  // fiber), with no relationship to busGuard at all -- exactly the
  // hole this ticket closes for the other six entry points. Setting
  // this flag instead defers the actual I2C write to tickDrive(),
  // after busGuard.release() (see that function's own comment for the
  // exact point), the same deferred-request shape
  // kernel.rebasePosition() already uses for the kernel's own position
  // reference. kernel.rebasePosition() and the encoder-odometry x/y/
  // heading reset stay SYNCHRONOUS, as before -- only the OTOS write
  // becomes deferred.
  bool pendingOtosZero = false;
  // Staged cross-fiber stop: a stop requested while busGuard is held
  // (i.e. some fiber is mid kernel.step(), possibly parked in its own
  // encoder settle sleep) cannot write the motor ports immediately
  // without racing THAT fiber's own I2C traffic on the shared bus --
  // the exact hazard busGuard exists to prevent. Setting this flag
  // instead defers the port-level zero write to the fiber that already
  // holds the guard, delivered from inside tickDrive() itself, still
  // inside the guarded window, right before it releases the guard (see
  // that function's own comment for the exact point). When the guard
  // is free (the overwhelming majority of stops), softStop() still
  // writes immediately -- this flag is never touched for that path.
  bool pendingStop_ = false;

  // The ONE soft stop this file has: kernel-neutral plus the
  // port-level zero, staged behind busGuard when the guard is held.
  // Defined out-of-line below (right where the free function it
  // replaces used to sit), because its contract needs a full essay and
  // this struct is already long. Every stop path in this file calls
  // it: stopAll(), endMove(), the starvation watchdog, and
  // updateMove()'s move-completion branch.
  void softStop();

  // Plain, no-capture function pointer the protocol fiber registers
  // (registerTickServiceHook(), below) once it starts. tickDrive() calls
  // it, if set, once per call -- see that function's own comment for the
  // exact point (after this tick's own kernel.step()/settle work is
  // done, before the pacing sleep) and why: this is what lets a
  // dispatched RUN job's own `while (driveTick())` tick loop carry the
  // protocol fiber's other duties (wire/radio poll, telemetry, dispatch)
  // along with it, one tick at a time, instead of needing a second fiber
  // to service them concurrently. A plain function pointer, not
  // std::function, for the same reason NezhaMotorPort/OtosPort avoid
  // anything heap-allocating in this file's own composition.
  void (*serviceHook)() = nullptr;

  // The wire's OWN "0 = use the configured default" convenience field,
  // deliberately SEPARATE from kernel.config().fullDutyVelocity. Those
  // are two unrelated meanings of zero that used to be collapsed onto
  // one field: the wire layer's sentinel (this) vs. the kernel's own
  // "0 = uncalibrated, refuse VELOCITY" gate
  // (DifferentialDrive::checkCommandable()). See engineDefaultCruise()
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
  float defaultCruise_ = 150.0f;  // [mm/s]

  // The deadline the NEXT go-to gets: an ordinary config field
  // (`goto_timeout`, ordinal 39, kConfigAccessors below), backed by
  // this Rig member exactly the way defaultCruise_ above backs
  // `default_cruise`, so a bench host can `GET`/`SET` it. This is where
  // engineSetGoToDeadline() parks the fifth argument engineGoToRArmed()
  // cannot take -- see that shim for why it cannot.
  //
  // NOT one-shot: nothing zeroes it once a go-to consumes it. The block
  // layer sets it immediately before every _goToR() (motion.ts's
  // startGoTo()), so the last writer wins and a wire-set value is
  // overwritten by the next block-issued go-to. That ORDERING is the
  // contract.
  uint32_t goToDeadline = 0;  // [ms]

  // The go-to entry point's own yaw-rate ceiling
  // (engineSetGoToYawRate()/engineGoToRArmed() below) -- a SEPARATE
  // field/setter pair rather than a 5th engineGoToRArmed() parameter,
  // for the same reason goToDeadline above is one. Deliberately still a
  // plain Rig field rather than a config row: the wire's own GO_TO_R
  // carries no yaw-rate ceiling, so there is no wire-side counterpart
  // for it to become one row of. One caller (sim.ts's
  // _setGoToYawRate(), called by motion.ts's startGoTo() immediately
  // before _goToR(), alongside _setGoToDeadline()).
  float pendingGoToYawRate_ = 0.0f;  // [cdeg/s]
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
    // K5 (this ticket, design motion-profile-unification.md
    // S4.5): the floor now lives in MotionLimits::vFloor (motion_limits.h,
    // default 70 mm/s -- the same MEASURED tovez/gopiv 2026-08-29 value
    // this field used to carry, see that field's own comment for the
    // full history: captures/tovez-taper-20260829/variants.json,
    // captures/gopiv-floor70-20260829/). A kernel floor under an
    // already-shaped profile is exactly the double-decision this design
    // removes (S3: "the ratio-preserving speed floor moves from the
    // kernel to the profiler") -- applySpeedFloor() (diffdrive.cpp)
    // stays in the vendored kernel for upstream firmware that still
    // wants it, but is inert here with vMin pinned at 0.
    cfg.vMin = 0.0f;                 // [counts/s]
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
    //
    // Raised 2.0 -> 4.0 (sprint 031 ticket 015). MEASURED tovez
    // 2026-09-05, firmware 1.20260904.5, six-and-twelve alternating
    // +-600 mm legs at cruise 100 mm/s, camera-truthed, gain applied
    // live via `SET twist_hold_gain` (captures/session-b-20260905/):
    //   gain 2 (old default)  mean |dheading| 2.88 deg over 18 legs
    //                         (g3-cruise100/, g3-cruise100-x12/)
    //   gain 4                mean |dheading| 2.10 deg over 12 legs
    //                         (twist-4-x12/)
    //   gain 6                mean |dheading| 1.67 deg over  6 legs
    //                         (twist-6/)
    // Gain 4 is the best of the three on the largest sample (12 legs),
    // so it is the new default. Two caveats this comment does NOT
    // smooth over: a 6-leg run at gain 4 gave 0.98 deg and did NOT
    // replicate at 12 legs (2.10) -- six legs is not enough at this
    // noise level, trust the 12-leg number. And gain 6's 1.67 deg is
    // itself only 6 legs, not comparable to the 12/18-leg figures above
    // -- it is NOT evidence gain 6 beats gain 4; that arm needs its own
    // 12-leg rerun before anyone bakes it.
    cfg.twistHoldGain = 4.0f;        // [1/s]
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
// The integration itself, its wheel baseline and the rebase-epoch guard
// all live on Rig::odometry (motion/odometry.h) now. This helper is the
// one thing that did NOT move: fetching the kernel's current Output.
// Odometry deliberately does not hold the kernel -- it integrates
// whatever Output it is handed, which is what makes it host-testable
// against a scripted wheel path with no kernel in the link at all.
static void odomUpdate(Rig& r) { r.odometry.update(r.kernel.output()); }

// ---- the soft stop --------------------------------------------------
// THE one soft stop this file has. Every stop path calls it and none
// writes the sequence out again: stopAll() (the `stop` block and the
// wire's STOP verb), endMove() (the `stop move` block), the starvation
// watchdog, and updateMove()'s own move-completion branch. (It absorbed
// the free function deliverStopNow(), a name src/DESIGN.md,
// tests/host/fake_ports.h and several code-review documents still use.)
//
// The three parts, in order:
//
//   engine.endMove()  clears the move engine's own in-flight state, so
//                     a later service() cannot re-command from it.
//   kernel.neutral()  disarms the kernel's HELD commanded velocity (a
//                     continuous drive holds up to kLeaseMax, one
//                     hour); without it the port zero below is
//                     momentary, because the very next step()
//                     re-commands the duty. Unconditional, so a stop
//                     after setWheels()/driveTwist() gets it too
//                     (stakeholder decision, 2026-08-26).
//   the port write    delivers the stop NOW.
//
// Why PORT-LEVEL (R-08/BLK-01): kernel.neutral() only STAGES a zero
// (diffdrive.cpp), delivered solely on a LATER kernel.step(), and
// step() writes duty BEFORE its two ~4 ms-per-wheel encoder settle
// sleeps. A stop issued from a fiber that is not the one inside step()
// therefore stages a neutral nobody delivers -- and if that same call
// is what ended a `while (tickDrive())` loop, not until the starvation
// watchdog fires ~100-150 ms later. Writing NezhaMotorPort::
// emergencyStop() on both motors lands the stop in the SAME tick,
// synchronously on the calling fiber, with no new ticker and no edit to
// the vendored kernel. Never kernel.emergencyStopMotors(): that also
// latches estopLatch_ (diffdrive.cpp), turning this resumable soft stop
// into a hard e-stop needing clearEmergencyStop().
//
// Staged while busGuard is held: the port write would otherwise race
// the I2C traffic of whichever OTHER fiber holds the guard (mid
// kernel.step(), possibly parked in its own settle sleep) -- the exact
// collision the guard exists to prevent. Rig::pendingStop_ hands the
// write to that fiber instead, delivered from inside tickDrive() just
// before it releases the guard, still within the same tick. When the
// guard is free (the common case) the write happens here, immediately.
void Rig::softStop() {
  engine.endMove();
  kernel.neutral();
  if (busGuard.held()) {
    pendingStop_ = true;
    return;
  }
  left.emergencyStop();
  right.emergencyStop();
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
  // Refused (a silent no-op), not superseding, while a wire motion or a
  // GENUINE block/job COLLISION already holds the drivetrain -- a
  // dispatched RUN job's own call proceeds instead, see
  // protocolTryTakeMotionOwnership()'s own comment above. Also the entry
  // point startDrive() (blocks/motion.ts) reaches, since it calls this
  // same block-facing driveTwist() before starting its own tick loop.
  if (!protocolTryTakeMotionOwnership()) return;
  Rig& r = ensure();
  const float yaw = static_cast<float>(yawRate) * kCdegToRad;  // [rad]
  const float twist = yaw * 0.5f * r.engine.effectiveTrackWidth();  // [mm/s]
  const float vx = static_cast<float>(speed);  // [mm/s]
  r.engine.wheelsV(vx - twist, vx + twist,
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
                    uint32_t duration) {  // [mm/s] [mm/s] [ms]
  Rig& r = ensure();
  // WHEELS supersedes any in-flight move-engine move -- wheelsV() itself
  // clears it (motion-api.md S6, motion_engine.h).
  r.engine.wheelsV(static_cast<float>(left), static_cast<float>(right),
                   duration);
}

void driveTwistTimed(int speed, int yawRate,
                     uint32_t duration) {  // [mm/s] [cdeg/s] [ms]
  Rig& r = ensure();
  const float yaw = static_cast<float>(yawRate) * kCdegToRad;  // [rad]
  const float twist = yaw * 0.5f * r.engine.effectiveTrackWidth();  // [mm/s]
  const float vx = static_cast<float>(speed);  // [mm/s]
  r.engine.wheelsV(vx - twist, vx + twist, duration);
}

// ---- wire motion-engine primitives (WireAdapter's WHEELS_X/MOVE_X
// handlers) --------------------------------------------------------------
// Same same-package forward-declaration convention as setWheelsTimed()/
// driveTwistTimed() above -- WireAdapter has no reference of its own to
// this Rig's `engine`. Wire-shaped units throughout (mm, mm/s, ms);
// `rotation` arrives at engineMoveX() ALREADY converted from the wire's
// milliradian integer (wire_adapter.cpp's mradToRad()). The wire's
// "0 means the configured default" substitution (engineDefaultCruise()
// below) is resolved BEFORE these are called, so neither ever sees the
// sentinel and `cruise <= 0` here is MotionEngine's own no-op.
void engineWheelsX(float left, float right, float cruise,
                   uint32_t timeout) {  // [mm] [mm] [mm/s] [ms]
  Rig& r = ensure();
  r.engine.wheelsX(left, right, cruise, timeout);
}

void engineMoveX(float distance, float rotation, float cruise,
                 uint32_t timeout) {  // [mm] [rad] [mm/s] [ms]
  Rig& r = ensure();
  r.engine.moveX(distance, rotation, cruise, timeout);
}

// The wire's "cruise == 0 means the configured default" substitution
// (motion-api.md S1.1: "an X-form's commanded value is a displacement
// ... pass 0 for the configured default"). Returns the Rig's own
// defaultCruise_ -- the `default_cruise` wire field, ordinal 15 --
// NOT kernel.config().fullDutyVelocity, which is the 100%-duty ceiling
// (~875 mm/s, so `cruise 0` meant the fastest move the robot can make)
// and whose own zero means the unrelated "uncalibrated, refuse
// VELOCITY" (DifferentialDrive::checkCommandable()). Returns 0 when
// defaultCruise_ is non-positive, an operator's way to force "no
// default available"; wire_adapter.cpp's verb handlers treat that as a
// range refusal, not a silently-accepted zero-speed command.
float engineDefaultCruise() {  // [mm/s]
  return ensure().defaultCruise_;
}

// SUC-003: two more wire-layer forwards, same same-package
// forward-declaration convention as engineDefaultCruise() above.
// engineADecel() returns MotionLimits::decel, which defaults to 400 and
// can never be set to 0 -- so onMoveX()'s own `engineADecel() > 0.0f`
// selector (wire_adapter.cpp) ALWAYS takes the distance-aware branch,
// and a MOVE_X `cruise == 0` resolves through
// engineDefaultCruiseForDistance() rather than the flat
// `default_cruise` field above. engineWheelsX()'s wire path reads
// neither: WHEELS_X/WHEELS_V keep the flat sentinel unconditionally.
float engineADecel() {  // [mm/s^2]
  return ensure().engine.limits().decel;
}

float engineDefaultCruiseForDistance(float distance) {  // [mm] -> [mm/s]
  return ensure().engine.defaultCruiseForDistance(distance);
}

// SUC-003: MOVE_X's own D input for the resolver above -- a pure pivot
// (distance == 0) still has a real wheel-travel distance, so onMoveX()
// (wire_adapter.cpp) reaches this instead of taking |distance| alone.
// Forwards onto MotionEngine::dominantAxisTravel(), the same `dominant`
// quantity beginSegment() itself reduces to.
float engineDominantAxisTravel(float distance, float rotation) {  // [mm] [rad] -> [mm]
  return ensure().engine.dominantAxisTravel(distance, rotation);
}

// True iff MotionEngine's move-engine state (MOVE_X/GO_TO_R/GO_TO_W's
// own tracked segment) is currently active -- one of the two reads
// WireAdapter's motion-completion resolution needs. Mirrors the `//%`
// block API's moving() but is deliberately its OWN function: a
// wire-shaped bridge must not depend on a block-API function's
// continued existence or shape, the same rationale setWheelsTimed()
// gives for staying separate from setWheels(). `rig == nullptr` (a wire
// session with no prior motion verb) answers false.
bool engineMoveActive() {
  return rig != nullptr && rig->engine.isMoveActive();
}

// The second such read -- true iff the most recent Segment to go
// inactive ended via its OWN deadline rather than by reaching its goal,
// an abort, or an external stop. Latched once on the engine's own tick
// (see MotionEngine::lastSegmentEndedByDeadline()) instead of being
// re-derived from a wire-side clock whenever a host happens to ask.
// `rig == nullptr` answers false.
bool engineMoveEndedByDeadline() {
  return rig != nullptr && rig->engine.lastSegmentEndedByDeadline();
}

// ---- move engine ----------------------------------------------------

//%
void startMove(int distance, int yaw, int speed, int yawRate) {
  // [mm] [cdeg] [mm/s] [cdeg/s]
  // Refused (a silent no-op), not superseding, while a wire motion or a
  // GENUINE block/job COLLISION already holds the drivetrain -- a
  // dispatched RUN job's own call (e.g. test.ts's straightRun() ->
  // tickedMove() -> this function, running synchronously inside
  // dispatchJob()) proceeds instead: see
  // protocolTryTakeMotionOwnership()'s own comment above.
  if (!protocolTryTakeMotionOwnership()) return;
  Rig& r = ensure();
  odomUpdate(r);
  const float requestedDistance = static_cast<float>(distance);  // [mm]
  const float rotation = static_cast<float>(yaw) * kCdegToRad;  // [rad]

  // This shim predates MotionEngine::moveX()'s single-`cruise` wire-
  // shaped signature (motion-api.md S2: move_x(distance,rot) ==
  // wheels_x(distance-rot*b/2, distance+rot*b/2)) -- `blocks/motion.ts`'s
  // block API still passes two INDEPENDENT rate ceilings (speed for the distance
  // axis, yawRate for the yaw axis), picking whichever axis takes
  // LONGER at its own ceiling as the move's shared duration. Reconciled
  // via MotionEngine::reconcileDualRateCruise() (motion_engine.h), not
  // by favoring one of the two legacy rates: it derives the single
  // cruise that reproduces the EXACT SAME commanded velocity/twist this
  // dual-rate math has always produced, so move()/whileMoving()'s
  // observable behavior is unchanged. The floor-to-nonzero-before-
  // converting-units step below (avoiding a divide-by-zero, not a
  // meaningful floor) stays here rather than moving into that shared
  // method, because it operates on the RAW int-scale inputs, before
  // either is converted to mm/s or rad/s.
  const float speedFloored =
      static_cast<float>(speed > 0 ? speed : 1);          // [mm/s]
  const float yawRateFloored =
      static_cast<float>(yawRate > 0 ? yawRate : 1) * kCdegToRad;  // [rad/s]

  const MotionEngine::DualRateReconciliation rr =
      r.engine.reconcileDualRateCruise(requestedDistance, rotation, speedFloored,
                                       yawRateFloored);
  if (rr.cruise <= 0.0f) {
    // Nothing to do -- release right away rather than leaving kBlock
    // held with no move in flight and no tick loop coming to notice.
    protocolReleaseBlockOwnership();
    return;
  }

  // moveX() is ONE blended segment for any (distance, rotation): both
  // axes finish together, so the budget is the longer axis's duration,
  // the same reconciliation that derived `cruise` above.
  const float budgetDuration =
      rr.distDuration > rr.yawDuration ? rr.distDuration : rr.yawDuration;

  // Backstop: covers the end-of-move taper (service()) -- the last
  // ~15 deg / ~40 mm run at reduced rate, adding up to ~1 s. This is
  // moveX()'s own `timeout` -- a REAL backstop the wire's own MOVE_X
  // carries as a required field, not an internally re-derived one.
  const uint32_t timeout =
      static_cast<uint32_t>(budgetDuration * 1000.0f) + 1500u;

  r.engine.moveX(requestedDistance, rotation, rr.cruise, timeout);
}

//%
bool updateMove() {
  if (rig == nullptr) return false;
  Rig& r = *rig;
  // odomUpdate() only while a move was actually active -- pose stays
  // lazily updated (poseX()/Y()/heading() on demand) otherwise.
  const bool wasActive = r.engine.isMoveActive();
  if (wasActive) odomUpdate(r);
  const bool moveActive = r.engine.service();
  // Cross-fiber stop delivery (sprint 006 ticket 002, BLK-01(b)): this
  // poller's own call path -- isMoving() (moveProgress() is read-only;
  // see verify-blocks.md's BLK-12 spot check, which confirmed
  // isMoving()'s "checks state only" doc is false but REFUTED that same
  // claim for moveProgress()) -- can end a move at its deadline backstop
  // without tickDrive() ever running. Mirrors tickDrive()'s own
  // wasActive && !moveActive gate, but delivers the port write HERE
  // instead of relying on a settle-loop re-step this call path never
  // runs. Sprint 033 ticket 004: routed through Rig::softStop() with
  // the other three stop paths -- this branch used to call the
  // port-write half alone. See that method's own comment above for why
  // the two calls it gains are inert on this particular path.
  if (wasActive && !moveActive) r.softStop();
  return moveActive;
}

// Forward declaration: commandLooksActive() is defined further down, in
// its own clearly delineated section right before the starvation
// watchdog (it was written there first, for the watchdog's own use) --
// tickDrive() below needs it too now, for its return value (sprint 007
// ticket 002, closes R-10/API-01: see that function's own comment for
// what it checks, and tickDrive()'s own comment below for why).
static bool commandLooksActive(const Rig& r);

// Forward declaration: otosRef() (the OTOS lazy singleton) is defined
// further down, alongside the other OTOS shim entry points -- see its
// own comment there. tickDrive() below needs it too now, to perform
// the deferred pendingOtosZero write after busGuard.release() -- see
// that section of tickDrive()'s own comment.
static OtosPort& otosRef();

// ---- tick engine --------------------------------------------------------
// tickDrive(): the caller-driven replacement for the kernel's own
// now-unwired fiber (see ensure()'s comment). Runs exactly one
// kernel.step() + service() on the CALLER's fiber -- always, even with
// nothing active, or continuous-mode driving never progresses -- then
// self-paces to the next absolute 24 ms deadline, the same
// absolute-deadline pacing DifferentialDrive::run() uses. Consecutive
// calls anchor to the previous deadline so no drift accumulates; a gap
// re-anchors to now rather than catching up a burst of overdue ticks.
//
// Returns commandLooksActive(r) -- a move still in flight OR nonzero
// applied duty -- computed AFTER service(). Raw moveActive is the wrong
// read here: wheelsV()/wheelsX() clear the move planner before
// tickDrive() is ever called, so a continuous-mode `while (tickDrive())`
// loop reading it exits on its first iteration and the robot stops on
// the starvation watchdog ~150 ms later. A position-mode move's final
// tick still returns false, because the settle loop below drives
// applied duty to zero before this function returns.
//%
bool tickDrive() {
  Rig& r = ensure();
  const uint64_t cycleStart = r.clock.nowMicros();  // [us]
  r.lastTick = cycleStart;  // the watchdog's only freshness signal

  // Concurrency guard: check-and-set with no intervening yield is
  // atomic on CODAL's cooperative fibers, so this is safe against a
  // second fiber also calling tickDrive() -- it just waits (a short
  // timed poll, since the busy fiber may itself be parked in step()'s
  // settle sleeps) until the flag clears rather than racing
  // kernel.step(). This is now the SAME BusGuard every OTOS shim
  // entry point acquires, not a private stepBusy flag -- see
  // Rig::busGuard's own comment.
  r.busGuard.acquire(r.sleeper);
  r.kernel.step();

  // isDriving() (seg_.active || hold_.active), NOT isMoveActive()
  // (seg_.active alone): this feeds the settle-loop gate below, and a
  // continuous WHEELS_V/MOVE_V Hold reaching its own deadline inside
  // service() stages kernel_.neutral() exactly as a Segment's arrival
  // does while never setting seg_.active. Under the narrower read the
  // settle loop never ran for a Hold's natural end; the staged neutral
  // waited on a further kernel.step() that never came once the wire
  // lease elapsed and protocol.cpp stopped ticking, so Output froze at
  // its last mid-drive reading and STATUS's `active` bit read stuck
  // "still moving" forever. Pinned by
  // tests/host/test_wire_motion_verbs.py.
  const bool wasActive = r.engine.isDriving();
  // UNCONDITIONAL, every tick (R-09/BLK-05): odomUpdate() diffs against
  // the last kernel Output it consumed and re-stamps it, so it is a
  // no-op on a tick with no new encoder movement. Gated on `wasActive`
  // it never ran at all for continuous-mode driving
  // (setWheels()/driveTwist() with no move-engine move active), and the
  // next pose read then integrated the ENTIRE driven interval as one
  // straight chord at one midpoint heading -- for a closed loop, an
  // error the size of the whole path. `wasActive` is still computed,
  // for the settle-loop gate below (a different concern: folding
  // post-move coast counts into pose). updateMove() keeps its own,
  // narrower odometry gate for the TS blocking-poll path.
  odomUpdate(r);
  const bool moveActive = r.engine.service();

  // Settle before reporting "done". service() ends the move by posting
  // kernel.neutral(), which reaches the MOTORS only on a LATER
  // kernel.step() -- and a `while (tickDrive())` caller exits the
  // moment this returns false. Without the extra stepping the wheels
  // coast at the last commanded duty until the starvation watchdog's
  // port-level stop ~100-150 ms later: +9-13 deg per turn, +15-22 mm
  // per leg (bench root-cause, 2026-08-20). One extra step is not
  // enough either -- its own encoder read can land mid-spin-down and
  // freeze Output, and every post-move DIAG, at a nonzero velocity
  // forever. settleToRest() (motion_engine.h, host-tested) keeps
  // stepping, bounded, until both wheels witness the stop. Reached for
  // a continuous Hold's natural deadline as well as a Segment's
  // arrival, since `wasActive` is isDriving().
  if (wasActive && !moveActive) {
    r.engine.settleToRest();
    odomUpdate(r);  // coast counts -> pose before the final TLM
  }

  // Staged cross-fiber stop delivery: some OTHER fiber called
  // Rig::softStop() while THIS fiber held the guard above, and could
  // not write the motor ports itself without racing this fiber's own
  // I2C traffic -- see Rig::pendingStop_'s and Rig::softStop()'s own
  // comments. Deliver it now, still inside the guarded window this
  // fiber already owns, so no other fiber can interleave its own I2C
  // traffic between this write and release() below. This lands within
  // the SAME tick the request was staged in, the same guarantee an
  // unstaged softStop() has always given. The port write is spelled out
  // here rather than calling softStop() again: this is the DELIVERY of
  // an already-decided stop, and re-entering softStop() would re-run
  // its endMove()/neutral() and re-take the held() branch it is the
  // consumer of.
  if (r.pendingStop_) {
    r.pendingStop_ = false;
    r.left.emergencyStop();
    r.right.emergencyStop();
  }
  r.busGuard.release();

  // Deferred OTOS zero: SET rebase
  // (cfgSetRebase()) only ARMS pendingOtosZero -- the actual
  // I2C write happens HERE, on whichever fiber is ticking, exactly like
  // kernel.rebasePosition()'s own deferred-request shape. Consumed
  // AFTER busGuard.release() (so this tick's own kernel.step() is not
  // held up by an extra I2C round trip) but the write itself still
  // acquires/releases the SAME guard around its own body -- with no
  // yield between this release() and that reacquire(), no other fiber
  // can interleave here (see BusGuard's own comment), so this is safe
  // even though the guard is briefly unheld in between.
  if (r.pendingOtosZero) {
    r.pendingOtosZero = false;
    r.busGuard.acquire(r.sleeper);
    otosRef().setPose(0.0f, 0.0f, 0.0f);
    r.busGuard.release();
  }

  // Service hook: fires exactly here on EVERY call -- after this tick's
  // own kernel.step()/settle work and the deferred OTOS zero above are
  // both done (busGuard released again) and before the pacing sleep
  // below -- and NEVER inside a busGuard-held window: step() already
  // yields twice in there for its own encoder select-to-read settle,
  // and landing arbitrary wire/radio/dispatch work in that window would
  // break bus discipline. Null whenever nothing has registered one (a
  // host test, or before the protocol fiber starts); see
  // Rig::serviceHook's own comment for what the registered callback
  // actually does and why it is itself a no-op for most callers of this
  // function.
  if (r.serviceHook) r.serviceHook();

  // Absolute-deadline self-pacing, lifted from DifferentialDrive::run()
  // (diffdrive.cpp:290-306): read the cadence from the kernel's own
  // config (still 24 ms per sprint.md's Design Rationale) rather than
  // duplicating the constant here.
  const uint64_t period =  // [us]
      static_cast<uint64_t>(r.kernel.config().cyclePeriod) * 1000ull;
  const bool consecutive =
      r.tickDeadline != 0 && cycleStart < r.tickDeadline + period;
  const uint64_t deadline =  // [us]
      consecutive ? r.tickDeadline + period : cycleStart + period;
  r.tickDeadline = deadline;

  const uint64_t now = r.clock.nowMicros();  // [us]
  if (now < deadline) {
    const uint32_t shortfall =  // [ms]
        static_cast<uint32_t>((deadline - now + 999) / 1000);
    r.sleeper.sleepMillis(shortfall);
  } else {
    r.sleeper.yield();
  }

  const bool active = commandLooksActive(r);
  // Releases kBlock ownership the first tick this drivetrain looks
  // idle -- a no-op unless a block-motion entry point actually holds
  // it (protocolReleaseBlockOwnership()'s own comment above), mirroring
  // how a wire obligation's own owner value drops back to kNone the
  // first pass it clears (run(), protocol.cpp).
  if (!active) protocolReleaseBlockOwnership();
  return active;
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

static constexpr uint32_t kWatchdogPeriod = 50;          // [ms]
static constexpr uint64_t kWatchdogTimeout = 100000ull;  // [us] ~4 periods

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
  // this ticket: isDriving() (seg_.active || hold_.active),
  // not isMoveActive() (seg_.active alone). wheelsV()/wheelsX() no
  // longer call kernel_.drive() synchronously -- service()'s lazy start
  // (design S6.5) means a freshly-armed continuous hold shows zero
  // applied duty for one extra tick (the command lands on the NEXT
  // step(), after service() stages it), so isMoveActive()'s old
  // Segment-only reading would let this fall through to the applied-
  // duty check below and read false for that one tick -- exactly the
  // starvation this function exists to prevent. isDriving() covers the
  // hold immediately, synchronously, the moment wheelsV() arms it.
  if (r.engine.isDriving()) return true;
  const DiffDrive::DifferentialDrive::Output out = r.kernel.output();
  return out.appliedDutyLeft != 0.0f || out.appliedDutyRight != 0.0f;
}

static void watchdogEntry(void* context) {
  Rig& r = *static_cast<Rig*>(context);
  while (true) {
    r.sleeper.sleepMillis(kWatchdogPeriod);
    const uint64_t now = r.clock.nowMicros();  // [us]
    const uint64_t sinceLastTick = now - r.lastTick;  // [us]
    if (sinceLastTick <= kWatchdogTimeout) continue;
    if (!commandLooksActive(r)) continue;
    // The same soft stop every other stop path takes: the move
    // engine's in-flight state cleared, the kernel commanded neutral
    // for whenever step() next runs, and a tick-independent port-level
    // zero write -- staged instead of immediate if busGuard is
    // currently held, so this fiber cannot land its own I2C traffic
    // inside another fiber's settle window. This watchdog used to
    // write all three out itself (and, before sprint 030, to write the
    // ports directly and unconditionally); see Rig::softStop()'s own
    // comment above for the full reasoning. Ordering note: it spelled
    // the first two in the opposite order, which reaches the same end
    // state -- endMove() stages its own neutral() when a move was
    // live, and the unconditional neutral() covers the case where none
    // was.
    r.softStop();
    // An abandoned block-motion call (started, never ticked) would
    // otherwise hold kBlock forever with nothing left to notice it is
    // idle -- this is the one background fiber that still can.
    protocolReleaseBlockOwnership();
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
  // (stakeholder decision, 2026-08-26) -- which is exactly what
  // Rig::softStop() is, and why this is one call rather than the three
  // it used to spell out. The unconditional kernel.neutral() inside it
  // is the part that makes this a full stop: engine.endMove() alone
  // stages a neutral only when a move-engine move (startMove/
  // startGoTo) is active, so after a continuous-drive command
  // (setWheelSpeeds/driveTwist) nothing would disarm the kernel's held
  // commanded velocity mode (up to kLeaseMax, one hour) and the
  // port-level zero would be momentary -- the very next step() would
  // re-command the duty.
  rig->softStop();
  // The block program itself says this move is over -- release right
  // away rather than waiting for tickDrive() to next notice the
  // drivetrain looks idle. A no-op if this call was never the one
  // holding kBlock in the first place (protocolReleaseBlockOwnership()'s
  // own comment above).
  protocolReleaseBlockOwnership();
}

// ---- stopping -------------------------------------------------------

//%
void stopAll() {
  Rig& r = ensure();
  // The "stop" block and the wire's STOP verb both land here, and take
  // the same one soft stop every other stop path takes -- see
  // Rig::softStop()'s own comment above (including its cross-fiber
  // delivery, sprint 006 ticket 002).
  r.softStop();
  // No-op unless THIS call is the one holding kBlock -- see
  // protocolReleaseBlockOwnership()'s own comment above (a wire-issued
  // STOP reaching this same function never holds kBlock in the first
  // place, so this is harmless for that caller too).
  protocolReleaseBlockOwnership();
}

//%
void estopAll() {
  Rig& r = ensure();
  r.engine.endMove();
  r.kernel.estop();
  r.kernel.emergencyStopMotors();
  // See stopAll()'s identical call just above.
  protocolReleaseBlockOwnership();
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
// separate fault classes (same principle Rig::softStop() above
// established for a different pair -- a stop must never silently
// become a latch, and clearing one latch must never silently clear the
// other). clearStall() is reachable from a dedicated `blocks/stop.ts` block
// AND the wire's `stall_clear` SET-action ConfigField
// (cfgSetStallClear(), below); isStalled() backs the matching
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
// convention protocolEmitLine/protocolCurrentRunText already use further
// down this file.
int protocolSerialDropCount();
int protocolRunDropCount();
int protocolEmitDropCount();
int protocolRunMalformedCount();
int protocolRadioRxFrameCount();
int protocolRadioRxAcceptedCount();
int protocolRadioRxOverrunDropCount();
int protocolRadioRxOversizeDropCount();

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
    // 26: SerialTransport::writeLine() drops -- the two-writer guard's
    // retry cap exhausted, or uBit.serial.send() itself failed.
    // Nonzero means lines were lost off the serial link.
    case 26: return protocolSerialDropCount();
    // 27: both wheels' encoder rebaselines -- an implausible-then-
    // consistent jump treated as a counter restart (a brick MCU reset)
    // instead of integrated as a multi-metre teleport. Nonzero means
    // an encoder counter restarted mid-session.
    case 27:
      return static_cast<int>(ensure().left.rebaselineCount_ +
                              ensure().right.rebaselineCount_);
    // 28: cleartext RUN payloads refused because every ring slot was
    // still in flight. Nonzero means a host out-ran the robot.
    case 28: return protocolRunDropCount();
    // 29: emitLine() calls refused because the outbound emit ring
    // (comms/emit_queue.h) was full. Nonzero means a caller queued
    // lines faster than the protocol fiber drained them onto the wire.
    case 29: return protocolEmitDropCount();
    // 30: cleartext RUN payloads refused by the bridge's SANITIZER --
    // empty, >= 48 bytes, non-printable, or an empty name. Distinct
    // from 28, which counts capacity refusals only; nonzero means a
    // host is sending malformed lines (which look exactly like radio
    // loss from the relay).
    case 30: return protocolRunMalformedCount();
    // 31-34: the radio RX path -- what arrived, what got through, and
    // the two ways the rest did not. 31 complete single-fragment lines
    // received; 32 delivered into the RX slot; 33 dropped because the
    // previous line was not drained yet; 34 dropped whole (never
    // truncated) for exceeding the 240-byte RX buffer.
    // 31 - 32 == 33 + 34 on any healthy build.
    case 31: return protocolRadioRxFrameCount();
    case 32: return protocolRadioRxAcceptedCount();
    case 33: return protocolRadioRxOverrunDropCount();
    case 34: return protocolRadioRxOversizeDropCount();
    default: return 0;
  }
}

// ---- pose -----------------------------------------------------------

//%
int poseX() {  // [mm]
  Rig& r = ensure();
  odomUpdate(r);  // a pose read ADVANCES odometry -- see odometry.h
  return static_cast<int>(std::lround(r.odometry.x()));
}

//%
int poseY() {  // [mm]
  Rig& r = ensure();
  odomUpdate(r);
  return static_cast<int>(std::lround(r.odometry.y()));
}

//%
int poseHeading() {  // [cdeg]
  Rig& r = ensure();
  odomUpdate(r);
  return static_cast<int>(std::lround(r.odometry.heading() * kRadToCdeg));
}

//%
void resetPose() {
  Rig& r = ensure();
  odomUpdate(r);  // consume any pending deltas first
  r.odometry.reset();
}

// ---- configuration --------------------------------------------------

//%
void setGeometry(int trackWidth, int calib) {  // [0.1 mm] [1e-4 mm/deg]
  Rig& r = ensure();
  if (trackWidth > 0)
    r.engine.setTrackWidth(static_cast<float>(trackWidth) * 0.1f);
  if (calib > 0) r.engine.setTravelCalib(static_cast<float>(calib) * 1e-4f);
}

// ---- shaping-field descriptor table ---------------------------------
// {ordinal, setter, field} rows for the ten config ordinals that map
// onto a MotionLimits member. setKernelValue()/getConfigValue() (below)
// consult this table first and kConfigAccessors (below it) second;
// between the two there is no per-field switch left in this file.
// `setter` is one of MotionLimits' own "positive, else keep" validated
// setters (motion_limits.h), called through a pointer-to-member-
// function exactly as `field` is read through a pointer-to-data-member,
// so this table adds no validation of its own -- it only ROUTES.
namespace {
struct LimitsFieldEntry {
  int ordinal;
  void (MotionLimits::*setter)(float);
  float MotionLimits::*field;
};

// Row order follows design motion-profile-unification.md S4.7's own
// wire-name table, NOT ordinal order, so the two read side by side
// without re-sorting either.
constexpr LimitsFieldEntry kLimitsFields[] = {
    {19, &MotionLimits::setAccel, &MotionLimits::accel},
    {20, &MotionLimits::setDecel, &MotionLimits::decel},
    {21, &MotionLimits::setVMax, &MotionLimits::vMax},
    {28, &MotionLimits::setJerk, &MotionLimits::jerk},
    {30, &MotionLimits::setOmegaMax, &MotionLimits::omegaMax},
    // 8: v_floor, the ordinal the kernel's old speed_floor used -- the
    // setter writes HERE now, and the kernel's own vMin stays pinned at
    // 0 (see ensure()'s Config seed above).
    {8, &MotionLimits::setVFloor, &MotionLimits::vFloor},
    {34, &MotionLimits::setOmegaFloor, &MotionLimits::omegaFloor},
    // 18: stop_distance, on the ordinal the old pivot_overrun used;
    // config_fields.h's row carries the rename's provenance.
    {18, &MotionLimits::setStopDistance, &MotionLimits::stopDistance},
    {35, &MotionLimits::setArriveDist, &MotionLimits::arriveDist},
    {36, &MotionLimits::setArriveYaw, &MotionLimits::arriveYaw},
    // 37: lag -- the drivetrain's first-order response lag [s]; see
    // motion_limits.h's own field comment.
    {37, &MotionLimits::setLag, &MotionLimits::lag},
};
constexpr size_t kLimitsFieldCount =
    sizeof(kLimitsFields) / sizeof(kLimitsFields[0]);

const LimitsFieldEntry* findLimitsField(int ordinal) {
  for (const auto& entry : kLimitsFields) {
    if (entry.ordinal == ordinal) return &entry;
  }
  return nullptr;
}
}  // namespace

// ---- config accessor table ------------------------------------------
// What each wire config ordinal DOES on GET and on SET. The ten shaping
// ordinals are answered by kLimitsFields above; every other one is a row
// here.
//
// The wire NAMES these ordinals answer to live once, in
// comms/config_fields.h, which wire_adapter.cpp reads to turn a
// `SET <name>`/`GET <name>` into an ordinal. The surface is split by
// portability, not preference: that header is host-portable and this
// file is not (it includes pxt.h and reaches into Rig, the kernel and
// the motion engine). Names and ordinals there, behaviour here, bound
// by the ordinal and checked by
// tests/host/test_config_surface_single_source.py, which fails if
// either side names an ordinal the other does not. Adding a field means
// one row in each, and nothing else.
//
// Values are UNSCALED in these accessors; setKernelValue()/
// getConfigValue() below own the wire's x1000 integer convention on the
// way in and on the way out.
//
// Named functions rather than lambdas written inline in the table: a
// non-capturing lambda's conversion to a function pointer became a
// constant expression only in C++17, and both embedded targets compile
// at C++11 -- an inline-lambda table would be built into RAM by a
// startup constructor instead of sitting in flash.
namespace {

float cfgGetMaxDuty(Rig& r) { return r.kernel.config().maxDuty; }
void cfgSetMaxDuty(Rig& r, float v) { r.kernel.setMaxDuty(v); }

float cfgGetFullDutyVelocity(Rig& r) {
  return r.kernel.config().fullDutyVelocity;
}
void cfgSetFullDutyVelocity(Rig& r, float v) {
  r.kernel.setFullDutyVelocity(v);
}

float cfgGetKp(Rig& r) { return r.kernel.config().kp; }
void cfgSetKp(Rig& r, float v) { r.kernel.setKp(v); }

float cfgGetKi(Rig& r) { return r.kernel.config().ki; }
void cfgSetKi(Rig& r, float v) { r.kernel.setKi(v); }

float cfgGetIMax(Rig& r) { return r.kernel.config().iMax; }
void cfgSetIMax(Rig& r, float v) { r.kernel.setIMax(v); }

float cfgGetKaff(Rig& r) { return r.kernel.config().kaff; }
void cfgSetKaff(Rig& r, float v) { r.kernel.setKaff(v); }

float cfgGetPidMax(Rig& r) { return r.kernel.config().pidMax; }
void cfgSetPidMax(Rig& r, float v) { r.kernel.setPidMax(v); }

float cfgGetTwistHoldGain(Rig& r) {
  return r.kernel.config().twistHoldGain;
}
void cfgSetTwistHoldGain(Rig& r, float v) { r.kernel.setTwistHoldGain(v); }

float cfgGetPosErrMax(Rig& r) { return r.kernel.config().posErrMax; }
void cfgSetPosErrMax(Rig& r, float v) { r.kernel.setPositionErrorMax(v); }

// The three stall parameters share one kernel setter, so each writes its
// own value alongside the OTHER two read back unchanged -- the same
// read-modify-write these three ordinals have always done.
float cfgGetStallSpeed(Rig& r) { return r.kernel.config().stallSpeed; }
void cfgSetStallSpeed(Rig& r, float v) {
  const DiffDrive::DifferentialDrive::Config c = r.kernel.config();
  r.kernel.setStall(v, c.stallDemand, c.stallWindow);
}

float cfgGetStallDemand(Rig& r) { return r.kernel.config().stallDemand; }
void cfgSetStallDemand(Rig& r, float v) {
  const DiffDrive::DifferentialDrive::Config c = r.kernel.config();
  r.kernel.setStall(c.stallSpeed, v, c.stallWindow);
}

float cfgGetStallWindow(Rig& r) { return r.kernel.config().stallWindow; }
void cfgSetStallWindow(Rig& r, float v) {
  const DiffDrive::DifferentialDrive::Config c = r.kernel.config();
  r.kernel.setStall(c.stallSpeed, c.stallDemand, v);
}

float cfgGetLambdaEnabled(Rig& r) {
  return r.kernel.config().lambdaEnabled ? 1.0f : 0.0f;
}
void cfgSetLambdaEnabled(Rig& r, float v) {
  r.kernel.setLambdaEnabled(v != 0.0f);
}

float cfgGetCrawlPulse(Rig& r) { return r.kernel.config().crawlPulse; }
void cfgSetCrawlPulse(Rig& r, float v) { r.kernel.setCrawlPulse(v); }

// default_cruise is the wire layer's OWN configured-default cruise
// (Rig::defaultCruise_, see engineDefaultCruise() above), not a kernel
// Config field. Same ">0, else keep" silent-ignore validation
// setGeometry() uses: `SET default_cruise 0` is accepted but does not
// clear the field, so there is deliberately no wire-level way to force
// "no default available".
float cfgGetDefaultCruise(Rig& r) { return r.defaultCruise_; }
void cfgSetDefaultCruise(Rig& r, float v) {
  if (v > 0.0f) r.defaultCruise_ = v;
}

// rotational_slip lives on MotionEngine, which owns its own ">0, else
// keep the prior value" validation -- no duplicate check here.
float cfgGetRotationalSlip(Rig& r) { return r.engine.rotationalSlip(); }
void cfgSetRotationalSlip(Rig& r, float v) { r.engine.setRotationalSlip(v); }

// stall_clear/estop_clear/rebase are write-triggered ACTIONS wearing a
// config field's clothes: only nonzero-vs-zero matters (a wire `SET
// stall_clear 1` arrives as 1000 and reaches here as 1.0f), and the
// magnitude is ignored. Their GET sides read the live latch back where
// there is one to read.
//
// clearStallLatch() and estopClear() are deliberately separate: the
// stall latch and the e-stop latch are distinct fault classes, and
// clearing one must never silently clear the other (the same principle
// Rig::softStop() follows for a different pair).
float cfgGetStallClear(Rig& r) {
  return r.kernel.output().stallHalted ? 1.0f : 0.0f;
}
void cfgSetStallClear(Rig& r, float v) {
  if (v != 0.0f) r.kernel.clearStallLatch();
}

// rebase zeroes the odometry frame, writing BOTH pose sources
// (seedPose()'s own "write both" contract): the encoder-integrated pose
// this file tracks AND the OTOS position register, so the two stay
// agreed at the new zero instead of OTOS keeping its old absolute
// reading. Only Odometry::reset() happens synchronously -- the other
// two are DEFERRED requests. kernel.rebasePosition() re-anchors at the
// NEXT step(), and Odometry's position-epoch guard keeps that
// legitimate discontinuity from reading as a spurious jump; the OTOS
// zero arms pendingOtosZero for tickDrive() to service after
// busGuard.release(), so the write lands on a fiber that owns the bus.
//
// GET is refused upstream (wire_adapter.cpp's onGet()): a rebase has no
// stored value and no latch, so any answer would be a manufactured 0.
float cfgGetRebase(Rig&) { return 0.0f; }
void cfgSetRebase(Rig& r, float v) {
  if (v == 0.0f) return;
  odomUpdate(r);  // consume pending deltas before the zero
  r.kernel.rebasePosition();
  r.odometry.reset();
  r.pendingOtosZero = true;
}

float cfgGetEstopClear(Rig& r) {
  return r.kernel.output().estopped ? 1.0f : 0.0f;
}
void cfgSetEstopClear(Rig& r, float v) {
  if (v != 0.0f) r.kernel.estopClear();
}

// straight_trim IS a stored kernel Config field, unlike default_cruise/
// rotational_slip above. Sign and magnitude are both meaningful; the
// kernel setter owns the finiteness check.
float cfgGetStraightTrim(Rig& r) { return r.kernel.config().straightTrim; }
void cfgSetStraightTrim(Rig& r, float v) { r.kernel.setStraightTrim(v); }

// goto_timeout: the deadline the next go-to gets (Rig::goToDeadline),
// backed by a Rig field rather than the kernel's Config, like
// default_cruise above. 0 is LEGAL here, unlike default_cruise's
// ">0, else keep": engineGoToRArmed() passes this straight to
// MotionEngine::goToR() as `timeout`, where 0 means "already expired",
// and refusing to store it would leave this field unable to read back a
// state the block layer can put the robot in. Negative is refused (the
// cast below is undefined for it, and a deadline in the past has no
// wire meaning); the upper end needs no guard, since the wire's x1000
// convention caps an arriving value near 2.1e6 ms.
float cfgGetGoToDeadline(Rig& r) {
  return static_cast<float>(r.goToDeadline);
}
void cfgSetGoToDeadline(Rig& r, float v) {
  if (v < 0.0f) return;
  r.goToDeadline = static_cast<uint32_t>(v);
}

struct ConfigAccessor {
  int ordinal;
  float (*get)(Rig&);          // [unscaled]
  void (*set)(Rig&, float);    // [unscaled]
};

constexpr ConfigAccessor kConfigAccessors[] = {
    {0, &cfgGetMaxDuty, &cfgSetMaxDuty},
    {1, &cfgGetFullDutyVelocity, &cfgSetFullDutyVelocity},
    {2, &cfgGetKp, &cfgSetKp},
    {3, &cfgGetKi, &cfgSetKi},
    {4, &cfgGetIMax, &cfgSetIMax},
    {5, &cfgGetKaff, &cfgSetKaff},
    {6, &cfgGetPidMax, &cfgSetPidMax},
    {7, &cfgGetTwistHoldGain, &cfgSetTwistHoldGain},
    {9, &cfgGetPosErrMax, &cfgSetPosErrMax},
    {10, &cfgGetStallSpeed, &cfgSetStallSpeed},
    {11, &cfgGetStallDemand, &cfgSetStallDemand},
    {12, &cfgGetStallWindow, &cfgSetStallWindow},
    {13, &cfgGetLambdaEnabled, &cfgSetLambdaEnabled},
    {14, &cfgGetCrawlPulse, &cfgSetCrawlPulse},
    {15, &cfgGetDefaultCruise, &cfgSetDefaultCruise},
    {16, &cfgGetRotationalSlip, &cfgSetRotationalSlip},
    {17, &cfgGetStallClear, &cfgSetStallClear},
    {32, &cfgGetRebase, &cfgSetRebase},
    {33, &cfgGetEstopClear, &cfgSetEstopClear},
    {38, &cfgGetStraightTrim, &cfgSetStraightTrim},
    {39, &cfgGetGoToDeadline, &cfgSetGoToDeadline},
};

const ConfigAccessor* findConfigAccessor(int ordinal) {
  for (const auto& entry : kConfigAccessors) {
    if (entry.ordinal == ordinal) return &entry;
  }
  return nullptr;
}

}  // namespace

//%
void setKernelValue(int field, int value) {  // [x1000 scaled]
  Rig& r = ensure();
  const float v = static_cast<float>(value) * 0.001f;
  if (const LimitsFieldEntry* shaping = findLimitsField(field)) {
    (r.engine.limits().*(shaping->setter))(v);
    return;
  }
  if (const ConfigAccessor* entry = findConfigAccessor(field)) entry->set(r, v);
  // An ordinal in neither table is silently ignored, the same way an
  // unrecognized field number always has been. No wire caller can get
  // here: wire_adapter.cpp only ever passes ordinals config_fields.h
  // names, and every one of those is in one of the two tables.
}

// ---- config read-back -------------------------------------------------
// The read-back counterpart to setKernelValue() above: same ordinals,
// same two tables, same x1000 scaling. Reads flow through each row's own
// get accessor, so a field's read and its write can no longer disagree
// about where the value lives. Deliberately NOT `//%`-annotated --
// C++-internal; wire_adapter.cpp is the only caller, via its own
// same-package forward declaration. An out-of-range field returns 0.
int getConfigValue(int field) {  // -> [x1000 scaled]
  Rig& r = ensure();
  float v = 0.0f;
  if (const LimitsFieldEntry* shaping = findLimitsField(field)) {
    v = r.engine.limits().*(shaping->field);
  } else if (const ConfigAccessor* entry = findConfigAccessor(field)) {
    v = entry->get(r);
  } else {
    return 0;
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
// engineDefaultCruise() above. `omega` arrives at engineMoveV()
// ALREADY converted from the wire's milliradian integer
// (wire_adapter.cpp's mradToRad()); `speed`'s <0/==0 "configured
// default" substitution is resolved by onGoToR()/onGoToW() BEFORE
// either of these is ever called, via engineDefaultCruise() above.
// Placed after otosRef(), not with engineWheelsX()/engineMoveX() above,
// because engineGoToW() below needs it.
void engineMoveV(float vx, float omega,
                 uint32_t duration) {  // [mm/s] [rad/s] [ms]
  Rig& r = ensure();
  r.engine.moveV(vx, omega, duration);
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
void engineGoToR(float x, float y, float speed, float arrive,
                 uint32_t timeout) {  // [ms]
  Rig& r = ensure();
  r.engine.goToR(x, y, speed, arrive, timeout);
}

// `//%`-annotated -- pre-arms the NEXT engineGoToRArmed() call's
// deadline, landing it in Rig::goToDeadline. PXT rejects a `//%` shim
// carrying more than four parameters: a real build of the original
// single five-parameter engineGoToR() shim reproduced "TS9200:
// Assertion failed" deterministically, surviving make_deploy.py's one
// automatic retry for the benign packaging abort that reports the same
// error code. Every `//%` shim in this file therefore stays at <=4
// params and this setter supplies the fifth argument. One caller
// (sim.ts's _setGoToDeadline(), called by motion.ts's startGoTo()
// immediately before _goToR()).
//%
void engineSetGoToDeadline(uint32_t timeout) {  // [ms]
  ensure().goToDeadline = timeout;
}

// `//%`-annotated -- same pre-arm shape as engineSetGoToDeadline()
// immediately above (see it for the <=4-param rule), added so
// engineGoToRArmed() below can reconcile a SEPARATE yaw-rate ceiling
// against `speed`, mirroring startMove()'s own (distance, yaw, speed,
// yawRate) shape. One caller only (sim.ts's _setGoToYawRate(), called
// by motion.ts's startGoTo() alongside _setGoToDeadline()).
//%
void engineSetGoToYawRate(int yawRate) {  // [cdeg/s]
  ensure().pendingGoToYawRate_ = static_cast<float>(yawRate);
}

// `//%`-annotated -- the block layer's own entry point onto the SAME
// goToR() the wire's GO_TO_R verb reaches via engineGoToR() above,
// just split to FOUR parameters (engineSetGoToDeadline() immediately
// above supplies the fifth, `timeout`, via Rig::goToDeadline) -- see
// that function's comment for why.
// Deliberately delegates to engineGoToR() above rather than calling
// r.engine.goToR() directly a second time, so the actual move-engine
// call site stays in exactly one place.
//
// MotionEngine::goToR() threads a SINGLE `speed`/cruise parameter
// through both its pivot and straight phases (motion_engine.cpp:
// queuePivotThenStraight() reuses the same `cruise` for both), exactly
// as moveX() does for startMove()'s own two axes -- so this reconciles
// `speed` against the pre-armed yaw-rate ceiling
// (Rig::pendingGoToYawRate_) the SAME way startMove() reconciles its
// own two rate ceilings, via the shared
// MotionEngine::reconcileDualRateCruise(). decomposeGoToR() reproduces
// goToR()'s own bearing-then-chord split decision first, so this
// reconciliation can never disagree with which phases goToR() will
// actually run: the pivot/straight pair (bearingRaw, chord) when it
// will split, or the single blended segment's own (theta, arcLength)
// pair when it will not (see motion_engine.h's own GoToRPlan comment).
//%
void engineGoToRArmed(float x, float y, float speed, float arrive) {
  // Refused (a silent no-op), not superseding, while a wire motion or a
  // GENUINE block/job COLLISION already holds the drivetrain -- a
  // dispatched RUN job's own call (e.g. test.ts's tickedGoTo(), which
  // "goto"/"face" run through) proceeds instead: see
  // protocolTryTakeMotionOwnership()'s own comment above (shims.cpp's
  // own top section). This is startGoTo()'s (blocks/motion.ts) own
  // entry point onto the move engine, the goTo() counterpart of
  // startMove() above.
  if (!protocolTryTakeMotionOwnership()) return;
  Rig& r = ensure();

  const MotionEngine::GoToRPlan plan = MotionEngine::decomposeGoToR(x, y);
  const float speedFloored = speed > 0.0f ? speed : 1.0f;  // [mm/s]
  const float yawRateFloored =
      (r.pendingGoToYawRate_ > 0.0f ? r.pendingGoToYawRate_ : 1.0f) *
      kCdegToRad;  // [rad/s]
  const float pivotRotation = plan.willSplit ? plan.bearingRaw : plan.theta;
  const float straightDistance =
      plan.willSplit ? plan.chord : plan.arcLength;

  const MotionEngine::DualRateReconciliation rr =
      r.engine.reconcileDualRateCruise(straightDistance, pivotRotation,
                                       speedFloored, yawRateFloored);
  // Nothing to reconcile (target essentially at the current pose) --
  // fall back to `speed` unchanged; goToR()'s own `arrive` gate below
  // is what actually decides this is a no-op, same as before this
  // reconciliation existed.
  const float cruise = rr.cruise > 0.0f ? rr.cruise : speedFloored;

  engineGoToR(x, y, cruise, arrive, r.goToDeadline);
}

// GO_TO_W's own PoseSource selection: the ONE place this project
// decides which PoseSource serves a GO_TO_W call, via
// selectPoseSource() (motion/odometry.h) -- this file's
// `gOtos`/otosRef() lazy singleton when `connected()` (initialized AND
// actually talking to the chip), the Rig-owned `odometry`
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
                uint32_t timeout) {  // [ms]
  OtosPort& otos = otosRef();
  Rig& r = ensure();
  PoseSource& pose = selectPoseSource(otos.connected(), otos, r.odometry);
  r.engine.goToW(pose, x, y, speed, arrive, timeout);
  return true;
}

// SUC-003: GO_TO_W's own D input for
// engineDefaultCruiseForDistance() above -- the TRUE body-frame
// chord from the robot's CURRENT pose to this call's WORLD-frame
// (worldX, worldY) target, not the target's distance from the world
// origin (hypot(worldX, worldY) alone, which is wrong whenever the
// robot is not sitting at the origin). Reuses the exact SAME
// PoseSource selection engineGoToW() above applies -- OtosPort when
// connected(), the Rig-owned odometry otherwise -- so this resolves
// against the pose the move will actually run from. Both PoseSource
// implementations' reads are plain accessors over already-cached state
// (otos_port.h / motion/odometry.h -- Odometry::x()/y() read the
// current frame and do not themselves integrate; only update() does,
// and this function does not call it) -- reading x()/y() here, a
// second time before engineGoToW() reads them again for the real
// dispatch, mutates nothing and cannot observe a different value than
// that dispatch will.
float engineGoToWChord(float worldX, float worldY) {
  OtosPort& otos = otosRef();
  Rig& r = ensure();
  PoseSource& pose = selectPoseSource(otos.connected(), otos, r.odometry);
  return std::hypot(worldX - pose.x(), worldY - pose.y());
}

// RETIRED (sprint 029 ticket 003/004, design motion-profile-
// unification.md S4.7/S8/S12): harmless no-op shims kept for one
// release so a MakeCode project saved before this sprint that still
// calls `setTaperWindows`/`setTaperFloors`/`setRampMs` compiles and
// runs -- it just does nothing now. These used to set end-of-move
// shaping fields (distTaper_/yawTaper_/distFloor_/turnFloor_/rampMs_)
// that ticket 003 DELETED from MotionEngine entirely -- the taper
// window, the floor fraction and the ramp time are all superseded by
// MotionLimits + VelocityShaper (design S4.1/S4.2): shaping is now
// `set config`'s own accel/decel/jerk/v_max/omega_max/v_floor/
// omega_floor fields (ConfigField, blocks/motion.ts), reachable only
// through limits(), which these three no-shim parameter shapes have no
// way to express (percent-of-cruise floors and counts-windows don't
// correspond to anything VelocityShaper reads). Neither of these three
// was ever a draggable BLOCK (no `//% block=` was ever declared for
// them -- only sim.ts/test.ts ever called them directly as plain
// TypeScript), so there is no toolbox entry to hide; "hidden" here
// means "does nothing", not "no longer visible in the palette". Design
// S12 open question 3 (may these be removed outright, or must they stay
// no-ops for a release) is undecided as of this ticket -- defaulting to
// the no-op posture per that question's own stated default.
//%
void setTaperWindows(int dist, int yaw) {
  (void)dist;
  (void)yaw;
}

//%
void setTaperFloors(int dist, int turn) {
  (void)dist;
  (void)turn;
}

//%
void setRampMs(int ms) {
  (void)ms;
}

// this ticket (design motion-profile-unification.md S4.7): the ONE
// shim `test.ts`'s two profile functions (openLoopProfile()/
// closedLoopProfile()) now call, replacing the three retired shims
// immediately above. Four `int` parameters, not five -- sprint 015
// ticket 006's own PXT packager finding (engineSetGoToDeadline()'s
// comment above) is why every `//%` shim in this file stays at <=4
// params; a MotionLimits-shaped call with five plain fields (accel,
// decel, vMax, omegaMax, plus a floor or arrival window) would cross
// that line, so this shim covers only the two profile-selected rate
// ceilings (design S4.7's own `setLimits({accel, decel, vMax,
// omegaMax})` pseudocode) -- floors and stop_distance stay per-robot,
// set once from the deploy bake (or `set config`), never per profile.
// No wire-scale (x1000) convention here, unlike setKernelValue() --
// plain student units straight through, matching startMove()'s own
// int-parameter shims.
//%
void setLimits(int accel, int decel, int vMax, int omegaMax) {  // [mm/s^2] [mm/s^2] [mm/s] [deg/s]
  Rig& r = ensure();
  MotionLimits& lim = r.engine.limits();
  lim.setAccel(static_cast<float>(accel));
  lim.setDecel(static_cast<float>(decel));
  lim.setVMax(static_cast<float>(vMax));
  lim.setOmegaMax(static_cast<float>(omegaMax));
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
  // Bus-ownership guard: begin() issues several
  // I2C writes and a polled read (otos_port.cpp) -- see BusGuard's own
  // comment for why every OTOS entry point now brackets its I2C body
  // this way. productId() below is a cached read, no I2C, safe outside
  // the guard.
  Rig& r = ensure();
  OtosPort& o = otosRef();
  r.busGuard.acquire(r.sleeper);
  o.begin();
  r.busGuard.release();
  return o.productId();
}

//%
bool otosRead() {
  Rig& r = ensure();
  r.busGuard.acquire(r.sleeper);
  const bool ok = otosRef().read();
  r.busGuard.release();
  return ok;
}

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
    case 8: {
      // Bus-ownership guard: imuCalibrationSamplesRemaining() issues a
      // live I2C read (otos_port.cpp readReg8), unlike every other case
      // above (cached fields set by the last read()/begin()) -- same
      // three-line acquire/I2C-call/release bracket as the six named
      // OTOS entry points above.
      Rig& r = ensure();
      r.busGuard.acquire(r.sleeper);
      const int remaining = o.imuCalibrationSamplesRemaining();
      r.busGuard.release();
      return remaining;
    }
    default: return 0;
  }
}

//%
void otosZero() {
  Rig& r = ensure();
  r.busGuard.acquire(r.sleeper);
  otosRef().zeroPose();
  r.busGuard.release();
}

//%
void otosCalibrate(int samples) {
  Rig& r = ensure();
  r.busGuard.acquire(r.sleeper);
  otosRef().calibrateImu(static_cast<uint8_t>(samples));
  r.busGuard.release();
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

// Same forward-declaration convention as protocolEmitLine above.
void protocolSetupWifi(const char* ssid, const char* password);

// Emptiness comes from the PXT String's OWN size, not from MSTR()'s
// toCharArray() result -- same trap registerRunName() documents below:
// at size 0, toCharArray() returns junk, not a clean "". Both locals
// stay in scope across protocolSetupWifi(), which copies before
// returning, so nothing here can outlive the call.
//%
void setupWifi(String ssid, String password) {
  if (ssid == nullptr) return;
  ManagedString ssidStr = ssid->getUTF8Size() == 0 ? ManagedString("") : MSTR(ssid);
  bool passwordEmpty = password == nullptr || password->getUTF8Size() == 0;
  ManagedString passwordStr = passwordEmpty ? ManagedString("") : MSTR(password);
  protocolSetupWifi(ssidStr.toCharArray(), passwordStr.toCharArray());
}

// Read back the text of whichever RUN command is CURRENTLY being
// dispatched -- valid only during the registered dispatch callback's own
// call, on this same fiber (registerRunDispatch()/runDispatch(), below).
// Same forward-declaration convention as protocolEmitLine above.
const char* protocolCurrentRunText();

//%
String runCommandText() {
  const char* text = protocolCurrentRunText();
  size_t len = 0;
  while (text[len] != '\0') ++len;
  return mkString(text, static_cast<int>(len));
}

// ---- RUN dispatch: the single TS callback protocol.cpp invokes -------
// run.ts's wireRunDispatch() registers ONE Action here, the first time
// any onRun()/onRunCommand() handler is bound -- the by-name lookup and
// dispatch logic inside that callback (matching the dequeued command's
// name against every registered handler) is unchanged from before;
// only what TRIGGERS the callback changes, from a MessageBus event
// fired at a second, forked fiber to a direct call from
// protocol.cpp's own dispatchJob()/handleRun() bypass, on the protocol
// fiber itself.
static Action gRunDispatchAction = nullptr;

//%
void registerRunDispatch(Action cb) {
  if (gRunDispatchAction) decr(gRunDispatchAction);
  gRunDispatchAction = cb;
  incr(gRunDispatchAction);
}

// ---- RUN registry: the names FUNCS discloses -------------------------
// run.ts's onRun() calls this as it binds each handler, so the C++ side
// can enumerate what this particular block program answers to
// (comms/run_registry.h). Registration ONLY -- dispatch still matches
// names in TypeScript, against TypeScript's own arrays, exactly as
// before; nothing here can make a name runnable or stop one being run.
//
// Both arguments cross as ManagedStrings and are copied into the
// registry's own fixed cells before this returns, so neither the
// ManagedString nor its buffer needs to outlive the call -- the
// borrowed-pointer contract Wire::Adapter's runName() keeps is
// satisfied by the TABLE's storage, not by the caller's.
//
// A null `signature` is normal, not an error: it is how a handler bound
// with no declared signature reaches here, and the registry stores it
// as the empty string, which makes execFuncs omit the field entirely.
//
// NOTE the layout below: PXT's shim scanner requires `//%` to sit
// IMMEDIATELY above the declaration it annotates. A comment between the
// two fails the build with "declaration not understood", naming the
// comment line rather than the real problem.
//%
void registerRunName(String name, String signature) {
  // Emptiness comes from the PXT String's OWN size: at size 0 the
  // ManagedString MSTR() builds has no clean "" in toCharArray().
  // MEASURED gopiv 2026-09-07, fw 1.20260907.1 -- every line read
  // `funcs <name> \xef\xbf\xbdc`, junk where an omitted signature goes.
  if (name == nullptr || name->getUTF8Size() == 0) return;
  ManagedString nameStr = MSTR(name);
  if (signature == nullptr || signature->getUTF8Size() == 0) {
    diffDrive::runRegistry().add(nameStr.toCharArray(), "");
    return;
  }
  ManagedString signatureStr = MSTR(signature);
  diffDrive::runRegistry().add(nameStr.toCharArray(),
                               signatureStr.toCharArray());
}

// Declares that a catch-all handler (run.ts's onRunCommand) is bound, so
// EVERY name is dispatchable and WireAdapter::onRun must stop refusing
// names the registry does not list. One-way: nothing unsets it, because
// a handler cannot be unbound.
//%
void registerRunCatchAll() { diffDrive::runRegistry().acceptAnyName(); }

// Invokes the registered callback, if one exists, on the CALLER's own
// fiber -- protocol.cpp reaches this via the same same-package
// forward-declaration convention as tickDrive(). Returns false with
// nothing registered yet (no onRun()/onRunCommand() call has ever run),
// a silent no-op, matching the old event path's own behavior when
// nothing had subscribed.
bool runDispatch() {
  if (!gRunDispatchAction) return false;
  runAction0(gRunDispatchAction);
  return true;
}

// Registers tickDrive()'s own service hook (see Rig::serviceHook's and
// tickDrive()'s own comments above) -- a plain C++-to-C++ seam, never
// called from TS, so deliberately NOT `//%`-annotated.
void registerTickServiceHook(void (*hook)()) { ensure().serviceHook = hook; }

//%
void otosSetOffset(int x, int y, int yaw) {  // [0.1 mm] [0.1 mm] [cdeg]
  Rig& r = ensure();
  r.busGuard.acquire(r.sleeper);
  otosRef().setOffset(static_cast<float>(x) * 0.1f,
                      static_cast<float>(y) * 0.1f,
                      static_cast<float>(yaw) * kCdegToRad);
  r.busGuard.release();
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
  r.odometry.seed(static_cast<float>(x), static_cast<float>(y), h);
  r.busGuard.acquire(r.sleeper);
  otosRef().setPose(static_cast<float>(x), static_cast<float>(y), h);
  r.busGuard.release();
}

}  // namespace diffDrive
