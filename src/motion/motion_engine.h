// motion_engine.h -- diffDrive::MotionEngine: the two-primitive reduction
// this project's motion surface is built on. Canonical spec (read-only,
// a different repo -- this project conforms to its grammar, it does not
// vendor its C++): radio-robot-lib/docs/design/motion-api.md S2
// ("Everything is constant-ratio wheel segments") and S2.1 ("b is the
// effective track width") are the whole design; read those two sections
// first.
//
// Host-portable by construction: this file and motion_engine.cpp include
// nothing but <cstdint>/<cmath>, diffdrive.h, segment.h, velocity_shaper.h
// and motion_limits.h -- no pxt.h, no CODAL type, anywhere -- so the
// native host test harness (tests/host/) links and exercises this class
// with no micro:bit involved. Both call paths this codebase has -- the
// TypeScript block API (`blocks/`) via shims.cpp's engine* forwards, and
// the wire adapter (wire_adapter.cpp) via the same forwards -- share
// this one implementation instead of duplicating the math.
//
// SPRINT 029 ("motion profile unification", docs/design/
// motion-profile-unification.md): this class used to carry two
// mutually-unaware shaping algorithms (a legacy elapsed-time ramp/taper
// and a constant-a braking-speed solve, selected by whether
// aAccelMmS2_/aDecelMmS2_ were nonzero) across a 360-line serviceMove().
// Both are GONE. There is now exactly one shaping object
// (VelocityShaper, velocity_shaper.h), one limits object (MotionLimits,
// motion_limits.h, reachable via limits()), and one plan object per
// in-flight command (Segment, segment.h, for position-mode moves; the
// Hold struct below, for continuous drive). service() (renamed from
// serviceMove()) is the ~40-line tick that runs whichever of the two is
// live -- design S5's pseudocode, implemented verbatim in
// motion_engine.cpp. See motion-profile-unification.md S12 for the
// design decisions this rewrite makes (floor is a profile concept now,
// not a kernel one; legacy shaping is deleted, not flagged; wheelsX()
// is closed-loop like moveX(); a segment starts from the floor, not
// from zero) and src/DESIGN.md S3 for the maintained summary.
//
// TWO PRIMITIVES (motion-api.md S1/S2): wheelsX() (per-wheel commanded
// DISTANCE, ratio-locked to a cruise ceiling so both wheels finish
// together) and wheelsV() (per-wheel commanded VELOCITY, held for
// `duration` -- `duration` IS the kernel's own lease, backstopping an
// abandoned hold_ the same way it always has). Everything else in the
// six-operation Motion API reduces onto these two plus the geometry they
// both depend on; see each method's own doc comment below for units and
// contract. Both clear any in-flight command first (motion-api.md S6:
// "wheels_* clears the planner" -- exactly one of Segment/Hold is ever
// live, a new command replaces both).
//
// MOVE ENGINE (motion-api.md S3.3-S3.5), restated as the three
// reductions moveX()/moveV()/goToR()/goToW() below build a Segment and
// hand it to service(). See each method's own comment for the exact
// reduction; SIGN CONVENTION and the pivot-first split threshold are
// unchanged from the code this class was extracted from.
//
// GEOMETRY (motion-api.md S2.1): `effectiveTrackWidth()` is a METHOD,
// deliberately never a stored field, computed as `trackWidth /
// rotationalSlip` every time it is asked for -- so a config read-back
// can never report a derived number as though it had been measured.
// `trackWidth` itself is NEVER "corrected" to make a turn land -- it is
// the one independently-verifiable number in the robot's geometry (a
// caliper reaches it). All rotational scrub correction belongs in
// `rotationalSlip`, separately measurable against camera truth; keeping
// the two apart is what lets a bad turn be diagnosed instead of merely
// compensated (S2.1, and this project's own standing rule -- see this
// repo's CLAUDE.md/sprint.md Success Criteria).
//
// SIGN CONVENTION, unchanged from the code this class is extracted from
// and from motion-api.md S2.1: CCW-positive. A positive twist/rotation
// turns LEFT and increases camera yaw; the left wheel is the slower one
// in a left turn. This is NOT re-derived from cable order anywhere in
// this file -- see tests/host/test_motion_engine_primitives.py's own
// explicit sign-convention tests, written so a future cable-order "fix"
// fails a test instead of shipping (this project has shipped that exact
// bug and patched it four times downstream).
#pragma once

#include <cstdint>

#include "../core/diffdrive.h"
#include "motion_limits.h"
#include "segment.h"
#include "velocity_shaper.h"

namespace diffDrive {

// PoseSource -- a minimal world-pose read port for goToW() (motion-api.md
// S3.6, S9.3 item 3: "the pose source is pluggable... OTOS when fitted,
// encoder odometry otherwise"). Three reads, nothing else, no CODAL/PXT
// dependency -- so a future robot with no OTOS at all (motion-api.md
// S3.6's own `gopiv` example) can supply a trivial always-stale
// implementation without breaking the interface, and the host test
// harness can supply a fake with no OTOS anywhere in the link.
// `OtosPort` (src/platform/otos_port.h) implements this for hardware;
// `FakePoseSource` (tests/host/fake_pose_source.h) implements it for
// tests.
class PoseSource {
 public:
  virtual ~PoseSource() = default;

  virtual float x() const = 0;  // [mm] world frame
  virtual float y() const = 0;  // [mm] world frame

  // [rad] world frame, CCW+. Wrap convention is IMPLEMENTATION-DEFINED
  // -- this interface does NOT mandate wrapped or unwrapped, because
  // this project's two hardware implementations legitimately disagree
  // by construction: `OtosPort` (src/platform/otos_port.h) reports heading
  // WRAPPED to (-pi, pi] (the chip's own int16 register, full scale
  // +/-pi); a Rig-odometry-backed source (motion-api.md S3.6's
  // encoder fallback, `EncoderPoseSource`) is deliberately UNWRAPPED,
  // matching `Rig`'s own odometry contract (`shims.cpp`'s `r.heading`
  // accumulates without normalizing). Both are contractually valid
  // because `MotionEngine::goToR()`/`goToW()` consume this value ONLY
  // through cos()/sin() (wrap-invariant) -- resolves code review
  // KERN-08, which found this comment's former unconditional
  // "(unwrapped)" claim contradicted by `OtosPort`'s own construction.
  // A caller that ever DIFFERENCES two heading() reads (rather than
  // taking their cos/sin) must NOT assume a shared wrap convention
  // across `PoseSource` implementations.
  virtual float heading() const = 0;
};

class MotionEngine {
 public:
  // `kernel`/`clock` are constructed and owned by the CALLER (shims.cpp's
  // Rig for hardware; the host test harness's own fixture for tests) --
  // this class only ever holds references, the same pattern
  // DiffDrive::DifferentialDrive itself uses for its own
  // Motor/Clock/Sleeper/FiberLauncher ports rather than owning them.
  // This class needs its own Clock reference, separate from the
  // kernel's: the move engine's shaping (VelocityShaper's own dt) and
  // its `timeout` backstop both need wall time independent of
  // whether/when the kernel has last step()'d, and kernel_.drive()'s
  // own clock_ reference (used to stamp a lease's `validUntil`) is
  // private to DifferentialDrive. Geometry defaults below are the
  // measured tovez/vevov bake -- see this class's own field comments for
  // the measurement behind each.
  MotionEngine(DiffDrive::DifferentialDrive& kernel,
               const DiffDrive::Clock& clock);

  // ---- geometry (motion-api.md S2.1) ----

  // [mm/deg] wheel travel per shaft degree; 1 count == 0.1 shaft degree,
  // so counts-per-mm is 10 / travelCalib.
  float travelCalib() const { return travelCalib_; }
  void setTravelCalib(float mmPerDeg) { travelCalib_ = mmPerDeg; }

  // [mm] the CALIPER-MEASURED track width. Never adjust this to correct
  // a turn -- see this file's header comment and motion-api.md S2.1.
  float trackWidth() const { return trackWidth_; }
  void setTrackWidth(float mm) { trackWidth_ = mm; }

  // [1] physical/odometric rotation ratio (wheel-contact scrub),
  // camera-measured against ground truth. This is where ALL rotational
  // correction lives -- never trackWidth. See rotationalSlip_'s own
  // field comment (below, next to its default) for the full camera
  // measurement and the derivation chain from that measurement to the
  // constant -- read that comment in full before setting a new value;
  // it names exactly the shortcut that would produce a plausible-looking
  // wrong number.
  float rotationalSlip() const { return rotationalSlip_; }

  // Sprint 007 ticket 005 (closes R-14/API-06): the setter this field
  // never had -- UC-013 (calibrating a non-reference chassis) had no
  // knob to reach `rotationalSlip_` except `set track width`, which the
  // doctrine above forbids using for this. Same ">0, else silently keep
  // the prior value" validation style setGeometry() already applies to
  // trackWidth/travelCalib (shims.cpp) -- inlined directly on the setter
  // here rather than at a shims.cpp call site, since rotationalSlip has
  // no dedicated wire-shaped wrapper the way trackWidth/travelCalib
  // share setGeometry().
  void setRotationalSlip(float slip) {
    if (slip > 0.0f) rotationalSlip_ = slip;
  }

  // [counts/mm] 1 count == 0.1 shaft degree.
  float countsPerMm() const { return 10.0f / travelCalib_; }

  // [mm] b = trackWidth / rotationalSlip (motion-api.md S2.1) -- a
  // METHOD, computed fresh on every call, deliberately never cached into
  // a field so a config read-back can never report a derived number as
  // though it had been measured.
  float effectiveTrackWidth() const { return trackWidth_ / rotationalSlip_; }

  // [mm/s] SUC-003: the distance-chosen default cruise speed (design
  // motion-profile-unification.md S8): v_default(D) = min(limits_.vMax,
  // sqrt(limits_.decel * D)) -- the triangle whose braking half fits in
  // D -- for the moveX()/goToR()/goToW() family's `cruise == 0` "use the
  // configured default" wire sentinel. Same "derived, never cached"
  // pattern as effectiveTrackWidth() above: computed fresh, every call,
  // from limits_ alone, so it can never drift from whatever accel/decel
  // shaping is currently configured. `distance` is clamped to >= 0
  // before the square root so a negative or degenerate leg length can
  // never produce NaN. this ticket: previously read
  // aDecelMmS2_/vMaxMmS_/brakeFrac_ and carried its own "legacy mode"
  // (aDecelMmS2_ == 0) escape hatch; both are gone -- limits_.decel is
  // never 0 (MotionLimits' own default is 400), so this always resolves
  // through the real formula now. See this ticket's own report for the
  // wire-layer consequence (shims.cpp's engineADecelMmS2()/
  // resolveMoveXCruise() now always takes the distance-aware branch).
  float defaultCruiseForDistance(float distance) const;  // [mm] -> [mm/s]

  // [mm] SUC-003 input helper for defaultCruiseForDistance() above: the
  // dominant-axis wheel-travel magnitude moveX()'s own wheels_x-style
  // reduction would produce for (distance, rotation) -- the same
  // `dominant` quantity startSegment() computes, restated here in mm
  // rather than counts so a PURE PIVOT (distance == 0, rotation !=
  // 0) still has a real, nonzero D instead of always resolving to 0 --
  // a pivot's wheels genuinely travel `|rotation| *
  // effectiveTrackWidth() / 2` mm each, even though the chassis itself
  // does not translate. Approximates the exact
  // `max(|distance - rotation*b/2|, |distance + rotation*b/2|)` split as
  // `max(|distance|, |rotation|*b/2)` -- cheaper, and never LARGER
  // than the exact split (same-signed terms only add), so a blended
  // move's resolved default cruise is never more optimistic than the
  // exact reduction would allow. Renamed from dominantAxisTravelMm() --
  // unit now a trailing comment, per this project's naming rule.
  float dominantAxisTravel(float distance, float rotation) const;  // [mm] [rad] -> [mm]

  // ---- the two primitives (motion-api.md S3.1/S3.2) ----

  // wheels_v(left, right, duration): hold each wheel at a commanded
  // velocity [mm/s] for `duration` [ms] -- duration IS the kernel's
  // lease, no reinterpretation. velocity = mean(left, right), twist =
  // half-differential (right - left) -- CCW-positive, per this file's
  // header comment. Clears any in-flight command first (motion-api.md
  // S6: "wheels_* clears the planner"). this ticket: no
  // longer drives the kernel synchronously -- arms `hold_` (target v,
  // twist, deadline) and resets the shaper; service() slews toward the
  // hold through the shaper every tick and issues the actual
  // kernel_.drive() call (design S4.4/S5). This costs one service() tick
  // (~24 ms) of extra latency before the first nonzero command lands,
  // identical to every other entry point now (design S6.5) -- see
  // shims.cpp's isDriving()/commandLooksActive() for how the
  // continuous-drive tick loop stays alive across that first tick.
  void wheelsV(float left, float right, uint32_t duration);  // [mm/s] [mm/s] [ms]

  // wheels_x(left, right, cruise, timeout): move each wheel a commanded
  // DISTANCE [mm] at a ratio locked to `cruise` [mm/s] (the DOMINANT
  // wheel's ceiling, motion-api.md S3.1) so both wheels finish together.
  // this ticket (design S12, S4.4's table): now a Segment,
  // CLOSED-LOOP on encoders like moveX() -- the dead-reckoned lease this
  // primitive used to compute (dominant/cruise, capped by timeout) is
  // gone; `timeout` is now the segment's own real deadline backstop
  // (motion-api.md S3.1: "timeout is a required backstop, not the stop
  // condition" -- previously true only of moveX()'s shaping layer, now
  // true of this primitive directly, since the two primitives had no
  // other reason to differ in how they end). A zero-magnitude command
  // (both wheels commanding no distance) or a non-positive cruise
  // commands nothing NEW -- but it is not purely inert: it also stops
  // any motion already in progress (stages kernel_.neutral()), including
  // a still-live wheelsV() hold, since this primitive's own "clear the
  // planner" step (above) never touches the kernel by itself. Clears
  // any in-flight command first, same as wheelsV() above.
  void wheelsX(float left, float right, float cruise, uint32_t timeout);  // [mm] [mm] [mm/s] [ms]

  // [rad] the |rotation| threshold moveX() (below) uses to decide
  // pivot-then-straight vs one blended segment -- the single source of
  // truth for `kTurnFirstAngle` (private, below), exposed here so a
  // caller that must mirror moveX()'s own split decision (e.g.
  // shims.cpp's startMove(), budgeting a caller-supplied timeout) reads
  // it from this class instead of re-typing the constant a second time.
  static constexpr float turnFirstAngle() { return kTurnFirstAngle; }

  // ---- move engine (motion-api.md S3.3-S3.5) -- see this file's header
  // comment for the shape of each reduction. ----

  // move_x(distance, rotation, cruise, timeout): see header comment.
  // Supersedes any in-flight command (this call's own prior phase, or a
  // previous moveX()/goToR()/wheelsX()/wheelsV() never finished) --
  // exactly one Segment or Hold is ever active at a time.
  void moveX(float distance, float rotation, float cruise,
             uint32_t timeout);  // [mm] [rad] [mm/s] [ms]

  // move_v(vx, omega, duration): the plain wheelsV reduction --
  // vx +- omega*b/2 -- held for `duration`, no shaping beyond what
  // wheelsV()'s own hold already gets. CCW-positive, per this file's
  // header comment.
  void moveV(float vx, float omega, uint32_t duration);  // [mm/s] [rad/s] [ms]

  // go_to_r(x, y, speed, arrive, timeout): see header comment. `x`
  // forward, `y` left, both [mm]; `speed` is the resulting segment's
  // cruise. A target within `arrive` [mm] of the current position
  // (radially: `hypot(x, y) <= arrive`; (0, 0) with any `arrive >= 0`
  // is included) is a no-op -- nothing is driven (sprint 006, KERN-04).
  // Otherwise this method makes its OWN pivot-vs-blend split decision
  // (sprint 006, KERN-02/03) instead of inheriting moveX()'s generic
  // one -- see header comment for why, and for the short-arc
  // normalization applied to the arc angle before that decision.
  void goToR(float x, float y, float speed, float arrive,
             uint32_t timeout);  // [mm] [mm] [mm/s] [mm] [ms]

  // go_to_w(x, y, speed, arrive, timeout): see header comment. `x`, `y`
  // are WORLD-frame [mm]; `pose` supplies the current world pose this
  // call reads ONCE, at call time -- not stored. Rotates the world-frame
  // delta (x - pose.x(), y - pose.y()) into the body frame by
  // -pose.heading() (this file's CCW-positive convention) and delegates
  // to goToR() above. A target equal to the current pose reduces to a
  // (0, 0) body-frame delta, which goToR() already treats as a no-op.
  void goToW(const PoseSource& pose, float x, float y, float speed,
             float arrive, uint32_t timeout);  // [mm] [mm] [mm/s] [mm] [ms]

  // The single per-tick advance (design S5, motion-profile-
  // unification.md): dispatches whichever of seg_/hold_ is active,
  // through shaper_, at most one kernel_.drive()/neutral() per call.
  // Renamed from serviceMove() (this ticket) -- no other
  // behavior change to this method's OWN contract: callers still invoke
  // it once per control cycle while isDriving()/isMoveActive(), and it
  // still owns nothing about odometry (callers update that themselves
  // around this call, exactly as before).
  bool service();

  // True iff a position-mode Segment (MOVE_X/GO_TO_R/GO_TO_W/WHEELS_X)
  // is in flight -- unchanged contract from before this ticket: a
  // continuous wheelsV() hold does NOT make this true (see isDriving()
  // below for the union of both).
  bool isMoveActive() const { return seg_.active; }

  // True iff EITHER a Segment or a continuous Hold is currently driving
  // the wheels -- this ticket addition, not itself part of
  // the design doc's pseudocode: wheelsV()/wheelsX()/moveX() no longer
  // call kernel_.drive() synchronously (design S6.5's lazy-start applies
  // uniformly), so a caller that used to infer "something is driving"
  // from the kernel's own Output.appliedDutyLeft/Right immediately after
  // arming a hold would see stale zero duty for one extra tick. This is
  // the accessor shims.cpp's commandLooksActive() now reads instead of
  // isMoveActive() for exactly that reason -- see that function's own
  // comment.
  bool isDriving() const { return seg_.active || hold_.active; }

  // Force-end the current command now (no-op if neither a Segment nor a
  // Hold is active): neutrals the kernel if something was active, resets
  // the shaper, then clears both seg_/hold_ (design S4.4's table).
  void endMove();

  // Fraction of the current Segment's dominant axis completed,
  // [0..1000]; 1000 if no Segment is active (matches "isMoving()? ->
  // false" reading as "already there"). A continuous wheelsV() hold has
  // no notion of "done" -- unaffected by this method, same as before.
  int progress() const;

  uint32_t wrongWayCount() const { return wrongWayCount_; }

  // ---- settle-tick decision (sprint 008 ticket 004) ----
  // Extracted verbatim from shims.cpp::tickDrive()'s former inline loop
  // -- see that call site's own comment, carried forward here, for the
  // full bench history this guards against (commit 3e919e5,
  // 2026-08-20): kernel_.neutral() only STAGES a zero command; delivery
  // to the motors happens on the kernel's NEXT step(), and that one
  // extra step's own encoder read can land mid-spin-down, freezing
  // Output.velocityLeft/Right at a nonzero value forever unless the
  // kernel keeps stepping until both wheels are MEASURED at rest.
  // Steps the kernel up to kSettleMaxSteps times, breaking as soon as
  // BOTH wheels' measured velocity (Output.velocityLeft/Right) reads
  // within kSettleRestCountsPerS of zero -- byte-for-byte the same
  // bounded-iteration/break-on-rest decision the loop it replaces made,
  // just relocated here.
  //
  // Deliberately does NOT fold anything into odometry, and knows
  // nothing about Rig-local x/y/heading -- odometry ownership stays
  // with the CALLER (shims.cpp's tickDrive()), which must call its own
  // odomUpdate()-equivalent itself, once, immediately after this
  // returns, exactly as the loop it replaces did.
  //
  // Never issues a new kernel_.drive()/neutral() command of its own --
  // it only steps the kernel and reads Output back, so a settled (or
  // already-neutral) input produces no additional nonzero duty. Callers
  // must invoke this from their own single ticker only (tickDrive() is
  // this codebase's one caller) -- this method starts no fiber and
  // creates no new caller of its own, so the "exactly one fiber ticks a
  // move" invariant is unaffected by this extraction.
  void settleToRest();

  // The ONE settable shaping surface (design S4.4): accel/decel/jerk/
  // vMax/omegaMax ceilings, vFloor/omegaFloor floors, and the arrival
  // windows (stopDistance/arriveDist/arriveYaw) all live on the returned
  // MotionLimits. Replaces the thirteen fields/thirteen setters this
  // ticket deletes (distTaper_, yawTaper_, distFloor_, turnFloor_,
  // rampMs_, brakeFrac_, plateauMinS_, profileExitMmS_, pivotOverrunMm_,
  // aAccelMmS2_, aDecelMmS2_, vMaxMmS_, jerkMmS3_, maxYawRateDegS_) --
  // see motion_limits.h for each surviving field's own comment.
  MotionLimits& limits() { return limits_; }
  const MotionLimits& limits() const { return limits_; }

 private:
  // |rotation| at/above this is NOT one blended segment -- pivot to the
  // new heading first, then travel straight (motion-api.md S3.3,
  // `navigator.cpp:237-240`'s measured `turn_first_angle`). 50 deg.
  static constexpr float kTurnFirstAngle = 0.8726646f;

  // settleToRest()'s own bound/threshold (sprint 008 ticket 004
  // extraction) -- byte-for-byte shims.cpp's former loop cap and its
  // former local `kRest`, just relocated and named. [steps] / [counts/s,
  // ~2 mm/s].
  static constexpr int kSettleMaxSteps = 12;
  static constexpr float kSettleRestCountsPerS = 25.0f;

  // [mm/s] [mm/s] the pair a caller reads back from axisLimits() below
  // -- a plain aggregate (no default member initializers, so it stays a
  // C++11 aggregate; see tests/host/test_cxx11_syntax_gate.py's own
  // header comment on why that distinction matters in this file).
  struct AxisLimits {
    float floor;
    float cap;
  };

  // Continuous-drive state (design S4.4): wheelsV()'s own target, slewed
  // toward through shaper_ every service() tick while active. Exactly
  // one of seg_/hold_ is ever live -- cancelMove() (below) clears both.
  struct Hold {
    bool active = false;
    float v = 0.0f;         // [mm/s] target mean velocity
    float twist = 0.0f;     // [mm/s] target half-differential
    float dominant = 0.0f;  // [mm/s] max(|v-twist|, |v+twist|) -- the
                            //   dominant wheel's own target speed
                            //   (design S5: `target = hold_.dominant`)
    uint32_t until = 0;     // [ms] the caller's duration deadline
  };

  // [ms] this engine's own notion of "now" -- see the constructor
  // comment on why a separate Clock reference is needed at all.
  uint32_t now() const;

  // design S6.2: converts THIS segment's own axis (pure turn -> deg/s,
  // else linear mm/s) into the dominant-wheel [mm/s] floor/cap
  // VelocityShaper::advance() wants, using limits_ and
  // effectiveTrackWidth(). Pure turn: floor = omegaFloorAsWheelSpeed(b),
  // cap = omegaMaxAsWheelSpeed(b) if limits_.omegaMax > 0, else +inf.
  // Anything else (a straight leg or a blended arc): floor = vFloor,
  // cap = +inf -- motion-api.md S3.3's own "the pivot rate is derived
  // from cruise, this design adds a ceiling and a floor on that derived
  // rate" note applies only to the dominant WHEEL speed on a pure turn;
  // an arc's linear axis has no such wire-facing cap of its own.
  AxisLimits axisLimits(const Segment& seg) const;

  // Builds seg_ from (distance, rotation, cruise): the shared
  // tail of wheelsX()'s per-wheel reduction and moveX()'s
  // distance/rotation reduction (both compute distTarget/yawTarget in
  // counts, then call this). Sets seg_.active/originPending/dominant/
  // dominantAxis, resets shaper_, and stages NO kernel_.drive() call --
  // design S6.5's lazy origin capture means the first real command is
  // issued by the FIRST service() call, not here. A zero-magnitude
  // command or a non-positive cruise leaves seg_.active false and
  // stages kernel_.neutral() unconditionally (same degenerate contract
  // wheelsX()/the old startSegment() always had).
  void beginSegment(float distTarget, float yawTarget, float cruise,
                    uint32_t deadline);  // [counts] [counts] [mm/s] [ms]

  // Queue a pivot to `pivotRotation` now, then `straightDistance` [mm]
  // straight once that pivot completes cleanly -- the shared tail of
  // moveX()'s own pivot-first split (motion-api.md S3.3) and goToR()'s
  // above-threshold bearing-pivot-then-chord split (sprint 006,
  // KERN-02): both are "pivot then straight," differing only in which
  // (rotation, distance) pair is queued. `deadline` is the ONE
  // deadline spanning both phases (motion-api.md's single `timeout`
  // field).
  void queuePivotThenStraight(float pivotRotation, float straightDistance,
                               float cruise,
                               uint32_t deadline);  // [rad] [mm] [mm/s] [ms]

  // service()'s own phase 1 -> phase 2 handoff tail (design S6.4): reads
  // seg_.pendingDistance/pendingCruise/deadline (captured into locals
  // BEFORE beginSegment() below resets seg_ to a fresh default), then
  // starts the straight phase exactly like a fresh beginSegment() call
  // -- lazy origin capture (S6.5) means this phase's own origin is
  // captured on the FOLLOWING service() tick, after the caller's own
  // step() has delivered the neutral service() already staged and K4
  // has disarmed the kernel's references.
  void beginPendingStraightPhase();

  // Clears BOTH seg_ and hold_ without touching the kernel -- the
  // shared tail of endMove() and of every primitive/reduction's own
  // "clear the planner" contract (motion-api.md S6). this ticket: now clears hold_ too (the old cancelMove() only ever cleared
  // move_, since wheelsV() had no engine-tracked state of its own
  // before this ticket) -- "exactly one of Segment/Hold is ever live, a
  // new command replaces both" (design S4.4).
  void cancelMove();

  DiffDrive::DifferentialDrive& kernel_;
  const DiffDrive::Clock& clock_;

  // vevov-measured travel calibration. Generic kits calibrate via
  // setTravelCalib()/setTrackWidth() (shims.cpp's setGeometry() block).
  //
  // CAMERA-MEASURED 2026-08-25 on the playfield, and this REPLACES the
  // 0.8102 that stood here. That entry came from a single tape
  // measurement (2026-08-19: commanded 80 cm, odometry believed 798 mm,
  // tape measured 825 mm -> 0.7837 * 825/798 = 0.8102) which raised the
  // constant. This measurement says the raise was in the WRONG
  // DIRECTION: the robot travels ~2.8% LESS than it believes, not more.
  // The new value lands within 0.5% of the 0.7837 that 2026-08-19
  // replaced, so this is close to a revert of that change.
  //
  // Why trust this over the tape: twelve `RUN:straight` legs at three
  // distances (30/55/85 cm), both directions, each bracketed by
  // overhead-AprilCam fixes taken AT REST. `RUN:straight` is the clean
  // probe -- test.ts documents it as wheels-only, with no OTOS, no world
  // frame and no heading correction, so nothing is quietly steering it.
  // The camera's own scale was verified in the same session against
  // three fixed field-tag pairs of known separation: +0.13%, -0.09%,
  // -0.11%. A tape over 80 cm cannot beat that.
  //
  //   commanded 85 cm -> odometry believed 85.10 cm (control is fine,
  //   0.1%) -> camera measured 82.7 cm.
  //
  // SCALE, not offset -- which is what makes this constant the right
  // knob. Fitting shortfall = a + b*distance over the three distances
  // gives b = 3.07% with a = -0.20 cm; forcing the physically-motivated
  // zero intercept gives 2.7608% with residuals under 0.21 cm. A
  // stopping/deadline overshoot would have shown up as a constant `a`
  // and left `b` near zero, and would NOT have been fixable here.
  //   0.8102 * (1 - 0.027608) = 0.7878
  //
  // KNOCK-ON FOR ROTATION, which must not be "fixed" twice: heading is
  // (wheel travel)/track, so this scale error propagated into rotation
  // identically. Isolated camera-truthed 90 deg pivots measured
  // camera/encoder 0.9805 BEFORE this change; 0.9805/0.9724 = 1.0093,
  // so once travel is right the robot should OVER-rotate by ~0.9% and
  // that residual -- not the raw 0.9805 -- is what rotationalSlip_
  // below would have to answer for. Re-measure rotation after this
  // lands before touching that constant.
  float travelCalib_ = 0.7878f;  // [mm/deg] wheel travel per shaft degree

  // [mm] MEASURED track (stakeholder tape, 2026-08-19). This is the
  // robot's geometry; it is never "corrected" -- turning slip is
  // modeled separately by rotationalSlip_ below.
  float trackWidth_ = 114.2f;

  // [1] physical/odometric rotation ratio (wheel-contact scrub).
  // CAMERA-MEASURED 2026-08-20 on the playfield, overhead AprilCam vs
  // commanded: six steady-state 180 deg pivots turned 164-166 deg
  // physical, ratio 0.915.
  //
  // 0.915 is NOT the slip -- do not set rotationalSlip_ to 0.915 (or to
  // any number reproduced by re-running this same experiment and
  // stopping at the ratio). It is the ratio between the ACTUAL physical
  // rotation and the rotation the firmware commanded that day, and what
  // the firmware commanded was itself computed through the STALE
  // effectiveTrackWidth in effect at the time it ran: trackWidth_
  // (114.2, already tape-measured 2026-08-19, unchanged since) divided
  // by the 1.040 rotationalSlip_ this entry replaces, i.e.
  // 114.2/1.040 = 109.8 mm. Since the robot under-rotated (164-166 <
  // 180), the TRUE effectiveTrackWidth must be LARGER than that 109.8 --
  // specifically 109.8/0.915 = 120.0 mm (dividing, not multiplying, by
  // the ratio -- effectiveTrackWidth and commanded-vs-actual rotation
  // move in opposite directions in motion_engine.cpp's kinematics: a
  // bigger b means the SAME wheel travel yields LESS rotation, matching
  // this measurement). Only then: slip = trackWidth_/effectiveTrackWidth
  // = 114.2/120.0 = 0.952. Reproducing 164-166/180 = 0.915 from the same
  // experiment and "fixing" 0.952 to match is exactly the bridge this
  // comment exists to block -- the dropped middle step (109.8 -> 120.0)
  // is what separates the two numbers.
  //
  // REPLACES 1.040, which came from a single camera pivot on 2026-08-19
  // and had the sign of the effect BACKWARDS (it said the robot
  // over-rotated; it under-rotates). The OTOS agreed with the camera to
  // 1.005 across ten pivots, so the sensor was never the problem -- this
  // constant was.
  float rotationalSlip_ = 0.952f;

  // ---- move/hold state (design S4.4) ----

  Segment seg_;
  Hold hold_;
  VelocityShaper shaper_;
  MotionLimits limits_;

  // [ms] the previous service() tick's own now(), for the shaper's
  // `dt` -- set at every genuine start (a fresh Segment, or a Hold
  // transitioning from idle/a superseded Segment) so the FIRST tick's
  // dt is measured from when the command was armed, not from some
  // unrelated earlier tick. A dead read before either seg_/hold_ has
  // ever gone active (service() returns before touching it).
  uint32_t lastTick_ = 0;

  // Moves aborted because the robot was rotating AWAY from the
  // commanded direction (service()). Cumulative since construction.
  uint32_t wrongWayCount_ = 0;
};

}  // namespace diffDrive
