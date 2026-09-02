// motion_engine.h -- diffDrive::MotionEngine: the two-primitive reduction
// this project's motion surface is built on. Canonical spec (read-only,
// a different repo -- this project conforms to its grammar, it does not
// vendor its C++): radio-robot-lib/docs/design/motion-api.md S2
// ("Everything is constant-ratio wheel segments") and S2.1 ("b is the
// effective track width") are the whole design; read those two sections
// first.
//
// Host-portable by construction: this file and motion_engine.cpp include
// nothing but <cstdint>/<cmath> and diffdrive.h -- no pxt.h, no CODAL
// type, anywhere -- so the native host test harness (tests/host/) links
// and exercises this class with no micro:bit involved. Both call paths
// this codebase has -- the TypeScript block API (`blocks/`) via
// shims.cpp's engine* forwards, and the wire adapter (wire_adapter.cpp) via
// the same
// forwards -- share this one implementation instead of duplicating the
// math.
//
// TWO PRIMITIVES (motion-api.md S1/S2): wheelsX() (per-wheel commanded
// DISTANCE, ratio-locked to a cruise ceiling so both wheels finish
// together) and wheelsV() (per-wheel commanded VELOCITY, held for
// `duration` -- `duration` IS the kernel's own lease). Everything else
// in the six-operation Motion API reduces onto these two plus the
// geometry they both depend on; see each method's own doc comment below
// for units and contract. Both clear the move engine's own in-flight
// state first (see MOVE ENGINE below) -- motion-api.md S6: "wheels_*
// clears the planner."
//
// MOVE ENGINE (motion-api.md S3.3-S3.5). The taper/ramp/wrong-way-abort/
// settle SHAPING that used to live in shims.cpp's Rig::serviceMove()/
// startMove() moves here verbatim (algorithm unchanged, only its home
// and calling convention), restated as the three reductions:
//
//   moveX(distance, rotation, cruise, timeout) -- body distance [mm] +
//     heading change [rad] CCW+, reduced onto wheelsX's ratio math
//     (distance -+ rotation*b/2). |rotation| >= 50 deg
//     (kTurnFirstAngleRad, motion-api.md S3.3's measured
//     `turn_first_angle`) with a nonzero distance is NOT one segment:
//     pivot to the new heading first, then travel the remainder
//     straight -- queued internally as one caller-visible moveX() call.
//     A live encoder-progress check (not just the dead-reckoned lease
//     wheelsX alone provides) is what stops each segment on arrival;
//     `timeout` is a REAL backstop tracked independently of that,
//     spanning the whole call (both phases, if two).
//   moveV(vx, omega, duration) -- the plain wheelsV reduction, no
//     shaping (a velocity hold has no "end" to taper toward).
//   goToR(x, y, speed, arrive, timeout) -- the spec's arc reduction
//     (turn angle theta = 2*atan2(y,x), arc length motion-api.md S3.5),
//     but goToR OWNS its own pivot-vs-blend split decision rather than
//     inheriting moveX()'s generic one (sprint 006, KERN-02): moveX()'s
//     |rotation| >= 50 deg split reissues theta/arc-length as pivot-
//     then-straight, which lands at a DIFFERENT endpoint than the
//     blended arc whenever it fires (arc length != chord length except
//     in the limit) -- goToR() instead pivots to the line-of-sight
//     bearing (atan2(y,x)) then drives the straight-line chord
//     (hypot(x,y)), which reaches (x, y) exactly, by construction.
//     `theta` is normalized to the short arc, (-pi, pi], before this
//     split decision (and before the plain-arc branch below threshold
//     uses it) -- doubling atan2's own principal value can otherwise
//     land up to just under +-2*pi, which is "the long way around" the
//     same constant-curvature circle as the short, wrapped angle (both
//     reach the same (x, y), but only the short one is a sane distance
//     to drive); this is what keeps a target nearly directly behind the
//     robot from being driven the long way around a huge circle
//     (sprint 006, KERN-03). `arrive` is now honored as a radial no-op
//     gate (sprint 006, KERN-04): `hypot(x, y) <= arrive` issues no
//     segment at all -- still a single-shot reduction, not the
//     supervisory re-solving loop motion-api.md S3.5 describes; a caller
//     that wants that re-issues goToR itself. This heuristic-free
//     reduction remains distinct from this project's own goToWorld() in
//     `blocks/world.ts`, a separate, TS-level turn-first/capped-curvature
//     call path (sprint.md Design Rationale: two paths sharing one primitive,
//     not one implementation).
//   goToW(pose, x, y, speed, arrive, timeout) -- the WORLD-frame
//     counterpart: motion-api.md S3.6, "go_to_w(x, y) ==
//     read pose -> world-to-body -> go_to_r". Reads `pose`'s current
//     (x, y, heading), rotates the world-frame delta into the body frame,
//     and delegates to goToR() above -- same single-shot, no-
//     supervisory-re-solve posture, same separateness from goToWorld()'s
//     own TS-level heuristic. `pose` is a PoseSource reference supplied
//     PER CALL (see the PoseSource class below) -- motion-api.md S9.3
//     item 3: "go_to_w's pose source is pluggable rather than assuming
//     an OTOS is fitted", because the fleet is not uniform (S3.6's own
//     `gopiv` example has no OTOS at all). MotionEngine holds no
//     PoseSource of its own; the caller (a wire adapter, a shim) chooses
//     which one to pass, which is what makes this class host-testable
//     with a fake pose with no OTOS anywhere in the link.
//
// serviceMove() is the per-tick advance: callers (shims.cpp's
// updateMove()/tickDrive(), formerly Rig::serviceMove()'s only callers)
// invoke it once per control cycle while isMoveActive() to re-scale the
// taper/ramp, check completion/deadline/stall/wrong-way, and reissue
// kernel_.drive() every tick while active -- the same "cheap, lease-safe
// reissue" scheme the code it is extracted from used, because gating
// reissues on a scale CHANGE would let the lease expire during any
// steady phase. Odometry (Rig's x/y/heading) stays OUT of this class --
// it is a shims.cpp/Rig concern this ticket does not move -- so callers
// must update it themselves around serviceMove(), exactly as the code
// this is extracted from did inside the old free-function serviceMove().
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
  // kernel's: the move engine's ramp (elapsed time since a segment
  // started) and its `timeout` backstop both need wall time independent
  // of whether/when the kernel has last step()'d, and kernel_.drive()'s
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

  // [mm] per-wheel END-OF-MOVE OVERRUN subtracted from every rotation
  // target (startSegment()'s yawTarget). MEASURED vevov 2026-08-29,
  // captures/vevov-square-20260829/runC.json + runC2.json, reports/
  // vevov-tour-C-firmware-and-telemetry-20260829.md S4: thirteen MOVE_X
  // pivots of 3..90 deg all landed a CONSTANT ~+2 deg past their
  // command in the robot's OWN encoders (+3.2 -> +5.1, -2.6 -> -4.4,
  // +87.3 -> +89.2 ...), camera agreeing; 2 deg at trackWidth 128 is
  // 2.2 mm per wheel. Constant, not a scale -- a 2 % correction would
  // have fixed the 90 deg corners and left a 3 deg pivot at 5 deg, so
  // it lives here as a per-wheel distance, not on rotationalSlip_.
  // Default 0 (no compensation) so no robot changes behaviour until
  // it has been measured; make_deploy.py's geometry bake sets it per
  // robot (radio-robot-lib config `firmware_bake.pivot_overrun_mm`),
  // and wire `SET pivot_overrun <mm>` tunes it live. Mechanism
  // UNVERIFIED: one kernel tick at the 70 mm/s speed floor is 2.3 mm.
  float pivotOverrunMm() const { return pivotOverrunMm_; }
  void setPivotOverrunMm(float mm) {
    if (mm >= 0.0f) pivotOverrunMm_ = mm;
  }

  // [counts/mm] 1 count == 0.1 shaft degree.
  float countsPerMm() const { return 10.0f / travelCalib_; }

  // [mm] b = trackWidth / rotationalSlip (motion-api.md S2.1) -- a
  // METHOD, computed fresh on every call, deliberately never cached into
  // a field so a config read-back can never report a derived number as
  // though it had been measured.
  float effectiveTrackWidth() const { return trackWidth_ / rotationalSlip_; }

  // [mm/s] SUC-003: the distance-chosen default cruise speed --
  // v_default(D) = min(vMaxMmS_, sqrt(2 * aDecelMmS2_ * brakeFrac_ *
  // D)) -- for the moveX()/goToR()/goToW() family's `cruise == 0` "use
  // the default" wire sentinel. Same "derived, never cached" pattern as
  // effectiveTrackWidth() above: computed fresh, every call, from the
  // SAME aDecelMmS2_/vMaxMmS_/brakeFrac_ fields (see their own comments
  // below) the taper's own braking-speed solve already reads, so the
  // two can never drift apart. This method carries no legacy branch of
  // its own -- WHETHER to call it at all (vs. the flat legacy default)
  // is a wire-layer decision the caller makes by checking aDecelMmS2_
  // first; with aDecelMmS2_ == 0.0f this simply returns 0.0f, which the
  // wire layer's existing non-positive-cruise refusal already treats
  // the same way it treats an explicit `cruise <= 0`. `distanceMm` is
  // clamped to >= 0 before the square root so a negative or degenerate
  // leg length can never produce NaN.
  float defaultCruiseForDistance(float distanceMm) const;

  // [mm] SUC-003 input helper for defaultCruiseForDistance() above: the
  // dominant-axis wheel-travel magnitude moveX()'s own wheels_x-style
  // reduction would produce for (distanceMm, rotationRad) -- the same
  // `dominant` quantity startSegment() computes (motion_engine.cpp),
  // restated here in mm rather than counts so a PURE PIVOT
  // (distanceMm == 0, rotationRad != 0) still has a real, nonzero D
  // instead of always resolving to 0 -- a pivot's wheels genuinely
  // travel `|rotationRad| * effectiveTrackWidth() / 2` mm each, even
  // though the chassis itself does not translate. Approximates the
  // exact `max(|distance - rotation*b/2|, |distance + rotation*b/2|)`
  // split as `max(|distanceMm|, |rotationRad|*b/2)` -- cheaper, and
  // never LARGER than the exact split (same-signed terms only add), so
  // a blended move's resolved default cruise is never more optimistic
  // than the exact reduction would allow.
  float dominantAxisTravelMm(float distanceMm, float rotationRad) const;

  // ---- the two primitives (motion-api.md S3.1/S3.2) ----

  // wheels_v(left, right, duration): hold each wheel at a commanded
  // velocity [mm/s] for `duration` [ms] -- duration IS the kernel's
  // lease, no reinterpretation. velocity = mean(left, right), twist =
  // half-differential (right - left) -- CCW-positive, per this file's
  // header comment. Clears any in-flight moveX()/goToR() move first
  // (motion-api.md S6: "wheels_* clears the planner" -- exactly one
  // subsystem owns motion).
  void wheelsV(float left, float right, uint32_t durationMs);

  // wheels_x(left, right, cruise, timeout): move each wheel a commanded
  // DISTANCE [mm] at a ratio locked to `cruise` [mm/s] (the DOMINANT
  // wheel's ceiling, motion-api.md S3.1) so both wheels finish together.
  // This primitive's bound is dead-reckoned: the dominant wheel's own
  // commanded distance divided by cruise gives the lease, capped by the
  // required `timeout` [ms] backstop -- no live encoder-progress check;
  // that closed-loop stop condition is moveX()'s own shaping layer,
  // below, built on top of this primitive's kinematics, not inside it.
  // A zero-magnitude command (both wheels commanding no distance) or a
  // non-positive cruise commands nothing NEW -- but it is not purely
  // inert: it also stops any motion already in progress (stages
  // kernel_.neutral()), including a still-live wheelsV() hold, since
  // this primitive's own "clear the planner" step (above) never touches
  // the kernel by itself. Clears any in-flight moveX()/goToR() move
  // first, same as wheelsV() above.
  void wheelsX(float left, float right, float cruise, uint32_t timeoutMs);

  // [rad] the |rotation| threshold moveX() (below) uses to decide
  // pivot-then-straight vs one blended segment -- the single source of
  // truth for `kTurnFirstAngleRad` (private, below), exposed here so a
  // caller that must mirror moveX()'s own split decision (e.g.
  // shims.cpp's startMove(), budgeting a caller-supplied timeout) reads
  // it from this class instead of re-typing the constant a second time.
  static constexpr float turnFirstAngleRad() { return kTurnFirstAngleRad; }

  // ---- move engine (motion-api.md S3.3-S3.5) -- see this file's header
  // comment for the shape of each reduction. ----

  // move_x(distance, rotation, cruise, timeout): see header comment.
  // Supersedes any in-flight move (this call's own prior phase, or a
  // previous moveX()/goToR() never finished) -- exactly one moveX()-
  // family move is ever active at a time.
  void moveX(float distance, float rotation, float cruise,
             uint32_t timeoutMs);

  // move_v(vx, omega, duration): the plain wheelsV reduction --
  // vx +- omega*b/2 -- held for `duration`, no shaping. CCW-positive,
  // per this file's header comment.
  void moveV(float vx, float omega, uint32_t durationMs);

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
             uint32_t timeoutMs);

  // go_to_w(x, y, speed, arrive, timeout): see header comment. `x`, `y`
  // are WORLD-frame [mm]; `pose` supplies the current world pose this
  // call reads ONCE, at call time -- not stored. Rotates the world-frame
  // delta (x - pose.x(), y - pose.y()) into the body frame by
  // -pose.heading() (this file's CCW-positive convention) and delegates
  // to goToR() above. A target equal to the current pose reduces to a
  // (0, 0) body-frame delta, which goToR() already treats as a no-op.
  void goToW(const PoseSource& pose, float x, float y, float speed,
             float arrive, uint32_t timeoutMs);

  // Advance the current move by one control cycle. See header comment
  // for the full contract (taper/ramp/deadline/wrong-way, one reissue
  // per call while active, neutral-on-end). No-op (returns false) if no
  // move is active. Callers own odometry around this call -- see header
  // comment.
  bool serviceMove();

  bool isMoveActive() const { return move_.active; }

  // Force-end the current move now (no-op if none): neutrals the kernel
  // if a move was active, then clears the move-engine's own state.
  void endMove();

  // Fraction of the current move's dominant axis completed, [0..1000];
  // 1000 if no move is active (matches "isMoving()? -> false" reading as
  // "already there").
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
  // returns, exactly as the loop it replaces did. This is a narrower
  // cut than a sprint-003-era comment on the old loop anticipated
  // ("extracting cleanly would mean moving odometry ownership into
  // motion_engine too") -- that objection is about extracting the
  // whole settle-then-integrate behavior as ONE unit; it does not apply
  // once the settle DECISION and the odometry fold stay two separate
  // calls, which is what this method's contract preserves.
  //
  // Never issues a new kernel_.drive()/neutral() command of its own --
  // it only steps the kernel and reads Output back, so a settled (or
  // already-neutral) input produces no additional nonzero duty. Callers
  // must invoke this from their own single ticker only (tickDrive() is
  // this codebase's one caller) -- this method starts no fiber and
  // creates no new caller of its own, so the "exactly one fiber ticks a
  // move" invariant is unaffected by this extraction.
  void settleToRest();

  // ---- end-of-move shaping knobs (settable per tour). See this class's
  // own field comments (below) for what each trades off. Getters added
  // below for read-back (these five previously had setters only). --
  float distTaper() const { return distTaper_; }
  void setDistTaper(float counts) { distTaper_ = counts; }
  float yawTaper() const { return yawTaper_; }
  void setYawTaper(float counts) { yawTaper_ = counts; }
  float distFloor() const { return distFloor_; }
  void setDistFloor(float fraction) { distFloor_ = fraction; }
  float turnFloor() const { return turnFloor_; }
  void setTurnFloor(float fraction) { turnFloor_ = fraction; }
  float rampMs() const { return rampMs_; }
  void setRampMs(float ms) { rampMs_ = ms; }

  // ---- constant-a acceleration/deceleration shaping. See this class's
  // own field comments (below, next to each default) for the measured
  // defect each constant replaces and the value behind each default.
  // All four default to values that select LEGACY MODE
  // (aAccelMmS2_ == 0.0f && aDecelMmS2_ == 0.0f): startSegment()/
  // serviceMove() run their original formulas bit-for-bit -- see
  // motion_engine.cpp's own comments at both call sites -- until a
  // caller sets a nonzero accel/decel. Validation mirrors
  // setPivotOverrunMm()/setRotationalSlip() above: an invalid input
  // silently keeps the prior value rather than erroring, so a bad
  // external write cannot corrupt engine state.
  float aAccelMmS2() const { return aAccelMmS2_; }
  void setAAccelMmS2(float mmS2) {
    if (mmS2 > 0.0f) aAccelMmS2_ = mmS2;
  }
  float aDecelMmS2() const { return aDecelMmS2_; }
  void setADecelMmS2(float mmS2) {
    if (mmS2 > 0.0f) aDecelMmS2_ = mmS2;
  }
  float vMaxMmS() const { return vMaxMmS_; }
  void setVMaxMmS(float mmS) {
    if (mmS > 0.0f) vMaxMmS_ = mmS;
  }
  float brakeFrac() const { return brakeFrac_; }
  void setBrakeFrac(float frac) {
    if (frac > 0.0f && frac <= 1.0f) brakeFrac_ = frac;
  }

  // Jerk bound, plateau demand and turn-rate cap. Same
  // invalid-input-keeps-prior-value validation as the four above, and
  // the same legacy-selecting default of 0: with jerkMmS3_ == 0 the
  // shaper stays first-order (bounded acceleration, stepped), and with
  // plateauMinS_ == 0 no cruise derate is applied.
  float jerkMmS3() const { return jerkMmS3_; }
  void setJerkMmS3(float mmS3) {
    if (mmS3 > 0.0f) jerkMmS3_ = mmS3;
  }
  float plateauMinS() const { return plateauMinS_; }
  void setPlateauMinS(float sec) {
    if (sec >= 0.0f) plateauMinS_ = sec;
  }
  float maxYawRateDegS() const { return maxYawRateDegS_; }
  void setMaxYawRateDegS(float degS) {
    if (degS > 0.0f) maxYawRateDegS_ = degS;
  }

  // [mm/s] PROFILE-COMPLETION EXIT. A move normally ends only when the
  // encoders land inside a fixed position margin -- 10 counts (0.79 mm)
  // on the distance axis, 4 counts (0.16 deg) on a pure turn. The
  // velocity profile is a separate thing: it decays toward zero at the
  // target, and then the taper FLOOR (distFloor_/turnFloor_) holds the
  // command up so the move can still close that last fraction of a
  // millimetre. Those two halves are why a shaped move still arrives
  // hot and then hunts -- the profile is overridden exactly where it
  // was supposed to land.
  //
  // Set this above 0 (and aDecelMmS2_ > 0) to end the move when the
  // PROFILE is done instead: the taper floor is bypassed so the command
  // genuinely decays, and the move terminates once the dominant axis's
  // commanded speed falls to this value. The residual position error is
  // accepted rather than chased.
  //
  // Why accepting it is reasonable: that 0.79 mm is in ENCODER COUNTS,
  // not in the world. Legs run 0.3-0.7% long, so a 600 mm leg is
  // already 2-4 mm out however precisely the counts land -- the crawl
  // buys precision in believed position only. A sensible value is just
  // above the drivetrain's stiction speed, since below that the command
  // cannot move the robot anyway and the move would otherwise stall out
  // to its deadline.
  //
  // 0 (the default) keeps the legacy positional exit exactly as it was.
  float profileExitMmS() const { return profileExitMmS_; }
  void setProfileExitMmS(float mmS) {
    if (mmS >= 0.0f) profileExitMmS_ = mmS;
  }

  // [mm/s] Largest cruise whose trapezoid still holds a plateau of
  // plateauMinS_ seconds over `distanceMm`, solving
  //   (1/2)(1/aAccel + 1/aDecel) v^2 + T v - D = 0
  // for the positive root. A move commanded above this reaches its
  // peak for a single control tick and is a triangle with a corner at
  // the apex -- MEASURED gopiv 2026-09-01,
  // captures/gopiv-profile-sweep-20260901/square120.json: every 90 deg
  // pivot of that tour held its peak for exactly one telemetry sample.
  // Returns 0 when the shaping constants are not set, which callers
  // read as "no opinion".
  float plateauCruiseMmS(float distanceMm) const;

  // [mm/s] Wheel speed equivalent to maxYawRateDegS_ for a pure turn:
  // omega * trackWidth/2. The wire's cruise argument is linear mm/s,
  // so without this a pivot inherits the straight-line speed and turns
  // far faster than intended -- MEASURED gopiv 2026-09-01 (same
  // capture): cruise 300 mm/s produced 254-285 deg/s.
  float yawRateCapMmS() const;

 private:
  // |rotation| at/above this is NOT one blended segment -- pivot to the
  // new heading first, then travel straight (motion-api.md S3.3,
  // `navigator.cpp:237-240`'s measured `turn_first_angle`). 50 deg.
  static constexpr float kTurnFirstAngleRad = 0.8726646f;

  // settleToRest()'s own bound/threshold (sprint 008 ticket 004
  // extraction) -- byte-for-byte shims.cpp's former loop cap and its
  // former local `kRest`, just relocated and named. [steps] / [counts/s,
  // ~2 mm/s].
  static constexpr int kSettleMaxSteps = 12;
  static constexpr float kSettleRestCountsPerS = 25.0f;

  // One move-engine segment's targets/commands, shared by moveX()'s
  // single-segment and pivot-then-straight forms. `deadline` is fixed
  // for the whole moveX() call (set once, in moveX()/goToR()) and is
  // NOT reset across a pivot-to-straight phase transition -- one
  // `timeout` bounds the whole call, matching the wire's one field.
  struct MoveState {
    bool active = false;
    bool hasPending = false;     // a queued second (straight) phase
    float pendingDistance = 0.0f;  // [mm] phase 2's distance, if pending
    float pendingCruise = 0.0f;    // [mm/s] phase 2's cruise, if pending
    // True for exactly one serviceMove() call: phase 1 just finished and
    // kernel_.neutral() was staged this tick, but phase 2's startSegment()
    // (which stages kernel_.drive()) must NOT run until a real
    // kernel_.step() has consumed that staged neutral -- see
    // serviceMove()'s own comment at both ends of this flag's lifetime for
    // why. `step()` is always the CALLER's call (tickDrive()/updateMove()
    // both run it once per tick, before serviceMove()), never this
    // class's own, so the wait is expressed as "resume on the NEXT
    // serviceMove() call" rather than a step the engine could take itself.
    bool awaitingHandoffNeutral = false;
    float posLeft0 = 0.0f, posRight0 = 0.0f;  // [counts]
    float distTarget = 0.0f;  // [counts] mean-axis target (signed)
    float yawTarget = 0.0f;   // [counts] half-differential target (signed)
    float velCmd = 0.0f;      // [counts/s] full-rate velocity command
    float twistCmd = 0.0f;    // [counts/s] full-rate twist command
    uint32_t startMs = 0;     // [ms] for the acceleration ramp
    float cmdScale = 1.0f;    // last commanded rate scale (ramp/taper)
    uint32_t deadline = 0;    // [ms] the caller's timeout backstop

    // ---- constant-a shaping's own per-segment state, read only when
    // aAccelMmS2_/aDecelMmS2_ > 0. ----
    float cruiseMmS = 0.0f;   // [mm/s] the raw `cruise` startSegment()
                              // was called with -- the accel
                              // integrator's own reference speed (see
                              // serviceMove()'s ramp block). Unused in
                              // legacy mode.
    uint32_t lastTickMs = 0;  // [ms] the PREVIOUS serviceMove() call's
                              // nowMs(), for the accel integrator's own
                              // `dt`. Set to startMs at segment start.
                              // Unused in legacy mode.
    float accelScalePerS = 0.0f;  // [scale/s] the jerk limiter's own
                                  // state: currently commanded
                                  // acceleration, expressed in the same
                                  // cruise-fraction units as cmdScale so
                                  // the two integrate together. Read
                                  // only when jerkMmS3_ > 0.
  };

  // [ms] this engine's own notion of "now" -- see the constructor
  // comment on why a separate Clock reference is needed at all.
  uint32_t nowMs() const;

  // Post one constant-ratio segment (motion-api.md S2's wheels_x
  // reduction: left = distance - rotation*b/2, right = distance +
  // rotation*b/2), ratio-normalized to `cruise` exactly as wheelsX()
  // does, but tracked in `move_` so serviceMove() can shape/advance it
  // tick by tick instead of firing once. A zero-magnitude command or a
  // non-positive cruise leaves `move_.active` false and stages
  // kernel_.neutral() -- commands nothing NEW, but stops any motion
  // already in progress, same contract as wheelsX(). The initial
  // kernel_.drive() lease is however
  // much time remains until `move_.deadline` (already set by the
  // caller), so an abandoned move still self-neutrals at the real
  // timeout even if nothing ever calls serviceMove() again.
  void startSegment(float distance, float rotation, float cruise);

  // Queue a pivot to `pivotRotation` now, then `straightDistance` [mm]
  // straight once that pivot completes cleanly -- the shared tail of
  // moveX()'s own pivot-first split (motion-api.md S3.3) and goToR()'s
  // above-threshold bearing-pivot-then-chord split (sprint 006,
  // KERN-02): both are "pivot then straight," differing only in which
  // (rotation, distance) pair is queued. The caller must set
  // `move_.deadline` (and clear `move_.hasPending`) first -- this
  // leaves the deadline untouched, which is what keeps one `timeout`
  // spanning both phases.
  void queuePivotThenStraight(float pivotRotation, float straightDistance,
                               float cruise);

  // Clears the move-engine's own state without touching the kernel --
  // the shared tail of endMove() and of wheelsX()/wheelsV()'s "clear the
  // planner" contract.
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

  // [mm] per-wheel end-of-move overrun compensated out of every
  // rotation target -- see pivotOverrunMm()'s own comment above for the
  // measurement. 0 == uncompensated (fleet default; vevov bakes 2.2).
  float pivotOverrunMm_ = 0.0f;

  // ---- move engine state ----

  MoveState move_;

  // Moves aborted because the robot was rotating AWAY from the
  // commanded direction (serviceMove). Cumulative since construction.
  uint32_t wrongWayCount_ = 0;

  // End-of-move shaping. The defaults are the accuracy-tuned values --
  // they took turn overshoot from several degrees to under one, which
  // an OPEN-LOOP tour needs because its errors accumulate forever. They
  // are also the dominant cost in a tour's wall clock (see
  // setDistTaper()'s call site in shims.cpp for the measured trade). A
  // CLOSED-LOOP caller can afford far less -- hence settable per tour.
  float distTaper_ = 400.0f;  // [counts] ~32 mm window
  float yawTaper_ = 180.0f;   // [counts] ~15 deg window
  float distFloor_ = 0.25f;   // [1] slowest fraction of commanded
  float turnFloor_ = 0.12f;   // [1] pure turns crawl slower
  float rampMs_ = 400.0f;     // [ms] acceleration ramp

  // ---- constant-a shaping constants ----

  // [mm/s^2] UNVERIFIED pending a bench sweep -- no robot has run this
  // yet. 0.0 selects LEGACY MODE (paired with aDecelMmS2_ == 0.0):
  // startSegment()/serviceMove() keep the original `elapsed/rampMs_`
  // ramp, including the 0.25f first-tick literal, completely unchanged.
  // A nonzero value switches on a velocity-slew integrator instead
  // (`v_cmd <= v_prev + aAccelMmS2_*dt`) -- see startSegment()'s and
  // serviceMove()'s own comments in motion_engine.cpp. This replaces a
  // ramp whose effective rate MEASURED from the compiled engine
  // (captures/motion-profile-probe-20260901/profile_probe.py) scales
  // with whatever cruise is commanded instead of being a fixed mm/s^2
  // rate: 184/368/551/720/924 mm/s^2 at cruise 100/200/300/400/600 --
  // i.e. ~1.875*cruise, not a real acceleration at all.
  float aAccelMmS2_ = 0.0f;

  // [mm/s^2] UNVERIFIED pending a bench sweep. 0.0 selects LEGACY MODE
  // (paired with aAccelMmS2_ == 0.0): serviceMove()'s deceleration
  // axis-scale stays the original `remain/distTaper_` fixed-window
  // formula, unchanged. A nonzero value switches on the constant-a
  // braking-speed solve (`v_allow = sqrt(2*aDecelMmS2_*remain_mm)`)
  // instead. On the dist axis this is gated by the kinematics
  // themselves (`v_cmd^2/(2*aDecelMmS2_)`, motion_engine.cpp's
  // constantDecelWindowMm()) rather than by distTaper_: a fixed-counts
  // window is smaller than that kinematic one at any meaningful cruise
  // and left this solve unreachable above roughly 200 mm/s (see that
  // function's own comment for the measured before/after). The
  // pure-turn yaw axis is unaffected by that change and still gates on
  // the fixed yawTaper_ counts window (see serviceMove()'s own
  // comment). This replaces a fixed-window taper whose demanded
  // deceleration MEASURED from the compiled engine (same capture as
  // above) grows as v^2: 105 mm/s^2 at cruise 100 rising to
  // 5081 mm/s^2 at cruise 600, collapsing the decel phase from 26
  // control ticks to 2 -- a demand no real robot can satisfy at the
  // higher cruise values.
  float aDecelMmS2_ = 0.0f;

  // [mm/s] global speed ceiling for a future distance-chosen
  // default-cruise resolver (`v_default(D) = min(vMaxMmS_, ...)`) that
  // will read this field. UNVERIFIED pending a bench sweep -- this is
  // a documented PLACEHOLDER, not a fit constant: informed by, not
  // measured as, a field-tour result MEASURED 2026-08-31,
  // captures/fleet-tours-speed-20260831.json -- 200 mm/s gave this
  // rig's best recorded closure (tigez) on the orange-dot tour while
  // 400 mm/s doubled the mean leg miss (2.0-2.1 cm vs 3.6-4.1 cm).
  // Must never be 0: that resolver's min() depends on this being a
  // real, positive ceiling unconditionally, even before aDecelMmS2_ is
  // ever set nonzero.
  float vMaxMmS_ = 250.0f;

  // [1] fraction of a default-speed move's own leg length a future
  // `v_default(D) = min(vMaxMmS_, sqrt(2*aDecelMmS2_*brakeFrac_*D))`
  // resolver will allot to braking. UNVERIFIED pending a bench sweep --
  // placeholder within an accuracy-first recommended range (0.35-0.4).
  // Not consulted by anything yet; that future resolver is its first
  // reader.
  float brakeFrac_ = 0.375f;

  // [mm/s^3] jerk bound for the second-order shaper. 0 selects the
  // first-order behaviour (bounded acceleration only), where the
  // commanded acceleration steps discontinuously at the accel->decel
  // handover. UNVERIFIED pending a bench sweep; simulated at 4000 in
  // captures/gopiv-profile-sweep-20260901/. Note a jerk phase lasts
  // aAccel/jerk seconds, so at the kernel's 24 ms tick a value much
  // above ~10000 rounds nothing a control cycle can resolve.
  float jerkMmS3_ = 0.0f;

  // [s] minimum cruise plateau plateauCruiseMmS() solves for. 0
  // disables the derate. Wants to be at least twice a jerk transition
  // (2*aAccel/jerk) so the accel and decel roundings cannot meet and
  // eat the plateau they were sized to protect.
  float plateauMinS_ = 0.0f;

  // [deg/s] ceiling on pure-turn angular rate. 0 disables the cap and
  // is the shipping default: a pivot then inherits the linear cruise
  // exactly as before. Defaulting this ACTIVE was tried and reverted --
  // it silently rescaled every legacy pure turn and broke the pinned
  // yaw-taper regression, which is precisely the behaviour that test
  // exists to protect. A caller opts in with SET max_yaw_rate 90.
  float maxYawRateDegS_ = 0.0f;

  // [mm/s] see setProfileExitMmS(). 0 selects the legacy positional
  // exit; paired with aDecelMmS2_ > 0 it selects profile completion.
  float profileExitMmS_ = 0.0f;
};

}  // namespace diffDrive
