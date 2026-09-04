// wire_adapter.h -- diffDrive::WireAdapter: the concrete Wire::Adapter
// for this robot. All six motion verbs have real effect. STOP/ESTOP/
// GET/SET and the engine* calls reach shims.cpp via same-package
// forward declarations (shims.cpp has no header of its own --
// wire_adapter.cpp's own forward-declaration block must stay
// signature-compatible with shims.cpp's real definitions). This class
// holds no reference of its own to a kernel or motion engine.
// Host-portable by construction: no pxt.h, no CODAL type, anywhere in
// this file or wire_adapter.cpp -- host tests supply their own
// definitions of the forward-declared shims.cpp functions. Identity
// fields are borrowed pointers (see the constructor's own doc comment).
// now() is backed by a NowMsFn supplied at composition time; with none
// supplied (nullptr), now() stays 0 and hasLiveMotionObligation()
// always answers false. Every ACCEPTED motion verb arms a
// motion-obligation deadline (`duration` for the V-forms, `timeout` for
// the X-forms/GO_TO-forms -- a conservative overestimate, harmless)
// that protocol.cpp's fiber loop polls to keep ticking the kernel. That
// obligation clears on whichever comes first: an explicit STOP/ESTOP, or
// (sprint 016 ticket 003) the pending motion being lazily discovered to
// have already resolved on its own -- see resolvePendingIfDue()'s own
// doc comment below.
//
// Sprint 004 ticket 004: telemetry projection joins this class's
// existing session/motion/safety/config seams. `buildSnapshot()`
// (returns a `const Wire::Snapshot&` into a MEMBER, mirroring
// radio-robot-lib's own DiffDriveAdapter::buildSnapshot()) reads live
// state through FIVE more forward-declared shims.cpp reads (poseX/
// poseY/poseHeading/otosGet/wheelSpeed -- wire_adapter.cpp's own
// forward-declaration block documents three real debugging hazards
// there: poseX/Y/heading MUTATE odometry and that is load-bearing,
// otosGet()'s first two fields are 0.1 mm while the third is already
// centidegrees, and otosGet() must NEVER be backed by otosRead() on
// this fiber). `telemetryEnabled()` is `mode_ != Wire::TlmMode::kOff` --
// protocol.cpp calls it once per emission tick to decide whether to
// build a Snapshot at all. `computeFlags()` (wire_adapter.cpp,
// anonymous namespace) is now a single function called from BOTH
// status() and buildSnapshot(), so STATUS's `flags=` and the telemetry
// `flags` column share one source and cannot drift; the same is true of
// `i2cf` via the shared diagValue(8) accessor. Neither buildSnapshot()
// nor telemetryEnabled() is part of Wire::Adapter's own interface --
// protocol.cpp calls them directly on this concrete class, exactly once
// per tick, and hands the SAME returned Snapshot reference to both
// WireHandler instances (see sprint.md's own Design Rationale for why
// once-per-tick, not once-per-handler).
#pragma once

#include <cstddef>
#include <cstdint>

#include "../core/motion_owner.h"
#include "wire_handler.h"

namespace diffDrive {

class WireAdapter : public Wire::Adapter {
 public:
  // [ms] WHEELS_V/MOVE_V duration ceiling (motion-api.md S1): duration
  // IS the lease -- a dead host cannot mean a runaway. Enforced here;
  // the handler holds no bounds table.
  static constexpr uint32_t kWheelsVDurationCeiling = 5000;  // [ms]

  // WIRE-08 (code review 2026-08-23): the widest magnitude this class
  // will cast from float to int at the wire boundary, in either
  // direction. parseInt32()/parseFloatField() (wire_handler.cpp) admit
  // the full wire grammar -- any int32, any finite float -- with no
  // ceiling of their own, but `static_cast<float>(x)` then
  // `static_cast<int>(...)` back is UB whenever the intermediate float
  // rounds outside int32's representable range. That happens well
  // *before* int32's own limit: float's 24-bit mantissa cannot hold
  // every integer near 2^31, so `static_cast<float>(2147483647)`
  // itself rounds UP to 2147483648.0f (2^31, one past INT32_MAX) --
  // casting that back to int is UB, benign-saturating on the Cortex-M
  // target's VCVT but INT32_MIN on the x86 host harness's cvttss2si
  // (this project's own `tests/host/`), i.e. host and target disagree
  // in SIGN for wire values in [2147483584, 2147483647]. This ceiling
  // is chosen with ~147M of headroom below INT32_MAX specifically so
  // that any float within [-kWireBoundaryCastCeiling,
  // kWireBoundaryCastCeiling] truncates to an IN-RANGE int32 on any
  // conforming compiler -- the C++ standard defines float->int
  // truncation (toward zero) whenever the truncated value is
  // representable, so a value that never approaches the boundary can
  // never hit the platform-dependent UB, on either the C++20 host or
  // the C++11 target. Used by onWheelsV() (left/right, [mm/s]) and
  // onSet() (the field's own value * 1000.0f, before std::lround) --
  // see each one's own comment (wire_adapter.cpp) for the exact
  // refusal. Deliberately NOT a claim about sane physical units (no
  // wheel needs to move at 2,000,000 m/s) -- that is sprint 008's own
  // constant-unification scope; this constant exists purely to keep
  // the cast itself well-defined and platform-identical.
  static constexpr float kWireBoundaryCastCeiling = 2000000000.0f;

  // Plain C function pointer, deliberately not std::function -- this
  // file must stay free of anything that could drag in CODAL or
  // heap-allocating machinery. Returns milliseconds on whatever clock
  // the composition root chose; this class only ever computes
  // DIFFERENCES against it (see hasLiveMotionObligation()), so its
  // epoch is unspecified and irrelevant.
  using NowMsFn = uint32_t (*)();

  // `identity`'s own pointer fields are borrowed (Wire::Identity's own
  // doc comment, wire_handler.h): the CALLER's identity strings must
  // outlive this adapter. Copied by value here (copies the pointers,
  // not the strings they point to). `now`, if supplied, must remain
  // valid for this adapter's whole lifetime -- in practice a free
  // function or a static member function, never a capturing closure
  // (the type above cannot express one). Defaults to nullptr for every
  // caller with no real clock to offer (every existing host test).
  explicit WireAdapter(const Wire::Identity& identity,
                       NowMsFn now = nullptr);

  // ---- Wire::Adapter: session ----
  void identity(Wire::Identity& out) const override;
  uint32_t now() const override;
  void status(Wire::StatusFields& out) const override;

  // Placeholder-at-construction, real identity later -- a CODAL identity
  // read is unsafe before uBit.init() (protocol.cpp calls this from its
  // fiber body). Same borrowed-pointer contract as the constructor.
  void setIdentity(const Wire::Identity& identity);

  // Mirrors Protocol's own single MotionOwner value: kNone (idle),
  // kWire (this adapter's own live motion obligation), kJob (a
  // dispatched RUN job), or kBlock (the block program's own fiber
  // holding a move directly). That field deliberately lives in
  // Protocol, not here (only Protocol can see a wire request, a
  // dispatched job, AND a block-motion call, all three) -- this setter
  // is the one seam Protocol uses to mirror it here, so the six motion
  // verb handlers below can refuse (kBusy) a wire motion arriving while
  // anything but kWire/kNone holds the drivetrain, instead of silently
  // overwriting or racing its move. Not part of Wire::Adapter's own
  // interface -- called only from protocol.cpp, the same "calls a
  // concrete-class method directly" convention setIdentity() above and
  // buildSnapshot()/telemetryEnabled() below already use.
  void setExternalOwner(MotionOwner owner);

  // ---- Wire::Adapter: motion ----

  // WHEELS_V: forwards to setWheelsTimed() (velocity=(l+r)/2,
  // twist=(r-l)/2 CCW+, duration = lease); only the duration ceiling is
  // enforced here -- no kernel refusal is observable (setWheelsTimed
  // returns void). WIRE-08: `left`/`right` are also refused (kRange)
  // outside +-kWireBoundaryCastCeiling before either reaches
  // setWheelsTimed()'s own static_cast<int> -- see that constant's own
  // doc comment above for the float-round-trip UB this closes.
  Wire::Result onWheelsV(float left, float right, uint32_t duration,
                         uint32_t id) override;

  // WHEELS_X: wire fields already mm/mm/mm-per-s/ms; `cruise` < 0
  // refused kRange (a ceiling has no sign); `cruise` == 0 -> configured
  // default via engineDefaultCruise(), refused kRange if that too is
  // unconfigured.
  Wire::Result onWheelsX(float left, float right, float cruise,
                         uint32_t timeout, uint32_t id) override;

  // MOVE_X: same cruise <0/==0 handling as onWheelsX() above. `rotation`
  // is THE mrad->rad seam (wire_adapter.cpp's mradToRad(), tested both
  // signs).
  Wire::Result onMoveX(float distance, float rotation, float cruise,
                       uint32_t timeout, uint32_t id) override;

  // MOVE_V: plain wheelsV reduction; `omega` through the same mrad seam
  // as MOVE_X's `rotation`; `duration` shares kWheelsVDurationCeiling.
  Wire::Result onMoveV(float v_x, float omega, uint32_t duration,
                       uint32_t id) override;

  // GO_TO_R: `speed` plays cruise's role (same <0/==0 handling); `arrive`
  // passes through unused (single-shot reduction); `timeout` is moveX's
  // backstop.
  Wire::Result onGoToR(float x, float y, float speed, float arrive,
                       uint32_t timeout, uint32_t id) override;

  // GO_TO_W: the world-frame counterpart, forwarded onto
  // MotionEngine::goToW() via shims.cpp's engineGoToW(). Same `speed`
  // <0/==0 handling as onGoToR() above. engineGoToW() selects its own
  // PoseSource (OtosPort when connected(), else the encoder-odometry
  // fallback -- motion-api.md S3.6) and always dispatches, so its bool
  // return is now unconditionally true; the `!engineGoToW(...)` refusal
  // below is dead code, kept only because the bool CONTRACT ("was a live
  // pose actually available") is unchanged. The return still does not
  // distinguish OTOS from drifting encoder odometry -- a caller that
  // needs to know reads STATUS's `otos=` flag first (motion-api.md
  // S3.6's own documented caveat).
  Wire::Result onGoToW(float x, float y, float speed, float arrive,
                       uint32_t timeout, uint32_t id) override;

  // ---- Wire::Adapter: safety ----
  void onEstop() override;                            // -> estopAll()
  Wire::Result onStop(bool immediate, uint32_t id) override;  // -> stopAll()

  // ---- Wire::Adapter: configuration -- a small field-name table
  // replacing the old ordinal verbs one-for-one; see wire_adapter.cpp's
  // kFields for the name<->ordinal mapping onto shims.cpp's
  // setKernelValue()/getConfigValue() field numbers. onSet() additionally
  // refuses (kRange) when `value * 1000.0f` -- the exact product passed
  // to std::lround() -- falls outside +-kWireBoundaryCastCeiling
  // (WIRE-08): an unclamped `SET pid_kp 3000000` would overflow `long`'s
  // 32-bit range before lround() ever runs (see kWireBoundaryCastCeiling's
  // own doc comment above). ----
  bool onGet(const char* name, float& out) const override;
  Wire::Result onSet(const char* name, float value, uint32_t id) override;
  size_t fieldCount() const override;
  const char* fieldName(size_t index) const override;

  // ---- Wire::Adapter: telemetry ----

  // TLM <mode> #<id>: sets the persisted subscription mode_ (protocol.md
  // S6.1), with two decisions pinned by sprint 008 ticket 005 (closing
  // tlm-auto-buffer-column-set-undefined.md -- previously kAuto/kBuffer
  // silently fell through to POSE's column set with no decision recorded
  // anywhere):
  //   - TlmMode::kAuto is a documented ALIAS for TlmMode::kPose -- same
  //     12 columns, same cadence, matching the pre-existing de facto
  //     behavior exactly (a zero-risk documentation-and-test fix, not a
  //     new feature).
  //   - TlmMode::kBuffer REFUSES (kUnimplemented, wire err 6) at this
  //     verb, before mode_ is ever touched -- no buffering mechanism
  //     exists anywhere in this codebase to give "buffer" real, narrower
  //     semantics yet, and answering err is more honest than emitting a
  //     column set nobody specified. A MERITS rejection (decodeTlm()
  //     already accepts "BUFFER" as well-formed), not a decode failure --
  //     same "state left unchanged on refusal" convention
  //     wire_handler.cpp's clampMotionTimeout()-based refusals already
  //     established for the six motion verbs (sprint 008 ticket 001).
  // TlmMode::kNow remains the pre-existing one-shot exception (never
  // stored into mode_) -- see wire_adapter.cpp's own onTlm() comment.
  // It now ALSO arms the one-shot flag consumeOneShotTelemetry() below
  // reads, giving TLM NOW a real emitted frame instead of an ack that
  // produces nothing.
  Wire::Result onTlm(Wire::TlmMode mode) override;

  // TLM NOW's one-shot delivery: true (and clears the flag) exactly
  // once for each accepted `TLM NOW #<id>`, regardless of whether a
  // subscription is even active -- see this class's own onTlm() for
  // where the flag is armed. Deliberately NOT a const query-then-clear
  // pair: whichever caller sees `true` back from this one call is the
  // one obligated to build and emit the frame, so a request can never
  // be observed by more than one caller and served twice (or by none,
  // if a caller only ever peeked). protocol.cpp calls this once per
  // fiber pass, independent of the periodic emission timer, so a
  // one-shot request is served on the very next pass rather than
  // waiting for that timer to elapse.
  bool consumeOneShotTelemetry();

  // ---- sprint 004 ticket 004: telemetry projection -- NOT part of
  // Wire::Adapter's own interface (see this file's header comment).
  // protocol.cpp calls these two directly. ----

  // Builds and returns this tick's telemetry Snapshot, scaled to wire
  // units, from live state reached through five forward-declared
  // shims.cpp reads (wire_adapter.cpp's own forward-declaration block).
  // Advances and wraps this adapter's own `seq_` ((seq_+1) & 0x7F,
  // protocol.md S6.2) -- WireHandler has no opinion on seq, it only
  // prints whatever this method hands it. POSE's 12 columns (`seq now
  // flags x y h ox oy oh vl vr i2cf`) are always present; FULL's 8 more
  // (`cyc posl posr dutl dutr lexc wrng cycovr`) are added only when
  // `mode_ == Wire::TlmMode::kFull`. Every other mode this adapter can
  // actually be subscribed in when this is called gets POSE's 12 and
  // stops there: kPose itself, and kAuto -- a documented ALIAS for
  // kPose (sprint 008 ticket 005: same 12 columns, same cadence, no
  // decision left unrecorded). kOff never reaches this call at all
  // (telemetryEnabled() gates it). kBuffer can never reach mode_ in the
  // first place -- onTlm() refuses it (kUnimplemented) before
  // assignment, so no telemetry frame is ever built for it. Returns a
  // reference into a MEMBER (`snapshot_`/`columns_`), not a temporary --
  // valid only until the next buildSnapshot() call, matching
  // Wire::Snapshot's own "borrowed for one emitTelemetry() call" doc
  // comment (wire_handler.h). NOT const: mutates `seq_` and the
  // snapshot/column members, and poseX()/poseY()/poseHeading() below
  // mutate live odometry too.
  const Wire::Snapshot& buildSnapshot();

  // True iff mode_ != Wire::TlmMode::kOff -- protocol.cpp's own
  // periodic-emission block reads this to decide whether to call
  // buildSnapshot() at all this tick (sprint.md's Design Rationale: no
  // Snapshot is ever built for a session with no subscriber).
  bool telemetryEnabled() const;

  // True iff a motion is genuinely outstanding, as far as this class has
  // NOTICED: the most recently ACCEPTED motion verb's own window has not
  // elapsed, AND nothing has since resolved it to a terminal reason.
  // Sprint 016 ticket 003 (closing wire-motion-obligation-never-clears.md):
  // resolvePendingIfDue()/forceResolvePending() now clear the underlying
  // `motionObligationActive_` themselves as soon as they commit a
  // resolution -- not only on an explicit STOP/ESTOP as before -- so a
  // goal-directed move (MOVE_X/GO_TO_R/GO_TO_W) that reaches its own goal
  // long before its declared `timeout` stops reading "live" as soon as
  // something next polls lastDone()/lastDoneReason(), instead of staying
  // true until that full timeout elapses. This is still a LAZY signal --
  // see resolvePendingIfDue()'s own doc comment below for exactly which
  // calls trigger the check -- not a push notification the instant the
  // engine itself finishes. Always false with no clock wired.
  //
  // This method now runs that same resolution itself, first, before
  // reading the deadline -- previously only lastDone()/lastDoneReason()
  // (reached from replyAck()/replyNack()/STATUS) did, so a caller that
  // never happens to poll one of those still saw a motion read "live"
  // for the rest of its ORIGINAL declared window even though the
  // kernel had been idle since it actually finished. protocol.cpp's
  // fiber loop polls only this accessor every pass, so that was exactly
  // the gap: a caller with no other reason to touch lastDone() got no
  // self-clear at all. See motionObligationDeadlineLive() below (the
  // private, non-resolving half of what this method used to do on its
  // own) for why the internal resolution path reads the raw deadline
  // directly instead of calling back into this method -- this method
  // calls resolvePendingIfDue(), which calls resolvePendingReason(),
  // which needs the raw deadline read too; routing that through THIS
  // method again would recurse forever.
  bool hasLiveMotionObligation() const;

  // ---- sprint 005 ticket 004: a REAL motion-completion signal (closes
  // wire-motion-completion-signal.md, R-23) -- superseding the "no
  // motion queue and no completion event on this adapter yet" decision
  // sprint 003 ticket 012 recorded at this exact spot (that comment's
  // own closing line called this "a natural candidate to revisit once a
  // real use case needs lastDone()/lastDoneReason() to mean something"
  // -- this sprint's closed-loop bench tooling is that use case).
  //
  // lastDone()/lastDoneReason() now report the accepted `id` and
  // Wire::DoneReason of whichever motion verb most recently reached a
  // terminal state, resolved FRESH on every call (S8.8 -- no cached
  // copy on WireHandler; this class's OWN state below is not a "cache"
  // in that sense, see the field comments). Two lease-style-resolvable
  // groups:
  //   - WHEELS_V/WHEELS_X/MOVE_V resolve done-vs-timeout-vs-superseded
  //     entirely from this class's OWN existing
  //     motionObligationActive_/motionObligationDeadline_ bookkeeping
  //     (already present for hasLiveMotionObligation()) -- no new
  //     dependency.
  //   - MOVE_X/GO_TO_R/GO_TO_W additionally need to know whether the
  //     underlying MotionEngine move is still active when the lease
  //     deadline is reached, to distinguish "reached its own stop
  //     condition early" (kStop) from "ran out the clock" (kTimeout).
  //     This is the ONE genuinely new read: engineMoveActive()
  //     (wire_adapter.cpp's own forward declaration), a thin, read-only
  //     bridge matching engineWheelsX()'s own convention exactly. This
  //     class still holds NO stored reference to MotionEngine/Rig --
  //     that boundary (this file's own header comment above) is
  //     unchanged.
  // `stall`/`estop` need NO new plumbing at all: both already reach
  // this class through the SAME diagValue()/computeFlags() path
  // STATUS's `flags=` and telemetry's `flags` column already use
  // (`stall_halted`/`estopped` are two of its eight diagnostic
  // booleans) -- see wire_adapter.cpp's resolvePendingReason().
  //
  // See wire_adapter.cpp's resolvePendingReason() (the pure
  // resolution logic), resolvePendingIfDue() (the lazy commit these two
  // accessors share), forceResolvePending() (the two edges -- a
  // supersede, an explicit STOP -- that must be captured AT THE MOMENT
  // they happen, from the six onXxx() handlers and onStop()), and
  // armPendingMotion() (what every accepted motion verb arms).
  uint32_t lastDone() const override;
  Wire::DoneReason lastDoneReason() const override;

  // No registration table -- this project's actual by-name test trigger
  // is protocol.cpp's own MessageBus RUN bridge (runSlots_/handleRun()),
  // a CODAL-specific mechanism this host-portable class must never
  // touch. Every RUN is ERR_UNKNOWN here, the same wire outcome any name
  // a real registration table would not recognize (protocol.md S6.3).
  Wire::Result onRun(const char* name, const char* const* argv, size_t argc,
                     char* result, size_t resultCapacity,
                     bool& hasResult) override;

 private:
  Wire::Identity identity_;
  Wire::TlmMode mode_ = Wire::TlmMode::kOff;

  // Set by onTlm() on an accepted `TLM NOW`, never by anything else;
  // cleared by consumeOneShotTelemetry() (above), never by anything
  // else. Deliberately independent of mode_ -- see onTlm()'s own
  // comment for why a one-shot request must not disturb whatever
  // subscription (or lack of one) is already persisted.
  bool oneShotTelemetryDue_ = false;

  // Set only via setExternalOwner() above -- see that method's own
  // comment. Compared against kNone everywhere below: this class only
  // ever needs "is something else holding the drivetrain right now",
  // never which of kJob/kBlock it is.
  MotionOwner externalOwner_ = MotionOwner::kNone;

  // ---- real clock + motion-obligation state ----
  NowMsFn now_ = nullptr;
  // `mutable`, same reason as the pendingActive_ family below: sprint 016
  // ticket 003 has resolvePendingIfDue() (a const method, called from the
  // const accessors lastDone()/lastDoneReason()) clear this the moment it
  // lazily discovers a pending motion has resolved, not only on an
  // explicit STOP/ESTOP as before.
  mutable bool motionObligationActive_ = false;
  uint32_t motionObligationDeadline_ = 0;  // [ms], now_'s own scale

  // ---- sprint 005 ticket 004: motion-completion tracking (S8.8) -----
  // `pendingActive_`/`pendingId_`/`pendingGoalDirected_` track the most
  // recently ACCEPTED motion verb not yet resolved to a terminal
  // reason; `lastDoneId_`/`lastDoneReason_` are the most recently
  // RESOLVED (committed) pair -- what lastDone()/lastDoneReason()
  // actually return. All five are `mutable`: resolvePendingIfDue()
  // (called from both of those const accessors) commits a
  // lazily-discovered terminal reason from inside a const call, and it
  // must stay committed afterward even if the diag/engine state that
  // first revealed it later changes (a stall latch cleared via
  // `stall_clear`, an estop cleared) -- this class reports what a
  // motion ACTUALLY ended with, not the live diagnostic state at read
  // time. Only armed/force-resolved with a real clock wired
  // (now_ != nullptr, matching motionObligationActive_'s own
  // gating) -- with no clock, lastDone()/lastDoneReason() keep
  // reporting the honest 0/kNone default forever, same as before this
  // ticket. See wire_adapter.cpp for the full resolution logic.
  mutable bool pendingActive_ = false;
  mutable uint32_t pendingId_ = 0;
  mutable bool pendingGoalDirected_ = false;
  mutable uint32_t lastDoneId_ = 0;
  mutable Wire::DoneReason lastDoneReason_ = Wire::DoneReason::kNone;

  // Pure, non-resolving read of the deadline itself: true iff
  // motionObligationActive_ is set and now_() has not yet reached
  // motionObligationDeadline_. This is what hasLiveMotionObligation()
  // (public, above) used to do as its entire body; it is now split out
  // so resolvePendingReason() below can read the SAME raw check
  // without going back through hasLiveMotionObligation() itself --
  // that method now resolves FIRST, and resolvePendingReason() runs
  // AS PART OF that resolution, so the two must never call each other.
  bool motionObligationDeadlineLive() const;

  // Pure function of currently observable state (diagValue()'s estop/
  // stall flags, motionObligationDeadlineLive() above, and -- for a
  // goal-directed pending motion only -- engineMoveActive()); never
  // mutates anything. Returns kNone whenever nothing is pending OR the
  // pending motion has not yet reached a terminal state -- callers
  // distinguish those two cases via pendingActive_ themselves.
  Wire::DoneReason resolvePendingReason() const;

  // Commits resolvePendingReason()'s result into lastDoneId_/
  // lastDoneReason_ (and clears pendingActive_) iff it is no longer
  // kNone; a no-op otherwise. Called from both lastDone() and
  // lastDoneReason() so polling either one alone is enough to notice a
  // newly terminal pending motion -- S8.8's "read fresh" contract.
  // Sprint 016 ticket 003: also clears `motionObligationActive_` on that
  // same commit -- this is the natural-completion clearing point that
  // was missing before (only onEstop()/onStop() cleared it), the actual
  // gap wire-motion-obligation-never-clears.md names.
  void resolvePendingIfDue() const;

  // Force-resolves a still-pending motion RIGHT NOW, at a call site
  // that itself knows a reason (a supersede in one of the six onXxx()
  // handlers, or an explicit STOP in onStop()) -- resolvePendingReason()
  // still gets first refusal, so an already-stalled/estopped pending
  // motion keeps THAT more specific reason instead of being overwritten
  // by the caller's forced one. A no-op if nothing is pending. Sprint 016
  // ticket 003: also clears `motionObligationActive_` on that same
  // commit, covering the "a later verb supersedes a still-live earlier
  // one" (kAborted) path -- this runs BEFORE the *new* verb re-arms
  // `motionObligationActive_ = true` a few lines later in the same
  // onXxx() handler, so ordering stays correct. onStop()'s own explicit
  // `motionObligationActive_ = false;` (wire_adapter.cpp) is now
  // redundant with this for the STOP path specifically, but is left in
  // place -- harmless (idempotent), and onEstop() still needs its own
  // explicit clear regardless since it deliberately does not call this
  // method at all (see onEstop()'s own comment).
  void forceResolvePending(Wire::DoneReason forcedReason);

  // Arms tracking for a freshly ACCEPTED motion verb -- call AFTER any
  // supersede has already been resolved (forceResolvePending(kAborted))
  // and AFTER motionObligationActive_/motionObligationDeadline_ are
  // set. `goalDirected`: true for MOVE_X/GO_TO_R/GO_TO_W (needs
  // engineMoveActive() to resolve), false for WHEELS_V/WHEELS_X/MOVE_V
  // (resolves purely from the deadline -- see resolvePendingReason()).
  void armPendingMotion(uint32_t id, bool goalDirected);

  // ---- sprint 004 ticket 004: telemetry projection state -----------
  // `seq_` wraps at 0x7F (protocol.md S6.2); buildSnapshot() advances it
  // BEFORE building each frame, so the first-ever frame reports seq 1,
  // not 0. `columns_`/`snapshot_` are members, not locals, so
  // buildSnapshot() can return a reference into them that stays valid
  // until the NEXT call (same "member, not a stack local" rationale
  // wire_handler.h's own emitBuf_ documents) -- sized for the widest
  // documented set (POSE's 12 plus FULL's 8 more, sprint.md's Phase B).
  uint8_t seq_ = 0;
  static constexpr size_t kMaxSnapshotColumns = 20;
  Wire::Column columns_[kMaxSnapshotColumns];
  Wire::Snapshot snapshot_;
};

}  // namespace diffDrive
