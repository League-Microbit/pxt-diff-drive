// wire_adapter.h -- diffDrive::WireAdapter: the concrete Wire::Adapter
// for this robot. All six motion verbs have real effect. STOP/ESTOP/
// GET/SET and the engine* calls reach shims.cpp via same-package
// forward declarations (shims.cpp has no header of its own --
// wire_adapter.cpp's forward-declaration block must stay
// signature-compatible with shims.cpp's real definitions). This class
// holds no reference of its own to a kernel or motion engine.
// Host-portable by construction: no pxt.h, no CODAL type, anywhere in
// this file or wire_adapter.cpp -- host tests supply their own
// definitions of the forward-declared shims.cpp functions. Identity
// fields are borrowed pointers (see the constructor's own doc comment).
//
// now() is backed by a NowMsFn supplied at composition time; with none
// supplied (nullptr), now() stays 0 and hasLiveMotionObligation()
// always answers false. Every ACCEPTED motion verb arms a
// motion-obligation deadline (`duration` for the V-forms, `timeout` for
// the X-forms/GO_TO-forms -- a conservative overestimate, harmless)
// that protocol.cpp's fiber loop polls to keep ticking the kernel. That
// obligation clears on whichever comes first: an explicit STOP/ESTOP,
// or the pending motion being lazily discovered to have already
// resolved -- see resolvePendingIfDue()'s own doc comment below.
//
// Telemetry projection lives here too. `buildSnapshot()` reads live
// state through five more forward-declared shims.cpp reads (poseX/
// poseY/poseHeading/otosGet/wheelSpeed -- wire_adapter.cpp's own
// forward-declaration block documents three real hazards there: poseX/
// Y/heading MUTATE odometry and that is load-bearing, otosGet()'s first
// two fields are 0.1 mm while the third is already centidegrees, and
// otosGet() must NEVER be backed by otosRead() on this fiber).
// `computeFlags()` is called from BOTH status() and buildSnapshot(), so
// STATUS's `flags=` and the telemetry `flags` column share one source
// and cannot drift; same for `i2cf` via diagValue(8). Neither
// buildSnapshot() nor telemetryEnabled() is part of Wire::Adapter's
// interface -- protocol.cpp calls them directly, once per tick, and
// hands the SAME Snapshot to both WireHandler instances.
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

  // The widest magnitude this class will cast from float to int at the
  // wire boundary, in either direction. The wire grammar itself admits
  // any int32 and any finite float, but `static_cast<float>(x)` then
  // `static_cast<int>(...)` back is UB whenever the intermediate float
  // rounds outside int32's representable range -- which happens well
  // *before* int32's own limit, since float's 24-bit mantissa cannot
  // hold every integer near 2^31: `static_cast<float>(2147483647)`
  // rounds UP to 2147483648.0f, and casting that back is
  // benign-saturating on the Cortex-M target's VCVT but INT32_MIN on
  // the x86 host harness's cvttss2si -- host and target disagree in
  // SIGN for wire values in [2147483584, 2147483647]. ~147M of headroom
  // below INT32_MAX keeps every in-range float truncating to an
  // in-range int32 on both. Used by onWheelsV() (left/right, [mm/s])
  // and onSet() (value * 1000.0f, before std::lround). NOT a claim
  // about sane physical units.
  static constexpr float kWireBoundaryCastCeiling = 2000000000.0f;

  // Plain C function pointer, deliberately not std::function -- this
  // file must stay free of anything that could drag in CODAL or
  // heap-allocating machinery. Returns milliseconds on whatever clock
  // the composition root chose; this class only ever computes
  // DIFFERENCES against it, so its epoch is unspecified and irrelevant.
  using NowMsFn = uint32_t (*)();

  // `identity`'s pointer fields are borrowed (Wire::Identity's own doc
  // comment): the CALLER's identity strings must outlive this adapter.
  // Copied by value here (the pointers, not the strings). `now`, if
  // supplied, must remain valid for this adapter's whole lifetime -- in
  // practice a free or static member function, never a capturing
  // closure (the type above cannot express one).
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
  // holding a move). That field deliberately lives in Protocol, not
  // here -- only Protocol can see all three -- and this setter is the
  // one seam Protocol uses to mirror it, so the six motion verb
  // handlers below can refuse (kBusy) a wire motion arriving while
  // anything but kWire/kNone holds the drivetrain. Not part of
  // Wire::Adapter's interface; called only from protocol.cpp.
  void setExternalOwner(MotionOwner owner);

  // ---- Wire::Adapter: motion ----

  // WHEELS_V: forwards to setWheelsTimed() (velocity=(l+r)/2,
  // twist=(r-l)/2 CCW+, duration = lease); only the duration ceiling is
  // enforced here -- no kernel refusal is observable (setWheelsTimed
  // returns void). `left`/`right` are also refused (kRange) outside
  // +-kWireBoundaryCastCeiling before either reaches setWheelsTimed()'s
  // own static_cast<int>.
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
  // return is unconditionally true and the `!engineGoToW(...)` refusal
  // below is dead code, kept only because the bool CONTRACT ("was a
  // live pose actually available") is unchanged. It does not
  // distinguish OTOS from drifting encoder odometry -- a caller that
  // needs to know reads STATUS's `otos=` flag first.
  Wire::Result onGoToW(float x, float y, float speed, float arrive,
                       uint32_t timeout, uint32_t id) override;

  // ---- Wire::Adapter: safety ----
  void onEstop() override;                            // -> estopAll()
  Wire::Result onStop(bool immediate, uint32_t id) override;  // -> stopAll()

  // ---- Wire::Adapter: configuration -- a field-name table; see
  // comms/config_fields.h for the name<->ordinal mapping onto
  // shims.cpp's setKernelValue()/getConfigValue() field numbers.
  // onSet() additionally refuses (kRange) when `value * 1000.0f` -- the
  // exact product passed to std::lround() -- falls outside
  // +-kWireBoundaryCastCeiling: an unclamped `SET pid_kp 3000000` would
  // overflow `long`'s 32-bit range before lround() ever runs. ----
  Wire::Result onGet(const char* name, float& out) const override;
  Wire::Result onSet(const char* name, float value, uint32_t id) override;
  size_t fieldCount() const override;
  const char* fieldName(size_t index) const override;

  // ---- Wire::Adapter: telemetry ----

  // TLM <mode> #<id>: sets the persisted subscription mode_ (protocol.md
  // S6.1). Two modes are special:
  //   - TlmMode::kAuto is a documented ALIAS for TlmMode::kPose -- same
  //     12 columns, same cadence.
  //   - TlmMode::kBuffer REFUSES (kUnimplemented, wire err 6) at this
  //     verb, before mode_ is ever touched: no buffering mechanism
  //     exists to give "buffer" real semantics, and answering err is
  //     more honest than emitting a column set nobody specified. A
  //     MERITS rejection (decodeTlm() already accepts "BUFFER" as
  //     well-formed), not a decode failure -- same "state left
  //     unchanged on refusal" convention as the six motion verbs.
  // TlmMode::kNow is the one-shot exception, never stored into mode_;
  // it arms the flag consumeOneShotTelemetry() below reads.
  Wire::Result onTlm(Wire::TlmMode mode) override;

  // TLM NOW's one-shot delivery: true (and clears the flag) exactly
  // once per accepted `TLM NOW #<id>`, whether or not a subscription is
  // active. Deliberately NOT a const query-then-clear pair: whichever
  // caller sees `true` is the one obligated to build and emit the
  // frame, so a request can never be served twice (or by none, if a
  // caller only peeked). protocol.cpp calls this once per fiber pass,
  // independent of the periodic emission timer.
  bool consumeOneShotTelemetry();

  // ---- telemetry projection -- NOT part of Wire::Adapter's own
  // interface (see this file's header comment). protocol.cpp calls
  // these two directly. ----

  // Builds and returns this tick's telemetry Snapshot, scaled to wire
  // units. Advances and wraps `seq_` ((seq_+1) & 0x7F, protocol.md
  // S6.2) -- WireHandler only prints what this hands it. POSE's 12
  // columns (`seq now flags x y h ox oy oh vl vr i2cf`) are always
  // present; FULL's 8 more (`cyc posl posr dutl dutr lexc wrng cycovr`)
  // only when `mode_ == Wire::TlmMode::kFull`. kPose and kAuto (its
  // alias) stop at the 12; kOff never reaches this call
  // (telemetryEnabled() gates it) and kBuffer can never reach mode_.
  // Returns a reference into a MEMBER, valid only until the next call.
  // NOT const: mutates `seq_`, the snapshot members, and -- through
  // poseX()/poseY()/poseHeading() -- live odometry.
  const Wire::Snapshot& buildSnapshot();

  // True iff mode_ != Wire::TlmMode::kOff -- protocol.cpp's periodic
  // emission block reads this to decide whether to call buildSnapshot()
  // at all this tick.
  bool telemetryEnabled() const;

  // True iff a motion is genuinely outstanding, as far as this class
  // has NOTICED: the most recently ACCEPTED motion verb's own window
  // has not elapsed, AND nothing has since resolved it to a terminal
  // reason. A LAZY signal, not a push notification the instant the
  // engine finishes; always false with no clock wired. This method runs
  // the resolution itself, FIRST, before reading the deadline, because
  // protocol.cpp's fiber loop polls only this accessor -- without that,
  // a goal-directed move reaching its goal early would keep reading
  // "live" for its whole declared timeout. The resolution path reads
  // motionObligationDeadlineLive() (below) rather than calling back
  // into this method, which would recurse forever.
  bool hasLiveMotionObligation() const;

  // ---- the motion-completion signal ----
  //
  // lastDone()/lastDoneReason() report the accepted `id` and
  // Wire::DoneReason of whichever motion verb most recently reached a
  // terminal state, resolved FRESH on every call (S8.8). Two groups:
  //   - WHEELS_V/WHEELS_X/MOVE_V resolve done-vs-timeout-vs-superseded
  //     entirely from this class's own motionObligationActive_/
  //     motionObligationDeadline_ bookkeeping.
  //   - MOVE_X/GO_TO_R/GO_TO_W additionally need engineMoveActive() (a
  //     thin read-only bridge) to tell "reached its own stop condition
  //     early" (kStop) from "ran out the clock" (kTimeout). This class
  //     still holds NO stored reference to MotionEngine/Rig.
  // `stall`/`estop` need no new plumbing: both already arrive through
  // the SAME diagValue()/computeFlags() path STATUS's `flags=` uses.
  // See wire_adapter.cpp's resolvePendingReason() (pure resolution),
  // resolvePendingIfDue() (the lazy commit), forceResolvePending() (the
  // two edges captured at the moment they happen) and
  // armPendingMotion() (what every accepted motion verb arms).
  uint32_t lastDone() const override;
  Wire::DoneReason lastDoneReason() const override;

  // No registration table. This project's actual by-name test trigger
  // is protocol.cpp's cleartext `RUN:` bridge -- RunBridge plus
  // dispatchJob(), running the job on the protocol fiber itself -- a
  // CODAL-side mechanism this host-portable class must never touch.
  // Every RUN is ERR_UNKNOWN here, the same wire outcome any name a
  // real registration table would not recognize (protocol.md S6.3).
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
  // `mutable`, same reason as the pendingActive_ family below:
  // resolvePendingIfDue() (a const method, called from the const
  // accessors lastDone()/lastDoneReason()) clears this the moment it
  // lazily discovers a pending motion has resolved.
  mutable bool motionObligationActive_ = false;
  uint32_t motionObligationDeadline_ = 0;  // [ms], now_'s own scale

  // ---- motion-completion tracking (S8.8) ----
  // `pendingActive_`/`pendingId_`/`pendingGoalDirected_` track the most
  // recently ACCEPTED motion verb not yet resolved to a terminal
  // reason; `lastDoneId_`/`lastDoneReason_` are the most recently
  // RESOLVED pair -- what lastDone()/lastDoneReason() return. All five
  // are `mutable`: resolvePendingIfDue() commits a lazily-discovered
  // terminal reason from inside a const call, and it must stay
  // committed even if the diag/engine state that revealed it later
  // changes (a stall latch cleared, an estop cleared) -- this class
  // reports what a motion ACTUALLY ended with, not the live diagnostic
  // state at read time. Only armed/force-resolved with a real clock
  // wired (now_ != nullptr); with none, lastDone()/lastDoneReason()
  // report the honest 0/kNone default forever.
  mutable bool pendingActive_ = false;
  mutable uint32_t pendingId_ = 0;
  mutable bool pendingGoalDirected_ = false;
  mutable uint32_t lastDoneId_ = 0;
  mutable Wire::DoneReason lastDoneReason_ = Wire::DoneReason::kNone;

  // Pure, non-resolving read of the deadline itself: true iff
  // motionObligationActive_ is set and now_() has not yet reached
  // motionObligationDeadline_. Split out from hasLiveMotionObligation()
  // so resolvePendingReason() can read the SAME raw check without
  // recursing back through it. The GOAL-DIRECTED branch reads
  // engineMoveEndedByDeadline() instead -- latched by the engine when a
  // Segment ends, rather than re-derived from now_() at whatever later
  // moment resolution runs (a late STATUS/ack poll used to misclassify
  // an early arrival as kTimeout). This is the only signal the
  // LEASE-STYLE branch has.
  bool motionObligationDeadlineLive() const;

  // Pure function of currently observable state (diagValue()'s estop/
  // stall flags; motionObligationDeadlineLive() for a LEASE-STYLE
  // pending motion; engineMoveActive()/engineMoveEndedByDeadline() for
  // a GOAL-DIRECTED one); never mutates anything. Returns kNone
  // whenever nothing is pending OR the pending motion has not yet
  // reached a terminal state -- callers distinguish those two via
  // pendingActive_ themselves.
  Wire::DoneReason resolvePendingReason() const;

  // Commits resolvePendingReason()'s result into lastDoneId_/
  // lastDoneReason_ (and clears pendingActive_) iff it is no longer
  // kNone; a no-op otherwise. Called from both lastDone() and
  // lastDoneReason() so polling either alone is enough to notice a
  // newly terminal pending motion -- S8.8's "read fresh" contract.
  // Also clears `motionObligationActive_` on that same commit: this is
  // the natural-completion clearing point.
  void resolvePendingIfDue() const;

  // Force-resolves a still-pending motion RIGHT NOW, at a call site
  // that itself knows a reason (a supersede in one of the six onXxx()
  // handlers, or an explicit STOP in onStop()) -- resolvePendingReason()
  // still gets first refusal, so an already-stalled/estopped pending
  // motion keeps THAT more specific reason. A no-op if nothing is
  // pending. Also clears `motionObligationActive_` on that commit,
  // covering the supersede (kAborted) path -- it runs BEFORE the *new*
  // verb re-arms that flag a few lines later in the same handler, so
  // ordering stays correct. onStop()'s own explicit clear is redundant
  // with this but harmless; onEstop() needs its own, since it
  // deliberately does not call this method at all.
  void forceResolvePending(Wire::DoneReason forcedReason);

  // Arms tracking for a freshly ACCEPTED motion verb -- call AFTER any
  // supersede has already been resolved (forceResolvePending(kAborted))
  // and AFTER motionObligationActive_/motionObligationDeadline_ are
  // set. `goalDirected`: true for MOVE_X/GO_TO_R/GO_TO_W (needs
  // engineMoveActive() to resolve), false for WHEELS_V/WHEELS_X/MOVE_V
  // (resolves purely from the deadline -- see resolvePendingReason()).
  void armPendingMotion(uint32_t id, bool goalDirected);

  // ---- telemetry projection state ----
  // `seq_` wraps at 0x7F (protocol.md S6.2); buildSnapshot() advances it
  // BEFORE building each frame, so the first-ever frame reports seq 1,
  // not 0. `columns_`/`snapshot_` are members, not locals, so
  // buildSnapshot() can return a reference into them that stays valid
  // until the NEXT call -- sized for the widest documented set (POSE's
  // 12 plus FULL's 8 more).
  uint8_t seq_ = 0;
  static constexpr size_t kMaxSnapshotColumns = 20;
  Wire::Column columns_[kMaxSnapshotColumns];
  Wire::Snapshot snapshot_;
};

}  // namespace diffDrive
