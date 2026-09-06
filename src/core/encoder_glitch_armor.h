// encoder_glitch_armor.h -- EncoderGlitchArmor: the raw-counts
// plausibility decision NezhaMotorPort::collect() applies to every
// encoder sample. Its own header rather than a NezhaMotorPort member
// because nezha_port.h includes pxt.h unconditionally (src/DESIGN.md
// S1's layering table) and so cannot be compiled host-side at all;
// <cstdint> and nothing else here is what lets
// tests/host/test_encoder_glitch_armor.py exercise the decision
// directly and encoder_glitch_armor_syntax_check.cpp keep it in the
// C++11 syntax gate. The call site in NezhaMotorPort::collect() is
// review-verified only -- see its own comment in nezha_port.cpp.
//
// **The problem it solves.** A two-strike rule that rejects an
// implausible jump, then accepts it once a second mutually-consistent
// reading follows, cannot tell "the wheel really moved that far
// between reads" (the hand-rotation re-sync path) apart from "the
// counter itself restarted" (a brick MCU reset or brownout --
// encOffset_ is captured once at begin() and never re-baselined).
// Both look identical: implausible first read, second read consistent
// with it. kAcceptAsRebaseline is the third outcome for that same
// trigger, which the caller turns into an offset re-anchor rather than
// an integrated jump -- without it a reset teleports odometry by ~4 m
// at a typical ~50k-count jump.
#pragma once

#include <cstdint>

namespace diffDrive {

// A pure, stateful (but hardware-free) plausibility gate over a stream
// of raw encoder counts. No I2C, no CODAL, no floating-point clock --
// evaluate() is a function of the raw counts stream alone, called once
// per successfully-read sample.
class EncoderGlitchArmor {
 public:
  enum class Decision : uint8_t {
    kAccept,             // plausible (or not yet primed): integrate as motion.
    kAcceptAsRebaseline, // second consecutive self-consistent reading after
                         // an implausible jump: the COUNTER restarted, not
                         // the wheel -- caller re-anchors its offset instead
                         // of integrating the jump as motion.
    kRejectPending,      // first implausible reading: held, awaiting a
                         // second consistent one to disambiguate the two
                         // outcomes above. Position/velocity/sampleTime all
                         // HOLD at the caller for this outcome.
  };

  // kMaxDeltaCounts -- the plausibility bound separating "real motion in
  // one sample gap" from "the counter jumped."
  //
  // Derivation (not a tuned round number -- this codebase has been bitten
  // by exactly that before: rotationalSlip = 0.952, a constant nobody can
  // re-derive, clasi/issues/rotational-slip-not-tunable.md):
  //
  //   - Kernel cadence: DiffDrive::DifferentialDrive::Config::cyclePeriod
  //     defaults to 24 ms (diffdrive.h), and NezhaMotorPort::collect() is
  //     called exactly once per tick (nezha_port.cpp's tick()). The
  //     common-case gap between two successful collects is one cycle.
  //   - But the gap this check must tolerate can span roughly TWO cycles:
  //     an I2C read failure (connected_ = false) or a first implausible
  //     reading (rejectPending_) both hold lastGoodRaw_ unchanged for at
  //     least one extra tick before the next comparison -- code review
  //     R-07 / KERN-07's own arithmetic independently lands on "24-48 ms"
  //     for this same reason (verify-kernel.md).
  //   - Physically achievable wheel velocity: fullDutyVelocity, this
  //     kernel's own MEASURED 100%-duty wheel rate (shims.cpp:
  //     cfg.fullDutyVelocity = 10795.0f -- the same figure the kernel's
  //     own velocity-mode duty conversion already treats as ground truth
  //     for "as fast as this wheel can turn," diffdrive.cpp's
  //     dutyPerSpeed = 1/fullDutyVelocity). No commanded motion can
  //     exceed it without the shaping pipeline's [-1, 1] duty clamp
  //     already having been violated.
  //   - Max plausible delta in one 24 ms cycle:
  //       10795 counts/s * 0.024 s ~= 259 counts.
  //     Over the worst-case ~2-cycle gap: ~= 518 counts.
  //   - kMaxDeltaCounts = 5000 sits ~10x above that worst-case plausible
  //     motion delta (headroom for I2C jitter, PID overshoot, and several
  //     consecutive held ticks) while sitting ~10x below the smallest
  //     confirmed discontinuity magnitude this ticket targets (a brick
  //     reset's ~50,000-count jump, R-07). Comfortable separation on both
  //     sides, derived from measured hardware limits and the kernel's own
  //     configured cadence -- not picked to make a specific bench run
  //     pass.
  static constexpr int32_t kMaxDeltaCounts = 5000;

  // Evaluates one new raw sample against the accumulated two-strike
  // state and returns the plausibility decision, mutating
  // lastGoodRaw_/lastRejectedRaw_/rejectPending_. Before priming (see
  // markPrimed()/seedLastGoodRaw() below) every reading is accepted
  // unconditionally -- an un-begun port has no baseline to judge one
  // against.
  //
  // Explicit raw-zero rejection: a destroyed sample (an interposed I2C
  // transaction landing inside this counter's own select-to-read settle
  // window) reads back as an exact raw 0. Magnitude alone cannot catch
  // that while lastGoodRaw_ is itself still small -- early after a
  // fresh baseline, |0 - lastGoodRaw_| sits comfortably inside
  // kMaxDeltaCounts and would otherwise fall straight through to
  // kAccept below, integrating the destroyed sample as real motion (a
  // jump toward zero, then back, the next time a genuine reading
  // arrives). Treat any exact 0 as suspect the moment the counter has
  // ever produced a nonzero good reading, regardless of magnitude --
  // routed through the SAME two-strike disambiguation the magnitude
  // check below uses (handleImplausible()), so a counter that has
  // genuinely restarted -- one implausible reading followed by a
  // second consistent with it, zero or otherwise -- still reaches
  // kAcceptAsRebaseline; only a lone, unconfirmed zero is held pending.
  Decision evaluate(int32_t raw) {
    if (primed_) {
      if (raw == 0 && lastGoodRaw_ != 0) return handleImplausible(raw);
      const int32_t delta = raw - lastGoodRaw_;
      const int32_t mag = delta < 0 ? -delta : delta;
      if (mag > kMaxDeltaCounts) return handleImplausible(raw);
    }
    rejectPending_ = false;
    lastGoodRaw_ = raw;
    return Decision::kAccept;
  }

  // The last raw count value this armor accepted as truth (either
  // outcome that returns kAccept/kAcceptAsRebaseline updates this). The
  // caller's own rebaseline() reads this the same way it always read
  // NezhaMotorPort's private lastGoodRaw_ member before this extraction.
  int32_t lastGoodRaw() const { return lastGoodRaw_; }

  // Seeds the two-strike baseline without going through evaluate() --
  // for NezhaMotorPort::begin()'s own initial median-of-3 read, which
  // is not itself a "sample stream" evaluation.
  void seedLastGoodRaw(int32_t raw) { lastGoodRaw_ = raw; }

  // Arms the plausibility check. Split from seedLastGoodRaw() because
  // begin() arms unconditionally even when its initial read produced no
  // usable sample (good == 0): lastGoodRaw_ then stays at its default
  // of 0 while the gate is live, and that corner case is deliberate.
  void markPrimed() { primed_ = true; }

 private:
  // The shared two-strike disambiguation: an implausible reading (raw-
  // zero or over-magnitude, evaluate() has already decided which) is
  // held on its first appearance; a SECOND reading self-consistent with
  // the first rejected one (within kMaxDeltaCounts of it) reclassifies
  // both as a counter restart instead of integrated motion. One holder
  // of this logic means the zero check and the magnitude check can
  // never drift apart on what counts as "consistent".
  Decision handleImplausible(int32_t raw) {
    const int32_t rejDelta = raw - lastRejectedRaw_;
    const int32_t rejMag = rejDelta < 0 ? -rejDelta : rejDelta;
    if (rejectPending_ && rejMag <= kMaxDeltaCounts) {
      rejectPending_ = false;
      lastGoodRaw_ = raw;
      return Decision::kAcceptAsRebaseline;
    }
    lastRejectedRaw_ = raw;
    rejectPending_ = true;
    return Decision::kRejectPending;
  }

  int32_t lastGoodRaw_ = 0;
  int32_t lastRejectedRaw_ = 0;
  bool rejectPending_ = false;
  bool primed_ = false;
};

}  // namespace diffDrive
