// wire_handler.h -- Wire::WireHandler: protocol v6's ASCII line-grammar
// mechanics (radio-robot-lib/docs/design/protocol.md S2, S2.1, S3.1,
// S3.2) PLUS the reliability layer (S8, S8.9 -- the canonical spec; this
// project conforms to that grammar, it does not vendor radio-robot-lib's
// C++). feed() reassembles arbitrary byte blocks into '\n'-terminated
// lines, tokenizes each line in place on runs of ' ' (no allocation, no
// std::string -- S3.2), enforces case-as-direction (S2.1: commands
// UPPERCASE, replies lowercase, verb lookup case-SENSITIVE), and
// dispatches every verb this project currently implements: the seven
// unsequenced exemptions (HELLO, PING, ESTOP, HELP, ID, VER, STATUS --
// S8.3) plus the five non-motion sequenced verbs (GET, SET, TLM, STOP,
// RUN).
//
// The rule that draws that line, added 2026-08-27: A VERB CARRIES A
// SEQUENCE ID IFF DUPLICATE EXECUTION OR REORDERING COULD CAUSE HARM,
// i.e. iff it changes state. Queries (ID/VER/STATUS) and liveness/
// orientation verbs (HELLO/PING/HELP) are idempotent, so sequencing
// them bought nothing and cost a silent drop of the bare form plus a
// payload-less stale ack on a resend. GET is a deliberate exception:
// read-only but still sequenced, to keep the config plane symmetric
// with SET (stakeholder direction).
//
// HELP joined the unsequenced set on 2026-08-27 by stakeholder
// direction. It is a human-typed diagnostic verb -- the FIRST thing an
// operator types into a raw relay session -- and requiring a `#<id>`
// on it meant a bare `HELP` was silently dropped, which is exactly the
// wrong answer for the one verb whose entire job is telling a confused
// operator what to do next. It is forgiving like PING: any arity, with
// or without an id, always answers.
//
// ---- The reliability layer (S8), in one paragraph ----
//
// Every sequenced verb carries a MANDATORY trailing id, `#<n>`, that is
// also a strictly incrementing sequence number starting at 1. Handler
// state is EXACTLY one value -- expectedNext_ (next id expected;
// gapOutstanding_ is GONE, 2026-08-26, S8.5: its only reader was the
// deleted telemetry ack piggyback) -- deliberately no
// clock and no timer anywhere (S8.1): feed() stays a pure function of
// its input bytes plus this small state. dispatch() resolves the id
// FIRST, against expectedNext_, classifying every inbound id into
// exactly one of three cases (S8.1's table):
//   - id == expectedNext_ : decode the verb's own fields FIRST (S8.9);
//     only if decoding succeeds does the sequence advance
//     (expectedNext_ = id + 1) and `ack <id> <lastDone> <reason>` go
//     out. A decode failure at this point -- unrecognized verb, wrong
//     arity, or an unparseable field -- does NOT advance the sequence:
//     it replies `nack <expectedNext_> <lastDone> <reason>` (still
//     naming the SAME id, since it was never accepted) plus
//     `err <code> #<id>`; a stalled stream keeps re-nacking because
//     every subsequent inbound line re-triggers the same nack (S8.1 --
//     there is no periodic re-nack, 2026-08-26, S8.5) until a
//     well-formed line finally supplies
//     that same id (S8.9 -- "decode failure is a NAK", the central
//     2026-08-22 change: a corrupted leg of a multi-leg routine is
//     resent, not silently skipped).
//   - id < expectedNext_ : a stale retransmit -- the host never saw our
//     ack for something we already accepted. Do NOT re-execute (a
//     resent WHEELS_V must not drive the wheels twice, once motion
//     verbs land); reply `ack <expectedNext_ - 1> <lastDone> <reason>`,
//     the already-accepted id, not the resent one. `#0` is not
//     special-cased anywhere in this file: since expectedNext_ starts
//     at (and never goes below) 1, an inbound `#0` is unconditionally
//     `< expectedNext_` and falls into this bucket with zero extra code.
//   - id > expectedNext_ : a numeric gap -- discard, do NOT execute, and
//     do not even look up the verb; reply
//     `nack <expectedNext_> <lastDone> <reason>`. A gap stalls the
//     stream ON PURPOSE: every
//     subsequent command, however well-formed, is nacked identically
//     until the missing id arrives.
// A merits rejection -- the verb decoded fine but the Adapter refuses it
// on its own terms (e.g. an out-of-range value) -- is a DIFFERENT case
// from a decode failure: it ACKS and ADVANCES (the line arrived intact),
// paired with `err <code> #<id>` on top of that ack. Decode failure and
// merits rejection are the two cases S8.9 keeps sharply distinct.
//
// `<lastDone>`/`<reason>` are read FRESH off Adapter::lastDone()/
// lastDoneReason() every time an ack/nack is formatted (S8.8) -- there
// is no cached copy anywhere in this class. `err <code> #<id>` orders
// code first, id last (S8.6) -- the id is always a line's LAST token,
// commands and replies alike.
//
// HELLO/ESTOP/PING (S8.3) never carry an id at all and are maximally
// forgiving of trailing content -- see dispatch()'s own comment. HELLO
// additionally resets expectedNext_ (a (re)connecting
// host's own resync point) but does NOT touch the Adapter's
// lastDone()/lastDoneReason() -- that state is Adapter-owned and a
// handler-level reset has no business reaching into it (S8.8).
//
// Angles (rotation, omega) are milliradian integers on the wire
// (motion-api.md S9.1) -- decoded here with the ordinary signed-integer
// field parser, same as any other field; the degrees-at-the-API
// conversion is a LANGUAGE BINDING's job, not this file's.
//
// Host-portable by construction: no pxt.h, no CODAL type, anywhere in
// this file or wire_handler.cpp. See tests/host/wire_grammar_shim.cpp
// for the native host test harness this module is exercised through.
#pragma once

#include <cstddef>
#include <cstdint>

namespace Wire {

// Sink -- where finished reply lines go. Exactly one write() per
// formatted line, INCLUDING the trailing '\n'; the caller owns
// transport (serial, radio, or a test's recording buffer). Mirrors
// radio-robot-lib's own Protocol::Sink split (protocol_handler.h).
class Sink {
 public:
  virtual ~Sink() = default;
  virtual void write(const char* data, size_t length) = 0;
};

// Everything HELLO/ID/VER read off. Every pointer is borrowed: the
// adapter owns the storage (a string literal or a robot-config field)
// and must keep it alive at least until the identity() call that
// requested it returns. Mirrors radio-robot-lib's own Protocol::Identity
// (adapter.h) -- drivetrain/profile/version join name/serial, which
// ID/VER read alongside HELLO's own banner fields. ID now reads `name`
// too (a fourth, appended wire field) -- see wire_handler.cpp's
// execId() for the wire-format reasoning and protocol.cpp's own
// kProfile comment for why `name`, not `profile`, is this struct's
// authoritative board-identity field.
struct Identity {
  const char* name = "";
  const char* serial = "";
  const char* drivetrain = "";
  const char* profile = "";
  const char* version = "";
};

// STATUS's `k=v` payload. `tlm` is the CURRENT subscription mode's own
// lowercase wire name ("off"/"pose"/"full"/"auto"/"buffer") -- the
// handler does not re-derive it from TlmMode, so an adapter tracking its
// own mode state machine never has to reconcile it against the
// handler's opinion of what "current" means. Mirrors radio-robot-lib's
// own Protocol::StatusFields (adapter.h).
struct StatusFields {
  bool ready = false;
  bool active = false;
  bool connLeft = false;
  bool connRight = false;
  bool otos = false;
  bool wedge = false;
  uint32_t flags = 0;
  // Sprint 004 ticket 004: the I2C fault counter, closing
  // status-lost-diag-numeric-surface.md -- the retired DIAG verb's own
  // most-important numeric field (a wedged/unpowered Nezha brick shows
  // up here as a climbing count, not just a boolean "wedge" flag).
  // Sourced by WireAdapter::status() from the SAME diagValue(8) call
  // the telemetry `i2cf` column also reads, so the two can never
  // disagree (sprint.md's own Design Rationale). Decimal on the wire
  // (execStatus()'s `i2cf=%ld`), unlike `flags`' hex -- a raw fault
  // count has no bitfield meaning to pack.
  int32_t i2cf = 0;
  // Sprint 010 ticket 003: the kernel's own heartbeat counter, closing
  // unpowered-nezha-brick-wedges-program-at-boot.md's 2026-08-24
  // correction -- a robot nothing has ever ticked and a robot with a
  // genuinely unreachable brick used to report the IDENTICAL STATUS
  // line (ready=0 connL=0 connR=0 i2cf=0), because ready/connL/connR/
  // i2cf are all only ever written from inside step()/collect(), which
  // never ran either way. `cyc` is the discriminator: 0 means "this
  // kernel has never ticked" (every other field's 0 is meaningless,
  // not a fault), nonzero means the kernel is running and every other
  // field means what it says. Sourced by WireAdapter::status() from the
  // SAME diagValue(16) call the telemetry `cyc` column already reads
  // (src/comms/wire_adapter.cpp), so the two can never disagree -- mirrors
  // `i2cf` immediately above, sprint 004 ticket 004's identical
  // same-source guarantee. Unsigned and decimal on the wire
  // (execStatus()'s `cyc=%lu`): a cycle count never goes negative and
  // has no bitfield meaning to pack.
  uint32_t cyc = 0;
  const char* tlm = "off";
};

// One named, already-scaled telemetry value (protocol.md S5.2: `thdr
// <col>...` then `t <v>...`). Mirrors radio-robot-lib's own
// Column shape (src/protocol/adapter.h:113-139) -- this project's own
// value type, not a vendored copy. `value` is always an
// already-scaled plain integer -- this class has no opinion on what a
// column MEANS or how it was derived, only how it prints: `hex` picks
// lowercase hex with no `0x` prefix (flags-shaped columns); everything
// else prints signed base-10.
// Sprint 004 ticket 007 (remediating ticket 005's thrown exception):
// this struct's default member initializers below are legal C++20 but
// disqualify it from being a C++11 aggregate -- and BOTH real embedded
// build targets compile at -std=c++11 (baked into the pxt-microbit
// target's own yotta/CMake toolchain files), while tests/host/ compiles
// at -std=c++20 (test_kernel_harness.py), which is why 253 host tests
// passed against `columns_[i++] = {"name", value, hex};` call sites
// (WireAdapter::buildSnapshot(), src/comms/wire_adapter.cpp) that could not
// actually be compiled for the robot. Explicit `Column() = default;`
// plus this 3-argument converting constructor fix that WITHOUT dropping
// the NSDMIs (dropping them would leave every default-constructed
// `Column columns_[kMaxSnapshotColumns]` -- wire_adapter.h -- holding
// indeterminate values until every element is filled) and WITHOUT
// touching any of the ~20 already-correct call sites (each already
// passes exactly these 3 positional arguments, matching this
// constructor's signature exactly). See
// host-tests-compile-newer-standard-than-target.md (sprint 008) for the
// systemic gap this is one confirmed instance of.
struct Column {
  const char* name = "";
  int32_t value = 0;
  bool hex = false;

  Column() = default;
  Column(const char* name_, int32_t value_, bool hex_)
      : name(name_), value(value_), hex(hex_) {}
};

// One telemetry frame's worth of columns (protocol.md S5.2). `columns`
// is BORROWED: the caller (WireAdapter::buildSnapshot(), ticket 004)
// owns the backing array and must keep it alive only for the duration
// of the emitTelemetry(snapshot) call it is passed to -- WireHandler
// copies what it needs for its own header memo (see kMaxHeaderColumns/
// kMaxHeaderNameBytes below) and formats the rest immediately, keeping
// no borrowed pointer alive past that one call. Mirrors radio-robot-
// lib's own Snapshot shape (adapter.h:113-139).
//
// Shares Column's exact NSDMI shape immediately above, and is therefore
// ALSO not a C++11 aggregate for the identical reason (sprint 004
// ticket 007) -- but unlike Column, no site anywhere in src/ or
// tests/host/ ever brace-initializes a Snapshot (every site
// default-constructs one, then assigns `.columns`/`.count`
// field-by-field), so this is a latent structural twin of Column's
// defect, not a live one. Deliberately left unfixed here: there is no
// call site it would protect, and adding constructors it doesn't need
// would be scope creep against this ticket's own two confirmed defects.
struct Snapshot {
  const Column* columns = nullptr;
  size_t count = 0;
};

// Maps 1:1 onto the wire outcome (protocol.md S4; wire codes per S6.1).
// There is no kDuplicateId: the handler's own strict sequencing already
// makes a duplicate id structurally unreachable by any Adapter call
// (S2.2/S8.1's retransmit row never dispatches), so the wire's own error
// code 11 (ERR_DUPLICATE_ID) does not exist here at all -- it is
// deleted, not merely unused.
enum class Result : uint8_t {
  kOk,             // -> the ack alone; no further reply
  kUnknown,        // -> err 1 #<id>   ERR_UNKNOWN
  kBadArg,         // -> err 2 #<id>   ERR_BADARG
  kRange,          // -> err 3 #<id>   ERR_RANGE
  kFull,           // -> err 4 #<id>   ERR_FULL
  kUnimplemented,  // -> err 6 #<id>   ERR_UNIMPLEMENTED
  kNotReady,       // -> err 8 #<id>   ERR_NOT_CONFIGURED
  kBusy,           // -> err 10 #<id>  ERR_BUSY
};

// TLM subscription modes (S6.1's wire token set). The handler only
// decodes the wire token ("OFF"/"POSE"/"FULL"/"NOW"/"AUTO"/"BUFFER")
// into this enum and hands it to onTlm() -- what each mode DOES is
// entirely the adapter's business.
enum class TlmMode : uint8_t {
  kOff,
  kPose,
  kFull,
  kNow,
  kAuto,
  kBuffer,
};

// The reliability layer's completion-reason vocabulary (S8.8): the
// reasons a motion can finish, plus kNone for "nothing has completed
// yet" -- the wire spelling "none" is what lastDone() == 0 pairs with.
// Every sequenced verb's ack/nack piggybacks this pair (S8.8), not only
// the motion ones.
//
// kStall (sprint 005 ticket 004, closing wire-motion-completion-
// signal.md/R-23): purely additive -- no existing wire consumer reads
// it, since nothing ever produced it before this ticket. Wire spelling
// "stall", matching the kernel's own stall-latch semantics
// (`stallHalted`/`stall_clear`, sprint 007 ticket 001) rather than
// inventing a second notion of "stalled" -- see
// diffDrive::WireAdapter::resolvePendingReason() (wire_adapter.cpp) for
// where this is actually produced. Deliberately NOT folded into
// kAborted (a stalled drivetrain and a superseded command are different
// failure classes a host needs to tell apart) or kEstop (stall is
// drivetrain-local, not the same safety condition) -- sprint.md's own
// Design Rationale.
enum class DoneReason : uint8_t {
  kNone,     // -> "none"    -- lastDone() == 0, nothing completed yet
  kStop,     // -> "stop"    -- the stop condition was met, or stop() ended it
  kTimeout,  // -> "timeout" -- the backstop fired
  kEstop,    // -> "estop"   -- a panic stop ended it
  kAborted,  // -> "aborted" -- the caller abandoned it
  kStall,    // -> "stall"   -- the kernel's stall latch halted the drivetrain
};

// Adapter -- the seam behind every verb this file currently dispatches:
// identity (HELLO/ID/VER), a clock (PING), an ESTOP hook, STATUS's own
// fields, GET/SET's field table, TLM's mode hook, STOP, RUN's
// invoke-by-name, and the reliability layer's completion channel
// (lastDone()/lastDoneReason(), polled fresh on every ack/nack, S8.8).
//
// This file only declares the CONTRACT: diffDrive::WireAdapter
// (src/comms/wire_adapter.h) is the production implementation, backed by this
// robot's real identity/config/shims.cpp surface; tests/host/
// wire_mock_adapter.h's WireMockAdapter is the test double (a recording
// stand-in, never linked into production code). See each motion
// method's own doc comment below for its exact wire units.
class Adapter {
 public:
  virtual ~Adapter() = default;

  // ---- session ----
  virtual void identity(Identity& out) const = 0;
  virtual uint32_t now() const = 0;  // [ms], for PING's `pong <now>`
  virtual void status(StatusFields& out) const = 0;

  // ---- motion: the six verbs (motion-api.md S9.1, sprint 003 ticket
  // 004). Angles (rotation, omega) arrive already decoded from the
  // wire's milliradian integers into float milliradians -- degrees-at-
  // the-API is a LANGUAGE BINDING's conversion, not this seam's. ----
  virtual Result onWheelsV(float left, float right,  // [mm/s] [mm/s]
                           uint32_t duration,          // [ms]
                           uint32_t id) = 0;
  virtual Result onWheelsX(float left, float right,  // [mm] [mm]
                           float cruise,               // [mm/s]
                           uint32_t timeout,            // [ms]
                           uint32_t id) = 0;
  virtual Result onMoveX(float distance, float rotation,  // [mm] [mrad]
                        float cruise, uint32_t timeout,    // [mm/s] [ms]
                        uint32_t id) = 0;
  virtual Result onMoveV(float v_x, float omega,        // [mm/s] [mrad/s]
                        uint32_t duration, uint32_t id) = 0;  // [ms]
  virtual Result onGoToR(float x, float y, float speed,   // [mm] [mm] [mm/s]
                        float arrive, uint32_t timeout,    // [mm] [ms]
                        uint32_t id) = 0;
  virtual Result onGoToW(float x, float y, float speed,
                        float arrive, uint32_t timeout,
                        uint32_t id) = 0;

  // ---- safety ----
  virtual void onEstop() = 0;
  // `immediate` is STOP's own optional `now` token (a deceleration
  // CHOICE, not a different verb) -- an adapter with no ramp of its own
  // is free to treat both identically.
  virtual Result onStop(bool immediate, uint32_t id) = 0;

  // ---- configuration -- pure delegation, no storage in this file
  // (protocol.md S7: which names are valid is entirely the adapter's
  // business) ----
  virtual bool onGet(const char* name, float& out) const = 0;
  virtual Result onSet(const char* name, float value, uint32_t id) = 0;
  virtual size_t fieldCount() const = 0;  // for a bare GET
  virtual const char* fieldName(size_t index) const = 0;

  // ---- telemetry ----
  virtual Result onTlm(TlmMode mode) = 0;

  // ---- the reliability layer's completion channel (S8.8) -- POLLED
  // fresh by this class every time it formats an ack/nack line; no
  // callback, no clock, no cached copy anywhere in WireHandler. An
  // adapter with no completion event of its own returns 0/kNone
  // forever, which is wire-correct even though it is functionally inert
  // on that adapter. ----
  virtual uint32_t lastDone() const = 0;
  virtual DoneReason lastDoneReason() const = 0;

  // ---- invocation by name (protocol.md's RUN section) -- this class
  // holds no function table, does no name resolution, and does no type
  // conversion; it only parses "RUN <name> [arg...] #id" into a name and
  // the RAW, unconverted argument tokens that followed it, and hands
  // them here unchanged. See the .cpp's execRun() for the full contract
  // (borrowed pointers, sanitization of the returned text, synchronous
  // invocation). ----
  virtual Result onRun(const char* name, const char* const* argv, size_t argc,
                       char* result, size_t resultCapacity,
                       bool& hasResult) = 0;
};

class WireHandler {
 public:
  // Wire line ceiling, protocol.md S2: "Max line: 240 bytes including
  // the terminator." The handler sizes its buffer off this one
  // constant so the number is spelled exactly once.
  static constexpr size_t kMaxLineBytes = 240;

  WireHandler(Adapter& adapter, Sink& sink);

  // Feed an arbitrary block from the port -- may contain zero, one, or
  // several complete lines, and may end mid-line. Partial lines are
  // buffered across calls; complete lines are parsed and dispatched
  // immediately, in the order they complete. Must survive (protocol.md
  // S2, S3.1):
  //   - several complete lines in one block;
  //   - a block ending mid-line (the remainder is buffered to the next
  //     feed() call);
  //   - a block that is only a line fragment;
  //   - a lone '\r' immediately before '\n' (stripped as a terminal
  //     artifact; '\r' appears nowhere else on the wire);
  //   - a blank or all-whitespace line (ignored SILENTLY -- a terminal
  //     artifact, not an error; does NOT count malformed);
  //   - a line longer than the 240-byte maximum: discarded to the next
  //     '\n' and counted malformed -- NEVER truncated into a
  //     still-parseable prefix that would be a command the host never
  //     sent;
  //   - embedded NULs, arbitrary binary garbage, or a multi-KB blast
  //     with no newline at all (bounded by the fixed line buffer and
  //     the overflow rule above; never overflows, never allocates).
  //
  // A characterization note on embedded NULs, carried over from
  // radio-robot-lib's own protocol.md S9.4: every wire-touching
  // comparison here (verb lookup, tokenizing) runs on a NUL-terminated
  // C string, per the no-allocation, no-std::string constraint (S3.2).
  // A NUL byte anywhere inside a line therefore acts as an early
  // terminator for THIS line only -- e.g. "PING\0extra\n" dispatches
  // exactly like a bare "PING\n", silently discarding "extra" with no
  // malformed-count increment -- rather than being rejected outright.
  // This is a known, pinned characterization (see
  // test_embedded_nul_immediately_after_verb_matches_bare_verb in
  // tests/host/test_wire_grammar.py), not a bug: a real fix would mean
  // abandoning C-string comparisons throughout the parser, in tension
  // with the no-std::string firmware constraint this file is written
  // to. The one place this class does NOT just let the C-string view
  // win silently: a line whose first non-space byte IS the embedded
  // NUL (e.g. "\0PING\n") would otherwise leave the tokenizer's own
  // internal token array uninitialized (a real memory-safety hazard,
  // not just a surprising parse) -- wire_handler.cpp's onLineComplete()
  // guards this explicitly and counts it malformed instead.
  void feed(const char* data, size_t length);

  // HELLO's reply, byte-identical to the unsolicited boot banner a
  // caller would send at connect time (protocol.md S4/S6):
  // "device NEZHA2 robot <name> <serial>\n".
  void sendBanner();

  // The telemetry frame (protocol.md S5.2): emits, in order, as THREE
  // separate Sink::write() calls (never concatenated into one) --
  //   1. `thdr <col>...\n`, but only when a fresh header is DUE (see
  //      below);
  //   2. `t <v>...\n`, always, one value per column in `snapshot`, in
  //      the same order as the most recently emitted header.
  // NOTHING ELSE (2026-08-26, protocol.md S8.5): the ack/nack keepalive
  // that used to ride as a third write is DELETED -- an ack/nack is only
  // ever a direct reply to an inbound sequenced line, never a beacon. A
  // subscriber that wants to know whether its last command landed sends
  // a command (e.g. STATUS) and reads that command's own ack.
  // A fresh header is DUE when: this is the very first call ever made
  // on this instance; the column set changed since the last header
  // (count, any column's name, OR any column's hex-ness -- a memo that
  // compared only names/count would miss a hex-ness-only flip); or
  // kHeaderRefreshFrames calls have elapsed since the last header,
  // whichever comes first. That last case is what keeps a late-
  // attaching listener over a lossy broadcast radio from being
  // permanently locked out of decoding (sprint.md SUC-004) -- it has
  // nothing to do with the column set changing at all.
  //
  // `snapshot`'s backing array is borrowed only for the duration of
  // this call; this class copies what it needs of it (the header memo)
  // and touches nothing else afterward.
  void emitTelemetry(const Snapshot& snapshot);

  // emitReliability() is GONE (2026-08-26, protocol.md S8.5): the
  // periodic/piggybacked ack-nack keepalive it carried is deleted
  // outright, per stakeholder direction ("an ack or a nack is only a
  // response to a message, not a beacon"). The only remaining origin of
  // every ack/nack is dispatch()'s own per-inbound-line reply (S8.1);
  // a lost ack/nack heals via the host's own retransmit or poll.

  // Lines dropped as: an unrecognized verb or one this file does not
  // (yet) implement, wrong arity, an unparseable field, a sequenced
  // verb whose mandatory id was missing or malformed, or an overlong
  // line (discarded to the next '\n'). This INCLUDES a decode failure on
  // an in-order sequenced id (protocol.md S8.9) -- unlike a numeric gap
  // (id > expectedNext_) or a stale retransmit (id < expectedNext_),
  // NEITHER of which is ever counted here, since neither one's content
  // is ever even inspected. A lowercase-led inbound verb -- another
  // robot's reply overheard on a shared channel, S2.1 -- is dropped
  // silently and does NOT increment this, and neither does a
  // blank/all-whitespace line.
  uint32_t malformedCount() const { return malformedCount_; }

  // Builds "help" plus a space-separated `name` for every entry in
  // `names` (`nameCount` entries) into `buf` (capacity `bufCap`),
  // followed by '\n'. The terminator is written LAST but into a byte
  // the content-filling loop is structurally forbidden to reach (its
  // bound stops one byte short of the NUL as well as the '\n'), so it
  // is always the LISTED NAMES that truncate if they would ever
  // overflow `bufCap` -- never the terminator. Returns the number of
  // bytes written, excluding the closing NUL. Public and static purely
  // so a host test can drive it directly with a synthetic, arbitrarily
  // long name list -- independent of kCommandTable, which today is far
  // too small to ever exercise the truncation path this proves safe.
  // Longest `help ...` line emitBuild will produce before starting a
  // new one. Deliberately well under kMaxLineBytes: see emitHelp().
  static constexpr size_t kHelpChunkBytes = 60;
  static size_t buildHelpLine(char* buf, size_t bufCap,
                               const char* const* names, size_t nameCount);

 private:
  void appendByte(char c);
  void onLineComplete();

  // Splits `line` (already NUL-terminated) into tokens in place on runs
  // of ' ', collapsing separators and ignoring leading/trailing
  // whitespace (protocol.md S2/S3.2). Returns the TRUE total token
  // count (verb included), which may exceed `maxTokens` -- only the
  // first `maxTokens` pointers are stored.
  static size_t tokenizeLine(char* line, char** tokens, size_t maxTokens);

  // Resolves the mandatory trailing id against expectedNext_ (protocol.md
  // S8.1), decodes the verb's own fields BEFORE sending any reply at all
  // for the in-order case (S8.9), and dispatches. `lastFieldToken` is
  // the line's raw last token (verb included), found by a backward scan
  // done BEFORE tokenizeLine() mutates the line -- see the .cpp's
  // findLastFieldToken() for why. nullptr means the line was just the
  // verb, with nothing after it to resolve an id from.
  void dispatch(char* verb, char** fields, size_t fieldCount,
                const char* lastFieldToken);

  // A DECODE FAILURE on an in-order id (protocol.md S8.9): the sequence
  // does NOT advance -- `id` is still expectedNext_ (that equality is
  // what routed dispatch() into this function at all), so nacking
  // expectedNext_ unchanged tells the host to resend EXACTLY this id.
  void handleDecodeFailure(uint32_t id, uint8_t code);
  void replyAck(uint32_t ackedId);   // "ack <ackedId> <lastDone> <reason>\n"
  void replyNack(uint32_t nextId);   // "nack <nextId> <lastDone> <reason>\n"
  void replyErr(uint32_t id, uint8_t code);  // "err <code> #<id>\n" (S8.6)
  void writeLine(const char* text);  // one Sink::write() per line
  static uint8_t resultCode(Result result);
  static const char* doneReasonWireName(DoneReason reason);

  void handleHello();
  void handlePing();
  // Emits the reliability reminder -- `nack <expectedNext_>` -- iff a
  // gap or decode-failure stall is outstanding. Called at the END of
  // every unsequenced verb's reply EXCEPT ESTOP and HELLO (see
  // dispatch()). Silent on a clean stream, which is the whole point.
  void emitReminderIfStalled();
  // Writes the verb listing. Shared by dispatch()'s unsequenced HELP
  // interception and execHelp()'s table row, so both paths emit
  // byte-identical output.
  void emitHelp();
  void handleEstop();

  // Every DECODE function is pure: no adapter call, no sink write, no
  // mutation of handler state. It answers exactly one question -- "does
  // this line's own content parse?" -- so dispatch() can decide
  // ack-vs-nack BEFORE anything with a wire or Adapter side effect runs
  // (protocol.md S8.9). Returns false (a DECODE FAILURE) for wrong arity
  // or an unparseable field; true otherwise. `fields`/`fieldCount` here
  // EXCLUDE the id (already resolved and stripped by dispatch()).
  using DecodeFn = bool (WireHandler::*)(char** fields, size_t fieldCount);

  // Every EXECUTE function runs ONLY after dispatch() has already
  // decided the line decodes AND has already sent the `ack` for it -- so
  // an execute function is free to write informational reply lines
  // (id/ver/status/help/get/ret) directly to the sink; nothing it does
  // can race the ack that must precede those lines on the wire. It
  // reports any ADAPTER-level (merits) rejection through `errCode` (0 ==
  // kOk == no err line; nonzero == the wire code dispatch() will emit as
  // `err <errCode> #<id>` right after whatever this function itself
  // already wrote).
  using ExecuteFn = void (WireHandler::*)(char** fields, size_t fieldCount,
                                          uint32_t id, uint8_t& errCode);

  struct VerbEntry {
    const char* name;
    DecodeFn decode;
    ExecuteFn execute;
  };

  // HELLO/PING/ESTOP's own rows are trivial stand-ins (decodeAlwaysTrue/
  // execNoop) NEVER actually invoked through this table -- dispatch()
  // intercepts all three by verb identity before any id is even looked
  // at (protocol.md S8.3). They are still present here purely so HELP's
  // generated listing (execHelp()) walks ONE table for every verb name
  // this file knows about and cannot drift from the dispatcher.
  //
  // WIRE-09 (code review 2026-08-23): deliberately declared with NO
  // explicit size -- the definition in wire_handler.cpp supplies the
  // bound, deduced from its own initializer list. An explicit `[18]`
  // here (the old spelling) meant the count was spelled twice, and
  // *removing* a row from the .cpp's initializer (or missing one while
  // renaming) compiled SILENTLY: the array zero-filled the vacated
  // slot, that entry's `name` read back nullptr, and the first inbound
  // sequenced verb that walked the table into it hit
  // `strcmp(verb, nullptr)` -- UB, a hard fault in practice, on every
  // subsequent unrecognized-verb line or HELP call (the two paths that
  // walk the whole table; see verify-wire.md's own scope correction --
  // NOT every command, only those two). Leaving the size to be deduced
  // makes a row COUNT change of either sign visible from the deduced
  // bound; the constructor's own static_assert (wire_handler.cpp) pins
  // the expected count so a future accidental removal fails to
  // COMPILE instead of silently shipping.
  static const VerbEntry kCommandTable[];

  // Field-token storage cap for one line, verb-exclusive (id excluded --
  // it is resolved separately, see dispatch()'s own comment). Every
  // fixed-arity verb this file wires up has at most 5 data fields
  // (GO_TO_R/GO_TO_W's x/y/speed/arrive/timeout) -- comfortably inside
  // this cap. RUN is the one exception: its arity is open-ended, so this
  // cap doubles as RUN's own hard ceiling on how many raw DATA tokens it
  // will trust fields[] to hold pointers for at all -- decodeRun()
  // checks its own fieldCount against this constant BEFORE indexing
  // fields[].
  static constexpr size_t kMaxFieldTokens = 20;

  // RUN's own ceiling on how many ARGUMENTS (excluding the function
  // name) it will forward to onRun() -- a firmware resource limit (the
  // fixed argv[] array execRun() builds on the stack), not a claim about
  // any real function's arity.
  static constexpr size_t kMaxRunArgs = 16;

  // RUN's stringified return value -- an ARRAY SIZE (content bytes plus
  // the NUL terminator), sized so the WHOLE reply line -- "ret " + this
  // text + " #<id>" at id's maximum width (a 10-digit uint32_t) + '\n'
  // -- can never exceed kMaxLineBytes even before execRun()'s own
  // sanitize pass, which can only shrink the text further, never grow
  // it.
  static constexpr size_t kMaxRunResultBytes =
      kMaxLineBytes - 4 /* "ret " */ - 12 /* " #4294967295" */;

  // ---- per-verb decode/execute pairs -- see DecodeFn/ExecuteFn's own
  // comments above for the shared contract. ----
  bool decodeNoFields(char** fields, size_t fieldCount);
  void execId(char** fields, size_t fieldCount, uint32_t id, uint8_t& errCode);
  void execVer(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);
  void execStatus(char** fields, size_t fieldCount, uint32_t id,
                  uint8_t& errCode);
  void execHelp(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

  bool decodeGet(char** fields, size_t fieldCount);
  void execGet(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  bool decodeSet(char** fields, size_t fieldCount);
  void execSet(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  bool decodeTlm(char** fields, size_t fieldCount);
  void execTlm(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  // ---- motion: WHEELS_X / WHEELS_V / MOVE_X / MOVE_V / GO_TO_R /
  // GO_TO_W (motion-api.md S9.1's wire mapping). Every decode function
  // here is a plain arity + signed/unsigned-integer-field-parseability
  // check, same DecodeFn contract as every other verb; every exec
  // function re-parses the same fields (decode already proved they
  // succeed) and forwards them to the Adapter as floats -- wire integer
  // -> float for arithmetic convenience.
  //
  // Sprint 008 (wire-timeout-hardening.md, R-06 + R-18): every one of
  // these six exec functions now runs its own `timeout`/`duration`
  // field through wire_handler.cpp's shared clampMotionTimeout() helper
  // BEFORE calling the Adapter -- 0 is refused (Result::kRange, matching
  // the existing `cruise <= 0` refusal precedent) and any value above
  // 2^31-1 is silently clamped down to it. This is deliberately in the
  // exec (not decode) phase: value-range refusal is a MERITS rejection
  // (ack + err), not a decode failure (nack) -- the line itself parses
  // fine; only its meaning is out of range, same class as the
  // cruise/speed <0/==0 handling WireAdapter::onWheelsX() et al. already
  // do. See wire_handler.cpp's kMaxMotionTimeoutMs/clampMotionTimeout()
  // for the full rationale. ----
  bool decodeWheelsX(char** fields, size_t fieldCount);
  void execWheelsX(char** fields, size_t fieldCount, uint32_t id,
                   uint8_t& errCode);

  bool decodeWheelsV(char** fields, size_t fieldCount);
  void execWheelsV(char** fields, size_t fieldCount, uint32_t id,
                   uint8_t& errCode);

  bool decodeMoveX(char** fields, size_t fieldCount);
  void execMoveX(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  bool decodeMoveV(char** fields, size_t fieldCount);
  void execMoveV(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  bool decodeGoToR(char** fields, size_t fieldCount);
  void execGoToR(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  // Identical field shape to GO_TO_R (motion-api.md S9.1) -- decodeGoToW
  // simply delegates to decodeGoToR; execGoToW is its own function only
  // because it must call onGoToW(), not onGoToR().
  bool decodeGoToW(char** fields, size_t fieldCount);
  void execGoToW(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  bool decodeStop(char** fields, size_t fieldCount);
  void execStop(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

  bool decodeRun(char** fields, size_t fieldCount);
  void execRun(char** fields, size_t fieldCount, uint32_t id,
              uint8_t& errCode);

  // Trivial stand-ins for HELLO/PING/ESTOP's own kCommandTable rows --
  // see kCommandTable's own comment for why these exist but are never
  // actually invoked.
  bool decodeAlwaysTrue(char** fields, size_t fieldCount);
  void execNoop(char** fields, size_t fieldCount, uint32_t id,
               uint8_t& errCode);

  // ---- telemetry header memo (protocol.md S5.2, sprint.md Phase B) --
  // headerChanged() decides whether emitTelemetry() owes a fresh
  // `thdr`; rememberHeader() then copies the just-emitted header's own
  // shape into headerNames_/headerHex_/headerCount_ so the NEXT call
  // has something to compare against. Deliberately a COPY, not a
  // borrowed pointer into the caller's own Snapshot -- the caller is
  // free to mutate or destroy its own Column array the instant
  // emitTelemetry() returns; this memo must not care. ----
  bool headerChanged(const Snapshot& snapshot) const;
  void rememberHeader(const Snapshot& snapshot);
  void emitHeader(const Snapshot& snapshot);  // "thdr <col>...\n"
  void emitFrame(const Snapshot& snapshot);   // "t <v>...\n"

  Adapter& adapter_;
  Sink& sink_;

  char lineBuf_[kMaxLineBytes] = {};
  size_t lineLen_ = 0;
  bool overflowing_ = false;
  uint32_t malformedCount_ = 0;

  // ---- the reliability layer's own state (protocol.md S8.1) -- one
  // integer plus one bool, and deliberately NO clock/timer.
  // `lastDone_` is NOT here: it lives on the Adapter (S8.8), polled
  // fresh on every ack/nack, never cached on this class. ----
  uint32_t expectedNext_ = 1;    // next sequence id expected from the host

  // gapOutstanding_ was DELETED 2026-08-26 (S8.5) along with the
  // telemetry ack piggyback, whose emitReliability() was its only
  // reader. It came back 2026-08-27, stakeholder-approved, and the
  // scope of its return matters more than its existence:
  //
  //   IT IS A REPLY PREDICATE ONLY.
  //
  // Its sole job is gating whether an inbound UNSEQUENCED line's reply
  // carries the reminder (see emitReminderIfStalled()). It does NOT
  // restore a periodic emission, a telemetry-carried ack/nack, or a
  // beacon of any kind -- all of that stays deleted and S8.5's
  // anti-beacon rule is untouched. An idle connection is still
  // COMPLETELY silent on both carriers: `feed()` remains the only
  // origin of every emission this class makes, and nothing here has a
  // clock.
  //
  // Approved with that scope stated, because re-adding it partially
  // undoes the stakeholder's own direction from the day before. The
  // requirement it serves: an unsequenced verb must be issuable at any
  // time with no id (that is the gating half, and it is absolute), but
  // it MAY still carry back "your last command didn't land" when
  // something is actually wrong. Sequence gating and reply emission are
  // separable, and only the first was ever objected to.
  //
  // It cannot be derived from expectedNext_ alone: that counter cannot
  // distinguish "clean, waiting for #5 which the host has not sent yet"
  // from "stalled, discarded #6, still want #5".
  bool gapOutstanding_ = false;

  // ---- telemetry header memo state (protocol.md S5.2) -- a COPY of
  // the most recently emitted header's shape, sized generously above
  // any realistic column set (sprint.md's own widest set, POSE+FULL,
  // is 20 columns; column names in this project are all <=6 chars) so
  // headerChanged()'s per-column comparison never has to worry about
  // storage running out for a real caller. A Snapshot wider than
  // kMaxHeaderColumns is treated as always-changed by headerChanged()
  // (a safe fallback -- see its own .cpp comment), never a buffer
  // overrun. ----
  static constexpr size_t kMaxHeaderColumns = 40;
  static constexpr size_t kMaxHeaderNameBytes = 16;
  char headerNames_[kMaxHeaderColumns][kMaxHeaderNameBytes] = {};
  bool headerHex_[kMaxHeaderColumns] = {};
  size_t headerCount_ = 0;
  bool everEmittedHeader_ = false;  // false until the very first thdr

  // The 20-frame (~1 Hz at this project's 50 ms emission cadence)
  // forced header refresh (sprint.md SUC-004) -- counts calls to
  // emitTelemetry(snapshot) since the last thdr was emitted (for ANY
  // reason: a real change, the very first call, or this same
  // staleness trigger), reset to 1 every time one goes out since the
  // call that emits it counts as the first frame of the next streak.
  static constexpr uint32_t kHeaderRefreshFrames = 20;
  uint32_t framesSinceHeader_ = 0;

  // Telemetry's own member-owned scratch buffer (never a stack local
  // in emitHeader()/emitFrame() -- see this file's own kMaxLineBytes
  // comment and sprint.md's Phase B formatting constraints: the
  // protocol fiber is 2 KB, and radio_transport.h:128 records a
  // measured hard-fault from exactly this mistake elsewhere in this
  // project). Sized identically to lineBuf_ (the wire's own 240-byte
  // line ceiling) rather than reused: lineBuf_ is RX-only reassembly
  // state (fed byte-by-byte from feed()), and aliasing an unrelated TX
  // formatting buffer onto it would be a correctness landmine for a
  // future edit, not a real memory saving.
  char emitBuf_[kMaxLineBytes] = {};
};

}  // namespace Wire
