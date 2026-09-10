// wire_handler.h -- Wire::WireHandler: protocol v6's ASCII line-grammar
// mechanics plus the reliability layer (radio-robot-lib/docs/design/
// protocol.md S2, S2.1, S3.1, S3.2, S8, S8.9 -- the canonical spec;
// this project conforms to that grammar, it does not vendor
// radio-robot-lib's C++). feed() reassembles arbitrary byte blocks into
// '\n'-terminated lines, tokenizes each line in place on runs of ' '
// (no allocation, no std::string -- S3.2), enforces case-as-direction
// (S2.1: commands UPPERCASE, replies lowercase, verb lookup
// case-SENSITIVE), and dispatches the seven unsequenced verbs (HELLO,
// PING, ESTOP, HELP, ID, VER, STATUS -- S8.3) plus the sequenced ones
// (GET, SET, TLM, STOP, RUN and the six motion verbs).
//
// A VERB CARRIES A SEQUENCE ID IFF ITS CORRECTNESS DEPENDS ON ITS
// POSITION IN THE STREAM -- either executing it twice changes the
// robot, or answering it out of order gives a wrong answer. ID/VER/
// STATUS answer session constants and HELLO/PING/HELP are liveness/
// orientation verbs, so all seven are position-independent and
// maximally forgiving of trailing content. GET is read-only but
// ORDERED by SET, so it stays sequenced.
//
// ---- The reliability layer (S8) ----
//
// Every sequenced verb carries a MANDATORY trailing `#<n>`, strictly
// incrementing from 1. Handler state is expectedNext_ plus
// gapOutstanding_ (a reply predicate -- see its own comment below);
// deliberately no clock and no timer anywhere (S8.1), so feed() stays a
// pure function of its input bytes plus that small state. dispatch()
// resolves the id FIRST, against expectedNext_ (S8.1's table):
//   - id == expectedNext_ : decode the verb's own fields FIRST (S8.9);
//     only if decoding succeeds does the sequence advance
//     (expectedNext_ = id + 1) and `ack <id> <lastDone> <reason>` go
//     out. A decode failure -- unrecognized verb, wrong arity, an
//     unparseable field -- does NOT advance: it replies
//     `nack <expectedNext_> <lastDone> <reason>` (naming the SAME id,
//     since it was never accepted) plus `err <code> #<id>`, and every
//     subsequent inbound line re-triggers that nack until a well-formed
//     line finally supplies that id. There is no periodic re-nack. A
//     corrupted leg of a multi-leg routine is resent, not silently
//     skipped.
//   - id < expectedNext_ : a stale retransmit -- the host never saw our
//     ack for something we already accepted. Do NOT re-execute; reply
//     `ack <expectedNext_ - 1>`, the already-accepted id, not the
//     resent one. `#0` needs no special case here: expectedNext_ starts
//     at (and never goes below) 1, so an inbound `#0` is
//     unconditionally below it.
//   - id > expectedNext_ : a numeric gap -- discard, do NOT execute, do
//     not even look up the verb, and reply `nack <expectedNext_>`. A
//     gap stalls the stream ON PURPOSE until the missing id arrives.
// A MERITS rejection -- the verb decoded fine but the Adapter refuses
// its content -- is a DIFFERENT case: it ACKS and ADVANCES (the line
// arrived intact), paired with `err <code> #<id>` on top of that ack.
// S8.9 keeps decode failure and merits rejection sharply distinct.
//
// `<lastDone>`/`<reason>` are read FRESH off Adapter::lastDone()/
// lastDoneReason() every time an ack/nack is formatted (S8.8) -- no
// cached copy anywhere in this class. `err <code> #<id>` orders code
// first, id last (S8.6): the id is always a line's LAST token, commands
// and replies alike.
//
// HELLO additionally resets expectedNext_ (a (re)connecting host's own
// resync point) but does NOT touch the Adapter's lastDone()/
// lastDoneReason() -- that state is Adapter-owned and a handler-level
// reset has no business reaching into it (S8.8).
//
// Angles (rotation, omega) are milliradian integers on the wire
// (motion-api.md S9.1), decoded here with the ordinary signed-integer
// field parser; the degrees-at-the-API conversion is a LANGUAGE
// BINDING's job, not this file's.
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
// adapter owns the storage -- a string literal, a robot-config field,
// or (sprint 037) a Protocol-owned buffer seeded from a literal at
// construction and mutable at runtime via a setter (`role`/
// `commonName` via `Protocol::setDeviceRole()`) -- and must keep it
// alive at least until the identity() call that requested it returns.
// `name` (read from silicon) is the authoritative board-identity
// field, NOT `profile` (build provenance) -- see protocol.cpp's own
// kProfile comment.
struct Identity {
  const char* name = "";
  const char* serial = "";
  const char* drivetrain = "";
  const char* role = "";
  const char* commonName = "";
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
  // The I2C fault counter: a wedged or unpowered Nezha brick shows up
  // here as a climbing count, not merely as the boolean `wedge` flag.
  // Sourced by WireAdapter::status() from the SAME diagValue(8) call
  // the telemetry `i2cf` column reads, so the two can never disagree.
  // Decimal on the wire (execStatus()'s `i2cf=%ld`), unlike `flags`'
  // hex -- a raw fault count has no bitfield meaning to pack.
  int32_t i2cf = 0;
  // The kernel's own heartbeat counter, and the discriminator between
  // "this kernel has never ticked" and "the brick is unreachable": both
  // used to report an IDENTICAL STATUS line (ready=0 connL=0 connR=0
  // i2cf=0), because ready/connL/connR/i2cf are only ever written from
  // inside step()/collect(), which never ran either way. cyc == 0 means
  // every other field's 0 is meaningless, not a fault; nonzero means
  // the kernel is running and every other field means what it says.
  // Same single-source guarantee as `i2cf` above -- diagValue(16),
  // shared with the telemetry `cyc` column. Unsigned decimal on the
  // wire: a cycle count never goes negative.
  uint32_t cyc = 0;
  const char* tlm = "off";
};

// One named, already-scaled telemetry value (protocol.md S5.2: `thdr
// <col>...` then `t <v>...`). `value` is always an already-scaled plain
// integer -- this type has no opinion on what a column MEANS, only on
// how it prints: `hex` picks lowercase hex with no `0x` prefix
// (flags-shaped columns); everything else prints signed base-10.
//
// Explicit ctor: the NSDMIs make this a non-aggregate under the target's
// -std=c++11, so the `{"name", value, hex}` call sites need one; the
// C++20 host would compile without it. Dropping the NSDMIs instead
// would leave every default-constructed `Column columns_[...]` holding
// indeterminate values until every element is filled.
struct Column {
  const char* name = "";
  int32_t value = 0;
  bool hex = false;

  Column() = default;
  Column(const char* name_, int32_t value_, bool hex_)
      : name(name_), value(value_), hex(hex_) {}
};

// One telemetry frame's worth of columns (protocol.md S5.2). `columns`
// is BORROWED: the caller owns the backing array and must keep it alive
// only for the duration of the emitTelemetry(snapshot) call it is
// passed to -- WireHandler copies what it needs for its own header memo
// (kMaxHeaderColumns/kMaxHeaderNameBytes below) and formats the rest
// immediately, keeping no borrowed pointer alive past that one call.
// Shares Column's NSDMI shape and is therefore also a non-aggregate
// under -std=c++11, but no site anywhere brace-initializes a Snapshot,
// so it needs no constructor of its own.
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
  kWriteOnly,      // -> err 12 #<id>  ERR_WRITE_ONLY
};

// ERR_WRITE_ONLY (12): "that name exists, and it cannot be read." The
// reference grammar numbers its own codes 1-11 (11, ERR_DUPLICATE_ID,
// is deleted but still spent), so 12 is the first number free of that
// range rather than one of the 5/7/9 holes inside it -- a hole there is
// a code an older host may already have an opinion about. Without it,
// an advertised field answering the SAME err 1 a typo gets tells the
// operator nothing about which of the two just happened. `rebase` is
// the one such field today: a write-triggered action with no stored
// value behind it, refused rather than answered with a manufactured 0
// (WireAdapter::onGet()).
constexpr uint8_t kErrWriteOnly = 12;

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
// Every sequenced verb's ack/nack piggybacks this pair, not only the
// motion ones.
//
// kStall's wire spelling matches the kernel's own stall latch
// (`stallHalted`/`stall_clear`) rather than inventing a second notion
// of "stalled"; it is deliberately NOT folded into kAborted (a stalled
// drivetrain and a superseded command are different failure classes a
// host needs to tell apart) or kEstop (stall is drivetrain-local, not
// the same safety condition). Produced by
// diffDrive::WireAdapter::resolvePendingReason().
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
  // Reads one config field. Exactly three answers are legal, and they
  // are distinguishable BECAUSE a bool could not tell the last two
  // apart:
  //   kOk        -- `out` holds the value; the caller emits `get`.
  //   kUnknown   -- no such name (a typo).
  //   kWriteOnly -- the name exists but has nothing readable behind it
  //                 (a write-triggered action, not a stored value).
  // A bare GET dumps only the kOk names; a named GET reports the other
  // two as their own error codes.
  virtual Result onGet(const char* name, float& out) const = 0;
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

  // ---- the RUN registry, made readable (protocol.md's FUNCS section)
  // -- the shape fieldCount()/fieldName() give the config surface, and
  // walked by execFuncs() the way a bare GET walks that one. This class
  // still holds no function table; it discloses the adapter's.
  // runCount() 0 is VALID (an empty allowlist). runName(i)/
  // runSignature(i) are borrowed, never null, "" out of range; a
  // signature is an OPTIONAL single token nothing parses. Both are
  // sanitized before the sink -- neither may forge a wire line. ----
  virtual size_t runCount() const = 0;
  virtual const char* runName(size_t index) const = 0;
  virtual const char* runSignature(size_t index) const = 0;

  // ---- the WiFi credential store, made readable -- the SAME shape
  // as the RUN registry seam directly above:
  // adapter-declared count, walked by execWifiCred() the way execFuncs()
  // walks runCount()/runName()/runSignature(). This class still holds
  // no credential table of its own; it discloses the adapter's.
  //
  // wifiCredCount() is the store's FIXED slot count (NOT the occupied
  // count) -- index IS the slot number 1:1, so execWifiCred() can print
  // it straight through with no separate slot-number channel. A slot
  // that is not occupied answers false (skip it -- the enumeration
  // lists occupied slots only, same as execFuncs() skipping an
  // unnamed RunRegistry entry); an occupied slot answers true with
  // ssidOut filled (borrowed capacity ssidCap, always NUL-terminated)
  // and hasPasswordOut set. NEVER the password itself -- this mirrors
  // WifiCredentialStore's own get()/hasPassword() split
  // (wifi_credential_store.h's header comment) precisely so the wire
  // layer cannot accidentally wire up the wrong one; hasPassword() is
  // the only accessor this seam may call.
  //
  // wifiCredSet()/wifiCredClear() mutate one slot and report the
  // outcome as an ordinary Result -- kRange for an out-of-range slot
  // or a string that does not fit (WifiCredentialStore::set() rejects
  // rather than truncates; see that file's header comment), kOk
  // otherwise. Both are on the SEQUENCED half of this verb -- see
  // kCommandTable's own WIFICRED row comment (wire_handler.cpp) for
  // why. ----
  virtual size_t wifiCredCount() const = 0;
  virtual bool wifiCredSlot(size_t index, char* ssidOut, size_t ssidCap,
                            bool& hasPasswordOut) const = 0;
  virtual Result wifiCredSet(int slot, const char* ssid,
                            const char* password) = 0;
  virtual Result wifiCredClear(int slot) = 0;
};

class WireHandler {
 public:
  // Wire line ceiling, protocol.md S2: "Max line: 240 bytes including
  // the terminator." The handler sizes its buffer off this one
  // constant so the number is spelled exactly once.
  static constexpr size_t kMaxLineBytes = 240;

  // The sequence space's ceiling. `expectedNext_` may REACH this value
  // -- it is what "the last legal id has been accepted" looks like --
  // but no inbound line may CARRY it, so `expectedNext_ = id + 1` can
  // never wrap. Legal ids are therefore [1, kMaxSequenceId - 1].
  // Without the reservation, accepting #4294967295 sets expectedNext_
  // to 0: a value no id can equal or fall below, so every subsequent
  // line nacks asking for id 0 (not legal either) and
  // `replyAck(expectedNext_ - 1)` underflows on top.
  static constexpr uint32_t kMaxSequenceId = 0xFFFFFFFFu;

  // Whether an inbound line's `#<id>` may be executed at all. Public
  // and static so a host test can drive the boundary directly: the
  // states around it are otherwise reachable only by sending four
  // billion commands. The sequence space's OTHER illegal id, `0`, is
  // not this predicate's business -- dispatch() answers it earlier, and
  // differently.
  static bool sequenceIdIsExecutable(uint32_t id) {
    return id != kMaxSequenceId;
  }

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
  // Embedded NULs (radio-robot-lib's protocol.md S9.4): every
  // wire-touching comparison here runs on a NUL-terminated C string,
  // per the no-allocation, no-std::string constraint (S3.2), so a NUL
  // anywhere inside a line acts as an early terminator for THAT line --
  // "PING\0extra\n" dispatches exactly like a bare "PING\n", silently
  // discarding "extra" with no malformed-count increment. A pinned
  // characterization, not a bug: a real fix would mean abandoning
  // C-string comparisons throughout the parser. The one exception -- a
  // line whose first non-space byte IS the NUL -- would otherwise leave
  // the tokenizer's own token array uninitialized (a real memory-safety
  // hazard, not just a surprising parse), so onLineComplete() guards it
  // explicitly and counts it malformed instead.
  void feed(const char* data, size_t length);

  // HELLO's reply, byte-identical to the unsolicited boot banner a
  // caller would send at connect time (protocol.md S4/S6):
  // "device NEZHA2 robot <name> <serial>\n".
  void sendBanner();

  // The telemetry frame (protocol.md S5.2): emits, in order, as TWO
  // separate Sink::write() calls (never concatenated into one) --
  //   1. `thdr <col>...\n`, but only when a fresh header is DUE (see
  //      below);
  //   2. `t <v>...\n`, always, one value per column in `snapshot`, in
  //      the same order as the most recently emitted header.
  // NOTHING ELSE (protocol.md S8.5): an ack/nack is only ever a direct
  // reply to an inbound sequenced line, never a beacon. A subscriber
  // that wants to know whether its last command landed sends a command
  // (e.g. STATUS) and reads that command's own ack.
  //
  // A fresh header is DUE when: this is the very first call ever made
  // on this instance; the column set changed since the last header
  // (count, any column's name, OR any column's hex-ness -- a memo that
  // compared only names/count would miss a hex-ness-only flip); or
  // kHeaderRefreshFrames calls have elapsed, whichever comes first.
  // That last case is what keeps a late-attaching listener over a lossy
  // broadcast radio from being permanently locked out of decoding; it
  // has nothing to do with the column set changing at all.
  //
  // `snapshot`'s backing array is borrowed only for the duration of
  // this call; this class copies what it needs of it (the header memo)
  // and touches nothing else afterward.
  void emitTelemetry(const Snapshot& snapshot);

  // Lines dropped as: an unrecognized verb or one this file does not
  // (yet) implement, wrong arity, an unparseable field, a sequenced
  // verb whose mandatory id was missing or malformed, or an overlong
  // line. This INCLUDES a decode failure on an in-order sequenced id
  // (S8.9) -- unlike a numeric gap or a stale retransmit, NEITHER of
  // which is ever counted here, since neither one's content is even
  // inspected. A lowercase-led inbound verb (another robot's reply
  // overheard on a shared channel, S2.1) and a blank line are dropped
  // silently and do NOT increment this.
  uint32_t malformedCount() const { return malformedCount_; }

  // Builds "help" plus a space-separated `name` for every entry in
  // `names` (`nameCount` entries) into `buf` (capacity `bufCap`),
  // followed by '\n'. The terminator is written LAST but into a byte
  // the content-filling loop is structurally forbidden to reach, so it
  // is always the LISTED NAMES that truncate on overflow, never the
  // terminator -- the same reserve-two-bytes shape emitHeader()/
  // emitFrame() follow, handing the terminator to terminateEmitBuf().
  // Returns bytes written, excluding the closing NUL. Public and static
  // purely so a host test can drive it with a synthetic, arbitrarily
  // long name list; kCommandTable is far too small to exercise the
  // truncation path this proves safe. kHelpChunkBytes is the longest
  // `help ...` line emitHelp() will produce before starting a new one,
  // deliberately well under kMaxLineBytes.
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
  //
  // A token whose first byte is '"' is a QUOTED token (WIFICRED SET's
  // <ssid>/<password>, the only fields that may contain spaces): reads
  // to the next '"' followed by a separator/EOL, decoding `\"` to a
  // literal '"' (the only escape). Only a LEADING '"' triggers this, so
  // every other verb's fields -- always keywords or integers -- are
  // unaffected. An unterminated quote swallows the rest of the line
  // (id included), which just starves the verb of a data field; the id
  // itself is unaffected since it is resolved from the raw line before
  // this runs (findLastFieldToken()), and the arity check downstream
  // (e.g. decodeWifiCred()) rejects the short field count normally.
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
  // (S8.9). Returns false (a DECODE FAILURE) for wrong arity or an
  // unparseable field; true otherwise. `fields`/`fieldCount` here
  // EXCLUDE the id (already resolved and stripped by dispatch()).
  using DecodeFn = bool (WireHandler::*)(char** fields, size_t fieldCount);

  // Every EXECUTE function runs ONLY after dispatch() has decided the
  // line decodes AND has already sent its `ack` -- so it is free to
  // write informational reply lines (id/ver/status/help/get/ret)
  // straight to the sink; nothing it does can race the ack that must
  // precede them. It reports any ADAPTER-level (merits) rejection
  // through `errCode` (0 == kOk == no err line; nonzero == the wire
  // code dispatch() emits as `err <errCode> #<id>` right after).
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
  // at (S8.3). They are still present purely so HELP's generated
  // listing (execHelp()) walks ONE table for every verb name this file
  // knows about and cannot drift from the dispatcher.
  //
  // Deliberately declared with NO explicit size: the definition in
  // wire_handler.cpp supplies the bound, deduced from its own
  // initializer list. An explicit size meant the count was spelled
  // twice, and *removing* a row from the .cpp's initializer compiled
  // SILENTLY -- the array zero-filled the vacated slot, that entry's
  // `name` read back nullptr, and the first line that walked the table
  // into it hit `strcmp(verb, nullptr)`: UB, a hard fault in practice,
  // on every later unrecognized-verb line or HELP call (the two paths
  // that walk the whole table). The constructor's own static_assert
  // (wire_handler.cpp) pins the expected count, so an accidental
  // removal fails to COMPILE instead of silently shipping.
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
  // the NUL), sized so the WHOLE reply line -- "ret " + this text +
  // " #<id>" at id's maximum width + '\n' -- can never exceed
  // kMaxLineBytes, even before execRun()'s own sanitize pass (which can
  // only shrink the text).
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
  // here is a plain arity + integer-field-parseability check, same
  // DecodeFn contract as every other verb; every exec re-parses the
  // same fields (decode already proved they succeed) and forwards them
  // to the Adapter as floats.
  //
  // Every one of the six execs runs its own `timeout`/`duration` field
  // through clampMotionTimeout() (wire_handler.cpp) BEFORE calling the
  // Adapter: 0 is refused (Result::kRange, matching the `cruise <= 0`
  // refusal precedent) and anything above 2^31-1 is silently clamped.
  // Deliberately in the exec, not the decode phase -- a value-range
  // refusal is a MERITS rejection (ack + err), not a decode failure
  // (nack): the line parses fine, only its meaning is out of range. ----
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

  // No data fields (shares decodeNoFields with ID/VER/STATUS/HELP), but
  // SEQUENCED unlike those four: its reply is variable-length and the
  // ack is the terminator. See execFuncs().
  void execFuncs(char** fields, size_t fieldCount, uint32_t id,
                uint8_t& errCode);

  // WIFICRED -- bare enumeration OR a `SET`/
  // `CLEAR` sub-verb in fields[0], NOT three separate kCommandTable
  // rows: the bare form's own arity (zero data fields) already needs
  // its own case, so folding SET/CLEAR's field-count checks into the
  // same decode function costs nothing extra and keeps one verb name
  // on the wire for the whole feature, matching how GET's bare vs.
  // named forms already share one row. decodeWifiCred() only checks
  // SHAPE (which sub-verb, how many fields, does the slot parse as an
  // integer) -- see execWifiCred()'s own comment for why the slot
  // RANGE check and the ssid/password LENGTH check both live in exec,
  // not here.
  bool decodeWifiCred(char** fields, size_t fieldCount);
  void execWifiCred(char** fields, size_t fieldCount, uint32_t id,
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
  // Writes the '\n' and the NUL that end every emitted telemetry line,
  // into the two bytes the two functions above reserve by stopping
  // their content at sizeof(emitBuf_) - 2. See its definition
  // (wire_handler.cpp) for what a missing terminator costs downstream.
  void terminateEmitBuf(size_t contentLength);

  Adapter& adapter_;
  Sink& sink_;

  char lineBuf_[kMaxLineBytes] = {};
  size_t lineLen_ = 0;
  bool overflowing_ = false;
  uint32_t malformedCount_ = 0;

  // ---- the reliability layer's own state (protocol.md S8.1) -- one
  // integer plus one bool, and deliberately NO clock/timer. `lastDone_`
  // is NOT here: it lives on the Adapter (S8.8), polled fresh on every
  // ack/nack, never cached on this class. ----
  uint32_t expectedNext_ = 1;    // next sequence id expected from the host

  // A REPLY PREDICATE, and nothing more. Set when a numeric gap or a
  // decode failure stalls the stream (dispatch()/handleDecodeFailure()),
  // cleared by an accepted in-order line and by HELLO. Its one reader is
  // emitReminderIfStalled(), which decides whether an inbound
  // UNSEQUENCED verb's reply also carries `nack <expectedNext_>`, so an
  // operator issuing an unsequenced verb at any time still learns that
  // an earlier numbered command did not land. It restores no periodic
  // emission, no telemetry-carried ack/nack and no beacon of any kind
  // (S8.5): feed() remains the only origin of every emission this class
  // makes, nothing here has a clock, and an idle connection stays
  // COMPLETELY silent on both carriers. It cannot be derived from
  // expectedNext_ alone -- that counter cannot distinguish "clean,
  // waiting for #5 which the host has not sent yet" from "stalled,
  // discarded #6, still want #5".
  bool gapOutstanding_ = false;

  // ---- telemetry header memo state (protocol.md S5.2) -- a COPY of
  // the most recently emitted header's shape, sized generously above
  // any realistic column set (the widest set, POSE+FULL, is 20
  // columns; column names in this project are all <=6 chars) so
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
  // forced header refresh -- counts calls to
  // emitTelemetry(snapshot) since the last thdr was emitted (for ANY
  // reason: a real change, the very first call, or this same
  // staleness trigger), reset to 1 every time one goes out since the
  // call that emits it counts as the first frame of the next streak.
  static constexpr uint32_t kHeaderRefreshFrames = 20;
  uint32_t framesSinceHeader_ = 0;

  // Telemetry's own member-owned scratch buffer (never a stack local in
  // emitHeader()/emitFrame(): the protocol fiber is 2 KB, and
  // radio_transport.h records a measured hard-fault from exactly this
  // mistake elsewhere in this project). Sized identically to lineBuf_
  // (the wire's own 240-byte line ceiling) rather than reused --
  // lineBuf_ is RX-only reassembly state fed byte-by-byte from feed(),
  // and aliasing an unrelated TX formatting buffer onto it would be a
  // correctness landmine for a future edit, not a real memory saving.
  char emitBuf_[kMaxLineBytes] = {};

  // RUN's own reply-body scratch, same reasoning applied to execRun():
  // a member instead of a stack local, so this array's storage never
  // adds to that function's frame. argv[] stays a plain local there --
  // it is genuinely needed as the adapter call's own argument, before
  // any early-return point exists ahead of it.
  char runResult_[kMaxRunResultBytes] = {};
};

}  // namespace Wire
