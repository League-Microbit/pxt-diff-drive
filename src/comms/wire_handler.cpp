// wire_handler.cpp -- see wire_handler.h for the full contract this
// file implements. Shape ported from radio-robot-lib's own
// protocol_handler.cpp feed()/tokenizeLine()/dispatch() skeleton
// (radio-robot-lib/docs/design/protocol.md S2-S3, S8) -- this is not a
// vendored copy.
#include "wire_handler.h"

#include <cerrno>
#include <cmath>
#include <cstdio>   // plain snprintf, not std::snprintf: this ARM cross
                    // compiler's newlib-nano <cstdio> declares snprintf
                    // globally but never puts it in namespace std --
                    // same gotcha protocol.cpp documents for its own
                    // snprintf call. Same story for strtof() below --
                    // every OTHER std:: function this file uses
                    // (strtol/strtoul/isnan/isinf/strcmp/strlen/memcpy)
                    // genuinely is in namespace std on this toolchain;
                    // only these two are not.
#include <cstdlib>
#include <cstring>

namespace Wire {

namespace {

// strtoul skips leading whitespace and, absent a digits-only pre-scan,
// would accept a leading '+' as valid syntax -- neither is a well-formed
// id (protocol.md S2.2: "the digits are bare and unsigned"). A pre-pass
// that requires EVERY byte to be an ASCII digit before strtoul() ever
// runs means "#+5", "#-5", and "# 5" are all correctly rejected as
// not-an-id, rather than silently parsing a prefix.
bool parseIdDigits(const char* text, uint32_t& out) {
  if (text == nullptr || text[0] == '\0') return false;
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p < '0' || *p > '9') return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  unsigned long value = std::strtoul(text, &endPtr, 10);
  if (endPtr == text || *endPtr != '\0') return false;
  if (errno == ERANGE || value > UINT32_MAX) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

// Resolves `token` (the line's raw last token, or nullptr if the line
// was just the verb) as a mandatory sequence id (protocol.md S2.2/S8):
// must be present and match `#[0-9]+` exactly. There is no "#0 is
// special" branch here at all -- every well-formed id, including 0, is
// handled identically by dispatch()'s own three-way sequence compare.
bool parseMandatoryId(const char* token, uint32_t& id) {
  if (token == nullptr || token[0] != '#') return false;
  return parseIdDigits(token + 1, id);
}

// The raw LAST token of `line` (verb included), independent of any
// fixed-size fields[] array's own storage cap.
//
// MUST be called BEFORE tokenizeLine() mutates any of `line`'s
// separator spaces to '\0': it walks real ' ' bytes backward from the
// end of the string. Returns nullptr if `line` has no token besides the
// verb itself (nothing after it to resolve an id from).
const char* findLastFieldToken(const char* line) {
  const char* end = line + std::strlen(line);
  const char* p = end;
  while (p > line && *(p - 1) == ' ') --p;  // skip trailing spaces
  while (p > line && *(p - 1) != ' ') --p;  // scan back through the token
  return p == line ? nullptr : p;
}

// Config values (SET) and the six motion verbs' own fields (motion-
// api.md S9.1) are the places non-id numeric fields appear on the wire.
// "No exponents, no NaN, no inf" -- nothing in this project ever needs a
// robot to accept "1e10" or "nan" as a gain; tokenizing on ' ' still
// leaves '\t'/'\v'/'\f'/'\r' as LEGAL, ordinary field bytes that
// strtol/strtoul/strtof would otherwise silently skip as leading
// whitespace (a C-standard behavior, not a project choice).
bool isWireSpace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' ||
         c == '\r';
}

// The six motion verbs' own fields are base-10 integers, optionally
// signed for left/right/distance/rotation/v_x/omega/x/y/speed/arrive,
// unsigned for timeout/duration. "Strict" means the WHOLE field must be
// consumed by strtol/strtoul -- a trailing letter or stray interior byte
// makes the field unparseable ("wrong arity is a rejection, not a
// best-effort parse" extended to field content).
bool parseInt32(const char* field, int32_t& out) {
  if (field == nullptr || field[0] == '\0' || isWireSpace(field[0])) {
    return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  long value = std::strtol(field, &endPtr, 10);
  if (endPtr == field || *endPtr != '\0') return false;
  if (errno == ERANGE || value < INT32_MIN || value > INT32_MAX) return false;
  out = static_cast<int32_t>(value);
  return true;
}

// strtoul silently accepts a leading '-' and wraps around, which would
// turn "-5" into a huge unsigned value instead of failing -- reject it
// up front. Deliberately NOT as strict as parseIdDigits() above (which
// also bars a leading '+'): the id's own grammar is `#[0-9]+` exactly,
// but timeout/duration are ordinary signed-integer-family wire fields
// with no such narrower rule of their own.
bool parseUint32(const char* field, uint32_t& out) {
  if (field == nullptr || field[0] == '\0' || field[0] == '-' ||
      isWireSpace(field[0])) {
    return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  unsigned long value = std::strtoul(field, &endPtr, 10);
  if (endPtr == field || *endPtr != '\0') return false;
  if (errno == ERANGE || value > UINT32_MAX) return false;
  out = static_cast<uint32_t>(value);
  return true;
}

// Sprint 008 (wire-timeout-hardening.md, R-06 + R-18, code review
// 2026-08-23): the shared ceiling every one of the six motion verbs'
// `timeout`/`duration` field is clamped against, in each exec function
// below, BEFORE the value ever reaches the Adapter (WireAdapter's own
// obligation-window math, MotionEngine::wheelsX()'s lease-clamp
// arithmetic, and the kernel's own lease/deadline math therefore never
// see an out-of-range value -- none of them needs its own defensive
// check).
//
// This is a SIBLING of wire_adapter.h's kWireBoundaryCastCeiling
// (2e9), not a reuse of it, and deliberately so: that constant bounds a
// float->int32 CAST at the wire boundary (chosen with headroom below
// where float's 24-bit mantissa stops representing every integer
// exactly, so the cast itself stays well-defined) -- this one bounds a
// uint32_t value directly, with no float involved anywhere in its own
// path. 2^31-1 is exactly this project's own signed-difference
// wraparound-safe half-range -- the same idiom
// WireAdapter::hasLiveMotionObligation() already relies on
// (`static_cast<int32_t>(nowMs - deadlineMs) < 0`): a `timeout` at or
// below this ceiling can never make `now + timeout` wrap PAST `now`
// itself, so that comparison stays correct for any `now`. Reusing
// kWireBoundaryCastCeiling's literal value (2e9, not 2^31-1) here would
// leave ~147M ms of headroom where the wraparound-safety guarantee
// above no longer holds; reusing the symbol itself would mean this file
// including wire_adapter.h, inverting this project's own layering rule
// (src/DESIGN.md S1: wire_adapter depends on wire_handler, never the
// reverse -- this file stays host-portable with no project includes at
// all). The two ceilings are close in magnitude only because "very
// large but still safe" happens to land in the same neighborhood in
// both domains.
constexpr uint32_t kMaxMotionTimeout = 2147483647u;  // 2^31 - 1

// Applied identically to WHEELS_X/WHEELS_V/MOVE_X/MOVE_V/GO_TO_R/GO_TO_W's
// own timeout/duration field (see kMaxMotionTimeout's own doc comment
// above for why, and DESIGN.md S14's Design Rationale for the
// reject-vs-clamp choice on each end): `0` is refused outright
// (matching the existing precedent that `cruise <= 0` already refuses
// rather than silently reinterpreting a nonsensical input, WireAdapter
// ::onWheelsX() et al.) -- both of today's two disagreeing "0" meanings
// (WHEELS_X's stale-lease lurch, MOVE_X's instant no-op) are confirmed
// bugs, not designs worth preserving. A value above the ceiling is
// silently clamped down to it: a host sending an oversized timeout is
// asking for "run for a very long time," which clamping serves;
// rejecting would force every large-sentinel-using host to learn this
// project's specific ceiling. Returns false (reject) for exactly 0,
// leaving `timeout` unmodified; otherwise clamps `timeout` in place to
// at most kMaxMotionTimeout and returns true.
bool clampMotionTimeout(uint32_t& timeout) {
  if (timeout == 0) return false;
  if (timeout > kMaxMotionTimeout) timeout = kMaxMotionTimeout;
  return true;
}

bool parseFloatField(const char* field, float& out) {
  if (field == nullptr || field[0] == '\0' || isWireSpace(field[0])) {
    return false;
  }
  for (const char* p = field; *p != '\0'; ++p) {
    // 'e'/'E' bars decimal-exponent notation ("1e10"). 'x'/'X' bars C99
    // hex float notation ("0x1p3") -- strtof accepts this syntax
    // unconditionally otherwise.
    if (*p == 'e' || *p == 'E' || *p == 'x' || *p == 'X') return false;
  }
  char* endPtr = nullptr;
  errno = 0;
  float value = strtof(field, &endPtr);
  if (endPtr == field || *endPtr != '\0') return false;
  if (std::isnan(value) || std::isinf(value)) return false;
  out = value;
  return true;
}

bool parseTlmMode(const char* field, TlmMode& mode) {
  struct ModeEntry {
    const char* name;
    TlmMode mode;
  };
  static constexpr ModeEntry kModes[] = {
      {"OFF", TlmMode::kOff},   {"POSE", TlmMode::kPose},
      {"FULL", TlmMode::kFull}, {"NOW", TlmMode::kNow},
      {"AUTO", TlmMode::kAuto}, {"BUFFER", TlmMode::kBuffer},
  };
  for (const auto& entry : kModes) {
    if (std::strcmp(field, entry.name) == 0) {
      mode = entry.mode;
      return true;
    }
  }
  return false;
}

// formatConfigValue()'s own bound on the INPUT magnitude, applied BEFORE
// scaling -- NOT a post-scale clamp on the scaled product, which is the
// defect this constant closes (get-full-duty-velocity-returns-
// garbage.md). The OLD code scaled in a `uint32_t` intermediate and
// clamped THAT to `kMaxScaled` (~UINT32_MAX): since `uint32_t` cannot
// represent `magnitude * 1,000,000` for any magnitude past ~4295 no
// matter where inside its own range the clamp threshold sits, EVERY
// field whose real magnitude reached that line clamped to the exact
// same wrong constant (4294.967040) -- fullDutyVelocity (10795.0) was
// simply the first of today's 18 kFields entries to cross it, not a
// field-specific defect (confirmed by reading every seeded Config value
// in shims.cpp's ensure()).
//
// Mirroring kWireBoundaryCastCeiling's own doc-comment style
// (wire_adapter.h) but deliberately a SIBLING constant, not a reuse:
// that one bounds a float->int32_t CAST at the SET/inbound boundary;
// this one bounds the magnitude a GET reply is willing to report at
// all, entirely on the OUTBOUND side, and reusing the adapter's own
// symbol would mean this host-portable, no-project-includes file
// (src/DESIGN.md S4) including wire_adapter.h, inverting this project's
// wire_adapter-depends-on-wire_handler layering rule -- same reasoning
// kMaxMotionTimeout's own doc comment above already applies to a
// different pair of ceilings. 1,000,000.0f is chosen with two orders of
// magnitude of headroom above this project's largest real config value
// (fullDutyVelocity, 10795.0 counts/s) while keeping the scaled product
// (`kGetValueCeiling * kDivisor` == 1e12) comfortably inside `double`'s
// exact-integer range (2^53, ~9.007e15) -- see formatConfigValue()'s own
// comment below for why that headroom is what makes the wide
// intermediate safe. A magnitude beyond this ceiling is CLAMPED to the
// ceiling itself: a suspiciously round, always-identical, documented
// number no real configured value could ever coincide with -- standing
// in clear contrast to the old bug's plausible-looking wrong digits,
// and honest about "this is a saturation flag" the instant an operator
// notices the same round value on more than one field.
constexpr float kGetValueCeiling = 1000000.0f;  // 1e6

// formatConfigValue() -- six fractional digits, always present, no
// exponent, using integer arithmetic because newlib-nano's printf (the
// eventual firmware target) has no %f. formatConfigValue(0.02f) ->
// "0.020000", formatConfigValue(-51.5f) -> "-51.500000".
//
// `value` is NOT wire-parsed here -- it is whatever the ADAPTER's own
// onGet() handed back (parseFloatField already rejects NaN/Inf on the
// way IN), so this function cannot assume it is finite. +-Inf is already
// handled correctly below: `magnitude` compares greater than
// kGetValueCeiling and gets clamped before scaling ever runs. NaN does
// not: every comparison against a NaN is false, so the ceiling clamp
// would never trigger for one -- there is no wire spelling for NaN, so
// fail safe to 0.0 rather than invent one, exactly as before.
//
// The scaling intermediate is `double`, not `uint32_t` -- see
// kGetValueCeiling's own comment above for why that pairing (a bounded
// input, a wide intermediate) closes the overflow rather than merely
// relocating it. `double` exactly represents every integer this
// function's own bounded `scaled` can reach, so no precision is lost by
// widening; the final narrowing to `uint32_t` (wholePart/fracPart, each
// individually far under UINT32_MAX once magnitude is bounded) is what
// stays safe to print via `%lu` on this project's own embedded target.
void formatConfigValue(float value, char* out, size_t cap) {
  if (std::isnan(value)) value = 0.0f;
  constexpr uint32_t kDivisor = 1000000u;  // 10^6 -- six fixed digits
  const bool negative = value < 0.0f;
  float magnitude = negative ? -value : value;
  if (magnitude > kGetValueCeiling) magnitude = kGetValueCeiling;
  const double scaled =
      static_cast<double>(magnitude) * static_cast<double>(kDivisor) + 0.5;
  const uint64_t scaledInt = static_cast<uint64_t>(scaled);
  const uint32_t wholePart = static_cast<uint32_t>(scaledInt / kDivisor);
  const uint32_t fracPart = static_cast<uint32_t>(scaledInt % kDivisor);
  snprintf(out, cap, "%s%lu.%06lu", negative ? "-" : "",
                static_cast<unsigned long>(wholePart),
                static_cast<unsigned long>(fracPart));
}

// Copies `text` into `out` (a buffer of `outCap` bytes), STRIPPING every
// '\n'/'\r' byte rather than rejecting the call outright -- RUN's own
// returned-value formatting calls this to keep an embedded terminator
// byte from forging a second wire line. `text == nullptr` is treated
// exactly like `text == ""`. Truncates, never overflows, once `out` is
// full -- always NUL-terminates within `outCap`. Returns the number of
// bytes written (excluding the terminator).
size_t sanitizeLineText(const char* text, char* out, size_t outCap) {
  if (text == nullptr) text = "";
  size_t len = 0;
  for (const char* p = text; *p != '\0' && len + 1 < outCap; ++p) {
    if (*p == '\n' || *p == '\r') continue;  // stripped -- never reaches out
    out[len++] = *p;
  }
  out[len] = '\0';
  return len;
}

}  // namespace

const WireHandler::VerbEntry WireHandler::kCommandTable[] = {
    {"HELLO", &WireHandler::decodeAlwaysTrue, &WireHandler::execNoop},
    {"PING", &WireHandler::decodeAlwaysTrue, &WireHandler::execNoop},
    {"ID", &WireHandler::decodeNoFields, &WireHandler::execId},
    {"VER", &WireHandler::decodeNoFields, &WireHandler::execVer},
    {"STATUS", &WireHandler::decodeNoFields, &WireHandler::execStatus},
    {"HELP", &WireHandler::decodeNoFields, &WireHandler::execHelp},
    {"GET", &WireHandler::decodeGet, &WireHandler::execGet},
    {"SET", &WireHandler::decodeSet, &WireHandler::execSet},
    {"TLM", &WireHandler::decodeTlm, &WireHandler::execTlm},
    {"WHEELS_X", &WireHandler::decodeWheelsX, &WireHandler::execWheelsX},
    {"WHEELS_V", &WireHandler::decodeWheelsV, &WireHandler::execWheelsV},
    {"MOVE_X", &WireHandler::decodeMoveX, &WireHandler::execMoveX},
    {"MOVE_V", &WireHandler::decodeMoveV, &WireHandler::execMoveV},
    {"GO_TO_R", &WireHandler::decodeGoToR, &WireHandler::execGoToR},
    {"GO_TO_W", &WireHandler::decodeGoToW, &WireHandler::execGoToW},
    {"STOP", &WireHandler::decodeStop, &WireHandler::execStop},
    {"ESTOP", &WireHandler::decodeAlwaysTrue, &WireHandler::execNoop},
    {"RUN", &WireHandler::decodeRun, &WireHandler::execRun},
};

WireHandler::WireHandler(Adapter& adapter, Sink& sink)
    : adapter_(adapter), sink_(sink) {
  // WIRE-09 (code review 2026-08-23): pins kCommandTable's deduced size
  // at compile time -- see that member's own doc comment
  // (wire_handler.h) for the silent-zero-fill defect this closes.
  // Placed in a member function (rather than at namespace scope right
  // after the array's own definition above) because kCommandTable is
  // private: an id-expression naming a private member is subject to
  // access control even inside an unevaluated sizeof operand, and only
  // member/friend context is exempt from that check. Evaluated purely
  // at compile time -- this constructor need not even run for a
  // mismatched count to fail the build.
  static_assert(sizeof(kCommandTable) / sizeof(kCommandTable[0]) == 18,
                "kCommandTable verb count");
}

// ---- feed() / line reassembly ------------------------------------------

void WireHandler::feed(const char* data, size_t length) {
  for (size_t i = 0; i < length; ++i) appendByte(data[i]);
}

void WireHandler::appendByte(char c) {
  if (c == '\n') {
    onLineComplete();
    return;
  }
  if (overflowing_) return;  // discard content until the next '\n'
  if (lineLen_ >= kMaxLineBytes - 1) {
    // Storing this byte would make the line's content alone reach
    // kMaxLineBytes - 1, i.e. the line (content + '\n') would exceed
    // the wire's 240-byte cap. Discard to the next '\n' rather than
    // truncate: a truncated prefix that still parses as a legal verb
    // with legal arity would be a command the host never sent.
    overflowing_ = true;
    lineLen_ = 0;
    return;
  }
  lineBuf_[lineLen_++] = c;
}

void WireHandler::onLineComplete() {
  if (overflowing_) {
    overflowing_ = false;
    lineLen_ = 0;
    ++malformedCount_;
    return;
  }
  // A lone '\r' immediately before '\n' is a terminal artifact and is
  // stripped; '\r' appears nowhere else on the wire (protocol.md S2).
  if (lineLen_ > 0 && lineBuf_[lineLen_ - 1] == '\r') --lineLen_;
  lineBuf_[lineLen_] = '\0';

  // A blank or all-whitespace line is ignored SILENTLY (protocol.md S2)
  // -- a terminal artifact, not an error; it does NOT count malformed.
  // Cheap pre-check before the real tokenizer runs.
  bool anyNonSpace = false;
  for (size_t i = 0; i < lineLen_; ++i) {
    if (lineBuf_[i] != ' ') {
      anyNonSpace = true;
      break;
    }
  }
  if (!anyNonSpace) {
    lineLen_ = 0;
    return;
  }

  // The mandatory trailing id (protocol.md S8) must be located BEFORE
  // tokenizeLine() below mutates any separator space to '\0' -- see
  // findLastFieldToken()'s own comment.
  const char* lastFieldToken = findLastFieldToken(lineBuf_);

  char* tokens[kMaxFieldTokens];
  size_t count = tokenizeLine(lineBuf_, tokens, kMaxFieldTokens);
  // anyNonSpace being true normally guarantees at least the verb token
  // was found -- EXCEPT for one C-string edge case anyNonSpace's own
  // byte-by-byte scan does not share with tokenizeLine()'s scan: an
  // embedded NUL is "not a space" (anyNonSpace treats it as content),
  // but it terminates tokenizeLine()'s NUL-terminated-string view
  // immediately, same as the real end of the buffer. A line whose
  // first non-space byte is an embedded NUL (e.g. "\0PING\n") is
  // exactly this case: anyNonSpace correctly sees non-space content at
  // that byte, but tokenizeLine()'s forward scan sees only an empty
  // string there and returns 0 tokens, leaving `tokens[0]`
  // uninitialized. Guard it explicitly rather than dereferencing that:
  // this is corrupted content, not the grammar's own narrowly-defined
  // blank-line/lowercase-reply exceptions, so it is malformed, not
  // silently dropped.
  if (count == 0) {
    ++malformedCount_;
    lineLen_ = 0;
    return;
  }
  char* verb = tokens[0];
  dispatch(verb, tokens + 1, count - 1, lastFieldToken);
  lineLen_ = 0;
}

// ---- tokenizing (protocol.md S2, S3.2) ----------------------------------

size_t WireHandler::tokenizeLine(char* line, char** tokens,
                                  size_t maxTokens) {
  size_t count = 0;
  char* p = line;
  while (true) {
    while (*p == ' ') ++p;  // skip a run of separator spaces (sp ::= ' '+)
    if (*p == '\0') break;  // end of line -- no more tokens
    if (count < maxTokens) tokens[count] = p;
    ++count;
    while (*p != '\0' && *p != ' ') ++p;  // scan to next separator or end
    if (*p == '\0') break;
    *p = '\0';  // terminate this token
    ++p;        // step past the separator byte just nulled
  }
  return count;
}

// ---- dispatch / the reliability layer -----------------------------------
// protocol.md S8 in full; this is the state machine summary from
// wire_handler.h's own file header, implemented.

void WireHandler::dispatch(char* verb, char** fields, size_t fieldCount,
                            const char* lastFieldToken) {
  // Case is direction (protocol.md S2.1): commands are UPPERCASE,
  // replies are lowercase, and verb lookup is case-sensitive. A verb
  // starting with a lowercase letter can never be a command this file
  // recognizes -- it is another robot's reply, overheard on a shared
  // channel, and is dropped SILENTLY, not counted malformed.
  if (verb[0] >= 'a' && verb[0] <= 'z') return;

  // ESTOP and PING are outside the sequence entirely and maximally
  // forgiving of trailing content (protocol.md S8.3): ANY line whose
  // verb token is exactly "ESTOP"/"PING" executes/answers, regardless
  // of arity or what follows -- "ESTOP", "ESTOP 1 2 3", "ESTOP #5" all
  // behave identically.
  if (std::strcmp(verb, "ESTOP") == 0) {
    handleEstop();
    return;
  }
  if (std::strcmp(verb, "PING") == 0) {
    // Whether PING should be MAXIMALLY FORGIVING (like ESTOP) or STRICT
    // zero-arity (like HELLO) is this file's own call, matching
    // protocol_handler.cpp's own resolution: forgiving, so a host still
    // appending an old-style `#<id>` to PING out of habit keeps working
    // unchanged, and PING (liveness) can never itself wedge on a syntax
    // nit.
    handlePing();
    emitReminderIfStalled();
    return;
  }
  if (std::strcmp(verb, "HELP") == 0) {
    // HELP is unsequenced and forgiving, like PING (2026-08-27,
    // stakeholder direction). It is the verb a human types FIRST into a
    // raw relay session, usually because they do not yet know the
    // grammar -- so answering it must not itself depend on knowing the
    // grammar. `HELP`, `HELP #1`, `HELP #99`, `HELP whatever` all emit
    // the same listing. Being outside the sequence, it neither acks nor
    // advances expectedNext_, exactly like PING.
    emitHelp();
    emitReminderIfStalled();
    return;
  }
  if (std::strcmp(verb, "HELLO") == 0) {
    // HELLO's own arity is strict zero-fields (protocol.md S8.3): a
    // HELLO with a trailing field is wrong arity, same as any other
    // extra field, and (being outside the sequence entirely) has no
    // ack/nack to anchor an err against -- silently malformed, exactly
    // like an unrecognized verb.
    if (fieldCount != 0) {
      ++malformedCount_;
      return;
    }
    handleHello();
    return;
  }

  // ---- the read-only QUERY verbs are unsequenced too (2026-08-27) ----
  //
  // ID/VER/STATUS answer a question and change nothing. A duplicate is
  // harmless and a lost one is recovered by simply asking again, so a
  // sequence id buys them no reliability at all -- while costing two
  // real, reported failures:
  //
  //   1. a bare `ID` parsed as #0, fell below expectedNext_, and was
  //      silently dropped; and
  //   2. a RESENT `ID #1` drew `ack 1 0 none` with NO `id` line -- the
  //      S8.1 stale-retransmit row, which for a state-changing verb
  //      correctly means "I already have everything through here, stop
  //      resending", but for a query reads as "accepted, then answered
  //      nothing."
  //
  // The governing rule, and the reason the sequenced/unsequenced split
  // is drawn exactly here:
  //
  //   A VERB IS SEQUENCED IFF ITS CORRECTNESS DEPENDS ON ITS POSITION
  //   IN THE STREAM -- either because executing it twice changes the
  //   robot, or because answering it out of order yields a wrong
  //   answer.
  //
  // Note the second clause: "changes state" alone is NOT the test, and
  // collapsing it to that is what makes GET look like an exception when
  // it is not (see below). ID/VER/HELP answer session CONSTANTS --
  // identity burned into the chip, a compile-time version, a
  // compile-time verb list -- so they are position-independent. TLM
  // mutates subscription state and MOVE/RUN/WHEELS move a robot, so
  // they are not.
  //
  // Forgiving of any trailing content, like PING: `ID`, `ID #1` and
  // `ID #99` all answer identically, and none of them touch
  // expectedNext_.
  //
  // GET stays SEQUENCED and is NOT an exception to the rule -- it is
  // read-only but ORDER-dependent, which the rule already covers.
  // `SET kp 500 #7` / `GET kp #8`: if #7 is lost, a sequenced GET #8 is
  // nacked and never answered, which is correct -- the host must not
  // receive the pre-SET value and read it as the post-SET value. An
  // unsequenced GET would return the stale value with no marker saying
  // it predates a pending write: a silently wrong answer to a config
  // question. (Framing owed to radio-robot-lib-85, 2026-08-27, which
  // is protocol.md's owner.)
  {
    char* noFields[1] = {nullptr};
    uint8_t ignoredErr = 0;
    if (std::strcmp(verb, "ID") == 0) {
      execId(noFields, 0, 0, ignoredErr);
      emitReminderIfStalled();
      return;
    }
    if (std::strcmp(verb, "VER") == 0) {
      execVer(noFields, 0, 0, ignoredErr);
      emitReminderIfStalled();
      return;
    }
    if (std::strcmp(verb, "STATUS") == 0) {
      execStatus(noFields, 0, 0, ignoredErr);
      // Known, ACCEPTED redundancy: STATUS already reports next=/done=/
      // reason= in its own payload, so a trailing nack restates it a
      // different way. Kept deliberately -- "every unsequenced verb
      // except ESTOP and HELLO carries the reminder" is a rule that
      // fits in one's head; carving out STATUS on top of those two does
      // not.
      emitReminderIfStalled();
      return;
    }
  }

  // ---- everything else is on the sequenced plane (protocol.md
  // S8.1/S8.9): a mandatory, well-formed #<id> is REQUIRED as the
  // line's last token, independent of whether the verb itself is even
  // recognized. ----
  uint32_t id = 0;
  if (!parseMandatoryId(lastFieldToken, id)) {
    // No trailing field at all, or one that isn't a well-formed
    // '#'[0-9]+ -- the line cannot be sequence-classified.
    //
    // 2026-08-27, stakeholder direction: this used to be answered with
    // TOTAL SILENCE, on the reasoning that with no id there is nothing
    // to compare against expectedNext_ and therefore nothing to say.
    // That reasoning is wrong, and it produced the single most
    // confusing behaviour on the wire:
    //
    //     WHEELS_X 100 100 2000      <- typed, no id
    //     (nothing at all)
    //
    // The operator cannot tell that from a dead robot, a dropped
    // packet, an unknown verb, or a wedged link. "If I don't put a
    // number on something, there should be an error... Where's my NAK,
    // or my error, or ANYTHING that tells me what happened?"
    //
    // There IS something to say, and it is the same thing a gap says:
    // `nack <expectedNext_>` -- "I did not run that; send me id N."
    // That is precisely the information a host missing an id needs, and
    // it is the reply this branch was already capable of producing.
    //
    // SCOPED TO RECOGNIZED VERBS ONLY. An unrecognized verb still gets
    // silence, deliberately: the radio channel is shared, and answering
    // arbitrary uppercase garbage would make this robot chatter at
    // every corrupted line and every other robot's traffic that happens
    // to survive the case gate. A verb this handler actually implements
    // is addressed to it; unknown tokens are not assumed to be.
    //
    // Does NOT touch the sequence: expectedNext_ is unchanged, nothing
    // executes, and gapOutstanding_ is NOT set -- a missing id is a
    // malformed line, not evidence that a numbered command was lost.
    ++malformedCount_;
    const VerbEntry* known = nullptr;
    for (const auto& e : kCommandTable) {
      if (std::strcmp(verb, e.name) == 0) {
        known = &e;
        break;
      }
    }
    if (known != nullptr) replyNack(expectedNext_);
    return;
  }

  if (id == 0) {
    // `#0` is NEVER a legal id -- ids start at 1 and expectedNext_ never
    // goes below it (S2.2). Until 2026-08-27 this fell into the ordinary
    // stale-retransmit bucket with no special-casing, so it was answered
    // `ack <expectedNext_ - 1>` -- and on a fresh session that is
    // literally `ack 0 0 none`: a receipt for a command that never
    // existed, was never accepted, and did not run. Reported from the
    // field as:
    //
    //     WHEELS_X 100 100 1000 #0
    //     ack 0 0 none                 <- and the robot did not move
    //
    // which is the same "accepted, then nothing happened" shape as
    // GET's silent unknown name and the payload-less stale ack. A
    // never-legal id is a malformed LINE, not a retransmit of anything,
    // so it is answered the way every other unusable id now is: `nack
    // <expectedNext_>`, "I did not run that; send me id N."
    //
    // Sequence untouched, nothing executes, and gapOutstanding_ is NOT
    // set -- nothing was lost, the id was simply invalid.
    ++malformedCount_;
    replyNack(expectedNext_);
    return;
  }

  // The id itself is always fields[fieldCount - 1] once well-formed
  // (findLastFieldToken() found it), so the verb's own DATA fields are
  // everything before it.
  const size_t dataFieldCount = fieldCount - 1;

  if (id < expectedNext_) {
    // A stale retransmit -- the host never saw our ack for something we
    // already accepted. Do NOT re-execute (a resent command must not
    // run twice, once motion verbs exist); just re-state what we
    // already have.
    replyAck(expectedNext_ - 1);
    return;
  }
  if (id > expectedNext_) {
    gapOutstanding_ = true;  // reply predicate only -- see the field's
                              // own comment in wire_handler.h
    // A numeric gap: something between expectedNext_ and id never
    // arrived (or arrived out of order). Discard -- do NOT execute, and
    // do not even look up the verb -- and tell the host exactly what we
    // need next. Every further inbound line re-triggers this same nack
    // until the missing id arrives (S8.1) -- that per-inbound-line
    // repeat is the whole retransmit story (2026-08-26, S8.5: no
    // periodic re-nack exists).
    replyNack(expectedNext_);
    return;
  }

  // id == expectedNext_: find the verb and DECODE its own fields BEFORE
  // sending any reply at all (protocol.md S8.9 -- "decode failure is a
  // NAK"): this is what lets a corrupted leg of a multi-command routine
  // be resent rather than silently skipped.
  const VerbEntry* entry = nullptr;
  for (const auto& e : kCommandTable) {
    if (std::strcmp(verb, e.name) == 0) {
      entry = &e;
      break;
    }
  }
  if (entry == nullptr) {
    // Unrecognized verb: a decode failure exactly like a known verb's
    // own bad arity or unparseable field -- the sequence does NOT
    // advance.
    handleDecodeFailure(id, resultCode(Result::kUnknown));
    return;
  }
  if (!(this->*entry->decode)(fields, dataFieldCount)) {
    handleDecodeFailure(id, resultCode(Result::kBadArg));
    return;
  }

  // Decoded fine: the line arrived intact. The sequence advances and
  // the ack is sent UNCONDITIONALLY at this point -- "did the bytes
  // arrive, in order, and did they parse" is answered here regardless
  // of whether the ADAPTER goes on to refuse the content on its own
  // merits (protocol.md S8.2).
  // In-order and decoded: whatever was outstanding has now arrived.
  expectedNext_ = id + 1;
  gapOutstanding_ = false;
  replyAck(id);

  uint8_t errCode = 0;
  (this->*entry->execute)(fields, dataFieldCount, id, errCode);
  if (errCode != 0) replyErr(id, errCode);
}

void WireHandler::handleDecodeFailure(uint32_t id, uint8_t code) {
  // The sequence does NOT advance: `id` is still expectedNext_ at this
  // point (that equality is what routed dispatch() into this function
  // at all), so nacking expectedNext_ unchanged tells the host to
  // resend EXACTLY this id. A stalled stream keeps re-nacking because
  // every subsequent inbound line re-triggers nack(expectedNext_)
  // exactly like a numeric gap would (S8.1), until a well-formed line
  // finally arrives carrying this same id -- there is no periodic
  // re-nack (2026-08-26, S8.5).
  ++malformedCount_;
  gapOutstanding_ = true;  // a decode-failure stall holds the stream
                            // exactly as a numeric gap does
  replyNack(expectedNext_);
  replyErr(id, code);
}

void WireHandler::replyAck(uint32_t ackedId) {
  char buf[56];
  snprintf(buf, sizeof(buf), "ack %lu %lu %s\n",
                static_cast<unsigned long>(ackedId),
                static_cast<unsigned long>(adapter_.lastDone()),
                doneReasonWireName(adapter_.lastDoneReason()));
  writeLine(buf);
}

void WireHandler::replyNack(uint32_t nextId) {
  char buf[56];
  snprintf(buf, sizeof(buf), "nack %lu %lu %s\n",
                static_cast<unsigned long>(nextId),
                static_cast<unsigned long>(adapter_.lastDone()),
                doneReasonWireName(adapter_.lastDoneReason()));
  writeLine(buf);
}

void WireHandler::replyErr(uint32_t id, uint8_t code) {
  // Field order: code THEN #id -- the id is always the LAST token of
  // ANY line under this grammar, replies included (protocol.md S8.6).
  char buf[32];
  snprintf(buf, sizeof(buf), "err %u #%lu\n", static_cast<unsigned>(code),
                static_cast<unsigned long>(id));
  writeLine(buf);
}

void WireHandler::writeLine(const char* text) {
  sink_.write(text, std::strlen(text));
}

uint8_t WireHandler::resultCode(Result result) {
  switch (result) {
    case Result::kOk: return 0;  // never used as an error code
    case Result::kUnknown: return 1;
    case Result::kBadArg: return 2;
    case Result::kRange: return 3;
    case Result::kFull: return 4;
    case Result::kUnimplemented: return 6;
    case Result::kNotReady: return 8;
    case Result::kBusy: return 10;
  }
  return 1;  // unreachable with every enumerator handled above; kept so
             // a FUTURE enumerator trips -Wswitch instead of silently
             // falling through a default case
}

const char* WireHandler::doneReasonWireName(DoneReason reason) {
  switch (reason) {
    case DoneReason::kNone: return "none";
    case DoneReason::kStop: return "stop";
    case DoneReason::kTimeout: return "timeout";
    case DoneReason::kEstop: return "estop";
    case DoneReason::kAborted: return "aborted";
    case DoneReason::kStall: return "stall";
  }
  return "none";  // unreachable with every enumerator handled above
}

// ---- the three unsequenced verbs -----------------------------------------

void WireHandler::handleHello() {
  // HELLO resets the reliability layer's own sequencing state
  // (protocol.md S8.3) -- the session-start resync a (re)connecting
  // host performs. It does NOT touch the Adapter's own
  // lastDone()/lastDoneReason() (S8.8): that state is Adapter-owned, and
  // a handler-level reset has no business reaching into it. An Adapter
  // that wants a HELLO to also clear ITS OWN notion of "last completed
  // motion" is free to do so from wherever it observes HELLO itself.
  expectedNext_ = 1;
  gapOutstanding_ = false;  // nothing can be outstanding after a reset
  sendBanner();  // protocol.md S4: HELLO's reply is byte-identical to
                 // the unsolicited boot banner
}

void WireHandler::emitReminderIfStalled() {
  // "Hey, I need to remind you that your last command didn't work."
  //
  // 2026-08-27, stakeholder direction. An unsequenced verb is issuable
  // at ANY time with no id -- that gating rule is absolute and is what
  // he objected to. It is separable from whether the reply may CARRY a
  // reliability line, which he does not object to: "I don't actually
  // mind if ID, VER, and help also return an ACK/NAK. What I mind is
  // that they REQUIRE an ACK/NAK."
  //
  // Conditional, not unconditional. Silent on a clean stream -- no
  // receipt on every PING/ID/VER, which would also put a second line on
  // the wire for every query at ch4's measured 66-83% per-line
  // delivery, where two lines both arriving is materially worse than
  // one. It speaks up only when something is actually wrong, which is
  // what makes it a reminder rather than a receipt.
  //
  // This does NOT violate S8.5's anti-beacon rule. That rule objected
  // to PERIODICITY, not to unsolicited-looking acks: a line emitted in
  // reply to an inbound line is still a response to a message. The rule
  // widens from "only in direct response to an inbound SEQUENCED line"
  // to "only in direct response to an INBOUND line". An idle connection
  // stays completely silent, which is the property that mattered.
  //
  // NOT called for ESTOP (S8.3: its reply is the bare word `estop`, no
  // fields ever, and it must never queue behind an outbound reply -- a
  // panic stop carries no diagnostic freight) or HELLO (it resets
  // expectedNext_, so it would be reporting on state it just erased).
  if (!gapOutstanding_) return;
  replyNack(expectedNext_);
}

void WireHandler::handlePing() {
  // Unsequenced and maximally forgiving (see dispatch()'s own comment
  // at the PING branch): ANY line whose verb is PING replies `pong`,
  // regardless of what -- if anything -- follows it.
  char buf[32];
  snprintf(buf, sizeof(buf), "pong %lu\n",
                static_cast<unsigned long>(adapter_.now()));
  writeLine(buf);
}

void WireHandler::handleEstop() {
  // ESTOP is outside the sequence entirely (protocol.md S8.3) --
  // execute BEFORE replying so a panic stop never queues behind an
  // outbound reply.
  adapter_.onEstop();
  writeLine("estop\n");
}

// ---- trivial stand-ins for HELLO/PING/ESTOP's table rows -----------------
// Never actually invoked through kCommandTable (all three are
// intercepted by verb identity in dispatch() before the table lookup
// ever runs) -- present purely so HELP's generated listing walks one
// table for every verb name.

bool WireHandler::decodeAlwaysTrue(char** fields, size_t fieldCount) {
  (void)fields;
  (void)fieldCount;
  return true;
}

void WireHandler::execNoop(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
}

// ---- session verbs ---------------------------------------------------------
// ID/VER/STATUS/HELP all take zero DATA fields (id already stripped by
// dispatch()) -- any remaining field at all is wrong arity, a decode
// failure.

bool WireHandler::decodeNoFields(char** fields, size_t fieldCount) {
  (void)fields;
  return fieldCount == 0;
}

void WireHandler::execId(char** fields, size_t fieldCount, uint32_t id,
                          uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  Identity identity;
  adapter_.identity(identity);
  // `name` appended as a FOURTH field --
  // `id <drivetrain> <profile> <version> <name>`. Strictly additive:
  // fields 0-2 are byte-identical to the 3-field reply radio-robot-lib
  // pins outside this repo (its own wire-format spec, conformance
  // fixture, and an independent handler implementation), so any
  // positional consumer reading only fields 0..2 is unaffected. `name`
  // is identity.name (Protocol::buildIdentity()'s
  // microbit_friendly_name() read) -- see protocol.cpp's own kProfile
  // comment for why `profile` (field 1) is deliberately NOT this verb's
  // identity source anymore.
  //
  // Buffer bumped 96 -> 128 and re-verified against the worst case,
  // since `profile` (kProfile) is not type-bounded -- it is either the
  // checked-in "unbaked" placeholder or a deploy-time-baked robot
  // config filename stem (tools/make_deploy.py's _inject_profile()),
  // which has no code-enforced length cap. Every stem in
  // radio-robot-lib/config/robots/ today is 5-11 chars (longest:
  // "tovez_nocal"); budgeted here to 48 to leave real headroom past
  // that. `version` (kVersion) is currently "1.0.10" (6 chars),
  // drift-tested against pxt.json; budgeted to 24. `drivetrain`
  // (kDrivetrain) is a fixed compile-time literal, currently
  // "diffdrive" (9 chars) -- counted exactly, not budgeted, since it
  // can only change via a code edit. `name` is NOT a budget either: a
  // micro:bit friendly name is ALWAYS exactly MICROBIT_NAME_LENGTH (5)
  // ASCII letters -- an algorithmic hardware fact
  // (codal-microbit-v2/source/MicroBitDevice.cpp's
  // microbit_friendly_name()), not a convention, so this field's
  // contribution is exact. Worst case: "id " (3) + drivetrain (9) +
  // " " (1) + profile budget (48) + " " (1) + version budget (24) +
  // " " (1) + name (5, exact) + "\n" (1) + NUL (1) = 94, comfortably
  // under 128.
  char buf[128];
  snprintf(buf, sizeof(buf), "id %s %s %s %s\n", identity.drivetrain,
                identity.profile, identity.version, identity.name);
  writeLine(buf);
}

void WireHandler::execVer(char** fields, size_t fieldCount, uint32_t id,
                           uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  Identity identity;
  adapter_.identity(identity);
  char buf[64];
  snprintf(buf, sizeof(buf), "ver %s\n", identity.version);
  writeLine(buf);
}

void WireHandler::execStatus(char** fields, size_t fieldCount, uint32_t id,
                              uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  StatusFields status;
  adapter_.status(status);
  // Sprint 004 ticket 004: `i2cf=%ld` joins `flags=%x` -- decimal, not
  // hex (SUC-005's own AC: a copy-pasted hex bit would silently turn
  // i2cf=26 into i2cf=1a). Buffer size bumped from 176 to 200 to keep
  // headroom now that a signed 32-bit field (`i2cf`, up to 11 chars
  // incl. sign) joined the line -- 176 already had margin (the
  // previous worst case measures under 100 bytes), this just keeps
  // that margin honest rather than trimming it to the wire.
  //
  // Sprint 010 ticket 003: `cyc=%lu` joins `i2cf=` immediately after it
  // (both are kernel-health-cousin fields -- see StatusFields::cyc's own
  // doc comment, wire_handler.h). Re-verified against 200: the widest
  // possible line is "status ready=1 active=1 connL=1 connR=1 otos=1 "
  // "wedge=1 flags=ffffffff i2cf=-2147483648 cyc=4294967295 tlm=buffer "
  // "next=4294967295\n" -- 8 single-digit bools (8B), an 8-hex-digit
  // flags (15B incl. "flags="), an 11-char signed i2cf (17B incl.
  // " i2cf="), a 10-digit unsigned cyc (15B incl. " cyc="), tlm's
  // longest wire name "buffer" (11B incl. " tlm="), and a 10-digit
  // next (16B incl. " next="), plus the "status " prefix (7B) and
  // trailing '\n' (1B) -- measures well under 130 bytes total, so 200
  // still keeps comfortable headroom; no bump needed.
  // 2026-08-27: `done=<n> reason=<tok>` join the line, appended AFTER
  // next= (additively -- S6's k=v replies let an older parser ignore
  // unknown keys, so this is backward compatible for free).
  //
  // This is REQUIRED by, and lands before, STATUS becoming unsequenced.
  // Since S8.5 deleted the telemetry piggyback, (lastDone, reason) only
  // ever rides a direct reply -- and radio-robot-lib's own host
  // (src/host/robot_v6/reliability.py, poll_completion) pokes the robot
  // with a STATUS purely to provoke an ack carrying a fresh pair. An
  // unsequenced STATUS emits no ack, so that poll would go silent and
  // completion delivery would die quietly. Carrying the pair on the
  // status line itself removes the dependency on an ack entirely, and
  // closes what protocol.md S8.7 already called "a gap, not a
  // considered omission."
  //
  // Read fresh off the adapter at format time, exactly as replyAck()
  // does -- never cached (S8.8).
  //
  // Width: the two new keys add at most " done=4294967295" (16B) and
  // " reason=" plus the longest wire name ("aborted", 7B) = 15B, i.e.
  // 31B on top of the ~130B worst case documented above. 200 still
  // holds it with room to spare.
  char buf[200];
  snprintf(buf, sizeof(buf),
                "status ready=%d active=%d connL=%d connR=%d otos=%d "
                "wedge=%d flags=%x i2cf=%ld cyc=%lu tlm=%s next=%lu "
                "done=%lu reason=%s\n",
                status.ready ? 1 : 0, status.active ? 1 : 0,
                status.connLeft ? 1 : 0, status.connRight ? 1 : 0,
                status.otos ? 1 : 0, status.wedge ? 1 : 0,
                static_cast<unsigned int>(status.flags),
                static_cast<long>(status.i2cf),
                static_cast<unsigned long>(status.cyc), status.tlm,
                static_cast<unsigned long>(expectedNext_),
                static_cast<unsigned long>(adapter_.lastDone()),
                doneReasonWireName(adapter_.lastDoneReason()));
  writeLine(buf);
}

size_t WireHandler::buildHelpLine(char* buf, size_t bufCap,
                                   const char* const* names,
                                   size_t nameCount) {
  if (bufCap == 0) return 0;
  if (bufCap == 1) {
    buf[0] = '\0';
    return 0;
  }
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < bufCap - 2) buf[pos++] = *text++;
  };
  append("help");
  for (size_t i = 0; i < nameCount; ++i) {
    append(" ");
    append(names[i]);
  }
  buf[pos++] = '\n';
  buf[pos] = '\0';
  return pos;
}

void WireHandler::emitHelp() {
  // Walks kCommandTable at runtime for the name list, so it cannot
  // drift from the dispatcher -- the SAME table dispatch() looks verbs
  // up in. buildHelpLine() owns the terminator guarantee; see its own
  // comment.
  constexpr size_t kVerbCount =
      sizeof(kCommandTable) / sizeof(kCommandTable[0]);
  const char* names[kVerbCount];
  for (size_t i = 0; i < kVerbCount; ++i) names[i] = kCommandTable[i].name;

  // Emit SEVERAL SHORT lines rather than one long one.
  //
  // MEASURED 2026-08-27, tovez over the torture->channel-3 relay
  // (marginal link, see the relay notes): `HELP #1` returned
  // `ack 1 0 none` and NO help line at all. The 16-byte ack survived
  // the hop; the ~110-byte single help line did not. From the
  // operator's seat that reads as "accepted, then answered nothing",
  // which is worse than a clean failure. Short lines are far likelier
  // to survive a lossy radio hop, and a partial listing still tells
  // the operator something. `GET` with no fields already establishes
  // one-line-per-item as this handler's idiom.
  size_t i = 0;
  while (i < kVerbCount) {
    size_t n = 0;
    size_t width = 4;  // "help"
    while (i + n < kVerbCount) {
      const size_t add = 1 + std::strlen(names[i + n]);
      if (n > 0 && width + add > kHelpChunkBytes) break;
      width += add;
      ++n;
    }
    char buf[kMaxLineBytes];
    buildHelpLine(buf, sizeof(buf), names + i, n);
    writeLine(buf);
    i += n;
  }
}

void WireHandler::execHelp(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fields;
  (void)fieldCount;
  (void)id;
  errCode = 0;
  // Normally unreachable: dispatch() intercepts HELP by verb identity
  // before the table lookup (2026-08-27). The row stays in
  // kCommandTable so HELP still appears in its own listing, and this
  // delegates so both paths can never diverge.
  emitHelp();
}

// ---- configuration: pure delegation, no storage here (protocol.md S7) ----

bool WireHandler::decodeGet(char** fields, size_t fieldCount) {
  (void)fields;
  return fieldCount <= 1;
}

void WireHandler::execGet(char** fields, size_t fieldCount, uint32_t id,
                           uint8_t& errCode) {
  (void)id;
  errCode = 0;  // Cleared here; set to ERR_UNKNOWN below if the NAMED
                // form is handed a field the adapter does not declare.

  char buf[kMaxLineBytes];
  char formatted[32];
  if (fieldCount == 0) {
    // Bare GET: dump every field the adapter declares, one line each.
    size_t total = adapter_.fieldCount();
    for (size_t i = 0; i < total; ++i) {
      const char* name = adapter_.fieldName(i);
      float value = 0.0f;
      if (!adapter_.onGet(name, value)) continue;
      formatConfigValue(value, formatted, sizeof(formatted));
      snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
      writeLine(buf);
    }
    return;
  }

  const char* name = fields[0];
  float value = 0.0f;
  // Unknown name -> `err 1 #<id>` ALONGSIDE the ack (2026-08-27,
  // stakeholder-approved). The ack still fires: the line arrived and
  // decoded fine, so this is a MERITS rejection (S8.2), not a decode
  // failure -- the sequence advances exactly as it would for a
  // successful GET.
  //
  // This used to be a silent no-`get`-line answer, deliberately, on the
  // reading that GET "never produces an err". Three reasons that was
  // wrong:
  //
  //  1. It is asymmetric with SET for no stated reason. An unknown SET
  //     name is `err 1`. Same config plane, same mistake (a typo'd
  //     field name), and one verb tells you while the other shrugs.
  //  2. The information exists and was thrown away. Adapter::onGet
  //     returns bool; the handler KNEW the name was unknown and
  //     declined to report it. Not a case we cannot detect -- one we
  //     detected and stayed quiet about.
  //  3. On a lossy link the silence is ambiguous in the worst way. An
  //     ack with no `get` line has two live explanations -- wrong name,
  //     or the `get` line was dropped -- and the operator cannot tell
  //     them apart. Measured on this rig 2026-08-27: 66-75% per-line
  //     delivery on radio ch4. `err 1` disambiguates for the cost of
  //     one short line, which is also the line most likely to survive.
  //
  // Found the hard way: this path produced "accepted, then answered
  // nothing" -- the exact symptom the stakeholder had been objecting to
  // all day -- and it was hit, unnoticed, in this repo's own
  // verification capture. The failure mode is that it does not look
  // like a failure. Bare `GET #id` is untouched: it dumps the declared
  // field list and has no unknown name to report.
  if (!adapter_.onGet(name, value)) {
    errCode = resultCode(Result::kUnknown);
    return;
  }
  formatConfigValue(value, formatted, sizeof(formatted));
  snprintf(buf, sizeof(buf), "get %s %s\n", name, formatted);
  writeLine(buf);
}

bool WireHandler::decodeSet(char** fields, size_t fieldCount) {
  if (fieldCount != 2) return false;
  float discard = 0.0f;
  return parseFloatField(fields[1], discard);
}

void WireHandler::execSet(char** fields, size_t fieldCount, uint32_t id,
                           uint8_t& errCode) {
  (void)fieldCount;
  float value = 0.0f;
  parseFloatField(fields[1], value);  // decodeSet() already proved this
                                       // succeeds
  Result result = adapter_.onSet(fields[0], value, id);
  errCode = resultCode(result);
}

// ---- telemetry -------------------------------------------------------------

bool WireHandler::decodeTlm(char** fields, size_t fieldCount) {
  if (fieldCount != 1) return false;
  TlmMode discard;
  return parseTlmMode(fields[0], discard);
}

void WireHandler::execTlm(char** fields, size_t fieldCount, uint32_t id,
                           uint8_t& errCode) {
  (void)fieldCount;
  (void)id;
  TlmMode mode;
  parseTlmMode(fields[0], mode);  // decodeTlm() already proved this succeeds
  // Sprint 008 ticket 005: the adapter's own Result now surfaces on the
  // wire for TLM, same as every other merits-checked verb dispatched
  // through this table (ack unconditionally above, then `err <code>
  // #<id>` on top iff errCode != 0, per dispatch()'s own comment) --
  // previously this was hardcoded to 0 (errCode never set from the
  // call's actual return), which is why TLM BUFFER's own refusal
  // (WireAdapter::onTlm(), kUnimplemented) could never reach the wire
  // no matter what the adapter decided. Every mode that still returns
  // kOk (OFF/POSE/FULL/NOW/AUTO) is unaffected: resultCode(kOk) == 0,
  // identical to the old hardcoded value.
  Result result = adapter_.onTlm(mode);
  errCode = resultCode(result);
}

// ---- motion: WHEELS_X / WHEELS_V / MOVE_X / MOVE_V / GO_TO_R / GO_TO_W ----
// motion-api.md S9.1's wire mapping. Angles (rotation, omega) are
// milliradian integers on the wire (S9.1: "degrees at the API,
// milliradian integers on the wire ... the conversion lives in the
// binding, in one place" -- NOT this file's job), decoded here with the
// ordinary signed-integer field parser and handed to the Adapter as
// float milliradians -- wire integer -> float for arithmetic
// convenience.

bool WireHandler::decodeWheelsX(char** fields, size_t fieldCount) {
  if (fieldCount != 4) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseUint32(fields[3], discardU);
}

void WireHandler::execWheelsX(char** fields, size_t fieldCount, uint32_t id,
                              uint8_t& errCode) {
  (void)fieldCount;
  int32_t left = 0, right = 0, cruise = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], left);
  parseInt32(fields[1], right);
  parseInt32(fields[2], cruise);
  parseUint32(fields[3], timeout);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. Rejecting here
  // means engineWheelsX() (and therefore MotionEngine::wheelsX()'s own
  // lease-clamp arithmetic) never even runs for timeout == 0, closing
  // R-06's stale-lease bug at the source rather than downstream.
  if (!clampMotionTimeout(timeout)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result =
      adapter_.onWheelsX(static_cast<float>(left), static_cast<float>(right),
                         static_cast<float>(cruise), timeout, id);
  errCode = resultCode(result);
}

bool WireHandler::decodeWheelsV(char** fields, size_t fieldCount) {
  if (fieldCount != 3) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseUint32(fields[2], discardU);
}

void WireHandler::execWheelsV(char** fields, size_t fieldCount, uint32_t id,
                              uint8_t& errCode) {
  (void)fieldCount;
  int32_t left = 0, right = 0;
  uint32_t duration = 0;
  parseInt32(fields[0], left);
  parseInt32(fields[1], right);
  parseUint32(fields[2], duration);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. WHEELS_V's own
  // kWheelsVDurationCeiling (5000 ms, wire_adapter.h) still applies
  // downstream, unchanged -- this only rules out 0 and the >2^31-1
  // wraparound class, both of which sat well outside that ceiling
  // anyway.
  if (!clampMotionTimeout(duration)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result = adapter_.onWheelsV(static_cast<float>(left),
                                     static_cast<float>(right), duration, id);
  errCode = resultCode(result);
}

bool WireHandler::decodeMoveX(char** fields, size_t fieldCount) {
  if (fieldCount != 4) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseUint32(fields[3], discardU);
}

void WireHandler::execMoveX(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fieldCount;
  int32_t distance = 0, rotation = 0, cruise = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], distance);
  parseInt32(fields[1], rotation);
  parseInt32(fields[2], cruise);
  parseUint32(fields[3], timeout);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. Rejecting here
  // means engineMoveX() never runs for timeout == 0, so
  // MotionEngine::moveX()'s own `move_.deadline = now() + timeout`
  // never gets set to "now" (this verb's own instant-no-op behavior,
  // confirmed unchanged from the ticket's own description by reading
  // motion_engine.cpp directly -- see this ticket's own report).
  if (!clampMotionTimeout(timeout)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result = adapter_.onMoveX(static_cast<float>(distance),
                                   static_cast<float>(rotation),
                                   static_cast<float>(cruise), timeout, id);
  errCode = resultCode(result);
}

bool WireHandler::decodeMoveV(char** fields, size_t fieldCount) {
  if (fieldCount != 3) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseUint32(fields[2], discardU);
}

void WireHandler::execMoveV(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fieldCount;
  int32_t v_x = 0, omega = 0;
  uint32_t duration = 0;
  parseInt32(fields[0], v_x);
  parseInt32(fields[1], omega);
  parseUint32(fields[2], duration);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. MOVE_V shares
  // WHEELS_V's own kWheelsVDurationCeiling downstream (unchanged).
  if (!clampMotionTimeout(duration)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result = adapter_.onMoveV(static_cast<float>(v_x),
                                   static_cast<float>(omega), duration, id);
  errCode = resultCode(result);
}

bool WireHandler::decodeGoToR(char** fields, size_t fieldCount) {
  if (fieldCount != 5) return false;
  int32_t discard32 = 0;
  uint32_t discardU = 0;
  return parseInt32(fields[0], discard32) && parseInt32(fields[1], discard32) &&
         parseInt32(fields[2], discard32) && parseInt32(fields[3], discard32) &&
         parseUint32(fields[4], discardU);
}

void WireHandler::execGoToR(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fieldCount;
  int32_t x = 0, y = 0, speed = 0, arrive = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], x);
  parseInt32(fields[1], y);
  parseInt32(fields[2], speed);
  parseInt32(fields[3], arrive);
  parseUint32(fields[4], timeout);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. Rejecting here
  // means engineGoToR() never runs for timeout == 0, so
  // MotionEngine::goToR()'s own deadline math (identical
  // "now() + timeout" shape to moveX(), see execMoveX()'s comment
  // above) never sees the instant-no-op input either.
  if (!clampMotionTimeout(timeout)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result =
      adapter_.onGoToR(static_cast<float>(x), static_cast<float>(y),
                       static_cast<float>(speed), static_cast<float>(arrive),
                       timeout, id);
  errCode = resultCode(result);
}

bool WireHandler::decodeGoToW(char** fields, size_t fieldCount) {
  return decodeGoToR(fields, fieldCount);  // identical field shape
}

void WireHandler::execGoToW(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  (void)fieldCount;
  int32_t x = 0, y = 0, speed = 0, arrive = 0;
  uint32_t timeout = 0;
  parseInt32(fields[0], x);
  parseInt32(fields[1], y);
  parseInt32(fields[2], speed);
  parseInt32(fields[3], arrive);
  parseUint32(fields[4], timeout);
  // Sprint 008 (R-06 + R-18): shared reject-0/clamp-above-2^31-1 bound --
  // see clampMotionTimeout()'s own doc comment above. Same rationale as
  // execGoToR() immediately above -- GO_TO_W shares GO_TO_R's identical
  // field shape and deadline math (via MotionEngine::goToR()).
  if (!clampMotionTimeout(timeout)) {
    errCode = resultCode(Result::kRange);
    return;
  }
  Result result =
      adapter_.onGoToW(static_cast<float>(x), static_cast<float>(y),
                       static_cast<float>(speed), static_cast<float>(arrive),
                       timeout, id);
  errCode = resultCode(result);
}

// ---- STOP: `STOP [now] #<id>` ---------------------------------------------

bool WireHandler::decodeStop(char** fields, size_t fieldCount) {
  if (fieldCount == 0) return true;
  return fieldCount == 1 && std::strcmp(fields[0], "now") == 0;
}

void WireHandler::execStop(char** fields, size_t fieldCount, uint32_t id,
                            uint8_t& errCode) {
  const bool immediate = fieldCount == 1;  // decodeStop() already proved
                                            // this is exactly "now"
  (void)fields;
  Result result = adapter_.onStop(immediate, id);
  errCode = resultCode(result);
}

// ---- RUN: parse-and-delegate only, per wire_handler.h's own onRun() doc --
//
// This handler holds no function table, does no name resolution, and
// does no type conversion -- it extracts the function-name token and
// the raw argument tokens that follow it, and hands them to the adapter
// unchanged. decodeRun()'s own DECODE FAILURES are purely structural: no
// function name at all, or more raw tokens than this line's fixed-size
// arrays can safely hold pointers for. An UNKNOWN function name, or a
// wrong arity the ADAPTER itself detects, is NOT a decode failure --
// RUN's own grammar was satisfied (a name plus some argument tokens), so
// those are merits rejections the adapter reports through its own
// Result (protocol.md S8.9).

bool WireHandler::decodeRun(char** fields, size_t fieldCount) {
  (void)fields;
  if (fieldCount == 0) return false;  // no function name at all -- this
                                       // covers "RUN #7" (the id
                                       // consumes the only field), which
                                       // S8.9 lists explicitly among its
                                       // own decode-failure examples --
                                       // it NACKs, not acks.
  if (fieldCount > kMaxFieldTokens - 1) return false;  // storage overflow
  const size_t argc = fieldCount - 1;
  return argc <= kMaxRunArgs;
}

void WireHandler::execRun(char** fields, size_t fieldCount, uint32_t id,
                           uint8_t& errCode) {
  const char* name = fields[0];
  const size_t argc = fieldCount - 1;  // fields[1 .. fieldCount-1]

  const char* argv[kMaxRunArgs];
  for (size_t i = 0; i < argc; ++i) argv[i] = fields[1 + i];

  char result[kMaxRunResultBytes] = {};
  bool hasResult = false;
  Result outcome =
      adapter_.onRun(name, argv, argc, result, sizeof(result), hasResult);

  errCode = resultCode(outcome);
  if (outcome != Result::kOk) return;
  if (!hasResult) return;  // a void-returning function: the ack already
                            // sent is the whole story.

  // Sanitize the ADAPTER's own returned text before it reaches the
  // sink -- the same '\n'/'\r'-stripping rule any debug text sent over
  // the wire would get. kMaxRunResultBytes already guarantees the sanitized text
  // plus "ret "/" #<id>"/'\n' fits kMaxLineBytes, and sanitizing can
  // only shrink it further, never risk overflow.
  char sanitized[kMaxRunResultBytes];
  sanitizeLineText(result, sanitized, sizeof(sanitized));

  // +1: kMaxLineBytes already counts the WIRE content up to and
  // including '\n', but snprintf() also needs room for its own NUL
  // terminator -- a content string that legitimately reaches the full
  // 240 bytes needs a 241-byte buffer, or snprintf silently truncates
  // the last byte (here, the trailing '\n' itself) to make room for the
  // NUL it always writes.
  char buf[kMaxLineBytes + 1];
  snprintf(buf, sizeof(buf), "ret %s #%lu\n", sanitized,
                static_cast<unsigned long>(id));
  writeLine(buf);
}

// ---- unsolicited emissions -------------------------------------------------

void WireHandler::sendBanner() {
  Identity identity;
  adapter_.identity(identity);
  char buf[96];
  snprintf(buf, sizeof(buf), "device NEZHA2 robot %s %s\n", identity.name,
                identity.serial);
  writeLine(buf);
}

void WireHandler::emitTelemetry(const Snapshot& snapshot) {
  // Count THIS call first -- see framesSinceHeader_'s own header
  // comment for why the arithmetic is deliberately "increment, then
  // compare, then reset to 1 (not 0) when due": that is what makes the
  // 20th call the one that re-emits, not the 21st.
  ++framesSinceHeader_;
  const bool due =
      headerChanged(snapshot) || framesSinceHeader_ >= kHeaderRefreshFrames;
  if (due) {
    emitHeader(snapshot);
    rememberHeader(snapshot);
    framesSinceHeader_ = 1;  // this call is frame 1 of the next streak
  }
  emitFrame(snapshot);
  // No reliability line rides here any more (2026-08-26, protocol.md
  // S8.5, stakeholder direction: "an ack or a nack is only a response
  // to a message, not a beacon"): an ack/nack is only ever a direct
  // reply to an inbound sequenced line. This closes the LAST unsolicited
  // ack path -- sprint 024 ticket 001 removed the free-running beacon on
  // non-subscribed transports, and this removes the telemetry-ON
  // piggyback too (a stale TLM subscription plus the radio path's frame
  // throttle produced an ack-only barrage on an idle link). A lost
  // ack/nack heals via the HOST's own retransmit/poll, one round trip
  // later (S8.1's stale-retransmit re-ack; a gap re-nacks per inbound
  // line).
}

// A memo comparing only count/names would miss a hex-ness-only flip
// (same names, same count, one column's rendering changes from decimal
// to hex or back) -- explicitly called out in the issue this ticket
// closes as the lazy-memo trap. Every column is compared on all three
// of name, hex-ness, and (once, up front) count.
bool WireHandler::headerChanged(const Snapshot& snapshot) const {
  if (!everEmittedHeader_) return true;  // nothing to compare against yet
  if (snapshot.count != headerCount_) return true;
  // A Snapshot wider than the memo's own storage cap cannot be
  // compared column-by-column against what was actually remembered
  // (rememberHeader() below only copies the first kMaxHeaderColumns of
  // it) -- treat it as always-changed rather than either overrunning
  // headerNames_/headerHex_ or silently comparing a truncated prefix.
  // No real caller in this project approaches this cap (sprint.md's
  // widest set is 20 columns).
  if (snapshot.count > kMaxHeaderColumns) return true;
  for (size_t i = 0; i < snapshot.count; ++i) {
    if (headerHex_[i] != snapshot.columns[i].hex) return true;
    if (std::strcmp(headerNames_[i], snapshot.columns[i].name) != 0) {
      return true;
    }
  }
  return false;
}

void WireHandler::rememberHeader(const Snapshot& snapshot) {
  size_t n = snapshot.count;
  if (n > kMaxHeaderColumns) n = kMaxHeaderColumns;
  for (size_t i = 0; i < n; ++i) {
    snprintf(headerNames_[i], kMaxHeaderNameBytes, "%s",
             snapshot.columns[i].name);
    headerHex_[i] = snapshot.columns[i].hex;
  }
  headerCount_ = snapshot.count;
  everEmittedHeader_ = true;
}

void WireHandler::emitHeader(const Snapshot& snapshot) {
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(emitBuf_) - 1) {
      emitBuf_[pos++] = *text++;
    }
  };
  append("thdr");
  for (size_t i = 0; i < snapshot.count; ++i) {
    append(" ");
    append(snapshot.columns[i].name);
  }
  append("\n");
  emitBuf_[pos] = '\0';
  writeLine(emitBuf_);
}

void WireHandler::emitFrame(const Snapshot& snapshot) {
  size_t pos = 0;
  auto append = [&](const char* text) {
    while (*text != '\0' && pos < sizeof(emitBuf_) - 1) {
      emitBuf_[pos++] = *text++;
    }
  };
  char numBuf[16];
  append("t");
  for (size_t i = 0; i < snapshot.count; ++i) {
    append(" ");
    const Column& col = snapshot.columns[i];
    if (col.hex) {
      // Lowercase hex, no "0x" prefix -- a flags-shaped column's bit
      // pattern reinterpreted as unsigned, same convention
      // execStatus()'s own `flags=%x` already uses.
      snprintf(numBuf, sizeof(numBuf), "%x",
               static_cast<unsigned int>(col.value));
    } else {
      snprintf(numBuf, sizeof(numBuf), "%ld", static_cast<long>(col.value));
    }
    append(numBuf);
  }
  append("\n");
  emitBuf_[pos] = '\0';
  writeLine(emitBuf_);
}

}  // namespace Wire
