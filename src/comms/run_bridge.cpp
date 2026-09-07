// run_bridge.cpp -- RunBridge's sanitize/bypass/park rules. See
// run_bridge.h for what each of them is for.
#include "run_bridge.h"

#include <cstring>

namespace diffDrive {
namespace {

// Printable ASCII, the only alphabet a cleartext RUN payload may use.
bool isPrintable(uint8_t c) { return c >= 0x20 && c <= 0x7E; }

}  // namespace

bool RunBridge::isBypassName(const char* text) {
  size_t nameLen = 0;
  while (text[nameLen] != '\0' && text[nameLen] != ':') ++nameLen;
  const char* const kBypassNames[] = {"abort", "clearestop"};
  for (const char* name : kBypassNames) {
    if (std::strlen(name) == nameLen && std::memcmp(text, name, nameLen) == 0)
      return true;
  }
  return false;
}

RunBridge::Offer RunBridge::malformed() {
  // Saturating, like the ring's own drop count: a refusal count that
  // wrapped to zero would read as "nothing was refused".
  if (malformedCount_ != UINT32_MAX) ++malformedCount_;
  return Offer::kMalformed;
}

RunBridge::Offer RunBridge::offer(const uint8_t* data, size_t len) {
  if (data == nullptr || len == 0) return malformed();

  // Strip one trailing '\r' (raw-terminal artifact), then copy the
  // payload verbatim. Anything outside printable ASCII -- or too long
  // for a slot -- is malformed and dropped. The name/argument split is
  // NOT done here: this layer stays a transport for the text, and the
  // TypeScript layer owns the vocabulary.
  //
  // Every refusal below goes through malformed() and is COUNTED. It
  // used to be a bare `return`, which made a payload that was one byte
  // too long -- a plausible length for a real command with several
  // numeric arguments -- indistinguishable, from the operator's seat,
  // from radio loss: the command simply never happened and nothing
  // anywhere said so. The ring's drop count answers a different
  // question (capacity), so it cannot stand in for this one.
  if (data[len - 1] == '\r') --len;
  if (len == 0 || len >= kTextBytes) return malformed();
  char text[kTextBytes];
  for (size_t i = 0; i < len; ++i) {
    if (!isPrintable(data[i])) return malformed();
    text[i] = static_cast<char>(data[i]);
  }
  text[len] = '\0';
  if (text[0] == ':') return malformed();  // empty name

  if (isBypassName(text)) {
    stage(text);
    return Offer::kBypass;
  }

  if (queue_.enqueue(text, static_cast<int>(len)) < 0) {
    // Every slot is still in flight. Refusing is the point: a bare
    // write cursor would have overwritten one, and the handler holding
    // it would then have run a command nobody sent. The refusal is
    // counted and readable rather than silent, so a host that out-runs
    // the robot can find out.
    return Offer::kDropped;
  }
  return Offer::kQueued;
}

bool RunBridge::dispatchOne() {
  const int slot = queue_.peek();
  if (slot < 0) return false;
  stage(queue_.at(slot));
  queue_.release(slot);
  return true;
}

void RunBridge::stage(const char* text) {
  std::strncpy(currentText_, text, kTextBytes - 1);
  currentText_[kTextBytes - 1] = '\0';
}

}  // namespace diffDrive
