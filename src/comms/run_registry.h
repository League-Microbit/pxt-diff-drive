// run_registry.h -- the names FUNCS discloses: what this robot can
// actually be told to run.
//
// This repo's runnable surface is the CLEARTEXT `RUN:<name>[:<arg>...]`
// carve-out (protocol.h), dispatched by name against the handlers a
// block program bound with `onRun()` (blocks/run.ts). That table lives
// in TypeScript, is built at program start, and differs per program --
// a line-following program registers `line`/`linesense`, a tour program
// registers `tour`/`pivot`/`straight`. Nothing in C++ could see it, so
// `FUNCS` had nothing to enumerate.
//
// This is the mirror. `onRun()` publishes each name here as it binds
// its handler (shims.cpp's registerRunName), and WireAdapter reads the
// table back out through the Adapter's runCount()/runName()/
// runSignature() seam. It is a MIRROR, deliberately: TypeScript keeps
// owning dispatch, and a name that fails to land here (see the
// saturation note below) costs discoverability, never executability.
//
// `onRunCommand()` -- the catch-all that answers EVERY name -- publishes
// NOTHING here, on purpose: the set of names it makes runnable is
// unbounded and cannot be enumerated. So a program built only on it
// (test/testrig.ts is exactly that) reports an empty FUNCS listing
// while still dispatching every name. That is honest rather than
// misleading; a listing that claimed otherwise would be neither.
//
// NOT the v6 `RUN <name> #id` verb's registry. The two are told apart by
// PUNCTUATION, not by verb: protocol.cpp's routeLine() diverts a line
// whose first four bytes are the literal `RUN:` to the cleartext bridge,
// and sends everything else -- the space-separated v6 `RUN` verb
// included -- to the v6 stack. So both forms are reachable; only the
// colon one has anything behind it, since WireAdapter::onRun registers
// nothing and answers every name kUnknown. FUNCS therefore lists what
// `RUN:<name>` will dispatch -- the question an operator or a host UI is
// actually asking -- rather than reporting the empty v6 allowlist and
// calling it an answer.
//
// Host-portable on purpose -- no pxt.h, no CODAL types, nothing but
// <cstddef>/<cstdint>/<cstring> -- the same split run_queue.h and
// run_bridge.h already use, so the table's sizing and truncation rules
// can be exercised with no firmware at all.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace diffDrive {

// A fixed table, sized for a generous block program and NOT growable:
// this is firmware, and the alternative to a bound is a heap allocation
// on a device whose heap exhaustion presents as a silent boot death
// (run.ts's own note on panic 980).
//
// Overflow SATURATES: registration 33 and beyond is dropped and counted
// rather than evicting an earlier name. Dropping the newest is the
// honest failure here -- the alternative, a ring that overwrites, would
// make FUNCS list a table that no longer matches what dispatch does,
// which is exactly the "short list read as the whole list" failure the
// protocol's FUNCS section refuses. overflowCount() is what tells a
// reader the listing is partial.
template <int Slots = 32, int Bytes = 24>
class RunRegistry {
 public:
  static constexpr int kSlots = Slots;
  static constexpr int kBytes = Bytes;  // name/signature, content + NUL

  // Publish one registered name, with an optional single-token
  // signature. Both are TRUNCATED to fit rather than refused: a name
  // too long to store is still a name the dispatcher will answer to,
  // and half of it in the listing beats none. (The wire's own
  // sanitize/truncate pass in execFuncs is a separate, later defence --
  // this one is about the table's own fixed cells.)
  //
  // Idempotent by name: re-registering an existing name updates its
  // signature instead of adding a second row. A block program that
  // binds two handlers to one name -- legal, and run.ts dispatches to
  // both -- must still appear once in the listing, because the listing
  // enumerates addressable NAMES, not handler bindings.
  //
  // An empty name is refused outright and not counted as an overflow:
  // it is not addressable, so it is not a registration.
  bool add(const char* name, const char* signature) {
    if (name == nullptr || name[0] == '\0') return false;

    const int existing = find(name);
    if (existing >= 0) {
      copyInto(signatures_[existing], signature);
      return true;
    }
    if (count_ >= Slots) {
      if (overflow_ < UINT32_MAX) ++overflow_;
      return false;
    }
    copyInto(names_[count_], name);
    copyInto(signatures_[count_], signature);
    ++count_;
    return true;
  }

  int count() const { return count_; }

  // Set once a catch-all handler is bound (run.ts's onRunCommand),
  // which makes EVERY name dispatchable and so makes this table a
  // non-exhaustive sample rather than an allowlist. WireAdapter::onRun
  // stops refusing unlisted names when this is true -- FUNCS still
  // lists only what was named, because an unbounded set cannot be
  // enumerated; see this file's own header comment.
  void acceptAnyName() { acceptsAny_ = true; }
  bool acceptsAnyName() const { return acceptsAny_; }

  // Never null -- both are handed straight to a string API by the
  // caller, the same contract run_queue.h's at() keeps.
  const char* name(int index) const {
    return (index < 0 || index >= count_) ? "" : names_[index];
  }
  const char* signature(int index) const {
    return (index < 0 || index >= count_) ? "" : signatures_[index];
  }

  // Registrations dropped because the table was full. Saturating, for
  // run_queue.h's own reason: a count that wrapped to zero would read
  // as "nothing was lost".
  uint32_t overflowCount() const { return overflow_; }

  // Index of `name`, or -1. Exposed because a caller that wants to know
  // whether a name is listed should not have to walk the table itself.
  int find(const char* name) const {
    if (name == nullptr) return -1;
    for (int i = 0; i < count_; ++i) {
      if (std::strcmp(names_[i], name) == 0) return i;
    }
    return -1;
  }

 private:
  // Copies at most Bytes-1 characters plus a NUL. A null or empty
  // source stores the empty string, which is how "no signature" is
  // spelled -- execFuncs omits the field entirely for it.
  static void copyInto(char* dest, const char* src) {
    if (src == nullptr) {
      dest[0] = '\0';
      return;
    }
    int i = 0;
    while (src[i] != '\0' && i < Bytes - 1) {
      dest[i] = src[i];
      ++i;
    }
    dest[i] = '\0';
  }

  char names_[Slots][Bytes] = {};
  char signatures_[Slots][Bytes] = {};
  int count_ = 0;
  uint32_t overflow_ = 0;
  bool acceptsAny_ = false;
};

// The one table the firmware shares between the TypeScript registration
// shim (shims.cpp) and the wire adapter that discloses it
// (wire_adapter.cpp). A function-local static rather than a namespace-
// scope object for the initialisation-order reason run.ts already
// documents from the TypeScript side: registration happens from a block
// program's top-level code, which on this target can run before a
// translation unit's own namespace-scope constructors have.
RunRegistry<>& runRegistry();

}  // namespace diffDrive
