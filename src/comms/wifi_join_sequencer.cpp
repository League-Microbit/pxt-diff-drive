// wifi_join_sequencer.cpp -- see wifi_join_sequencer.h. Host-portable:
// no pxt.h, no CODAL.
#include "wifi_join_sequencer.h"

namespace diffDrive {

WifiJoinSequencer::WifiJoinSequencer(WifiLink& link, WifiCredentialStore& store)
    : link_(link),
      store_(store),
      ssidBuf_{0},
      passwordBuf_{0},
      begun_(false),
      walking_(false),
      currentSlot_(0),
      attemptsOnSlot_(0),
      lastObservedState_(WifiLink::kDisabled) {}

void WifiJoinSequencer::begin(const WifiLink::Config& configTemplate) {
  configTemplate_ = configTemplate;
  begun_ = true;
  walking_ = false;  // (re)start the walk from scratch on the next service()
}

void WifiJoinSequencer::service() {
  if (begun_ && !walking_) {
    // First pass since begin() (or since the store went from "some
    // occupied slot" back to "none" -- see advanceTo()'s own comment).
    // Scanning from slot 0 here does NOT force a store re-read (see
    // advanceTo(): the reread fires only once the scan wraps back
    // PAST slot 0 having already visited it), which is correct -- the
    // store's cache is already current as of whatever last populated
    // it (WifiFlashPort's own first-access begin(), or a prior
    // set()/clear() call, both of which write the cache in place).
    advanceTo(0);
  }

  link_.service();

  if (!walking_) return;  // empty store: nothing more to do, ever

  if (link_.state() == WifiLink::kBackoff &&
      lastObservedState_ != WifiLink::kBackoff) {
    // Just entered backoff: one full bring-up attempt on the current
    // slot has finished (either the explicit join failed, or an
    // earlier strict configure step did). WifiLink itself already
    // restarts from AT+RST on its own after kBackoffDelay
    // (serviceBackoff()) -- this class only decides whether that
    // retry should stay on THIS slot (do nothing: WifiLink still holds
    // the Config from beginSlot()) or move to the next occupied one.
    ++attemptsOnSlot_;
    if (isDefinitiveCredentialFailure(link_.lastJoinError()) ||
        attemptsOnSlot_ >= kMaxAttemptsPerSlot) {
      advanceTo(currentSlot_ + 1);  // may call beginSlot(), which
                                     // begin()s WifiLink on a new
                                     // Config and so changes its state
                                     // synchronously (WifiLink::begin())
                                     // -- read state AFTER, not the
                                     // value captured before this call.
    }
  }
  // Read fresh here, not a value captured earlier in this call: the
  // advanceTo() branch above may have just begin()'d WifiLink on a new
  // slot, which resets its state machine synchronously
  // (WifiLink::begin() -> enterState(kConfigure)) -- recording that
  // stale pre-begin() state would make the NEW slot's first genuine
  // kBackoff look like a repeat of one that already happened.
  if (walking_) lastObservedState_ = link_.state();
}

void WifiJoinSequencer::advanceTo(int fromSlot) {
  for (int i = 0; i < WifiCredentialStore::kSlots; ++i) {
    const int slot = (fromSlot + i) % WifiCredentialStore::kSlots;
    if (slot == 0 && i > 0) {
      // Wrapped back to slot 0 -- re-read fresh from flash so a wire
      // SET made mid-walk (WIFICRED SET, ticket 003) is picked up on
      // this new lap, per sprint architecture Design Rationale #3: no
      // dedicated re-point mechanism needed anywhere else.
      store_.begin();
    }
    if (store_.occupied(slot)) {
      beginSlot(slot);
      return;
    }
  }
  // Every slot empty (the store was empty when service() first ran, or
  // every entry was WIFICRED CLEARed mid-walk): leave `walking_` as it
  // is -- false on the very first scan (Protocol's own empty-store
  // fallback owns the link in that case, see this file's header
  // comment), or true with the link still mid-bring-up on whatever
  // slot it last held if every entry vanished mid-walk, which is
  // harmless (that slot's own credentials are still what WifiLink is
  // using; the next advance() call re-scans and will find nothing
  // again until something is re-provisioned).
}

void WifiJoinSequencer::beginSlot(int slot) {
  if (!store_.get(slot, ssidBuf_, passwordBuf_)) return;  // occupied() was
                                                          // just true above;
                                                          // defensive only.
  currentSlot_ = slot;
  attemptsOnSlot_ = 0;
  walking_ = true;

  WifiLink::Config config = configTemplate_;
  config.ssid = ssidBuf_;
  config.password = passwordBuf_;
  config.forceExplicitJoin = true;
  link_.begin(config);
}

// Retry-vs-advance policy, MEASURED gopiv 2026-09-09
// (captures/wifi-join-codes-20260909/notes.md, sprint 038 ticket 001):
//
//   code | meaning (vendor doc; codes 2 and 3 CONFIRMED on this
//        | module, 1 and 4 remain UNVERIFIED -- see wifi_link.h's own
//        | lastJoinError() comment)          | this class's policy
//   -----|--------------------------------------|-------------------
//   2    | wrong password                       | DEFINITIVE: advance
//        |                                       | to the next slot now
//   3    | cannot find target AP                 | retryable -- not a
//        |                                       | statement that the
//        |                                       | credential is
//        |                                       | wrong, only that
//        |                                       | this AP isn't
//        |                                       | answering right now
//   1    | timeout (UNVERIFIED)                  | retryable
//   4    | connect failed (UNVERIFIED)            | retryable
//   0    | no code captured -- a plain
//        | kJoinTimeout, or a strict
//        | configure-step failure before any
//        | AT+CWJAP= was even sent               | retryable
//
// Only code 2 is proof the STORED CREDENTIAL itself is wrong. Every
// other outcome gets kMaxAttemptsPerSlot retries on the same slot
// (WifiLink's own restart-from-AT+RST cadence) before this class
// advances anyway (service()'s attemptsOnSlot_ check), so an
// all-wrong store still cannot wedge forever on one slot.
bool WifiJoinSequencer::isDefinitiveCredentialFailure(int joinError) {
  return joinError == 2;
}

}  // namespace diffDrive
