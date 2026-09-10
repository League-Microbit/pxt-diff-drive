// wifi_join_sequencer.h -- WifiJoinSequencer: walks a WifiCredentialStore
// list on boot, driving one WifiLink through each occupied slot in turn
// until one joins. New in sprint 038 ticket 005 (SUC-004, R5).
//
// HOST-PORTABLE BY CONSTRUCTION: no pxt.h, no CODAL -- same discipline
// WifiLink and WifiCredentialStore already follow, so the whole
// slot-ordering / retry-vs-advance policy runs under tests/host/
// against WifiLink's existing fake-module harness plus
// WifiCredentialStore's fake WifiFlashPort
// (tests/host/wifi_join_sequencer_shim.cpp, test_wifi_join_sequencer.py).
//
// == Division of labor (sprint architecture Design Rationale #2) ==
//
// WifiLink still knows only how to bring up ONE AT session for ONE
// (ssid, password) -- Config/begin()'s shape is unchanged (additive:
// one new field, see wifi_link.h). This class is the thing that
// decides WHICH slot's credentials to hand it next, and when. It never
// touches AT command text or flash record layout directly -- only
// WifiCredentialStore's own get()/occupied()/begin() surface.
//
// == begin()/service() mirrors WifiLink's own split ==
//
// begin(configTemplate) records the non-credential parts of a Config
// (hostname/port/hostPort/tcpServer -- ssid/password are overwritten
// per slot and so are ignored from the template) and does no I/O
// itself, exactly like WifiLink::begin(). The credential walk itself
// only starts on the FIRST service() call after begin() -- so a caller
// that never calls begin() at all (an empty credential store; see
// Protocol::serviceWifi()'s own precedence branching) gets a
// service() that is a pure pass-through to the underlying
// WifiLink::service(), never touching the link's Config -- IDENTICAL
// observable behavior to driving that WifiLink directly with no
// sequencer involved (this ticket's own empty-store regression-guard
// acceptance criterion).
//
// == forceExplicitJoin: every Config this class builds sets it true ==
//
// MEASURED gopiv 2026-09-09 (captures/wifi-join-codes-20260909/notes.md,
// badpw-boot.log): the Ai-WB2 module auto-rejoins its own remembered AP
// after the AT+RST that begins every WifiLink::kConfigure pass, which
// WifiLink's default AT+CWJAP? poll (matched on SSID name alone) cannot
// distinguish from the module actually using the credential under
// test. Walking a list is meaningless unless the credential actually
// attempted is the one this class selected -- so every Config built
// here sets forceExplicitJoin = true (wifi_link.h's Config), which
// makes serviceJoin() send AT+CWQAP then the explicit AT+CWJAP=
// instead of polling. See wifi_link.h's own comment on the field for
// the LANDMINE this stays clear of.
//
// == Retry-vs-advance policy ==
//
// See isDefinitiveCredentialFailure()'s own comment in the .cpp -- the
// policy table is written there, next to the code it governs, citing
// ticket 001's hardware confirmation (codes 2 and 3 measured; 1 and 4
// remain UNVERIFIED, per WifiLink::lastJoinError()'s own comment).
//
// == Wrap-and-reread (sprint architecture Design Rationale #3) ==
//
// After the last occupied slot's outcome is decided, the scan for the
// next occupied slot wraps back through slot 0 and re-reads the store
// fresh (WifiCredentialStore::begin()) at that point -- this is what
// lets a wire SET made mid-walk (ticket 003, WIFICRED SET) be picked
// up on the very next lap, with no dedicated re-point mechanism
// anywhere else (WifiLink has none, and needs none).
#pragma once

#include "wifi_credential_store.h"
#include "wifi_link.h"

namespace diffDrive {

class WifiJoinSequencer {
 public:
  // `link` and `store` are borrowed for the life of the sequencer, the
  // same ownership shape WifiLink itself uses for WifiUart& --
  // Protocol owns all three and outlives this class.
  WifiJoinSequencer(WifiLink& link, WifiCredentialStore& store);

  // Arms the walk: records `configTemplate`'s hostname/port/hostPort/
  // tcpServer (its ssid/password are ignored -- every slot supplies its
  // own). Does no I/O -- the walk itself starts on the next service()
  // call. Calling this again restarts the walk from scratch, same as
  // WifiLink::begin() restarting bring-up.
  void begin(const WifiLink::Config& configTemplate);

  // One bounded step, called once per poll in place of a direct
  // wifiLink_.service() call (Protocol::serviceWifi()). Before begin()
  // has ever been called, this is a pure pass-through to
  // WifiLink::service() -- see this file's header comment on why that
  // matters for the empty-store case.
  void service();

  // Which store slot is currently being tried (or was most recently
  // begun) -- test introspection, and useful for a future DBG:wifi
  // ssid= field (ticket 006). Meaningless (0) before the first slot is
  // begun.
  int currentSlot() const { return currentSlot_; }

  // The SSID of the slot currently being tried (or most recently
  // begun) -- "" before the first slot is begun, same "meaningless
  // before the first slot is begun" caveat as currentSlot() above (and
  // for the same reason: ssidBuf_ is zero-initialized but beginSlot()
  // has never copied a real value into it yet). This is the accessor
  // currentSlot()'s own comment anticipated: Protocol::emitWifiDebug()
  // reads it for the R1 `ssid=` field (sprint 038 ticket 006).
  const char* currentSsid() const { return walking_ ? ssidBuf_ : ""; }

  // Whether the current slot's stored credential has a non-empty
  // password -- NEVER the password itself, same never-a-passphrase-
  // accessor discipline WifiCredentialStore::hasPassword() documents
  // (this file's own header comment). Read by emitWifiDebug() for the
  // R1 `haspw=` field, same call site as currentSsid() above.
  bool currentHasPassword() const {
    return walking_ && passwordBuf_[0] != '\0';
  }

  // How many full bring-up attempts the current slot has used so far
  // this lap -- test introspection.
  int attemptsOnSlot() const { return attemptsOnSlot_; }

  // True once this sequencer has actually begun WifiLink on some slot
  // -- false for the lifetime of an empty store (begin() was never
  // called, or the store had zero occupied slots on every scan since).
  bool walking() const { return walking_; }

  // Retry cap: a retryable outcome (AP not found, timeout, ...) gets
  // this many full bring-up attempts on the SAME slot before the
  // sequencer advances anyway -- an all-wrong store must not wedge
  // forever on slot 0 (this ticket's own acceptance criteria).
  static constexpr int kMaxAttemptsPerSlot = 2;

 private:
  // Scans for the next occupied slot starting at `fromSlot`
  // (inclusive), forward, wrapping once through
  // WifiCredentialStore::kSlots. Re-reads the store (store_.begin())
  // the moment the scan reaches slot 0 having already advanced past
  // it once (i.e. a genuine wrap, not the very first scan starting at
  // slot 0) -- see this file's header comment. Begins WifiLink on the
  // first occupied slot found; if the store is (or has become)
  // entirely empty, leaves `walking_`/the link exactly as they were.
  void advanceTo(int fromSlot);

  // Copies slot's credentials into this object's own buffers (Config
  // borrows pointers, so they must outlive the WifiLink session --
  // same reason Protocol::wifiFlashSsid_/wifiFlashPassword_ existed),
  // resets the per-slot attempt counter, and begins WifiLink on it
  // with forceExplicitJoin = true.
  void beginSlot(int slot);

  static bool isDefinitiveCredentialFailure(int joinError);

  WifiLink& link_;
  WifiCredentialStore& store_;

  WifiLink::Config configTemplate_;
  char ssidBuf_[WifiCredentialStore::kSsidBytes];
  char passwordBuf_[WifiCredentialStore::kPasswordBytes];

  bool begun_;    // begin() has been called -- configTemplate_ is valid
  bool walking_;  // this class has actually begun the link on some slot
  int currentSlot_;
  int attemptsOnSlot_;
  WifiLink::State lastObservedState_;
};

}  // namespace diffDrive
