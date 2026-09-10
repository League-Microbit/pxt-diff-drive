// wifi_credential_store.h -- WifiCredentialStore: a bounded list of
// (ssid, password) records persisted in ONE dedicated flash page,
// reached through the WifiFlashPort seam (src/platform/wifi_flash_port.h).
//
// HOST-PORTABLE BY CONSTRUCTION: no pxt.h, no CODAL -- same discipline
// wifi_link.h documents for WifiLink, and for the same reason: the
// whole record layout, truncation-vs-reject policy, and list semantics
// run under tests/host/ against a fake WifiFlashPort
// (tests/host/wifi_flash_port_shim.cpp, test_wifi_credential_store.py),
// with nothing about real flash in the loop.
//
// == Why this exists, and why not KeyValueStorage / MicroBitFileSystem ==
//
// `uBit.storage` (KeyValueStorage) is a 16-byte-key/32-byte-value/
// 5-pair store -- it cannot hold even one `(ssid, password)` record,
// let alone a list.
// `MicroBitFileSystem` was rejected as unjustified complexity: a
// directory/block-allocation filesystem sized for files, whose own
// default scratch page is the SAME page the flash driver's low-level
// erase-merge write path already uses for every write on this chip.
// This class is the alternative: a handful of fixed-size records in one
// page, written through MicroBitFlash::flash_write() via WifiFlashPort
// (never a raw write that assumes no erase is needed).
//
// == Record layout, and why REJECT rather than truncate ==
//
// One fixed-size slot: an SSID field sized to match
// `Protocol::wifiSsid_` (33 bytes incl. NUL) and a password field sized
// to match `Protocol::wifiPassword_` (64 bytes incl. NUL), plus one
// validity byte. `Protocol::setupWifi()` already truncates an
// over-length ssid/password silently (snprintf into a fixed buffer);
// this store deliberately does NOT inherit that behavior without
// saying so -- set() REJECTS (no write happens) a string that would not
// fit with its own NUL, per this ticket's own acceptance criteria. A
// caller that wants "truncate instead" must decide that for itself
// before calling set(); this class will never silently hand back
// something shorter than what was asked to be stored.
//
// == Erased flash reads as EMPTY, not corrupted ==
//
// A never-written (or freshly erased) flash page reads back as all
// 0xFF. This store's validity byte is a specific marker (kValidByte,
// below) distinct from both 0xFF (erased) and 0x00 (what clear()
// writes) -- so an erased page decodes as "every slot empty," not as
// noise to reject or crash on. See begin()'s comment.
//
// == Never a passphrase accessor the wire can reach by accident ==
//
// get() reads the real password back out -- it exists for WifiLink's
// own use (a `Config` needs the actual string) and MUST NOT be wired to
// any network-reachable surface. hasPassword() is the accessor meant
// for exactly that: "is a password set," never the password itself --
// what the WIFICRED wire verb's enumeration is expected to call.
// Keeping these as two differently-named methods, rather than one
// get() a caller could carelessly wire up, is the "clearly separated"
// half of the passphrase-safety requirement (the other half, never
// tracing a passphrase into a log line, is WifiLink's
// lastCommand()/startCommand() contract, unrelated code path).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../platform/wifi_flash_port.h"

namespace diffDrive {

class WifiCredentialStore {
 public:
  // K, the sprint architecture's Open Question 4 default. Fixed at
  // compile time -- this is firmware, not a growable list.
  static constexpr int kSlots = 8;

  static constexpr size_t kSsidBytes = 33;      // incl NUL; matches Protocol::wifiSsid_
  static constexpr size_t kPasswordBytes = 64;  // incl NUL; matches Protocol::wifiPassword_

  // On-flash record layout, one per slot, contiguous within the page:
  // ssid bytes, then password bytes, then one validity byte. THE ORDER
  // AND SIZE OF THESE FIELDS IS THE ON-FLASH FORMAT -- changing them
  // is a migration, not a refactor (out of scope here; no migration
  // story exists yet, per the sprint architecture's Migration Concerns).
  static constexpr size_t kSsidOffset = 0;
  static constexpr size_t kPasswordOffset = kSsidOffset + kSsidBytes;
  static constexpr size_t kValidOffset = kPasswordOffset + kPasswordBytes;
  static constexpr size_t kRecordBytes = kValidOffset + 1;

  static_assert(
      static_cast<uint32_t>(kSlots) * static_cast<uint32_t>(kRecordBytes) <=
          WifiFlashPort::kPageBytes,
      "WifiCredentialStore's K slots must fit inside one WifiFlashPort page");

  // `port` is borrowed for the life of the store (the composition
  // root's own WifiFlashPort[Codal], mirroring WifiLink's borrowed
  // WifiUart&).
  explicit WifiCredentialStore(WifiFlashPort& port) : port_(port) {}

  // Reads every slot's occupancy/content from flash into the small
  // in-RAM cache every other method below reads -- so a caller-facing
  // get()/occupied()/hasPassword() never touches flash itself. Safe to
  // call again (re-reads). An all-0xFF (erased) or otherwise unreadable
  // page decodes as every slot empty -- never a crash, never treated as
  // corrupt data (see this file's header comment).
  void begin();

  bool occupied(int slot) const;

  // True iff ANY slot is occupied -- the "does this board have a
  // provisioned list at all" check Protocol::serviceWifi()'s lazy-begin
  // uses to decide whether WifiJoinSequencer owns the join or the
  // setupWifi()/baked single-credential path does (per the precedence
  // rule in this project's WiFi architecture docs).
  bool anyOccupied() const;

  // Copies slot's NUL-terminated ssid/password into the caller's
  // buffers -- each must be at least kSsidBytes/kPasswordBytes. Returns
  // false, buffers untouched, for an out-of-range or unoccupied slot.
  // Passing nullptr for either output skips that copy (so a caller that
  // only wants the ssid need not provide a password buffer) -- but see
  // this file's header comment on where this method may and may not be
  // called from.
  bool get(int slot, char* ssidOut, char* passwordOut) const;

  // "Is a password set" -- true iff `slot` is occupied AND its stored
  // password is non-empty. Never returns the password itself; see this
  // file's header comment.
  bool hasPassword(int slot) const;

  // Writes ssid/password into `slot`. Rejects -- no write happens --
  // when `slot` is out of range, or when either string (counting its
  // own NUL) does not fit in kSsidBytes/kPasswordBytes: REJECTED, not
  // silently truncated (see this file's header comment). `password` of
  // "" is a legal record (an open network); `ssid` of "" is accepted at
  // this layer too -- the join caller, not this class, decides whether
  // an empty-ssid slot is ever attempted.
  bool set(int slot, const char* ssid, const char* password);

  // Marks `slot` empty. Writes a zero-filled record over the WHOLE
  // slot, not just its validity byte, so a cleared slot's old
  // passphrase does not linger readable in flash. Other slots are
  // unaffected -- the underlying WifiFlashPort::write() only ever
  // touches the addressed byte range. False for an out-of-range slot.
  bool clear(int slot);

 private:
  // A record whose validity byte reads back as exactly this marker is
  // occupied. Deliberately distinct from BOTH 0xFF (an erased/
  // never-written page -- "empty," not corrupted) and 0x00 (what
  // clear() writes -- also "empty," by choice, so a cleared slot and a
  // never-written one are indistinguishable from the outside, which is
  // the correct behavior: neither should be attempted).
  static constexpr uint8_t kValidByte = 0xA5;

  struct Slot {
    bool occupied = false;
    char ssid[kSsidBytes] = {0};
    char password[kPasswordBytes] = {0};
  };

  // Bounded copy: copies up to cap-1 bytes from `src` (which need not
  // itself be NUL-terminated within [0, cap) -- raw flash content isn't
  // trusted to be) and always NUL-terminates `dst`. Used both for
  // decoding a flash record (begin()) and for handing a cached field
  // back out (get()).
  static void copyField(char* dst, const char* src, size_t cap);

  WifiFlashPort& port_;
  Slot cache_[kSlots];
};

// The firmware's one shared store instance, reached by WireAdapter's
// WIFICRED seam. DECLARED here (host-portable
// -- this header still reaches no pxt.h) so WireAdapter's own
// host-portable translation unit can call it, mirroring
// run_registry.h's runRegistry() split: the DEFINITION lives in
// wifi_flash_port.cpp -- the one CODAL-coupled translation unit this
// feature has -- because building the real instance means constructing
// a WifiFlashPortCodal, which only that file may name. A function-local
// static for the same reason runRegistry() is one: constructed on
// first call, never destroyed, so a block program's own top-level
// registration ordering can never race it.
//
// This function-local static REMAINS the storage, even after sprint
// 038 ticket 005 added WifiJoinSequencer (src/comms/
// wifi_join_sequencer.h), which needs this same store -- deliberately:
// Protocol constructs its WifiJoinSequencer member by binding this
// function's return (a real, constructor-injected `WifiCredentialStore&`,
// same ownership shape WifiLink itself uses for WifiUart&), so both
// WireAdapter's WIFICRED verb and Protocol's own join-sequencing share
// the ONE instance without Protocol having to separately construct and
// then hand off a second one. Re-pointing this free function later
// still changes only its callers, not WIFICRED's own wire-level
// behavior or its tests.
WifiCredentialStore& wifiCredentialStore();

}  // namespace diffDrive
