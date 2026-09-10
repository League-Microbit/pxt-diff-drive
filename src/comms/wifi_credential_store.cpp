// wifi_credential_store.cpp -- see wifi_credential_store.h.
#include "wifi_credential_store.h"

namespace diffDrive {

void WifiCredentialStore::copyField(char* dst, const char* src, size_t cap) {
  size_t i = 0;
  for (; i + 1 < cap && src[i] != '\0'; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

void WifiCredentialStore::begin() {
  for (int slot = 0; slot < kSlots; ++slot) {
    uint8_t raw[kRecordBytes];
    port_.read(static_cast<uint32_t>(slot) * kRecordBytes, raw, kRecordBytes);

    Slot& s = cache_[slot];
    s.occupied = (raw[kValidOffset] == kValidByte);
    if (s.occupied) {
      copyField(s.ssid, reinterpret_cast<const char*>(raw + kSsidOffset), kSsidBytes);
      copyField(s.password, reinterpret_cast<const char*>(raw + kPasswordOffset),
                kPasswordBytes);
    } else {
      s.ssid[0] = '\0';
      s.password[0] = '\0';
    }
  }
}

bool WifiCredentialStore::occupied(int slot) const {
  if (slot < 0 || slot >= kSlots) return false;
  return cache_[slot].occupied;
}

bool WifiCredentialStore::anyOccupied() const {
  for (int slot = 0; slot < kSlots; ++slot) {
    if (cache_[slot].occupied) return true;
  }
  return false;
}

bool WifiCredentialStore::get(int slot, char* ssidOut, char* passwordOut) const {
  if (slot < 0 || slot >= kSlots) return false;
  if (!cache_[slot].occupied) return false;
  if (ssidOut != nullptr) copyField(ssidOut, cache_[slot].ssid, kSsidBytes);
  if (passwordOut != nullptr) copyField(passwordOut, cache_[slot].password, kPasswordBytes);
  return true;
}

bool WifiCredentialStore::hasPassword(int slot) const {
  if (slot < 0 || slot >= kSlots) return false;
  return cache_[slot].occupied && cache_[slot].password[0] != '\0';
}

bool WifiCredentialStore::set(int slot, const char* ssid, const char* password) {
  if (slot < 0 || slot >= kSlots) return false;
  if (ssid == nullptr) ssid = "";
  if (password == nullptr) password = "";
  // REJECT, not truncate: see this file's header comment. ">=" because
  // the field must also hold the terminating NUL.
  if (std::strlen(ssid) >= kSsidBytes) return false;
  if (std::strlen(password) >= kPasswordBytes) return false;

  uint8_t raw[kRecordBytes];
  std::memset(raw, 0, sizeof(raw));
  std::memcpy(raw + kSsidOffset, ssid, std::strlen(ssid));
  std::memcpy(raw + kPasswordOffset, password, std::strlen(password));
  raw[kValidOffset] = kValidByte;

  if (!port_.write(static_cast<uint32_t>(slot) * kRecordBytes, raw, kRecordBytes)) {
    return false;
  }

  Slot& s = cache_[slot];
  s.occupied = true;
  copyField(s.ssid, ssid, kSsidBytes);
  copyField(s.password, password, kPasswordBytes);
  return true;
}

bool WifiCredentialStore::clear(int slot) {
  if (slot < 0 || slot >= kSlots) return false;

  uint8_t raw[kRecordBytes];
  std::memset(raw, 0, sizeof(raw));  // valid byte 0x00 != kValidByte -> empty;
                                      // also wipes any prior ssid/password bytes.
  if (!port_.write(static_cast<uint32_t>(slot) * kRecordBytes, raw, kRecordBytes)) {
    return false;
  }

  cache_[slot] = Slot();
  return true;
}

}  // namespace diffDrive
