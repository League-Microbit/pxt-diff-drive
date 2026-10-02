// wifi_address.h -- a robot's fixed WiFi address. Every robot defaults to
// 10.55.<group>.<channel>, the radio group and channel its five-letter
// name derives, on a /16 whose gateway is 10.55.255.254.
#pragma once

#include <cstddef>
#include <cstdio>

namespace diffDrive {

constexpr const char* kWifiGateway = "10.55.255.254";
constexpr const char* kWifiNetmask = "255.255.0.0";

// "a.b.c.d" plus its NUL.
constexpr size_t kWifiAddressBytes = 16;

// Writes `name`'s default address into `out`. False, `out` untouched,
// when `name` is not a robot name.
inline bool defaultWifiAddress(const char* name, char* out, size_t cap) {
  static const char consonants[] = "zvgpt";
  static const char vowels[] = "uoiea";
  if (name == nullptr) return false;
  unsigned n = 0;
  for (int p = 0; p < 5; ++p) {
    const char* alphabet = (p % 2 == 0) ? consonants : vowels;
    int index = -1;
    for (int i = 0; i < 5; ++i) {
      if (name[p] != '\0' && alphabet[i] == name[p]) index = i;
    }
    if (index < 0) return false;
    n = n * 5 + static_cast<unsigned>(index);
  }
  if (name[5] != '\0') return false;
  snprintf(out, cap, "10.55.%u.%u", 15 + n % 241, 11 + n % 73);
  return true;
}

}  // namespace diffDrive
