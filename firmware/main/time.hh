#pragma once

#include <ctime>

// UNIX time when the system was booted
extern time_t bootTimestamp;

constexpr bool isValidTimestamp(const time_t ts) {
  return ts >= 1577836800; // 2020-01-01 UTC
}

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
time_t getTimestamp();

void startSntp();
