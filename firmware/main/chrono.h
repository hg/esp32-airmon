#pragma once

#include <ctime>

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
[[nodiscard]] time_t getTimestamp();
void fixInvalidTime(time_t &tm);
void startSntp();