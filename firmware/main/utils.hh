#pragma once

#include "freertos/FreeRTOS.h"

constexpr size_t KiB(const size_t kb) { return kb * 1024; }

constexpr TickType_t secToTicks(const TickType_t seconds) {
  return seconds * configTICK_RATE_HZ;
}

constexpr TickType_t msToTicks(const TickType_t ms) {
  return pdMS_TO_TICKS(ms);
}
