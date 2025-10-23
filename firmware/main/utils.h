#pragma once

#include "freertos/FreeRTOS.h"

constexpr size_t KiB(const size_t kb) { return kb * 1024; }

constexpr TickType_t seconds(const TickType_t sec) {
  return sec * configTICK_RATE_HZ;
}

constexpr TickType_t milliseconds(const TickType_t ms) {
  return pdMS_TO_TICKS(ms);
}
