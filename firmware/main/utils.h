#pragma once

#include "freertos/FreeRTOS.h"

inline size_t KiB(size_t kb) {
  return kb * 1024;
}

inline TickType_t seconds(unsigned sec) {
  return sec * configTICK_RATE_HZ;
}

inline TickType_t millis(unsigned ms) {
  return pdMS_TO_TICKS(ms);
}
