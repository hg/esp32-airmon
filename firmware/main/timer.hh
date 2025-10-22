#pragma once

#include "freertos/FreeRTOS.h"
#include <chrono>
#include <limits>

class Timer {
public:
  // creates and starts the timer
  Timer() : start{xTaskGetTickCount()} {}

  // returns passed time in milliseconds
  unsigned millis() const {
    const TickType_t now = xTaskGetTickCount();

    const unsigned long passed =
        now >= start ? now - start
                     : std::numeric_limits<TickType_t>::max() - start + now;

    return passed * 1000 / configTICK_RATE_HZ;
  }

private:
  const TickType_t start;
};
