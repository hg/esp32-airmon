#pragma once

#include "freertos/FreeRTOS.h"
#include <chrono>
#include <limits>

class Timer {
public:
  // creates and starts the timer
  Timer() : start{xTaskGetTickCount()} {}

  // returns passed time in seconds
  TickType_t seconds() const {
    const TickType_t now = xTaskGetTickCount();

    const TickType_t passed =
        now >= start
            ? now - start
            : std::numeric_limits<decltype(start)>::max() - start + now;

    return passed / configTICK_RATE_HZ;
  }

private:
  const TickType_t start;
};
