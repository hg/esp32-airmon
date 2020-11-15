#pragma once

#include "freertos/FreeRTOS.h"
#include <chrono>

class Timer {
public:
  // creates and starts the timer
  Timer() { ticks = xTaskGetTickCount(); }

  // returns passed time in seconds
  TickType_t seconds() {
    return (xTaskGetTickCount() - ticks) / configTICK_RATE_HZ;
  }

private:
  TickType_t ticks;
};
