#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Timer {
public:
  // creates and starts the timer
  Timer() : start{xTaskGetTickCount()} {}

  // returns passed time in milliseconds
  unsigned millis() const;

private:
  const TickType_t start;
};
