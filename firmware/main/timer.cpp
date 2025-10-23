#include "timer.h"
#include <chrono>
#include <limits>

unsigned Timer::millis() const {
  const TickType_t now = xTaskGetTickCount();

  const unsigned long passed =
      now >= start ? now - start
                   : std::numeric_limits<TickType_t>::max() - start + now;

  return passed * 1000 / configTICK_RATE_HZ;
}