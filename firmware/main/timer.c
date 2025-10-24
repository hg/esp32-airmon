#include "timer.h"
#include "esp_timer.h"

void timer_reset(timer *timer) {
  timer->start = esp_timer_get_time();
}

unsigned long timer_millis(const timer *timer) {
  int64_t now = esp_timer_get_time();
  return (now - timer->start) / 1000;
}
