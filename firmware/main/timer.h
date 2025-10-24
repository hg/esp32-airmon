#pragma once

#include "freertos/FreeRTOS.h"

typedef struct {
  int64_t start;
} timer;

void timer_reset(timer *t);
unsigned long timer_millis(const timer *t);