#pragma once

#include "measurement.h"
#include "queue.h"
#include <driver/gpio.h>

typedef struct {
  const char *const name;
  const gpio_num_t pin;
  queue *queue;
} temp_sensor;

void temp_start(temp_sensor *sens, queue *q);