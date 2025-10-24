#pragma once

#include "measurement.h"
#include "queue.h"

typedef struct {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rx;
  const gpio_num_t tx;
  queue *queue;
} co2_sensor;

void co2_start(co2_sensor *sens, queue *q);
