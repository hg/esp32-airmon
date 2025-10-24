#pragma once

#include "queue.h"
#include <driver/gpio.h>
#include <driver/uart.h>
#include <lwip/def.h>

typedef struct {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rx;
  const gpio_num_t tx;
  queue *queue;
} pm_sensor;

void pms_start(pm_sensor *sens, queue *q);
