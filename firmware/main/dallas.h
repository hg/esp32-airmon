#pragma once

#include "measurement.h"
#include "queue.h"
#include <driver/gpio.h>

namespace ds {

struct TempSensor {
  const char *const name;
  const gpio_num_t pin;
  Queue<Measurement> *queue;

  void start(Queue<Measurement> &msQueue);
};

} // namespace ds