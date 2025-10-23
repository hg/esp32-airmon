#pragma once

#include "measurement.hh"
#include "queue.hh"
#include <driver/gpio.h>

namespace ds {

struct TempSensor {
  const char *const name;
  const gpio_num_t pin;
  Queue<Measurement> *queue;

  void start(Queue<Measurement> &msQueue);
};

} // namespace ds