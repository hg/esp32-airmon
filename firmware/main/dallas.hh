#pragma once

#include "measurement.hh"
#include "queue.hh"
#include <driver/gpio.h>
#include <driver/rmt.h>
#include <ds18b20.h>
#include <memory>

namespace ds {

struct TempSensor {
  const char *const name;
  const gpio_num_t pin;
  const rmt_channel_t rxChan;
  const rmt_channel_t txChan;
  Queue<Measurement> *queue;

  void start(Queue<Measurement> &msQueue);

private:
  [[noreturn]] static void collectionTask(void *arg);

  void runMeasurements(const DS18B20_Info &device);
};

} // namespace ds
