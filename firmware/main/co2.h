#pragma once

#include "measurement.h"
#include "queue.h"
#include <optional>

namespace co2 {

using Co2Level = uint16_t;

static constexpr Co2Level Co2LevelNone = 0;

namespace cmd {

using Address = uint8_t;
using FunctionCode = uint8_t;

struct Command {
  Address address;
  FunctionCode functionCode;
  uint16_t startingAddress;
  uint16_t quantityOfRegisters;
  uint16_t crc;
} __attribute__((packed));

struct Co2Response {
  Address address;
  FunctionCode functionCode;
  uint8_t len;
  int16_t co2;
  uint16_t crc;
} __attribute__((packed));

static_assert(sizeof(Co2Response) == 7);

} // namespace cmd

struct Sensor {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rxPin;
  const gpio_num_t txPin;
  Queue<Measurement> *queue;

  void start(Queue<Measurement> &msQueue);

private:
  [[noreturn]] static void taskCollection(void *arg);
  Co2Level readCo2() const;

  int writeCommand(const cmd::Command &cmd) const;
  esp_err_t flushInput() const;
  esp_err_t flushOutput(TickType_t wait) const;
};

} // namespace co2