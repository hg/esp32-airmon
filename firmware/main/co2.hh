#pragma once

#include "measurement.hh"
#include "queue.hh"
#include <optional>

namespace co2 {

using Co2Level = uint16_t;

namespace cmd {

using Address = uint8_t;
using FunctionCode = uint8_t;

static constexpr Address kAddressAny = 0xfe;
static constexpr FunctionCode kReadInputRegisters = 0x04;

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
  uint16_t co2;
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
  [[noreturn]] static void collectionTask(void *arg);

  [[nodiscard]] std::optional<Co2Level> readCo2();

  int writeCommand(const cmd::Command &cmd);
  esp_err_t flushInput();
  esp_err_t flushOutput(TickType_t wait);
};

} // namespace co2
