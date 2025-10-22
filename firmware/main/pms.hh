#pragma once

#include "pm.hh"
#include "queue.hh"
#include <driver/gpio.h>
#include <driver/uart.h>
#include <lwip/def.h>

struct Measurement;

namespace pms {

namespace cmd {

struct Command {
  uint16_t magic;
  uint8_t command;
  uint16_t data;
  uint16_t checksum;
} __attribute__((packed));

} // namespace cmd

struct Response {
  uint16_t magic;
  uint16_t frameLen;
  Pm<uint16_t> pm;
  uint16_t reserved;
  uint16_t checksum;

  [[nodiscard]] uint16_t calcChecksum() const;

  void swapBytes();
} __attribute__((packed));

struct ResponseSum {
  Pm<uint32_t> pm;
  uint32_t count;

  void add(const Response &resp);
  void avg();
  void reset();
};

struct Station {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rxPin;
  const gpio_num_t txPin;
  Queue<Measurement> *queue;

  int readResponse(Response &resp, TickType_t wait) const;
  int writeCommand(const cmd::Command &cmd) const;
  esp_err_t flushInput() const;
  esp_err_t flushOutput(TickType_t wait) const;
  void start(Queue<Measurement> &msQueue);

private:
  static constexpr uint16_t pmsFrameLen =
      sizeof(Response) - sizeof(Response::magic) - sizeof(Response::frameLen);

  [[noreturn]] static void taskCollection(void *arg);
  bool collectionIter(ResponseSum &avg) const;
};

} // namespace pms
