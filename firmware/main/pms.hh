#pragma once

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
  uint16_t pm1McgStd;
  uint16_t pm2McgStd;
  uint16_t pm10McgStd;
  uint16_t pm1McgAtm;
  uint16_t pm2McgAtm;
  uint16_t pm10McgAtm;
  uint16_t pm03Count;
  uint16_t pm05Count;
  uint16_t pm1Count;
  uint16_t pm2Count;
  uint16_t pm5Count;
  uint16_t pm10Count;
  uint16_t reserved;
  uint16_t checksum;

  uint16_t calcChecksum() const;

  void swapBytes();
} __attribute__((packed));

struct Station {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rxPin;
  const gpio_num_t txPin;
  Queue<Measurement> *queue;

  int readResponse(Response &resp, TickType_t wait) const;

  int writeCommand(const cmd::Command &cmd) const;

  esp_err_t flushInput() const;

  void start(Queue<Measurement> &msQueue);

private:
  static constexpr size_t pmsFrameLen = sizeof(Response) -
                                        sizeof(Response::magic) -
                                        sizeof(Response::frameLen);

  [[noreturn]] static void collectionTask(void *arg);
};

struct ResponseSum {
  uint32_t count;
  struct {
    uint32_t pm1Mcg;
    uint32_t pm2Mcg;
    uint32_t pm10Mcg;
  } std;
  struct {
    uint32_t pm1Mcg;
    uint32_t pm2Mcg;
    uint32_t pm10Mcg;
  } atm;
  struct {
    uint32_t pm03Count;
    uint32_t pm05Count;
    uint32_t pm1Count;
    uint32_t pm2Count;
    uint32_t pm5Count;
    uint32_t pm10Count;
  } cnt;

  void addMeasurement(const Response &resp);

  void reset();
};

} // namespace pms
