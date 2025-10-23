#include "pms.hh"
#include "common.hh"
#include "measurement.hh"
#include "time.hh"
#include "timer.hh"
#include "utils.hh"
#include <algorithm>
#include <cstring>
#include <numeric>

namespace pms {

namespace cmd {

static constexpr uint16_t magic = 0x4d42;

static constexpr Command initCmd(uint8_t cmd, uint8_t hi, uint8_t lo) {
  return Command{
      .magic = magic,
      .command = cmd,
      .data = PP_HTONS((hi << 8) + lo),
      .checksum = PP_HTONS(0x4d + 0x42 + cmd + lo + hi),
  };
}

static constexpr Command cmdRead = initCmd(0xe2, 0x00, 0x00);
static constexpr Command cmdModePassive = initCmd(0xe1, 0x00, 0x00);
static constexpr Command cmdModeActive = initCmd(0xe1, 0x00, 0x01);
static constexpr Command cmdSleep = initCmd(0xe4, 0x00, 0x00);
static constexpr Command cmdWakeup = initCmd(0xe4, 0x00, 0x01);

} // namespace cmd

uint16_t Response::calcChecksum() const {
  return std::accumulate(reinterpret_cast<const uint8_t *>(&magic),
                         reinterpret_cast<const uint8_t *>(&checksum), 0);
}

void Response::swapBytes() {
  std::transform(&frameLen, &checksum + 1, &frameLen, &lwip_htons);
}

bool Station::collectionIter(ResponseSum &avg) const {
  Response res{};

  int received = readResponse(res, secToTicks(5));

  if (received != sizeof(res)) {
    if (received == -1) {
      ESP_LOGE(logTag, "uart receive failed");
    }
    return false;
  }

  if (res.magic != cmd::magic) {
    ESP_LOGW(logTag, "invalid magic number 0x%x", res.magic);
    return false;
  }

  res.swapBytes();

  if (res.frameLen != pmsFrameLen) {
    ESP_LOGW(logTag, "invalid frame length %d", res.frameLen);
    return false;
  }

  uint16_t checksum = res.calcChecksum();

  if (checksum != res.checksum) {
    ESP_LOGW(logTag, "checksum 0x%x, want 0x%x", res.checksum, checksum);
    return false;
  }

  avg.add(res);

  ESP_LOGI(logTag, "cur PM: 1=%dµg, 2.5=%dµg, 10=%dµg", res.pm.atm.pm1Mcg,
           res.pm.atm.pm2Mcg, res.pm.atm.pm10Mcg);

  return true;
}

[[noreturn]] void Station::taskCollection(void *const arg) {
  Station &station{*reinterpret_cast<Station *>(arg)};

  Measurement ms{.type = MeasurementType::PARTICULATES, .sensor = station.name};

  while (true) {
    int sent = station.writeCommand(cmd::cmdWakeup);
    if (sent != sizeof(cmd::cmdWakeup)) {
      ESP_LOGE(logTag, "could not send wakeup command");
      station.flushOutput(portMAX_DELAY);
      vTaskDelay(secToTicks(1));
      continue;
    }

    int warmup = 0;

    while (true) {
      Timer tm{};
      ResponseSum avg{};

      for (int iter = 0; iter < 20; ++iter) {
        if (!station.collectionIter(avg)) {
          station.flushInput();
        }
      }

      if (warmup < 2) {
        warmup++;
        ESP_LOGI(logTag, "warmup %d/2 finished", warmup);
        continue;
      }

      if (avg.count == 0) {
        ESP_LOGW(logTag, "no measurements to average out");
        continue;
      }

      avg.avg();

      ms.updateTime();
      ms.set(avg.pm);

      ESP_LOGI(logTag, "avg PM: 1=%uµg, 2.5=%uµg, 10=%uµg (in %d ms)",
               ms.pm.atm.pm1Mcg, ms.pm.atm.pm2Mcg, ms.pm.atm.pm10Mcg,
               tm.millis());

      if (!station.queue->putRetrying(ms)) {
        ESP_LOGE(logTag, "could not queue averaged PM measurement");
      }
    }
  }
}

int Station::readResponse(Response &resp, const TickType_t wait) const {
  return uart_read_bytes(port, &resp, sizeof(resp), wait);
}

int Station::writeCommand(const cmd::Command &cmd) const {
  return uart_write_bytes(port, &cmd, sizeof(cmd));
}

esp_err_t Station::flushInput() const { return uart_flush_input(port); }

esp_err_t Station::flushOutput(const TickType_t wait) const {
  return uart_wait_tx_done(port, wait);
}

void Station::start(Queue<Measurement> &msQueue) {
  queue = &msQueue;

  constexpr uart_config_t conf{
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  constexpr size_t rxBuf = sizeof(Response) * 10;
  assert(rxBuf >= UART_HW_FIFO_LEN(this.port));

  ESP_ERROR_CHECK(uart_driver_install(port, rxBuf, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &conf));
  ESP_ERROR_CHECK(
      uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  xTaskCreate(taskCollection, "meas_pm", KiB(4), this, 4, nullptr);
}

void ResponseSum::add(const Response &resp) {
  pm += resp.pm;
  ++count;
}

void ResponseSum::avg() {
  if (count > 0) {
    pm /= count;
    count = 0;
  }
}

} // namespace pms