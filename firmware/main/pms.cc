#include "pms.hh"
#include "common.hh"
#include "measurement.hh"
#include "settings.hh"
#include "state.hh"
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
  std::transform(&frameLen, (&checksum) + 1, &frameLen, &lwip_htons);
}

[[noreturn]] void Station::collectionTask(void *const arg) {
  Station &station{*reinterpret_cast<Station *>(arg)};

  Response res{};
  ResponseSum sum{};
  Measurement ms{.type = MeasurementType::PARTICULATES, .sensor = station.name};

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    int sent = station.writeCommand(cmd::cmdWakeup);
    if (sent != sizeof(cmd::cmdWakeup)) {
      ESP_LOGE(logTag, "could not send wakeup command");
      station.flushOutput(portMAX_DELAY);
      vTaskDelay(secToTicks(1));
      continue;
    }

    // wait for the station to wake up and send us the first command
    station.flushInput();
    station.readResponse(res, portMAX_DELAY);

    // discard first measurements as recommended by the manual
    vTaskDelay(secToTicks(30));
    station.flushInput();

    // if network is down, queue average measurements to conserve RAM
    const bool sendEach = appState->check(AppState::STATE_NET_CONNECTED);
    if (!sendEach) {
      sum.reset();
    }

    Timer execTime;
    bool periodOverflow = false;

    for (int successful = 0; successful < CONFIG_PARTICULATE_MEASUREMENTS;) {
      if (execTime.seconds() >= CONFIG_PARTICULATE_PERIOD_SECONDS) {
        ESP_LOGE(logTag, "PM measurement took too much time");
        periodOverflow = true;
        break;
      }

      const int received = station.readResponse(res, secToTicks(5));

      if (received != sizeof(res)) {
        if (received == -1) {
          ESP_LOGE(logTag, "uart receive failed");
        }
        station.flushInput();
        continue;
      }

      if (res.magic != cmd::magic) {
        ESP_LOGW(logTag, "invalid magic number 0x%x", res.magic);
        station.flushInput();
        continue;
      }

      res.swapBytes();

      if (res.frameLen != pmsFrameLen) {
        ESP_LOGW(logTag, "invalid frame length %d", res.frameLen);
        station.flushInput();
        continue;
      }

      const uint16_t checksum = res.calcChecksum();
      if (checksum != res.checksum) {
        ESP_LOGW(logTag, "checksum 0x%x, expected 0x%x", res.checksum,
                 checksum);
        station.flushInput();
        continue;
      }

      if (sendEach) {
        ms.time = getTimestamp();
        ms.set(res.pm);
        if (!station.queue->putRetrying(ms)) {
          ESP_LOGE(logTag, "could not queue particulate measurement");
        }
      } else {
        sum.addMeasurement(res);
      }

      ESP_LOGI(logTag, "read PM: 1=%dµg, 2.5=%dµg, 10=%dµg", res.pm.atm.pm1Mcg,
               res.pm.atm.pm2Mcg, res.pm.atm.pm10Mcg);

      ++successful;
    }

    ESP_LOGI(logTag, "measurement finished in %u s", execTime.seconds());

    if (!sendEach) {
      ms.time = getTimestamp();
    }

    sent = station.writeCommand(cmd::cmdSleep);
    if (sent != sizeof(cmd::cmdSleep)) {
      ESP_LOGE(logTag, "could not send sleep command");
    }

    if (periodOverflow) {
      // measurement period overflow, skip next iteration
      lastWake = xTaskGetTickCount();
    } else {
      if (!sendEach) {
        sum.calcAvg();
        ms.set(sum.pm);
        ESP_LOGI(logTag, "avg PM: 1=%u, 2.5=%u, 10=%u", ms.pm.atm.pm1Mcg,
                 ms.pm.atm.pm2Mcg, ms.pm.atm.pm10Mcg);

        if (!station.queue->putRetrying(ms)) {
          ESP_LOGE(logTag, "could not queue averaged PM measurement");
        }
      }
    }

    vTaskDelayUntil(&lastWake, secToTicks(appSettings.period.pm));
  }
}

int Station::readResponse(Response &resp, const TickType_t wait) {
  return uart_read_bytes(port, &resp, sizeof(resp), wait);
}

int Station::writeCommand(const cmd::Command &cmd) {
  return uart_write_bytes(port, &cmd, sizeof(cmd));
}

esp_err_t Station::flushInput() { return uart_flush_input(port); }

esp_err_t Station::flushOutput(const TickType_t wait) {
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
  static_assert(rxBuf >= UART_FIFO_LEN);

  ESP_ERROR_CHECK(uart_driver_install(port, rxBuf, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &conf));
  ESP_ERROR_CHECK(
      uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  char buf[32];
  snprintf(buf, sizeof(buf), "pm_%s", name);

  xTaskCreate(collectionTask, buf, KiB(2), this, 4, nullptr);
}

void ResponseSum::addMeasurement(const Response &resp) {
  pm += resp.pm;
  ++count;
}

void ResponseSum::reset() { memset(this, 0, sizeof(*this)); }

void ResponseSum::calcAvg() {
  if (count > 0) {
    pm /= count;
    count = 0;
  }
}

} // namespace pms
