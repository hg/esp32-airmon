#include "co2.hh"
#include "crc16.hh"
#include "driver/uart.h"
#include "esp_err.h"
#include "measurement.hh"
#include "time.hh"
#include <cstdio>
#include <esp_log.h>
#include <sys/cdefs.h>

static const char *const kTag = "co2";

namespace co2::cmd {

static Command initCmd(Command &&cmd) {
  const auto *const bytes = reinterpret_cast<const uint8_t *>(&cmd);
  cmd.crc = crc16(bytes, sizeof(cmd) - sizeof(cmd.crc));
  return cmd;
}

static const Command kCmdReadCo2Level = initCmd({
    .address = kAddressAny,
    .functionCode = kReadInputRegisters,
    .startingAddress = PP_HTONS(3),
    .quantityOfRegisters = PP_HTONS(1),
});

} // namespace co2::cmd

namespace co2 {

[[noreturn]] void Sensor::taskCollection(void *const arg) {
  Sensor &sensor{*reinterpret_cast<Sensor *>(arg)};
  Measurement ms{.type = MeasurementType::CO2, .sensor = sensor.name};

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    const auto written = sensor.writeCommand(cmd::kCmdReadCo2Level);

    if (written != sizeof(cmd::kCmdReadCo2Level)) {
      ESP_LOGE(kTag, "could not send command (written %d bytes)", written);
      vTaskDelay(secToTicks(5));
      sensor.flushInput();
      sensor.flushOutput(secToTicks(1));
      continue;
    }

    const std::optional<Co2Level> co2 = sensor.readCo2();

    if (co2.has_value()) {
      ms.time = getTimestamp();
      ms.co2 = co2.value();
      sensor.queue->put(ms, portMAX_DELAY);

      ESP_LOGI(kTag, "CO2 concentration: %d PPM", ms.co2);
    } else {
      ESP_LOGE(kTag, "could not receive CO2 data from sensor");
    }

    vTaskDelayUntil(&lastWake, secToTicks(4));
  }
}

int Sensor::writeCommand(const cmd::Command &cmd) {
  return uart_write_bytes(port, &cmd, sizeof(cmd));
}

esp_err_t Sensor::flushInput() { return uart_flush_input(port); }

esp_err_t Sensor::flushOutput(const TickType_t wait) {
  return uart_wait_tx_done(port, wait);
}

void Sensor::start(Queue<Measurement> &msQueue) {
  queue = &msQueue;

  constexpr uart_config_t conf{.baud_rate = 9600,
                               .data_bits = UART_DATA_8_BITS,
                               .parity = UART_PARITY_DISABLE,
                               .stop_bits = UART_STOP_BITS_1,
                               .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                               .source_clk = UART_SCLK_APB};

  constexpr size_t rxBuf = sizeof(cmd::Co2Response) * 24;
  static_assert(rxBuf >= UART_FIFO_LEN);

  ESP_ERROR_CHECK(uart_driver_install(port, rxBuf, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &conf));
  ESP_ERROR_CHECK(
      uart_set_pin(port, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  char buf[32];
  snprintf(buf, sizeof(buf), "co2_%s", name);

  xTaskCreate(taskCollection, buf, KiB(4), this, 4, nullptr);
}

[[nodiscard]] std::optional<Co2Level> Sensor::readCo2() {
  cmd::Co2Response response{};

  const auto read =
      uart_read_bytes(port, &response, sizeof(response), portMAX_DELAY);

  if (read != sizeof(cmd::Co2Response)) {
    ESP_LOGE(kTag, "invalid response length %d bytes", read);
    return std::nullopt;
  }

  const uint16_t crc = crc16(reinterpret_cast<const uint8_t *>(&response),
                             sizeof(response) - sizeof(response.crc));

  if (crc != response.crc) {
    ESP_LOGE(kTag, "invalid CRC 0x%x (expected 0x%x)", crc, response.crc);
    return std::nullopt;
  }

  if (response.co2 <= 0) {
    ESP_LOGE(kTag, "sensor returned zero or negative CO2 (%d)", response.co2);
    return std::nullopt;
  }

  const auto level = static_cast<Co2Level>(PP_NTOHS(response.co2));

  return std::make_optional(level);
}

} // namespace co2
