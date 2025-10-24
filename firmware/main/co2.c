#include "co2.h"
#include "crc16.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "measurement.h"
#include "stdio.h"
#include "utils.h"
#include <esp_log.h>
#include <sys/cdefs.h>

typedef uint16_t co2_level;
typedef uint16_t co2_crc;
typedef uint8_t co2_address;
typedef uint8_t co2_code;

typedef struct {
  co2_address address;
  co2_code functionCode;
  uint16_t startingAddress;
  uint16_t quantityOfRegisters;
  co2_crc crc;
} __attribute__((packed)) co2_command;

static_assert(sizeof(co2_command) == 8, "invalid co2 command size");

typedef struct {
  co2_address address;
  co2_code functionCode;
  uint8_t len;
  int16_t co2;
  co2_crc crc;
} __attribute__((packed)) co2_response;

static_assert(sizeof(co2_response) == 7, "invalid co2 response size");

static const char *TAG = "air/co2";

static co2_crc cmd_crc(const co2_command *cmd) {
  const uint8_t *bytes = (const uint8_t *)cmd;
  return crc16(bytes, sizeof(*cmd) - sizeof(cmd->crc));
}

static co2_crc resp_crc(const co2_response *re) {
  const uint8_t *bytes = (const uint8_t *)re;
  return crc16(bytes, sizeof(*re) - sizeof(re->crc));
}

static int sensor_send_cmd(co2_sensor *sens, const co2_command *cmd) {
  return uart_write_bytes(sens->port, cmd, sizeof(*cmd));
}

static esp_err_t sensor_flush_input(co2_sensor *sens) {
  return uart_flush_input(sens->port);
}

static esp_err_t sensor_flush_output(co2_sensor *sens) {
  return uart_wait_tx_done(sens->port, seconds(1));
}

static co2_level sensor_read_co2(co2_sensor *sens) {
  co2_response re = {};

  int read = uart_read_bytes(sens->port, &re, sizeof(re), portMAX_DELAY);

  if (read != sizeof(co2_response)) {
    ESP_LOGE(TAG, "invalid response length %d bytes", read);
    return 0;
  }

  co2_crc crc = resp_crc(&re);

  if (crc != re.crc) {
    ESP_LOGE(TAG, "invalid CRC 0x%x (expected 0x%x)", crc, re.crc);
    return 0;
  }

  if (re.co2 <= 0) {
    ESP_LOGE(TAG, "sensor returned zero or negative CO2 (%d)", re.co2);
    return 0;
  }

  return PP_NTOHS(re.co2);
}

[[noreturn]]
static void task_co2(void *arg) {
  co2_sensor *sens = arg;

  measurement ms = {
      .type = MEASURE_CO2,
      .sensor = sens->name,
  };

  TickType_t lastWake = xTaskGetTickCount();

  co2_command cmd_read_co2 = {
      .address = 0xfe,      // any address
      .functionCode = 0x04, // read input registers
      .startingAddress = PP_HTONS(3),
      .quantityOfRegisters = PP_HTONS(1),
  };
  cmd_read_co2.crc = cmd_crc(&cmd_read_co2);

  while (true) {
    int written = sensor_send_cmd(sens, &cmd_read_co2);

    if (written != sizeof(cmd_read_co2)) {
      ESP_LOGE(TAG, "could not send command (written %d bytes)", written);
      vTaskDelay(seconds(5));
      sensor_flush_input(sens);
      sensor_flush_output(sens);
      continue;
    }

    co2_level level = sensor_read_co2(sens);
    if (level != 0) {
      measure_set_co2(&ms, level);
      queue_put(sens->queue, &ms);
      ESP_LOGI(TAG, "CO2 concentration: %d PPM", ms.co2);
    } else {
      ESP_LOGE(TAG, "could not receive CO2 data from sensor");
    }

    xTaskDelayUntil(&lastWake, seconds(4));
  }
}

void co2_start(co2_sensor *sens, queue *q) {
  sens->queue = q;

  uart_config_t conf = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  const int rx_size = sizeof(co2_response) * 24;
  static_assert(rx_size >= UART_HW_FIFO_LEN(sens->port), "rx_size too low");

  esp_err_t err = uart_driver_install(sens->port, rx_size, 0, 0, NULL, 0);
  ESP_ERROR_CHECK(err);

  err = uart_param_config(sens->port, &conf);
  ESP_ERROR_CHECK(err);

  err = uart_set_pin(sens->port, sens->tx, sens->rx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  ESP_ERROR_CHECK(err);

  char buf[configMAX_TASK_NAME_LEN];
  snprintf(buf, sizeof(buf), "co2/%s", sens->name);

  xTaskCreate(task_co2, buf, fromKiB(4), sens, 4, NULL);
}
