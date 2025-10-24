#include "pms.h"
#include "common.h"
#include "measurement.h"
#include "timer.h"
#include "utils.h"

typedef struct {
  uint16_t magic;
  uint8_t command;
  uint16_t data;
  uint16_t checksum;
} __attribute__((packed)) pm_command;

typedef struct {
  uint16_t magic;
  uint16_t frame_len;
  PM16 pm;
  uint16_t reserved;
  uint16_t checksum;
} __attribute__((packed)) pm_response;

typedef struct {
  PM32 pm;
  uint32_t count;
} pm_accum;

static const char *TAG = "air/pm";

static const uint16_t PM_MAGIC = 0x4d42;

static const uint16_t PM_FRAME_LEN = sizeof(pm_response) -
                                     sizeof((pm_response){}.magic) -
                                     sizeof((pm_response){}.frame_len);

#define COMMAND(cmd, hi, lo)                                                   \
  {                                                                            \
      .magic = PM_MAGIC,                                                       \
      .command = cmd,                                                          \
      .data = PP_HTONS((hi << 8) + lo),                                        \
      .checksum = PP_HTONS(0x4d + 0x42 + cmd + lo + hi),                       \
  };

// static const pm_command cmd_read = COMMAND(0xe2, 0x00, 0x00);
// static const pm_command cmd_passive = COMMAND(0xe1, 0x00, 0x00);
// static const pm_command cmd_active = COMMAND(0xe1, 0x00, 0x01);
// static const pm_command cmd_sleep = COMMAND(0xe4, 0x00, 0x00);
static const pm_command cmd_wakeup = COMMAND(0xe4, 0x00, 0x01);

static uint16_t calc_checksum(const pm_response *re) {
  uint16_t sum = 0;

  for (const uint8_t *p = (uint8_t *)&re->magic; p < (uint8_t *)&re->checksum;
       ++p) {
    sum += *p;
  }

  return sum;
}

static void swap_bytes(pm_response *re) {
  for (uint16_t *p = &re->frame_len; p <= &re->checksum; ++p) {
    *p = lwip_htons(*p);
  }
}

static int pm_read_response(pm_sensor *st, pm_response *resp,
                            const TickType_t wait) {
  return uart_read_bytes(st->port, resp, sizeof(pm_response), wait);
}

static int pm_write_command(pm_sensor *st, const pm_command *cmd) {
  return uart_write_bytes(st->port, cmd, sizeof(pm_command));
}

static esp_err_t pm_flush_input(pm_sensor *st) {
  return uart_flush_input(st->port);
}

static esp_err_t pm_flush_output(pm_sensor *st, const TickType_t wait) {
  return uart_wait_tx_done(st->port, wait);
}

static void pm_accum_reset(pm_accum *sum) {
  memset(sum, 0, sizeof(pm_accum));
}

static void pm_accum_add(pm_accum *acc, const pm_response *res) {
  uint32_t *dst = (uint32_t *)&acc->pm;
  uint16_t *src = (uint16_t *)&res->pm;

  for (size_t f = 0; f < PM_FIELDS; ++f) {
    dst[f] += src[f];
  }

  acc->count++;
}

static bool collect(pm_sensor *sens, pm_accum *avg) {
  pm_response resp = {};

  int received = pm_read_response(sens, &resp, seconds(5));

  if (received != sizeof(pm_response)) {
    if (received == -1) {
      ESP_LOGE(TAG, "uart receive failed");
    }
    return false;
  }

  if (resp.magic != PM_MAGIC) {
    ESP_LOGW(TAG, "invalid magic number 0x%x", resp.magic);
    return false;
  }

  swap_bytes(&resp);

  if (resp.frame_len != PM_FRAME_LEN) {
    ESP_LOGW(TAG, "invalid frame length %d", resp.frame_len);
    return false;
  }

  uint16_t checksum = calc_checksum(&resp);

  if (checksum != resp.checksum) {
    ESP_LOGW(TAG, "checksum 0x%x, want 0x%x", resp.checksum, checksum);
    return false;
  }

  pm_accum_add(avg, &resp);

  ESP_LOGI(TAG, "cur PM: 1=%dµg, 2.5=%dµg, 10=%dµg", resp.pm.atm.pm1_ug,
           resp.pm.atm.pm2_ug, resp.pm.atm.pm10_ug);

  return true;
}

[[noreturn]]
static void task_collect(void *arg) {
  pm_sensor *sens = arg;

  measurement ms = {
      .type = MEASURE_PM,
      .sensor = sens->name,
  };

  while (true) {
    int sent = pm_write_command(sens, &cmd_wakeup);
    if (sent != sizeof(cmd_wakeup)) {
      ESP_LOGE(TAG, "could not send wakeup command");
      pm_flush_output(sens, portMAX_DELAY);
      vTaskDelay(seconds(1));
      continue;
    }

    for (bool ready = false;;) {
      timer exec = {};
      timer_reset(&exec);

      pm_accum avg = {};
      pm_accum_reset(&avg);

      // roughly once per 15 seconds
      for (int iter = 0; iter < 18; ++iter) {
        if (!collect(sens, &avg)) {
          pm_flush_input(sens);
        }
      }

      if (!ready) {
        ready = true;
        ESP_LOGI(TAG, "warmup finished");
        continue;
      }

      if (avg.count == 0) {
        ESP_LOGW(TAG, "no measurements to average out");
        continue;
      }

      measure_set_pm(&ms, &avg.pm, avg.count);

      ESP_LOGI(TAG, "avg PM: 1=%huµg, 2.5=%huµg, 10=%huµg (in %lu ms)",
               ms.pm.atm.pm1_ug, ms.pm.atm.pm2_ug, ms.pm.atm.pm10_ug,
               timer_millis(&exec));

      if (!queue_put(sens->queue, &ms)) {
        ESP_LOGE(TAG, "queue is full");
      }
    }
  }

  configASSERT(false);
}

void pms_start(pm_sensor *sens, queue *q) {
  sens->queue = q;

  const uart_config_t conf = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  const int rx_buf = sizeof(pm_response) * 10;
  static_assert(rx_buf >= UART_HW_FIFO_LEN(st->port), "rx_buf too small");

  esp_err_t err;

  err = uart_driver_install(sens->port, rx_buf, 0, 0, NULL, 0);
  ESP_ERROR_CHECK(err);

  err = uart_param_config(sens->port, &conf);
  ESP_ERROR_CHECK(err);

  err = uart_set_pin(sens->port, sens->tx, sens->rx, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE);
  ESP_ERROR_CHECK(err);

  char buf[configMAX_TASK_NAME_LEN];
  snprintf(buf, sizeof(buf), "co2/%s", sens->name);

  xTaskCreate(task_collect, buf, fromKiB(4), sens, 4, NULL);
}
