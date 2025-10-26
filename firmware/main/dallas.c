#include "dallas.h"
#include "common.h"
#include "ds18b20.h"
#include "onewire_bus.h"
#include "utils.h"

static const char *TAG = "air/tmp/ds";

static void process(temp_sensor *ts) {
  onewire_bus_handle_t bus;
  onewire_bus_config_t bus_config = {
      .bus_gpio_num = ts->pin,
      .flags = {.en_pull_up = true},
  };
  onewire_bus_rmt_config_t rmt_config = {
      .max_rx_bytes = 32,
  };
  ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

  ds18b20_device_handle_t sensor;
  onewire_device_iter_handle_t iter = NULL;
  onewire_device_t next_onewire_device;
  esp_err_t search = ESP_OK;

  // create 1-wire device iterator, which is used for device search
  ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));

  do {
    search = onewire_device_iter_get_next(iter, &next_onewire_device);

    if (search == ESP_OK) {
      ds18b20_config_t ds_cfg = {};
      onewire_device_address_t address;

      esp_err_t err = ds18b20_new_device_from_enumeration(&next_onewire_device,
                                                          &ds_cfg, &sensor);

      if (err == ESP_OK) {
        ds18b20_get_device_address(sensor, &address);
        ESP_LOGI(TAG, "found DS18B20 at %016llX", address);
        break;
      }

      ESP_LOGI(TAG, "found unknown device at %016llX",
               next_onewire_device.address);
    }
  } while (search != ESP_ERR_NOT_FOUND);

  ESP_ERROR_CHECK(onewire_del_device_iter(iter));
  ESP_ERROR_CHECK(ds18b20_set_resolution(sensor, DS18B20_RESOLUTION_12B));

  measurement ms = {
      .type = MEASURE_TEMP,
      .sensor = ts->name,
  };

  while (true) {
    float temp = .0f;

    vTaskDelay(seconds(15));

    ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion_for_all(bus));
    ESP_ERROR_CHECK(ds18b20_get_temperature(sensor, &temp));
    ESP_LOGI(TAG, "temperature (ds18b20): %.2f°C", temp);

    measure_set_temp(&ms, temp);

    if (!queue_put(ts->queue, &ms)) {
      ESP_LOGE(TAG, "queue is full");
    }
  }
}

[[noreturn]]
static void task_temp(void *arg) {
  temp_sensor *sensor = arg;

  ESP_LOGI(TAG, "starting temp task for %s", sensor->name);

  while (true) {
    vTaskDelay(seconds(2));
    process(sensor);
    ESP_LOGE(TAG, "restarting failed sensor %s", sensor->name);
  }
}

void temp_start(temp_sensor *sens, queue *q) {
  sens->queue = q;

  char buf[configMAX_TASK_NAME_LEN];
  snprintf(buf, sizeof(buf), "temp/%s", sens->name);

  xTaskCreate(task_temp, buf, fromKiB(4), sens, 2, NULL);
}
