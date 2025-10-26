#include "temp.h"
#include "common.h"
#include "driver/temperature_sensor.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "utils.h"

static const char *TAG = "air/tmp/in";

static void on_fire(TimerHandle_t tm) {
  temperature_sensor_handle_t sens = pvTimerGetTimerID(tm);
  configASSERT(sens);

  float temp = .0f;
  esp_err_t err = temperature_sensor_get_celsius(sens, &temp);

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "temperature (internal): %.2f°C", temp);
  } else {
    ESP_LOGE(TAG, "failed to measure temperature: %s", esp_err_to_name(err));
  }
}

void temp_start_int() {
  esp_err_t err;
  temperature_sensor_handle_t sens = NULL;
  temperature_sensor_config_t conf = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-30, 50);

  err = temperature_sensor_install(&conf, &sens);
  ESP_RETURN_VOID_ON_ERROR(err, TAG, "temp install failed");

  err = temperature_sensor_enable(sens);
  ESP_RETURN_VOID_ON_ERROR(err, TAG, "temp enable failed");

  TimerHandle_t tm = xTimerCreate("temp_int", seconds(30), true, sens, on_fire);

  if (xTimerStart(tm, 0) != pdPASS) {
    ESP_LOGE(TAG, "unable to start internal temp timer");
  }
}