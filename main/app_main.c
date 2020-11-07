#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sys/time.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "ds18b20.h"
#include "owb.h"
#include "owb_rmt.h"

typedef struct {
  const char *name;
  const int pin;
  const rmt_channel_t rx;
  const rmt_channel_t tx;
  QueueHandle_t queue;
} sensor_config;

typedef struct {
  time_t time;
  float temp;
  const char *sensor;
} temp_measurement;

static sensor_config temp_sensors[] = {
    {"room", 5, RMT_CHANNEL_0, RMT_CHANNEL_1, NULL},
    // {"street", 12, RMT_CHANNEL_2, RMT_CHANNEL_3, NULL},
};

typedef enum { Connected = BIT0, Disconnected = BIT1 } wifi_event_bit;

static const int kSensorResolution = (DS18B20_RESOLUTION_12_BIT);

// one second in ticks
static const int kSecond = 1000 / portTICK_PERIOD_MS;

// delay between two temperature measurements
static const int kTempDelay = CONFIG_TEMPERATURE_PERIOD_SECONDS * kSecond;

// keep that many hours of temperature measurements in case internet goes down
static const int kTempQueueLength =
    CONFIG_TEMPERATURE_KEEP_HOURS * 60 * 60 / CONFIG_TEMPERATURE_PERIOD_SECONDS;

static const char *const kTag = "airmon";

static DS18B20_Info *search_temp_sensor(const OneWireBus *const owb) {
  for (;;) {
    bool found = false;
    OneWireBus_SearchState search_state = {0};

    owb_search_first(owb, &search_state, &found);
    if (found) {
      break;
    }

    ESP_LOGD(kTag, "temp sensor not found, retrying");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }

  OneWireBus_ROMCode rom_code = {0};
  const owb_status status = owb_read_rom(owb, &rom_code);

  if (status == OWB_STATUS_OK) {
    char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
    owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
    ESP_LOGI(kTag, "found device %s", rom_code_s);
  } else {
    ESP_LOGE(kTag, "could not read ROM code: %d", status);
    return NULL;
  }

  // Create DS18B20 devices on the 1-Wire bus
  DS18B20_Info *const device = ds18b20_malloc(); // heap allocation
  ds18b20_init_solo(device, owb);                // only one device on bus
  ds18b20_use_crc(device, true); // enable CRC check on all reads
  ds18b20_set_resolution(device, kSensorResolution);

  return device;
}

static OneWireBus *initialize_bus(owb_rmt_driver_info *const driver_info,
                                  const sensor_config *const config) {
  OneWireBus *owb =
      owb_rmt_initialize(driver_info, config->pin, config->tx, config->rx);

  owb_use_crc(owb, true); // enable CRC check for ROM code

  return owb;
}

static void run_temp_measurements(const DS18B20_Info *const device,
                                  const sensor_config *const config) {
  int error_count = 0;

  while (error_count < 10) {
    TickType_t last_wake_time = xTaskGetTickCount();

    temp_measurement measurement;
    const DS18B20_ERROR err =
        ds18b20_convert_and_read_temp(device, &measurement.temp);

    if (err != DS18B20_OK) {
      ESP_LOGW(kTag, "measurement failed in sensor %s", config->name);
      ++error_count;
    } else {
      error_count = 0;

      struct timeval tm;
      gettimeofday(&tm, NULL);

      measurement.sensor = config->name;
      measurement.time = tm.tv_sec;

      BaseType_t sent;
      do {
        sent = xQueueSendToBack(config->queue, &measurement, kSecond);
        if (sent == errQUEUE_FULL) {
          temp_measurement buf;
          xQueueReceive(config->queue, &buf, 0);
        }
      } while (sent == errQUEUE_FULL);
    }

    vTaskDelayUntil(&last_wake_time, kTempDelay);
  }
}

_Noreturn void collect_temps(const sensor_config *const config) {
  ESP_LOGI(kTag, "starting temp collection task for %s", config->name);

  for (;;) {
    vTaskDelay(2 * kSecond);

    owb_rmt_driver_info rmt_driver_info;
    initialize_bus(&rmt_driver_info, config);

    OneWireBus *const owb = &rmt_driver_info.bus;
    DS18B20_Info *device = search_temp_sensor(owb);

    run_temp_measurements(device, config);
    ESP_LOGE(kTag, "sensor %s failed, restarting", config->name);

    // clean up dynamically allocated data
    ds18b20_free(&device);
    owb_uninitialize(owb);
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  EventGroupHandle_t wf_init_evt = arg;

  if (event_base == WIFI_EVENT) {
    if (event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect();
    }
    if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      esp_wifi_connect();
    }
  }

  if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *evt = event_data;
      ESP_LOGI(kTag, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
      xEventGroupSetBits(wf_init_evt, Connected);
    }
  }
}

void wifi_init_sta() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t wf_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wf_init_conf));

  esp_event_handler_instance_t evt_any_id, evt_got_ip;

  EventGroupHandle_t wf_init_evt = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, wf_init_evt, &evt_any_id));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, wf_init_evt, &evt_got_ip));

  wifi_config_t wf_conf = {
      .sta = {.ssid = CONFIG_WIFI_SSID,
              .password = CONFIG_WIFI_PASSWORD,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
              .pmf_cfg = {.capable = true, .required = false}},
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wf_conf));
  ESP_ERROR_CHECK(esp_wifi_start());

  for (;;) {
    EventBits_t bits = xEventGroupWaitBits(
        wf_init_evt, Connected | Disconnected, pdTRUE, pdFALSE, portMAX_DELAY);

    if (bits & Connected) {
      ESP_LOGI(kTag, "connected to wifi");
    }

    if (bits & Disconnected) {
      ESP_LOGI(kTag, "disconnected from wifi");
    }
  }

  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
      IP_EVENT, IP_EVENT_STA_GOT_IP, evt_got_ip));
  ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
      WIFI_EVENT, ESP_EVENT_ANY_ID, evt_any_id));

  vEventGroupDelete(wf_init_evt);
}

static void init_nvs() {
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);
}

_Noreturn void app_main() {
  esp_log_level_set("*", ESP_LOG_INFO);

  // To debug, use 'make menuconfig' to set default Log level to DEBUG, then
  // uncomment:
  // esp_log_level_set("owb", ESP_LOG_DEBUG);
  // esp_log_level_set("ds18b20", ESP_LOG_DEBUG);
  // esp_log_level_set("owb_rmt", ESP_LOG_DEBUG);

  init_nvs();

  xTaskCreate(wifi_init_sta, "wifi_init", 4096, NULL, 1, NULL);

  QueueHandle_t temp_queue =
      xQueueCreate(kTempQueueLength, sizeof(temp_measurement));

  if (!temp_queue) {
    ESP_LOGW(kTag, "trying a smaller temp queue");

    temp_queue = xQueueCreate(10, sizeof(temp_measurement));

    if (!temp_queue) {
      ESP_LOGE(kTag, "temp queue could not be created");
      esp_restart();
    }
  }

  for (int i = 0; i < sizeof(temp_sensors) / sizeof(*temp_sensors); ++i) {
    char name[24];

    sensor_config *conf = &temp_sensors[i];
    conf->queue = temp_queue;

    snprintf(name, sizeof(name), "temperature_%d", conf->pin);

    xTaskCreate((TaskFunction_t)collect_temps, name, 2048, conf, 1, NULL);
  }

  for (;;) {
    temp_measurement temp = {0};

    if (xQueueReceive(temp_queue, &temp, portMAX_DELAY)) {
      printf("%s: %.1f°C\n", temp.sensor, temp.temp);
    }
  }
}
