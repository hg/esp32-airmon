#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "assert.h"
#include "time.h"

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

typedef enum { MS_TEMPERATURE, MS_PARTICULATES } measurement_type;

typedef struct {
  measurement_type type;
  time_t time;
  const char *sensor;
  union {
    float temp;
    struct {
      float pm1;
      float pm2;
      float pm10;
    } part;
  } data;
} measurement;

static sensor_config temp_sensors[] = {
    {"room", 5, RMT_CHANNEL_0, RMT_CHANNEL_1, NULL},
    // {"street", 12, RMT_CHANNEL_2, RMT_CHANNEL_3, NULL},
};

typedef enum { Connected = BIT0, Disconnected = BIT1 } wifi_event_bit;

static const int64_t kMicrosecondsPerSecond = 1000 * 1000;

static const int kSensorResolution = (DS18B20_RESOLUTION_12_BIT);

// one second in ticks
static const int kSecond = 1000 / portTICK_PERIOD_MS;

// delay between two temperature measurements
static const int kTempDelay = CONFIG_TEMPERATURE_PERIOD_SECONDS * kSecond;

// keep that many hours of temperature measurements in case internet goes down
static const int kTempQueueLength =
    CONFIG_TEMPERATURE_KEEP_HOURS * 60 * 60 / CONFIG_TEMPERATURE_PERIOD_SECONDS;

// tag for application logs
static const char *const kTag = "airmon";

// time when the system was booted
static volatile time_t boot_timestamp = 0;

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
static time_t get_timestamp() {
  if (boot_timestamp > 0) {
    return time(NULL);
  } else {
    return esp_timer_get_time() / kMicrosecondsPerSecond;
  }
}

static void time_sync_notification() {
  boot_timestamp = time(NULL);
  ESP_LOGI(kTag, "sntp time update finished");
  // TODO: trigger time ready event
}

static void start_sntp_update() {
  static bool time_update_started = false;

  if (!time_update_started) {
    time_update_started = true;

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification);
    sntp_init();

    ESP_LOGI(kTag, "sntp time update started");
  }
}

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
  measurement temp = {.type = MS_TEMPERATURE, .sensor = config->name};

  while (error_count < 10) {
    TickType_t last_wake_time = xTaskGetTickCount();

    const DS18B20_ERROR err =
        ds18b20_convert_and_read_temp(device, &temp.data.temp);

    if (err != DS18B20_OK) {
      ++error_count;
      ESP_LOGW(kTag, "measurement failed in sensor %s", config->name);
    } else {
      error_count = 0;

      temp.time = get_timestamp();

      BaseType_t sent;
      do {
        sent = xQueueSendToBack(config->queue, &temp, kSecond);
        if (sent == errQUEUE_FULL) {
          measurement buf;
          xQueueReceive(config->queue, &buf, 0);
        }
      } while (sent == errQUEUE_FULL);
    }

    vTaskDelayUntil(&last_wake_time, kTempDelay);
  }
}

_Noreturn void task_collect_temps(const sensor_config *const config) {
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
      start_sntp_update();
    }
  }
}

static void wifi_init() {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t wf_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wf_init_conf));
}

_Noreturn static void task_wifi_init_sta() {
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

static void nvs_init() {
  esp_err_t ret = nvs_flash_init();

  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);
}

static void log_init() {
  esp_log_level_set("*", ESP_LOG_INFO);

  // To debug, use 'make menuconfig' to set default Log level to DEBUG, then
  // uncomment:
  // esp_log_level_set("owb", ESP_LOG_DEBUG);
  // esp_log_level_set("ds18b20", ESP_LOG_DEBUG);
  // esp_log_level_set("owb_rmt", ESP_LOG_DEBUG);
  // esp_log_level_set(kTag, ESP_LOG_DEBUG);
}

static QueueHandle_t make_measurement_queue() {
  size_t len = kTempQueueLength;
  QueueHandle_t temp_queue = NULL;

  while (!temp_queue && len > 0) {
    temp_queue = xQueueCreate(len, sizeof(measurement));
    len /= 2;
  }

  if (!temp_queue) {
    ESP_LOGE(kTag, "could not initialize temperature queue");
    esp_restart();
  }

  return temp_queue;
}

static void start_temp_tasks(const QueueHandle_t ms_queue) {
  const size_t len = sizeof(temp_sensors) / sizeof(*temp_sensors);
  char name[24];

  for (int i = 0; i < len; ++i) {
    sensor_config *const conf = &temp_sensors[i];
    conf->queue = ms_queue;

    snprintf(name, sizeof(name), "ms_temp_%d", conf->pin);

    xTaskCreate((TaskFunction_t)task_collect_temps, name, 2048, conf, 1, NULL);
  }
}

static bool is_valid_timestamp(const time_t ts) {
  const time_t min_valid_ts = 1577836800; // 2020-01-01 UTC
  return ts >= min_valid_ts;
}

_Noreturn void app_main() {
  log_init();
  nvs_init();
  wifi_init();

  xTaskCreate(task_wifi_init_sta, "wifi_init", 2048, NULL, 1, NULL);

  const QueueHandle_t ms_queue = make_measurement_queue();
  start_temp_tasks(ms_queue);

  for (int i = 0; i < 60 * 60; ++i) {
    vTaskDelay(kSecond);
  }

  for (;;) {
    struct tm timeinfo;
    char time_str[20];
    measurement ms;

    if (xQueueReceive(ms_queue, &ms, portMAX_DELAY)) {
      if (!is_valid_timestamp(ms.time)) {
        ms.time += boot_timestamp;
      }

      localtime_r(&ms.time, &timeinfo);
      strftime(time_str, sizeof(time_str), "%F %H:%M:%S", &timeinfo);

      printf("%s\t%s\t", time_str, ms.sensor);

      switch (ms.type) {
      case MS_TEMPERATURE:
        printf("%.1f°C\n", ms.data.temp);
        break;

      case MS_PARTICULATES:
        printf("PM1 %.2f, PM2.5 %.2f, PM10 %.2f\n", ms.data.part.pm1,
               ms.data.part.pm2, ms.data.part.pm10);
        break;
      }
    }
  }
}
