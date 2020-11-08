#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_wifi.h"

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

static const int64_t kMicrosecondsPerSecond = 1000 * 1000;

// one second in ticks
static const int kSecond = 1000 / portTICK_PERIOD_MS;

// delay between two temperature measurements
static const int kTempDelay = CONFIG_TEMPERATURE_PERIOD_SECONDS * kSecond;

// delay between two particulate matter measurements
// static const int kPartDelay = CONFIG_PARTICULATE_PERIOD_SECONDS * kSecond;

static StaticQueue_t measurement_queue;
static measurement measurement_queue_buf[CONFIG_MEASUREMENT_QUEUE_SIZE];

// tag for application logs
static const char *const kTag = "airmon";

// UNIX time when the system was booted
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

static void sntp_update_start_once() {
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

  if (status != OWB_STATUS_OK) {
    ESP_LOGE(kTag, "could not read ROM code: %d", status);
    return NULL;
  }

  char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
  owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
  ESP_LOGI(kTag, "found device %s", rom_code_s);

  // Create DS18B20 devices on the 1-Wire bus
  DS18B20_Info *const device = ds18b20_malloc(); // heap allocation
  ds18b20_init_solo(device, owb);                // only one device on bus
  ds18b20_use_crc(device, true); // enable CRC check on all reads
  ds18b20_set_resolution(device, DS18B20_RESOLUTION_12_BIT);

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

    if (device) {
      run_temp_measurements(device, config);
      ds18b20_free(&device);
    }

    owb_uninitialize(owb);

    ESP_LOGE(kTag, "sensor %s failed, restarting", config->name);
  }
}

static void handle_ip_event(void *const arg, const esp_event_base_t event_base,
                            const int32_t event_id, void *const event_data) {
  assert(event_base == IP_EVENT);

  switch (event_id) {
  case IP_EVENT_STA_GOT_IP: {
    const ip_event_got_ip_t *const evt = event_data;
    ESP_LOGI(kTag, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
    sntp_update_start_once();
    break;
  }

  case IP_EVENT_STA_LOST_IP:
    ESP_LOGI(kTag, "lost ip");
    break;

  default:
    ESP_LOGI(kTag, "unexpected ip event %d", event_id);
    break;
  }
}

static void handle_wifi_event(void *const arg,
                              const esp_event_base_t event_base,
                              const int32_t event_id, void *const event_data) {
  assert(event_base == WIFI_EVENT);

  switch (event_id) {
  case WIFI_EVENT_STA_START:
    esp_wifi_connect();
    break;

  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGI(kTag, "connected to AP");
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGI(kTag, "disconnected from AP");
    esp_wifi_connect();
    break;

  default:
    ESP_LOGI(kTag, "unexpected sta event %d", event_id);
    break;
  }
}

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-lwip-init-phase
static void app_init_wifi() {
  // initialize LwIP and main event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // create station interface
  esp_netif_create_default_wifi_sta();

  // initialize Wi-Fi driver
  wifi_init_config_t wf_init_conf = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wf_init_conf));

  // bind event handlers
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, handle_wifi_event, NULL, NULL));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, handle_ip_event, NULL, NULL));

  // connect to station
  wifi_config_t wf_conf = {
      .sta = {.ssid = CONFIG_WIFI_SSID,
              .password = CONFIG_WIFI_PASSWORD,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
              .pmf_cfg = {.capable = true, .required = false}},
  };
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wf_conf));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static void app_init_log() {
  esp_log_level_set("*", ESP_LOG_INFO);

  // To debug, use 'make menuconfig' to set default Log level to DEBUG, then
  // uncomment:
  // esp_log_level_set("owb", ESP_LOG_DEBUG);
  // esp_log_level_set("ds18b20", ESP_LOG_DEBUG);
  // esp_log_level_set("owb_rmt", ESP_LOG_DEBUG);
  // esp_log_level_set(kTag, ESP_LOG_DEBUG);
}

static QueueHandle_t make_measurement_queue() {
  const QueueHandle_t queue =
      xQueueCreateStatic(CONFIG_MEASUREMENT_QUEUE_SIZE, sizeof(measurement),
                         (uint8_t *)measurement_queue_buf, &measurement_queue);

  configASSERT(queue);

  return queue;
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
  app_init_log();
  app_init_wifi();

  const QueueHandle_t ms_queue = make_measurement_queue();
  start_temp_tasks(ms_queue);

  // wait for SNTP time update for at most 10 minutes
  for (int i = 0; boot_timestamp == 0 && i < 10 * 60; ++i) {
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
