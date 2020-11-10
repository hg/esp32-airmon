#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "time.h"

#include "ds18b20.h"
#include "owb.h"
#include "owb_rmt.h"

#define KiB(kb) (kb * 1024)

typedef struct {
  const char *name;
  const gpio_num_t pin;
  const rmt_channel_t rx;
  const rmt_channel_t tx;
  QueueHandle_t queue;
} sensor_config;

typedef struct {
  enum { MS_TEMPERATURE, MS_PARTICULATES } type;
  time_t time;
  const char *sensor;
  union {
    float temp;
    struct {
      float pm1;
      float pm2;
      float pm10;
    } pm;
  };
} measurement;

static sensor_config temp_sensors[] = {
    {"room", GPIO_NUM_5, RMT_CHANNEL_0, RMT_CHANNEL_1, NULL},
    // {"street", 12, RMT_CHANNEL_2, RMT_CHANNEL_3, NULL},
};

// delay between two temperature measurements
static const int delay_temp =
    CONFIG_TEMPERATURE_PERIOD_SECONDS * configTICK_RATE_HZ;

// delay between two particulate matter measurements
// static const int kPartDelay = CONFIG_PARTICULATE_PERIOD_SECONDS * kSecond;

static struct {
  StaticQueue_t queue;
  measurement buffer[CONFIG_MEASUREMENT_QUEUE_SIZE];
} measurement_queue;

typedef struct {
  esp_mqtt_client_handle_t handle;
  psk_hint_key_t psk_hint;
  char msg[256];
  EventGroupHandle_t event;
  const char *cmd_topic;
  const char *resp_topic;
} mqtt_client;

typedef struct {
  const char *const cmd;
  bool (*handler)(mqtt_client *const client, const esp_mqtt_event_handle_t evt);
} mqtt_cmd_handler;

// tag for application logs
static const char *const kTag = "airmon";

// UNIX time when the system was booted
static time_t boot_timestamp = 0;

enum {
  STATE_TIME_VALID = BIT0,
  STATE_NET_CONNECTED = BIT1,
  STATE_NET_DISCONNECTED = BIT2,
};

static EventGroupHandle_t state_evt;

static void wait_state(const EventBits_t bits) {
  xEventGroupWaitBits(state_evt, bits, false, true, portMAX_DELAY);
}

enum {
  MQTT_READY = BIT0,
  MQTT_ACK = BIT1,
};

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
static time_t get_timestamp() {
  const int64_t us_per_sec = 1000 * 1000;

  if (boot_timestamp > 0) {
    return time(NULL);
  } else {
    return esp_timer_get_time() / us_per_sec;
  }
}

static bool mqtt_is_broadcast_cmd(const esp_mqtt_event_handle_t evt) {
  return strncmp(evt->topic, "cmd/*", evt->topic_len) == 0;
}

static bool mqtt_handle_ping(mqtt_client *const client,
                             const esp_mqtt_event_handle_t evt) {
  const bool bcast = mqtt_is_broadcast_cmd(evt);
  const char *const topic = bcast ? "response/*" : client->resp_topic;
  const char *const resp = bcast ? CONFIG_DEV_NAME ": pong" : "pong";
  return esp_mqtt_client_publish(client->handle, topic, resp, 0, 1, 0) != -1;
}

_Noreturn static bool mqtt_handle_restart(mqtt_client *const client,
                                          const esp_mqtt_event_handle_t evt) {
  esp_mqtt_client_publish(client->handle, evt->topic, "restarting", 0, 1, 0);
  esp_restart();
}

static const mqtt_cmd_handler mqtt_handlers[] = {
    {"ping", mqtt_handle_ping},
    {"restart", mqtt_handle_restart},
};

static void mqtt_subscribe_to_commands(const mqtt_client *const client) {
  esp_mqtt_client_subscribe(client->handle, "cmd/*", 2);
  esp_mqtt_client_subscribe(client->handle, client->cmd_topic, 2);
}

static bool mqtt_handle_message(mqtt_client *const client,
                                const esp_mqtt_event_handle_t evt) {
  const size_t num_handlers = sizeof(mqtt_handlers) / sizeof(*mqtt_handlers);

  for (size_t i = 0; i < num_handlers; ++i) {
    const mqtt_cmd_handler *const handler = &mqtt_handlers[i];
    if (strncmp(handler->cmd, evt->data, evt->data_len) == 0) {
      return handler->handler(client, evt);
    }
  }

  return false;
}

static esp_err_t mqtt_event_handler(const esp_mqtt_event_handle_t evt) {
  mqtt_client *const client = evt->user_context;

  switch (evt->event_id) {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(client->event, MQTT_READY);
    mqtt_subscribe_to_commands(client);
    ESP_LOGI(kTag, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    xEventGroupClearBits(client->event, MQTT_READY);
    ESP_LOGI(kTag, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    xEventGroupSetBits(client->event, MQTT_ACK);
    ESP_LOGD(kTag, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA:
    ESP_LOGI(kTag, "mqtt message received (id %d)", evt->msg_id);
    if (!mqtt_handle_message(client, evt)) {
      ESP_LOGI(kTag, "could not handle message (id %d)", evt->msg_id);
    }
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(kTag, "mqtt subscription successful (%d)", evt->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(kTag, "mqtt error %d", evt->error_handle->error_type);
    break;

  default:
    break;
  }

  return ESP_OK;
}

// convert hex string to array of bytes and return how many bytes were stored
static size_t hex_str_to_bytes(const char *const str, uint8_t *const buf,
                               const size_t buf_len) {
  const size_t hex_len = strlen(str);
  const size_t psk_len = hex_len / 2;

  configASSERT(hex_len % 2 == 0);

  for (size_t i = 0; i < psk_len; ++i) {
    char b[3] = {0, 0, 0};
    b[0] = str[i * 2];
    b[1] = str[i * 2 + 1];
    buf[i] = strtol(b, NULL, 16);
  }

  return psk_len;
}

static mqtt_client *mqtt_client_create(const char *const broker_uri,
                                       const char *const psk_hex) {
  const size_t hex_len = strlen(psk_hex);
  if (hex_len % 2) {
    ESP_LOGE(kTag, "invalid psk hex length");
    return NULL;
  }

  const size_t psk_len = hex_len / 2;
  uint8_t *const psk = malloc(psk_len);

  if (hex_str_to_bytes(psk_hex, psk, psk_len) != psk_len) {
    free(psk);
    ESP_LOGE(kTag, "could not parse psk hex");
    return NULL;
  }

  mqtt_client *const client = malloc(sizeof(mqtt_client));

  client->psk_hint.hint = CONFIG_MQTT_HINT;
  client->psk_hint.key = psk;
  *((size_t *)&(client->psk_hint.key_size)) = psk_len;

  client->cmd_topic = "cmd/" CONFIG_DEV_NAME;
  client->resp_topic = "response/" CONFIG_DEV_NAME;

  const esp_mqtt_client_config_t conf = {.uri = CONFIG_MQTT_BROKER_URI,
                                         .event_handle = mqtt_event_handler,
                                         .user_context = client,
                                         .psk_hint_key = &client->psk_hint,
                                         .keepalive = 30};

  client->handle = esp_mqtt_client_init(&conf);
  if (client->handle == NULL) {
    ESP_LOGE(kTag, "esp_mqtt_client_init failed");
    goto cleanup;
  }

  if (esp_mqtt_client_start(client->handle) != ESP_OK) {
    ESP_LOGE(kTag, "esp_mqtt_client_start failed");
    goto cleanup;
  }

  client->event = xEventGroupCreate();

  return client;

cleanup:
  free(psk);
  free(client);
  return NULL;
}

static void mqtt_client_free(mqtt_client *const client) {
  if (!client || !client->handle) {
    ESP_LOGE(kTag, "attempt to free already destroyed mqtt client");
  } else {
    esp_mqtt_client_stop(client->handle);
    esp_mqtt_client_destroy(client->handle);
    vEventGroupDelete(client->event);
    client->handle = NULL;
    free(client);
  }
}

static void time_sync_notification() {
  if (boot_timestamp == 0) {
    boot_timestamp = time(NULL);
    xEventGroupSetBits(state_evt, STATE_TIME_VALID);
  }
  ESP_LOGI(kTag, "sntp time update finished");
}

static void task_sntp_update() {
  wait_state(STATE_NET_CONNECTED);

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(time_sync_notification);
  sntp_init();

  ESP_LOGI(kTag, "sntp time update started");

  vTaskDelete(NULL);
}

static DS18B20_Info *search_temp_sensor(const OneWireBus *const owb) {
  for (bool found = false; !found;) {
    OneWireBus_SearchState search_state = {0};
    const owb_status status = owb_search_first(owb, &search_state, &found);

    if (status != OWB_STATUS_OK) {
      ESP_LOGE(kTag, "owb search failed: %d", status);
      return NULL;
    }
    if (!found) {
      ESP_LOGD(kTag, "temp sensor not found, retrying");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }

  OneWireBus_ROMCode rom_code = {0};
  const owb_status status = owb_read_rom(owb, &rom_code);

  if (status != OWB_STATUS_OK) {
    ESP_LOGE(kTag, "could not read ROM code: %d", status);
    return NULL;
  }

  char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
  owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
  ESP_LOGI(kTag, "found device 0x%s", rom_code_s);

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

static void queue_send_retrying(const QueueHandle_t queue,
                                const void *const data,
                                const size_t data_size) {
  BaseType_t sent;
  do {
    sent = xQueueSendToBack(queue, data, pdMS_TO_TICKS(1000));
    if (sent == errQUEUE_FULL) {
      uint8_t buf[data_size];
      xQueueReceive(queue, buf, 0);
    }
  } while (sent == errQUEUE_FULL);
}

static void run_temp_measurements(const DS18B20_Info *const device,
                                  const sensor_config *const config) {
  int error_count = 0;
  measurement ms = {.type = MS_TEMPERATURE, .sensor = config->name};
  TickType_t last_wake_time = xTaskGetTickCount();

  while (error_count < 4) {
    const DS18B20_ERROR err = ds18b20_convert_and_read_temp(device, &ms.temp);

    if (err != DS18B20_OK) {
      ++error_count;
      ESP_LOGW(kTag, "measurement failed in %s, err %d", config->name, err);
    } else {
      error_count = 0;
      ms.time = get_timestamp();
      queue_send_retrying(config->queue, &ms, sizeof(measurement));
    }

    vTaskDelayUntil(&last_wake_time, delay_temp);
  }
}

_Noreturn void task_collect_temps(const sensor_config *const config) {
  ESP_LOGI(kTag, "starting temp collection task for %s", config->name);

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(2000));

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
  configASSERT(event_base == IP_EVENT);

  switch (event_id) {
  case IP_EVENT_STA_GOT_IP: {
    xEventGroupClearBits(state_evt, STATE_NET_DISCONNECTED);
    xEventGroupSetBits(state_evt, STATE_NET_CONNECTED);

    const ip_event_got_ip_t *const evt = event_data;
    ESP_LOGI(kTag, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
    break;
  }

  case IP_EVENT_STA_LOST_IP:
    xEventGroupClearBits(state_evt, STATE_NET_CONNECTED);
    xEventGroupSetBits(state_evt, STATE_NET_DISCONNECTED);
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
  configASSERT(event_base == WIFI_EVENT);

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
#ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_DEBUG);
#else
  esp_log_level_set("*", ESP_LOG_INFO);
#endif
}

static QueueHandle_t make_measurement_queue() {
  const size_t item_size = sizeof(*measurement_queue.buffer);

  const QueueHandle_t queue = xQueueCreateStatic(
      sizeof(measurement_queue.buffer) / item_size, item_size,
      (uint8_t *)measurement_queue.buffer, &measurement_queue.queue);

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

    xTaskCreate((TaskFunction_t)task_collect_temps, name, KiB(2), conf, 2,
                NULL);
  }
}

static void start_pm_task(const QueueHandle_t ms_queue) {
  // TODO: implement pollution collector, currently no-op
  // xTaskCreate(task_collect_pm, "ms_pm", KiB(2), ms_queue, 2, NULL);
}

static bool is_valid_timestamp(const time_t ts) {
  const time_t min_valid_ts = 1577836800; // 2020-01-01 UTC
  return ts >= min_valid_ts;
}

static void format_temp_msg(char *const msg, const size_t len,
                            const measurement *const ms) {
  snprintf(msg, len,
           "{"
           "\"dev\":\"" CONFIG_DEV_NAME "\","
           "\"time\":%ld,"
           "\"sens\":\"%s\","
           "\"temp\":%f"
           "}",
           ms->time, ms->sensor, ms->temp);
}

static void format_part_msg(char *const msg, const size_t len,
                            const measurement *const ms) {
  snprintf(msg, len,
           "{"
           "\"dev\":\"" CONFIG_DEV_NAME "\","
           "\"time\":%ld,"
           "\"sens\":\"%s\","
           "\"pm1\":%f,"
           "\"pm2\":%f,"
           "\"pm10\":%f,"
           "}",
           ms->time, ms->sensor, ms->pm.pm1, ms->pm.pm2, ms->pm.pm10);
}

static bool mqtt_send_measurement(mqtt_client *const mq,
                                  const measurement *const ms) {
  const char *type = "";

  switch (ms->type) {
  case MS_TEMPERATURE:
    format_temp_msg(mq->msg, sizeof(mq->msg), ms);
    type = "meas/temp";
    break;

  case MS_PARTICULATES:
    format_part_msg(mq->msg, sizeof(mq->msg), ms);
    type = "meas/part";
    break;

  default:
    ESP_LOGE(kTag, "invalid message type %d", ms->type);
    return false;
  }

  for (int result = -1; result < 0;) {
    result = esp_mqtt_client_publish(mq->handle, type, mq->msg, 0, 1, 1);
    if (result < 0) {
      ESP_LOGI(kTag, "mqtt publish failed, retrying");
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }

  return true;
}

static void mqtt_client_wait_ready(mqtt_client *const client) {
  xEventGroupWaitBits(client->event, MQTT_READY, false, false, portMAX_DELAY);
}

static void restart_peripheral(const periph_module_t module) {
  periph_module_disable(module);
  periph_module_enable(module);
}

_Noreturn void app_main() {
  app_init_log();

  // reset peripherals in case of prior crash
  restart_peripheral(PERIPH_RMT_MODULE);
  restart_peripheral(PERIPH_UART0_MODULE);

  state_evt = xEventGroupCreate();

  app_init_wifi();

  xTaskCreate(task_sntp_update, "sntp_update", KiB(2), NULL, 1, NULL);

  // start pollution collectors right away, we can fix time later
  const QueueHandle_t ms_queue = make_measurement_queue();
  start_temp_tasks(ms_queue);
  start_pm_task(ms_queue);

  wait_state(STATE_NET_CONNECTED);

  mqtt_client *const client =
      mqtt_client_create(CONFIG_MQTT_BROKER_URI, CONFIG_MQTT_PSK);

  if (!client) {
    ESP_LOGE(kTag, "could not initialize mqtt client");
    esp_restart();
  }

  wait_state(STATE_TIME_VALID);

  for (measurement ms;;) {
    mqtt_client_wait_ready(client);

    xQueueReceive(ms_queue, &ms, portMAX_DELAY);

    // fix time if it was assigned before SNTP data became available
    if (!is_valid_timestamp(ms.time)) {
      ms.time += boot_timestamp;
    }

    if (!mqtt_send_measurement(client, &ms)) {
      ESP_LOGE(kTag, "measurement send failed");
    }
  }

  mqtt_client_free(client);
  esp_restart();
}
