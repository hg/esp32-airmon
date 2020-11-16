#include "algorithm"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "driver/uart.h"
#include "ds18b20.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_sntp.h"
#include "esp_system.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "functional"
#include "lwip/err.h"
#include "mqtt_client.h"
#include "numeric"
#include "nvs.h"
#include "nvs_flash.h"
#include "owb.h"
#include "owb_rmt.h"
#include "queue.hh"
#include "time.h"
#include "timer.hh"
#include "utils.hh"

// tag for application logs
static const char *const logTag = CONFIG_DEV_NAME;

extern const uint8_t caPemStart[] asm("_binary_ca_pem_start");

// UNIX time when the system was booted
static time_t bootTimestamp = 0;

constexpr time_t minValidTimestamp = 1577836800; // 2020-01-01 UTC

static constexpr bool isValidTimestamp(const time_t ts) {
  return ts >= minValidTimestamp;
}

struct {
  const char *devName;
  struct {
    const char *ssid;
    const char *pass;
  } wifi;
  struct {
    const char *broker;
    const char *username;
    const char *password;
  } mqtt;
} appSettings;

// delay between two temperature measurements
static const int delayTemp = secToTicks(CONFIG_TEMPERATURE_PERIOD_SECONDS);

// delay between two particulate matter measurements
static const int delayPm = secToTicks(CONFIG_PARTICULATE_PERIOD_SECONDS);

struct PmsResponse {
  uint16_t magic;
  uint16_t frameLen;
  uint16_t pm1McgStd;
  uint16_t pm2McgStd;
  uint16_t pm10McgStd;
  uint16_t pm1McgAtm;
  uint16_t pm2McgAtm;
  uint16_t pm10McgAtm;
  uint16_t pm03Count;
  uint16_t pm05Count;
  uint16_t pm1Count;
  uint16_t pm2Count;
  uint16_t pm5Count;
  uint16_t pm10Count;
  uint16_t reserved;
  uint16_t checksum;

  uint16_t calcChecksum() const;
  void swapBytes();
} __attribute__((packed));

struct PmMeasurementSum {
  uint32_t count;
  struct {
    uint32_t pm1Mcg;
    uint32_t pm2Mcg;
    uint32_t pm10Mcg;
  } std;
  struct {
    uint32_t pm1Mcg;
    uint32_t pm2Mcg;
    uint32_t pm10Mcg;
  } atm;
  struct {
    uint32_t pm03Count;
    uint32_t pm05Count;
    uint32_t pm1Count;
    uint32_t pm2Count;
    uint32_t pm5Count;
    uint32_t pm10Count;
  } cnt;

  void addMeasurement(const PmsResponse &resp);
  void reset();
};

enum class MeasurementType { MS_TEMPERATURE, MS_PARTICULATES };

struct Measurement {
  MeasurementType type;
  time_t time;
  const char *sensor;
  union {
    float temp;
    struct {
      struct {
        uint16_t pm1Mcg;
        uint16_t pm2Mcg;
        uint16_t pm10Mcg;
      } std;
      struct {
        uint16_t pm1Mcg;
        uint16_t pm2Mcg;
        uint16_t pm10Mcg;
      } atm;
      struct {
        uint16_t pm03Count;
        uint16_t pm05Count;
        uint16_t pm1Count;
        uint16_t pm2Count;
        uint16_t pm5Count;
        uint16_t pm10Count;
      } cnt;
    } pm;
  };

  // fixes time if it was assigned before SNTP data became available
  void fixTime() {
    if (!isValidTimestamp(time)) {
      time += bootTimestamp;
    }
  }

  void set(const PmsResponse &res) {
    pm.atm.pm1Mcg = res.pm1McgAtm;
    pm.atm.pm2Mcg = res.pm2McgAtm;
    pm.atm.pm10Mcg = res.pm10McgAtm;

    pm.std.pm1Mcg = res.pm1McgStd;
    pm.std.pm2Mcg = res.pm2McgStd;
    pm.std.pm10Mcg = res.pm10McgStd;

    pm.cnt.pm03Count = res.pm03Count;
    pm.cnt.pm05Count = res.pm05Count;
    pm.cnt.pm1Count = res.pm1Count;
    pm.cnt.pm2Count = res.pm2Count;
    pm.cnt.pm5Count = res.pm5Count;
    pm.cnt.pm10Count = res.pm10Count;
  }

  void set(const PmMeasurementSum &sum) {
    pm.atm.pm1Mcg = sum.atm.pm1Mcg / sum.count;
    pm.atm.pm2Mcg = sum.atm.pm2Mcg / sum.count;
    pm.atm.pm10Mcg = sum.atm.pm10Mcg / sum.count;

    pm.std.pm1Mcg = sum.std.pm1Mcg / sum.count;
    pm.std.pm2Mcg = sum.std.pm2Mcg / sum.count;
    pm.std.pm10Mcg = sum.std.pm10Mcg / sum.count;

    pm.cnt.pm03Count = sum.cnt.pm03Count / sum.count;
    pm.cnt.pm05Count = sum.cnt.pm05Count / sum.count;
    pm.cnt.pm1Count = sum.cnt.pm1Count / sum.count;
    pm.cnt.pm2Count = sum.cnt.pm2Count / sum.count;
    pm.cnt.pm5Count = sum.cnt.pm5Count / sum.count;
    pm.cnt.pm10Count = sum.cnt.pm10Count / sum.count;
  }

  const char *getType() const {
    switch (type) {

    case MeasurementType::MS_TEMPERATURE:
      return "meas/temp";

    case MeasurementType::MS_PARTICULATES:
      return "meas/part";

    default:
      return nullptr;
    }
  }

  bool formatMsg(char *const msg, const size_t len) const {
    switch (type) {
    case MeasurementType::MS_TEMPERATURE: {
      constexpr auto tpl = R"({"dev":"%s","time":%ld,"sens":"%s","temp":%f})";
      snprintf(msg, len, tpl, appSettings.devName, time, sensor, temp);
      return true;
    }

    case MeasurementType::MS_PARTICULATES: {
      constexpr auto tpl =
          R"({"dev":"%s","time":%ld,"sens":"%s","std":{"pm1":%u,"pm2.5":%u,"pm10":%u},"atm":{"pm1":%u,"pm2.5":%u,"pm10":%u},"cnt":{"pm0.3":%u,"pm0.5":%u,"pm1":%u,"pm2.5":%u,"pm5":%u,"pm10":%u}})";
      snprintf(msg, len, tpl, appSettings.devName, time, sensor, pm.std.pm1Mcg,
               pm.std.pm2Mcg, pm.std.pm10Mcg, pm.atm.pm1Mcg, pm.atm.pm2Mcg,
               pm.atm.pm10Mcg, pm.cnt.pm03Count, pm.cnt.pm05Count,
               pm.cnt.pm1Count, pm.cnt.pm2Count, pm.cnt.pm5Count,
               pm.cnt.pm10Count);
      return true;
    }

    default:
      ESP_LOGE(logTag, "invalid message type %d", static_cast<int>(type));
      return false;
    }
  }
};

struct TempSensor {
  const char *name;
  const gpio_num_t pin;
  const rmt_channel_t rxChan;
  const rmt_channel_t txChan;
  Queue<Measurement> *queue;
};

uint16_t PmsResponse::calcChecksum() const {
  return std::accumulate(reinterpret_cast<const uint8_t *>(&magic),
                         reinterpret_cast<const uint8_t *>(&checksum), 0);
}

void PmsResponse::swapBytes() {
  std::transform(&frameLen, (&checksum) + 1, &frameLen,
                 [](uint16_t num) -> uint16_t { return ntohs(num); });
}

struct PmsCommand {
  uint16_t magic;
  uint8_t command;
  uint16_t data;
  uint16_t checksum;
} __attribute__((packed));

struct PmsStation {
  const char *name;
  const uart_port_t port;
  const gpio_num_t rxPin;
  const gpio_num_t txPin;
  Queue<Measurement> *queue;

  int readResponse(PmsResponse &resp, const TickType_t wait) const {
    return uart_read_bytes(port, &resp, sizeof(resp), wait);
  }

  int writeCommand(const PmsCommand &cmd) const {
    return uart_write_bytes(port, &cmd, sizeof(cmd));
  }

  esp_err_t flushInput() const { return uart_flush_input(port); }
};

static constexpr size_t pmsFrameLen = sizeof(PmsResponse) -
                                      sizeof(PmsResponse::magic) -
                                      sizeof(PmsResponse::frameLen);

void PmMeasurementSum::reset() { memset(this, 0, sizeof(*this)); }

void PmMeasurementSum::addMeasurement(const PmsResponse &resp) {
  atm.pm1Mcg += resp.pm1McgAtm;
  atm.pm2Mcg += resp.pm2McgAtm;
  atm.pm10Mcg += resp.pm10McgAtm;

  std.pm1Mcg += resp.pm1McgStd;
  std.pm2Mcg += resp.pm2McgStd;
  std.pm10Mcg += resp.pm10McgStd;

  cnt.pm03Count += resp.pm03Count;
  cnt.pm05Count += resp.pm05Count;
  cnt.pm1Count += resp.pm1Count;
  cnt.pm2Count += resp.pm2Count;
  cnt.pm5Count += resp.pm5Count;
  cnt.pm10Count += resp.pm10Count;

  ++count;
}

static TempSensor tempSensors[] = {
    {.name = "room",
     .pin = GPIO_NUM_5,
     .rxChan = RMT_CHANNEL_0,
     .txChan = RMT_CHANNEL_1,
     .queue = nullptr},
};

static PmsStation pmsStations[] = {
    {.name = "room",
     .port = UART_NUM_1,
     .rxPin = GPIO_NUM_25,
     .txPin = GPIO_NUM_27,
     .queue = nullptr},
};

enum {
  MqttReady = BIT0,
  MqttAck = BIT1,
};

struct MqttClient {
  esp_mqtt_client_handle_t handle;
  EventGroupHandle_t event;
  char *cmdTopic;
  const char *respTopic;
  const char *cert;

  void waitReady() const {
    xEventGroupWaitBits(event, MqttReady, false, false, portMAX_DELAY);
  }

  bool send(const char *const topic, const char *const data) {
    for (int result = -1; result < 0;) {
      result = esp_mqtt_client_publish(handle, topic, data, 0, 1, 1);
      if (result < 0) {
        ESP_LOGI(logTag, "mqtt publish failed, retrying");
        vTaskDelay(secToTicks(5));
      }
    }

    return true;
  }
};

using MqttHandlerFunc = bool(MqttClient *const client,
                             const esp_mqtt_event_handle_t evt,
                             const char *const respTopic,
                             const char *const args);

struct MqttCmdHandler {
  const char *const cmd;
  std::function<MqttHandlerFunc> handler;
};

static const char *const mqttFallbackResponseTopic = "response/*";

enum AppState {
  STATE_TIME_VALID = BIT0,
  STATE_NET_CONNECTED = BIT1,
  STATE_NET_DISCONNECTED = BIT2,
};

static EventGroupHandle_t appState;

static void waitState(const AppState bits) {
  xEventGroupWaitBits(appState, static_cast<EventBits_t>(bits), false, true,
                      portMAX_DELAY);
}

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
static time_t getTimestamp() {
  const int64_t us_per_sec = 1000 * 1000;

  if (bootTimestamp > 0) {
    return time(nullptr);
  } else {
    return esp_timer_get_time() / us_per_sec;
  }
}

static bool mqttIsBroadcastCmd(const esp_mqtt_event_handle_t evt) {
  return strncmp(evt->topic, "cmd/*", evt->topic_len) == 0;
}

static bool mqttHandlePing(MqttClient *const client,
                           const esp_mqtt_event_handle_t evt,
                           const char *const respTopic,
                           const char *const args) {
  const char *data;
  char buf[64];

  if (mqttIsBroadcastCmd(evt)) {
    snprintf(buf, sizeof(buf), "pong: %s", appSettings.devName);
    data = buf;
  } else {
    data = "pong";
  }

  return esp_mqtt_client_publish(client->handle, respTopic, data, 0, 1, 0) !=
         -1;
}

[[noreturn]] static bool mqttHandleRestart(MqttClient *const client,
                                           const esp_mqtt_event_handle_t evt,
                                           const char *const respTopic,
                                           const char *const args) {
  esp_mqtt_client_publish(client->handle, respTopic, "restarting", 0, 1, 0);
  esp_restart();
}

static bool mqttHandleOta(MqttClient *const client,
                          const esp_mqtt_event_handle_t evt,
                          const char *const respTopic, const char *const args) {
  const esp_http_client_config_t config{
      .url = args,
      .cert_pem = client->cert,
  };

  const esp_err_t ret = esp_https_ota(&config);

  if (ret == ESP_OK) {
    ESP_LOGI(logTag, "OTA update finished");
    esp_mqtt_client_publish(evt->client, respTopic, "OTA success", 0, 1, 0);
    esp_restart();
  }

  ESP_LOGE(logTag, "could not perform OTA update: 0x%x", ret);
  esp_mqtt_client_publish(evt->client, respTopic, "OTA failed", 0, 1, 0);

  return false;
}

static const MqttCmdHandler mqttHandlers[] = {
    {"ping", mqttHandlePing},
    {"restart", mqttHandleRestart},
    {"ota", mqttHandleOta},
};

static void mqttSubscribeToCommands(const MqttClient *const client) {
  esp_mqtt_client_subscribe(client->handle, "cmd/*", 2);
  if (client->cmdTopic) {
    esp_mqtt_client_subscribe(client->handle, client->cmdTopic, 2);
  }
}

static bool mqttHandleMessage(MqttClient *const client,
                              const esp_mqtt_event_handle_t evt) {
  constexpr size_t numHandlers = sizeof(mqttHandlers) / sizeof(*mqttHandlers);

  for (size_t i = 0; i < numHandlers; ++i) {
    const MqttCmdHandler &handler = mqttHandlers[i];
    const size_t cmdLen = strlen(handler.cmd);

    if (evt->data_len < cmdLen) {
      continue;
    }
    if (strncmp(handler.cmd, evt->data, cmdLen)) {
      continue;
    }

    const char *const topic =
        mqttIsBroadcastCmd(evt) ? "response/*" : client->respTopic;

    if (evt->data_len > cmdLen) {
      size_t argsPos = cmdLen;
      if (isspace(evt->data[argsPos])) {
        ++argsPos;
      }
      char args[evt->data_len - argsPos + 1];
      strncpy(args, &evt->data[argsPos], sizeof(args));
      args[sizeof(args) - 1] = '\0';

      return handler.handler(client, evt, topic, args);
    } else {
      return handler.handler(client, evt, topic, nullptr);
    }
  }

  return false;
}

static esp_err_t mqttEventHandler(const esp_mqtt_event_handle_t evt) {
  MqttClient *const client = reinterpret_cast<MqttClient *>(evt->user_context);

  switch (evt->event_id) {
  case MQTT_EVENT_CONNECTED:
    xEventGroupSetBits(client->event, MqttReady);
    mqttSubscribeToCommands(client);
    ESP_LOGI(logTag, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    xEventGroupClearBits(client->event, MqttReady);
    ESP_LOGI(logTag, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    xEventGroupSetBits(client->event, MqttAck);
    ESP_LOGD(logTag, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA:
    ESP_LOGI(logTag, "mqtt message received (id %d)", evt->msg_id);
    if (!mqttHandleMessage(client, evt)) {
      ESP_LOGI(logTag, "could not handle message (id %d)", evt->msg_id);
    }
    break;

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(logTag, "mqtt subscription successful (%d)", evt->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(logTag, "mqtt error %d", evt->error_handle->error_type);
    break;

  default:
    break;
  }

  return ESP_OK;
}

static char *mqttPrepareTopic(const char *const prefix,
                              const char *const suffix) {
  // prefix + '/' + suffix + '\0'
  const size_t topicLen = strlen(prefix) + 1 + strlen(suffix) + 1;
  char *const cmdTopic = new char[topicLen];
  snprintf(cmdTopic, topicLen, "%s/%s", prefix, suffix);
  return cmdTopic;
}

static MqttClient *mqttClientCreate(const char *const brokerUri,
                                    const uint8_t *const caCert,
                                    const char *const username,
                                    const char *const password) {
  MqttClient *const client = new MqttClient;

  client->cmdTopic = mqttPrepareTopic("cmd", username);
  client->respTopic = mqttPrepareTopic("response", username);

  client->cert = reinterpret_cast<const char *>(caCert);

  const esp_mqtt_client_config_t conf = {
      .event_handle = mqttEventHandler,
      .uri = brokerUri,
      .username = username,
      .password = password,
      .keepalive = 30,
      .user_context = client,
      .cert_pem = client->cert,
  };

  client->handle = esp_mqtt_client_init(&conf);
  configASSERT(client->handle);

  ESP_ERROR_CHECK(esp_mqtt_client_start(client->handle));

  client->event = xEventGroupCreate();

  return client;
}

static esp_err_t mqttClientFree(MqttClient *const client) {
  if (!client || !client->handle) {
    ESP_LOGE(logTag, "attempt to free already destroyed mqtt client");
    return ESP_ERR_INVALID_STATE;
  }

  esp_mqtt_client_stop(client->handle);
  esp_mqtt_client_destroy(client->handle);
  client->handle = nullptr;

  vEventGroupDelete(client->event);

  delete[] client->respTopic;
  delete[] client->cmdTopic;
  delete client;

  return ESP_OK;
}

static void onTimeUpdated(timeval *const tm) {
  if (bootTimestamp == 0) {
    bootTimestamp = time(nullptr);
    xEventGroupSetBits(appState, STATE_TIME_VALID);
  }
  ESP_LOGI(logTag, "sntp time update finished");
}

static void taskSntpUpdate(void *const arg) {
  waitState(STATE_NET_CONNECTED);

  sntp_setoperatingmode(SNTP_OPMODE_POLL);
  sntp_setservername(0, "pool.ntp.org");
  sntp_set_time_sync_notification_cb(onTimeUpdated);
  sntp_init();

  ESP_LOGI(logTag, "sntp time update started");

  vTaskDelete(nullptr);
}

static DS18B20_Info *searchTempSensor(const OneWireBus *const owb) {
  for (bool found = false; !found;) {
    OneWireBus_SearchState search_state;
    const owb_status status = owb_search_first(owb, &search_state, &found);

    if (status != OWB_STATUS_OK) {
      ESP_LOGE(logTag, "owb search failed: %d", status);
      return nullptr;
    }
    if (!found) {
      ESP_LOGD(logTag, "temp sensor not found, retrying");
      vTaskDelay(msToTicks(500));
    }
  }

  OneWireBus_ROMCode rom_code;
  const owb_status status = owb_read_rom(owb, &rom_code);

  if (status != OWB_STATUS_OK) {
    ESP_LOGE(logTag, "could not read ROM code: %d", status);
    return nullptr;
  }

  char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
  owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
  ESP_LOGI(logTag, "found device 0x%s", rom_code_s);

  // Create DS18B20 devices on the 1-Wire bus
  DS18B20_Info *const device = ds18b20_malloc(); // heap allocation
  ds18b20_init_solo(device, owb);                // only one device on bus
  ds18b20_use_crc(device, true); // enable CRC check on all reads
  ds18b20_set_resolution(device, DS18B20_RESOLUTION_12_BIT);

  return device;
}

static OneWireBus *initializeBus(owb_rmt_driver_info *const driver_info,
                                 const TempSensor *const config) {
  OneWireBus *owb = owb_rmt_initialize(driver_info, config->pin, config->txChan,
                                       config->rxChan);

  owb_use_crc(owb, true); // enable CRC check for ROM code

  return owb;
}

static void runTempMeasurements(const DS18B20_Info *const device,
                                const TempSensor *const config) {
  int error_count = 0;
  Measurement ms = {.type = MeasurementType::MS_TEMPERATURE,
                    .sensor = config->name};
  TickType_t lastWakeTime = xTaskGetTickCount();

  while (error_count < 4) {
    const DS18B20_ERROR err = ds18b20_convert_and_read_temp(device, &ms.temp);

    if (err != DS18B20_OK) {
      ++error_count;
      ESP_LOGW(logTag, "measurement failed in %s, err %d", config->name, err);
    } else {
      error_count = 0;
      ms.time = getTimestamp();
      if (!config->queue->putRetrying(ms)) {
        ESP_LOGE(logTag, "could not put temp measurement into queue");
      }
    }

    vTaskDelayUntil(&lastWakeTime, delayTemp);
  }
}

[[noreturn]] void taskCollectTemps(void *const arg) {
  TempSensor *const sensor = reinterpret_cast<TempSensor *>(arg);

  ESP_LOGI(logTag, "starting temp collection task for %s", sensor->name);

  while (true) {
    vTaskDelay(secToTicks(2));

    owb_rmt_driver_info rmt_driver_info;
    initializeBus(&rmt_driver_info, sensor);

    OneWireBus *const owb = &rmt_driver_info.bus;
    DS18B20_Info *device = searchTempSensor(owb);

    if (device) {
      runTempMeasurements(device, sensor);
      ds18b20_free(&device);
    }

    owb_uninitialize(owb);

    ESP_LOGE(logTag, "sensor %s failed, restarting", sensor->name);
  }
}

static void handleIpEvent(void *const arg, const esp_event_base_t event_base,
                          const int32_t event_id, void *const event_data) {
  configASSERT(event_base == IP_EVENT);

  switch (event_id) {
  case IP_EVENT_STA_GOT_IP: {
    xEventGroupClearBits(appState, STATE_NET_DISCONNECTED);
    xEventGroupSetBits(appState, STATE_NET_CONNECTED);

    const ip_event_got_ip_t *const evt =
        reinterpret_cast<ip_event_got_ip_t *>(event_data);
    ESP_LOGI(logTag, "got ip %d.%d.%d.%d", IP2STR(&evt->ip_info.ip));
    break;
  }

  case IP_EVENT_STA_LOST_IP:
    xEventGroupClearBits(appState, STATE_NET_CONNECTED);
    xEventGroupSetBits(appState, STATE_NET_DISCONNECTED);
    ESP_LOGI(logTag, "lost ip");
    break;

  default:
    ESP_LOGI(logTag, "unexpected ip event %d", event_id);
    break;
  }
}

static void handleWifiEvent(void *const arg, const esp_event_base_t event_base,
                            const int32_t event_id, void *const event_data) {
  configASSERT(event_base == WIFI_EVENT);

  switch (event_id) {
  case WIFI_EVENT_STA_START:
    esp_wifi_connect();
    break;

  case WIFI_EVENT_STA_CONNECTED:
    ESP_LOGI(logTag, "connected to AP");
    break;

  case WIFI_EVENT_STA_DISCONNECTED:
    ESP_LOGI(logTag, "disconnected from AP");
    esp_wifi_connect();
    break;

  default:
    ESP_LOGI(logTag, "unexpected sta event %d", event_id);
    break;
  }
}

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/wifi.html#wi-fi-lwip-init-phase
static void initWifi() {
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
      WIFI_EVENT, ESP_EVENT_ANY_ID, handleWifiEvent, nullptr, nullptr));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, handleIpEvent, nullptr, nullptr));

  wifi_scan_threshold_t thres{
      .authmode = WIFI_AUTH_WPA2_PSK,
  };

  // connect to station
  wifi_config_t wf_conf{
      .sta{
          .threshold = thres,
          .pmf_cfg{.capable = true, .required = false},
      },
  };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
  strncpy(reinterpret_cast<char *>(wf_conf.sta.ssid), appSettings.wifi.ssid,
          sizeof(wf_conf.sta.ssid));
  strncpy(reinterpret_cast<char *>(wf_conf.sta.password), appSettings.wifi.pass,
          sizeof(wf_conf.sta.password));
#pragma GCC diagnostic pop

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wf_conf));
  ESP_ERROR_CHECK(esp_wifi_start());
}

static void initLog() {
#ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_DEBUG);
#else
  esp_log_level_set("*", ESP_LOG_INFO);
#endif
}

static void startTempTasks(Queue<Measurement> &ms_queue) {
  const size_t len = sizeof(tempSensors) / sizeof(*tempSensors);
  char name[24];

  for (int i = 0; i < len; ++i) {
    TempSensor *const conf = &tempSensors[i];
    conf->queue = &ms_queue;

    snprintf(name, sizeof(name), "ms_temp_%d", conf->pin);

    xTaskCreate(taskCollectTemps, name, KiB(2), conf, 2, nullptr);
  }
}

static constexpr uint16_t pmsMagic = 0x4d42;

static constexpr PmsCommand initCmd(uint8_t cmd, uint8_t datah, uint8_t datal) {
  return PmsCommand{
      .magic = pmsMagic,
      .command = cmd,
      .data = PP_HTONS((datah << 8) + datal),
      .checksum = PP_HTONS(0x4d + 0x42 + cmd + datal + datah),
  };
}

static constexpr PmsCommand pmsCmdRead = initCmd(0xe2, 0x00, 0x00);

static constexpr PmsCommand pmsCmdModePassive = initCmd(0xe1, 0x00, 0x00);

static constexpr PmsCommand pmsCmdModeActive = initCmd(0xe1, 0x00, 0x01);

static constexpr PmsCommand pmsCmdSleep = initCmd(0xe4, 0x00, 0x00);

static constexpr PmsCommand pmsCmdWakeup = initCmd(0xe4, 0x00, 0x01);

[[noreturn]] static void taskCollectPm(void *const arg) {
  PmsStation &station{*reinterpret_cast<PmsStation *>(arg)};

  PmsResponse res;
  PmMeasurementSum sum;
  Measurement ms{.type = MeasurementType::MS_PARTICULATES,
                 .sensor = station.name};

  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    int sent = station.writeCommand(pmsCmdWakeup);
    if (sent != sizeof(pmsCmdWakeup)) {
      ESP_LOGE(logTag, "could not send wakeup command");
      vTaskDelay(secToTicks(1));
      continue;
    }

    // wait for the station to wake up and send us the first command
    station.flushInput();
    station.readResponse(res, portMAX_DELAY);

    // discard first measurements as recommended by the manual
    vTaskDelay(secToTicks(30));
    station.flushInput();

    // if MQTT or Wi-Fi are down, queue average measurements to conserve RAM
    const bool sendEach = xEventGroupGetBits(appState) & MqttReady;

    if (!sendEach) {
      sum.reset();
    }

    Timer execTime;

    for (int successful = 0; successful < CONFIG_PARTICULATE_MEASUREMENTS;) {
      const int received = station.readResponse(res, secToTicks(5));

      if (received != sizeof(res)) {
        if (received == -1) {
          ESP_LOGE(logTag, "uart receive failed");
        }
        station.flushInput();
        continue;
      }

      if (res.magic != pmsMagic) {
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
        ms.set(res);
        if (!station.queue->putRetrying(ms)) {
          ESP_LOGE(logTag, "could not queue particulate measurement");
        }
      } else {
        sum.addMeasurement(res);
      }

      ESP_LOGI(logTag, "read PM: 1=%dµg, 2.5=%dµg, 10=%dµg", res.pm1McgAtm,
               res.pm2McgAtm, res.pm10McgAtm);

      ++successful;
    }

    ESP_LOGI(logTag, "measurement finished in %u s", execTime.seconds());

    if (!sendEach) {
      ms.time = getTimestamp();
      ms.set(sum);
    }

    sent = station.writeCommand(pmsCmdSleep);
    if (sent != sizeof(pmsCmdSleep)) {
      ESP_LOGE(logTag, "could not send sleep command");
    }

    if (!sendEach) {
      ESP_LOGI(logTag, "avg PM: 1=%u, 2.5=%u, 10=%u", ms.pm.atm.pm1Mcg,
               ms.pm.atm.pm2Mcg, ms.pm.atm.pm10Mcg);

      if (!station.queue->putRetrying(ms)) {
        ESP_LOGE(logTag, "could not queue averaged particulate measurement");
      }
    }

    vTaskDelayUntil(&lastWake, delayPm);
  }
}

static void startPmTasks(Queue<Measurement> &ms_queue) {
  const uart_config_t conf{
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_APB,
  };

  const size_t rxBuf = sizeof(PmsResponse) * 10;

  for (PmsStation &stat : pmsStations) {
    stat.queue = &ms_queue;

    ESP_ERROR_CHECK(uart_driver_install(stat.port, rxBuf, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(stat.port, &conf));
    ESP_ERROR_CHECK(uart_set_pin(stat.port, stat.txPin, stat.rxPin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreate(taskCollectPm, "ms_pm", KiB(2), &stat, 4, nullptr);
  }
}

static void restartPeripheral(const periph_module_t mod) {
  periph_module_disable(mod);
  periph_module_enable(mod);
}

static void initNvs() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
}

static esp_err_t readStringSetting(nvs_handle_t nvs, const char *const name,
                                   const char **const dst) {
  ESP_LOGI(logTag, "reading setting %s from NVS", name);

  size_t len;
  esp_err_t err = nvs_get_str(nvs, name, nullptr, &len);
  if (err != ESP_OK) {
    ESP_LOGI(logTag, "setting %s not found or not available: %x", name, err);
    return err;
  }
  char *const buf = new char[len];
  err = nvs_get_str(nvs, name, buf, &len);
  if (err == ESP_OK) {
    *dst = buf;
    ESP_LOGI(logTag, "setting %s read: [%s]", name, buf);
  } else {
    delete[] buf;
    ESP_LOGE(logTag, "could not read setting %s: %x", name, err);
  }
  return err;
}

static esp_err_t readSettings() {
  appSettings.devName = CONFIG_DEV_NAME;
  appSettings.wifi.ssid = CONFIG_WIFI_SSID;
  appSettings.wifi.pass = CONFIG_WIFI_PASSWORD;
  appSettings.mqtt.broker = CONFIG_MQTT_BROKER_URI;
  appSettings.mqtt.username = CONFIG_MQTT_USERNAME;
  appSettings.mqtt.password = CONFIG_MQTT_PASSWORD;

  nvs_handle_t nvs;

  const esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs);
  if (err != ESP_OK) {
    ESP_LOGE(logTag, "could not open NVS for reading settings: %d", err);
    return err;
  }

  readStringSetting(nvs, "dev/name", &appSettings.devName);
  readStringSetting(nvs, "wifi/ssid", &appSettings.wifi.ssid);
  readStringSetting(nvs, "wifi/pass", &appSettings.wifi.pass);
  readStringSetting(nvs, "mqtt/broker", &appSettings.mqtt.broker);
  readStringSetting(nvs, "mqtt/username", &appSettings.mqtt.username);
  readStringSetting(nvs, "mqtt/password", &appSettings.mqtt.password);

  nvs_close(nvs);

  return ESP_OK;
}

void initSntp() {
  xTaskCreate(taskSntpUpdate, "sntp_update", KiB(2), nullptr, 1, nullptr);
}

extern "C" [[noreturn]] void app_main() {
  appState = xEventGroupCreate();

  // reset peripherals in case of prior crash
  restartPeripheral(PERIPH_RMT_MODULE);
  restartPeripheral(PERIPH_UART0_MODULE);

  initLog();
  initNvs();
  readSettings();
  initWifi();
  initSntp();

  // start pollution collectors right away, we can fix time later
  Queue<Measurement> queue{CONFIG_MEASUREMENT_QUEUE_SIZE};
  startTempTasks(queue);
  startPmTasks(queue);

  waitState(STATE_NET_CONNECTED);

  MqttClient *client =
      mqttClientCreate(appSettings.mqtt.broker, caPemStart,
                       appSettings.mqtt.username, appSettings.mqtt.password);

  waitState(STATE_TIME_VALID);

  for (char buf[256];;) {
    client->waitReady();

    Measurement ms = queue.take();
    ms.fixTime();
    ms.formatMsg(buf, sizeof(buf));

    if (!client->send(ms.getType(), buf)) {
      ESP_LOGE(logTag, "measurement send failed");
    }
  }

  configASSERT(false);
}
