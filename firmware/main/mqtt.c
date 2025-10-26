#include "mqtt.h"
#include "common.h"
#include "esp_tls.h"
#include "utils.h"
#include <esp_log.h>

static const char *TAG = "air/mqtt";

static void mqtt_set_state(mqtt_client *mq, mqtt_state bits) {
  xEventGroupSetBits(mq->event, bits);
}

static void mqtt_clear_state(mqtt_client *mq, mqtt_state bits) {
  xEventGroupClearBits(mq->event, bits);
}

static void handle_event(void *event_handler_arg, esp_event_base_t event_base,
                         int32_t event_id, void *event_data) {
  mqtt_client *mq = event_handler_arg;
  esp_mqtt_event_t *evt = event_data;

  switch (event_id) {
  case MQTT_EVENT_CONNECTED:
    mqtt_set_state(mq, MQTT_READY);
    ESP_LOGI(TAG, "mqtt connected");
    break;

  case MQTT_EVENT_DISCONNECTED:
    mqtt_clear_state(mq, MQTT_READY);
    ESP_LOGI(TAG, "mqtt disconnected");
    break;

  case MQTT_EVENT_PUBLISHED:
    ESP_LOGD(TAG, "mqtt broker received message %d", evt->msg_id);
    break;

  case MQTT_EVENT_DATA: {
    ESP_LOGI(TAG, "mqtt message ignored (id %d)", evt->msg_id);
    break;
  }

  case MQTT_EVENT_SUBSCRIBED:
    ESP_LOGI(TAG, "mqtt subscription successful (%d)", evt->msg_id);
    break;

  case MQTT_EVENT_ERROR:
    ESP_LOGE(TAG, "mqtt error %d", evt->error_handle->error_type);
    break;

  default:
    break;
  }
}

// convert hex string to array of bytes and return how many bytes were stored
static size_t hex_to_bytes(const char *str, uint8_t *buf) {
  const size_t hex_len = strlen(str);
  configASSERT(hex_len % 2 == 0);

  const size_t psk_len = hex_len / 2;

  for (size_t i = 0; i < psk_len; ++i) {
    char b[3] = {0, 0, 0};
    b[0] = str[i * 2];
    b[1] = str[i * 2 + 1];
    buf[i] = strtol(b, NULL, 16);
  }

  return psk_len;
}

void mqtt_init(mqtt_client *mq, const char *uri, const char *hint,
               const char *psk) {
  mq->event = xEventGroupCreate();

  uint8_t *buf = malloc(strlen(psk) / 2);
  size_t psk_len = hex_to_bytes(psk, buf);

  psk_hint_key_t *key = malloc(sizeof(psk_hint_key_t));
  key->key = buf;
  key->key_size = psk_len;
  key->hint = hint;

  const esp_mqtt_client_config_t conf = {
      .broker =
          {
              .address = {.uri = uri},
              .verification = {.psk_hint_key = key},
          },
  };
  mq->handle = esp_mqtt_client_init(&conf);
  configASSERT(mq->handle);

  esp_mqtt_client_register_event(mq->handle, MQTT_EVENT_ANY, handle_event, mq);

  ESP_ERROR_CHECK(esp_mqtt_client_start(mq->handle));
}


void mqtt_deinit(mqtt_client *mq) {
  esp_mqtt_client_stop(mq->handle);
  esp_mqtt_client_destroy(mq->handle);
  mq->handle = NULL;

  vEventGroupDelete(mq->event);
  mq->event = NULL;
}

void mqtt_wait(mqtt_client *mq, mqtt_state st) {
  xEventGroupWaitBits(mq->event, st, false, false, portMAX_DELAY);
}

bool mqtt_send(mqtt_client *mq, const char *topic, const char *data) {
  for (int i = 0; i < 10; ++i) {
    int msg_id =
        esp_mqtt_client_publish(mq->handle, topic, data, strlen(data), 1, 1);

    if (msg_id < 0) {
      ESP_LOGI(TAG, "mqtt publish failed, retrying");
      vTaskDelay(seconds(1));
    } else {
      return true;
    }
  }

  return false;
}
