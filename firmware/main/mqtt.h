#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <mqtt_client.h>

typedef enum : EventBits_t {
  MQTT_READY = BIT0,
} mqtt_state;

typedef struct {
  esp_mqtt_client_handle_t handle;
  EventGroupHandle_t event;
} mqtt_client;

void mqtt_init(mqtt_client *mq, const char *uri, const char *hint,
               const char *psk);
void mqtt_deinit(mqtt_client *mq);
void mqtt_wait(mqtt_client *mq, mqtt_state st);
bool mqtt_send(mqtt_client *mq, const char *topic, const char *data);
