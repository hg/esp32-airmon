#pragma once

#include <esp_bit_defs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

typedef enum : EventBits_t {
  APP_TIME_VALID = BIT0,
  APP_ONLINE = BIT1,
} state_bit;

typedef struct {
  EventGroupHandle_t handle;
} app_state;

void state_init(app_state *st);
void state_deinit(app_state *st);

void state_wait(app_state *st, state_bit bits);
bool state_check(app_state *st, state_bit bits);
void state_set(app_state *st, state_bit bits);
void state_reset(app_state *st, state_bit bits);
