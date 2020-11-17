#pragma once

#include "esp_bit_defs.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

// tag for application logs
const char *const logTag = CONFIG_DEV_NAME;

#define LOG_ERR(err, fmt, ...)                                                 \
  ESP_LOGE(logTag, "[0x%x]: " fmt, err, ##__VA_ARGS__)
