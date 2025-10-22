#pragma once

#include <esp_bit_defs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

enum class AppState : EventBits_t {
  STATE_TIME_VALID = BIT0,
  STATE_NET_CONNECTED = BIT1,
  STATE_NET_DISCONNECTED = BIT2,
};

class State {
public:
  State() : handle{xEventGroupCreate()} {}

  State(const State &) = delete;

  ~State() { vEventGroupDelete(handle); }

  State &operator=(const State &) = delete;

  void wait(AppState bits) const;
  [[nodiscard]] bool check(AppState bits) const;
  void set(AppState bits) const;
  void reset(AppState bits) const;

private:
  const EventGroupHandle_t handle;
};

extern State *appState;
