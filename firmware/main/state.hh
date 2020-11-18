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
  State() { handle = xEventGroupCreate(); }

  State(const State &) = delete;

  ~State() { vEventGroupDelete(handle); }

  State &operator=(const State &) = delete;

  void wait(AppState bits);

  [[nodiscard]] bool check(AppState bits);

  void set(AppState bits);

  void reset(AppState bits);

private:
  EventGroupHandle_t handle;
};

extern State *appState;
