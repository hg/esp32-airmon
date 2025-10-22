#include "state.hh"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

AppState inline operator|(const AppState lhs, const AppState rhs) {
  return static_cast<AppState>(static_cast<EventBits_t>(lhs) |
                               static_cast<EventBits_t>(rhs));
}

void State::wait(const AppState bits) const {
  xEventGroupWaitBits(handle, static_cast<EventBits_t>(bits), false, true,
                      portMAX_DELAY);
}

bool State::check(AppState bits) const {
  return xEventGroupGetBits(handle) & static_cast<EventBits_t>(bits);
}

void State::set(AppState bits) const {
  xEventGroupSetBits(handle, static_cast<EventBits_t>(bits));
}

void State::reset(AppState bits) const {
  xEventGroupClearBits(handle, static_cast<EventBits_t>(bits));
}
