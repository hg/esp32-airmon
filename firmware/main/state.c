#include "state.h"
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

void state_init(app_state *st) {
  st->handle = xEventGroupCreate();
}

void state_deinit(app_state *st) {
  vEventGroupDelete(st->handle);
  st->handle = NULL;
}

void state_wait(app_state *st, state_bit bits) {
  xEventGroupWaitBits(st->handle, bits, false, true, portMAX_DELAY);
}

bool state_check(app_state *st, state_bit bits) {
  return xEventGroupGetBits(st->handle) & bits;
}

void state_set(app_state *st, state_bit bits) {
  xEventGroupSetBits(st->handle, bits);
}

void state_reset(app_state *st, state_bit bits) {
  xEventGroupClearBits(st->handle, bits);
}
