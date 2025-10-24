#pragma once

#include "state.h"
#include "time.h"

// If time has been initialized, returns it as a UNIX timestamp.
// Otherwise, returns time in seconds since last boot.
time_t get_timestamp();
time_t fix_time(time_t tm);
void sntp_start(app_state *state);
