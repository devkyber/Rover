#ifndef SCOUT_NETWORK_H
#define SCOUT_NETWORK_H
#include "esp_err.h"
#include <stdbool.h>
esp_err_t scout_network_init(void);
bool scout_network_maintenance(void);
bool scout_motion_busy(void);
#endif
