#ifndef SCOUT_ACTUATORS_H
#define SCOUT_ACTUATORS_H
#include "esp_err.h"
#include "control.h"
esp_err_t scout_actuators_init(void);
esp_err_t scout_actuators_apply(const scout_outputs_t *out, uint64_t now);
void scout_actuators_park(void);
bool scout_actuators_fault(void);
#endif
