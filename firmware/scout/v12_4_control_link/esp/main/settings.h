#ifndef SCOUT_SETTINGS_H
#define SCOUT_SETTINGS_H
#include "control.h"
#include "cJSON.h"
bool scout_mechanism_from_json(const cJSON *object,scout_mechanism_t *mechanism);
#endif
