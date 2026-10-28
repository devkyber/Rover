#ifndef SCOUT_PROTOCOL_H
#define SCOUT_PROTOCOL_H
#include <stdbool.h>
/* Bound recursion before invoking cJSON on externally supplied commands. */
bool scout_json_bounded(const char *text,unsigned maximum_depth);
bool scout_command_known(const char *name);
#endif
