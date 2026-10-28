#ifndef SCOUT_URGENT_H
#define SCOUT_URGENT_H
#include <stdbool.h>
#include <stdint.h>
/* One slot per severity: an old estop cannot hide a newer valid soft stop.
 * Caller serializes offer/take/reset with the transport's critical section. */
typedef enum { URGENT_NONE=0, URGENT_COMPRESSION_STOP=1, URGENT_STOP=2, URGENT_ESTOP=3 } scout_urgent_kind_t;
typedef struct { uint32_t session,sequence,barrier; scout_urgent_kind_t kind; } scout_urgent_item_t;
typedef struct { scout_urgent_item_t slots[3]; } scout_urgent_t;
bool scout_urgent_parse(const char *json,uint32_t session,scout_urgent_item_t *item);
void scout_urgent_reset(scout_urgent_t *inbox);
void scout_urgent_offer(scout_urgent_t *inbox,const scout_urgent_item_t *item);
bool scout_urgent_take(scout_urgent_t *inbox,uint32_t session,uint32_t last_sequence,scout_urgent_item_t *item);
const char *scout_urgent_name(scout_urgent_kind_t kind);
#endif
