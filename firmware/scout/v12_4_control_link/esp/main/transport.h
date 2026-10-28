#ifndef SCOUT_TRANSPORT_H
#define SCOUT_TRANSPORT_H
#include "esp_err.h"
#include "wire.h"
#define SCOUT_COMMAND_MAX 2048
#define SCOUT_JSON_MAX 1536
typedef struct { uint32_t session; char json[SCOUT_COMMAND_MAX]; } scout_command_t;
esp_err_t scout_transport_init(void);
bool scout_transport_command(scout_command_t *command);
bool scout_transport_urgent(uint32_t session,uint32_t last_sequence,scout_command_t *command,uint32_t *barrier);
uint32_t scout_transport_session(void);
bool scout_transport_data_ready(void);
void scout_transport_json(const char *json);
bool scout_transport_file_chunk(uint32_t expected_session,uint32_t offset,const void *data,size_t size);
void scout_transport_sample(uint8_t source,uint8_t flags,uint64_t time,const void *payload,size_t size);
uint32_t scout_transport_dropped(void);
#endif
