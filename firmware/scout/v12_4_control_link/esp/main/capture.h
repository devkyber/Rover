#ifndef SCOUT_CAPTURE_H
#define SCOUT_CAPTURE_H
#include "esp_err.h"
#include "wire.h"
typedef struct {
    bool prepared, recording;
    uint32_t records, dropped, write_errors;
    uint64_t start_us, stop_us;
} scout_capture_status_t;
esp_err_t scout_capture_init(void);
esp_err_t scout_capture_prepare(void);
esp_err_t scout_capture_start(const scout_header_t *h);
void scout_capture_sample(uint8_t source, uint8_t flags, uint64_t time,
                          const void *payload, size_t size);
esp_err_t scout_capture_flush(void);
esp_err_t scout_capture_service(void);
esp_err_t scout_capture_stop(void);
esp_err_t scout_capture_dump(void);
esp_err_t scout_capture_export(bool (*chunk)(uint32_t offset,const void *data,size_t size));
scout_capture_status_t scout_capture_status(void);
#endif
