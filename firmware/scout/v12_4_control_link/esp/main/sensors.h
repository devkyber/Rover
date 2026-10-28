#ifndef SCOUT_SENSORS_H
#define SCOUT_SENSORS_H
#include "esp_err.h"
#include "wire.h"
typedef struct {
    uint8_t icm_id, adxl_id, mag_id;
    bool icm_ok, adxl_ok, mag_ok;
    uint32_t icm_hz, adxl_hz, icm_frames, adxl_frames, mag_frames;
    uint32_t icm_full, adxl_full, io_errors;
    uint64_t last_icm_us, last_adxl_us, last_mag_us;
    int encoder_count, battery_mv, current_adc_mv;
    bool driver_fault, encoder_ready;
    int encoder_a, encoder_b;
    uint32_t encoder_edges_a, encoder_edges_b;
} scout_sensors_t;
typedef void (*scout_sample_cb)(uint8_t source, uint8_t flags, uint64_t time,
                               const void *data, size_t size);
esp_err_t scout_sensors_init(scout_sensors_t *s);
esp_err_t scout_sensors_profile(scout_sensors_t *s, bool shock);
void scout_sensors_poll(scout_sensors_t *s, scout_sample_cb cb);
void scout_motor_park(void);
esp_err_t scout_encoder_read(int *value);
#endif
