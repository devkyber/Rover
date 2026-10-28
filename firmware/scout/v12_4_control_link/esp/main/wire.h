#ifndef SCOUT_WIRE_H
#define SCOUT_WIRE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { SCOUT_RECORD_SIZE = 40, SCOUT_DATA_OFFSET = 4096 };
enum scout_source { SRC_ICM=1, SRC_ADXL=2, SRC_MAG=3, SRC_HEALTH=4, SRC_PHASE=5, SRC_SUMMARY=6 };
enum scout_flags { FLAG_FIFO_FULL=1, FLAG_TIME_ESTIMATED=2, FLAG_SATURATED=4,
                   FLAG_MAG_UNQUALIFIED=8, FLAG_SENSOR_ERROR=16 };
typedef struct __attribute__((packed)) {
    uint64_t time_us;
    uint32_t sequence;
    uint8_t source, flags;
    uint16_t payload_size;
    uint8_t payload[20];
    uint32_t crc32;
} scout_record_t;
typedef struct __attribute__((packed)) {
    char magic[8];
    uint16_t version, record_size;
    uint32_t icm_hz, adxl_hz, mag_hz, duration_ms;
    uint64_t start_us;
    uint8_t icm_id, adxl_id, mag_id, profile;
    uint8_t reserved[20];
    uint32_t crc32;
} scout_header_t;
_Static_assert(sizeof(scout_record_t)==40, "wire record size");
_Static_assert(sizeof(scout_header_t)==64, "wire header size");
uint32_t scout_crc32(const void *data, size_t size);
void scout_record_init(scout_record_t *r, uint64_t time, uint32_t seq,
                       uint8_t source, uint8_t flags, const void *payload, size_t size);
bool scout_record_valid(const scout_record_t *r);
#endif
