#include "wire.h"
#include <string.h>
uint32_t scout_crc32(const void *data, size_t size)
{
    const uint8_t *p=data; uint32_t crc=0xffffffffu;
    for (size_t n=0; n<size; ++n) {
        crc ^= p[n];
        for (unsigned b=0; b<8; ++b) crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
void scout_record_init(scout_record_t *r, uint64_t time, uint32_t seq,
                       uint8_t source, uint8_t flags, const void *payload, size_t size)
{
    memset(r,0,sizeof(*r)); r->time_us=time; r->sequence=seq;
    r->source=source; r->flags=flags;
    if (size>sizeof(r->payload)) size=sizeof(r->payload);
    r->payload_size=(uint16_t)size;
    if (size) memcpy(r->payload,payload,size);
    r->crc32=scout_crc32(r,offsetof(scout_record_t,crc32));
}
bool scout_record_valid(const scout_record_t *r)
{
    return r->source>=SRC_ICM && r->source<=SRC_SUMMARY && r->payload_size<=20 &&
        r->crc32==scout_crc32(r,offsetof(scout_record_t,crc32));
}
