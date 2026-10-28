#include "capture.h"
#include "esp_partition.h"
#include "esp_timer.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static const esp_partition_t *partition;
static scout_capture_status_t state;
static scout_record_t pending[256];
static size_t pending_count, offset;
static uint32_t sequence;
/* All APIs are owned by the acquisition task: no concurrent flash writes. */
esp_err_t scout_capture_init(void)
{
    partition=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,0x40,"scout_log");
    return partition?ESP_OK:ESP_ERR_NOT_FOUND;
}
esp_err_t scout_capture_prepare(void)
{
    if (!partition || state.recording) return ESP_ERR_INVALID_STATE;
    state.prepared=false;
    esp_err_t rc=esp_partition_erase_range(partition,0,partition->size);
    if (rc==ESP_OK) { memset(&state,0,sizeof(state)); state.prepared=true; }
    return rc;
}
esp_err_t scout_capture_start(const scout_header_t *h)
{
    if (!partition || !state.prepared || state.recording) return ESP_ERR_INVALID_STATE;
    esp_err_t rc=esp_partition_write(partition,0,h,sizeof(*h));
    if (rc!=ESP_OK) return rc;
    state=(scout_capture_status_t){.recording=true,.start_us=h->start_us,
        .stop_us=h->start_us+(uint64_t)h->duration_ms*1000};
    sequence=0; pending_count=0; offset=SCOUT_DATA_OFFSET;
    return ESP_OK;
}
void scout_capture_sample(uint8_t source,uint8_t flags,uint64_t time,
                           const void *payload,size_t size)
{
    if (!state.recording) return;
    uint32_t seq=sequence++;
    if (pending_count>=256 || offset+(pending_count+1)*sizeof(scout_record_t)>partition->size) {
        ++state.dropped; return;
    }
    scout_record_init(&pending[pending_count++],time,seq,source,flags,payload,size);
}
esp_err_t scout_capture_flush(void)
{
    if (!state.recording) return ESP_ERR_INVALID_STATE;
    if (!pending_count) return ESP_OK;
    size_t bytes=pending_count*sizeof(scout_record_t);
    esp_err_t rc=esp_partition_write(partition,offset,pending,bytes);
    if (rc!=ESP_OK) {
        ++state.write_errors; state.dropped+=(uint32_t)pending_count;
        pending_count=0; state.recording=false; return rc;
    }
    offset+=bytes; state.records+=(uint32_t)pending_count; pending_count=0;
    return ESP_OK;
}
esp_err_t scout_capture_stop(void)
{
    if (!state.recording) return ESP_ERR_INVALID_STATE;
    esp_err_t rc=scout_capture_flush();
    state.recording=false;
    /* A separate, one-time footer. A reset before this leaves recoverable records. */
    uint32_t footer[8]={0x454e4431,state.records,state.dropped,state.write_errors,
        (uint32_t)offset,(uint32_t)state.start_us,(uint32_t)(state.start_us>>32),0};
    footer[7]=scout_crc32(footer,28);
    esp_err_t end=esp_partition_write(partition,128,footer,sizeof(footer));
    return rc==ESP_OK?end:rc;
}
scout_capture_status_t scout_capture_status(void) { return state; }
esp_err_t scout_capture_export(bool (*chunk)(uint32_t,const void *,size_t))
{
    if (!partition || state.recording || !chunk) return ESP_ERR_INVALID_STATE;
    scout_header_t h;
    esp_err_t rc=esp_partition_read(partition,0,&h,sizeof(h));
    if (rc!=ESP_OK) return rc;
    if (memcmp(h.magic,"SCOUT001",8) || h.version!=1 || h.record_size!=40 ||
        h.crc32!=scout_crc32(&h,offsetof(scout_header_t,crc32))) return ESP_ERR_INVALID_STATE;
    size_t end=SCOUT_DATA_OFFSET;
    scout_record_t r;
    while (end+sizeof(r)<=partition->size) {
        if ((rc=esp_partition_read(partition,end,&r,sizeof(r)))!=ESP_OK) return rc;
        if (!scout_record_valid(&r)) break;
        end+=sizeof(r);
    }
    uint8_t data[1024];
    for (size_t at=0;at<end;at+=sizeof(data)) {
        size_t n=end-at>sizeof(data)?sizeof(data):end-at;
        if ((rc=esp_partition_read(partition,at,data,n))!=ESP_OK) return rc;
        if (!chunk((uint32_t)at,data,n)) return ESP_ERR_TIMEOUT;
    }
    return chunk((uint32_t)end,NULL,0)?ESP_OK:ESP_ERR_TIMEOUT;
}
esp_err_t scout_capture_service(void)
{
    return pending_count*sizeof(scout_record_t)>=4096 ? scout_capture_flush() : ESP_OK;
}
esp_err_t scout_capture_dump(void)
{
    if (!partition || state.recording) return ESP_ERR_INVALID_STATE;
    scout_header_t h;
    esp_err_t rc=esp_partition_read(partition,0,&h,sizeof(h));
    if (rc!=ESP_OK) return rc;
    if (memcmp(h.magic,"SCOUT001",8) || h.record_size!=40 || h.version!=1 ||
        h.crc32!=scout_crc32(&h,offsetof(scout_header_t,crc32))) return ESP_ERR_INVALID_STATE;
    /* Recover even after sudden power loss: find the valid CRC prefix. */
    size_t end=SCOUT_DATA_OFFSET; scout_record_t record;
    while (end+sizeof(record)<=partition->size) {
        rc=esp_partition_read(partition,end,&record,sizeof(record));
        if (rc!=ESP_OK) return rc;
        if (!scout_record_valid(&record)) break;
        end+=sizeof(record);
    }
    printf("BEGIN_SCOUT %u\n",(unsigned)end);
    uint8_t bytes[128];
    for (size_t at=0;at<end;at+=sizeof(bytes)) {
        size_t n=end-at>sizeof(bytes)?sizeof(bytes):end-at;
        rc=esp_partition_read(partition,at,bytes,n);
        if (rc!=ESP_OK) return rc;
        printf("D %08x ",(unsigned)at);
        for (size_t i=0;i<n;++i) printf("%02x",bytes[i]);
        putchar('\n');
    }
    printf("END_SCOUT\n"); fflush(stdout);
    return ESP_OK;
}
