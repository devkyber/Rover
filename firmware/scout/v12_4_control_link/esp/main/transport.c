#include "transport.h"
#include "network.h"
#include "cJSON.h"
#include "protocol.h"
#include "urgent.h"
#include "usb_tx.h"
#include "sdkconfig.h"
#include "driver/usb_serial_jtag.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

typedef struct { uint32_t session; uint16_t size; uint8_t type; char text[SCOUT_JSON_MAX]; } reply_t;
static QueueHandle_t commands,replies,samples;
static portMUX_TYPE urgent_lock=portMUX_INITIALIZER_UNLOCKED;
static scout_urgent_t urgent;
static atomic_uint session,dropped;
#ifdef CONFIG_SCOUT_LINK_WIFI
static atomic_uint data_session;
#endif
static QueueHandle_t files;
static uint32_t sample_sequence,frame_sequence;
static char line[SCOUT_COMMAND_MAX];
static size_t line_used;
static bool line_overflow;
static void connect_session(uint32_t nonce);
static void hello(uint32_t nonce);
#ifndef CONFIG_SCOUT_LINK_WIFI
static uint32_t last_usb_nonce;
static scout_usb_tx_t usb_tx;
_Static_assert(SCOUT_JSON_MAX+16<=SCOUT_USB_TX_MAX,"USB frame storage");
#endif
#ifdef CONFIG_SCOUT_LINK_WIFI
static int client=-1;
#endif
uint32_t scout_transport_session(void) { return atomic_load(&session); }
bool scout_transport_data_ready(void)
{
#ifdef CONFIG_SCOUT_LINK_WIFI
    return scout_transport_session() && atomic_load(&data_session)==scout_transport_session();
#else
    return scout_transport_session()!=0;
#endif
}
uint32_t scout_transport_dropped(void) { return atomic_load(&dropped); }
bool scout_transport_command(scout_command_t *c) { return xQueueReceive(commands,c,0)==pdTRUE; }
bool scout_transport_urgent(uint32_t expected_session,uint32_t last_sequence,scout_command_t *command,uint32_t *barrier)
{
    scout_urgent_item_t item;
    portENTER_CRITICAL(&urgent_lock);
    bool ready=scout_urgent_take(&urgent,expected_session,last_sequence,&item);
    portEXIT_CRITICAL(&urgent_lock);
    if (!ready) return false;
    /* Cancel commands already waiting behind the stop. A stop never needs queue space. */
    xQueueReset(commands);
    command->session=item.session;
    snprintf(command->json,sizeof(command->json),"{\"cmd\":\"%s\",\"session\":%lu,\"seq\":%lu}",
        scout_urgent_name(item.kind),(unsigned long)item.session,(unsigned long)item.sequence);
    *barrier=item.barrier; return true;
}
void scout_transport_json(const char *json)
{
    reply_t r={.session=scout_transport_session()};
    if (!r.session || strlen(json)>=sizeof(r.text)) return;
    memcpy(r.text,json,strlen(json)+1);
    r.type=2; r.size=(uint16_t)strlen(json);
    if (xQueueSend(replies,&r,0)!=pdTRUE) atomic_fetch_add(&dropped,1);
}
bool scout_transport_file_chunk(uint32_t expected_session,uint32_t offset,const void *data,size_t size)
{
    reply_t r={.session=expected_session,.type=3,.size=(uint16_t)(size+4)};
    if (!r.session || r.session!=scout_transport_session() || size>SCOUT_JSON_MAX-4) return false;
    memcpy(r.text,&offset,4);
    if (size) memcpy(r.text+4,data,size);
#ifdef CONFIG_SCOUT_LINK_WIFI
    if (!scout_transport_data_ready()) return false;
    return xQueueSend(files,&r,pdMS_TO_TICKS(200))==pdTRUE && r.session==scout_transport_session();
#else
    return xQueueSend(replies,&r,pdMS_TO_TICKS(200))==pdTRUE && r.session==scout_transport_session();
#endif
}
void scout_transport_sample(uint8_t source,uint8_t flags,uint64_t time,const void *payload,size_t size)
{
    if (source!=SRC_ICM && source!=SRC_ADXL && source!=SRC_MAG) return;
    scout_record_t r;
    scout_record_init(&r,time,sample_sequence++,source,flags,payload,size);
    if (scout_transport_data_ready() && xQueueSend(samples,&r,0)!=pdTRUE) atomic_fetch_add(&dropped,1);
}
static void received(const uint8_t *data,size_t size)
{
    for (size_t i=0;i<size;++i) {
        if (data[i]=='\n' || data[i]=='\r') {
            if (line_used && !line_overflow) {
#ifndef CONFIG_SCOUT_LINK_WIFI
                line[line_used]=0;
                cJSON *request=scout_json_bounded(line,2)?cJSON_ParseWithOpts(line,NULL,true):NULL;
                const cJSON *kind=cJSON_GetObjectItemCaseSensitive(request,"cmd");
                const cJSON *nonce=cJSON_GetObjectItemCaseSensitive(request,"nonce");
                bool handshake=cJSON_IsString(kind) && !strcmp(kind->valuestring,"hello") &&
                    cJSON_IsNumber(nonce) && nonce->valuedouble>=1 && nonce->valuedouble<=UINT32_MAX &&
                    floor(nonce->valuedouble)==nonce->valuedouble;
                uint32_t token=handshake?(uint32_t)nonce->valuedouble:0;
                cJSON_Delete(request);
                if (handshake) {
                    if (token!=last_usb_nonce || !scout_transport_session()) {
                        last_usb_nonce=token; connect_session(token);
                    } else hello(token);
                    line_used=0; line_overflow=false; continue;
                }
                if (!scout_transport_session()) { line_used=0; line_overflow=false; continue; }
#endif
                scout_command_t c={.session=scout_transport_session()};
                memcpy(c.json,line,line_used); c.json[line_used]=0;
                scout_urgent_item_t stop;
                if (scout_urgent_parse(c.json,c.session,&stop)) {
                    portENTER_CRITICAL(&urgent_lock);
                    scout_urgent_offer(&urgent,&stop);
                    portEXIT_CRITICAL(&urgent_lock);
                } else if (xQueueSend(commands,&c,0)!=pdTRUE)
                    scout_transport_json("{\"event\":\"error\",\"error\":\"command_queue_full\"}");
            } else if (line_overflow)
                scout_transport_json("{\"event\":\"error\",\"error\":\"command_too_long\"}");
            line_used=0; line_overflow=false;
        } else if (!data[i]) line_overflow=true;
        else if (line_used<sizeof(line)-1) line[line_used++]=(char)data[i];
        else line_overflow=true;
    }
}
static void hello(uint32_t nonce)
{
    char text[420];
    snprintf(text,sizeof(text),"{\"event\":\"hello\",\"version\":1,\"session\":%lu,\"nonce\":%lu,"
        "\"hardware\":\"ESP32-S3 PCB\",\"api\":\"scout-process-v12\",\"servo_count\":1,\"launch_supported\":true,\"sensors\":[\"ICM-45605\",\"ADXL375\",\"MMC5983MA\"],"
        "\"record_bytes\":40,\"command_timeout_ms\":500}",
        (unsigned long)scout_transport_session(),(unsigned long)nonce);
    scout_transport_json(text);
}
static void connect_session(uint32_t nonce)
{
#ifndef CONFIG_SCOUT_LINK_WIFI
    scout_usb_tx_reset(&usb_tx);
#endif
    portENTER_CRITICAL(&urgent_lock);
    scout_urgent_reset(&urgent);
    portEXIT_CRITICAL(&urgent_lock);
    xQueueReset(samples); xQueueReset(replies); xQueueReset(commands);
    line_used=0; line_overflow=false; frame_sequence=0;
    uint32_t token=esp_random(); if (!token) token=1;
    atomic_store(&session,token); hello(nonce);
}
#ifdef CONFIG_SCOUT_LINK_WIFI
static bool write_all(int fd,const uint8_t *p,size_t n)
{
    uint64_t deadline=(uint64_t)esp_timer_get_time()+100000;
    while (n && (uint64_t)esp_timer_get_time()<deadline) {
        int k=send(fd,p,n,0);
        if (k<0 && errno==EINTR) continue;
        if (k<=0) return false;
        p+=k; n-=(size_t)k;
    }
    return !n;
}
static bool frame(int fd,uint32_t *sequence,uint8_t type,const void *data,size_t size)
{
    uint8_t buf[SCOUT_JSON_MAX+16];
    if (size>SCOUT_JSON_MAX) return false;
    memcpy(buf,"JSF1",4); buf[4]=(uint8_t)size; buf[5]=(uint8_t)(size>>8);
    buf[6]=type; buf[7]=1;
    uint32_t seq=(*sequence)++; memcpy(buf+8,&seq,4);
    memcpy(buf+12,data,size);
    uint32_t crc=scout_crc32(buf,12+size); memcpy(buf+12+size,&crc,4);
    return write_all(fd,buf,size+16);
}
#else
static bool usb_stage(uint8_t type,const void *data,size_t size,unsigned sample_count)
{
    uint8_t buf[SCOUT_JSON_MAX+16];
    if (size>SCOUT_JSON_MAX || scout_usb_tx_pending(&usb_tx)) return false;
    memcpy(buf,"JSF1",4); buf[4]=(uint8_t)size; buf[5]=(uint8_t)(size>>8);
    buf[6]=type; buf[7]=1;
    memcpy(buf+8,&frame_sequence,4);
    memcpy(buf+12,data,size);
    uint32_t crc=scout_crc32(buf,12+size); memcpy(buf+12+size,&crc,4);
    if (!scout_usb_tx_stage(&usb_tx,buf,size+16,sample_count)) return false;
    ++frame_sequence; /* A retained frame is retried verbatim, never renumbered. */
    return true;
}
static int usb_write(const uint8_t *data,size_t size,void *context)
{
    (void)context;
    /* ESP-IDF returns all bytes or zero when its TX ring is full. Zero is not a
       disconnect. Never block RX/urgent commands while waiting for the reader. */
    return usb_serial_jtag_write_bytes(data,size,0);
}
#endif
static bool transmit(void)
{
#ifdef CONFIG_SCOUT_LINK_WIFI
    int fd=client;
    reply_t r;
    for (unsigned i=0;i<4 && xQueueReceive(replies,&r,0)==pdTRUE;++i)
        if (r.session==scout_transport_session() && !frame(fd,&frame_sequence,r.type,r.text,r.size)) return false;
    return true;
#else
    if (uxQueueMessagesWaiting(replies)) {
        unsigned skipped=scout_usb_tx_discard_samples(&usb_tx);
        if (skipped) atomic_fetch_add(&dropped,skipped);
    }
    if (!scout_usb_tx_pending(&usb_tx)) {
        reply_t r;
        for (unsigned i=0;i<4 && xQueueReceive(replies,&r,0)==pdTRUE;++i) {
            if (r.session!=scout_transport_session()) continue;
            if (!usb_stage(r.type,r.text,r.size,0)) return false;
            break; /* Keep control replies in FIFO order, including a blocked one. */
        }
        if (!scout_usb_tx_pending(&usb_tx) && !uxQueueMessagesWaiting(replies)) {
            scout_record_t batch[32]; unsigned n=0;
            while (n<32 && xQueueReceive(samples,&batch[n],0)==pdTRUE) ++n;
            if (n && !usb_stage(1,batch,n*sizeof(*batch),n)) return false;
        }
    }
    /* One nonblocking attempt per I/O loop. BUSY keeps the session, pending
       bytes and control order intact, then yields directly back to RX. */
    return scout_usb_tx_step(&usb_tx,usb_write,NULL)!=SCOUT_USB_TX_ERROR;
#endif
}
#ifdef CONFIG_SCOUT_LINK_WIFI
static void bulk_task(void *arg)
{
    (void)arg;
    int listener=socket(AF_INET,SOCK_STREAM,IPPROTO_IP),one=1;
    if (listener<0) { vTaskDelete(NULL); return; }
    setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in address={.sin_family=AF_INET,.sin_port=htons(9001),.sin_addr.s_addr=htonl(INADDR_ANY)};
    if (bind(listener,(struct sockaddr *)&address,sizeof(address)) || listen(listener,1)) {
        close(listener); vTaskDelete(NULL); return;
    }
    for (;;) {
        int fd=accept(listener,NULL,NULL);
        if (fd<0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        struct timeval timeout={.tv_usec=100000};
        setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        timeout.tv_usec=20000; setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
        char auth[80]={0}; unsigned used=0; bool terminated=false;
        uint64_t deadline=(uint64_t)esp_timer_get_time()+500000;
        while (used<sizeof(auth)-1 && (uint64_t)esp_timer_get_time()<deadline) {
            char ch; int n=recv(fd,&ch,1,0);
            if (n<=0) break;
            if (ch=='\n') { terminated=true; break; }
            auth[used++]=ch;
        }
        cJSON *root=terminated && scout_json_bounded(auth,2)?cJSON_ParseWithOpts(auth,NULL,true):NULL;
        const cJSON *value=cJSON_GetObjectItemCaseSensitive(root,"session");
        uint32_t token=scout_transport_session();
        bool valid=token && cJSON_IsNumber(value) && value->valuedouble==(double)token;
        cJSON_Delete(root);
        if (!valid) { close(fd); continue; }
        xQueueReset(samples); xQueueReset(files); atomic_store(&data_session,token);
        uint32_t sequence=0;
        bool okay=true;
        while (okay && scout_transport_session()==token) {
            reply_t file;
            if (xQueueReceive(files,&file,0)==pdTRUE && file.session==token)
                okay=frame(fd,&sequence,file.type,file.text,file.size);
            scout_record_t batch[32]; unsigned count=0;
            while (count<32 && xQueueReceive(samples,&batch[count],0)==pdTRUE) ++count;
            if (okay && count) okay=frame(fd,&sequence,1,batch,count*sizeof(*batch));
            char ch; int n=recv(fd,&ch,1,MSG_DONTWAIT);
            if (!n || (n<0 && errno!=EWOULDBLOCK && errno!=EAGAIN && errno!=EINTR)) break;
            if (n>0) break; /* data channel is receive-only after its session binding */
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        atomic_store(&data_session,0); close(fd);
    }
}
#endif
static void io_task(void *arg)
{
    (void)arg;
#ifdef CONFIG_SCOUT_LINK_WIFI
    int listener=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
    if (listener<0) { vTaskDelete(NULL); return; }
    int one=1; setsockopt(listener,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
    struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(9000),.sin_addr.s_addr=htonl(INADDR_ANY)};
    if (bind(listener,(struct sockaddr *)&addr,sizeof(addr)) || listen(listener,1)) {
        close(listener); vTaskDelete(NULL); return;
    }
#else
    usb_serial_jtag_driver_config_t config={.tx_buffer_size=4096,.rx_buffer_size=4096};
    if (usb_serial_jtag_driver_install(&config)!=ESP_OK) { vTaskDelete(NULL); return; }
#endif
    for (;;) {
#ifdef CONFIG_SCOUT_LINK_WIFI
        client=accept(listener,NULL,NULL);
        if (client<0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        struct timeval timeout={.tv_usec=20000};
        setsockopt(client,SOL_SOCKET,SO_SNDTIMEO,&timeout,sizeof(timeout));
        setsockopt(client,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
#else
        /* A connected USB bus does not imply an open serial reader; heartbeat still gates motion. */
        while (!usb_serial_jtag_is_connected()) vTaskDelay(pdMS_TO_TICKS(20));
#endif
#ifdef CONFIG_SCOUT_LINK_WIFI
        connect_session(0);
#else
        scout_usb_tx_reset(&usb_tx);
        last_usb_nonce=0; atomic_store(&session,0);
#endif
        bool okay=true;
        while (okay) {
            uint8_t bytes[256];
#ifdef CONFIG_SCOUT_LINK_WIFI
            int n=recv(client,bytes,sizeof(bytes),MSG_DONTWAIT);
            if (!n || (n<0 && errno!=EWOULDBLOCK && errno!=EAGAIN && errno!=EINTR)) break;
#else
            if (!usb_serial_jtag_is_connected()) break;
            int n=usb_serial_jtag_read_bytes(bytes,sizeof(bytes),0);
#endif
            if (n>0) received(bytes,(size_t)n);
            okay=!scout_transport_session() || transmit(); vTaskDelay(pdMS_TO_TICKS(1));
        }
        atomic_store(&session,0);
#ifdef CONFIG_SCOUT_LINK_WIFI
        close(client); client=-1;
#else
        scout_usb_tx_reset(&usb_tx);
#endif
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
esp_err_t scout_transport_init(void)
{
    commands=xQueueCreate(8,sizeof(scout_command_t));
    files=xQueueCreate(8,sizeof(reply_t));
    replies=xQueueCreate(12,sizeof(reply_t)); samples=xQueueCreate(1024,sizeof(scout_record_t));
    if (!commands || !replies || !samples || !files) return ESP_ERR_NO_MEM;
#ifdef CONFIG_SCOUT_LINK_WIFI
    esp_err_t rc=scout_network_init(); if (rc!=ESP_OK) return rc;
    if (xTaskCreatePinnedToCore(bulk_task,"imu_link",8192,NULL,4,NULL,0)!=pdPASS) return ESP_ERR_NO_MEM;
#endif
    return xTaskCreatePinnedToCore(io_task,"jetson_link",8192,NULL,5,NULL,0)==pdPASS?ESP_OK:ESP_ERR_NO_MEM;
}
