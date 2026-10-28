// Clegg Wi-Fi acquisition only. No wheel, lift, or servo control.
// ADXL375 Rev B: mode 3, >=2 MHz for 3200 Hz, six-byte FIFO reads, >=5 us gap.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/stdio/driver.h"
#include "pico/multicore.h"
#include "pico/rand.h"
#include "pico/unique_id.h"
#include "pico/bootrom.h"
#include "hardware/spi.h"
#include "clegg_config.h"
#include "wifi_secrets.h"
#include "nina.h"

enum { FIFO_LOSS=1, RING_LOSS=2, RAW_LIMIT=4, TIME_REVERSED=8 };
typedef struct __attribute__((packed)) {
    uint64_t seq, estimated_us, fifo_poll_us;
    int16_t x, y, z;
    uint16_t flags, fifo_depth;
} Sample;
_Static_assert(sizeof(Sample)==34,"clegg_v1 sample layout changed");
_Static_assert(CLEGG_NETWORK_PACKET_SAMPLES>=1&&CLEGG_NETWORK_PACKET_SAMPLES<=(2048-10)/sizeof(Sample),"clegg_v1 packet count bound");
_Static_assert(12+10+CLEGG_NETWORK_PACKET_SAMPLES*sizeof(Sample)<=CLEGG_NINA_MAX_BLOCK,"NINA TCP write block bound");
static Sample ring[CLEGG_RING_SAMPLES];
static volatile uint32_t head, tail, fifo_losses, ring_drops;
static volatile bool requested, active, sensor_ok;
static volatile uint32_t stop_request, stop_ack;
static uint32_t stop_failures;
static uint64_t boot_id;
static uint8_t device_id;
static int client_socket=-1,server_socket=-1,announce_socket=-1;
static uint32_t network_failures=0,packets=0;
static char nina_version[32];
static uint8_t initial_regs[7];
static const uint8_t reg_ids[7]={0x00,0x2c,0x2d,0x2e,0x31,0x38,0x39};

static void read_bytes(uint8_t reg, uint8_t *out, size_t n) {
    uint8_t cmd=0x80|reg|(n>1?0x40:0);
    gpio_put(CLEGG_SPI_CS_GPIO,0);
    spi_write_blocking(spi0,&cmd,1);
    spi_read_blocking(spi0,0,out,n);
    gpio_put(CLEGG_SPI_CS_GPIO,1);
}
static uint8_t rd(uint8_t reg) { uint8_t v;read_bytes(reg,&v,1);return v; }
static void wr(uint8_t reg,uint8_t value) {
    uint8_t data[2]={reg,value};
    gpio_put(CLEGG_SPI_CS_GPIO,0);spi_write_blocking(spi0,data,2);gpio_put(CLEGG_SPI_CS_GPIO,1);
}
static void standby_clear_old_data(void) {
    // A RP2040 reset does not reset the sensor. Discard pre-arm FIFO contents
    // in bypass, then read all data registers to clear their old overrun latch.
    // INT_SOURCE alone does not clear DATA_READY/watermark/overrun (Rev B p23).
    wr(0x2d,0);wr(0x38,0);
    uint8_t old_data[6];read_bytes(0x32,old_data,sizeof old_data);
    sleep_us(CLEGG_FIFO_READ_GAP_US);(void)rd(0x30);
}
static void acquire(void) {
    uint64_t seq=0,previous_us=0;
    uint16_t pending_loss=0;
    for (;;) {
        if (!requested) {
            if (active) wr(0x2d,0);
            /* Publish the acknowledgement after the last FIFO/ring access.
             * A new START must not hide a brief STOP from this core. */
            previous_us=0;pending_loss=0;
            uint32_t stopped=stop_request;
            __dmb();active=false;__dmb();stop_ack=stopped;
            sleep_us(CLEGG_FIFO_POLL_MS*1000);continue;
        }
        if (!active) {
            device_id=rd(0);
            if (device_id!=0xe5) {sensor_ok=false;sleep_ms(100);continue;}
            standby_clear_old_data();wr(0x2c,CLEGG_RATE_REGISTER);
            wr(0x31,0x0b);wr(0x2e,0x01);wr(0x38,0x90);wr(0x2d,0x08);
            sensor_ok=rd(0x2c)==CLEGG_RATE_REGISTER && rd(0x31)==0x0b && rd(0x38)==0x90 && rd(0x2d)==0x08;
            if (!sensor_ok) {wr(0x2d,0);requested=false;continue;}
            active=true;previous_us=0;
        }
        uint8_t interrupt=rd(0x30);
        uint64_t poll=time_us_64();
        unsigned count=rd(0x39)&0x3f;
        if ((interrupt&1)||count>=32) {fifo_losses++;pending_loss|=FIFO_LOSS;}
        if (count>32) {sensor_ok=false;requested=false;continue;}
        for (unsigned i=0;i<count;i++) {
            uint8_t raw[6];read_bytes(0x32,raw,6);sleep_us(CLEGG_FIFO_READ_GAP_US);
            Sample s={0};s.seq=seq++;s.fifo_poll_us=poll;s.fifo_depth=count;
            s.estimated_us=poll-((uint64_t)(count-1-i)*1000000/CLEGG_SAMPLE_HZ);
            s.x=(int16_t)(raw[0]|raw[1]<<8);s.y=(int16_t)(raw[2]|raw[3]<<8);s.z=(int16_t)(raw[4]|raw[5]<<8);
            s.flags=pending_loss;
            if (abs(s.x)>=CLEGG_RAW_LIMIT_COUNTS||abs(s.y)>=CLEGG_RAW_LIMIT_COUNTS||abs(s.z)>=CLEGG_RAW_LIMIT_COUNTS) s.flags|=RAW_LIMIT;
            if (previous_us && s.estimated_us<=previous_us) s.flags|=TIME_REVERSED;
            previous_us=s.estimated_us;
            uint32_t next=(head+1)%CLEGG_RING_SAMPLES;
            if (next==tail) {ring_drops++;pending_loss|=RING_LOSS;continue;}
            ring[head]=s;__dmb();head=next;pending_loss=0;
        }
        sleep_us(CLEGG_FIFO_POLL_MS*1000);
    }
}
static bool quiesce_acquisition(const char *reason) {
    uint32_t ticket=stop_request+1;
    requested=false;__dmb();stop_request=ticket;__dmb();
    uint64_t until=time_us_64()+(uint64_t)CLEGG_NINA_IO_TIMEOUT_MS*1000;
    while(stop_ack!=ticket) {
        if(time_us_64()>=until) {
            stop_failures++;
            printf("ACQUIRE stop_timeout reason=%s failures=%u\n",reason,stop_failures);
            return false;
        }
        tight_loop_contents();
    }
    /* Core 1 cannot add samples again until this core publishes START. */
    __dmb();tail=head;__dmb();
    return true;
}
static uint32_t crc_step(uint32_t crc,const uint8_t *data,size_t n) {
    for(size_t i=0;i<n;i++){crc^=data[i];for(int b=0;b<8;b++)crc=(crc>>1)^(0xedb88320u&-(crc&1));}
    return crc;
}
static void frame(uint8_t type,const void *data,uint16_t n) {
    static uint8_t frame_bytes[CLEGG_NINA_MAX_BLOCK];
    if(n+12>sizeof frame_bytes||client_socket<0)return;
    uint8_t h[12]={'C','G','0','1',1,type,n&255,n>>8,0,0,0,0};
    uint32_t crc=crc_step(crc_step(0xffffffff,h+4,4),data,n)^0xffffffff;
    memcpy(h+8,&crc,4);
    memcpy(frame_bytes,h,12);memcpy(frame_bytes+12,data,n);
    if(nina_send(client_socket,frame_bytes,n+12)!=(int)(n+12)){network_failures++;quiesce_acquisition("send_failed");nina_stop(client_socket);client_socket=-1;}
    else packets++;
}
static void info(void) {
    char uid[2*PICO_UNIQUE_BOARD_ID_SIZE_BYTES+1];static char json[1500];
    pico_get_unique_board_id_string(uid,sizeof uid);
    int n=snprintf(json,sizeof json,"{\"type\":\"info\",\"protocol\":\"clegg_v1\",\"firmware\":\"%s\",\"device_id\":\"%s\",\"boot_id\":\"%016llx\",\"board\":\"Nano RP2040 Connect\",\"sensor_declared\":\"ADXL375\",\"devid\":%u,\"spi_hz\":%u,\"sample_hz\":%u,\"nominal_g_per_lsb\":%.6f,\"calibration_id\":null,\"timestamp_source\":\"fifo_poll_nominal_backdate\",\"timestamp_validated\":false,\"filter_delay_s\":null,\"initial_registers\":[%u,%u,%u,%u,%u,%u,%u],\"record_bytes\":%u,\"max_packet_samples\":%u,\"wifi_power_mode_requested\":%u,\"transport\":\"wifi_tcp\",\"nina_firmware\":\"%s\"}",CLEGG_SOURCE_ID,uid,(unsigned long long)boot_id,device_id,CLEGG_SPI_HZ,CLEGG_SAMPLE_HZ,(double)CLEGG_NOMINAL_G_PER_LSB,initial_regs[0],initial_regs[1],initial_regs[2],initial_regs[3],initial_regs[4],initial_regs[5],initial_regs[6],(unsigned)sizeof(Sample),CLEGG_NETWORK_PACKET_SAMPLES,CLEGG_NINA_POWER_MODE,nina_version);
    frame(1,json,n);
}
static void status(void) {
    static char json[400];int n=snprintf(json,sizeof json,"{\"type\":\"status\",\"boot_id\":\"%016llx\",\"active\":%s,\"sensor_ok\":%s,\"fifo_losses\":%u,\"ring_drops\":%u,\"stop_failures\":%u,\"device_us\":%llu}",(unsigned long long)boot_id,active?"true":"false",sensor_ok?"true":"false",fifo_losses,ring_drops,stop_failures,(unsigned long long)time_us_64());
    frame(4,json,n);
}
static void command(const char *line){
    uint64_t rx=time_us_64();
    if(!strcmp(line,"INFO"))info();
    else if(!strcmp(line,"START")){if(!requested&&quiesce_acquisition("start")){__dmb();requested=true;}}
    else if(!strcmp(line,"STOP"))quiesce_acquisition("stop");
    else if(!strncmp(line,"SYNC ",5)){
        char *end;unsigned long nonce=strtoul(line+5,&end,10);
        if(!*end){static char json[250];int n=snprintf(json,sizeof json,"{\"type\":\"sync\",\"boot_id\":\"%016llx\",\"nonce\":%lu,\"rx_us\":%llu,\"tx_us\":%llu}",(unsigned long long)boot_id,nonce,(unsigned long long)rx,(unsigned long long)time_us_64());frame(3,json,n);}
    }
}
static void usb_status(void){
    uint8_t ip[4]={0},subnet[4];nina_local_ip(ip,subnet);
    printf("CLEGG_WIFI source=%s sensor=%s devid=0x%02x wifi=%d ip=%u.%u.%u.%u active=%u fifo=%u ring=%u packets=%u failed=%u nina=%s reason=%d driver=%s\n",CLEGG_SOURCE_ID,sensor_ok?"OK":"ERROR",device_id,nina_status(),ip[0],ip[1],ip[2],ip[3],active,fifo_losses,ring_drops,packets,network_failures,nina_version,nina_reason_code(),nina_error());
}
static bool parse_ipv4(const char *text,uint8_t ip[4]) {
    unsigned a,b,c,d;char extra;
    if(sscanf(text,"%u.%u.%u.%u%c",&a,&b,&c,&d,&extra)!=4||a>255||b>255||c>255||d>255)return false;
    ip[0]=a;ip[1]=b;ip[2]=c;ip[3]=d;return true;
}
static bool setup_nina(void) {
    bool ok=nina_init();
    printf("INIT nina_bus=%u\n",ok);
    if(!ok)return false;
    if(!nina_firmware_version(nina_version,sizeof nina_version))return false;
    printf("INIT nina=%s\n",nina_version);
    if(CLEGG_WIFI_STATIC_IP[0]) {
        uint8_t ip[4],gateway[4],subnet[4];
        if(!parse_ipv4(CLEGG_WIFI_STATIC_IP,ip)||!parse_ipv4(CLEGG_WIFI_GATEWAY,gateway)||
           !parse_ipv4(CLEGG_WIFI_NETMASK,subnet)) {
            printf("WIFI invalid static IPv4 configuration\n");return false;
        }
        if(!nina_config(ip,gateway,subnet)) {
            printf("WIFI static configuration failed driver=%s\n",nina_error());return false;
        }
        printf("WIFI static IP=%u.%u.%u.%u\n",ip[0],ip[1],ip[2],ip[3]);
    }
    return true;
}
static void announce(void) {
    uint8_t ip[4],subnet[4],broadcast[4];
    if(!nina_local_ip(ip,subnet)||!(ip[0]|ip[1]|ip[2]|ip[3])||
       !(subnet[0]|subnet[1]|subnet[2]|subnet[3])) {network_failures++;return;}
    for(unsigned i=0;i<4;i++)broadcast[i]=ip[i]|(uint8_t)~subnet[i];
    if(announce_socket<0)announce_socket=nina_udp_begin(0);
    if(announce_socket<0){network_failures++;return;}
    char uid[2*PICO_UNIQUE_BOARD_ID_SIZE_BYTES+1];static char json[384];
    pico_get_unique_board_id_string(uid,sizeof uid);
    int length=snprintf(json,sizeof json,"{\"type\":\"clegg_announce\",\"protocol\":\"clegg_v1\",\"device_id\":\"%s\",\"boot_id\":\"%016llx\",\"port\":%u,\"sample_hz\":%u,\"firmware\":\"%s\"}",uid,(unsigned long long)boot_id,CLEGG_TCP_PORT,CLEGG_SAMPLE_HZ,CLEGG_SOURCE_ID);
    if(length<0||length>=(int)sizeof json||
       !nina_udp_send(announce_socket,broadcast,CLEGG_DISCOVERY_PORT,json,(size_t)length)) {
        network_failures++;nina_stop(announce_socket);announce_socket=-1;
    }
}
int main(void){
    stdio_init_all();boot_id=get_rand_64();
    gpio_init(CLEGG_SPI_CS_GPIO);gpio_put(CLEGG_SPI_CS_GPIO,1);gpio_set_dir(CLEGG_SPI_CS_GPIO,GPIO_OUT);
    spi_init(spi0,CLEGG_SPI_HZ);spi_set_format(spi0,8,SPI_CPOL_1,SPI_CPHA_1,SPI_MSB_FIRST);
    gpio_set_function(CLEGG_SPI_MISO_GPIO,GPIO_FUNC_SPI);gpio_set_function(CLEGG_SPI_MOSI_GPIO,GPIO_FUNC_SPI);gpio_set_function(CLEGG_SPI_SCK_GPIO,GPIO_FUNC_SPI);
    for(unsigned i=0;i<7;i++)initial_regs[i]=rd(reg_ids[i]);device_id=initial_regs[0];sensor_ok=device_id==0xe5;
    standby_clear_old_data();
    multicore_launch_core1(acquire);sleep_ms(2500);printf("INIT sensor=0x%02x\n",device_id);
    bool radio_ready=setup_nina();
    char input[80],usb[40];unsigned used=0,usb_used=0;
    uint64_t retry=0,last_usb=0,last_check=0,last_packet=0,last_status=0,last_announce=0,last_server_try=0;
    int wifi=0;bool listening=false,was_connected=false;
    for(;;){
        uint64_t now=time_us_64();int c;
        while((c=getchar_timeout_us(0))>=0){if(c=='\n'){usb[usb_used]=0;usb_used=0;if(!strcmp(usb,"BOOT")){if(quiesce_acquisition("boot"))reset_usb_boot(0,0);}else usb_status();}else if(c!='\r'&&usb_used<sizeof usb-1)usb[usb_used++]=c;}
        if(!last_check||now-last_check>1000000){wifi=nina_status();last_check=now;}
        if(wifi!=3){
            if(requested||active)quiesce_acquisition("wifi_lost");
            if(was_connected) {
                /* Legacy STOP closes client sockets but cannot close listeners.
                 * Reset once per lost WLAN to release every module socket. */
                client_socket=server_socket=announce_socket=-1;listening=false;used=0;
                was_connected=false;retry=last_check=last_announce=last_server_try=0;
                printf("WIFI lost; resetting module sockets\n");radio_ready=setup_nina();
                now=time_us_64();
            }
            if(!retry||now-retry>CLEGG_WIFI_RETRY_MS*1000ULL){
                if(!radio_ready)radio_ready=setup_nina();
                if(radio_ready) {
                    printf("WIFI join\n");
                    wifi=nina_join(CLEGG_WIFI_SSID,CLEGG_WIFI_PASSWORD,CLEGG_WIFI_JOIN_TIMEOUT_MS);
                    int reason=nina_reason_code();
                    printf("WIFI join_result=%d reason=%d driver=%s\n",wifi,reason,nina_error());
                    usb_status();
                }
                retry=last_check=time_us_64();
            }
            sleep_ms(2);continue;
        }
        if(!was_connected) {
            if(!nina_set_power_mode(CLEGG_NINA_POWER_MODE)) {
                network_failures++;wifi=-1;was_connected=true;continue;
            }
            printf("WIFI power_mode_requested=%u acknowledged=1\n",CLEGG_NINA_POWER_MODE);
        }
        was_connected=true;
        if(!listening) {
            if(!last_server_try||now-last_server_try>CLEGG_WIFI_RETRY_MS*1000ULL) {
                last_server_try=now;server_socket=nina_server_begin(CLEGG_TCP_PORT);listening=server_socket>=0;usb_status();
            }
            if(!listening){sleep_ms(2);continue;}
        }
        if(client_socket<0){int next=nina_server_client(server_socket);if(next>=0&&next<255){if(quiesce_acquisition("accept")){client_socket=next;used=0;info();}else nina_stop(next);}}
        if(client_socket>=0){
            if(nina_client_connected(client_socket)<=0){quiesce_acquisition("client_closed");nina_stop(client_socket);client_socket=-1;}
            else{
                int ready=nina_available(client_socket);uint8_t bytes[80];
                if(ready>0){int got=nina_recv(client_socket,bytes,ready>80?80:ready);for(int i=0;i<got;i++){c=bytes[i];if(c=='\n'){input[used]=0;used=0;command(input);}else if(c!='\r'){if(used<sizeof input-1)input[used++]=c;else used=0;}}}
                uint32_t ready_count=(head+CLEGG_RING_SAMPLES-tail)%CLEGG_RING_SAMPLES;
                if(requested&&active&&ready_count&&(ready_count>=CLEGG_NETWORK_PACKET_SAMPLES||now-last_packet>CLEGG_PARTIAL_PACKET_MS*1000ULL)){
                    static uint8_t payload[10+CLEGG_NETWORK_PACKET_SAMPLES*sizeof(Sample)];unsigned count=0;memcpy(payload,&boot_id,8);
                    while(tail!=head&&count<CLEGG_NETWORK_PACKET_SAMPLES){__dmb();memcpy(payload+10+count*sizeof(Sample),&ring[tail],sizeof(Sample));__dmb();tail=(tail+1)%CLEGG_RING_SAMPLES;count++;}
                    payload[8]=count&255;payload[9]=count>>8;frame(2,payload,10+count*sizeof(Sample));last_packet=now;
                }
                if(now-last_status>1000000){status();last_status=now;}
            }
        }
        if(!last_announce||now-last_announce>CLEGG_WIFI_ANNOUNCE_MS*1000ULL){announce();last_announce=now;}
        if(now-last_usb>2000000){usb_status();last_usb=now;}
        sleep_us(100);
    }
}
