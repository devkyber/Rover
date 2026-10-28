/* Minimal legacy NINA SPI protocol for Nano RP2040 Connect.
 * Protocol reference: Arduino WiFiNINA 1.8.14 utility/{spi_drv,server_drv,wifi_drv}
 * and arduino/nina-fw tag 1.4.8 main/CommandHandler.cpp. Written as a standalone
 * Pico SDK driver; no Arduino/Mbed runtime or shared SPI0 state is used.
 */
#include "nina.h"
#include "clegg_config.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

/* Fixed internal board routing, separate from the external ADXL SPI0 pins. */
enum { PIN_RX=CLEGG_NINA_SPI_RX_GPIO, PIN_TX=CLEGG_NINA_SPI_TX_GPIO, PIN_SCK=CLEGG_NINA_SPI_SCK_GPIO, PIN_CS=CLEGG_NINA_SPI_CS_GPIO, PIN_ACK=CLEGG_NINA_ACK_GPIO,
       PIN_RESET=CLEGG_NINA_RESET_GPIO, PIN_GPIO0=CLEGG_NINA_GPIO0, MAX_BLOCK=CLEGG_NINA_MAX_BLOCK };
enum { START=0xe0, END=0xee, ERROR=0xef, REPLY=0x80,
       SET_NET=0x10, SET_PASSPHRASE=0x11, SET_IP=0x14, GET_REASON=0x1f,
       GET_STATUS=0x20, GET_IP=0x21, SCAN=0x27, SERVER_BEGIN=0x28,
       DATA_SENT=0x2a, AVAILABLE=0x2b, STOP=0x2e, CLIENT_STATE=0x2f,
       CLIENT_BEGIN=0x2d, DISCONNECT=0x30, FW_VERSION=0x37, SEND_UDP=0x39, GET_SOCKET=0x3f,
       SEND=0x44, RECV=0x45, INSERT_UDP=0x46 };
typedef struct { const uint8_t *data; uint16_t length; } InParam;
typedef struct { uint8_t *data; uint16_t capacity, length; } OutParam;
static uint32_t io_timeout_ms=CLEGG_NINA_IO_TIMEOUT_MS, errors;
static uint64_t deadline;
static const char *last_error="not initialized";
static bool initialized;

static bool fail(const char *message) {
    gpio_put(PIN_CS,1);last_error=message;errors++;return false;
}
const char *nina_error(void) { return last_error; }
uint32_t nina_error_count(void) { return errors; }
void nina_set_timeout_ms(uint32_t milliseconds) {
    if(milliseconds)io_timeout_ms=milliseconds;
}
static bool expired(void) { return time_us_64()>=deadline; }
static bool wait_ack(bool high) {
    while(gpio_get(PIN_ACK)!=high) {
        if(expired())return fail(high?"ACK did not rise":"ACK did not become ready");
        tight_loop_contents();
    }
    return true;
}
static bool select_slave(void) {
    if(!wait_ack(false))return false;
    gpio_put(PIN_CS,0);
    return wait_ack(true);
}
/* Byte duplex transfer with a deadline, including a stopped SPI clock. */
static bool exchange(uint8_t out,uint8_t *in) {
    while(!spi_is_writable(spi1)) {
        if(expired())return fail("SPI TX timeout");
        tight_loop_contents();
    }
    spi_get_hw(spi1)->dr=out;
    while(!spi_is_readable(spi1)) {
        if(expired())return fail("SPI RX timeout");
        tight_loop_contents();
    }
    uint8_t received=(uint8_t)spi_get_hw(spi1)->dr;
    if(in)*in=received;
    return true;
}
static bool put(uint8_t byte) { return exchange(byte,NULL); }
static bool get(uint8_t *byte) { return exchange(0xff,byte); }
static bool write_length(uint16_t length,bool wide) {
    if(wide&&!put((uint8_t)(length>>8)))return false;
    return put((uint8_t)length);
}
static bool read_length(uint16_t *length,bool wide) {
    uint8_t hi=0,lo;
    if(wide&&!get(&hi))return false;
    if(!get(&lo))return false;
    *length=((uint16_t)hi<<8)|lo;return true;
}
/* expected<0 permits a variable number of reply parameters (scan).
 * Every request, including 16-bit data commands, is padded to 4 bytes.
 * Parameter lengths on the wire are BE; numeric byte buffers retain their
 * documented endianness (socket counts and requested receive size are LE).
 */
static bool request(uint8_t command,const InParam *input,unsigned input_count,
                    bool wide_input,OutParam *output,unsigned output_capacity,
                    int expected,bool wide_output,unsigned *actual_count) {
    if(!initialized)return fail("driver not initialized");
    last_error="ok";
    deadline=time_us_64()+(uint64_t)io_timeout_ms*1000;
    if(!select_slave())return false;
    if(!put(START)||!put(command)||!put((uint8_t)input_count))return false;
    size_t sent=3;
    for(unsigned i=0;i<input_count;i++) {
        if((!wide_input&&input[i].length>255)||
           !write_length(input[i].length,wide_input))return fail("invalid request length");
        sent+=wide_input?2:1;
        for(unsigned j=0;j<input[i].length;j++)if(!put(input[i].data[j]))return false;
        sent+=input[i].length;
    }
    if(!put(END))return false;
    for(sent++;sent%4;sent++)if(!put(0xff))return false;
    gpio_put(PIN_CS,1);

    /* A scan, association command, or TCP write can take longer than bytes. */
    deadline=time_us_64()+(uint64_t)io_timeout_ms*1000;
    if(!select_slave())return false;
    uint8_t byte=0;
    unsigned attempts=0;
    do {
        if(!get(&byte))return false;
        if(byte==ERROR)return fail("module rejected command");
        if(++attempts>1024||expired())return fail("reply start timeout");
    } while(byte!=START);
    if(!get(&byte))return false;
    if(byte!=(uint8_t)(command|REPLY))return fail("reply command mismatch");
    if(!get(&byte))return false;
    unsigned count=byte;
    if(count>output_capacity||(expected>=0&&count!=(unsigned)expected))
        return fail("reply parameter count mismatch");
    for(unsigned i=0;i<count;i++) {
        uint16_t length;
        if(!read_length(&length,wide_output))return false;
        if(length>output[i].capacity)return fail("reply exceeds output capacity");
        output[i].length=length;
        for(unsigned j=0;j<length;j++)if(!get(&output[i].data[j]))return false;
    }
    if(!get(&byte))return false;
    if(byte!=END)return fail("reply terminator mismatch");
    gpio_put(PIN_CS,1);
    if(actual_count)*actual_count=count;
    return true;
}
static int one(uint8_t command,const InParam *input,unsigned count,bool wide_input) {
    uint8_t value[2]={0};OutParam output={value,sizeof value,0};
    if(!request(command,input,count,wide_input,&output,1,1,false,NULL))return -1;
    if(output.length==1)return value[0];
    if(output.length==2)return value[0]|((unsigned)value[1]<<8);
    fail("invalid scalar reply length");return -1;
}
static bool valid_socket(uint8_t socket) {
    if(socket<10)return true;
    return fail("invalid socket number");
}
static int socket_scalar(uint8_t command,uint8_t socket) {
    if(!valid_socket(socket))return -1;
    InParam input={&socket,1};return one(command,&input,1,false);
}

bool nina_init(void) {
    initialized=false;
    gpio_init(PIN_CS);gpio_put(PIN_CS,1);gpio_set_dir(PIN_CS,GPIO_OUT);
    gpio_init(PIN_ACK);gpio_set_dir(PIN_ACK,GPIO_IN);
    gpio_init(PIN_RESET);gpio_set_dir(PIN_RESET,GPIO_OUT);
    gpio_init(PIN_GPIO0);gpio_put(PIN_GPIO0,1);gpio_set_dir(PIN_GPIO0,GPIO_OUT);
    gpio_put(PIN_RESET,0);sleep_ms(10);gpio_put(PIN_RESET,1);sleep_ms(750);
    /* GPIO0 becomes the module's data-available IRQ after boot. */
    gpio_put(PIN_GPIO0,0);gpio_set_dir(PIN_GPIO0,GPIO_IN);
    spi_init(spi1,CLEGG_NINA_SPI_HZ);
    spi_set_format(spi1,8,SPI_CPOL_0,SPI_CPHA_0,SPI_MSB_FIRST);
    gpio_set_function(PIN_RX,GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX,GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,GPIO_FUNC_SPI);
    initialized=true;last_error="ok";
    deadline=time_us_64()+(uint64_t)io_timeout_ms*1000;
    return wait_ack(false);
}
bool nina_firmware_version(char *out,size_t capacity) {
    if(!out||capacity<2||capacity>65535)return fail("invalid firmware buffer");
    out[0]=0;OutParam output={(uint8_t*)out,(uint16_t)(capacity-1),0};
    if(!request(FW_VERSION,NULL,0,false,&output,1,1,false,NULL))return false;
    out[output.length]=0;return true;
}
int nina_status(void) { return one(GET_STATUS,NULL,0,false); }
int nina_reason_code(void) { return one(GET_REASON,NULL,0,false); }
bool nina_set_power_mode(uint8_t mode) {
    if(mode>1)return fail("invalid Wi-Fi power mode");
    InParam input={&mode,1};return one(0x17,&input,1,false)==1;
}
bool nina_disconnect(void) { return one(DISCONNECT,NULL,0,false)==1; }
int nina_join(const char *ssid,const char *password,uint32_t timeout_ms) {
    if(!ssid||!ssid[0]||strlen(ssid)>32||!password||strlen(password)>63) {
        fail("invalid Wi-Fi credentials length");return -1;
    }
    InParam input[2]={{(const uint8_t*)ssid,(uint16_t)strlen(ssid)},
                      {(const uint8_t*)password,(uint16_t)strlen(password)}};
    int result=one(password[0]?SET_PASSPHRASE:SET_NET,input,password[0]?2:1,false);
    if(result!=1){if(result>=0)fail("module refused Wi-Fi join command");return -1;}
    uint64_t until=time_us_64()+(uint64_t)timeout_ms*1000;
    int status;
    do {
        status=nina_status();
        if(status<0||status==3)return status;
        if(time_us_64()>=until)break;
        sleep_ms(200);
    }while(true);
    return status;
}
bool nina_local_ip(uint8_t ip[4],uint8_t subnet[4]) {
    if(!ip)return fail("invalid IP output buffer");
    uint8_t dummy=0xff,gateway[4],discard_subnet[4];InParam input={&dummy,1};
    OutParam output[3]={{ip,4,0},{subnet?subnet:discard_subnet,4,0},{gateway,4,0}};
    if(!request(GET_IP,&input,1,false,output,3,3,false,NULL))return false;
    return output[0].length==4&&output[1].length==4&&output[2].length==4;
}
bool nina_config(const uint8_t ip[4],const uint8_t gateway[4],const uint8_t subnet[4]) {
    if(!ip||!gateway||!subnet)return fail("invalid static IP configuration");
    /* status() initializes the old firmware's WiFi/tcpip subsystem. */
    if(nina_status()<0)return false;
    uint8_t valid=3,actual_ip[4],actual_subnet[4];
    InParam input[4]={{&valid,1},{ip,4},{gateway,4},{subnet,4}};
    /* The 1.4.8 handler leaves its scalar ACK value unspecified. */
    if(one(SET_IP,input,4,false)<0)return false;
    if(!nina_local_ip(actual_ip,actual_subnet))return false;
    if(memcmp(ip,actual_ip,4)||memcmp(subnet,actual_subnet,4))
        return fail("static IP readback mismatch");
    return true;
}
int nina_server_begin(uint16_t port) {
    int socket=one(GET_SOCKET,NULL,0,false);
    if(socket<0)return -1;
    if(socket>=10){fail("no socket available");return -1;}
    uint8_t number=(uint8_t)socket,mode=0,be_port[2]={(uint8_t)(port>>8),(uint8_t)port};
    InParam input[3]={{be_port,2},{&number,1},{&mode,1}};
    if(one(SERVER_BEGIN,input,3,false)!=1)return -1;
    return socket;
}
int nina_server_client(uint8_t server_socket) {
    return socket_scalar(AVAILABLE,server_socket);
}
int nina_client_connected(uint8_t socket) {
    int state=socket_scalar(CLIENT_STATE,socket);
    return state<0?-1:state==4;
}
int nina_available(uint8_t socket) { return socket_scalar(AVAILABLE,socket); }
int nina_recv(uint8_t socket,void *data,size_t capacity) {
    if(!data||!capacity)return 0;
    if(!valid_socket(socket))return -1;
    int available=nina_available(socket);
    if(available<=0)return available;
    if(capacity>MAX_BLOCK)capacity=MAX_BLOCK;
    if(capacity>(unsigned)available)capacity=(unsigned)available;
    uint8_t le_count[2]={(uint8_t)capacity,(uint8_t)(capacity>>8)};
    InParam input[2]={{&socket,1},{le_count,2}};
    OutParam output={(uint8_t*)data,(uint16_t)capacity,0};
    if(!request(RECV,input,2,true,&output,1,1,true,NULL))return -1;
    return output.length;
}
int nina_send(uint8_t socket,const void *data,size_t length) {
    if(!valid_socket(socket))return -1;
    if(!data||!length||length>MAX_BLOCK){fail("invalid TCP send block");return -1;}
    InParam input[2]={{&socket,1},{(const uint8_t*)data,(uint16_t)length}};
    int sent=one(SEND,input,2,true);
    if(sent<0)return -1;
    if(sent>(int)length){fail("invalid TCP sent byte count");return -1;}
    if(one(DATA_SENT,input,1,false)!=1){fail("TCP write acknowledgement failed");return -1;}
    return sent;
}
bool nina_stop(uint8_t socket) { return socket_scalar(STOP,socket)==1; }
int nina_udp_begin(uint16_t local_port) {
    int socket=one(GET_SOCKET,NULL,0,false);
    if(socket<0)return -1;
    if(socket>=10){fail("no UDP socket available");return -1;}
    uint8_t number=(uint8_t)socket,mode=1,be_port[2]={(uint8_t)(local_port>>8),(uint8_t)local_port};
    InParam input[3]={{be_port,2},{&number,1},{&mode,1}};
    if(one(SERVER_BEGIN,input,3,false)!=1){nina_stop(number);return -1;}
    return socket;
}
bool nina_udp_send(uint8_t socket,const uint8_t destination[4],uint16_t port,const void *data,size_t length) {
    if(!valid_socket(socket))return false;
    if(!destination||!data||!length||length>MAX_BLOCK)return fail("invalid UDP datagram");
    uint8_t mode=1,be_port[2]={(uint8_t)(port>>8),(uint8_t)port};
    InParam begin[4]={{destination,4},{be_port,2},{&socket,1},{&mode,1}};
    if(one(CLIENT_BEGIN,begin,4,false)!=1)return fail("UDP beginPacket failed");
    InParam payload[2]={{&socket,1},{(const uint8_t*)data,(uint16_t)length}};
    if(one(INSERT_UDP,payload,2,true)!=1)return fail("UDP payload insertion failed");
    if(socket_scalar(SEND_UDP,socket)!=1)return fail("UDP endPacket failed");
    return true;
}
int nina_scan(char ssids[][33],size_t capacity) {
    if(!ssids||!capacity){fail("invalid scan output buffer");return -1;}
    if(capacity>10)capacity=10;
    /* Receive all public scan slots even when caller only needs the first. */
    uint8_t found[10][32];OutParam output[10];
    for(unsigned i=0;i<10;i++)output[i]=(OutParam){found[i],32,0};
    unsigned total;
    uint32_t previous=io_timeout_ms;if(io_timeout_ms<CLEGG_WIFI_SCAN_TIMEOUT_MS)io_timeout_ms=CLEGG_WIFI_SCAN_TIMEOUT_MS;
    bool ok=request(SCAN,NULL,0,false,output,10,-1,false,&total);
    io_timeout_ms=previous;
    if(!ok)return -1;
    unsigned saved=total<capacity?total:(unsigned)capacity;
    for(unsigned i=0;i<saved;i++){memcpy(ssids[i],found[i],output[i].length);ssids[i][output[i].length]=0;}
    return (int)saved;
}
