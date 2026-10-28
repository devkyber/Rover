#include "sensors.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_intr_alloc.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "scout_icm45605.h"
#include <stdlib.h>
#include <string.h>

static spi_device_handle_t icm_spi, adxl_spi;
static inv_imu_device_t imu;
static pcnt_unit_handle_t encoder;
/* One ISR service core writes these aligned 32-bit counters; task reads are atomic.
 * Diagnostic edge counts may wrap and are separate from filtered PCNT counts. */
static volatile uint32_t encoder_edges_a, encoder_edges_b;
static void IRAM_ATTR encoder_edge_a(void *arg) { (void)arg; ++encoder_edges_a; }
static void IRAM_ATTR encoder_edge_b(void *arg) { (void)arg; ++encoder_edges_b; }
static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t calibration;
static uint64_t next_mag, next_health;

void scout_motor_park(void)
{
    /* Emergency motor park; normal outputs are owned by the control task. */
    gpio_set_level(SCOUT_GPIO_DRV_SLEEP,0);
    gpio_set_level(SCOUT_GPIO_MOTOR_PWM,0);
    gpio_set_level(SCOUT_GPIO_MOTOR_DIR,0);
}
static esp_err_t spi_open(spi_host_device_t host, int sck, int mosi, int miso,
                          int cs, spi_device_handle_t *device)
{
    spi_bus_config_t bus={.mosi_io_num=mosi,.miso_io_num=miso,.sclk_io_num=sck,
        .quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=256};
    esp_err_t rc=spi_bus_initialize(host,&bus,SPI_DMA_DISABLED);
    if (rc!=ESP_OK) return rc;
    spi_device_interface_config_t config={.clock_speed_hz=2000000,.mode=3,
        .spics_io_num=cs,.queue_size=1};
    return spi_bus_add_device(host,&config,device);
}
static esp_err_t hw_read(spi_device_handle_t device, uint8_t command, void *out, size_t n)
{
    uint8_t tx[256]={0},rx[256]={0};
    if (!device || n>255) return ESP_ERR_INVALID_ARG;
    tx[0]=command;
    spi_transaction_t t={.length=(n+1)*8,.tx_buffer=tx,.rx_buffer=rx};
    esp_err_t rc=spi_device_polling_transmit(device,&t);
    if (rc==ESP_OK) memcpy(out,rx+1,n);
    return rc;
}
static esp_err_t hw_write(spi_device_handle_t device, uint8_t command,
                           const uint8_t *data, size_t n)
{
    uint8_t tx[256];
    if (!device || n>255) return ESP_ERR_INVALID_ARG;
    tx[0]=command; memcpy(tx+1,data,n);
    spi_transaction_t t={.length=(n+1)*8,.tx_buffer=tx};
    return spi_device_polling_transmit(device,&t);
}
static int icm_read(void *context,uint8_t reg,uint8_t *out,uint32_t n)
{ return hw_read((spi_device_handle_t)context,reg|0x80,out,n)==ESP_OK ? 0 : -1; }
static int icm_write(void *context,uint8_t reg,const uint8_t *data,uint32_t n)
{ return hw_write((spi_device_handle_t)context,reg&0x7f,data,n)==ESP_OK ? 0 : -1; }
static esp_err_t adxl_write(uint8_t reg,uint8_t value)
{ return hw_write(adxl_spi,reg,&value,1); }
static esp_err_t adxl_read(uint8_t reg,void *data,size_t n)
{ return hw_read(adxl_spi,reg|0x80|(n>1?0x40:0),data,n); }

/* MMC5983MA has its own physical SPI pins: mode 3, slow GPIO transport. */
static uint8_t mag_byte(uint8_t value)
{
    uint8_t out=0;
    for (int bit=7;bit>=0;--bit) {
        gpio_set_level(SCOUT_GPIO_SPI_MAG_SCK,0);
        gpio_set_level(SCOUT_GPIO_SPI_MAG_MOSI,(value>>bit)&1);
        esp_rom_delay_us(1);
        gpio_set_level(SCOUT_GPIO_SPI_MAG_SCK,1);
        esp_rom_delay_us(1);
        out=(uint8_t)((out<<1)|gpio_get_level(SCOUT_GPIO_SPI_MAG_MISO));
    }
    return out;
}
static void mag_read(uint8_t reg,uint8_t *out,size_t n)
{
    gpio_set_level(SCOUT_GPIO_MAG_CS,0); esp_rom_delay_us(1);
    mag_byte(reg|0x80);
    for (size_t i=0;i<n;++i) out[i]=mag_byte(0);
    gpio_set_level(SCOUT_GPIO_MAG_CS,1);
}
static void mag_write(uint8_t reg,uint8_t value)
{
    gpio_set_level(SCOUT_GPIO_MAG_CS,0); esp_rom_delay_us(1);
    mag_byte(reg&0x7f); mag_byte(value);
    gpio_set_level(SCOUT_GPIO_MAG_CS,1);
}
static esp_err_t init_encoder_adc(void)
{
    pcnt_unit_config_t unit={.low_limit=-30000,.high_limit=30000,
        .flags.accum_count=true};
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit,&encoder),"scout","pcnt unit");
    pcnt_chan_config_t a={.edge_gpio_num=SCOUT_GPIO_ENC_A,
                          .level_gpio_num=SCOUT_GPIO_ENC_B};
    pcnt_chan_config_t b={.edge_gpio_num=SCOUT_GPIO_ENC_B,
                          .level_gpio_num=SCOUT_GPIO_ENC_A};
    pcnt_channel_handle_t ca,cb;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(encoder,&a,&ca),"scout","pcnt A");
    ESP_RETURN_ON_ERROR(pcnt_new_channel(encoder,&b,&cb),"scout","pcnt B");
    /* Match JumperControl's positive 00→01→11→10→00 quadrature sequence. */
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(ca,PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE),"scout","pcnt edge A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(cb,PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE),"scout","pcnt edge B");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(ca,PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE),"scout","pcnt level A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(cb,PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE),"scout","pcnt level B");
    pcnt_glitch_filter_config_t filter={.max_glitch_ns=1000};
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(encoder,&filter),"scout","pcnt filter");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder,-30000),"scout","pcnt low");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(encoder,30000),"scout","pcnt high");
    ESP_RETURN_ON_ERROR(pcnt_unit_enable(encoder),"scout","pcnt enable");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(encoder),"scout","pcnt clear");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(encoder),"scout","pcnt start");
    esp_err_t isr=gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr!=ESP_ERR_INVALID_STATE) ESP_RETURN_ON_ERROR(isr,"scout","encoder ISR service");
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(SCOUT_GPIO_ENC_A,GPIO_INTR_ANYEDGE),"scout","encoder A interrupt");
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(SCOUT_GPIO_ENC_B,GPIO_INTR_ANYEDGE),"scout","encoder B interrupt");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(SCOUT_GPIO_ENC_A,encoder_edge_a,NULL),"scout","encoder A ISR");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(SCOUT_GPIO_ENC_B,encoder_edge_b,NULL),"scout","encoder B ISR");
    adc_oneshot_unit_init_cfg_t ac={.unit_id=ADC_UNIT_1};
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&ac,&adc),"scout","adc unit");
    adc_oneshot_chan_cfg_t cc={.atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_DEFAULT};
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc,ADC_CHANNEL_5,&cc),"scout","VBAT ADC");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc,ADC_CHANNEL_6,&cc),"scout","current ADC");
    adc_cali_curve_fitting_config_t cal={.unit_id=ADC_UNIT_1,
        .atten=ADC_ATTEN_DB_12,.bitwidth=ADC_BITWIDTH_DEFAULT};
    /* Missing eFuse calibration is reported as -1 mV, never guessed. */
    if (adc_cali_create_scheme_curve_fitting(&cal,&calibration)!=ESP_OK) calibration=NULL;
    return ESP_OK;
}
esp_err_t scout_sensors_profile(scout_sensors_t *s,bool shock)
{
    if (!s->icm_ok || !s->adxl_ok) return ESP_ERR_INVALID_STATE;
    int rc=inv_imu_set_accel_mode(&imu,PWR_MGMT0_ACCEL_MODE_OFF);
    rc |= inv_imu_set_gyro_mode(&imu,PWR_MGMT0_GYRO_MODE_OFF);
    rc |= inv_imu_set_accel_frequency(&imu,shock?ACCEL_CONFIG0_ACCEL_ODR_6400_HZ:ACCEL_CONFIG0_ACCEL_ODR_400_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu,shock?GYRO_CONFIG0_GYRO_ODR_6400_HZ:GYRO_CONFIG0_GYRO_ODR_400_HZ);
    rc |= inv_imu_set_accel_fsr(&imu,ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G);
    rc |= inv_imu_set_gyro_fsr(&imu,GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
    inv_imu_fifo_config_t fifo={.gyro_en=1,.accel_en=1,.hires_en=0,.fifo_wm_th=8,
        .fifo_mode=FIFO_CONFIG0_FIFO_MODE_STREAM,.fifo_depth=FIFO_CONFIG0_FIFO_DEPTH_MAX};
    rc |= inv_imu_set_fifo_config(&imu,&fifo);
    rc |= inv_imu_set_accel_mode(&imu,PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu,PWR_MGMT0_GYRO_MODE_LN);
    if (rc) return ESP_FAIL;
    ESP_RETURN_ON_ERROR(adxl_write(0x2d,0),"scout","ADXL standby");
    ESP_RETURN_ON_ERROR(adxl_write(0x38,0),"scout","ADXL bypass");
    /* Rev B p24: fixed bits D3/D1/D0 must be 1, unlike ADXL345. */
    ESP_RETURN_ON_ERROR(adxl_write(0x31,0x0b),"scout","ADXL 4-wire/right-justified");
    ESP_RETURN_ON_ERROR(adxl_write(0x2c,shock?0x0f:0x0d),"scout","ADXL rate");
    ESP_RETURN_ON_ERROR(adxl_write(0x38,0x88),"scout","ADXL stream/WM8");
    ESP_RETURN_ON_ERROR(adxl_write(0x2d,0x08),"scout","ADXL measure");
    vTaskDelay(pdMS_TO_TICKS(50));
    if (inv_imu_flush_fifo(&imu)) return ESP_FAIL;
    ESP_RETURN_ON_ERROR(adxl_write(0x38,0),"scout","ADXL flush");
    ESP_RETURN_ON_ERROR(adxl_write(0x38,0x88),"scout","ADXL restart");
    const uint8_t regs[]={0x31,0x2c,0x38,0x2d};
    const uint8_t expected[]={0x0b,shock?0x0f:0x0d,0x88,0x08};
    for (unsigned i=0;i<sizeof(regs);++i) {
        uint8_t value=0;
        ESP_RETURN_ON_ERROR(adxl_read(regs[i],&value,1),"scout","ADXL readback");
        if (value!=expected[i]) return ESP_FAIL;
    }
    s->icm_hz=shock?6400:400; s->adxl_hz=shock?3200:800;
    return ESP_OK;
}
esp_err_t scout_sensors_init(scout_sensors_t *s)
{
    memset(s,0,sizeof(*s));
    gpio_config_t out={.mode=GPIO_MODE_OUTPUT};
    out.pin_bit_mask=(1ULL<<SCOUT_GPIO_SPI_MAG_SCK)|(1ULL<<SCOUT_GPIO_SPI_MAG_MOSI)|(1ULL<<SCOUT_GPIO_MAG_CS);
    ESP_RETURN_ON_ERROR(gpio_config(&out),"scout","mag outputs");
    gpio_set_level(SCOUT_GPIO_MAG_CS,1); gpio_set_level(SCOUT_GPIO_SPI_MAG_SCK,1);
    gpio_config_t input={.pin_bit_mask=(1ULL<<SCOUT_GPIO_SPI_MAG_MISO)|
        (1ULL<<SCOUT_GPIO_DRV_NFAULT),.mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    ESP_RETURN_ON_ERROR(gpio_config(&input),"scout","input pins");
    ESP_RETURN_ON_ERROR(init_encoder_adc(),"scout","encoder/ADC");
    ESP_RETURN_ON_ERROR(spi_open(SPI2_HOST,SCOUT_GPIO_SPI_ICM_SCK,SCOUT_GPIO_SPI_ICM_MOSI,
        SCOUT_GPIO_SPI_ICM_MISO,SCOUT_GPIO_ICM_CS,&icm_spi),"scout","ICM bus");
    ESP_RETURN_ON_ERROR(spi_open(SPI3_HOST,SCOUT_GPIO_SPI_ADXL_SCK,SCOUT_GPIO_SPI_ADXL_MOSI,
        SCOUT_GPIO_SPI_ADXL_MISO,SCOUT_GPIO_ADXL_CS,&adxl_spi),"scout","ADXL bus");
    imu.transport=(inv_imu_transport_t){.context=icm_spi,.read_reg=icm_read,
        .write_reg=icm_write,.serif_type=UI_SPI4,.sleep_us=esp_rom_delay_us};
    vTaskDelay(pdMS_TO_TICKS(20));
    s->icm_ok=inv_imu_get_who_am_i(&imu,&s->icm_id)==0 && s->icm_id==SCOUT_ICM45605_WHO_AM_I;
    s->adxl_ok=adxl_read(0,&s->adxl_id,1)==ESP_OK && s->adxl_id==0xe5;
    mag_read(0x2f,&s->mag_id,1); s->mag_ok=s->mag_id==0x30;
    if (s->icm_ok && (inv_imu_soft_reset(&imu) || scout_icm45605_configure_pads(&imu))) s->icm_ok=false;
    if (s->mag_ok) {
        mag_write(0x0a,0x80); vTaskDelay(pdMS_TO_TICKS(20));
        mag_write(0x09,0x28); esp_rom_delay_us(1000); /* auto SR + initial SET */
        mag_write(0x0a,0); mag_write(0x0b,0xbc); /* 50 Hz, periodic SET */
    }
    return s->icm_ok && s->adxl_ok ? scout_sensors_profile(s,false) : ESP_ERR_NOT_FOUND;
}
void scout_sensors_poll(scout_sensors_t *s,scout_sample_cb cb)
{
    uint64_t now=(uint64_t)esp_timer_get_time();
    if (s->icm_ok) {
        uint16_t count=0; inv_imu_int_state_t status={0};
        if (inv_imu_get_int_status(&imu,INV_IMU_INT1,&status) || inv_imu_get_frame_count(&imu,&count)) ++s->io_errors;
        else {
            if (status.INV_FIFO_FULL) ++s->icm_full;
            unsigned n=count>32?32:count;
            for (unsigned i=0;i<n;++i) {
                inv_imu_fifo_data_t data={0};
                if (inv_imu_get_fifo_frame(&imu,&data)) { ++s->io_errors; break; }
                if ((data.header.Byte&0x60)!=0x60 || (data.header.Byte&0x90)) { ++s->io_errors; continue; }
                uint8_t payload[16]={0},flags=FLAG_TIME_ESTIMATED;
                memcpy(payload,data.byte_16.accel_data,6); memcpy(payload+6,data.byte_16.gyro_data,6);
                payload[12]=(uint8_t)data.byte_16.temp_data; payload[13]=data.header.Byte;
                memcpy(payload+14,&data.byte_16.timestamp,2);
                for (int axis=0;axis<3;++axis) {
                    if (data.byte_16.accel_data[axis]==INT16_MIN || data.byte_16.gyro_data[axis]==INT16_MIN) flags|=FLAG_SENSOR_ERROR;
                    if (abs(data.byte_16.accel_data[axis])>=32700 || abs(data.byte_16.gyro_data[axis])>=32700) flags|=FLAG_SATURATED;
                }
                if (flags&FLAG_SENSOR_ERROR) ++s->io_errors;
                if (status.INV_FIFO_FULL) flags|=FLAG_FIFO_FULL;
                uint64_t age=((uint64_t)(count-1-i)*1000000)/s->icm_hz;
                if (cb) cb(SRC_ICM,flags,now>=age?now-age:0,payload,sizeof(payload));
                ++s->icm_frames;
                if (!(flags&FLAG_SENSOR_ERROR)) s->last_icm_us=now;
            }
        }
    }
    if (s->adxl_ok) {
        uint8_t fifo=0,status=0;
        if (adxl_read(0x30,&status,1)!=ESP_OK || adxl_read(0x39,&fifo,1)!=ESP_OK) ++s->io_errors;
        else {
            unsigned count=fifo&0x3f,n=count>32?32:count;
            if ((status&1) || count>=32) ++s->adxl_full;
            for (unsigned i=0;i<n;++i) {
                int16_t data[3];
                if (adxl_read(0x32,data,sizeof(data))!=ESP_OK) { ++s->io_errors; break; }
                uint8_t flags=FLAG_TIME_ESTIMATED;
                if ((status&1)||count>=32) flags|=FLAG_FIFO_FULL;
                /* ADXL375 ±200g uses approximately 49mg/LSB, not 16-bit full scale. */
                for (int axis=0;axis<3;++axis) if (abs(data[axis])>=4000) flags|=FLAG_SATURATED;
                uint64_t age=((uint64_t)(count-1-i)*1000000)/s->adxl_hz;
                if (cb) cb(SRC_ADXL,flags,now>=age?now-age:0,data,sizeof(data));
                ++s->adxl_frames; s->last_adxl_us=now;
                esp_rom_delay_us(5); /* FIFO-pop timing at SPI >1.6MHz */
            }
        }
    }
    if (s->mag_ok && now>=next_mag) {
        next_mag=now+20000;
        uint8_t status; mag_read(8,&status,1);
        if (status&1) {
            uint8_t raw[7]; int32_t xyz[3]; mag_read(0,raw,sizeof(raw));
            for (unsigned i=0;i<3;++i) xyz[i]=(int32_t)(((uint32_t)raw[2*i]<<10)|
                ((uint32_t)raw[2*i+1]<<2)|((raw[6]>>(6-2*i))&3))-131072;
            if (cb) cb(SRC_MAG,FLAG_MAG_UNQUALIFIED,now,xyz,sizeof(xyz));
            ++s->mag_frames; s->last_mag_us=now; mag_write(8,1);
        }
    }
    if (now>=next_health) {
        next_health=now+100000;
        s->encoder_ready=pcnt_unit_get_count(encoder,&s->encoder_count)==ESP_OK;
        if (!s->encoder_ready) ++s->io_errors;
        s->encoder_a=gpio_get_level(SCOUT_GPIO_ENC_A);
        s->encoder_b=gpio_get_level(SCOUT_GPIO_ENC_B);
        s->encoder_edges_a=encoder_edges_a; s->encoder_edges_b=encoder_edges_b;
        int raw=0,mv=0; s->battery_mv=-1; s->current_adc_mv=-1;
        if (adc_oneshot_read(adc,ADC_CHANNEL_5,&raw)==ESP_OK && calibration &&
            adc_cali_raw_to_voltage(calibration,raw,&mv)==ESP_OK) s->battery_mv=mv*256/56;
        if (adc_oneshot_read(adc,ADC_CHANNEL_6,&raw)==ESP_OK && calibration &&
            adc_cali_raw_to_voltage(calibration,raw,&mv)==ESP_OK) s->current_adc_mv=mv;
        s->driver_fault=gpio_get_level(SCOUT_GPIO_DRV_NFAULT)==0;
        int32_t data[5]={s->encoder_count,s->battery_mv,s->current_adc_mv,
            (int32_t)(s->icm_full+s->adxl_full),s->driver_fault};
        if (cb) cb(SRC_HEALTH,s->io_errors?FLAG_SENSOR_ERROR:0,now,data,sizeof(data));
    }
}

esp_err_t scout_encoder_read(int *value)
{ return encoder ? pcnt_unit_get_count(encoder,value) : ESP_ERR_INVALID_STATE; }
