#include "actuators.h"
#include "board_pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static int motor_direction;
static uint64_t wake_us;
static bool ready;
static esp_err_t motor_pwm(unsigned percent)
{
    esp_err_t rc=ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2,(percent*1023u)/100u);
    return rc==ESP_OK?ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_2):rc;
}
void scout_actuators_park(void)
{
    gpio_set_level(SCOUT_GPIO_DRV_SLEEP,0);
    motor_direction=0; wake_us=0;
    if (ready) {
        motor_pwm(0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0);
    }
    else gpio_set_level(SCOUT_GPIO_MOTOR_PWM,0);
}
bool scout_actuators_fault(void)
{ return gpio_get_level(SCOUT_GPIO_DRV_NFAULT)==0; }
esp_err_t scout_actuators_init(void)
{
    gpio_config_t motor={.pin_bit_mask=(1ULL<<SCOUT_GPIO_DRV_SLEEP)|
        (1ULL<<SCOUT_GPIO_MOTOR_PWM)|(1ULL<<SCOUT_GPIO_MOTOR_DIR),.mode=GPIO_MODE_OUTPUT};
    esp_err_t rc=gpio_config(&motor); if (rc!=ESP_OK) return rc;
    scout_actuators_park(); gpio_set_level(SCOUT_GPIO_MOTOR_DIR,0);
    gpio_config_t fault={.pin_bit_mask=1ULL<<SCOUT_GPIO_DRV_NFAULT,
        .mode=GPIO_MODE_INPUT,.pull_up_en=GPIO_PULLUP_ENABLE};
    if ((rc=gpio_config(&fault))!=ESP_OK) return rc;
    /* J4/GPIO46 stays a plain low GPIO; it is never configured as a PWM channel. */
    gpio_config_t unused={.pin_bit_mask=1ULL<<SCOUT_GPIO_SERVO1_PWM,.mode=GPIO_MODE_OUTPUT};
    if ((rc=gpio_config(&unused))!=ESP_OK) return rc;
    gpio_set_level(SCOUT_GPIO_SERVO1_PWM,0);
    ledc_timer_config_t timer={.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_0,
        .duty_resolution=LEDC_TIMER_14_BIT,.freq_hz=50,.clk_cfg=LEDC_AUTO_CLK};
    if ((rc=ledc_timer_config(&timer))!=ESP_OK) return rc;
    ledc_channel_config_t ch={.gpio_num=SCOUT_GPIO_SERVO2_PWM,.speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_0,.timer_sel=LEDC_TIMER_0,.duty=0,.hpoint=0};
    if ((rc=ledc_channel_config(&ch))!=ESP_OK) return rc;
    ledc_timer_config_t mt={.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_1,
        .duty_resolution=LEDC_TIMER_10_BIT,.freq_hz=20000,.clk_cfg=LEDC_AUTO_CLK};
    if ((rc=ledc_timer_config(&mt))!=ESP_OK) return rc;
    ledc_channel_config_t mc={.gpio_num=SCOUT_GPIO_MOTOR_PWM,.speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_2,.timer_sel=LEDC_TIMER_1,.duty=0,.hpoint=0};
    if ((rc=ledc_channel_config(&mc))!=ESP_OK) return rc;
    ready=true; return ESP_OK;
}
static esp_err_t servo(ledc_channel_t channel,bool enabled,unsigned us)
{
    if (enabled && (us<544 || us>2400)) return ESP_ERR_INVALID_ARG;
    uint32_t duty=enabled?(us*16384u+10000u)/20000u:0;
    esp_err_t rc=ledc_set_duty(LEDC_LOW_SPEED_MODE,channel,duty);
    return rc==ESP_OK?ledc_update_duty(LEDC_LOW_SPEED_MODE,channel):rc;
}
esp_err_t scout_actuators_apply(const scout_outputs_t *out,uint64_t now)
{
    if (!ready || !out || out->motor< -1 || out->motor>1 || (out->motor && (out->motor_percent<1 || out->motor_percent>100))) {
        scout_actuators_park(); return ESP_ERR_INVALID_STATE;
    }
    if (servo(LEDC_CHANNEL_0,out->loading_enabled,out->loading_us)!=ESP_OK) {
        scout_actuators_park(); return ESP_FAIL;
    }
    if (!out->motor) {
        gpio_set_level(SCOUT_GPIO_DRV_SLEEP,0); motor_pwm(0);
        motor_direction=0; wake_us=0; return ESP_OK;
    }
    if (out->motor!=motor_direction) {
        gpio_set_level(SCOUT_GPIO_DRV_SLEEP,0);
        if (motor_pwm(0)!=ESP_OK) { scout_actuators_park(); return ESP_FAIL; }
        gpio_set_level(SCOUT_GPIO_MOTOR_DIR,out->motor>0);
        gpio_set_level(SCOUT_GPIO_DRV_SLEEP,1);
        motor_direction=out->motor; wake_us=now+1500; /* DRV8876 wake max 1ms. */
    }
    if (now>=wake_us) {
        if (scout_actuators_fault()) { scout_actuators_park(); return ESP_FAIL; }
        if (motor_pwm(out->motor_percent)!=ESP_OK) {
            scout_actuators_park(); return ESP_FAIL;
        }
    }
    return ESP_OK;
}
