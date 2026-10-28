/* GPIO map extracted from the final PCB and schematic, 2026-10-08.
 * GPIO numbers are not module pad numbers. This header contains no drivers.
 * USB pins 19/20 are reserved for native USB. UART0 pins 43/44 are used by MMC5983MA.
 */
#ifndef SCOUT_BOARD_PINS_H
#define SCOUT_BOARD_PINS_H

enum scout_board_gpio {
    SCOUT_GPIO_ESP_BOOT = 0, /* U1 pad 4 */
    SCOUT_GPIO_ENC_B = 2, /* U1 pad 6 */
    SCOUT_GPIO_ENC_A = 4, /* U1 pad 8 */
    SCOUT_GPIO_VBAT_ADC = 6, /* U1 pad 10 */
    SCOUT_GPIO_MOTOR_CURRENT_ADC = 7, /* U1 pad 11 */
    SCOUT_GPIO_DRV_NFAULT = 12, /* U1 pad 16 */
    SCOUT_GPIO_DRV_SLEEP = 13, /* U1 pad 17 */
    SCOUT_GPIO_MOTOR_DIR = 14, /* U1 pad 18 */
    SCOUT_GPIO_MOTOR_PWM = 15, /* U1 pad 19 */
    SCOUT_GPIO_USB_DM = 19, /* U1 pad 23 */
    SCOUT_GPIO_USB_DP = 20, /* U1 pad 24 */
    SCOUT_GPIO_SPI_ADXL_SCK = 26, /* U1 pad 26 */
    SCOUT_GPIO_SPI_ADXL_MISO = 33, /* U1 pad 28 */
    SCOUT_GPIO_ADXL_INT1 = 34, /* U1 pad 29 */
    SCOUT_GPIO_ICM_INT1 = 35, /* U1 pad 31 */
    SCOUT_GPIO_SPI_ICM_MISO = 36, /* U1 pad 32 */
    SCOUT_GPIO_SPI_ICM_MOSI = 37, /* U1 pad 33 */
    SCOUT_GPIO_SPI_ICM_SCK = 38, /* U1 pad 34 */
    SCOUT_GPIO_ICM_CS = 39, /* U1 pad 35 */
    SCOUT_GPIO_SPI_MAG_MISO = 40, /* U1 pad 36 */
    SCOUT_GPIO_MAG_CS = 41, /* U1 pad 37 */
    SCOUT_GPIO_SPI_MAG_SCK = 42, /* U1 pad 38 */
    SCOUT_GPIO_SPI_MAG_MOSI = 43, /* U1 pad 39 */
    SCOUT_GPIO_MAG_INT = 44, /* U1 pad 40 */
    SCOUT_GPIO_SERVO2_PWM = 45, /* U1 pad 41 */
    SCOUT_GPIO_SERVO1_PWM = 46, /* U1 pad 44 */
    SCOUT_GPIO_SPI_ADXL_MOSI = 47, /* U1 pad 27 */
    SCOUT_GPIO_ADXL_CS = 48, /* U1 pad 30 */
};

#define SCOUT_ICM45605_WHO_AM_I 0xE5u

#endif
