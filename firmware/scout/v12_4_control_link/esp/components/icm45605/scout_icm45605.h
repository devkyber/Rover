#ifndef SCOUT_ICM45605_H
#define SCOUT_ICM45605_H
#include "imu/inv_imu_driver.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Call after inv_imu_init/reset completes and before enabling measurements.
 * Requires the official driver transport and a serialized sensor bus.
 * Board: AP SPI; pins 2/3/9 NC, pin 7 VDDIO, pins 10/11 GND.
 */
int scout_icm45605_configure_pads(inv_imu_device_t *device);
#ifdef __cplusplus
}
#endif
#endif
