#include "scout_icm45605.h"

/* Official TDK register map, commit 80533dd8716d2466ef1bf88cff25ec0ee254c838.
 * Use masks rather than compiler-dependent bitfield packing.
 */
int scout_icm45605_configure_pads(inv_imu_device_t *device)
{
    static const struct { uint32_t reg; uint8_t disable; } settings[] = {
        { IPREG_BAR_REG_58, 0x48 }, /* AP_CS / AP_SCLK pull-enable */
        { IPREG_BAR_REG_59, 0x92 }, /* AP_SDI / AP_SDO / pin 7 pull-enable */
        { IPREG_BAR_REG_60, 0x04 }, /* grounded RESV (pin 10) */
        { IPREG_BAR_REG_61, 0x08 }, /* grounded RESV (pin 11) */
    };
    if (!device) return INV_IMU_ERROR_BAD_ARG;
    for (unsigned i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        uint8_t before, after, checked;
        int rc = inv_imu_read_reg(device, settings[i].reg, 1, &before);
        if (rc) return rc;
        after = (uint8_t)(before & (uint8_t)~settings[i].disable);
        if (after != before) {
            rc = inv_imu_write_reg(device, settings[i].reg, 1, &after);
            if (rc) return rc;
        }
        rc = inv_imu_read_reg(device, settings[i].reg, 1, &checked);
        if (rc) return rc;
        if (checked != after) return INV_IMU_ERROR;
    }
    return INV_IMU_OK;
}
