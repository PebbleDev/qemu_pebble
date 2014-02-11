/*
 * Unknown I2C device with the address 0x18
 *
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_UNKI2C_H
#define QEMU_UNKI2C_H

#include "hw/i2c/i2c.h"
//#include "hw/misc/lis3dh_regs.h"

#define TYPE_UNKI2C "unki2c"
#define UNKI2C(obj) OBJECT_CHECK(UNKI2CState, (obj), TYPE_UNKI2C)

typedef struct UNKI2CState {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint8_t config;
} UNKI2CState;

#endif
