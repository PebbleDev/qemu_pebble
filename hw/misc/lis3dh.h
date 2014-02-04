/*
 * STMicroelectronics LIS3DH
 *
 * Browse the data sheet:
 *
 *    http://datasheet.octopart.com/LIS3DH-STMicroelectronics-datasheet-10026716.pdf
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_LIS3DH_H
#define QEMU_LIS3DH_H

#include "hw/i2c/i2c.h"
//#include "hw/misc/lis3dh_regs.h"

#define TYPE_LIS3DH "lis3dh"
#define LIS3DH(obj) OBJECT_CHECK(LIS3DHState, (obj), TYPE_LIS3DH)

typedef struct LIS3DHState {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint8_t config;
} LIS3DHState;

#endif
