/*
 * Freescale MAG3110 Three-Axis, Digital Magnetometer
 *
 * Browse the data sheet:
 *
 *    http://www.freescale.com/files/sensors/doc/data_sheet/MAG3110.pdf
 *
 * Copyright (C) 2014 Jens Andersen <jens.andersen@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_MAG3110_H
#define QEMU_MAG3110_H

#include "hw/i2c/i2c.h"
//#include "hw/misc/mag3110_regs.h"

#define TYPE_MAG3110 "mag3110"
#define MAG3110(obj) OBJECT_CHECK(MAG3110State, (obj), TYPE_MAG3110)

/**
 * MAG3110State:
 * @config: Bits 5 and 6 (value 32 and 64) determine the precision of the
 * temperature. See Table 8 in the data sheet.
 *
 * @see_also: http://www.ti.com/lit/gpn/mag3110
 */
typedef struct MAG3110State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint8_t config;
} MAG3110State;

#endif
