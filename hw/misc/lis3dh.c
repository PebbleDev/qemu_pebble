/*
 * STMicroelectronics LIS3DH Magnetometer.
 *
 * Copyright (C) 2014 Jens Andersen <jens.andersen@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "lis3dh.h"
#include "qapi/visitor.h"

#define LIS3DH_DEBUG

#ifdef LIS3DH_DEBUG
#define DPRINT(fmt, ...)                                       \
    do { fprintf(stderr, "LIS3DH: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...)
#endif

static void lis3dh_interrupt_update(LIS3DHState *s)
{
    //qemu_set_irq(s->pin, check
}

static void lis3dh_read(LIS3DHState *s)
{
    s->len = 0;
    DPRINT("Reading register 0x%X\n", s->pointer);
    switch (s->pointer) {
        case 0xF:
            s->buf[s->len ++] = 0x33;
            break;

        default:
            DPRINT("Read unknown register 0x%X\n", s->pointer);
            break;
    }
}

static void lis3dh_write(LIS3DHState *s)
{
    switch (s->pointer) {
        default:
            DPRINT("Write to unknown register 0x%X\n", s->pointer);
            break;
    }
}

static int lis3dh_rx(I2CSlave *i2c)
{
    LIS3DHState *s = LIS3DH(i2c);
    int value;
    if (s->len < 2) {
        value = s->buf[s->len ++];
    } else {
        value = 0xff;
    }
    DPRINT("Read: %d\n", value);
    return value;

}

static int lis3dh_tx(I2CSlave *i2c, uint8_t data)
{
    LIS3DHState *s = LIS3DH(i2c);
    DPRINT("Write 0x%X\n", data);
    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
         }
        s->len++;
        lis3dh_write(s);
    }

    return 0;
}

static void lis3dh_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS3DHState *s = LIS3DH(i2c);
    DPRINT("I2C Event: %d\n", event);
    if (event == I2C_START_RECV) {
        lis3dh_read(s);
    }

    s->len = 0;
}

static int lis3dh_post_load(void *opaque, int version_id)
{
    LIS3DHState *s = opaque;

    lis3dh_interrupt_update(s);
    return 0;
}

static const VMStateDescription vmstate_lis3dh = {
    .name = "LIS3DH",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .post_load = lis3dh_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_UINT8(len, LIS3DHState),
        VMSTATE_UINT8_ARRAY(buf, LIS3DHState, 2),
        VMSTATE_UINT8(pointer, LIS3DHState),
        VMSTATE_UINT8(config, LIS3DHState),
        VMSTATE_I2C_SLAVE(i2c, LIS3DHState),
        VMSTATE_END_OF_LIST()
    }
};

static void lis3dh_reset(I2CSlave *i2c)
{
    LIS3DHState *s = LIS3DH(i2c);

    s->pointer = 0;
    s->config = 0;

    lis3dh_interrupt_update(s);
}

static int lis3dh_init(I2CSlave *i2c)
{
    LIS3DHState *s = LIS3DH(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);

    lis3dh_reset(&s->i2c);

    return 0;
}

static void lis3dh_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = lis3dh_init;
    k->event = lis3dh_event;
    k->recv = lis3dh_rx;
    k->send = lis3dh_tx;
    dc->vmsd = &vmstate_lis3dh;
}

static const TypeInfo lis3dh_info = {
    .name          = TYPE_LIS3DH,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LIS3DHState),
    .class_init    = lis3dh_class_init,
};

static void lis3dh_register_types(void)
{
    type_register_static(&lis3dh_info);
}

type_init(lis3dh_register_types)
