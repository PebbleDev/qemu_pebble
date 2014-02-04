/*
 * FreeScale MAG3110 Magnetometer.
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
#include "mag3110.h"
#include "qapi/visitor.h"

#define MAG3110_DEBUG

#ifdef MAG3110_DEBUG
#define DPRINT(fmt, ...)                                       \
    do { fprintf(stderr, "MAG3110: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...)
#endif

static void mag3110_interrupt_update(MAG3110State *s)
{
    //qemu_set_irq(s->pin, check
}

static void mag3110_read(MAG3110State *s)
{
    s->len = 0;
    DPRINT("Reading register 0x%X\n", s->pointer);
    switch (s->pointer) {
        case 7:
            s->buf[s->len ++] = 0xC4;
            break;

        default:
            DPRINT("Read unknown register 0x%X\n", s->pointer);
            break;
    }
}

static void mag3110_write(MAG3110State *s)
{
    switch (s->pointer) {
        default:
            DPRINT("Write to unknown register 0x%X\n", s->pointer);
            break;
    }
}

static int mag3110_rx(I2CSlave *i2c)
{
    MAG3110State *s = MAG3110(i2c);
    int value;
    if (s->len < 2) {
        value = s->buf[s->len ++];
    } else {
        value = 0xff;
    }
    DPRINT("Read: %d\n", value);
    return value;

}

static int mag3110_tx(I2CSlave *i2c, uint8_t data)
{
    MAG3110State *s = MAG3110(i2c);
    DPRINT("Write 0x%X\n", data);
    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
         }
        s->len++;
        mag3110_write(s);
    }

    return 0;
}

static void mag3110_event(I2CSlave *i2c, enum i2c_event event)
{
    MAG3110State *s = MAG3110(i2c);
    DPRINT("I2C Event: %d\n", event);
    if (event == I2C_START_RECV) {
        mag3110_read(s);
    }

    s->len = 0;
}

static int mag3110_post_load(void *opaque, int version_id)
{
    MAG3110State *s = opaque;

    mag3110_interrupt_update(s);
    return 0;
}

static const VMStateDescription vmstate_mag3110 = {
    .name = "MAG3110",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .post_load = mag3110_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_UINT8(len, MAG3110State),
        VMSTATE_UINT8_ARRAY(buf, MAG3110State, 2),
        VMSTATE_UINT8(pointer, MAG3110State),
        VMSTATE_UINT8(config, MAG3110State),
        VMSTATE_I2C_SLAVE(i2c, MAG3110State),
        VMSTATE_END_OF_LIST()
    }
};

static void mag3110_reset(I2CSlave *i2c)
{
    MAG3110State *s = MAG3110(i2c);

    s->pointer = 0;
    s->config = 0;

    mag3110_interrupt_update(s);
}

static int mag3110_init(I2CSlave *i2c)
{
    MAG3110State *s = MAG3110(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);

    mag3110_reset(&s->i2c);

    return 0;
}

static void mag3110_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = mag3110_init;
    k->event = mag3110_event;
    k->recv = mag3110_rx;
    k->send = mag3110_tx;
    dc->vmsd = &vmstate_mag3110;
}

static const TypeInfo mag3110_info = {
    .name          = TYPE_MAG3110,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(MAG3110State),
    .class_init    = mag3110_class_init,
};

static void mag3110_register_types(void)
{
    type_register_static(&mag3110_info);
}

type_init(mag3110_register_types)
