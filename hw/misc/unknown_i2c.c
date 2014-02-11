/*
 * STMicroelectronics UNKI2C Magnetometer.
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
#include "unknown_i2c.h"
#include "qapi/visitor.h"

#define UNKI2C_DEBUG

#ifdef UNKI2C_DEBUG
#define DPRINT(fmt, ...)                                       \
    do { fprintf(stderr, "UNKI2C: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...)
#endif

static void unki2c_interrupt_update(UNKI2CState *s)
{
    //qemu_set_irq(s->pin, check
}

static void unki2c_read(UNKI2CState *s)
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

static void unki2c_write(UNKI2CState *s)
{
    switch (s->pointer) {
        default:
            DPRINT("Write to unknown register 0x%X\n", s->pointer);
            break;
    }
}

static int unki2c_rx(I2CSlave *i2c)
{
    UNKI2CState *s = UNKI2C(i2c);
    int value;
    if (s->len < 2) {
        value = s->buf[s->len ++];
    } else {
        value = 0xff;
    }
    DPRINT("Read: %d\n", value);
    return value;

}

static int unki2c_tx(I2CSlave *i2c, uint8_t data)
{
    UNKI2CState *s = UNKI2C(i2c);
    DPRINT("Write 0x%X\n", data);
    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
         }
        s->len++;
        unki2c_write(s);
    }

    return 0;
}

static void unki2c_event(I2CSlave *i2c, enum i2c_event event)
{
    UNKI2CState *s = UNKI2C(i2c);
    DPRINT("I2C Event: %d\n", event);
    if (event == I2C_START_RECV) {
        unki2c_read(s);
    }

    s->len = 0;
}

static int unki2c_post_load(void *opaque, int version_id)
{
    UNKI2CState *s = opaque;

    unki2c_interrupt_update(s);
    return 0;
}

static const VMStateDescription vmstate_unki2c = {
    .name = "UNKI2C",
    .version_id = 0,
    .minimum_version_id = 0,
    .minimum_version_id_old = 0,
    .post_load = unki2c_post_load,
    .fields      = (VMStateField []) {
        VMSTATE_UINT8(len, UNKI2CState),
        VMSTATE_UINT8_ARRAY(buf, UNKI2CState, 2),
        VMSTATE_UINT8(pointer, UNKI2CState),
        VMSTATE_UINT8(config, UNKI2CState),
        VMSTATE_I2C_SLAVE(i2c, UNKI2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void unki2c_reset(I2CSlave *i2c)
{
    UNKI2CState *s = UNKI2C(i2c);

    s->pointer = 0;
    s->config = 0;

    unki2c_interrupt_update(s);
}

static int unki2c_init(I2CSlave *i2c)
{
    UNKI2CState *s = UNKI2C(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);

    unki2c_reset(&s->i2c);

    return 0;
}

static void unki2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = unki2c_init;
    k->event = unki2c_event;
    k->recv = unki2c_rx;
    k->send = unki2c_tx;
    dc->vmsd = &vmstate_unki2c;
}

static const TypeInfo unki2c_info = {
    .name          = TYPE_UNKI2C,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(UNKI2CState),
    .class_init    = unki2c_class_init,
};

static void unki2c_register_types(void)
{
    type_register_static(&unki2c_info);
}

type_init(unki2c_register_types)
