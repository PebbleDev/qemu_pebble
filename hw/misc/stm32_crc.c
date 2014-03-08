/*
 * STM32F2 Microcontroller CRC controller
 *
 * Copyright (C) 2014 Jens Andersen <jens.andersen@gmail.com
 *
 * Implementation based on ST Microelectronics "RM0033 Reference Manual"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "hw/arm/stm32.h"
#include "qemu/bitops.h"
#include <stdio.h>

#define TYPE_STM32_CRC_DEVICE "stm32-crc"
#define STM32_CRC_DEVICE(obj) \
    OBJECT_CHECK(Stm32CRC, (obj), TYPE_STM32_CRC_DEVICE)


/* DEFINITIONS*/
#define DEBUG_STM32_CRC

#ifdef DEBUG_STM32_CRC
#define DPRINT(fmt, ...)                                       \
    do { DPRINTF("STM32_CRC", s->periph, fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...) {}
#endif

#define CRC_DR_OFFSET 0x0
#define CRC_IDR_OFFSET 0x4
#define CRC_CR_OFFSET 0x8
#define CRC_CR_RESET_BIT 0

#define POLYNOMIAL 0x04C11DB7

typedef struct Stm32CRC {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    /* Private */
    MemoryRegion iomem;

    /* Register Values */
    uint32_t
        CRC_DR,
        CRC_IDR;
} Stm32CRC;



static void stm32_crc_CRC_DR_write(Stm32CRC* s, uint32_t value)
{
    uint32_t crc = s->CRC_DR ^ value;
    uint32_t i, carry;
    for(i=0; i<32; i++)
    {
        carry = ((crc & 0x80000000L) ? 1 : 0);
        crc = crc << 1;
        if(carry)
        {
            crc ^= POLYNOMIAL;
        }
    }

    s->CRC_DR = crc & 0xFFFFFFFF;
}

static uint64_t stm32_crc_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32CRC *s = (Stm32CRC*)opaque;
    uint64_t value = 0;
    assert(size == 4);
    switch(offset) {
        case CRC_DR_OFFSET:
            value = s->CRC_DR;
            break;
        case CRC_IDR_OFFSET:
            value = s->CRC_IDR;
            break;
        case CRC_CR_OFFSET:
            value = 0;
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
    DPRINT("Read from 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    return value;
}

static void stm32_crc_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32CRC *s = (Stm32CRC*)opaque;
    assert(size == 4);
    DPRINT("Write to 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    switch(offset) {
        case CRC_DR_OFFSET:
            stm32_crc_CRC_DR_write(s, value);
            break;
        case CRC_IDR_OFFSET:
            s->CRC_IDR = value & 0xFF;
            break;
        case CRC_CR_OFFSET:
            if(extract32(value, CRC_CR_RESET_BIT, 1))
                s->CRC_DR = 0xFFFFFFFF;
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_crc_ops = {
    .read = stm32_crc_read,
    .write = stm32_crc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,

    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_crc_reset(DeviceState *dev)
{
    Stm32CRC *s = STM32_CRC_DEVICE(dev);
    s->CRC_DR = 0xFFFFFFFF;
}


static int stm32_crc_init(SysBusDevice *dev)
{
    Stm32CRC *s = STM32_CRC_DEVICE(dev);

    memory_region_init_io(&s->iomem, NULL, &stm32_crc_ops, s,
                          "crc", 0x400);

    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}


static Property stm32_crc_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32CRC, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_crc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_crc_init;
    dc->reset = stm32_crc_reset;
    dc->props = stm32_crc_properties;
}

static TypeInfo stm32_crc_info = {
    .name  = "stm32-crc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32CRC),
    .class_init = stm32_crc_class_init
};

static void stm32_crc_register_types(void)
{
    type_register_static(&stm32_crc_info);
}

type_init(stm32_crc_register_types)
