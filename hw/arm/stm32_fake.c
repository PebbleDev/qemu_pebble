/*
 * STM32 Microcontroller Fake Register module.
 * Used to dump register accesses live.
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
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
#include "hw/arm/stm32_clktree.h"
#include "qemu/bitops.h"
#include <stdio.h>

#define TYPE_STM32_FAKE_DEVICE "stm32-fake"
#define STM32_FAKE_DEVICE(obj) \
    OBJECT_CHECK(Stm32Fake, (obj), TYPE_STM32_FAKE_DEVICE)


/* DEFINITIONS*/

/* See README for DEBUG details. */
#define DEBUG_STM32_FAKE

#ifdef DEBUG_STM32_FAKE
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_FAKE: " fmt , ## __VA_ARGS__); fflush(stdout); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


struct Stm32Fake{
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    uint32_t size;

    /* Private */
    MemoryRegion iomem;
    uint32_t *regs;
};


static uint64_t stm32_fake_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Fake *s = (Stm32Fake *)opaque;
    DPRINTF("Read (%s), Offset=0x%x, size=%u\n", stm32_periph_name(s->periph), (unsigned int)offset, size);
   return s->regs[offset];
}

static void stm32_fake_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Fake *s = (Stm32Fake *)opaque;
    DPRINTF("Write (%s), Offset=0x%x, Size=%u, value = %" PRIx64 "\n", stm32_periph_name(s->periph), (unsigned int)offset, size, value);
    s->regs[offset] = value;
}

static const MemoryRegionOps stm32_fake_ops = {
    .read = stm32_fake_read,
    .write = stm32_fake_write,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static int stm32_fake_init(SysBusDevice *dev)
{
    Stm32Fake *s = STM32_FAKE_DEVICE(dev);

    memory_region_init_io(&s->iomem, NULL, &stm32_fake_ops, s,
                          "fake", s->size);

    sysbus_init_mmio(dev, &s->iomem);
    s->regs = g_new(uint32_t, s->size);
    memset(s->regs, 0, sizeof(uint32_t)*s->size);
    return 0;
}

/*void stm32_fake_reset(void)
{
}*/

static Property stm32_fake_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Fake, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_UINT32("size", Stm32Fake, size, 0),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_fake_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_fake_init;
//    dc->reset = stm32_fake_reset;
    dc->props = stm32_fake_properties;
}

static TypeInfo stm32_fake_info = {
    .name  = "stm32-fake",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Fake),
    .class_init = stm32_fake_class_init
};

static void stm32_fake_register_types(void)
{
    type_register_static(&stm32_fake_info);
}

type_init(stm32_fake_register_types)
