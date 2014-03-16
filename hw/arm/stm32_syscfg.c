/*
 * STM32F2 Microcontroller SYSCFG controller
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


#define TYPE_STM32_SYSCFG_DEVICE "stm32-syscfg"
#define STM32_SYSCFG_DEVICE(obj) \
    OBJECT_CHECK(Stm32Syscfg, (obj), TYPE_STM32_SYSCFG_DEVICE)


/* DEFINITIONS*/

#define DEBUG_STM32_SYSCFG

#ifdef DEBUG_STM32_SYSCFG
#define DPRINT(fmt, ...)                                       \
    do { DPRINTF("STM32_SYSCFG", s->periph, fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...) {}
#endif

#define SYSCFG_MEMRM_OFFSET 0x0
#define SYSCFG_EXTICR1_OFFSET 0x8
#define SYSCFG_EXTICR2_OFFSET 0xc
#define SYSCFG_EXTICR3_OFFSET 0x10
#define SYSCFG_EXTICR4_OFFSET 0x14

typedef struct Stm32Syscfg {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    DeviceState *gpio_dev[9]; /* Gpio A to I */
    DeviceState *exti_dev;
    /* Private */
    MemoryRegion iomem;

    /* Register Values */
    uint32_t
        exti[16];


    /* Register Field Values */
//    uint32_t

//    qemu_irq irq;
} Stm32Syscfg;

static void stm32_syscfg_SYSCFG_EXTICR_write(Stm32Syscfg *s, uint32_t regnum,
        uint32_t value, bool init)
{
    uint32_t start_exti = regnum << 2;
    uint32_t i, tmpval;
    DPRINT("Exticr_write regnum=%u, value=0x%X\n", regnum, value);
    for(i=0; i <4; i++)
    {
        uint32_t curr_exti = start_exti + i;
        tmpval = extract32(value, i << 2, 4);
        uint32_t old_gpio = extract32(s->exti[regnum], i<<2, 4);
        assert(tmpval <= 8);
        if(old_gpio != tmpval || init)
        {
            qemu_irq irq = qdev_get_gpio_in(s->exti_dev, curr_exti);
            DPRINT("Connecting EXTI GPIO%c to Pin %u\n", 'A' + tmpval, curr_exti);
            if(tmpval != old_gpio)
            {
                DPRINT("Disonnecting EXTI GPIO%c to Pin %u\n", 'A' + old_gpio, curr_exti);
                sysbus_connect_irq(SYS_BUS_DEVICE(s->gpio_dev[old_gpio]), curr_exti, NULL);
            }

            sysbus_connect_irq(SYS_BUS_DEVICE(s->gpio_dev[tmpval]), curr_exti, irq);
        }
    }
    s->exti[regnum] = value;
}


static uint64_t stm32_syscfg_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Syscfg *s = (Stm32Syscfg*)opaque;
    uint64_t value;
    assert(size == 4);
    switch(offset) {
        case SYSCFG_MEMRM_OFFSET:
            // We don't support remapping
            value = 0;
            break;
        case SYSCFG_EXTICR1_OFFSET:
        case SYSCFG_EXTICR2_OFFSET:
        case SYSCFG_EXTICR3_OFFSET:
        case SYSCFG_EXTICR4_OFFSET:
            value = s->exti[offset - SYSCFG_EXTICR1_OFFSET];
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
    DPRINT("Read from 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    return value;
}

static void stm32_syscfg_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Syscfg *s = (Stm32Syscfg*)opaque;
    assert(size == 4);
    DPRINT("Write to 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    switch(offset) {
        case SYSCFG_EXTICR1_OFFSET:
        case SYSCFG_EXTICR2_OFFSET:
        case SYSCFG_EXTICR3_OFFSET:
        case SYSCFG_EXTICR4_OFFSET:
            stm32_syscfg_SYSCFG_EXTICR_write(s, (offset - SYSCFG_EXTICR1_OFFSET) >> 2, value, false);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_syscfg_ops = {
    .read = stm32_syscfg_read,
    .write = stm32_syscfg_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 4,

    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_syscfg_reset(DeviceState *dev)
{
    Stm32Syscfg *s = STM32_SYSCFG_DEVICE(dev);
    memset(s->exti, 0, sizeof(s->exti));
    stm32_syscfg_SYSCFG_EXTICR_write(s, 0, 0x0, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 1, 0x0, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 2, 0x0, true);
    stm32_syscfg_SYSCFG_EXTICR_write(s, 3, 0x0, true);

}


static int stm32_syscfg_init(SysBusDevice *dev)
{
    Stm32Syscfg *s = STM32_SYSCFG_DEVICE(dev);
    uint8_t i;
    char path[1024];
    memory_region_init_io(&s->iomem, NULL, &stm32_syscfg_ops, s,
                          "syscfg", 0x400);

    sysbus_init_mmio(dev, &s->iomem);
    for(i=0; i<9; i++)
    {
        sprintf(path, "/machine/stm32/gpio[%c]", ('a' + i));
        s->gpio_dev[i] = DEVICE(object_resolve_path(path, NULL));
        assert(s->gpio_dev[i]);
    }
    s->exti_dev = DEVICE(object_resolve_path("/machine/stm32/exti", NULL));
    assert(s->exti_dev);

    return 0;
}


static Property stm32_syscfg_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Syscfg, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_syscfg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_syscfg_init;
    dc->reset = stm32_syscfg_reset;
    dc->props = stm32_syscfg_properties;
}

static TypeInfo stm32_syscfg_info = {
    .name  = "stm32-syscfg",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Syscfg),
    .class_init = stm32_syscfg_class_init
};

static void stm32_syscfg_register_types(void)
{
    type_register_static(&stm32_syscfg_info);
}

type_init(stm32_syscfg_register_types)
