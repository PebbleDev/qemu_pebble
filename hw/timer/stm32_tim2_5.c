/*
 * STM32F2 Timer 2 to 5
 * Copyright (C) 2014 Jens Andersen <jens.andersen@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu-common.h"
#include "qemu/bitops.h"
#include "hw/ptimer.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/arm/stm32.h"

#define TYPE_STM32_TIM25_DEVICE "stm32-tim25"
#define STM32_TIM25_DEVICE(obj) \
    OBJECT_CHECK(STM32TIM25State, (obj), TYPE_STM32_TIM25_DEVICE)

#define DEBUG_TIM25

#ifdef DEBUG_TIM25
#define DPRINT(fmt, ...) \
    do { DPRINTF("STM32_TIM25", s->periph, fmt, ## __VA_ARGS__); } while(0)
#else
#define DPRINT(fmt, ...) do {} while(0)
#endif

#define TIM_CR1_OFFSET 0x0
#define TIM_CR1_CEN_BIT 0
#define TIM_CR1_UDIS_BIT 1
#define TIM_CR1_URS_BIT 2
#define TIM_CR1_DIR_BIT 4

#define TIM_CR2_OFFSET 0x4
#define TIM_SMCR_OFFSET 0x8
#define TIM_DIER_OFFSET 0xc
#define TIM_DIER_UIE_BIT 0

#define TIM_SR_OFFSET 0x10
#define TIM_SR_UIF_BIT 0
#define TIM_EGR_OFFSET 0x14
#define TIM_EGR_UG_BIT 0
#define TIM_EGR_TG_BIT 6

#define TIM_CCMR1_OFFSET 0x18
#define TIM_CCMR2_OFFSET 0x1c
#define TIM_CCER_OFFSET 0x20
#define TIM_CNT_OFFSET 0x24
#define TIM_PSC_OFFSET 0x28
#define TIM_ARR_OFFSET 0x2c
#define TIM_CCR1_OFFSET 0x34
#define TIM_CCR2_OFFSET 0x38
#define TIM_CCR3_OFFSET 0x3c
#define TIM_CCR4_OFFSET 0x40
#define TIM_DCR_OFFSET 0x48
#define TIM_DMAR_OFFSET 0x4c
#define TIM_OR_OFFSET 0x50


typedef struct STM32TIM25State {
    SysBusDevice busdev;
    MemoryRegion iomem;

    bool enabled;
    bool direction;
    qemu_irq irq;
    uint32_t
        TIM_CR1,
        TIM_CR2,
        TIM_SR,
        TIM_EGR,
        TIM_PSC,
        TIM_ARR,
        TIM_DIER;


    ptimer_state *ptimer;
    uint32_t freq;
    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;

    Stm32Rcc *stm32_rcc;
} STM32TIM25State;



static void stm32_tim25_trigger_interrupt(STM32TIM25State *s)
{
    if(extract32(s->TIM_DIER, TIM_DIER_UIE_BIT, 1))
    {
//        DPRINT("Triggering irq\n");
        s->TIM_SR |= BIT(TIM_SR_UIF_BIT);
        qemu_irq_raise(s->irq);
    }
}

static void stm32_tim25_tick(void *opaque)
{
    STM32TIM25State *s = (STM32TIM25State*)opaque;
//    DPRINT("Timer tick with counter %u and freq %u\n", s->TIM_ARR, s->freq);
    stm32_tim25_trigger_interrupt(s);
    ptimer_set_freq(s->ptimer, s->freq);
    ptimer_set_count(s->ptimer, s->TIM_ARR);
    ptimer_run(s->ptimer, 1);
}


static void stm32_tim25_update_frequency(STM32TIM25State *s)
{
    uint32_t freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
    if(freq > 0 && s->TIM_PSC > 0)
    {
        s->freq = freq / (s->TIM_PSC);
        DPRINT("%s is running at %lu Hz.\n", stm32_periph_name(s->periph),
               (unsigned long)s->freq);
//        ptimer_set_freq(s->ptimer, s->freq);
    }
}

static void stm32_tim25_clk_irq_handler(void *opaque, int n, int level)
{
    STM32TIM25State *s = (STM32TIM25State*)opaque;
    stm32_tim25_update_frequency(s);
}

static uint32_t stm32_tim25_TIM_CR1_read(STM32TIM25State* s)
{
    return s->enabled << TIM_CR1_CEN_BIT |
           s->direction << TIM_CR1_DIR_BIT;
}

static void stm32_tim25_TIM_CR1_write(STM32TIM25State* s, uint32_t value)
{
    if(!s->enabled && test_bit(TIM_CR1_CEN_BIT, (const unsigned long*)&value))
    {
        assert(s->TIM_ARR > 0);
        assert(s->freq > 0);
        ptimer_set_freq(s->ptimer, s->freq);
        ptimer_set_count(s->ptimer, s->TIM_ARR);
        ptimer_run(s->ptimer, 1);
    } else if (s->enabled && !test_bit(TIM_CR1_CEN_BIT, (const unsigned long*)&value))
    {
        ptimer_stop(s->ptimer);
    }
    s->direction = test_bit(TIM_CR1_DIR_BIT, (const unsigned long*)&value);

}

static void stm32_tim25_TIM_ARR_write(STM32TIM25State* s, uint32_t value)
{

    s->TIM_ARR = value & 0xffff; /* We only support timer3-4 right now */
    /* TODO: DO STUFF */
}

static void stm32_tim25_TIM_PSC_write(STM32TIM25State* s, uint32_t value)
{
    s->TIM_PSC = value & 0xffff;
    stm32_tim25_update_frequency(s);
}

static void stm32_tim25_TIM_PSC2_write(STM32TIM25State* s, uint32_t value)
{
    s->TIM_PSC |= (value & 0xffff) << 16;
}

static void stm32_tim25_TIM_EGR_write(STM32TIM25State* s, const uint64_t value)
{
    if(test_bit(TIM_EGR_UG_BIT, (const uint64_t*)&value))
    {
        // Re-initialize counter
        DPRINT("Re-initializing counter\n");
        if(!test_bit(TIM_CR1_UDIS_BIT, &value))
        {
 /*           if(!test_bit(TIM_CR1_URS_BIT, &value))
            stm32_tim25_trigger_interrupt(s);
                    qemu_irq_raise(s->irq);
                s->TIM_SR |= TIM_SR_UIF_BIT;*/
        }
    }
    if(test_bit(TIM_EGR_TG_BIT, (const uint64_t*)&value))
    {
        DPRINT("Generate Trigger\n");
    }
}

static uint64_t stm32_tim25_read(void *opaque, hwaddr offset,
        unsigned size)
{

    STM32TIM25State* s = (STM32TIM25State*)opaque;

    uint64_t value = 0;
    switch(offset)
    {
        case TIM_CR1_OFFSET:
            value = stm32_tim25_TIM_CR1_read(s);
            break;

        case TIM_DIER_OFFSET:
            value = s->TIM_DIER;
            break;
        case TIM_SR_OFFSET:
            value = s->TIM_SR;
            break;
        case TIM_EGR_OFFSET:
            value = s->TIM_EGR;
            break;
        case TIM_PSC_OFFSET:
            value = s->TIM_PSC;
            break;

        case TIM_ARR_OFFSET:
            value = s->TIM_ARR;
            break;

        default:
            DPRINT("WARNING: Not implemented: 0x%X\n", (uint32_t)offset);
            //STM32_NOT_IMPL_REG(offset, size);
            break;
    }
    DPRINT("Read [0x%X] = 0x%X\n", (uint32_t)offset, (uint32_t)value);
    return value;
}

static void stm32_tim25_write(void *opaque, hwaddr offset,
        uint64_t value, unsigned size)
{
    STM32TIM25State* s = (STM32TIM25State*)opaque;
    DPRINT("Write [0x%X] = 0x%X\n", (uint32_t)offset, (uint32_t)value);
    switch(offset)
    {
        case TIM_CR1_OFFSET:
            stm32_tim25_TIM_CR1_write(s, value);
            break;
        case TIM_DIER_OFFSET:
            s->TIM_DIER = value;
            break;

        case TIM_SR_OFFSET:
            s->TIM_SR = value & BIT(TIM_SR_UIF_BIT);
            if(!extract32(s->TIM_SR, TIM_SR_UIF_BIT, 1))
            {
                qemu_irq_lower(s->irq);
            }
            break;
        case TIM_EGR_OFFSET:
            stm32_tim25_TIM_EGR_write(s, value);
            break;
        case TIM_PSC_OFFSET:
            stm32_tim25_TIM_PSC_write(s, value);
            break;
        case TIM_PSC_OFFSET+2:
            stm32_tim25_TIM_PSC2_write(s, value);
            break;
        case TIM_ARR_OFFSET:
            stm32_tim25_TIM_ARR_write(s, value);
            break;
        default:
            DPRINT("Not implemented 0x%X\n", (uint32_t)offset);
            //STM32_NOT_IMPL_REG(offset, size);
            break;
    }

}

static void stm32_tim25_reset(DeviceState *d)
{
    STM32TIM25State *s = (STM32TIM25State *)d;
    s->TIM_CR1 = 0x0000;
    s->TIM_CR2 = 0x0000;
    s->TIM_SR = 0x0000;
    s->TIM_EGR = 0x0000;
    s->TIM_PSC = 0x0000;
    s->TIM_ARR = 0x0000;
    s->TIM_DIER = 0x0000;
}

static const MemoryRegionOps stm32_tim25_ops = {
    .read = stm32_tim25_read,
    .write = stm32_tim25_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .impl.min_access_size = 2,
    .impl.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static Property stm32_tim25_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", STM32TIM25State, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", STM32TIM25State, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};



static int stm32_tim25_init(SysBusDevice *dev)
{
    STM32TIM25State *s = STM32_TIM25_DEVICE(dev);
    QEMUBH *bh;
    qemu_irq *clk_irq;

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, NULL, &stm32_tim25_ops, s, "stm32-tim25", 0x400);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq);

    bh = qemu_bh_new(stm32_tim25_tick, s);
    s->ptimer = ptimer_init(bh);
    s->freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
    if(s->freq > 0)
        ptimer_set_freq(s->ptimer, s->freq);
    clk_irq =
          qemu_allocate_irqs(stm32_tim25_clk_irq_handler, (void *)s, 1);
    stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);

    return 0;
}


static void stm32_tim25_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_tim25_init;
    dc->reset = stm32_tim25_reset;
    dc->props = stm32_tim25_properties;
}

static const TypeInfo stm32_tim25_info = {
    .name           = "stm32-tim25",
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(STM32TIM25State),
    .class_init     = stm32_tim25_class_init,
};

static void stm32_tim25_register_types(void)
{
    type_register_static(&stm32_tim25_info);
}

type_init(stm32_tim25_register_types)
