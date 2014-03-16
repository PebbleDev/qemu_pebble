/*
 * STM32F2 Microcontroller ADC controller
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


#define TYPE_STM32_ADC_DEVICE "stm32-adc"
#define STM32_ADC_DEVICE(obj) \
    OBJECT_CHECK(Stm32ADC, (obj), TYPE_STM32_ADC_DEVICE)


/* DEFINITIONS*/
#define DEBUG_STM32_ADC

#ifdef DEBUG_STM32_ADC
#define DPRINT(fmt, ...)                                       \
    do { DPRINTF("STM32_ADC", s->periph, fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...) {}
#endif


// General registers
#define ADC_CSR_OFFSET 0x0
#define ADC_CSR_OVR_BIT 5
#define ADC_CSR_STRT_BIT 4
#define ADC_CSR_JSTRT_BIT 3
#define ADC_CSR_JEOC_BIT 2
#define ADC_CSR_EOC_BIT 1
#define ADC_CSR_AWD_BIT 0

#define ADC_CCR_OFFSET 0x4
#define ADC_CCR_TSVREFE_BIT 23
#define ADC_CCR_VBATE_BIT 22
#define ADC_CCR_ADCPRE_START 16
#define ADC_CCR_ADCPRE_LENGTH 2
#define ADC_CCR_DMA_START 14
#define ADC_CCR_DMA_LENGTH 2
#define ADC_CCR_DDS_BIT 13
#define ADC_CCR_DELAY_START 8
#define ADC_CCR_DELAY_LENGTH 4
#define ADC_CCR_MULTI_START 0
#define ADC_CCR_MULTI_LENGTH 5

#define ADC_CDR_OFFSET 0x8
#define ADC_CDR_DATA2_START 16
#define ADC_CDR_DATA2_LENGTH 16
#define ADC_CDR_DATA1_START 0
#define ADC_CDR_DATA1_LENGTH 16

// Per Channel registers
#define ADC_SR_OFFSET 0x0
#define ADC_SR_STRT_BIT 5
#define ADC_SR_EOC_BIT 1

#define ADC_CR1_OFFSET 0x4
#define ADC_CR1_RES_START 24
#define ADC_CR1_RES_LENGTH 2
#define ADC_CR2_OFFSET 0x8
#define ADC_CR2_SWSTART_BIT 30
#define ADC_CR2_ALIGN_BIT 11

#define ADC_SMPR1_OFFSET 0xc
#define ADC_SQR1_OFFSET 0x2c
#define ADC_SQR2_OFFSET 0x30
#define ADC_SQR3_OFFSET 0x34
#define ADC_DR_OFFSET 0x4c

typedef struct Stm32ADC {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    /* Private */
    MemoryRegion iomem;

    /* Register Values */
    uint32_t
        ADC_CSR,
        ADC_CCR,
        ADC_CDR;

    struct adc_channel_t
    {
        uint32_t
            ADC_SR,
            ADC_CR1,
            ADC_CR2,
            ADC_SQR1;

        bool direction;
        uint8_t resolution;
        uint16_t data;
    } channels[3];

    /* Register Field Values */

//    qemu_irq irq;
} Stm32ADC;




static void stm32_adc_ADC_CR1_write(Stm32ADC *s, struct adc_channel_t *chan, uint32_t reg, uint32_t value)
{
    chan->ADC_CR1 = value;
    chan->resolution = extract32(value, ADC_CR1_RES_START, ADC_CR1_RES_LENGTH);
}


static void stm32_adc_ADC_CR2_write(Stm32ADC *s, struct adc_channel_t *chan, uint32_t reg, uint32_t value)
{
    if(!extract32(chan->ADC_CR2, ADC_CR2_SWSTART_BIT, 1) && extract32(value, ADC_CR2_SWSTART_BIT, 1 ))
    {
        chan->ADC_SR |= BIT(ADC_SR_STRT_BIT) | BIT(ADC_SR_EOC_BIT);
    }
    chan->ADC_CR2 = value & ~BIT(ADC_CR2_SWSTART_BIT);

    chan->direction = extract32(value, ADC_CR2_ALIGN_BIT, 1);
}

static void stm32_adc_ADC_CCR_write(Stm32ADC *s, uint32_t reg, uint32_t value)
{
    s->ADC_CCR = value;
}

static uint16_t STM32_ADC_adc_channel_read_data(Stm32ADC *s, struct adc_channel_t *chan)
{
    uint16_t value=0x0;
    switch(chan->resolution)
    {
        case 0x00:
            // 12 bit
            if(chan->direction == 0) // Right alignment
            {
                value = chan->data & 0xfff;
            }
            else
            {
                value = chan->data << 4 & 0xFFF0;
            }
            break;
        default:
            DPRINT("Resolution not supported %d\n", chan->resolution);
            hw_error("ARG!\n");
            break;
    }
    return value;
}

static uint32_t stm32_adc_channel_read(Stm32ADC *s, uint32_t channel, uint32_t reg)
{
    struct adc_channel_t *chan = &s->channels[channel];
    uint32_t value = 0;
    switch(reg)
    {
        case ADC_SR_OFFSET:
            value = chan->ADC_SR;
            break;
        case ADC_CR1_OFFSET:
            value = chan->ADC_CR1;
            break;
        case ADC_CR2_OFFSET:
            value = chan->ADC_CR2;
            break;
        case ADC_SQR1_OFFSET:
            value = chan->ADC_SQR1;
            break;
        case ADC_DR_OFFSET:
            chan->ADC_SR &= ~BIT(ADC_SR_EOC_BIT);
            value = STM32_ADC_adc_channel_read_data(s, chan);
            break;
        default:
            DPRINT("Reg 0x%X not implemented yet\n", reg);
//            STM32_NOT_IMPL_REG(reg, 4);
            break;
    }
    DPRINT("Read from channel %d, register 0x%X, value 0x%X\n", channel, reg, value);
    return value;
}

static uint32_t stm32_adc_general_read(Stm32ADC *s, uint32_t reg)
{
    uint32_t value = 0;
    switch(reg)
    {
        case ADC_CSR_OFFSET:
            value = s->channels[0].ADC_SR | s->channels[1].ADC_SR << 8 |
                    s->channels[2].ADC_SR << 16;
            break;
        case ADC_CCR_OFFSET:
            value = s->ADC_CCR;
            break;
        case ADC_CDR_OFFSET:
            STM32_NOT_IMPL_REG(reg, 4);
            break;
        default:
            STM32_NOT_IMPL_REG(reg, 4);
            break;
    }
    DPRINT("Read from general, register 0x%X, value 0x%X\n", reg, value);
    return value;
}

static void stm32_adc_channel_write(Stm32ADC *s, uint32_t channel, uint32_t reg, uint32_t value)
{
    DPRINT("Write to channel %d, register 0x%X with value 0x%X\n", channel, reg, (unsigned int)value);
    struct adc_channel_t *chan = &s->channels[channel];

    switch(reg)
    {
        case ADC_SR_OFFSET:
            chan->ADC_SR = value;
            break;

        case ADC_CR1_OFFSET:
            stm32_adc_ADC_CR1_write(s, chan, reg, value);
            break;

        case ADC_CR2_OFFSET:
            stm32_adc_ADC_CR2_write(s, chan, reg, value);
            break;
        default:
            DPRINT("Reg 0x%X not implemented yet\n", reg);

//            STM32_NOT_IMPL_REG(reg, 4);
            break;
    }
}

static void stm32_adc_general_write(Stm32ADC *s, uint32_t reg, uint32_t value)
{
    DPRINT("Write to general register 0x%X with value 0x%X\n", reg, value);
    switch(reg)
    {
        case ADC_CSR_OFFSET:
            STM32_WARN_RO_REG(reg);
            break;
        case ADC_CCR_OFFSET:
            stm32_adc_ADC_CCR_write(s, reg, value);
            break;
        case ADC_CDR_OFFSET:
            STM32_NOT_IMPL_REG(reg, 4);
            break;
        default:
            STM32_NOT_IMPL_REG(reg, 4);
            break;
    }
}

static uint64_t stm32_adc_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32ADC *s = (Stm32ADC*)opaque;
    uint64_t value = 0;
    assert(size == 4);
    int channel = offset>>8;
    int reg = offset & 0xFF;
    switch(channel) {
        case 0x0:
        case 0x1:
        case 0x2:
            value = stm32_adc_channel_read(s, channel, reg);
            break;
        case 0x3:
            value = stm32_adc_general_read(s, reg);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
    DPRINT("Read from 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    return value;
}

static void stm32_adc_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32ADC *s = (Stm32ADC*)opaque;
    assert(size == 4);
    DPRINT("Write to 0x%X with value 0x%X\n", (unsigned int)offset, (unsigned int)value);
    int channel = offset>>8;
    int reg = offset & 0xFF;
    switch(channel) {
        case 0x0:
        case 0x1:
        case 0x2:
            stm32_adc_channel_write(s, channel, reg, value);
            break;
        case 0x3:
            stm32_adc_general_write(s, reg, value);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_adc_ops = {
    .read = stm32_adc_read,
    .write = stm32_adc_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,

    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_adc_reset(DeviceState *dev)
{
    Stm32ADC *s = STM32_ADC_DEVICE(dev);
    s->channels[0].data = 0xFFFF;
    s->channels[1].data = 0xFFFF;
    s->channels[2].data = 0xFFFF;
}


static int stm32_adc_init(SysBusDevice *dev)
{
    Stm32ADC *s = STM32_ADC_DEVICE(dev);

    memory_region_init_io(&s->iomem, NULL, &stm32_adc_ops, s,
                          "adc", 0x400);

    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}


static Property stm32_adc_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32ADC, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_adc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_adc_init;
    dc->reset = stm32_adc_reset;
    dc->props = stm32_adc_properties;
}

static TypeInfo stm32_adc_info = {
    .name  = "stm32-adc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32ADC),
    .class_init = stm32_adc_class_init
};

static void stm32_adc_register_types(void)
{
    type_register_static(&stm32_adc_info);
}

type_init(stm32_adc_register_types)
