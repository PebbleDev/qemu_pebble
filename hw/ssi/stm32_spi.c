/*
 * STM32 Microcontroller SPI controller
 *
 * Copyright (C) 2013 Jens Andersen
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

#include "hw/ssi.h"


/* See README for DEBUG details. */
//#define DEBUG_STM32_SPI

#ifdef DEBUG_STM32_SPI
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_SPI: " fmt , ## __VA_ARGS__); fflush(stdout); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif



/* DEFINITIONS*/
#define SPI_CR1_OFFSET 0x0
#define SPI_CR1_CPHA_BIT 0
#define SPI_CR1_CPOL_BIT 1
#define SPI_CR1_MSTR_BIT 2
#define SPI_CR1_BR_START 3
#define SPI_CR1_BR_LENGTH 3
#define SPI_CR1_BR_MASK 0x38
#define SPI_CR1_SPE_BIT 6
#define SPI_CR1_LSBFIRST_BIT 7
#define SPI_CR1_SSI_BIT 8
#define SPI_CR1_SSM_BIT 9
#define SPI_CR1_RXONLY_BIT 10
#define SPI_CR1_DFF_BIT 11
#define SPI_CR1_CRCNEXT_BIT 12
#define SPI_CR1_CRCEN_BIT 13
#define SPI_CR1_BIDIOE_BIT 14
#define SPI_CR1_BIDIMODE_BIT 15

#define SPI_CR2_OFFSET 0x4
#define SPI_CR2_RXDMAEN_BIT 0
#define SPI_CR2_TXDMAEN_BIT 1
#define SPI_CR2_SSOE_BIT 2
#define SPI_CR2_FRF_BIT 4
#define SPI_CR2_ERRIE_BIT 5
#define SPI_CR2_RXNEIE_BIT 6
#define SPI_CR2_TXEIE_BIT 7

#define SPI_SR_OFFSET 0x8
#define SPI_SR_RXNE_BIT 0
#define SPI_SR_TXE_BIT 1
#define SPI_SR_CHSIDE_BIT 2
#define SPI_SR_UDR_BIT 3
#define SPI_SR_CRCERR_BIT 4
#define SPI_SR_MODF_BIT 5
#define SPI_SR_OVR_BIT 6
#define SPI_SR_BSY_BIT 7
#define SPI_SR_TIFRFE_BIT 8

#define SPI_DR_OFFSET 0xc
#define SPI_DR_DR_START 0
#define SPI_DR_DR_LENGTH 16
#define SPI_DR_DR_MASK 0xFFFF
#define SPI_CRCPR_OFFSET 0x10
#define SPI_RXCRCR_OFFSET 0x14
#define SPI_TXCRCR_OFFSET 0x18
#define SPI_I2SCFGR_OFFSET 0x1c



struct Stm32Spi{
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    uint8_t num_cs;
    qemu_irq *cs_lines;

    /* Private */
    MemoryRegion iomem;
    SSIBus *spi;
    qemu_irq irq;
    int irqline;


    /* Registers */
    uint16_t
        SPI_CR1,
        SPI_CR2,
        SPI_SR,
        SPI_DR,
        SPI_CRCPR,
        SPI_RXCRCR,
        SPI_TXCRCR;
};


static void stm32_spi_update_cs(Stm32Spi *s)
{
/*   int i;

    for (i = 0; i < s->num_cs; ++i) {
        qemu_set_irq(s->cs_lines[i], !(~s->regs[R_SPISSR] & 1 << i));
    }*/
}

static void stm32_spi_update_irq(Stm32Spi *s)
{
//    uint32_t pending;

/*    s->regs[R_IPISR] |=
            (!fifo8_is_empty(&s->rx_fifo) ? IRQ_DRR_NOT_EMPTY : 0) |
            (fifo8_is_full(&s->rx_fifo) ? IRQ_DRR_FULL : 0);*/

/*    pending = s->regs[R_IPISR] & s->regs[R_IPIER];

    pending = pending && (s->regs[R_DGIER] & R_DGIER_IE);
    pending = !!pending;

    // This call lies right in the data paths so don't call the
    //   irq chain unless things really changed.
    if (pending != s->irqline) {
        s->irqline = pending;
        DB_PRINT("irq_change of state %d ISR:%x IER:%X\n",
                    pending, s->regs[R_IPISR], s->regs[R_IPIER]);
        qemu_set_irq(s->irq, pending);
    }*/

}



static uint64_t stm32_spi_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Spi *s = (Stm32Spi *)opaque;
    uint16_t val;
    assert(size == 2);
    switch(offset)
    {
        case SPI_CR1_OFFSET:
            val = s->SPI_CR1;
            break;
        case SPI_CR2_OFFSET:
            val = s->SPI_CR2;
            break;
        case SPI_SR_OFFSET:
            val = s->SPI_SR;
            s->SPI_SR |= SPI_SR_TXE_BIT;
            break;
        case SPI_DR_OFFSET:
            s->SPI_SR &= ~SPI_SR_RXNE_BIT;
            val = s->SPI_DR;
            s->SPI_DR = 0x0;
            break;
        case SPI_CRCPR_OFFSET:
            val = s->SPI_CRCPR;
            break;
        case SPI_I2SCFGR_OFFSET:
            val = 0;
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
    DPRINTF("Read (%s), Offset=0x%x, value=0x%x\n", stm32_periph_name(s->periph), (unsigned int)offset, val);

    return val;
}


static void stm32_SPI_DR_write(Stm32Spi *s, uint32_t value)
{
    uint32_t rx;
    s->SPI_SR &= ~SPI_SR_TXE_BIT;

/*    if(s->SPI_CR1 & SPI_CR1_DFF_BIT)
    {*/
        rx = ssi_transfer(s->spi, value);
//    }
    s->SPI_DR = rx;
    s->SPI_SR |= SPI_SR_RXNE_BIT;
    s->SPI_SR |= SPI_SR_TXE_BIT;

}

static void stm32_spi_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Spi *s = (Stm32Spi *)opaque;
    DPRINTF("Write (%s), Offset=0x%x, Size=%u, value = %" PRIx64 "\n", stm32_periph_name(s->periph), (unsigned int)offset, size, value);
    assert(size == 2);
    switch(offset)
    {
        case SPI_CR1_OFFSET:
            s->SPI_CR1 = (uint16_t)value;
            break;
        case SPI_CR2_OFFSET:
            s->SPI_CR2 = (uint16_t)value;
            break;
        case SPI_SR_OFFSET:
            s->SPI_SR = (uint16_t)value;
            break;
        case SPI_DR_OFFSET:
            stm32_SPI_DR_write(s, (uint32_t)value);
            break;
        case SPI_CRCPR_OFFSET:
            s->SPI_CRCPR = (uint16_t)value;
        case SPI_I2SCFGR_OFFSET:
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_spi_ops = {
    .read = stm32_spi_read,
    .write = stm32_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_spi_reset(DeviceState *dev)
{
    Stm32Spi *s = FROM_SYSBUS(Stm32Spi, SYS_BUS_DEVICE(dev));

    s->SPI_CR1 = 0x0000;
    s->SPI_CR2 = 0x0000;
    s->SPI_SR = 0x0002;
    s->SPI_DR = 0;
    stm32_spi_update_irq(s);
    stm32_spi_update_cs(s);
}

static Property stm32_spi_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Spi, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_UINT8("num-ss-bits", Stm32Spi, num_cs, 1),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_spi_realize(DeviceState *dev, Error **errp)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);
    Stm32Spi *s = FROM_SYSBUS(Stm32Spi, sbd);
    int i;

    s->spi = ssi_create_bus(dev, "spi");
    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new(qemu_irq, s->num_cs);
    ssi_auto_connect_slaves(DEVICE(s), s->cs_lines, s->spi);
    for (i = 0; i < s->num_cs; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }

    memory_region_init_io(&s->iomem, &stm32_spi_ops, s,
                          "stm32-spi", 0x400);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irqline = -1;
}

static void stm32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
//    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

//    k->init = stm32_spi_init;
    dc->realize = stm32_spi_realize;
    dc->reset = stm32_spi_reset;
    dc->props = stm32_spi_properties;
}

static TypeInfo stm32_spi_info = {
    .name  = "stm32-spi",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Spi),
    .class_init = stm32_spi_class_init
};

static void stm32_spi_register_types(void)
{
    type_register_static(&stm32_spi_info);
}

type_init(stm32_spi_register_types)
