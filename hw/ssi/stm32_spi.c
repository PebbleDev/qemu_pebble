/*
 * TI OMAP processor's Multichannel SPI emulation.
 *
 * Copyright (C) 2007-2009 Nokia Corporation
 *
 * Original code for OMAP2 by Andrzej Zaborowski <andrew@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) any later version of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include "hw/hw.h"
#include "hw/arm/stm32.h"
#include "hw/arm/stm32_clktree.h"

/* Multichannel SPI */
struct stm32_spi_s {
    MemoryRegion iomem;
    qemu_irq irq;
    int chnum;

    uint32_t sysconfig;
    uint32_t systest;
    uint32_t irqst;
    uint32_t irqen;
    uint32_t wken;
    uint32_t control;

    struct stm32_spi_ch_s {
        qemu_irq txdrq;
        qemu_irq rxdrq;
        uint32_t (*txrx)(void *opaque, uint32_t, int);
        void *opaque;

        uint32_t tx;
        uint32_t rx;

        uint32_t config;
        uint32_t status;
        uint32_t control;
    } ch[4];
};

static uint64_t stm32_spi_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    struct stm32_spi_s *s = (struct stm32_spi_s *) opaque;
/*    int ch = 0;
    uint32_t ret;

    if (size != 4) {
        return omap_badwidth_read32(opaque, addr);
    }*/

    switch (addr) {
    default:
        STM32_BAD_REG(addr, size);
    return 0;
}

static void stm32_spi_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
//    struct stm32_spi_s *s = (struct stm32_spi_s *) opaque;
/*    int ch = 0;

    if (size != 4) {
        return omap_badwidth_write32(opaque, addr, value);
    }*/

    switch (addr) {
    default:
        STM32_BAD_REG(addr, size);
        return;
    }
}

static const MemoryRegionOps stm32_spi_ops = {
    .read = stm32_spi_read,
    .write = stm32_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

struct stm32_spi_s *stm32_spi_init(int chnum,
                qemu_irq irq, qemu_irq *drq, Clk fclk, Clk iclk)
{
    struct stm32_spi_s *s = (struct stm32_spi_s *)
            g_malloc0(sizeof(struct stm32_spi_s));
    struct stm32_spi_ch_s *ch = s->ch;

    s->irq = irq;
    s->chnum = chnum;
    while (chnum --) {
        ch->txdrq = *drq ++;
        ch->rxdrq = *drq ++;
        ch ++;
    }
//    stm32_spi_reset(s);

      printf("
/*    memory_region_init_io(&s->iomem, &stm32_spi_ops, s, "stm32.spi",
                          2);*/
//    omap_l4_attach(ta, 0, &s->iomem);

    return s;
}

void stm32_spi_attach(struct stm32_spi_s *s,
                uint32_t (*txrx)(void *opaque, uint32_t, int), void *opaque,
                int chipselect)
{
    if (chipselect < 0 || chipselect >= s->chnum)
        hw_error("%s: Bad chipselect %i\n", __FUNCTION__, chipselect);

    s->ch[chipselect].txrx = txrx;
    s->ch[chipselect].opaque = opaque;
}
