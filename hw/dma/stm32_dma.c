/*
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
#include "qemu-common.h"
#include "qemu/timer.h"
#include "hw/arm/stm32.h"
#include "hw/irq.h"
#include "hw/arm/soc_dma.h"

#define DEBUG_STM32_DMA

#ifdef DEBUG_STM32_DMA
#define DPRINT(fmt, ...)                                       \
    do { DPRINTF("STM32_DMA", s->periph, fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...)
#endif


#define NUM_STREAMS 8

#define DMA_LISR_OFFSET 0x0
#define DMA_HISR_OFFSET 0x4
#define DMA_LIFCR_OFFSET 0x8
#define DMA_HIFCR_OFFSET 0xC

#define DMA_STREAM_START 0x10
#define DMA_STREAM_LENGTH 0x18

#define DMA_SxCR_OFFSET 0x0
#define DMA_SxCR_CHSEL_START 25
#define DMA_SxCR_CHSEL_LENGTH 2
#define DMA_SxCR_MBURST_START 23
#define DMA_SxCR_MBURST_LENGTH 2
#define DMA_SxCR_PBURST_START 21
#define DMA_SxCR_PBURST_LENGTH 2
#define DMA_SxCR_CT_BIT 19
#define DMA_SxCR_DBM_BIT 18
#define DMA_SxCR_PL_START 16
#define DMA_SxCR_PL_LENGTH 2
#define DMA_SxCR_PINCOS_BIT 15
#define DMA_SxCR_MSIZE_START 13
#define DMA_SxCR_MSIZE_LENGTH 2
#define DMA_SxCR_PSIZE_START 11
#define DMA_SxCR_PSIZE_LENGTH 2
#define DMA_SxCR_MINC_BIT 10
#define DMA_SxCR_PINC_BIT 9
#define DMA_SxCR_CIRC_BIT 8
#define DMA_SxCR_DIR_START 6
#define DMA_SxCR_DIR_LENGTH 2
#define DMA_SxCR_PFCTRL_BIT 5
#define DMA_SxCR_TCIE_BIT 4
#define DMA_SxCR_HTIE_BIT 3
#define DMA_SxCR_TEIE_BIT 2
#define DMA_SxCR_DMEIE_BIT 1
#define DMA_SxCR_EN_BIT 0

#define DMA_SxNDTR_OFFSET 0x4
#define DMA_SxPAR_OFFSET 0x8
#define DMA_SxM0AR_OFFSET 0xC
#define DMA_SxM1AR_OFFSET 0x10

#define DMA_SxFCR_OFFSET 0x14
#define DMA_SxFCR_FEIE_BIT 7
#define DMA_SxFCR_FS_START 3
#define DMA_SxFCR_FS_LENGTH 3
#define DMA_SxFCR_FTH_START 0
#define DMA_SxFCR_FTH_LENGTH 2



typedef struct stm32_dma_stream_state {
    uint32_t num;

    /* Registers values */
    uint32_t DMA_SxPAR;
    bool DMA_SxCR_PINC;
    uint32_t DMA_SxCR_PSIZE;

    uint32_t DMA_SxM0AR;
    bool DMA_SxCR_MINC;
    uint32_t DMA_SxCR_MSIZE;

    uint32_t direction;
    uint32_t priority;
    uint32_t size;

    bool enabled;
    uint32_t DMA_SxFCR_FTH;
    bool DMA_SxFCR_FEIE;

    bool DMA_SxCR_TCIE;
    bool DMA_SxCR_TEIE;
    bool DMA_SxCR_DMEIE;

    bool disableOnCRRead;
    qemu_irq irq;
} stm32_dma_stream_state;

typedef struct stm32_dma_state {
    /* Inherited */
    SysBusDevice busdev;

    /* Private */
    MemoryRegion iomem;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;
    void *stm32_gpio_prop;

    /* Referneces */
    Stm32Rcc *stm32_rcc;

    struct stm32_dma_stream_state stream[NUM_STREAMS];


    /* Registers */
    uint32_t DMA_LISR, DMA_HISR;
} stm32_dma_state;

#define TYPE_STM32_DMA_DEVICE "stm32-dma"
#define STM32_DMA_DEVICE(obj) \
    OBJECT_CHECK(stm32_dma_state, (obj), TYPE_STM32_DMA_DEVICE)


static const char* stm32_dma_register_name(uint32_t offset)
{
    switch(offset)
    {
        case DMA_SxCR_OFFSET:
            return "DMA_SxCR";
        case DMA_SxNDTR_OFFSET:
            return "DMA_SxNDTR";
        case DMA_SxPAR_OFFSET:
            return "DMA_SxPAR";
        case DMA_SxM0AR_OFFSET:
            return "MA_SxM0AR";
        case DMA_SxM1AR_OFFSET:
            return "DMA_SxM1AR";
        case DMA_SxFCR_OFFSET:
            return "DMA_SxFCR";
        default:
            return "Unknown";
  }
}

#define DMA_TCIE_MASK (1 << 5 | 1  << 11 | 1 << 21 | 1 <<27)

static void stm32_dma_update_irq(stm32_dma_state *s, stm32_dma_stream_state *stream)
{
    if(stream->DMA_SxCR_TCIE/* && (s->DMA_LISR & DMA_TCIE_MASK || s->DMA_HISR & DMA_TCIE_MASK)*/)
    {
        DPRINT("Raising IRQ for stream %d\n", stream->num);
        qemu_irq_raise(stream->irq);
    }
}

// This only supports memory to peripheral right now. Hacky-hacky.
static void stm32_dma_process_tx(stm32_dma_state *s, stm32_dma_stream_state *stream)
{
    uint32_t curr_src = stream->DMA_SxM0AR;
    uint32_t curr_dst = stream->DMA_SxPAR;

    uint32_t bytes_per_read = stream->DMA_SxCR_MSIZE == 0 ? 1 : (stream->DMA_SxCR_MSIZE == 1 ? 2 : 4);
    uint32_t bytes_per_write = stream->DMA_SxCR_PSIZE == 0 ? 1 : (stream->DMA_SxCR_PSIZE == 1 ? 2 : 4);

    DPRINT("Processing %d bytes from 0x%08X to 0x%08X at %d bytes per read, %d bytes per write\n", stream->size, curr_src, curr_dst, bytes_per_read, bytes_per_write);
    while(stream->size > 0)
    {
        uint32_t data=0;
        cpu_physical_memory_read(curr_src, &data, bytes_per_read);
        data = (((data * 0x0802LU & 0x22110LU) | (data * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16) & 0xFF;
        DPRINT("Reading %d bytes from 0x%08X and writing to 0x%08X, value=0x%X (corrected: 0x%X)\n", bytes_per_read, curr_src, curr_dst, data, data & ((bytes_per_read << 4) - 1));
//        data &= ((bytes_per_read << 4) - 1);
        cpu_physical_memory_write(curr_dst, &data, bytes_per_write);
        
        if(stream->DMA_SxCR_MINC)
            curr_src += bytes_per_read;
        if(stream->DMA_SxCR_PINC)
            curr_dst += bytes_per_write;

        stream->size--;
    }
    stream->disableOnCRRead = true;
    switch(stream->num)
    {
        case 0:
            s->DMA_LISR |= 1 << 5;
            break;
        case 1:
            s->DMA_LISR |= 1 << 11;
            break;
        case 2:
            s->DMA_LISR |= 1 << 21;
            break;
        case 3:
            s->DMA_LISR |= 1 << 27;
            break;
        case 4:
            s->DMA_HISR |= 1 << 5;
            break;
        case 5:
            s->DMA_HISR |= 1 << 11;
            break;
        case 6:
            s->DMA_HISR |= 1 << 21;
            break;
        case 7:
            s->DMA_HISR |= 1 << 27;
            break;

    }

    stm32_dma_update_irq(s, stream);
}

static uint32_t stm32_dma_DMA_SxCR_read(stm32_dma_state *s, stm32_dma_stream_state *stream)
{
    uint32_t value =
            (stream->direction << DMA_SxCR_DIR_START) |
            (stream->DMA_SxCR_TCIE << DMA_SxCR_TCIE_BIT) |
            (stream->DMA_SxCR_TEIE << DMA_SxCR_TEIE_BIT) |
            (stream->DMA_SxCR_DMEIE << DMA_SxCR_DMEIE_BIT) |
            (stream->DMA_SxCR_PINC << DMA_SxCR_PINC_BIT) |
            (stream->DMA_SxCR_MINC << DMA_SxCR_MINC_BIT) |
            (stream->DMA_SxCR_PSIZE << DMA_SxCR_PSIZE_START) |
            (stream->DMA_SxCR_MSIZE << DMA_SxCR_MSIZE_START) |
            (stream->priority << DMA_SxCR_PL_START) |
            (stream->enabled);
    if(stream->disableOnCRRead)
    {
        stream->disableOnCRRead = false;
        stream->enabled = false;
    }
    return value;
}

static void stm32_dma_DMA_SxCR_write(stm32_dma_state *s, stm32_dma_stream_state *stream, uint32_t value, bool init)
{
    bool new_enabled = extract32(value, DMA_SxCR_EN_BIT, 1);
    if(!stream->enabled && new_enabled)
    {
        stream->enabled = new_enabled;
        stm32_dma_process_tx(s, stream);
    }
    stream->direction = extract32(value, DMA_SxCR_DIR_START, DMA_SxCR_DIR_LENGTH);
    if(stream->direction != 0x1 && !init)
        DPRINT("WARNING: Direction != Memory-to-peripheral\n");

    stream->DMA_SxCR_TCIE = extract32(value, DMA_SxCR_TCIE_BIT, 1);
    stream->DMA_SxCR_TEIE = extract32(value, DMA_SxCR_TEIE_BIT, 1);
    stream->DMA_SxCR_DMEIE = extract32(value, DMA_SxCR_DMEIE_BIT, 1);

    stream->DMA_SxCR_PINC = extract32(value, DMA_SxCR_PINC_BIT, 1);
    stream->DMA_SxCR_MINC = extract32(value, DMA_SxCR_MINC_BIT, 1);

    stream->DMA_SxCR_PSIZE = extract32(value, DMA_SxCR_PSIZE_START, DMA_SxCR_PSIZE_LENGTH);
    stream->DMA_SxCR_MSIZE = extract32(value, DMA_SxCR_MSIZE_START, DMA_SxCR_MSIZE_LENGTH);

    stream->priority = extract32(value, DMA_SxCR_PL_START, DMA_SxCR_PL_LENGTH);
}

static uint32_t stm32_dma_stream_read(stm32_dma_state *s, hwaddr addr)
{
    uint32_t value = 0;
    uint32_t offset = (addr - DMA_STREAM_START) % DMA_STREAM_LENGTH;
    uint32_t stream_num = (addr - DMA_STREAM_START - offset) / DMA_STREAM_LENGTH;
    stm32_dma_stream_state *stream = &s->stream[stream_num];

    switch(offset)
    {
        case DMA_SxCR_OFFSET:
            value = stm32_dma_DMA_SxCR_read(s, stream);
            break;
        case DMA_SxNDTR_OFFSET:
            value = stream->size;
            break;
        case DMA_SxPAR_OFFSET:
            value = stream->DMA_SxPAR;
            break;
        case DMA_SxM0AR_OFFSET:
            value = stream->DMA_SxM0AR;
            break;
        case DMA_SxM1AR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        case DMA_SxFCR_OFFSET:
            value = (4 << DMA_SxFCR_FS_START) | stream->DMA_SxFCR_FTH | stream->DMA_SxFCR_FEIE << DMA_SxFCR_FEIE_BIT;
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
    DPRINT("Read from Stream %d, register %s [0x%02X] with value 0x%08X\n", stream_num, stm32_dma_register_name(offset), offset, value);
    return value;
}


static void stm32_dma_stream_write(stm32_dma_state *s, hwaddr addr, uint32_t value)
{
    uint32_t offset = (addr - DMA_STREAM_START) % DMA_STREAM_LENGTH;
    uint32_t stream_num = (addr - DMA_STREAM_START - offset) / DMA_STREAM_LENGTH;
    stm32_dma_stream_state *stream = &s->stream[stream_num];

    DPRINT("Write to Stream %d, register %s [0x%02X] with value 0x%08X\n", stream_num, stm32_dma_register_name(offset), offset, value);

    switch(offset)
    {
        case DMA_SxCR_OFFSET:
            stm32_dma_DMA_SxCR_write(s, stream, value, false);
            break;
        case DMA_SxNDTR_OFFSET:
            stream->size = value & 0xFFFF;
            break;
        case DMA_SxPAR_OFFSET:
            stream->DMA_SxPAR = value;
            break;
        case DMA_SxM0AR_OFFSET:
            stream->DMA_SxM0AR = value;
            break;
        case DMA_SxM1AR_OFFSET:
            if(value != 0)
                STM32_NOT_IMPL_REG(offset, 4);
            break;
        case DMA_SxFCR_OFFSET:
            stream->DMA_SxFCR_FTH = extract32(value, DMA_SxFCR_FTH_START, DMA_SxFCR_FTH_LENGTH);
            stream->DMA_SxFCR_FEIE = extract32(value, DMA_SxFCR_FEIE_BIT, 1);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}


static uint64_t stm32_dma_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    struct stm32_dma_state *s = (struct stm32_dma_state *) opaque;
    uint32_t value=0;

    assert(size == 4);

    switch (addr) {
        case DMA_LISR_OFFSET:
            value = s->DMA_LISR;
            break;
        case DMA_HISR_OFFSET:
            value = s->DMA_HISR;
            break;
        case DMA_LIFCR_OFFSET:
        case DMA_HIFCR_OFFSET:
            STM32_WARN_WO_REG(addr);
            break;
        case 0x10 ... 0xCC:
            value = stm32_dma_stream_read(s, addr);
            break;
        default:
           STM32_BAD_REG(addr, 4);
            break;
    }

    return value;
}

static void stm32_dma_write(void *opaque, hwaddr addr,
                           uint64_t value, unsigned size)
{
    struct stm32_dma_state *s = (struct stm32_dma_state *) opaque;

    assert(size == 4);

    switch (addr) {
        case DMA_LISR_OFFSET:
        case DMA_HISR_OFFSET:
            STM32_WARN_RO_REG(addr);
            break;
        case DMA_LIFCR_OFFSET:
            s->DMA_LISR &= ~(value & 0xF7D0F7D);
            break;
        case DMA_HIFCR_OFFSET:
            s->DMA_HISR &= ~(value & 0xF7D0F7D);
            break;
        case 0x10 ... 0xCC:
            stm32_dma_stream_write(s, addr, value);
            break;
        default:
           STM32_BAD_REG(addr, 4);
            break;
    }
}

static const MemoryRegionOps stm32_dma_ops = {
    .read = stm32_dma_read,
    .write = stm32_dma_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


/* XXX: this won't be needed once soc_dma knows about clocks.  */
/*static void stm32_dma_clk_update(void *opaque, int line, int on)
{
    struct stm32_dma_state *s = (struct stm32_dma_state *) opaque;
    int i;

    s->dma->freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
}*/

static void stm32_dma_reset(DeviceState *dev)
{
    stm32_dma_state *s = STM32_DMA_DEVICE(dev);
    s->DMA_LISR = 0x0;
    s->DMA_HISR = 0x0;
}


static int stm32_dma_init(SysBusDevice *dev)
{
    stm32_dma_state *s = STM32_DMA_DEVICE(dev);
    int i;
    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, NULL, &stm32_dma_ops, s,
                          "dma", 0x0400);
    sysbus_init_mmio(dev, &s->iomem);


    for(i=0; i<NUM_STREAMS; i++)
    {
        s->stream[i].num = i;
        sysbus_init_irq(dev, &s->stream[i].irq);
    }
    return 0;
}


static Property stm32_dma_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", stm32_dma_state, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", stm32_dma_state, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_dma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_dma_init;
    dc->reset = stm32_dma_reset;
    dc->props = stm32_dma_properties;
}

static TypeInfo stm32_dma_info = {
    .name  = "stm32-dma",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(stm32_dma_state),
    .class_init = stm32_dma_class_init
};

static void stm32_dma_register_types(void)
{
    type_register_static(&stm32_dma_info);
}

type_init(stm32_dma_register_types)



