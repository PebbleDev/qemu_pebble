/*
 *  STM32 I2C Emulation
 *  Copyright (C) 2014 Jens Andersen <jens.andersen@gmail.com>
 *  Based in part on:
 *  Exynos4210 I2C Bus Serial Interface Emulation
 *
 *  Copyright (C) 2012 Samsung Electronics Co Ltd.
 *    Maksim Kozlov, <m.kozlov@samsung.com>
 *    Igor Mitsyanko, <i.mitsyanko@samsung.com>
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

#include "qemu/timer.h"
#include "qemu/bitops.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/arm/stm32.h"

#ifndef STM32_I2C_DEBUG
#define STM32_I2C_DEBUG                 1
#endif

#define TYPE_STM32_I2C                  "stm32-i2c"
#define STM32_I2C(obj)                  \
    OBJECT_CHECK(Stm32I2CState, (obj), TYPE_STM32_I2C)

/* Stm32 I2C memory map */
#define STM32_I2C_MEM_SIZE              0x400

#define I2C_CR1_OFFSET                  0x0
#define I2C_CR1_SWRST_BIT 15
#define I2C_CR1_ALERT_BIT 13
#define I2C_CR1_PEC_BIT 12
#define I2C_CR1_POS_BIT 11
#define I2C_CR1_ACK_BIT 10
#define I2C_CR1_STOP_BIT 9
#define I2C_CR1_START_BIT 8
#define I2C_CR1_NOSTRETCH_BIT 7
#define I2C_CR1_ENGC_BIT 6
#define I2C_CR1_ENPEC_BIT 5
#define I2C_CR1_ENARP_BIT 4
#define I2C_CR1_SMBTYPE_BIT 3
#define I2C_CR1_SMBUS_BIT 1
#define I2C_CR1_PE_BIT 0

#define I2C_CR2_OFFSET 0x4
#define I2C_CR2_LAST_BIT 12
#define I2C_CR2_DMAEN_BIT 11
#define I2C_CR2_ITBUFEN_BIT 10
#define I2C_CR2_ITEVTEN_BIT 9
#define I2C_CR2_ITERREN_BIT 8
#define I2C_CR2_FREQ_START 0
#define I2C_CR2_FREQ_LENGTH 6

#define I2C_OAR1_OFFSET 0x8
#define I2C_OAR1_ADDMODE_BIT 15
#define I2C_OAR1_ADD2_START 8
#define I2C_OAR1_ADD2_LENGTH 2
#define I2C_OAR1_ADD_START 1
#define I2C_OAR1_ADD_LENGTH 7
#define I2C_OAR1_ADD0_BIT 0

#define I2C_OAR2_OFFSET 0x0C
#define I2C_OAR2_ADD2_START 0
#define I2C_OAR2_ADD2_LENGTH 8

#define I2C_DR_OFFSET 0x10

#define I2C_SR1_OFFSET 0x14
#define I2C_SR1_SMBALERT_BIT 15
#define I2C_SR1_TIMEOUT_BIT 14
#define I2C_SR1_PECERR_BIT 12
#define I2C_SR1_OVR_BIT 11
#define I2C_SR1_AF_BIT 10
#define I2C_SR1_ARLO_BIT 9
#define I2C_SR1_BERR_BIT 8
#define I2C_SR1_TXE_BIT 7
#define I2C_SR1_RXNE_BIT 6
#define I2C_SR1_STOPF_BIT 5
#define I2C_SR1_ADD10_BIT 3
#define I2C_SR1_BTF_BIT 2
#define I2C_SR1_ADDR_BIT 1
#define I2C_SR1_SB_BIT 0
#define I2C_CR2_ITEVTEN_IRQ_MASK (I2C_SR1_SB_BIT | I2C_SR1_ADDR_BIT | I2C_SR1_ADD10_BIT | I2C_SR1_STOPF_BIT | I2C_SR1_BTF_BIT)
#define I2C_CR2_ITEVTEN_ITBUFEN_IRQ_MASK (I2C_SR1_RXNE_BIT | I2C_SR1_TXE_BIT)
#define I2C_CR2_IERREN_IRQ_MASK (I2C_SR1_BERR_BIT | I2C_SR1_ARLO_BIT | I2C_SR1_AF_BIT | I2C_SR1_OVR_BIT | I2C_SR1_PECERR_BIT | I2C_SR1_TIMEOUT_BIT | I2C_SR1_SMBALERT_BIT)


#define I2C_SR2_OFFSET 0X18
#define I2C_SR2_PEC_START 8
#define I2C_SR2_PEC_LENGTH 8
#define I2C_SR2_DUALF_BIT 7
#define I2C_SR2_SMBHOST_BIT 6
#define I2C_SR2_SMBDEFAULT_BIT 5
#define I2C_SR2_GENCALL_BIT 4
#define I2C_SR2_TRA_BIT 2
#define I2C_SR2_BUSY_BIT 1
#define I2C_SR2_MSL_BIT 0

#define I2C_CCR_OFFSET 0x1C
#define I2C_CCR_FS_BIT 15
#define I2C_CCR_DUTY 14
#define I2C_CCR_CCR_START 0
#define I2C_CCR_CCR_LENGTH 12

#define I2C_TRISE_OFFSET 0x20
#define I2C_TRISE_TRISE_START 0x0
#define I2C_TRISE_TRISE_LENGTH 6


#if STM32_I2C_DEBUG
#define DPRINT(fmt, args...)              \
    do { fprintf(stderr, "STM32_I2C: "fmt, ## args); } while (0)

static const char *stm32_i2c_get_regname(unsigned offset)
{
    switch (offset) {
    case I2C_CR1_OFFSET:
        return "I2C_CR1";
    case I2C_CR2_OFFSET:
        return "I2C_CR2";
    case I2C_OAR1_OFFSET:
        return "I2C_OAR1";
    case I2C_OAR2_OFFSET:
        return "I2C_OAR2";
    case I2C_DR_OFFSET:
        return "I2C_DR";
    case I2C_SR1_OFFSET:
        return "I2C_SR1";
    case I2C_SR2_OFFSET:
        return "I2C_SR2";
    case I2C_CCR_OFFSET:
        return "I2C_CCR";
    case I2C_TRISE_OFFSET:
        return "I2C_TRISE";
    default:
        return "[UNKNOWN]";
    }
}

#else
#define DPRINT(fmt, args...)              do { } while (0)
#endif

enum I2CState
{
    I2C_NONE=-1,
    I2C_SB_SET=0,
    I2C_NACKED_ADDR,
    I2C_WAIT_ADDRESS,
    I2C_TRANSMIT_BYTES,
};

typedef struct Stm32I2CState {
    SysBusDevice busdev;
    MemoryRegion iomem;
    i2c_bus *bus;
    qemu_irq irq[2];
    uint8_t addr;
    stm32_periph_t periph;
    bool enabled;
    enum I2CState state;
    uint16_t
        I2C_CR1,
        I2C_CR2,
        I2C_SR1,
        I2C_SR2,
        I2C_CCR,
        I2C_TRISE;

    bool scl_free;
} Stm32I2CState;

static inline void stm32_i2c_raise_interrupt(Stm32I2CState *s)
{
    uint8_t raise_event = 0, raise_error = 0, val;

    if (s->I2C_CR2 & (I2C_CR2_ITEVTEN_BIT) )
    {
        val = s->I2C_SR1 & I2C_CR2_ITEVTEN_IRQ_MASK;
        if(val)
            raise_event = 1;
    }
    if (s->I2C_CR2 & (I2C_CR2_ITEVTEN_BIT | I2C_CR2_ITBUFEN_BIT))
    {
        val = s->I2C_SR1 & I2C_CR2_ITEVTEN_ITBUFEN_IRQ_MASK;
        if(val)
            raise_event = 1;
    }
    if (s->I2C_CR2 & I2C_CR2_ITERREN_BIT)
    {
        val = s->I2C_SR1 & I2C_CR2_IERREN_IRQ_MASK;
        if(val)
            raise_error = 1;
    }

    if(raise_event)
        qemu_irq_raise(s->irq[0]);
    if(raise_error)
        qemu_irq_raise(s->irq[1]);
}

static void stm32_i2c_data_receive(void *opaque)
{
    Stm32I2CState *s = (Stm32I2CState *)opaque;
//    int ret;

    s->scl_free = false;
/*    s->i2cstat &= ~I2CSTAT_LAST_BIT;
    ret = i2c_recv(s->bus);
    if (ret < 0 && (s->i2ccon & I2CCON_ACK_GEN)) {
        s->i2cstat |= I2CSTAT_LAST_BIT;  // Data is not acknowledged
    } else {
        s->data = ret;
    }*/
    stm32_i2c_raise_interrupt(s);
}

static void stm32_i2c_data_send(void *opaque)
{
    Stm32I2CState *s = (Stm32I2CState *)opaque;

/*    s->I2C_SR1 |
    s->i2cstat &= ~I2CSTAT_LAST_BIT;
    s->scl_free = false;
    if (i2c_send(s->bus, s->i2cds) < 0 && (s->i2ccon & I2CCON_ACK_GEN)) {
        s->i2cstat |= I2CSTAT_LAST_BIT;
    }*/
    stm32_i2c_raise_interrupt(s);
}


static void stm32_i2c_I2C_DR_write(Stm32I2CState *s, uint32_t new_value)
{
    switch(s->state)
    {
        case I2C_WAIT_ADDRESS:
        {
            uint32_t addr = extract32(new_value, 1, 7);
            uint32_t recv = extract32(new_value, 0, 1);
            if(addr != s->addr)
            {
                DPRINT("Address not matching!?\n");
            }
            if(i2c_start_transfer(s->bus, addr, recv))
            {
                DPRINT("Not Acked by slave!? WADTODO!?\n");
                s->state = I2C_NACKED_ADDR;
            }
            else
            {
                s->I2C_SR1 |= I2C_SR1_TXE_BIT & I2C_SR1_ADDR_BIT;
                s->I2C_SR1 &= ~I2C_SR1_SB_BIT;
                s->I2C_SR2 |= (!recv) << I2C_SR2_TRA_BIT;

                s->state = I2C_TRANSMIT_BYTES;
            }
        }
        break;
        case I2C_TRANSMIT_BYTES:
            if(i2c_send(s->bus, new_value & 0xff))
            {
                DPRINT("i2c_send == True\n");
            }
            break;
        default:
            hw_error("Unsupported I2C state when writing DR!?\n");
            break;
    }
    stm32_i2c_raise_interrupt(s);
}
static void stm32_i2c_I2C_CR1_write(Stm32I2CState *s, uint32_t new_value)
{
    uint32_t tmpval;

    tmpval = extract32(new_value, I2C_CR1_PE_BIT, 1);
    if(!s->enabled && tmpval)
    {
    }
    s->enabled = tmpval;
    tmpval = extract32(new_value, I2C_CR1_START_BIT, 1);
    if(tmpval)
    {
        DPRINT("Start bit set, Waiting for address!\n");
        s->I2C_SR1 |= I2C_SR1_SB_BIT;
        s->I2C_SR2 |= I2C_SR2_MSL_BIT;
        s->state = I2C_WAIT_ADDRESS;
    }
//    s->I2C_CR1 = new_value & 0xFFFF;
    if(extract32(new_value, I2C_CR1_SWRST_BIT, 1))
    {
        DPRINT("Reset due to failure\n");
    }

    s->I2C_CR1 = new_value & 0xFFFF;
    if(0)
    {
        stm32_i2c_data_send(s);
        stm32_i2c_data_receive(s);
    }
    stm32_i2c_raise_interrupt(s);
}

static void stm32_i2c_I2C_OAR1_write(Stm32I2CState *s, uint32_t new_value)
{
    uint8_t tmpval;

    tmpval = extract32(new_value, I2C_OAR1_ADDMODE_BIT, 1);
    if(tmpval)
    {
        hw_error("STM32_I2C: 10-bit addressing not implemented! \n");
    }
    tmpval = extract32(new_value, I2C_OAR1_ADD_START, I2C_OAR1_ADD_LENGTH);
    if(tmpval)
    {
        s->addr = tmpval;
    }

}


static uint64_t stm32_i2c_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    Stm32I2CState *s = (Stm32I2CState *)opaque;
    uint8_t value;

//    assert(size == 2);

    switch (offset) {
    case I2C_CR1_OFFSET:
        value = s->I2C_CR1;
        break;
    case I2C_CR2_OFFSET:
        value = s->I2C_CR2;
        break;
    case I2C_OAR1_OFFSET:
        value = s->addr << I2C_OAR1_ADD_START;
        break;
    case I2C_OAR2_OFFSET:
        STM32_NOT_IMPL_REG(offset, size);
        break;
    case I2C_DR_OFFSET:
/*        value = s->i2cds;
        s->scl_free = true;
        if (STM32_I2C_MODE(s->i2cstat) == I2CMODE_MASTER_Rx &&
               (s->i2cstat & I2CSTAT_START_BUSY) &&
               !(s->i2ccon & I2CCON_INT_PEND)) {
            stm32_i2c_data_receive(s);
        }*/
        break;
    case I2C_SR1_OFFSET:
        value = s->I2C_SR1;
        break;
    case I2C_SR2_OFFSET:
        value = s->I2C_SR2;
        break;
    case I2C_CCR_OFFSET:
        value = s->I2C_CCR;
        break;
    case I2C_TRISE_OFFSET:
        value = s->I2C_TRISE;
        break;
    default:
        value = 0;
        DPRINT("ERROR: Bad read offset 0x%x\n", (unsigned int)offset);
        break;
    }

    DPRINT("read %s [0x%02x] -> 0x%02x\n", stm32_i2c_get_regname(offset),
            (unsigned int)offset, value);
    return value;
}

static void stm32_i2c_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    Stm32I2CState *s = (Stm32I2CState *)opaque;
    uint16_t v = value & 0xffff;

    assert(size == 2);
    DPRINT("write %s [0x%02x] <- 0x%02x\n", stm32_i2c_get_regname(offset),
            (unsigned int)offset, v);

    switch (offset) {

    case I2C_CR1_OFFSET:
        stm32_i2c_I2C_CR1_write(s, v);
        break;
    case I2C_CR2_OFFSET:
        // 
        s->I2C_CR2 = v;
        DPRINT("Setting frequency to %u\n", extract32(v, I2C_CR2_FREQ_START, I2C_CR2_FREQ_LENGTH));

        break;
    case I2C_OAR1_OFFSET:
        stm32_i2c_I2C_OAR1_write(s, v);
        break;
    case I2C_OAR2_OFFSET:
        STM32_NOT_IMPL_REG(offset, size);
        break;
    case I2C_DR_OFFSET:
        stm32_i2c_I2C_DR_write(s, v);
        break;
    case I2C_SR1_OFFSET:
        break;
    case I2C_SR2_OFFSET:
        break;
    case I2C_CCR_OFFSET:
        s->I2C_CCR = v;
        break;
    case I2C_TRISE_OFFSET:
        s->I2C_TRISE = v;
        break;

/*
    case I2CCON_ADDR:
        s->i2ccon = (v & ~I2CCON_INT_PEND) | (s->i2ccon & I2CCON_INT_PEND);
        if ((s->i2ccon & I2CCON_INT_PEND) && !(v & I2CCON_INT_PEND)) {
            s->i2ccon &= ~I2CCON_INT_PEND;
            qemu_irq_lower(s->irq);
            if (!(s->i2ccon & I2CCON_INTRS_EN)) {
                s->i2cstat &= ~I2CSTAT_START_BUSY;
            }

            if (s->i2cstat & I2CSTAT_START_BUSY) {
                if (s->scl_free) {
                    if (STM32_I2C_MODE(s->i2cstat) == I2CMODE_MASTER_Tx) {
                        stm32_i2c_data_send(s);
                    } else if (STM32_I2C_MODE(s->i2cstat) ==
                            I2CMODE_MASTER_Rx) {
                        stm32_i2c_data_receive(s);
                    }
                } else {
                    s->i2ccon |= I2CCON_INT_PEND;
                    qemu_irq_raise(s->irq);
                }
            }
        }
        break;
    case I2CSTAT_ADDR:
        s->i2cstat =
                (s->i2cstat & I2CSTAT_START_BUSY) | (v & ~I2CSTAT_START_BUSY);

        if (!(s->i2cstat & I2CSTAT_OUTPUT_EN)) {
            s->i2cstat &= ~I2CSTAT_START_BUSY;
            s->scl_free = true;
            qemu_irq_lower(s->irq);
            break;
        }

        // Nothing to do if in i2c slave mode 
        if (!I2C_IN_MASTER_MODE(s->i2cstat)) {
            break;
        }

        if (v & I2CSTAT_START_BUSY) {
            s->i2cstat &= ~I2CSTAT_LAST_BIT;
            s->i2cstat |= I2CSTAT_START_BUSY;    // Line is busy
            s->scl_free = false;

            // Generate start bit and send slave address
            if (i2c_start_transfer(s->bus, s->i2cds >> 1, s->i2cds & 0x1) &&
                    (s->i2ccon & I2CCON_ACK_GEN)) {
                s->i2cstat |= I2CSTAT_LAST_BIT;
            } else if (STM32_I2C_MODE(s->i2cstat) == I2CMODE_MASTER_Rx) {
                stm32_i2c_data_receive(s);
            }
            stm32_i2c_raise_interrupt(s);
        } else {
            i2c_end_transfer(s->bus);
            if (!(s->i2ccon & I2CCON_INT_PEND)) {
                s->i2cstat &= ~I2CSTAT_START_BUSY;
            }
            s->scl_free = true;
        }
        break;
    case I2CADD_ADDR:
        if ((s->i2cstat & I2CSTAT_OUTPUT_EN) == 0) {
            s->i2cadd = v;
        }
        break;
    case I2CDS_ADDR:
        if (s->i2cstat & I2CSTAT_OUTPUT_EN) {
            s->i2cds = v;
            s->scl_free = true;
            if (STM32_I2C_MODE(s->i2cstat) == I2CMODE_MASTER_Tx &&
                    (s->i2cstat & I2CSTAT_START_BUSY) &&
                    !(s->i2ccon & I2CCON_INT_PEND)) {
                stm32_i2c_data_send(s);
            }
        }
        break;
    case I2CLC_ADDR:
        s->i2clc = v;
        break;
*/
    default:
        DPRINT("ERROR: Bad write offset 0x%x\n", (unsigned int)offset);
        break;
    }
}

static const MemoryRegionOps stm32_i2c_ops = {
    .read = stm32_i2c_read,
    .write = stm32_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription stm32_i2c_vmstate = {
    .name = TYPE_STM32_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(I2C_CR1, Stm32I2CState),
        VMSTATE_UINT16(I2C_CR2, Stm32I2CState),
        VMSTATE_UINT16(I2C_SR1, Stm32I2CState),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_i2c_reset(DeviceState *d)
{
    Stm32I2CState *s = STM32_I2C(d);

/*    s->i2ccon  = 0x00;
    s->i2cstat = 0x00;
    s->i2cds   = 0xFF;
    s->i2clc   = 0x00;
    s->i2cadd  = 0xFF;*/
    s->scl_free = true;
    s->state = I2C_NONE;
}

static Property stm32_i2c_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32I2CState, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};


static int stm32_i2c_realize(SysBusDevice *dev)
{
    Stm32I2CState *s = STM32_I2C(dev);

    memory_region_init_io(&s->iomem, &stm32_i2c_ops, s, TYPE_STM32_I2C,
                          STM32_I2C_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq[0]);
    sysbus_init_irq(dev, &s->irq[1]);
    s->bus = i2c_init_bus(&dev->qdev, "i2c");
    return 0;
}

static void stm32_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sbdc = SYS_BUS_DEVICE_CLASS(klass);

    dc->vmsd = &stm32_i2c_vmstate;
    dc->reset = stm32_i2c_reset;
    dc->props = stm32_i2c_properties;
    sbdc->init = stm32_i2c_realize;
}

static const TypeInfo stm32_i2c_type_info = {
    .name = TYPE_STM32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32I2CState),
    .class_init = stm32_i2c_class_init,
};

static void stm32_i2c_register_types(void)
{
    type_register_static(&stm32_i2c_type_info);
}

type_init(stm32_i2c_register_types)
