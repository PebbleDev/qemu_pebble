/*
 * STM32 Microcontroller GPIO (General Purpose I/O) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on pl061.c
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

#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "qemu/bitops.h"

/* See README for DEBUG details. */
#define DEBUG_STM32_GPIO

#ifdef DEBUG_STM32_GPIO
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_GPIO: " fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif



/* DEFINITIONS*/

#define GPIOx_MODER_OFFSET   0x00
#define GPIOx_OTYPER_OFFSET  0x04
#define GPIOx_OSPEEDR_OFFSET 0x08
#define GPIOx_PUPDR_OFFSET   0x0c
#define GPIOx_IDR_OFFSET     0x10
#define GPIOx_ODR_OFFSET     0x14
#define GPIOx_BSRR_OFFSET    0x18
#define GPIOx_LCKR_OFFSET    0x1c
#define GPIOx_AFRL_OFFSET    0x20
#define GPIOx_AFRH_OFFSET    0x24

struct Stm32Gpio {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;

    uint32_t GPIOx_MODER;
    uint32_t GPIOx_ODR;
    uint32_t GPIOx_AFRL;
    uint32_t GPIOx_AFRH;

    uint16_t in;

    /* IRQs used to communicate with the machine implementation.
     * There is one IRQ for each pin.  Note that for pins configured
     * as inputs, the output IRQ state has no meaning.  Perhaps
     * the output should be updated to match the input in this case....
     */
    qemu_irq out_irq[STM32_GPIO_PIN_COUNT];

    /* IRQs which relay input pin changes to other STM32 peripherals */
    qemu_irq in_irq[STM32_GPIO_PIN_COUNT];
};



/* CALLBACKs */

/* Trigger fired when a GPIO input pin changes state (based
 * on an external stimulus from the machine).
 */
static void stm32_gpio_in_trigger(void *opaque, int irq, int level)
{
    Stm32Gpio *s = opaque;
    unsigned pin = irq;

    assert(pin < STM32_GPIO_PIN_COUNT);

    /* Update internal pin state. */
    s->in |= (level ? 1 : 0) << pin;

    /* Propagate the trigger to the input IRQs. */
    qemu_set_irq(s->in_irq[pin], level);
}



/* HELPER FUNCTIONS */

/* Gets the four configuration bits for the pin from the CRL or CRH
 * register.
 */
static uint8_t stm32_gpio_get_pin_config(Stm32Gpio *s, unsigned pin) {
    /* Simplify extract logic by combining both 32 bit regiters into
     * one 64 bit value.
     */
/*    uint64_t cr_64 = ((uint64_t)s->GPIOx_CRy[1] << 32) |
                      s->GPIOx_CRy[0];
    return extract64(cr_64, pin * 4, 4);*/
    // TODO: Trying to get pin config
    return 0;
}





/* REGISTER IMPLEMENTATION */

/* Write the Output Data Register.
 * Propagates the changes to the output IRQs.
 * Perhaps we should also update the input to match the output for
 * pins configured as outputs... */
static void stm32_gpio_GPIOx_ODR_write(Stm32Gpio *s, uint32_t new_value)
{
    uint32_t old_value;
    uint16_t changed, changed_out;
    unsigned pin;

    old_value = s->GPIOx_ODR;

    /* Update register value.  Per documentation, the upper 16 bits
     * always read as 0. */
    s->GPIOx_ODR = new_value & 0x0000ffff;

    /* Get pins that changed value */
    changed = old_value ^ new_value;

    /* Get changed pins that are outputs - we will not touch input pins */
    changed_out = changed & 0xffffff;// & s->dir_mask;

    if (changed_out) {
        for (pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
            /* If the value of this pin has changed, then update
             * the output IRQ.
             */
            if (changed_out & BIT(pin)) {
                qemu_set_irq(
                        DEVICE(s)->gpio_out[pin],
                        (s->GPIOx_ODR & BIT(pin)) ? 1 : 0);
            }
        }
    }
}

static void stm32_gpio_GPIOx_MODER_write(Stm32Gpio *s, uint32_t new_value)
{
    uint32_t old_value, mask;
    uint16_t changed;
    unsigned pin;

    old_value = s->GPIOx_MODER;
    /* Get pins that changed value */
    changed = old_value ^ new_value;

    for (pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        // TODO: DO something with the changed values
        mask = 0xff << pin;
        if((changed & mask) != 0)
        {
            // Mode is changed!
        }
        changed += 1;
    }
    s->GPIOx_MODER = new_value;
}

static uint64_t stm32_gpio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    assert(size == 4);

    switch (offset) {
        case GPIOx_MODER_OFFSET:
            return s->GPIOx_MODER;
        case GPIOx_OTYPER_OFFSET:
        case GPIOx_OSPEEDR_OFFSET:
        case GPIOx_PUPDR_OFFSET:
            /* We ignore writes to these for now as they are not required
               for emulation, probably */
            STM32_WARN_WO_REG(offset);
            return 0;
        case GPIOx_IDR_OFFSET:
            return s->in;
        case GPIOx_ODR_OFFSET:
            return s->GPIOx_ODR;
        case GPIOx_BSRR_OFFSET:
            STM32_WARN_WO_REG(offset);
            return 0;
        case GPIOx_LCKR_OFFSET:
            STM32_WARN_WO_REG(offset);
            /* Locking is not yet implemented */
            return 0;
        case GPIOx_AFRL_OFFSET:
            return s->GPIOx_AFRL;
        case GPIOx_AFRH_OFFSET:
            return s->GPIOx_AFRH;
        default:
            STM32_BAD_REG(offset, size);
            return 0;
    }
}

static void stm32_gpio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    uint32_t set_mask, reset_mask;
    Stm32Gpio *s = (Stm32Gpio *)opaque;

    assert(size == 4);

    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);
    DPRINTF("(Periph: %d) Write to 0x%X with value = %" PRIu64 "\n", s->periph, (unsigned int)offset, value);
    switch (offset) {
        case GPIOx_MODER_OFFSET:
            stm32_gpio_GPIOx_MODER_write(s, value);
            break;
        case GPIOx_OTYPER_OFFSET:
        case GPIOx_OSPEEDR_OFFSET:
        case GPIOx_PUPDR_OFFSET:
            STM32_WARN_RO_REG(offset);
            break;
        case GPIOx_IDR_OFFSET:
            STM32_WARN_RO_REG(offset);
            break;
        case GPIOx_ODR_OFFSET:
            stm32_gpio_GPIOx_ODR_write(s, value);
            break;
        case GPIOx_BSRR_OFFSET:
            /* Setting a bit sets or resets the corresponding bit in the output
             * register.  The lower 16 bits perform resets, and the upper 16
             * bits perform sets.  Register is write-only and so does not need
             * to store a value.  Sets take priority over resets, so we do
             * resets first.
             */
            set_mask = value & 0x0000ffff;
            reset_mask = ~(value >> 16) & 0x0000ffff;
            stm32_gpio_GPIOx_ODR_write(s,
                    (s->GPIOx_ODR & reset_mask) | set_mask);
            break;
        case GPIOx_AFRL_OFFSET:
            s->GPIOx_AFRL = value;
            break;
        case GPIOx_AFRH_OFFSET:
            s->GPIOx_AFRH = value;
            break;
        case GPIOx_LCKR_OFFSET:
            /* Locking is not implemented */
            STM32_NOT_IMPL_REG(offset, size);
            break;
        default:
            STM32_BAD_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_gpio_ops = {
    .read = stm32_gpio_read,
    .write = stm32_gpio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_gpio_reset(DeviceState *dev)
{
    int pin;
    Stm32Gpio *s = STM32_GPIO(dev);

    /* 0xa8000000 for Port A
     * 0x00000280 for port B
     * 0x00000000 for other */
    switch(s->periph)
    {
        case STM32_GPIOA:
            stm32_gpio_GPIOx_MODER_write(s, 0xa8000000);
            break;
        case STM32_GPIOB:
            stm32_gpio_GPIOx_MODER_write(s, 0x00000280);
            break;
        default:
           stm32_gpio_GPIOx_MODER_write(s, 0x00000000);
           break;
    }

    s->GPIOx_ODR = 0;
    s->GPIOx_AFRL = s->GPIOx_AFRH = 0;

    for(pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        qemu_irq_lower(s->out_irq[pin]);
    }

    /* Leave input state as it is - only outputs and config are affected
     * by the GPIO reset. */
}






/* PUBLIC FUNCTIONS */

uint8_t stm32_gpio_get_config_bits(Stm32Gpio *s, unsigned pin) {
    return (stm32_gpio_get_pin_config(s, pin) >> 2) & 0x3;
}

uint8_t stm32_gpio_get_mode_bits(Stm32Gpio *s, unsigned pin) {
    return stm32_gpio_get_pin_config(s, pin) & 0x3;
}






/* DEVICE INITIALIZATION */

static int stm32_gpio_init(SysBusDevice *dev)
{
    unsigned pin;
    Stm32Gpio *s = FROM_SYSBUS(Stm32Gpio, dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, &stm32_gpio_ops, s,
                          "gpio", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    qdev_init_gpio_in(&dev->qdev, stm32_gpio_in_trigger, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(&dev->qdev, s->out_irq, STM32_GPIO_PIN_COUNT);

    for(pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        sysbus_init_irq(dev, &s->in_irq[pin]);
    }

    return 0;
}

static Property stm32_gpio_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Gpio, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", Stm32Gpio, stm32_rcc_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_gpio_init;
    dc->reset = stm32_gpio_reset;
    dc->props = stm32_gpio_properties;
}

static TypeInfo stm32_gpio_info = {
    .name  = TYPE_STM32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Gpio),
    .class_init = stm32_gpio_class_init
};

static void stm32_gpio_register_types(void)
{
    type_register_static(&stm32_gpio_info);
}

type_init(stm32_gpio_register_types)
