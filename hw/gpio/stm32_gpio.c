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


#define TYPE_STM32_GPIO_DEVICE "stm32-gpio"
#define STM32_GPIO_DEVICE(obj) \
    OBJECT_CHECK(Stm32Gpio, (obj), TYPE_STM32_GPIO_DEVICE)

/* See README for DEBUG details. */
#define DEBUG_STM32_GPIO

#ifdef DEBUG_STM32_GPIO
#define DPRINT(fmt, ...)                                       \
    do { DPRINTF("STM32_GPIO", s->periph, fmt, ## __VA_ARGS__); } while (0)
#else
#define DPRINT(fmt, ...)
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

    uint16_t input;
    uint16_t output;
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

    DPRINT("Changed state of input-pin %u to %u\n", pin, !!level);
    assert(pin < STM32_GPIO_PIN_COUNT);

    /* Update internal pin state. */
    if(level)
        s->in |= BIT(pin);
    else
        s->in &= ~BIT(pin);

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
    uint64_t cr_64 = ((uint64_t)s->GPIOx_AFRL << 32) |
                      s->GPIOx_AFRH;
    return extract64(cr_64, pin * 4, 4);
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
    changed_out = changed & s->output;

    if (changed_out)
    {
        for (pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++)
        {
            /* If the value of this pin has changed, then update
             * the output IRQ.
             */
            if (changed_out & BIT(pin))
            {
                qemu_set_irq(
                        DEVICE(s)->gpio_out[pin],
                        (s->GPIOx_ODR & BIT(pin)) ? 1 : 0);
                DPRINT("%s: Pin %u set to %u\n", stm32_periph_name(s->periph), pin, (s->GPIOx_ODR & BIT(pin)) ? 1 : 0);
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

    for (pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++)
    {
        mask = 0x3 << (pin*2);
        if((old_value & mask) != (new_value & mask))
        {
            uint8_t val = (new_value & mask) >> (pin*2);
            const char *mode = NULL;
            switch(val)
            {
                case 0:
                    mode = "Input";
                    s->input |= BIT(pin);
                    s->output &= ~BIT(pin);
                    break;
                case 1:
                    mode = "Output";
                    s->input &= ~BIT(pin);
                    s->output |= BIT(pin);
                    break;
                case 2:
                    mode = "Alternate Function";
                    s->input &= ~BIT(pin);
                    s->output &= ~BIT(pin);
                    break;
                case 3:
                    mode = "Analog";
                    s->input &= ~BIT(pin);
                    s->output &= ~BIT(pin);
                    break;
                default:
                    DPRINT("Invalid mode!? 0x%X\n", val);
            }
            // Mode is changed!
            DPRINT("Port: %s Pin: %d changed to %s\n", stm32_periph_name(s->periph), pin, mode);

        }
        changed += 1;
    }
    s->GPIOx_MODER = new_value;
}

static void stm32_gpio_GPIOx_AFRx_write(Stm32Gpio *s, uint64_t new_value)
{
    int pin;
    for(pin=0; pin < STM32_GPIO_PIN_COUNT; pin++)
    {
//        int val = extract64(new_value, pin*4, 4);
/*        DPRINT("%s: Pin %u set to mode AF%u\n",
                stm32_periph_name(s->periph), pin, val);*/
    }
}

static uint64_t stm32_gpio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;
    uint32_t value;
    if(size != 4)
    {
        DPRINT("Warn: Gpio read of size %u from offset 0x%X\n", size, (uint32_t)offset);
    }

    switch (offset) {
        case GPIOx_MODER_OFFSET:
            value = s->GPIOx_MODER;
            break;
        case GPIOx_OTYPER_OFFSET:
        case GPIOx_OSPEEDR_OFFSET:
        case GPIOx_PUPDR_OFFSET:
            /* We ignore writes to these for now as they are not required
               for emulation, probably */
            STM32_WARN_WO_REG(offset);
            value = 0;
            break;
        case GPIOx_IDR_OFFSET:
            value = s->in;
            break;
        case GPIOx_ODR_OFFSET:
            value = s->GPIOx_ODR;
            break;
        case GPIOx_BSRR_OFFSET:
            STM32_WARN_WO_REG(offset);
            value = 0;
            break;
        case GPIOx_LCKR_OFFSET:
            STM32_WARN_WO_REG(offset);
            /* Locking is not yet implemented */
            value = 0;
            break;
        case GPIOx_AFRL_OFFSET:
            value = s->GPIOx_AFRL;
            break;
        case GPIOx_AFRH_OFFSET:
            value = s->GPIOx_AFRH;
            break;
        default:
            STM32_BAD_REG(offset, size);
            value = 0;
            break;
    }
    DPRINT("%s: Read of size %u from 0x%X with value = 0x%X\n", stm32_periph_name(s->periph), size, (uint32_t)offset, (uint32_t)value);

    return value;
}

static void stm32_gpio_writew(Stm32Gpio *s, hwaddr offset,
                       uint32_t value)
{
    uint32_t set_mask, reset_mask;
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
            stm32_gpio_GPIOx_AFRx_write(s, (uint64_t)s->GPIOx_AFRH << 32 | s->GPIOx_AFRL);
            break;
        case GPIOx_AFRH_OFFSET:
            s->GPIOx_AFRH = value;
            stm32_gpio_GPIOx_AFRx_write(s, (uint64_t)s->GPIOx_AFRH << 32 | s->GPIOx_AFRL);
            break;
        case GPIOx_LCKR_OFFSET:
            /* Locking is not implemented */
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}

static void stm32_gpio_writes(Stm32Gpio *s, hwaddr offset,
                       uint16_t value)
{
    uint32_t set_mask, reset_mask;

    switch(offset)
    {
        case GPIOx_BSRR_OFFSET:
            /* Setting a bit sets or resets the corresponding bit in the output
             * register.  The lower 16 bits perform resets, and the upper 16
             * bits perform sets.  Register is write-only and so does not need
             * to store a value.  Sets take priority over resets, so we do
             * resets first.
             */
            set_mask = value;
            stm32_gpio_GPIOx_ODR_write(s,
                    s->GPIOx_ODR | set_mask);
            break;
        case GPIOx_BSRR_OFFSET+2:
            /* Setting a bit sets or resets the corresponding bit in the output
             * register.  The lower 16 bits perform resets, and the upper 16
             * bits perform sets.  Register is write-only and so does not need
             * to store a value.  Sets take priority over resets, so we do
             * resets first.
             */
            reset_mask = ~(value) & 0x0000ffff;
            stm32_gpio_GPIOx_ODR_write(s,
                    s->GPIOx_ODR & reset_mask);
            break;
        default:
            STM32_BAD_REG(offset, 2);
            break;
    }
}

static void stm32_gpio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Gpio *s = (Stm32Gpio *)opaque;
    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);
    DPRINT("%s: Write of size %u to 0x%X with value = 0x%X\n", stm32_periph_name(s->periph), size, (uint32_t)offset, (uint32_t)value);
    switch(size)
    {
        case 2:
            stm32_gpio_writes(s, offset, (uint16_t)value);
            break;
        case 4:
            stm32_gpio_writew(s, offset, (uint32_t)value);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
    }
}


static const MemoryRegionOps stm32_gpio_ops = {
    .read = stm32_gpio_read,
    .write = stm32_gpio_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void stm32_gpio_reset(DeviceState *dev)
{
    int pin;
    Stm32Gpio *s = STM32_GPIO(dev);

    s->output = s->input = 0;
    s->GPIOx_MODER = 0;
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

static int stm32_gpio_init(SysBusDevice *sbd)
{
    unsigned pin;
    DeviceState *dev = DEVICE(sbd);
    Stm32Gpio *s = STM32_GPIO_DEVICE(dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;

    memory_region_init_io(&s->iomem, NULL, &stm32_gpio_ops, s,
                          "gpio", 0x03ff);
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(dev, stm32_gpio_in_trigger, STM32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(dev, s->out_irq, STM32_GPIO_PIN_COUNT);

    for(pin = 0; pin < STM32_GPIO_PIN_COUNT; pin++) {
        sysbus_init_irq(sbd, &s->in_irq[pin]);
    }
    s->in = 0;

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
