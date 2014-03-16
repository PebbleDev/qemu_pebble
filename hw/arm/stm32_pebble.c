/*
 * Pebble SmartWatch STM32 emulator
 * Copyright (C) 2013 Jens Andersen <jens.andersen@gmail.com>
 * Based on:
 * Olimex STM32 P103 Development Board
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
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

#include "hw/arm/stm32.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "ui/keymaps.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/ssi.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "hw/bt.h"

typedef struct
{
    qemu_irq outputs[4];
    bool prev_state[4];
} KbdState;

static void stm32_pebble_button_event(void *opaque, int idx, bool up)
{
    KbdState *s = (KbdState*)opaque;
    qemu_irq irq;
    irq = s->outputs[idx];
    if(s->prev_state[idx] == up)
        return;
    if(up)
        qemu_set_irq(irq, 0);
    else
        qemu_set_irq(irq, 1);
    s->prev_state[idx] = up;
}

static void stm32_pebble_key_event(void *opaque, int keycode)
{
    bool up = !!(keycode & SCANCODE_UP);
    int idx;
    switch(keycode & SCANCODE_KEYCODEMASK)
    {
        case 0x4B: // left
            idx = 0;
            break;
        case 0x48: // up
            idx = 1;
            break;
        case 0x4D: // right
            idx = 2;
            break;
        case 0x50: // down
            idx = 3;
            break;
        case 0x60:
            return;
        default:
            printf("Pebble: Unknown key pressed %c (0x%X)\n", keycode & SCANCODE_KEYCODEMASK, keycode );
            return;
    }

    stm32_pebble_button_event(opaque, idx, up);
}

static void stm32_pebble_init(QEMUMachineInitArgs *args)
{
    const char* kernel_filename = args->kernel_filename;

    stm32_init(/*flash_size*/512,
               /*ram_size*/512,
               kernel_filename,
               16000000,
               32768);

    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_b = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));
    DeviceState *gpio_h = DEVICE(object_resolve_path("/machine/stm32/gpio[h]", NULL));
    DeviceState *uart1 = DEVICE(object_resolve_path("/machine/stm32/uart[1]", NULL));
    DeviceState *uart3 = DEVICE(object_resolve_path("/machine/stm32/uart[3]", NULL));
    DeviceState *spi1 = DEVICE(object_resolve_path("/machine/stm32/spi[0]", NULL));
    DeviceState *spi2 = DEVICE(object_resolve_path("/machine/stm32/spi[1]", NULL));
    DeviceState *i2c1 = DEVICE(object_resolve_path("/machine/stm32/i2c[0]", NULL));
    DeviceState *i2c2 = DEVICE(object_resolve_path("/machine/stm32/i2c[1]", NULL));
    assert(gpio_a);
    assert(gpio_b);
    assert(gpio_c);
    assert(uart3);
    assert(spi1);
    assert(spi2);
    assert(i2c1);
    assert(i2c2);

    /* Connect RS232 to UART */
    stm32_uart_connect(
            (Stm32Uart *)uart3,
            serial_hds[0]);

    CharDriverState *radio = uart_cc256x_hci_init(NULL);
    stm32_uart_connect(
            (Stm32Uart *)uart1,
            radio);
    qdev_connect_gpio_out(gpio_h, 0, cc256xhci_pins_get(radio)[cc256xhci_pin_nshutdown]);
    //qdev_connect_gpio_out(uart1, 1, qdev_get_gpio_in(gpio_a, 11));
    sysbus_connect_irq(SYS_BUS_DEVICE(uart1), 1, qdev_get_gpio_in(gpio_a, 11));
    SysBusDevice *spibusdev = SYS_BUS_DEVICE(spi1);
    SSIBus *spibus = (SSIBus *)qdev_get_child_bus(spi1, "spi");
    assert(spibus);
    assert(spibusdev);

    DeviceState *flash_dev = ssi_create_slave(spibus, "n25q032a11");
    assert(flash_dev);

    qemu_irq cs_line = qdev_get_gpio_in(flash_dev, 0);
    qdev_connect_gpio_out(gpio_a, 4, cs_line);

    SysBusDevice *spibusdev2 = SYS_BUS_DEVICE(spi2);
    SSIBus *spibus2 = (SSIBus *)qdev_get_child_bus(spi2, "spi");
    assert(spibus2);
    assert(spibusdev2);

    DeviceState *lcd_dev = ssi_create_slave(spibus2, "ls01x_lcd");
    assert(lcd_dev);
    qemu_irq lcd_cs_line = qdev_get_gpio_in(lcd_dev, 0);
    qdev_connect_gpio_out(gpio_b, 12, lcd_cs_line);
    i2c_bus *i2cbus1 = (i2c_bus *)qdev_get_child_bus(i2c1, "i2c");
    i2c_create_slave(i2cbus1, "lis3dh", 0x19);
    i2c_create_slave(i2cbus1, "unki2c", 0x18);

    i2c_bus *i2cbus2 = (i2c_bus *)qdev_get_child_bus(i2c2, "i2c");
    i2c_create_slave(i2cbus2, "mag3110", 0xE);

    // For some reason pebble requires this pin High
    qemu_irq gpioa_2 = qdev_get_gpio_in(gpio_a, 2);
    qemu_set_irq(gpioa_2, 1);
/*
    qemu_irq gpioc_3 = qdev_get_gpio_in(gpio_c, 3);
    qemu_set_irq(gpioc_3, 1);*/

    KbdState* kbdstate = malloc(sizeof(KbdState));
    // Back
    kbdstate->outputs[0] = qemu_irq_invert(qdev_get_gpio_in(gpio_c, 3));
    // Up
    kbdstate->outputs[1] = qemu_irq_invert(qdev_get_gpio_in(gpio_a, 2));
    // Middle/Enter
    kbdstate->outputs[2] = qemu_irq_invert(qdev_get_gpio_in(gpio_c, 6));
    // Down
    kbdstate->outputs[3] = qemu_irq_invert(qdev_get_gpio_in(gpio_a, 1));

/*    kbdstate->outputs[0] = qdev_get_gpio_in(gpio_c, 3);
    kbdstate->outputs[1] = qdev_get_gpio_in(gpio_c, 6);
    kbdstate->outputs[2] = qdev_get_gpio_in(gpio_c, 11);
    kbdstate->outputs[3] = qdev_get_gpio_in(gpio_c, 12);*/
/*    qemu_irq_raise(kbdstate->outputs[0]);
    qemu_irq_raise(kbdstate->outputs[1]);
    qemu_irq_raise(kbdstate->outputs[2]);*/
    if(1)
    qemu_add_kbd_event_handler(stm32_pebble_key_event, kbdstate);
 }

static QEMUMachine stm32_pebble_machine = {
    .name = "stm32-pebble",
    .desc = "STM32 Pebble SmartWatch",
    .init = stm32_pebble_init,
    .block_default_type = IF_MTD,
    .max_cpus = 1,
    .no_sdcard = 1,
};


static void stm32_pebble_machine_init(void)
{
    qemu_register_machine(&stm32_pebble_machine);
}

machine_init(stm32_pebble_machine_init);
