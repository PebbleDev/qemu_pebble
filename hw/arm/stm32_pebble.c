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
#include "hw/irq.h"

typedef struct
{
    qemu_irq outputs[4];
} KbdState;

static void stm32_pebble_key_event(void *opaque, int keycode)
{
    KbdState *s = (KbdState*)opaque;
    qemu_irq irq;
    switch(keycode & SCANCODE_KEYCODEMASK)
    {
        case 2:
        case 3:
        case 4:
        case 5:
            irq = s->outputs[(keycode & SCANCODE_KEYCODEMASK) - 2];
            if(keycode & SCANCODE_UP)
                qemu_set_irq(irq, 0);
            else
                qemu_set_irq(irq, 1);
            break;

        default:
            printf("Pebble: Unknown key pressed %c (0x%X)\n", keycode & SCANCODE_KEYCODEMASK, keycode );
            break;
    }
}

static void stm32_pebble_init(QEMUMachineInitArgs *args)
{
    const char* kernel_filename = args->kernel_filename;

    stm32_init(/*flash_size*/0x0007ffff,
               /*ram_size*/0x0001ffff,
               kernel_filename,
               16000000,
               32768);

    DeviceState *gpio_a = DEVICE(object_resolve_path("/machine/stm32/gpio[a]", NULL));
    DeviceState *gpio_b = DEVICE(object_resolve_path("/machine/stm32/gpio[b]", NULL));
    DeviceState *gpio_c = DEVICE(object_resolve_path("/machine/stm32/gpio[c]", NULL));
    DeviceState *uart3 = DEVICE(object_resolve_path("/machine/stm32/uart[3]", NULL));
    DeviceState *spi1 = DEVICE(object_resolve_path("/machine/stm32/spi[0]", NULL));
    DeviceState *spi2 = DEVICE(object_resolve_path("/machine/stm32/spi[1]", NULL));
    assert(gpio_a);
    assert(gpio_b);
    assert(gpio_c);
    assert(uart3);
    assert(spi1);
    assert(spi2);

    /* Connect RS232 to UART */
    stm32_uart_connect(
            (Stm32Uart *)uart3,
            serial_hds[0]);

    SysBusDevice *spibusdev = SYS_BUS_DEVICE(spi1);
    SSIBus *spibus = (SSIBus *)qdev_get_child_bus(spi1, "spi");
    assert(spibus);
    assert(spibusdev);

    DeviceState *flash_dev = ssi_create_slave(spibus, "n25q032a");
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

    // For some reason pebble requires this pin High
/*    qemu_irq gpioa_2 = qdev_get_gpio_in(gpio_a, 2);
    qemu_set_irq(gpioa_2, 1);*/
/*
    qemu_irq gpioc_3 = qdev_get_gpio_in(gpio_c, 3);
    qemu_set_irq(gpioc_3, 1);*/

    KbdState* kbdstate = malloc(sizeof(KbdState));
    kbdstate->outputs[0] = qdev_get_gpio_in(gpio_c, 3);
    kbdstate->outputs[1] = qdev_get_gpio_in(gpio_c, 6);
    kbdstate->outputs[2] = qdev_get_gpio_in(gpio_c, 11);
    kbdstate->outputs[3] = qdev_get_gpio_in(gpio_c, 12);

    qemu_add_kbd_event_handler(stm32_pebble_key_event, kbdstate);
 }

static QEMUMachine stm32_pebble_machine = {
    .name = "stm32-pebble",
    .desc = "STM32 Pebble SmartWatch",
    .init = stm32_pebble_init,
    DEFAULT_MACHINE_OPTIONS,
};


static void stm32_pebble_machine_init(void)
{
    qemu_register_machine(&stm32_pebble_machine);
}

machine_init(stm32_pebble_machine_init);
