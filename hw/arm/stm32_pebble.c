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
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/ssi.h"
#include "hw/irq.h"

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
