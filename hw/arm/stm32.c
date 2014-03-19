/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 *
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

#include "hw/arm/stm32.h"
#include "exec/address-spaces.h"
#include "exec/gdbstub.h"
#include <time.h>

/* DEFINITIONS */
#define STM32_FLASH_ADDR_START 0x8000000
/* COMMON */

void stm32_hw_warn(const char *fmt, ...)
{
    va_list ap;
/*    CPUArchState *env;
    CPUState *cpu;*/

    va_start(ap, fmt);
    fprintf(stderr, "qemu stm32: hardware warning: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    cpu_dump_state(first_cpu, stderr, fprintf, 0);

/*    for(env = first_cpu; env != NULL; env = env->next_cpu) {
        cpu = ENV_GET_CPU(env);
        fprintf(stderr, "CPU #%d:\n", cpu_index(cpu));
    }*/
    va_end(ap);
}

int stm32_get_pc(void)
{
    ARMCPU *cpu = ARM_CPU(first_cpu);
    CPUARMState *env = &cpu->env;

    return env->regs[15];
}

static char buffer[32];

// NOTE: This is obviously non-reentrant, but should be OK.
const char *stm32_get_timestamp(void)
{
    struct timeval tval;
    gettimeofday(&tval, NULL);
    struct tm *ptm = localtime(&tval.tv_sec);
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d.%3d", ptm->tm_hour, ptm->tm_min, ptm->tm_sec, (unsigned int)(tval.tv_usec / 1000));
    return buffer;
}



/* PERIPHERALS */

const char *stm32_periph_name_arr[] = {
    [STM32_RCC] = "RCC",
    [STM32_GPIOA] = "GPIOA",
    [STM32_GPIOB] = "GPIOB",
    [STM32_GPIOC] = "GPIOC",
    [STM32_GPIOD] = "GPIOD",
    [STM32_GPIOE] = "GPIOE",
    [STM32_GPIOF] = "GPIOF",
    [STM32_GPIOG] = "GPIOG",
    [STM32_GPIOH] =  "GPIOH",
    [STM32_GPIOI] = "GPIOI",
    [STM32_UART1] = "UART1",
    [STM32_UART2] =  "UART2",
    [STM32_UART3] = "UART3",
    [STM32_UART4] = "UART4",
    [STM32_UART5] = "UART5",
    [STM32_UART6] = "UART6",
    [STM32_ADC1] = "ADC1",
    [STM32_ADC2] = "ADC2",
    [STM32_ADC3] = "ADC3",
    [STM32_DAC] = "DAC",
    [STM32_TIM1] = "TIM1",
    [STM32_TIM2] = "TIM2",
    [STM32_TIM3] = "TIM3",
    [STM32_TIM4] = "TIM4",
    [STM32_TIM5] = "TIM5",
    [STM32_TIM6] = "TIM6",
    [STM32_TIM7] = "TIM7",
    [STM32_TIM8] = "TIM8",
    [STM32_TIM9] = "TIM9",
    [STM32_TIM10] = "TIM10",
    [STM32_TIM11] = "TIM11",
    [STM32_TIM12] = "TIM12",
    [STM32_TIM13] = "TIM13",
    [STM32_TIM14] = "TIM14",
    [STM32_PWR] = "PWR",
    [STM32_I2C1] = "I2C1",
    [STM32_I2C2] = "I2C2",
    [STM32_I2C3] = "I2C3",
    [STM32_I2S1] = "I2S1",
    [STM32_I2S2] = "I2S2",
    [STM32_WWDG] = "WWDG",
    [STM32_IWDG] = "IWDG",
    [STM32_CAN1] = "CAN1",
    [STM32_CAN2] = "CAN2",
    [STM32_CAN] = "CAN",
    [STM32_USB] = "USB",
    [STM32_SPI1] = "SPI1",
    [STM32_SPI2] = "SPI2",
    [STM32_SPI3] = "SPI3",
    [STM32_DMA1] = "DMA1",
    [STM32_DMA2] = "DMA2",
    [STM32_EXTI] = "EXTI",
    [STM32_SDIO] = "SDIO",
    [STM32_FSMC] = "FSMC",
    [STM32_SYSCFG] = "SYSCFG",
    [STM32_FLASH] = "FLASH",
    [STM32_RTC] = "RTC",
    [STM32_CRC] = "CRC",
};

const char *stm32_periph_name(stm32_periph_t periph)
{
    assert(periph < STM32_PERIPH_COUNT);
    if(periph == STM32_PERIPH_UNDEFINED)
        return "UNDEFINED";
    return stm32_periph_name_arr[periph];
}





/* INITIALIZATION */

/* I copied sysbus_create_varargs and split it into two parts.  This is so that
 * you can set properties before calling the device init function.
 */

static DeviceState *stm32_init_periph(DeviceState *dev, stm32_periph_t periph,
                                        hwaddr addr, qemu_irq irq)
{
    qdev_init_nofail(dev);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);
    if (irq) {
        sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);
    }
    return dev;
}


static void stm32_create_fake_device(Object *stm32_container,
        stm32_periph_t periph, uint32_t offset, uint32_t size)
{
    DeviceState *fake_dev = qdev_create(NULL, "stm32-fake");
    qdev_prop_set_uint32(fake_dev, "size", size);
    QDEV_PROP_SET_PERIPH_T(fake_dev, "periph", periph);
    object_property_add_child(stm32_container, stm32_periph_name(periph), OBJECT(fake_dev), NULL);
    stm32_init_periph(fake_dev, periph, offset, NULL);
}


static void stm32_create_spi_dev(Object *stm32_container,
        stm32_periph_t periph, hwaddr offset, qemu_irq irq)
{
    char child_name[8];
    int spi_num = periph - STM32_SPI1;
    DeviceState *spi_dev = qdev_create(NULL, "stm32-spi");
    QDEV_PROP_SET_PERIPH_T(spi_dev, "periph", periph);
    snprintf(child_name, 8, "spi[%i]", spi_num);
    object_property_add_child(stm32_container, child_name, OBJECT(spi_dev), NULL);
    stm32_init_periph(spi_dev, periph, offset, irq);
}

static void stm32_create_i2c_dev(Object *stm32_container,
        stm32_periph_t periph, hwaddr offset, qemu_irq irq, qemu_irq irq2)
{
    char child_name[8];
    int i2c_num = periph - STM32_I2C1;
    DeviceState *i2c_dev = qdev_create(NULL, "stm32-i2c");
    QDEV_PROP_SET_PERIPH_T(i2c_dev, "periph", periph);
    snprintf(child_name, 8, "i2c[%i]", i2c_num);
    object_property_add_child(stm32_container, child_name, OBJECT(i2c_dev), NULL);
    stm32_init_periph(i2c_dev, periph, offset, NULL);
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c_dev), 0, irq);
    sysbus_connect_irq(SYS_BUS_DEVICE(i2c_dev), 1, irq2);
}


static void stm32_create_uart_dev(
        Object *stm32_container,
        stm32_periph_t periph,
        int uart_num,
        DeviceState *rcc_dev,
        DeviceState **gpio_dev,
        hwaddr addr,
        qemu_irq irq)
{
    char child_name[8];
    DeviceState *uart_dev = qdev_create(NULL, "stm32-uart");
    QDEV_PROP_SET_PERIPH_T(uart_dev, "periph", periph);
    qdev_prop_set_ptr(uart_dev, "stm32_rcc", rcc_dev);
    qdev_prop_set_ptr(uart_dev, "stm32_gpio", gpio_dev);
    snprintf(child_name, sizeof(child_name), "uart[%i]", uart_num);
    object_property_add_child(stm32_container, child_name, OBJECT(uart_dev), NULL);
    stm32_init_periph(uart_dev, periph, addr, irq);
}

static void stm32_create_timer25_dev(Object *stm32_container,
        stm32_periph_t periph,
        uint8_t timer_num, DeviceState *rcc_dev,
        hwaddr addr, qemu_irq irq)
{
    char child_name[8];
    DeviceState *tim_dev = qdev_create(NULL, "stm32-tim25");
    QDEV_PROP_SET_PERIPH_T(tim_dev, "periph", periph);
    qdev_prop_set_ptr(tim_dev, "stm32_rcc", rcc_dev);
    snprintf(child_name, sizeof(child_name), "tim[%i]", timer_num);
    object_property_add_child(stm32_container, child_name, OBJECT(tim_dev), NULL);
    stm32_init_periph(tim_dev, periph, addr, irq);

}

static void stm32_create_dma_dev(
        Object *stm32_container,
        stm32_periph_t periph,
        DeviceState *rcc_dev,
        hwaddr addr,
        qemu_irq irq0,
        qemu_irq irq1,
        qemu_irq irq2,
        qemu_irq irq3,
        qemu_irq irq4,
        qemu_irq irq5,
        qemu_irq irq6,
        qemu_irq irq7)
{
    char child_name[8];
    DeviceState *dma_dev = qdev_create(NULL, "stm32-dma");
    QDEV_PROP_SET_PERIPH_T(dma_dev, "periph", periph);
    qdev_prop_set_ptr(dma_dev, "stm32_rcc", rcc_dev);
    object_property_add_child(stm32_container, child_name, OBJECT(dma_dev), NULL);
    stm32_init_periph(dma_dev, periph, addr, NULL);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 0, irq0);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 1, irq1);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 2, irq2);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 3, irq3);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 4, irq4);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 5, irq5);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 6, irq6);
    sysbus_connect_irq(SYS_BUS_DEVICE(dma_dev), 7, irq7);
}

static uint64_t kernel_load_translate_fn(void *opaque, uint64_t from_addr) {
    if (from_addr == STM32_FLASH_ADDR_START) {
        return 0x00000000;
    }
    return from_addr;
}

qemu_irq *stm32_init(
            ram_addr_t flash_size,
            ram_addr_t ram_size,
            const char *kernel_filename,
            uint32_t osc_freq,
            uint32_t osc32_freq)
{
    MemoryRegion *address_space_mem = get_system_memory();
//    MemoryRegion *flash_alias_mem = g_malloc(sizeof(MemoryRegion));
    char *otpbacking = g_malloc(sizeof(char)*0x220);
    MemoryRegion *otp_mem = g_malloc(sizeof(MemoryRegion));
    qemu_irq *pic;
    int i;

    // OTP Bytes
    memset(otpbacking, 0xff, 0x210);
    memset(&otpbacking[0x210], 0, 0x10); // Option bytes
    strcpy(&otpbacking[0], "31M147B4C8EF");
    strcpy(&otpbacking[0x20], "V2R2");
    memset(&otpbacking[0x200], 0, 2);

    Object *stm32_container = container_get(qdev_get_machine(), "/stm32");

    pic = armv7m_translated_init(stm32_container, address_space_mem, flash_size, ram_size, kernel_filename, kernel_load_translate_fn, NULL, "cortex-m3");

/*    pic = armv7m_init(
              stm32_container,
              address_space_mem,
              flash_size,
              ram_size,
              kernel_filename,
              "cortex-m3");*/

    /* The STM32 family stores its Flash memory at some base address in memory
     * (0x08000000 for medium density devices), and then aliases it to the
     * boot memory space, which starts at 0x00000000 (the "System Memory" can also
     * be aliased to 0x00000000, but this is not implemented here). The processor
     * executes the code in the aliased memory at 0x00000000.  We need to make a
     * QEMU alias so that reads in the 0x08000000 area are passed through to the
     * 0x00000000 area. Note that this is the opposite of real hardware, where the
     * memory at 0x00000000 passes reads through the "real" flash memory at
     * 0x08000000, but it works the same either way. */
    /* TODO: Parameterize the base address of the aliased memory. */


    MemoryRegionSection mrs = memory_region_find(address_space_mem, 0, 4);
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(flash_alias, NULL,  "stm32f2xx.flash.alias", mrs.mr, 0, flash_size * 1024);
    memory_region_add_subregion(address_space_mem, STM32_FLASH_ADDR_START, flash_alias);

/*    memory_region_init_alias(
            flash_alias_mem,
            NULL,
            "stm32-flash-alias-mem",
            address_space_mem,
            0,
            flash_size);
    memory_region_add_subregion(address_space_mem, 0x08000000, flash_alias_mem);*/

    memory_region_init_ram_ptr(otp_mem, NULL, "stm32-flash-otp-mem", 0x220, otpbacking);
    memory_region_set_readonly(otp_mem, true);
    memory_region_add_subregion(address_space_mem, 0x1fff7800, otp_mem);

    DeviceState *rcc_dev = qdev_create(NULL, "stm32-rcc");
    qdev_prop_set_uint32(rcc_dev, "osc_freq", osc_freq);
    qdev_prop_set_uint32(rcc_dev, "osc32_freq", osc32_freq);
    object_property_add_child(stm32_container, "rcc", OBJECT(rcc_dev), NULL);
    stm32_init_periph(rcc_dev, STM32_RCC, 0x40023800, pic[STM32_RCC_IRQ]);

    DeviceState **gpio_dev = (DeviceState **)g_malloc0(sizeof(DeviceState *) * STM32_GPIO_COUNT);
    for(i = 0; i < STM32_GPIO_COUNT; i++) {
        char child_name[8];
        stm32_periph_t periph = STM32_GPIOA + i;
        gpio_dev[i] = qdev_create(NULL, TYPE_STM32_GPIO);
        QDEV_PROP_SET_PERIPH_T(gpio_dev[i], "periph", periph);
        qdev_prop_set_ptr(gpio_dev[i], "stm32_rcc", rcc_dev);
        snprintf(child_name, sizeof(child_name), "gpio[%c]", 'a' + i);
        object_property_add_child(stm32_container, child_name, OBJECT(gpio_dev[i]), NULL);
        stm32_init_periph(gpio_dev[i], periph, 0x40020000 + (i * 0x400), NULL);
    }

    DeviceState *exti_dev = qdev_create(NULL, TYPE_STM32_EXTI);
    object_property_add_child(stm32_container, "exti", OBJECT(exti_dev), NULL);
    stm32_init_periph(exti_dev, STM32_EXTI, 0x40013c00, NULL);
    SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);
    sysbus_connect_irq(exti_busdev, 0, pic[STM32_EXTI0_IRQ]);
    sysbus_connect_irq(exti_busdev, 1, pic[STM32_EXTI1_IRQ]);
    sysbus_connect_irq(exti_busdev, 2, pic[STM32_EXTI2_IRQ]);
    sysbus_connect_irq(exti_busdev, 3, pic[STM32_EXTI3_IRQ]);
    sysbus_connect_irq(exti_busdev, 4, pic[STM32_EXTI4_IRQ]);
    sysbus_connect_irq(exti_busdev, 5, pic[STM32_EXTI9_5_IRQ]);
    sysbus_connect_irq(exti_busdev, 6, pic[STM32_EXTI15_10_IRQ]);
    sysbus_connect_irq(exti_busdev, 7, pic[STM32_PVD_IRQ]);
    sysbus_connect_irq(exti_busdev, 8, pic[STM32_RTCAlarm_IRQ]);
    sysbus_connect_irq(exti_busdev, 9, pic[STM32_OTG_FS_WKUP_IRQ]);
    sysbus_connect_irq(exti_busdev, 10, pic[STM32_ETH_WKUP_IRQ]);
    sysbus_connect_irq(exti_busdev, 11, pic[STM32_OTG_HS_WKUP_IRQ]);
    sysbus_connect_irq(exti_busdev, 12, pic[STM32_TAMP_STAMP_IRQ]);
    sysbus_connect_irq(exti_busdev, 13, pic[STM32_RTC_WKUP_IRQ]);


    DeviceState *rtc_dev = qdev_create(NULL, "stm32-rtc");
    object_property_add_child(stm32_container, "rtc", OBJECT(rtc_dev), NULL);
    qdev_prop_set_ptr(rtc_dev, "stm32_rcc", rcc_dev);
    QDEV_PROP_SET_PERIPH_T(rtc_dev, "periph", STM32_RTC);
    stm32_init_periph(rtc_dev, STM32_RTC, 0x40002800, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 0, pic[STM32_RTCAlarm_IRQ]);
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 1, pic[STM32_RTC_WKUP_IRQ]);
    sysbus_connect_irq(SYS_BUS_DEVICE(rtc_dev), 2, pic[STM32_TAMP_STAMP_IRQ]);

    stm32_create_uart_dev(stm32_container, STM32_UART1, 1, rcc_dev, gpio_dev, 0x40011000, pic[STM32_UART1_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART2, 2, rcc_dev, gpio_dev, 0x40004400, pic[STM32_UART2_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART3, 3, rcc_dev, gpio_dev, 0x40004800, pic[STM32_UART3_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART4, 4, rcc_dev, gpio_dev, 0x40004c00, pic[STM32_UART4_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART5, 5, rcc_dev, gpio_dev, 0x40005000, pic[STM32_UART5_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART6, 6, rcc_dev, gpio_dev, 0x40011400, pic[STM32_UART6_IRQ]);

    stm32_create_spi_dev(stm32_container, STM32_SPI1, 0x40013000, pic[STM32_SPI1_IRQ]);
    stm32_create_spi_dev(stm32_container, STM32_SPI2, 0x40003800, pic[STM32_SPI2_IRQ]);
    stm32_create_spi_dev(stm32_container, STM32_SPI3, 0x40003c00, pic[STM32_SPI3_IRQ]);

    DeviceState *syscfg_dev = qdev_create(NULL, "stm32-syscfg");
    object_property_add_child(stm32_container, "syscfg", OBJECT(syscfg_dev), NULL);
    stm32_init_periph(syscfg_dev, STM32_SYSCFG, 0x40013800, NULL);

    stm32_create_fake_device(stm32_container, STM32_WWDG, 0x40002c00, 0x400);
    stm32_create_fake_device(stm32_container, STM32_IWDG, 0x40003000, 0x400);

    stm32_create_fake_device(stm32_container, STM32_FLASH, 0x40023C00, 0x400);

    stm32_create_dma_dev(stm32_container, STM32_DMA1, rcc_dev, 0x40026000,
                         pic[STM32_DMA1_STREAM0_IRQ],
                         pic[STM32_DMA1_STREAM1_IRQ],
                         pic[STM32_DMA1_STREAM2_IRQ],
                         pic[STM32_DMA1_STREAM3_IRQ],
                         pic[STM32_DMA1_STREAM4_IRQ],
                         pic[STM32_DMA1_STREAM5_IRQ],
                         pic[STM32_DMA1_STREAM6_IRQ],
                         pic[STM32_DMA1_STREAM7_IRQ]);

    stm32_create_dma_dev(stm32_container, STM32_DMA2, rcc_dev, 0x40026400,
                         pic[STM32_DMA2_STREAM0_IRQ],
                         pic[STM32_DMA2_STREAM1_IRQ],
                         pic[STM32_DMA2_STREAM2_IRQ],
                         pic[STM32_DMA2_STREAM3_IRQ],
                         pic[STM32_DMA2_STREAM4_IRQ],
                         pic[STM32_DMA2_STREAM5_IRQ],
                         pic[STM32_DMA2_STREAM6_IRQ],
                         pic[STM32_DMA2_STREAM7_IRQ]);

    stm32_create_fake_device(stm32_container, STM32_PWR, 0x40007000, 0x400);

    stm32_create_i2c_dev(stm32_container, STM32_I2C1, 0x40005400, pic[STM32_I2C1_EV_IRQ], pic[STM32_I2C1_ER_IRQ]);
    stm32_create_i2c_dev(stm32_container, STM32_I2C2, 0x40005800, pic[STM32_I2C2_EV_IRQ], pic[STM32_I2C2_ER_IRQ]);
    stm32_create_i2c_dev(stm32_container, STM32_I2C3, 0x40005c00, pic[STM32_I2C3_EV_IRQ], pic[STM32_I2C3_ER_IRQ]);

    // stm32_create_fake_device(stm32_container, STM32_ADC1, 0x40012000, 0x400);
    DeviceState *adc_dev = qdev_create(NULL, "stm32-adc");
    object_property_add_child(stm32_container, "adc", OBJECT(adc_dev), NULL);
    stm32_init_periph(adc_dev, STM32_ADC1, 0x40012000, NULL);





    DeviceState *crc_dev = qdev_create(NULL, "stm32-crc");
    object_property_add_child(stm32_container, "crc", OBJECT(crc_dev), NULL);
    stm32_init_periph(crc_dev, STM32_CRC, 0x40023000, NULL);



    stm32_create_timer25_dev(stm32_container, STM32_TIM2, 2, rcc_dev, 0x40000000, pic[STM32_TIM2_IRQ]);
    stm32_create_timer25_dev(stm32_container, STM32_TIM3, 3, rcc_dev, 0x40000400, pic[STM32_TIM3_IRQ]);
    stm32_create_timer25_dev(stm32_container, STM32_TIM4, 4, rcc_dev, 0x40000800, pic[STM32_TIM4_IRQ]);
    stm32_create_timer25_dev(stm32_container, STM32_TIM5, 5, rcc_dev, 0x40000c00, pic[STM32_TIM5_IRQ]);

    stm32_create_fake_device(stm32_container, STM32_TIM1, 0x40010000, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM6, 0x40001000, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM7, 0x40001400, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM12, 0x40001800, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM13, 0x40001c00, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM14, 0x40002000, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM8, 0x40010400, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM9, 0x40014000, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM10, 0x40014400, 0x400);
    stm32_create_fake_device(stm32_container, STM32_TIM11, 0x40014800, 0x400);

    return pic;
}
