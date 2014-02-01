/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on omap_clk.c
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
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "hw/arm/stm32.h"
#include "hw/arm/stm32_clktree.h"
#include "qemu/bitops.h"
#include <stdio.h>


/* DEFINITIONS*/

/* See README for DEBUG details. */
#define DEBUG_STM32_RCC

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define HSI_FREQ 16000000
#define LSI_FREQ 32000

#define RCC_CR_OFFSET           0x00
#define RCC_CR_PLLI2SRDY_BIT    27
#define RCC_CR_PLLI2SON_BIT     26
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_CSSON_BIT        19
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_HSICAL_START     8
#define RCC_CR_HSICAL_MASK      0x0000ff00
#define RCC_CR_HSITRIM_START    3
#define RCC_CR_HSITRIM_MASK     0x000000f8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0

#define RCC_PLLCFGR_OFFSET      0x4
#define RCC_PLLCFGR_PLLQ_START  24
#define RCC_PLLCFGR_PLLQ_LENGTH 4
#define RCC_PLLCFGR_PLLQ_MASK   0x0f000000
#define RCC_PLLCFGR_PLLSRC_BIT  22
#define RCC_PLLCFGR_PLLP_START  16
#define RCC_PLLCFGR_PLLP_LENGTH 2
#define RCC_PLLCFGR_PLLP_MASK   0x00030000
#define RCC_PLLCFGR_PLLN_START  6
#define RCC_PLLCFGR_PLLN_LENGTH 9
#define RCC_PLLCFGR_PLLN_MASK   0x00007fc0
#define RCC_PLLCFGR_PLLM_START  0
#define RCC_PLLCFGR_PLLM_LENGTH 6
#define RCC_PLLCFGR_PLLM_MASK   0x0000003f

#define RCC_CFGR_OFFSET          0x08
#define RCC_CFGR_MCO2_START      30
#define RCC_CFGR_MCO2_MASK       0xc0000000
#define RCC_CFGR_MCO2PRE_START   27
#define RCC_CFGR_MCO2PRE_MASK    0x38000000
#define RCC_CFGR_MCO1PRE_START   24
#define RCC_CFGR_MCO1PRE_MASK    0x07000000
#define RCC_CFGR_I2SSRC_BIT      23
#define RCC_CFGR_MCO1_START      21
#define RCC_CFGR_MCO1_MASK       0x00500000
#define RCC_CFGR_MCO_CL_MASK     0x0f000000
#define RCC_CFGR_RTCPRE_START    16
#define RCC_CFGR_RTCPRE_MASK     0x001f0000
#define RCC_CFGR_PPRE2_START     13
#define RCC_CFGR_PPRE2_MASK      0x0000e000
#define RCC_CFGR_PPRE1_START     10
#define RCC_CFGR_PPRE1_MASK      0x00001c00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000f0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000c
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

#define RCC_CIR_OFFSET 0x0c

#define RCC_AHB1RSTR_OFFSET 0x10

#define RCC_AHB2RSTR_OFFSET 0x14

#define RCC_AHB3RSTR_OFFSET 0x18

#define RCC_APB1RSTR_OFFSET 0x20

#define RCC_APB2RSTR_OFFSET 0x24


#define RCC_AHB1ENR_OFFSET 0x30
#define RCC_AHB1ENR_GPIOIEN_BIT 8
#define RCC_AHB1ENR_GPIOHEN_BIT 7
#define RCC_AHB1ENR_GPIOGEN_BIT 6
#define RCC_AHB1ENR_GPIOFEN_BIT 5
#define RCC_AHB1ENR_GPIOEEN_BIT 4
#define RCC_AHB1ENR_GPIODEN_BIT 3
#define RCC_AHB1ENR_GPIOCEN_BIT 2
#define RCC_AHB1ENR_GPIOBEN_BIT 1
#define RCC_AHB1ENR_GPIOAEN_BIT 0

#define RCC_AHB2ENR_OFFSET 0x34

#define RCC_AHB3ENR_OFFSET 0x38
#define RCC_AHB3ENR_FSMCEN 0

#define RCC_APB2ENR_OFFSET 0x44
#define RCC_APB2ENR_TIM11EN_BIT   18
#define RCC_APB2ENR_TIM10EN_BIT   17
#define RCC_APB2ENR_TIM9EN_BIT   16
#define RCC_APB2ENR_SYSCFGEN_BIT   14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT   9
#define RCC_APB2ENR_ADC1EN_BIT   8
#define RCC_APB2ENR_USART6EN_BIT   5
#define RCC_APB2ENR_USART1EN_BIT   4
#define RCC_APB2ENR_TIM8EN_BIT   1
#define RCC_APB2ENR_TIM1EN_BIT   0

#define RCC_APB1ENR_OFFSET 0x40
#define RCC_APB1ENR_DACEN_BIT    29
#define RCC_APB1ENR_PWREN_BIT    28
#define RCC_APB1ENR_CAN2EN_BIT   26
#define RCC_APB1ENR_CAN1EN_BIT   25
#define RCC_APB1ENR_I2C3EN_BIT   23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_USART5EN_BIT 20
#define RCC_APB1ENR_USART4EN_BIT 19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM14EN_BIT   8
#define RCC_APB1ENR_TIM13EN_BIT   7
#define RCC_APB1ENR_TIM12EN_BIT   6
#define RCC_APB1ENR_TIM7EN_BIT   5
#define RCC_APB1ENR_TIM6EN_BIT   4
#define RCC_APB1ENR_TIM5EN_BIT   3
#define RCC_APB1ENR_TIM4EN_BIT   2
#define RCC_APB1ENR_TIM3EN_BIT   1
#define RCC_APB1ENR_TIM2EN_BIT   0

#define RCC_AHB1LPENR_OFFSET 0x50
#define RCC_AHB2LPENR_OFFSET 0x54
#define RCC_AHB3LPENR_OFFSET 0x58
#define RCC_APB1LPENR_OFFSET 0x60
#define RCC_APB2LPENR_OFFSET 0x64



#define RCC_BDCR_OFFSET 0x70
#define RCC_BDCR_BDRST_BIT 16
#define RCC_BDCR_RTCEN_BIT 15
#define RCC_BDCR_RTCSEL_START 8
#define RCC_BDCR_RTCSEL_MASK 0x00000300
#define RCC_BDCR_RTCSEL_LENGTH 2
#define RCC_BDCR_LSERDY_BIT 1
#define RCC_BDCR_LSEON_BIT 0

#define RCC_CSR_OFFSET       0x74
#define RCC_CSR_LPWRRSTF_BIT 31
#define RCC_CSR_WWDGRSTF_BIT 30
#define RCC_CSR_WDGRSTF_BIT  29
#define RCC_CSR_STFRSTF_BIT  28
#define RCC_CSR_PORRSTF_BIT  27
#define RCC_CSR_PADRSTF_BIT  26
#define RCC_CSR_BORRSTF_BIT  25
#define RCC_CSR_RMVF_BIT     24
#define RCC_CSR_LSIRDY_BIT   1
#define RCC_CSR_LSION_BIT    0

#define RCC_SSCGR_OFFSET 0x80
#define RCC_PLLI2SCFGR_OFFSET 0x84

#define PLLSRC_HSI_SELECTED 0
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

struct Stm32Rcc {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    uint32_t osc_freq;
    uint32_t osc32_freq;

    /* Private */
    MemoryRegion iomem;

    /* Register Values */
    uint32_t
        RCC_AHB1ENR,
        RCC_AHB2ENR,
        RCC_AHB3ENR,
        RCC_APB1ENR,
        RCC_APB2ENR,
        RCC_BDCR;


    /* Register Field Values */
    uint32_t
        RCC_PLLCFGR_PLLM,
        RCC_PLLCFGR_PLLN,
        RCC_PLLCFGR_PLLP,
        RCC_PLLCFGR_PLLQ,
        RCC_PLLCFGR_PLLSRC,
        RCC_CFGR_MCO2,
        RCC_CFGR_MCO2PRE,
        RCC_CFGR_MCO1PRE,
        RCC_CFGR_MCO1,
        RCC_CFGR_I2SSRC,
        RCC_CFGR_RTCPRE,
        RCC_CFGR_PPRE1,
        RCC_CFGR_PPRE2,
        RCC_CFGR_HPRE,
        RCC_CFGR_SWS,
        RCC_CFGR_SW,
        RCC_BDCR_RTCSEL;

    Clk
        HSICLK,
        HSECLK,
        LSECLK,
        LSICLK,
        SYSCLK,
        RTCCLK,
        PLL_VCO,
        PLLCLK,
        HCLK, /* Output from AHB Prescaler */
        PCLK1, /* Output from APB1 Prescaler */
        PCLK2, /* Output from APB2 Prescaler */
        PERIPHCLK[STM32_PERIPH_COUNT];

    qemu_irq irq;
};



/* HELPER FUNCTIONS */

/* Enable the peripheral clock if the specified bit is set in the value. */
static void stm32_rcc_periph_enable(
                    Stm32Rcc *s,
                    uint32_t new_value,
                    bool init,
                    int periph,
                    uint32_t bit_pos)
{
    if(s->PERIPHCLK[periph] == NULL)
    {
        stm32_hw_warn("Attempted to enable clock that isn't set: %s", stm32_periph_name(periph));
        return;
    }
    if(new_value & BIT(bit_pos))
    {
        if(!clktree_is_enabled(s->PERIPHCLK[periph]))
            DPRINTF("Enabling periphial %s\n", stm32_periph_name(periph));
    }
    else
    {
        if(clktree_is_enabled(s->PERIPHCLK[periph]))
            DPRINTF("Disabling periphial %s\n", stm32_periph_name(periph));
    }

    clktree_set_enabled(s->PERIPHCLK[periph], new_value & BIT(bit_pos));
}



static const char* stm32_rcc_registername(hwaddr offset)
{

    switch(offset)
    {
        case RCC_AHB1RSTR_OFFSET:
            return "RCC_AHB1RSTR";
        case RCC_AHB2RSTR_OFFSET:
            return "RCC_AHB2RSTR";
        case RCC_AHB3RSTR_OFFSET:
            return "RCC_AHB3RSTR";
        case RCC_APB1RSTR_OFFSET:
            return "RCC_APB1RSTR";
        case RCC_APB2RSTR_OFFSET:
            return "RCC_APB2RSTR";
        default:
            return "Unknown";
    }
}


/* REGISTER IMPLEMENTATION */

/* Read the configuration register. */
static uint32_t stm32_rcc_RCC_CR_read(Stm32Rcc *s)
{
    /* Get the status of the clocks. */
    int pllon_bit = clktree_is_enabled(s->PLLCLK) ? 1 : 0;
    int hseon_bit = clktree_is_enabled(s->HSECLK) ? 1 : 0;
    int hsion_bit = clktree_is_enabled(s->HSICLK) ? 1 : 0;

    /* build the register value based on the clock states.  If a clock is on,
     * then its ready bit is always set.
     */
    return pllon_bit << RCC_CR_PLLRDY_BIT |
           pllon_bit << RCC_CR_PLLON_BIT |
           hseon_bit << RCC_CR_HSERDY_BIT |
           hseon_bit << RCC_CR_HSEON_BIT |
           hsion_bit << RCC_CR_HSIRDY_BIT |
           hsion_bit << RCC_CR_HSION_BIT;
}

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    bool new_pllon, new_hseon, new_hsion;

    new_pllon = new_value & BIT(RCC_CR_PLLON_BIT);
    if((clktree_is_enabled(s->PLLCLK) && !new_pllon) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        stm32_hw_warn("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLL_VCO, new_pllon);
    clktree_set_enabled(s->PLLCLK, new_pllon);

    new_hseon = new_value & BIT(RCC_CR_HSEON_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_hseon) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED ||
        (s->RCC_CFGR_SW == SW_PLL_SELECTED && s->RCC_PLLCFGR_PLLSRC == PLLSRC_HSE_SELECTED)
       )
      ) {
        stm32_hw_warn("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSECLK, new_hseon);

    new_hsion = new_value & BIT(RCC_CR_HSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_hseon) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED ||
        (s->RCC_CFGR_SW == SW_PLL_SELECTED && s->RCC_PLLCFGR_PLLSRC == PLLSRC_HSI_SELECTED)
       )
      ) {
        stm32_hw_warn("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSICLK, new_hsion);
}


static uint32_t stm32_rcc_RCC_PLLCFGR_read(Stm32Rcc *s)
{
    return (s->RCC_PLLCFGR_PLLM << RCC_PLLCFGR_PLLM_START) |
           (s->RCC_PLLCFGR_PLLN << RCC_PLLCFGR_PLLN_START) |
           (s->RCC_PLLCFGR_PLLP << RCC_PLLCFGR_PLLP_START) |
           (s->RCC_PLLCFGR_PLLSRC << RCC_PLLCFGR_PLLSRC_BIT) |
           (s->RCC_PLLCFGR_PLLQ << RCC_PLLCFGR_PLLQ_START);
}

static void stm32_rcc_RCC_PLLCFGR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    uint32_t new_PLLSRC, new_PLLP, new_PLLN, new_PLLM;
    /* PLLSRC */
    new_PLLSRC = extract32(new_value, RCC_PLLCFGR_PLLSRC_BIT, 1);
    if(!init) {
        if(clktree_is_enabled(s->PLLCLK) &&
           (new_PLLSRC != s->RCC_PLLCFGR_PLLSRC)) {
            stm32_hw_warn("Can only change PLLSRC while PLL is disabled");
        }
    }
    clktree_set_selected_input(s->PLLCLK, new_PLLSRC);
    s->RCC_PLLCFGR_PLLSRC = new_PLLSRC;

    new_PLLP = extract32(new_value,
                           RCC_PLLCFGR_PLLP_START,
                           RCC_PLLCFGR_PLLP_LENGTH);
    if(!init) {
          if(clktree_is_enabled(s->PLLCLK) &&
           (new_PLLP != s->RCC_PLLCFGR_PLLP)) {
               stm32_hw_warn("Can only change PLLP while PLL is disabled");
          }
    }
    assert(new_PLLP <= 0xf);
    clktree_set_scale(s->PLLCLK, 1, 2 + 2 * new_PLLP);

    s->RCC_PLLCFGR_PLLP = new_PLLP;

    new_PLLM = extract32(new_value,
                           RCC_PLLCFGR_PLLM_START,
                           RCC_PLLCFGR_PLLM_LENGTH);
    new_PLLN = extract32(new_value,
                           RCC_PLLCFGR_PLLN_START,
                           RCC_PLLCFGR_PLLN_LENGTH);

    if(!init) {
          if(clktree_is_enabled(s->PLLCLK) &&
           (new_PLLM != s->RCC_PLLCFGR_PLLM || new_PLLN != s->RCC_PLLCFGR_PLLN)) {
               stm32_hw_warn("Can only change PLLM/N while PLL is disabled");
          }
    }
    if(new_PLLN < 64 || new_PLLN > 432 ||
       new_PLLM < 2) {
       stm32_hw_warn("Invalid PLLM (%u) or PLLN (%u) set", new_PLLM, new_PLLN);
    } else {
        clktree_set_scale(s->PLL_VCO, new_PLLN, new_PLLM);
    }


    s->RCC_PLLCFGR_PLLM = new_PLLM;
    s->RCC_PLLCFGR_PLLN = new_PLLN;
    DPRINTF("PLLP=%u, PLLM=%u, PLLN=%u\n", new_PLLP, new_PLLM, new_PLLN);
}

static uint32_t stm32_rcc_RCC_CFGR_read(Stm32Rcc *s)
{
    return (s->RCC_CFGR_MCO2 << RCC_CFGR_MCO2_START) |
           (s->RCC_CFGR_MCO2PRE << RCC_CFGR_MCO2PRE_START) |
           (s->RCC_CFGR_MCO1PRE << RCC_CFGR_MCO1PRE_START) |
           (s->RCC_CFGR_I2SSRC << RCC_CFGR_I2SSRC_BIT) |
           (s->RCC_CFGR_MCO1PRE << RCC_CFGR_MCO1PRE_START) |
           (s->RCC_CFGR_RTCPRE << RCC_CFGR_RTCPRE_START) |
           (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
           (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
           (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
           (s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
           (s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}


static void stm32_rcc_RCC_CFGR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    // PPRE2
    s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
    if(s->RCC_CFGR_PPRE2 < 0x4) {
        clktree_set_scale(s->PCLK2, 1, 1);
    } else {
        clktree_set_scale(s->PCLK2, 1, 2 << (s->RCC_CFGR_PPRE2 & 0x3));
    }

    // PPRE1
    s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
        clktree_set_scale(s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(s->PCLK1, 1, 2 << (s->RCC_CFGR_PPRE1 & 0x3));
    }

    // HPRE
    s->RCC_CFGR_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
    if(s->RCC_CFGR_HPRE < 8) {
        clktree_set_scale(s->HCLK, 1, 1);
    } else {
        if(s->RCC_CFGR_HPRE < 12)
            clktree_set_scale(s->HCLK, 1, 2 << ((s->RCC_CFGR_HPRE & 0x7)));
        else
            clktree_set_scale(s->HCLK, 1, 2 << ((s->RCC_CFGR_HPRE & 0x7) + 1));
    }

    // SW
    s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
    switch(s->RCC_CFGR_SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
            break;
        default:
            hw_error("Invalid input selected for SYSCLK");
            break;
    }

    s->RCC_CFGR_RTCPRE = (new_value & RCC_CFGR_RTCPRE_MASK) >> RCC_CFGR_RTCPRE_START;
    if(s->RCC_CFGR_RTCPRE > 1)
    {
        clktree_set_scale(s->RTCCLK, 1, s->RCC_CFGR_RTCPRE);
    }
}

/* Write the AHB3 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_AHB3ENR_write(Stm32Rcc *s, uint32_t new_value,
                                        bool init)
{
    s->RCC_AHB3ENR = new_value;
}

/* Write the AHB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_AHB2ENR_write(Stm32Rcc *s, uint32_t new_value,
                                        bool init)
{
    s->RCC_AHB2ENR = new_value;
}

/* Write the AHB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_AHB1ENR_write(Stm32Rcc *s, uint32_t new_value,
                                        bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOI,
                            RCC_AHB1ENR_GPIOIEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOH,
                            RCC_AHB1ENR_GPIOHEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOG,
                            RCC_AHB1ENR_GPIOGEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOF,
                            RCC_AHB1ENR_GPIOFEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOE,
                            RCC_AHB1ENR_GPIOEEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOD,
                            RCC_AHB1ENR_GPIODEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOC,
                            RCC_AHB1ENR_GPIOCEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOB,
                            RCC_AHB1ENR_GPIOBEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_GPIOA,
                            RCC_AHB1ENR_GPIOAEN_BIT);
    s->RCC_AHB1ENR = new_value;
}

/* Write the APB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB2ENR_write(Stm32Rcc *s, uint32_t new_value,
                                        bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART1,
                            RCC_APB2ENR_USART1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART6,
                            RCC_APB2ENR_USART6EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG,
                            RCC_APB2ENR_SYSCFGEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_SPI1,
                            RCC_APB2ENR_SPI1EN_BIT);

    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM1,
                            RCC_APB2ENR_TIM1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM8,
                            RCC_APB2ENR_TIM8EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM9,
                            RCC_APB2ENR_TIM9EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM10,
                            RCC_APB2ENR_TIM10EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM11,
                            RCC_APB2ENR_TIM11EN_BIT);

    s->RCC_APB2ENR = new_value & 0xffffffff;
}

/* Write the APB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB1ENR_write(Stm32Rcc *s, uint32_t new_value,
                    bool init)
{
    stm32_rcc_periph_enable(s, new_value, init, STM32_I2C1,
                            RCC_APB1ENR_I2C1EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_I2C2,
                            RCC_APB1ENR_I2C2EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_I2C3,
                            RCC_APB1ENR_I2C3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART5,
                            RCC_APB1ENR_USART5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART4,
                            RCC_APB1ENR_USART4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART3,
                            RCC_APB1ENR_USART3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART2,
                            RCC_APB1ENR_USART2EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_WWDG,
                            RCC_APB1ENR_WWDGEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_PWR,
                            RCC_APB1ENR_PWREN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_SPI2,
                            RCC_APB1ENR_SPI2EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_SPI3,
                            RCC_APB1ENR_SPI3EN_BIT);

    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM14,
                            RCC_APB1ENR_TIM14EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM13,
                            RCC_APB1ENR_TIM13EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM12,
                            RCC_APB1ENR_TIM12EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM7,
                            RCC_APB1ENR_TIM7EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM6,
                            RCC_APB1ENR_TIM7EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM6,
                            RCC_APB1ENR_TIM6EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM5,
                            RCC_APB1ENR_TIM5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM4,
                            RCC_APB1ENR_TIM4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM3,
                            RCC_APB1ENR_TIM3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_TIM2,
                            RCC_APB1ENR_TIM2EN_BIT);

    s->RCC_APB1ENR = new_value & 0xffffffff;
}

static uint32_t stm32_rcc_RCC_BDCR_read(Stm32Rcc *s)
{
    int lseon_bit = clktree_is_enabled(s->LSECLK) ? 1 : 0;
    int rtcen_bit = clktree_is_enabled(s->RTCCLK) ? 1 : 0;

    return lseon_bit << RCC_BDCR_LSERDY_BIT |
           lseon_bit << RCC_BDCR_LSEON_BIT |
           rtcen_bit << RCC_BDCR_RTCEN_BIT |
           s->RCC_BDCR_RTCSEL << RCC_BDCR_RTCSEL_START;
}

static void stm32_rcc_RCC_BDCR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSECLK, new_value & BIT(RCC_BDCR_LSEON_BIT));
    clktree_set_enabled(s->RTCCLK, new_value & BIT(RCC_BDCR_RTCEN_BIT));

    int rtcsel = extract32(new_value, RCC_BDCR_RTCSEL_START, RCC_BDCR_RTCSEL_LENGTH) - 1;
    clktree_set_selected_input(s->RTCCLK, rtcsel);
    s->RCC_BDCR_RTCSEL = rtcsel;
    s->RCC_BDCR = new_value;
}

static void stm32_rcc_RCC_BDCR_writeb(Stm32Rcc *s, uint8_t offset, uint8_t new_value, bool init)
{
    uint32_t value = s->RCC_BDCR;
    uint8_t *valueb = (uint8_t*)&value;
    valueb[offset] = new_value;
    stm32_rcc_RCC_BDCR_write(s, value, init);

//    clktree_set_enabled(s->LSECLK, value & BIT(RCC_BDCR_LSEON_BIT));
}

/* Works the same way as stm32_rcc_RCC_CR_read */
static uint32_t stm32_rcc_RCC_CSR_read(Stm32Rcc *s)
{
    int lseon_bit = clktree_is_enabled(s->LSICLK) ? 1 : 0;

    return lseon_bit << RCC_CSR_LSIRDY_BIT |
           lseon_bit << RCC_CSR_LSION_BIT;
}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(Stm32Rcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSICLK, new_value & BIT(RCC_CSR_LSION_BIT));
}



static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    Stm32Rcc *s = (Stm32Rcc *)opaque;
    //DPRINTF("RCC_READ: Offset = 0x%X\n", (int)offset);
    switch (offset) {
        case RCC_CR_OFFSET:
            return stm32_rcc_RCC_CR_read(s);
        case RCC_CFGR_OFFSET:
            return stm32_rcc_RCC_CFGR_read(s);
        case RCC_PLLCFGR_OFFSET:
            return stm32_rcc_RCC_PLLCFGR_read(s);
        case RCC_CIR_OFFSET:
            return 0;
        case RCC_AHB1ENR_OFFSET:
            return s->RCC_AHB1ENR;
        case RCC_AHB2ENR_OFFSET:
            return s->RCC_AHB2ENR;
        case RCC_AHB3ENR_OFFSET:
            return s->RCC_AHB3ENR;
        case RCC_APB1RSTR_OFFSET:
        case RCC_APB2RSTR_OFFSET:
        case RCC_AHB1RSTR_OFFSET:
        case RCC_AHB2RSTR_OFFSET:
        case RCC_AHB3RSTR_OFFSET:
//            STM32_NOT_IMPL_REG(offset, 4);
            return 0;
        case RCC_APB2ENR_OFFSET:
            return s->RCC_APB2ENR;
        case RCC_APB1ENR_OFFSET:
            return s->RCC_APB1ENR;
        case RCC_BDCR_OFFSET:
            return stm32_rcc_RCC_BDCR_read(s);
        case RCC_CSR_OFFSET:
            return stm32_rcc_RCC_CSR_read(s);
            STM32_NOT_IMPL_REG(offset, 4);
            return 0;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}


static void stm32_rcc_writew(void *opaque, hwaddr offset,
                          uint64_t value)
{
    Stm32Rcc *s = (Stm32Rcc *)opaque;
    //DPRINTF("RCC_WRITE: Offset = 0x%X New_value = 0x%" PRIu64 "\n", (int)offset, value);

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_PLLCFGR_OFFSET:
            stm32_rcc_RCC_PLLCFGR_write(s, value, false);
            break;
        case RCC_CFGR_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CIR_OFFSET:
            /* Allow a write but don't take any action */
            break;
        case RCC_AHB1RSTR_OFFSET:
        case RCC_AHB2RSTR_OFFSET:
        case RCC_AHB3RSTR_OFFSET:
        case RCC_APB1RSTR_OFFSET:
        case RCC_APB2RSTR_OFFSET:
            DPRINTF("%s (0x%X): Attempting to reset Periph: %" PRIu64 "\n", stm32_rcc_registername(offset), (uint32_t)offset, value);
//            STM32_NOT_IMPL_REG(offset, 4);
            break;
        case RCC_AHB1ENR_OFFSET:
            stm32_rcc_RCC_AHB1ENR_write(s, value, false);
            break;
        case RCC_AHB2ENR_OFFSET:
            stm32_rcc_RCC_AHB2ENR_write(s, value, false);
            break;
        case RCC_AHB3ENR_OFFSET:
            stm32_rcc_RCC_AHB3ENR_write(s, value, false);
            break;
        case RCC_APB1ENR_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_APB2ENR_OFFSET:
            stm32_rcc_RCC_APB2ENR_write(s, value, false);
            break;
        case RCC_AHB1LPENR_OFFSET:
        case RCC_AHB2LPENR_OFFSET:
        case RCC_AHB3LPENR_OFFSET:
        case RCC_APB1LPENR_OFFSET:
        case RCC_APB2LPENR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        case RCC_SSCGR_OFFSET:
        case RCC_PLLI2SCFGR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 4);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}

static void stm32_rcc_writeb(void *opaque, hwaddr offset,
                          uint8_t value)
{
    Stm32Rcc *s = (Stm32Rcc *)opaque;
    switch(offset & 0xFFFFFFFC)
    {
        case RCC_BDCR_OFFSET:
            stm32_rcc_RCC_BDCR_writeb(s, offset - RCC_BDCR_OFFSET, value, false);
            break;
        default:
            STM32_BAD_REG(offset, 4);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            STM32_NOT_IMPL_REG(offset, size);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value);
            break;
        case 1:
            stm32_rcc_writeb(opaque, offset, (uint8_t)value);
            break;
        default:
            STM32_NOT_IMPL_REG(offset, size);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    Stm32Rcc *s = FROM_SYSBUS(Stm32Rcc, SYS_BUS_DEVICE(dev));

    stm32_rcc_RCC_CR_write(s, 0x00000083, true);
    stm32_rcc_RCC_PLLCFGR_write(s, 0x24003010, true);
    stm32_rcc_RCC_CFGR_write(s, 0x00000000, true);
    stm32_rcc_RCC_AHB3ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_AHB2ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_AHB1ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_APB2ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_APB1ENR_write(s, 0x00000000, true);
    stm32_rcc_RCC_BDCR_write(s, 0x00000000, true);
    stm32_rcc_RCC_CSR_write(s, 0x0c000000, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    Stm32Rcc *s = (Stm32Rcc *)opaque;

    uint32_t hclk_freq, ext_ref_freq;

    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if(hclk_freq > 0) {
        ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = get_ticks_per_sec() / hclk_freq;
        external_ref_clock_scale = get_ticks_per_sec() / ext_ref_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
                (unsigned long)hclk_freq, system_clock_scale);
    DPRINTF("Cortex SYSTICK ext ref frequency set to %lu Hz "
              "(scale set to %d).\n",
              (unsigned long)ext_ref_freq, external_ref_clock_scale);
#endif
}







/* PUBLIC FUNCTIONS */

void stm32_rcc_check_periph_clk(Stm32Rcc *s, stm32_periph_t periph)
{
    Clk clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    if(!clktree_is_enabled(clk)) {
        /* I assume writing to a peripheral register while the peripheral clock
         * is disabled is a bug and give a warning to unsuspecting programmers.
         * When I made this mistake on real hardware the write had no effect.
         */
        hw_error("Warning: You are attempting to use the %s peripheral while "
                 "its clock is disabled.\n", stm32_periph_name(periph));
    }
}

void stm32_rcc_set_periph_clk_irq(
        Stm32Rcc *s,
        stm32_periph_t periph,
        qemu_irq periph_irq)
{
    Clk clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    clktree_adduser(clk, periph_irq);
}

uint32_t stm32_rcc_get_periph_freq(
        Stm32Rcc *s,
        stm32_periph_t periph)
{
    Clk clk;

    clk = s->PERIPHCLK[periph];

    assert(clk != NULL);

    return clktree_get_output_freq(clk);
}


/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_init_clk(Stm32Rcc *s)
{
    int i;
    qemu_irq *hclk_upd_irq =
            qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", HSI_FREQ, false);
    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);

    s->PLL_VCO = clktree_create_clk("PLL_VCO", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0,
                        s->HSICLK, s->HSECLK, NULL);

    /* PLLCLK contains both the switch and the multiplier, which are shown as
     * two separate components in the clock tree diagram.
     */
    s->PLLCLK = clktree_create_clk("PLLCLK", 0, 1, false, 120000000, CLKTREE_NO_INPUT,
                        s->PLL_VCO, NULL);

    s->SYSCLK = clktree_create_clk("SYSCLK", 1, 1, true, 120000000, CLKTREE_NO_INPUT,
                        s->HSICLK, s->HSECLK, s->PLLCLK, NULL);

    s->RTCCLK = clktree_create_clk("RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, CLKTREE_NO_INPUT,
                        s->LSECLK, s->LSICLK, s->HSECLK, NULL);

    s->HCLK = clktree_create_clk("HCLK", 0, 1, true, 120000000, 0,
                        s->SYSCLK, NULL);
    clktree_adduser(s->HCLK, hclk_upd_irq[0]);

    s->PCLK1 = clktree_create_clk("PCLK1", 0, 1, true, 30000000, 0,
                        s->HCLK, NULL);
    s->PCLK2 = clktree_create_clk("PCLK2", 0, 1, true, 60000000, 0,
                        s->HCLK, NULL);

    /* Peripheral clocks */
    s->PERIPHCLK[STM32_SYSCFG] = clktree_create_clk("SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_WWDG] = clktree_create_clk("WWDG",  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_PWR] = clktree_create_clk("PWR",  1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);

    s->PERIPHCLK[STM32_GPIOA] = clktree_create_clk("GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOB] = clktree_create_clk("GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOC] = clktree_create_clk("GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOD] = clktree_create_clk("GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOE] = clktree_create_clk("GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOF] = clktree_create_clk("GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOG] = clktree_create_clk("GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOH] = clktree_create_clk("GPIOH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOI] = clktree_create_clk("GPIOI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_UART1] = clktree_create_clk("UART1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_UART2] = clktree_create_clk("UART2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART3] = clktree_create_clk("UART3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART4] = clktree_create_clk("UART4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART5] = clktree_create_clk("UART5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART6] = clktree_create_clk("UART6", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);

    s->PERIPHCLK[STM32_SPI1] = clktree_create_clk("SPI1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_SPI2] = clktree_create_clk("SPI2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_SPI3] = clktree_create_clk("SPI3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);

    s->PERIPHCLK[STM32_I2C1] = clktree_create_clk("I2C1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_I2C2] = clktree_create_clk("I2C2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_I2C3] = clktree_create_clk("I2C3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);

    s->PERIPHCLK[STM32_TIM1] = clktree_create_clk("TIM1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM2] = clktree_create_clk("TIM2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM3] = clktree_create_clk("TIM3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM4] = clktree_create_clk("TIM4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM5] = clktree_create_clk("TIM5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM6] = clktree_create_clk("TIM6", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM7] = clktree_create_clk("TIM7", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM8] = clktree_create_clk("TIM8", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM9] = clktree_create_clk("TIM9", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM10] = clktree_create_clk("TIM10", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM11] = clktree_create_clk("TIM11", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM12] = clktree_create_clk("TIM12", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM13] = clktree_create_clk("TIM13", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_TIM14] = clktree_create_clk("TIM14", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
}






static int stm32_rcc_init(SysBusDevice *dev)
{
    Stm32Rcc *s = FROM_SYSBUS(Stm32Rcc, dev);

    memory_region_init_io(&s->iomem, &stm32_rcc_ops, s,
                          "rcc", 0x400);

    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    stm32_rcc_init_clk(s);
//    stm32_rcc_reset(dev);
    return 0;
}


static Property stm32_rcc_properties[] = {
    DEFINE_PROP_UINT32("osc_freq", Stm32Rcc, osc_freq, 0),
    DEFINE_PROP_UINT32("osc32_freq", Stm32Rcc, osc32_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_rcc_init;
    dc->reset = stm32_rcc_reset;
    dc->props = stm32_rcc_properties;
}

static TypeInfo stm32_rcc_info = {
    .name  = "stm32-rcc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Rcc),
    .class_init = stm32_rcc_class_init
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
