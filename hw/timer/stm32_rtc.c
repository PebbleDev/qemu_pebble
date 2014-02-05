/*
 * STM32F2 Real Time Clock
 * Copyright (c) 2013 Jens Andersen <jens.andersen@gmail.com>
 * Based on:
 * Samsung exynos4210 Real Time Clock
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *  Ogurtsov Oleg <o.ogurtsov@samsung.com>
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

/* Description:
 * Register RTCCON:
 *  CLKSEL Bit[1] not used
 *  CLKOUTEN Bit[9] not used
 */

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qemu-common.h"
#include "qemu/bitops.h"
#include "hw/ptimer.h"

#include "hw/hw.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"

#include "hw/arm/stm32.h"

#define DEBUG_RTC 1

#if DEBUG_RTC
#define DPRINTF(fmt, ...) \
        do { fprintf(stderr, "STM32_RTC: [%24s:%5d] " fmt, __func__, __LINE__, \
                ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while (0)
#endif

#define     STM32_RTC_REG_MEM_SIZE     0x0400

#define RTC_TR_OFFSET 0x00
#define RTC_TR_SU_START 0
#define RTC_TR_SU_LENGTH 4
#define RTC_TR_ST_START 4
#define RTC_TR_ST_LENGTH 3
#define RTC_TR_MNU_START 8
#define RTC_TR_MNU_LENGTH 4
#define RTC_TR_MNT_START 12
#define RTC_TR_MNT_LENGTH 3
#define RTC_TR_HU_START 16
#define RTC_TR_HU_LENGTH 4
#define RTC_TR_HT_START 20
#define RTC_TR_HT_LENGTH 2
#define RTC_TR_PM_BIT 22

#define RTC_DR_OFFSET 0x04
#define RTC_DR_DU_START 0
#define RTC_DR_DU_LENGTH 4
#define RTC_DR_DT_START 4
#define RTC_DR_DT_LENGTH 2
#define RTC_DR_MU_START 8
#define RTC_DR_MU_LENGTH 4
#define RTC_DR_MT_BIT 12
#define RTC_DR_WDU_START 13
#define RTC_DR_WDU_LENGTH 3
#define RTC_DR_YU_START 16
#define RTC_DR_YU_LENGTH 4
#define RTC_DR_YT_START 20
#define RTC_DR_YT_LENGTH 4

#define RTC_CR_OFFSET 0x08
#define RTC_CR_WUCKSEL_START 0
#define RTC_CR_WUCKSEL_LENGTH 3
#define RTC_CR_TSEDGE_BIT 3
#define RTC_CR_REFCKON_BIT 4
#define RTC_CR_FMT_BIT 6
#define RTC_CR_DCE_BIT 7
#define RTC_CR_ALRAE_BIT 8
#define RTC_CR_ALRBE_BIT 9
#define RTC_CR_WUTE_BIT 10
#define RTC_CR_TSE_BIT 11
#define RTC_CR_ALRAIE_BIT 12
#define RTC_CR_ALRBIE 13
#define RTC_CR_WUTIE_BIT 14
#define RTC_CR_TSIE_BIT 15
#define RTC_CR_ADD1H_BIT 16
#define RTC_CR_SUB1H_BIT 17
#define RTC_CR_BKP_BIT 18
#define RTC_CR_POL_BIT 20
#define RTC_CR_OSEL_START 21
#define RTC_CR_OSEL_LENGTH 2
#define RTC_CR_COE_BIT 23

#define RTC_ISR_OFFSET       0x0c
#define RTC_ISR_ALRAWF_BIT 0
#define RTC_ISR_ALRBWF_BIT 1
#define RTC_ISR_WUTWF_BIT 2
#define RTC_ISR_INITS_BIT 4
#define RTC_ISR_RSF_BIT 5
#define RTC_ISR_INITF_BIT 6
#define RTC_ISR_INIT_BIT 7
#define RTC_ISR_ALRAF_BIT 8
#define RTC_ISR_ALRBF_BIT 9
#define RTC_ISR_WUTF_BIT 10
#define RTC_ISR_TSF_BIT 11
#define RTC_ISR_TSOVF_BIT 12
#define RTC_ISR_TAMP1F_BIT 13

#define RTC_PRER_OFFSET      0x10
#define RTC_WUTR_OFFSET      0x14
#define RTC_CALIBR_OFFSET    0x18
#define RTC_ALMAR_OFFSET    0x1c
#define RTC_ALRMBR_OFFSET          0x20
#define RTC_WPR_OFFSET         0x24
#define RTC_TSTR_OFFSET            0x30
#define RTC_RTC_TSDR_OFFSET        0x34
#define RTC_TAFCR_OFFSET           0x40
#define RTC_BKP0R_OFFSET           0x50
#define RTC_BKP19R_OFFSET          0x9c


#define     TICK_TIMER_ENABLE   0x0100
#define     TICNT_THRESHHOLD    2


#define     RTC_ENABLE          0x0001

#define     INTP_TICK_ENABLE    0x0001
#define     INTP_ALM_ENABLE     0x0002

#define     ALARM_INT_ENABLE    0x0040

#define     RTC_BASE_FREQ       32768


typedef enum
{
    RTC_LOCKED,
    RTC_UNLOCKED1,
    RTC_UNLOCKED
} stm32_rtc_states;

typedef struct STM32RTCState {

    SysBusDevice busdev;
    MemoryRegion iomem;

    stm32_rtc_states state;

    /* registers */
    uint32_t
        RTC_TR,
        RTC_DR,
        RTC_CR,
        RTC_PRER;


    /* register fields */
    uint32_t
        RTC_ISR_INIT,
        RTC_ISR_RSF;

    uint32_t    reg_intp;
    uint32_t    reg_rtccon;
    uint32_t    reg_ticcnt;
    uint32_t    reg_rtcalm;
    uint32_t    reg_almsec;
    uint32_t    reg_almmin;
    uint32_t    reg_almhour;
    uint32_t    reg_almday;
    uint32_t    reg_almmon;
    uint32_t    reg_almyear;
    uint32_t    reg_curticcnt;
    uint32_t    reg_bkp[0x4d];

    ptimer_state    *ptimer;        /* tick timer */
    ptimer_state    *ptimer_1Hz;    /* clock timer */
    uint32_t        freq;

    qemu_irq        alarm_irq; /* RTCAlarm IRQ */
    qemu_irq        wkup_irq;   /* RTC Wakeup IRQ */
    qemu_irq        tamp_stamp_irq;    /* RTC Tamp Stamp IRQ */

    struct tm   current_tm;     /* current time */
} STM32RTCState;

#define TICCKSEL(value) ((value & (0x0F << 4)) >> 4)

/*** VMState ***/
static const VMStateDescription vmstate_stm32_rtc_state = {
    .name = "stm32-rtc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(reg_intp, STM32RTCState),
        VMSTATE_UINT32(reg_rtccon, STM32RTCState),
        VMSTATE_UINT32(reg_ticcnt, STM32RTCState),
        VMSTATE_UINT32(reg_rtcalm, STM32RTCState),
        VMSTATE_UINT32(reg_almsec, STM32RTCState),
        VMSTATE_UINT32(reg_almmin, STM32RTCState),
        VMSTATE_UINT32(reg_almhour, STM32RTCState),
        VMSTATE_UINT32(reg_almday, STM32RTCState),
        VMSTATE_UINT32(reg_almmon, STM32RTCState),
        VMSTATE_UINT32(reg_almyear, STM32RTCState),
        VMSTATE_UINT32(reg_curticcnt, STM32RTCState),
        VMSTATE_PTIMER(ptimer, STM32RTCState),
        VMSTATE_PTIMER(ptimer_1Hz, STM32RTCState),
        VMSTATE_UINT32(freq, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_sec, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_min, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_hour, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_wday, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_mday, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_mon, STM32RTCState),
        VMSTATE_INT32(current_tm.tm_year, STM32RTCState),
        VMSTATE_END_OF_LIST()
    }
};

#define BCD3DIGITS(x) \
    ((uint32_t)to_bcd((uint8_t)(x % 100)) + \
    ((uint32_t)to_bcd((uint8_t)((x % 1000) / 100)) << 8))

/*static void check_alarm_raise(STM32RTCState *s)
{
    unsigned int alarm_raise = 0;
    struct tm stm = s->current_tm;

    if ((s->reg_rtcalm & 0x01) &&
        (to_bcd((uint8_t)stm.tm_sec) == (uint8_t)s->reg_almsec)) {
        alarm_raise = 1;
    }
    if ((s->reg_rtcalm & 0x02) &&
        (to_bcd((uint8_t)stm.tm_min) == (uint8_t)s->reg_almmin)) {
        alarm_raise = 1;
    }
    if ((s->reg_rtcalm & 0x04) &&
        (to_bcd((uint8_t)stm.tm_hour) == (uint8_t)s->reg_almhour)) {
        alarm_raise = 1;
    }
    if ((s->reg_rtcalm & 0x08) &&
        (to_bcd((uint8_t)stm.tm_mday) == (uint8_t)s->reg_almday)) {
        alarm_raise = 1;
    }
    if ((s->reg_rtcalm & 0x10) &&
         (to_bcd((uint8_t)stm.tm_mon) == (uint8_t)s->reg_almmon)) {
        alarm_raise = 1;
    }
    if ((s->reg_rtcalm & 0x20) &&
        (BCD3DIGITS(stm.tm_year) == s->reg_almyear)) {
        alarm_raise = 1;
    }

    if (alarm_raise) {
        DPRINTF("ALARM IRQ\n");
        // set irq status
        s->reg_intp |= INTP_ALM_ENABLE;
        qemu_irq_raise(s->alarm_irq);
    }
}*/

/*
 * RTC update frequency
 * Parameters:
 *     reg_value - current RTCCON register or his new value
 */
static void stm32_rtc_update_freq(STM32RTCState *s,
                                       uint32_t reg_value)
{
    uint32_t freq;

    freq = s->freq;
    /* set frequncy for time generator */
    s->freq = RTC_BASE_FREQ / (1 << TICCKSEL(reg_value));

    if (freq != s->freq) {
        ptimer_set_freq(s->ptimer, s->freq);
        DPRINTF("freq=%dHz\n", s->freq);
    }
}

/* month is between 0 and 11. */
static int get_days_in_month(int month, int year)
{
    static const int days_tab[12] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    int d;
    if ((unsigned)month >= 12) {
        return 31;
    }
    d = days_tab[month];
    if (month == 1) {
        if ((year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0)) {
            d++;
        }
    }
    return d;
}

/* update 'tm' to the next second */
static void rtc_next_second(struct tm *tm)
{
    int days_in_month;

    tm->tm_sec++;
    if ((unsigned)tm->tm_sec >= 60) {
        tm->tm_sec = 0;
        tm->tm_min++;
        if ((unsigned)tm->tm_min >= 60) {
            tm->tm_min = 0;
            tm->tm_hour++;
            if ((unsigned)tm->tm_hour >= 24) {
                tm->tm_hour = 0;
                /* next day */
                tm->tm_wday++;
                if ((unsigned)tm->tm_wday >= 7) {
                    tm->tm_wday = 0;
                }
                days_in_month = get_days_in_month(tm->tm_mon,
                                                  tm->tm_year + 1900);
                tm->tm_mday++;
                if (tm->tm_mday < 1) {
                    tm->tm_mday = 1;
                } else if (tm->tm_mday > days_in_month) {
                    tm->tm_mday = 1;
                    tm->tm_mon++;
                    if (tm->tm_mon >= 12) {
                        tm->tm_mon = 0;
                        tm->tm_year++;
                    }
                }
            }
        }
    }
}

/*
 * tick handler
 */
static void stm32_rtc_tick(void *opaque)
{
    STM32RTCState *s = (STM32RTCState *)opaque;

    DPRINTF("TICK IRQ\n");
    /* set irq status */
//    s->reg_intp |= INTP_TICK_ENABLE;
    /* raise IRQ */
//    qemu_irq_raise(s->tick_irq);

    /* restart timer */
    ptimer_set_count(s->ptimer, s->reg_ticcnt);
    ptimer_run(s->ptimer, 1);
}

/*
 * 1Hz clock handler
 */
static void stm32_rtc_1Hz_tick(void *opaque)
{
    STM32RTCState *s = (STM32RTCState *)opaque;

    rtc_next_second(&s->current_tm);
    DPRINTF("1Hz tick\n");

    /* raise IRQ */
/*    if ((s->RTC_CR & BIT(RTC_CR_ALRAE_BIT)) || (s->RTC_CR & BIT(RTC_CR_ALRBE_BIT))) {
        check_alarm_raise(s);
    }*/

    ptimer_set_count(s->ptimer_1Hz, RTC_BASE_FREQ);
    ptimer_run(s->ptimer_1Hz, 1);
}

/*
 * RTC Read
 */
static uint64_t stm32_rtc_read(void *opaque, hwaddr offset,
        unsigned size)
{
    uint32_t value = 0;
    STM32RTCState *s = (STM32RTCState *)opaque;
    assert(size == 4);

    switch (offset) {
        case RTC_TR_OFFSET:
            value = (to_bcd(s->current_tm.tm_hour) & 0x3f) << RTC_TR_HU_START |
                    (to_bcd(s->current_tm.tm_min) & 0x7f) << RTC_TR_MNU_START |
                    (to_bcd(s->current_tm.tm_sec) & 0x7f) << RTC_TR_SU_START;
            break;
        case RTC_DR_OFFSET:
            value = (to_bcd((s->current_tm.tm_year - 100)) & 0xff) << RTC_DR_YU_START |
                    (to_bcd(s->current_tm.tm_wday) & 0x1f) << RTC_DR_WDU_START |
                    (to_bcd(s->current_tm.tm_mon) & 0x3f) << RTC_DR_MU_START |
                    (to_bcd(s->current_tm.tm_mday) & 0x3f) << RTC_DR_DU_START;
            break;
        case RTC_WUTR_OFFSET:
        case RTC_CALIBR_OFFSET:
        case RTC_ALMAR_OFFSET:
        case RTC_ALRMBR_OFFSET:
        case RTC_WPR_OFFSET:
        case RTC_TSTR_OFFSET:
        case RTC_RTC_TSDR_OFFSET:
        case RTC_TAFCR_OFFSET:
            STM32_NOT_IMPL_REG(offset, size);
            break;
        case RTC_PRER_OFFSET:
            value = s->RTC_PRER;
            break;
        case RTC_CR_OFFSET:
            value = s->RTC_CR;
            break;
        case RTC_ISR_OFFSET:
            value = s->RTC_ISR_INIT << RTC_ISR_INIT_BIT |
                    s->RTC_ISR_INIT << RTC_ISR_INITF_BIT |
                    s->RTC_ISR_RSF << RTC_ISR_RSF_BIT |
                    (s->current_tm.tm_year > 0) << RTC_ISR_INITS_BIT |
                    0x00000007;
            break;
/*

    case INTP:
        value = s->reg_intp;
        break;
    case RTCCON:
        value = s->reg_rtccon;
        break;
    case TICCNT:
        value = s->reg_ticcnt;
        break;
    case RTCALM:
        value = s->reg_rtcalm;
        break;
    case ALMSEC:
        value = s->reg_almsec;
        break;
    case ALMMIN:
        value = s->reg_almmin;
        break;
    case ALMHOUR:
        value = s->reg_almhour;
        break;
    case ALMDAY:
        value = s->reg_almday;
        break;
    case ALMMON:
        value = s->reg_almmon;
        break;
    case ALMYEAR:
        value = s->reg_almyear;
        break;

    case BCDSEC:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_sec);
        break;
    case BCDMIN:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_min);
        break;
    case BCDHOUR:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_hour);
        break;
    case BCDDAYWEEK:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_wday);
        break;
    case BCDDAY:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_mday);
        break;
    case BCDMON:
        value = (uint32_t)to_bcd((uint8_t)s->current_tm.tm_mon + 1);
        break;
    case BCDYEAR:
        value = BCD3DIGITS(s->current_tm.tm_year);
        break;

    case CURTICNT:
        s->reg_curticcnt = ptimer_get_count(s->ptimer);
        value = s->reg_curticcnt;
        break;
*/
    default:
        if(offset >= RTC_BKP0R_OFFSET && offset <= RTC_BKP19R_OFFSET)
        {
            // BKP storage
            value =s->reg_bkp[offset - RTC_BKP0R_OFFSET];
            break;
        }
        fprintf(stderr,
                "[stm32-rtc: bad read offset " TARGET_FMT_plx "]\n",
                offset);
        break;
    }
    DPRINTF("Read from 0x%x, value 0x%x\n", (uint32_t)offset, (uint32_t)value);
    return value;
}

/*
 * RTC Write
 */
static void stm32_rtc_write(void *opaque, hwaddr offset,
        uint64_t value, unsigned size)
{
    STM32RTCState *s = (STM32RTCState *)opaque;
    assert(size == 4);
    DPRINTF("Write to 0x%x with value 0x%x\n", (uint32_t)offset, (uint32_t)value);
    switch (offset) {
        case RTC_TR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
//            s->RTC_TR = value;
//            STM32_NOT_IMPL_REG(offset, size);
            break;
        case RTC_DR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
//            STM32_NOT_IMPL_REG(offset, size);
//            break;
        case RTC_CR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            if(value != 0)
                STM32_NOT_IMPL_REG(offset, size);
            break;
        case RTC_ISR_OFFSET:
            s->RTC_ISR_INIT = extract32(value, RTC_ISR_INIT_BIT, 1);
            if(value & BIT(RTC_ISR_INIT_BIT))
            {
                DPRINTF("Entering Init mode!\n");
            }
            if(!(value & BIT(RTC_ISR_RSF_BIT)))
            {
                s->RTC_ISR_RSF = 1;
            }
//            s->RTC_ISR = value;
            break;
        case RTC_PRER_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            s->RTC_PRER = value;
            break;
        case RTC_WUTR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_CALIBR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_ALMAR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_ALRMBR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_WPR_OFFSET:
            switch(s->state)
            {
                case RTC_LOCKED:
                    if(value == 0xCA)
                        s->state = RTC_UNLOCKED1;
                    break;
                case RTC_UNLOCKED1:
                    if(value == 0x53)
                    {
                        s->state = RTC_UNLOCKED;
                        DPRINTF("Unlocked!\n");
                    }
                    else
                        s->state = RTC_LOCKED;
                    break;
                default:
                    DPRINTF("Invalid data written to WPR, resetting lockstate: 0x%X\n", (uint32_t)value);
                    s->state = RTC_LOCKED;
                    break;
            }
            break;
        case RTC_TSTR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_RTC_TSDR_OFFSET:
            assert(s->state == RTC_UNLOCKED);
            break;
        case RTC_TAFCR_OFFSET:
            break;
/*    case INTP:
        if (value & INTP_ALM_ENABLE) {
            qemu_irq_lower(s->alm_irq);
            s->reg_intp &= (~INTP_ALM_ENABLE);
        }
        if (value & INTP_TICK_ENABLE) {
            qemu_irq_lower(s->tick_irq);
            s->reg_intp &= (~INTP_TICK_ENABLE);
        }
        break;
    case RTCCON:
        if (value & RTC_ENABLE) {
            stm32_rtc_update_freq(s, value);
        }
        if ((value & RTC_ENABLE) > (s->reg_rtccon & RTC_ENABLE)) {
            // clock timer 
            ptimer_set_count(s->ptimer_1Hz, RTC_BASE_FREQ);
            ptimer_run(s->ptimer_1Hz, 1);
            DPRINTF("run clock timer\n");
        }
        if ((value & RTC_ENABLE) < (s->reg_rtccon & RTC_ENABLE)) {
            // tick timer
            ptimer_stop(s->ptimer);
            // clock timer
            ptimer_stop(s->ptimer_1Hz);
            DPRINTF("stop all timers\n");
        }
        if (value & RTC_ENABLE) {
            if ((value & TICK_TIMER_ENABLE) >
                (s->reg_rtccon & TICK_TIMER_ENABLE) &&
                (s->reg_ticcnt)) {
                ptimer_set_count(s->ptimer, s->reg_ticcnt);
                ptimer_run(s->ptimer, 1);
                DPRINTF("run tick timer\n");
            }
            if ((value & TICK_TIMER_ENABLE) <
                (s->reg_rtccon & TICK_TIMER_ENABLE)) {
                ptimer_stop(s->ptimer);
            }
        }
        s->reg_rtccon = value;
        break;
    case TICCNT:
        if (value > TICNT_THRESHHOLD) {
            s->reg_ticcnt = value;
        } else {
            fprintf(stderr,
                    "[stm32-rtc: bad TICNT value %u ]\n",
                    (uint32_t)value);
        }
        break;

    case RTCALM:
        s->reg_rtcalm = value;
        break;
    case ALMSEC:
        s->reg_almsec = (value & 0x7f);
        break;
    case ALMMIN:
        s->reg_almmin = (value & 0x7f);
        break;
    case ALMHOUR:
        s->reg_almhour = (value & 0x3f);
        break;
    case ALMDAY:
        s->reg_almday = (value & 0x3f);
        break;
    case ALMMON:
        s->reg_almmon = (value & 0x1f);
        break;
    case ALMYEAR:
        s->reg_almyear = (value & 0x0fff);
        break;

    case BCDSEC:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_sec = (int)from_bcd((uint8_t)value);
        }
        break;
    case BCDMIN:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_min = (int)from_bcd((uint8_t)value);
        }
        break;
    case BCDHOUR:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_hour = (int)from_bcd((uint8_t)value);
        }
        break;
    case BCDDAYWEEK:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_wday = (int)from_bcd((uint8_t)value);
        }
        break;
    case BCDDAY:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_mday = (int)from_bcd((uint8_t)value);
        }
        break;
    case BCDMON:
        if (s->reg_rtccon & RTC_ENABLE) {
            s->current_tm.tm_mon = (int)from_bcd((uint8_t)value) - 1;
        }
        break;
    case BCDYEAR:
        if (s->reg_rtccon & RTC_ENABLE) {
            // 3 digits
            s->current_tm.tm_year = (int)from_bcd((uint8_t)value) +
                    (int)from_bcd((uint8_t)((value >> 8) & 0x0f)) * 100;
        }
        break;
*/
    default:
        if(offset >= RTC_BKP0R_OFFSET && offset <= RTC_BKP19R_OFFSET)
        {
            // BKP storage
            s->reg_bkp[offset - RTC_BKP0R_OFFSET] = (uint32_t)value;
            return;
        }

        fprintf(stderr,
                "[stm32-rtc: bad write offset " TARGET_FMT_plx "]\n",
                offset);
        break;

    }
}

/*
 * Set default values to timer fields and registers
 */
static void stm32_rtc_reset(DeviceState *d)
{
    STM32RTCState *s = (STM32RTCState *)d;

    qemu_get_timedate(&s->current_tm, 0);

    DPRINTF("Get time from host: %04d-%d-%d (%d) %2d:%02d:%02d\n",
            1900 + s->current_tm.tm_year, s->current_tm.tm_mon, s->current_tm.tm_mday, s->current_tm.tm_wday,
            s->current_tm.tm_hour, s->current_tm.tm_min, s->current_tm.tm_sec);
    //s->RTC_ISR = 0x00000007;
    s->state = RTC_LOCKED;
    s->RTC_TR = to_bcd((uint8_t)s->current_tm.tm_hour) << RTC_TR_HU_START |
                to_bcd((uint8_t)s->current_tm.tm_min) << RTC_TR_MNU_START |
                to_bcd((uint8_t)s->current_tm.tm_sec) << RTC_TR_SU_START;

    s->reg_intp = 0;
    s->reg_rtccon = 0;
    s->reg_ticcnt = 0;
    s->reg_rtcalm = 0;
    s->reg_almsec = 0;
    s->reg_almmin = 0;
    s->reg_almhour = 0;
    s->reg_almday = 0;
    s->reg_almmon = 0;
    s->reg_almyear = 0;

    s->reg_curticcnt = 0;

 //   s->RTC_ISR |= 0x00000020;
    s->RTC_ISR_RSF = 1;
    s->RTC_CR = 0x00000000;
    s->RTC_PRER = 0x007f00ff;

//    stm32_rtc_update_freq(s, );
//    ptimer_stop(s->ptimer);
/*    ptimer_set_count(s->ptimer_1Hz, RTC_BASE_FREQ);
    ptimer_run(s->ptimer_1Hz, 1);*/
}

static const MemoryRegionOps stm32_rtc_ops = {
    .read = stm32_rtc_read,
    .write = stm32_rtc_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

/*
 * RTC timer initialization
 */
static int stm32_rtc_init(SysBusDevice *dev)
{
    STM32RTCState *s = FROM_SYSBUS(STM32RTCState, dev);
    QEMUBH *bh;

    bh = qemu_bh_new(stm32_rtc_tick, s);
    s->ptimer = ptimer_init(bh);
    ptimer_set_freq(s->ptimer, RTC_BASE_FREQ);
    stm32_rtc_update_freq(s, 0);

    bh = qemu_bh_new(stm32_rtc_1Hz_tick, s);
    s->ptimer_1Hz = ptimer_init(bh);
    ptimer_set_freq(s->ptimer_1Hz, RTC_BASE_FREQ);

    sysbus_init_irq(dev, &s->alarm_irq);
    sysbus_init_irq(dev, &s->wkup_irq);
    sysbus_init_irq(dev, &s->tamp_stamp_irq);

    memory_region_init_io(&s->iomem, &stm32_rtc_ops, s, "stm32-rtc",
            STM32_RTC_REG_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    return 0;
}

static void stm32_rtc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_rtc_init;
    dc->reset = stm32_rtc_reset;
    dc->vmsd = &vmstate_stm32_rtc_state;
}

static const TypeInfo stm32_rtc_info = {
    .name          = "stm32-rtc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(STM32RTCState),
    .class_init    = stm32_rtc_class_init,
};

static void stm32_rtc_register_types(void)
{
    type_register_static(&stm32_rtc_info);
}

type_init(stm32_rtc_register_types)
