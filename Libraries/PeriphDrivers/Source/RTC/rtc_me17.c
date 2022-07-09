/* ****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 *
 **************************************************************************** */

#include "mxc_device.h"
#include "rtc_regs.h"
#include "rtc.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "gpio_regs.h"
#include "mxc_errors.h"
#include "mcr_regs.h"
#include "rtc_reva.h"
#include "tmr.h"
#include "trimsir_regs.h"

#define SUBSECOND_MSEC_0        200
#define SEARCH_STEPS            7
#define SEARCH_TARGET           0x30d400  /* 1/2 of 32 MHz periods in 32.768 kHz */

#define RTCX1x_MASK             0x1F /* 5 bits */
#define RTCX2x_MASK             0x1F /* 5 bits */

#define NOM_32K_FREQ            32768
#define TICKS_PER_RTC           122

/* Converts a time in milleseconds to the equivalent RSSA register value. */
#define MSEC_TO_RSSA(x)         (unsigned int)(0x100000000ULL    - ((x * 4096) / 1000))

/********************************************/
/* Maxim Function Mapping                   */
/********************************************/

int MXC_RTC_EnableInt(uint32_t mask)
{
    return MXC_RTC_RevA_EnableInt((mxc_rtc_reva_regs_t*) MXC_RTC, mask);
}

int MXC_RTC_DisableInt(uint32_t mask)
{
    return MXC_RTC_RevA_DisableInt((mxc_rtc_reva_regs_t*) MXC_RTC, mask);
}

int MXC_RTC_SetTimeofdayAlarm(uint32_t ras)
{
    return MXC_RTC_RevA_SetTimeofdayAlarm((mxc_rtc_reva_regs_t*) MXC_RTC, ras);
}

int MXC_RTC_SetSubsecondAlarm(uint32_t rssa)
{
    return MXC_RTC_RevA_SetSubsecondAlarm((mxc_rtc_reva_regs_t*) MXC_RTC, rssa);
}

int MXC_RTC_Start(void)
{
    return MXC_RTC_RevA_Start((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_Stop(void)
{
    return MXC_RTC_RevA_Stop((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_Init(uint32_t sec, uint8_t ssec)
{
    MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;
    
    return MXC_RTC_RevA_Init((mxc_rtc_reva_regs_t*) MXC_RTC, sec, ssec);
}

int MXC_RTC_SquareWaveStart(mxc_rtc_freq_sel_t ft)
{
    MXC_GPIO_Config(&gpio_cfg_rtcsqw);
    MXC_MCR->outen |= MXC_F_MCR_OUTEN_SQWOUT_EN;

    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t*) MXC_RTC, MXC_RTC_REVA_SQUARE_WAVE_ENABLED, ft);
}

int MXC_RTC_SquareWaveStop(void)
{   
    MXC_MCR->outen &= ~(MXC_F_MCR_OUTEN_SQWOUT_EN);

    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t*) MXC_RTC, MXC_RTC_REVA_SQUARE_WAVE_DISABLED, 0);
}

int MXC_RTC_Trim(int8_t trm)
{
    return MXC_RTC_RevA_Trim((mxc_rtc_reva_regs_t*) MXC_RTC, trm);
}

int MXC_RTC_GetFlags(void)
{
    return MXC_RTC_RevA_GetFlags((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_ClearFlags(int flags)
{
    return MXC_RTC_RevA_ClearFlags((mxc_rtc_reva_regs_t*) MXC_RTC, flags);
}

int MXC_RTC_GetSubSecond(void)
{
    return MXC_RTC_RevA_GetSubSecond((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_GetSecond(void)
{
    return MXC_RTC_RevA_GetSecond((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_GetTime(uint32_t* sec, uint32_t* subsec)
{
    return MXC_RTC_RevA_GetTime((mxc_rtc_reva_regs_t*) MXC_RTC, sec, subsec);
}

int MXC_RTC_GetBusyFlag(void)
{
    return MXC_RTC_RevA_GetBusyFlag((mxc_rtc_reva_regs_t*) MXC_RTC);
}

int MXC_RTC_TrimCrystal(void)
{
  #if TARGET_NUM == 78000
    /* MAX78000 does not have the ERFO clock which the Trim function requires */
    return E_NOT_SUPPORTED;
  #endif 

    unsigned int search_step, elapsed;
    unsigned int upper, lower, trim, oldtrim, bestTrim, bestElapsed, bestElapsedDiff;
    unsigned int freq = NOM_32K_FREQ;
    int retval;

    /* Determine starting point for internal load capacitors */
    upper = RTCX1x_MASK;
    lower = 0;
    trim = (upper + lower) / 2;

    /* Initialize best trim variables */
    bestTrim = trim;
    bestElapsed = bestElapsedDiff = SEARCH_TARGET;

    /* Init timer to count 32 MHz periods */
    mxc_tmr_cfg_t tmr_cfg;
    tmr_cfg.pres = TMR_PRES_1;
    tmr_cfg.mode = TMR_MODE_CONTINUOUS;
    tmr_cfg.bitMode = TMR_BIT_MODE_32;
    tmr_cfg.clock = MXC_TMR_APB_CLK;
    tmr_cfg.cmp_cnt = 0xFFFFFFFF;
    tmr_cfg.pol = 0;
    MXC_TMR_Init(MXC_TMR3, &tmr_cfg, FALSE);

    /* Clear out any previous configuration */
    MXC_RTC_DisableInt(MXC_F_RTC_CTRL_TOD_ALARM_IE | MXC_F_RTC_CTRL_SSEC_ALARM_IE |
        MXC_F_RTC_CTRL_RDY_IE);
    MXC_RTC_ClearFlags(MXC_RTC_GetFlags());

    MXC_RTC->oscctrl &= ~(MXC_F_RTC_OSCCTRL_BYPASS | MXC_F_RTC_OSCCTRL_SQW_32K);

    /* Setup SSEC Alarm */
    MXC_RTC_DisableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE);
    retval = MXC_RTC_SetSubsecondAlarm(MSEC_TO_RSSA(SUBSECOND_MSEC_0));
    if (retval != E_NO_ERROR) {
        return retval;
    }
    MXC_RTC_EnableInt(MXC_F_RTC_CTRL_SSEC_ALARM_IE);

    /* Trim loop */
    search_step = 0;
    while (search_step < SEARCH_STEPS) {
        /* Set new trim point */
        oldtrim = trim;
        trim = (lower + upper) / 2;
        if ((search_step > 0) && (trim == oldtrim)) {
            /* Found trim value */
            break;
        }
        
        /* Set the trim values */
        MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_X1TRIM, (trim << MXC_F_TRIMSIR_RTC_X1TRIM_POS));
        MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_X2TRIM, (trim << MXC_F_TRIMSIR_RTC_X2TRIM_POS));

        /* Sleep to settle new caps */
        MXC_Delay(MXC_DELAY_MSEC(10));

        /* Start 200 msec sampling window */
        MXC_TMR_Stop(MXC_TMR3);
        MXC_TMR_SetCount(MXC_TMR3, 0);

        /* Wait for an RTC edge */
        MXC_RTC_ClearFlags(MXC_RTC_GetFlags());
        while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_SSEC_ALARM)) {}

        MXC_TMR_Start(MXC_TMR3);

        /* Wait for an RTC edge */
        MXC_RTC_ClearFlags(MXC_RTC_GetFlags());
        while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_SSEC_ALARM)) {}

        /* Capture the TMR count and adjust for processing delay */
        elapsed = MXC_TMR_GetCount(MXC_TMR3);
        MXC_TMR_Stop(MXC_TMR3);
        elapsed += 810;

        /* Binary search for optimal trim value */
        if (elapsed > SEARCH_TARGET) {
            /* Too slow */
            upper = trim;

            /* Record best setting */
            if((elapsed - SEARCH_TARGET) <= bestElapsedDiff) {
                bestElapsedDiff = elapsed - SEARCH_TARGET;
                bestElapsed = elapsed;
                bestTrim = trim;
            }
        } else {
            /* Too fast */
            lower = trim;

            /* Record best setting */
            if((SEARCH_TARGET - elapsed) <= bestElapsedDiff) {
                bestElapsedDiff = SEARCH_TARGET - elapsed;
                bestElapsed = elapsed;
                bestTrim = trim;
            }
        }

        search_step++;
    }

    /* Apply the closest trim setting */
    MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_X1TRIM, (bestTrim << MXC_F_TRIMSIR_RTC_X1TRIM_POS));
    MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_X2TRIM, (bestTrim << MXC_F_TRIMSIR_RTC_X2TRIM_POS));

    /* Adjust 32K freq if we can't get close enough to 32768 Hz */
    if(bestElapsed >= SEARCH_TARGET) {
        freq -= (((bestElapsed - SEARCH_TARGET) + (TICKS_PER_RTC/2-1)) / TICKS_PER_RTC);
    } else {
        freq += (((SEARCH_TARGET - bestElapsed) + (TICKS_PER_RTC/2-1)) / TICKS_PER_RTC);
    }

    /* Clear hardware state */
    MXC_TMR_Stop(MXC_TMR3);
    MXC_TMR_Shutdown(MXC_TMR3);
    MXC_RTC_ClearFlags(MXC_RTC_GetFlags());

    return freq;
}
