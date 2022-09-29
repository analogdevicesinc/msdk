/* *****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 **************************************************************************** */

#include "tmr.h"
#include "tmr_reva.h"
#include "tmr_common.h"

void MXC_TMR_Init(mxc_tmr_regs_t *tmr, mxc_tmr_cfg_t *cfg)
{
    int tmr_id = MXC_TMR_GET_IDX(tmr);
    MXC_ASSERT(tmr_id >= 0);

    //enable peripheral clock and configure gpio pins
    switch (tmr_id) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER0);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER0) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T0);
        MXC_GPIO_Config(&gpio_cfg_tmr0);
        break;

    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER1);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER1) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T1);
        MXC_GPIO_Config(&gpio_cfg_tmr1);
        break;

    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER2);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER2) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T2);
        MXC_GPIO_Config(&gpio_cfg_tmr2);
        break;

    case 3:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER3);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER3) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T3);
        MXC_GPIO_Config(&gpio_cfg_tmr3);
        break;

    case 4:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER4);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER4) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T4);
        MXC_GPIO_Config(&gpio_cfg_tmr4);
        break;

    case 5:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_TIMER5);
        while (MXC_GCR->rstr0 & MXC_F_GCR_RSTR0_TIMER5) {}
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_T5);
        MXC_GPIO_Config(&gpio_cfg_tmr5);
        break;
    }

    MXC_TMR_RevA_Init((mxc_tmr_reva_regs_t *)tmr, cfg);
}

void MXC_TMR_Shutdown(mxc_tmr_regs_t *tmr)
{
    int tmr_id = MXC_TMR_GET_IDX(tmr);
    MXC_ASSERT(tmr_id >= 0);

    MXC_TMR_RevA_Shutdown((mxc_tmr_reva_regs_t *)tmr);

    // System settigns
    // disable peripheral clock
    switch (tmr_id) {
    case 0:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T0);
        break;

    case 1:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T1);
        break;

    case 2:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T2);
        break;

    case 3:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T3);
        break;

    case 4:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T4);
        break;

    case 5:
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_T5);
        break;
    }
}

void MXC_TMR_Start(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_Start((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_Stop((mxc_tmr_reva_regs_t *)tmr);
}

int MXC_TMR_SetPWM(mxc_tmr_regs_t *tmr, uint32_t pwm)
{
    return MXC_TMR_RevA_SetPWM((mxc_tmr_reva_regs_t *)tmr, pwm);
}

uint32_t MXC_TMR_GetCompare(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCompare((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetCapture(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCapture((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetCount(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetCount((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_ClearFlags(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_RevA_ClearFlags((mxc_tmr_reva_regs_t *)tmr);
}

uint32_t MXC_TMR_GetFlags(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_RevA_GetFlags((mxc_tmr_reva_regs_t *)tmr);
}

void MXC_TMR_SetCompare(mxc_tmr_regs_t *tmr, uint32_t cmp_cnt)
{
    MXC_TMR_RevA_SetCompare((mxc_tmr_reva_regs_t *)tmr, cmp_cnt);
}

void MXC_TMR_SetCount(mxc_tmr_regs_t *tmr, uint32_t cnt)
{
    MXC_TMR_RevA_SetCount((mxc_tmr_reva_regs_t *)tmr, cnt);
}

void MXC_TMR_Delay(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_Common_Delay(tmr, us);
}

void MXC_TMR_TO_Start(mxc_tmr_regs_t *tmr, uint32_t us)
{
    MXC_TMR_RevA_TO_Start((mxc_tmr_reva_regs_t *)tmr, us);
}

int MXC_TMR_TO_Check(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Check(tmr);
}

void MXC_TMR_TO_Stop(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Stop(tmr);
}

void MXC_TMR_TO_Clear(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_TO_Clear(tmr);
}

unsigned int MXC_TMR_TO_Elapsed(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Elapsed(tmr);
}

unsigned int MXC_TMR_TO_Remaining(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_TO_Remaining(tmr);
}

void MXC_TMR_SW_Start(mxc_tmr_regs_t *tmr)
{
    MXC_TMR_Common_SW_Start(tmr);
}

unsigned int MXC_TMR_SW_Stop(mxc_tmr_regs_t *tmr)
{
    return MXC_TMR_Common_SW_Stop(tmr);
}

int MXC_TMR_GetTime(mxc_tmr_regs_t *tmr, uint32_t ticks, uint32_t *time, mxc_tmr_unit_t *units)
{
    return MXC_TMR_RevA_GetTime((mxc_tmr_reva_regs_t *)tmr, ticks, time, units);
}
