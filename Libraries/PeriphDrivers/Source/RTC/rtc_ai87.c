/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include "mxc_device.h"
#include "rtc_regs.h"
#include "rtc.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "gpio_regs.h"
#include "mxc_errors.h"
#include "mcr_regs.h"
#include "rtc_reva.h"

/********************************************/
/* Maxim Function Mapping                   */
/********************************************/

int MXC_RTC_EnableInt(uint32_t mask)
{
    return MXC_RTC_RevA_EnableInt((mxc_rtc_reva_regs_t *)MXC_RTC, mask);
}

int MXC_RTC_DisableInt(uint32_t mask)
{
    return MXC_RTC_RevA_DisableInt((mxc_rtc_reva_regs_t *)MXC_RTC, mask);
}

int MXC_RTC_SetTimeofdayAlarm(uint32_t ras)
{
    return MXC_RTC_RevA_SetTimeofdayAlarm((mxc_rtc_reva_regs_t *)MXC_RTC, ras);
}

int MXC_RTC_SetSubsecondAlarm(uint32_t rssa)
{
    return MXC_RTC_RevA_SetSubsecondAlarm((mxc_rtc_reva_regs_t *)MXC_RTC, rssa);
}

int MXC_RTC_Start(void)
{
    return MXC_RTC_RevA_Start((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_Stop(void)
{
    return MXC_RTC_RevA_Stop((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_Init(uint32_t sec, uint16_t ssec)
{
    MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;

    return MXC_RTC_RevA_Init((mxc_rtc_reva_regs_t *)MXC_RTC, sec, (ssec & MXC_F_RTC_SSEC_SSEC));
}

int MXC_RTC_SquareWave(mxc_rtc_reva_sqwave_en_t sqe, mxc_rtc_freq_sel_t ft)
{
    MXC_GPIO_Config(&gpio_cfg_rtcsqw);

    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t *)MXC_RTC, sqe, ft);
}

int MXC_RTC_SquareWaveStart(mxc_rtc_freq_sel_t fq)
{
    MXC_GPIO_Config(&gpio_cfg_rtcsqw);

    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t *)MXC_RTC, MXC_RTC_REVA_SQUARE_WAVE_ENABLED,
                                   fq);
}

int MXC_RTC_SquareWaveStop(void)
{
    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t *)MXC_RTC,
                                   MXC_RTC_REVA_SQUARE_WAVE_DISABLED, 0);
}

int MXC_RTC_Trim(int8_t trm)
{
    return MXC_RTC_RevA_Trim((mxc_rtc_reva_regs_t *)MXC_RTC, trm);
}

int MXC_RTC_GetFlags(void)
{
    return MXC_RTC_RevA_GetFlags((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_ClearFlags(int flags)
{
    return MXC_RTC_RevA_ClearFlags((mxc_rtc_reva_regs_t *)MXC_RTC, flags);
}

int MXC_RTC_GetSubSecond(void)
{
    return MXC_RTC_RevA_GetSubSecond((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_GetSecond(void)
{
    return MXC_RTC_RevA_GetSecond((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_GetSubSeconds(uint32_t *ssec)
{
    MXC_RTC->ctrl &= ~MXC_F_RTC_CTRL_RDY; // Ensure valid data is in SSEC register
    while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_RDY)) {}

    return MXC_RTC_RevA_GetSubSeconds((mxc_rtc_reva_regs_t *)MXC_RTC, ssec);
}

int MXC_RTC_GetSeconds(uint32_t *sec)
{
    MXC_RTC->ctrl &= ~MXC_F_RTC_CTRL_RDY; // Ensure valid data is in SEC register
    while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_RDY)) {}

    return MXC_RTC_RevA_GetSeconds((mxc_rtc_reva_regs_t *)MXC_RTC, sec);
}

int MXC_RTC_GetTime(uint32_t *sec, uint32_t *subsec)
{
    return MXC_RTC_RevA_GetTime((mxc_rtc_reva_regs_t *)MXC_RTC, sec, subsec);
}

int MXC_RTC_GetBusyFlag(void)
{
    return MXC_RTC_RevA_GetBusyFlag((mxc_rtc_reva_regs_t *)MXC_RTC);
}

int MXC_RTC_TrimCrystal(mxc_tmr_regs_t *tmr)
{
    int err;
    uint32_t sys_clk;

    if (MXC_TMR_GET_IDX(tmr) < 0 ||
        MXC_TMR_GET_IDX(tmr) > 3) { // Timer must support PCLK (ERFO) as clock source
        return E_BAD_PARAM;
    }

    sys_clk = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
              MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS; // Save current system clock
    if (sys_clk != MXC_SYS_CLOCK_EBO) { // Change system clock to reference oscillator source
        if ((err = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_EBO)) != E_NO_ERROR) {
            return err;
        }
    }

    mxc_tmr_cfg_t
        tmr_cfg; // Configure timer to trigger each interrupt NUM_PERIOD number of times within a second
    tmr_cfg.pres = MXC_TMR_PRES_1;
    tmr_cfg.mode = MXC_TMR_MODE_CONTINUOUS;
    tmr_cfg.bitMode = MXC_TMR_BIT_MODE_32;
    tmr_cfg.clock = MXC_TMR_APB_CLK;
    tmr_cfg.cmp_cnt = PeripheralClock / MXC_RTC_REVA_TRIM_PERIODS;
    tmr_cfg.pol = 0;
    MXC_TMR_Init(tmr, &tmr_cfg, false);

    err = MXC_RTC_RevA_TrimCrystal((mxc_rtc_reva_regs_t *)MXC_RTC, tmr);

    if (sys_clk != MXC_SYS_CLOCK_EBO) { // Restore previous system clock if we changed it
        MXC_SYS_Clock_Select(sys_clk);
    }

    return err;
}
