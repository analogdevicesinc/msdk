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

/* ***** Functions ***** */

int MXC_RTC_EnableInt(uint32_t mask)
{
    return MXC_RTC_RevA_EnableInt(MXC_RTC, mask);
}

int MXC_RTC_DisableInt(uint32_t mask)
{
    return MXC_RTC_RevA_DisableInt(MXC_RTC, mask);
}

int MXC_RTC_SetTimeofdayAlarm(uint32_t ras)
{
    return MXC_RTC_RevA_SetTimeofdayAlarm(MXC_RTC, ras);
}

int MXC_RTC_SetSubsecondAlarm(uint32_t rssa)
{
    return MXC_RTC_RevA_SetSubsecondAlarm(MXC_RTC, rssa);
}

int MXC_RTC_Start(void)
{
    return MXC_RTC_RevA_Start(MXC_RTC);
}

int MXC_RTC_Stop(void)
{
    return MXC_RTC_RevA_Stop(MXC_RTC);
}

int MXC_RTC_Init(uint32_t sec, uint8_t ssec)
{
    // Enable clock
    MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ERTCO_EN;

    return MXC_RTC_RevA_Init(MXC_RTC, sec, ssec);
}

int MXC_RTC_SquareWave(mxc_rtc_sqwave_en_t sqe, mxc_rtc_freq_sel_t ft)
{
    MXC_GPIO_Config(&gpio_cfg_rtcsqw);

    return MXC_RTC_RevA_SquareWave(MXC_RTC, sqe, ft);
}

int MXC_RTC_Trim(int8_t trm)
{
    return MXC_RTC_RevA_Trim(MXC_RTC, trm);
}

int MXC_RTC_GetFlags(void)
{
    return MXC_RTC_RevA_GetFlags();
}

int MXC_RTC_ClearFlags(int flags)
{
    return MXC_RTC_RevA_ClearFlags(flags);
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
    return MXC_RTC_RevA_GetTime(sec, subsec);
}

int MXC_RTC_GetBusyFlag(void)
{
    return MXC_RTC_RevA_GetBusyFlag();
}
