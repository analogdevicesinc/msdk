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

#include <stddef.h>
#include "rtc.h"
#include "rtc_reva.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "mxc_errors.h"

// *****************************************************************************
int MXC_RTC_SetTimeofdayAlarm(uint32_t ras)
{
    return MXC_RTC_RevA_SetTimeofdayAlarm((mxc_rtc_reva_regs_t *)MXC_RTC, ras);
}

// *****************************************************************************
int MXC_RTC_SetSubsecondAlarm(uint32_t rssa)
{
    return MXC_RTC_RevA_SetSubsecondAlarm((mxc_rtc_reva_regs_t *)MXC_RTC, rssa);
}

// *****************************************************************************
int MXC_RTC_Start(void)
{
    return MXC_RTC_RevA_Start((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_Stop(void)
{
    return MXC_RTC_RevA_Stop((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_Init(uint32_t sec, uint8_t ssec)
{
    MXC_SYS_RTCClockEnable();
    MXC_SYS_Reset_Periph(MXC_SYS_RESET_RTC);

    return MXC_RTC_RevA_Init((mxc_rtc_reva_regs_t *)MXC_RTC, sec, (ssec & MXC_F_RTC_SSEC_SSEC));
}

// *****************************************************************************
int MXC_RTC_SquareWaveStart(mxc_rtc_freq_sel_t fq)
{
    MXC_GPIO_Config(&gpio_cfg_rtcsqw);
    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t *)MXC_RTC, MXC_RTC_REVA_SQUARE_WAVE_ENABLED,
                                   fq);
}

// *****************************************************************************
int MXC_RTC_SquareWaveStop(void)
{
    return MXC_RTC_RevA_SquareWave((mxc_rtc_reva_regs_t *)MXC_RTC,
                                   MXC_RTC_REVA_SQUARE_WAVE_DISABLED, 0);
}

// *****************************************************************************
int MXC_RTC_EnableInt(uint32_t mask)
{
    return MXC_RTC_RevA_EnableInt((mxc_rtc_reva_regs_t *)MXC_RTC, mask);
}

// *****************************************************************************
int MXC_RTC_DisableInt(uint32_t mask)
{
    return MXC_RTC_RevA_DisableInt((mxc_rtc_reva_regs_t *)MXC_RTC, mask);
}

// *****************************************************************************
int MXC_RTC_Trim(int8_t trm)
{
    /* MAX32650 does not have RTC_TRIM register */
    return E_NOT_SUPPORTED;
}

// *****************************************************************************
int MXC_RTC_GetFlags(void)
{
    return MXC_RTC_RevA_GetFlags((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_ClearFlags(int flags)
{
    return MXC_RTC_RevA_ClearFlags((mxc_rtc_reva_regs_t *)MXC_RTC, flags);
}

// *****************************************************************************
int MXC_RTC_GetSubSecond(void)
{
    return MXC_RTC_RevA_GetSubSecond((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_GetSecond(void)
{
    return MXC_RTC_RevA_GetSecond((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_GetSubSeconds(uint32_t *ssec)
{
    // Ensure valid data is in SSEC register
    MXC_RTC->ctrl &= ~MXC_F_RTC_CTRL_READY;
    while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_READY)) {}

    return MXC_RTC_RevA_GetSubSeconds((mxc_rtc_reva_regs_t *)MXC_RTC, ssec);
}

// *****************************************************************************
int MXC_RTC_GetSeconds(uint32_t *sec)
{
    // Ensure valid data is in SEC register
    MXC_RTC->ctrl &= ~MXC_F_RTC_CTRL_READY;
    while (!(MXC_RTC->ctrl & MXC_F_RTC_CTRL_READY)) {}

    return MXC_RTC_RevA_GetSeconds((mxc_rtc_reva_regs_t *)MXC_RTC, sec);
}

// *****************************************************************************
int MXC_RTC_GetTime(uint32_t *sec, uint32_t *subsec)
{
    return MXC_RTC_RevA_GetTime((mxc_rtc_reva_regs_t *)MXC_RTC, sec, subsec);
}

// *****************************************************************************
int MXC_RTC_GetBusyFlag(void)
{
    return MXC_RTC_RevA_GetBusyFlag((mxc_rtc_reva_regs_t *)MXC_RTC);
}

// *****************************************************************************
int MXC_RTC_TrimCrystal(mxc_tmr_regs_t *tmr)
{
    /* MAX32650 does not have RTC_TRIM register */
    return E_NOT_SUPPORTED;
}
