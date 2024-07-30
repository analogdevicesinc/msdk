/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#include <stdbool.h>
#include <stddef.h>

#include "gpio_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "rtc.h"
#include "rtc_reva.h"
#include "tmr.h"

#if TARGET_NUM == 32650
#include "pwrseq_regs.h"
#endif

static void MXC_RTC_RevA_waitBusyToClear(void)
{
    while (MXC_RTC_REVA_IS_BUSY) {}
}

int MXC_RTC_RevA_GetBusyFlag(mxc_rtc_reva_regs_t *rtc)
{
    if (MXC_RTC_REVA_IS_BUSY) {
        return E_BUSY;
    }
    return E_SUCCESS;
}

int MXC_RTC_RevA_EnableInt(mxc_rtc_reva_regs_t *rtc, uint32_t mask)
{
    mask &= (MXC_F_RTC_REVA_CTRL_TOD_ALARM_IE | MXC_F_RTC_REVA_CTRL_SSEC_ALARM_IE |
             MXC_F_RTC_REVA_CTRL_RDY_IE);

    if (!mask) {
        /* No bits set? Wasn't something we can enable. */
        return E_BAD_PARAM;
    }

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl |= mask;

    /* If TOD and SSEC interrupt enable, check busy after CTRL register write*/
    mask &= ~MXC_F_RTC_REVA_CTRL_RDY_IE;

    if (mask) {
        MXC_RTC_RevA_waitBusyToClear();
    }
    return E_SUCCESS;
}

int MXC_RTC_RevA_DisableInt(mxc_rtc_reva_regs_t *rtc, uint32_t mask)
{
    mask &= (MXC_F_RTC_REVA_CTRL_TOD_ALARM_IE | MXC_F_RTC_REVA_CTRL_SSEC_ALARM_IE |
             MXC_F_RTC_REVA_CTRL_RDY_IE);

    if (!mask) {
        /* No bits set? Wasn't something we can enable. */
        return E_BAD_PARAM;
    }

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~mask;

    /* If TOD and SSEC interrupt enable, check busy after CTRL register write*/
    mask &= ~MXC_F_RTC_REVA_CTRL_RDY_IE;

    if (mask) {
        MXC_RTC_RevA_waitBusyToClear();
    }
    return E_SUCCESS;
}

int MXC_RTC_RevA_SetTimeofdayAlarm(mxc_rtc_reva_regs_t *rtc, uint32_t ras)
{
    // ras can only be written if BUSY = 0 & (RTCE = 0 or ADE = 0);
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->toda = (ras << MXC_F_RTC_REVA_TODA_TOD_ALARM_POS) & MXC_F_RTC_REVA_TODA_TOD_ALARM;

    return E_SUCCESS;
}

int MXC_RTC_RevA_SetSubsecondAlarm(mxc_rtc_reva_regs_t *rtc, uint32_t rssa)
{
    // ras can only be written if BUSY = 0 & (RTCE = 0 or ASE = 0);
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->sseca = (rssa << MXC_F_RTC_REVA_SSECA_SSEC_ALARM_POS) & MXC_F_RTC_REVA_SSECA_SSEC_ALARM;

    return E_SUCCESS;
}

int MXC_RTC_RevA_Start(mxc_rtc_reva_regs_t *rtc)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_WR_EN; // Allow writing to registers

    MXC_RTC_RevA_waitBusyToClear();

    // Can only write if WE=1 and BUSY=0
    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_EN; // setting RTCE = 1

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_WR_EN; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_RTC_RevA_Stop(mxc_rtc_reva_regs_t *rtc)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_WR_EN; // Allow writing to registers

    MXC_RTC_RevA_waitBusyToClear();

    // Can only write if WE=1 and BUSY=0
    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_EN; // setting RTCE = 0

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_WR_EN; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_RTC_RevA_Init(mxc_rtc_reva_regs_t *rtc, uint32_t sec, uint32_t ssec)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->ctrl = MXC_F_RTC_REVA_CTRL_WR_EN; //  Allow Writes

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl = MXC_RTC_REVA_CTRL_RESET_DEFAULT; // Start with a Clean Register

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_WR_EN; // Set Write Enable, allow writing to reg.

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ssec = ssec;

    MXC_RTC_RevA_waitBusyToClear();

    rtc->sec = sec;

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_WR_EN; // Prevent Writing...

    return E_SUCCESS;
}

int MXC_RTC_RevA_SquareWave(mxc_rtc_reva_regs_t *rtc, mxc_rtc_reva_sqwave_en_t sqe,
                            mxc_rtc_freq_sel_t ft)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_WR_EN; // Allow writing to registers

    MXC_RTC_RevA_waitBusyToClear();

    if (sqe == MXC_RTC_REVA_SQUARE_WAVE_ENABLED) {
        if (ft == MXC_RTC_F_32KHZ) { // if 32KHz output is selected...
            rtc->oscctrl |= MXC_F_RTC_REVA_OSCCTRL_SQW_32K; // Enable 32KHz wave

            MXC_RTC_RevA_waitBusyToClear();

            rtc->ctrl |= MXC_F_RTC_REVA_CTRL_SQW_EN; // Enable output on the pin
        } else { // if 1Hz, 512Hz, 4KHz output is selected
            rtc->oscctrl &=
                ~MXC_F_RTC_REVA_OSCCTRL_SQW_32K; // Must make sure that the 32KHz is disabled

            MXC_RTC_RevA_waitBusyToClear();

            rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_SQW_SEL;

            MXC_RTC_RevA_waitBusyToClear();

            rtc->ctrl |= (MXC_F_RTC_REVA_CTRL_SQW_EN | ft); // Enable Sq. wave,
        }

        MXC_RTC_RevA_waitBusyToClear();

        rtc->ctrl |= MXC_F_RTC_REVA_CTRL_EN; // Enable Real Time Clock
    } else { // Turn off the square wave output on the pin
        rtc->oscctrl &=
            ~MXC_F_RTC_REVA_OSCCTRL_SQW_32K; // Must make sure that the 32KHz is disabled

        MXC_RTC_RevA_waitBusyToClear();

        rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_SQW_EN; // No sq. wave output
    }

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_WR_EN; // Disable Writing to register

    return E_SUCCESS;
}

int MXC_RTC_RevA_Trim(mxc_rtc_reva_regs_t *rtc, int8_t trim)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    rtc->ctrl |= MXC_F_RTC_REVA_CTRL_WR_EN;

    MXC_RTC_RevA_waitBusyToClear();

    MXC_SETFIELD(rtc->trim, MXC_F_RTC_REVA_TRIM_TRIM, trim << MXC_F_RTC_REVA_TRIM_TRIM_POS);

    MXC_RTC_RevA_waitBusyToClear();

    rtc->ctrl &= ~MXC_F_RTC_REVA_CTRL_WR_EN; // Disable Writing to register

    return E_SUCCESS;
}

int MXC_RTC_RevA_GetFlags(mxc_rtc_reva_regs_t *rtc)
{
    return rtc->ctrl & (MXC_F_RTC_REVA_CTRL_TOD_ALARM | MXC_F_RTC_REVA_CTRL_SSEC_ALARM |
                        MXC_F_RTC_REVA_CTRL_RDY);
}

int MXC_RTC_RevA_ClearFlags(mxc_rtc_reva_regs_t *rtc, int flags)
{
    rtc->ctrl &= ~(flags & (MXC_F_RTC_REVA_CTRL_TOD_ALARM | MXC_F_RTC_REVA_CTRL_SSEC_ALARM |
                            MXC_F_RTC_REVA_CTRL_RDY));

    return E_SUCCESS;
}

int MXC_RTC_RevA_GetSubSecond(mxc_rtc_reva_regs_t *rtc)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

#if TARGET_NUM == 32650
    int ssec;
    if (ChipRevision > 0xA1) {
        ssec = ((MXC_PWRSEQ->ctrl >> 12) & 0xF00) | (rtc->ssec & 0xFF);
    } else {
        ssec = rtc->ssec;
    }
    return ssec;
#else
    return rtc->ssec;
#endif
}

int MXC_RTC_RevA_GetSecond(mxc_rtc_reva_regs_t *rtc)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    }

    return rtc->sec;
}

int MXC_RTC_RevA_GetSubSeconds(mxc_rtc_reva_regs_t *rtc, uint32_t *ssec)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    } else if (ssec == NULL) {
        return E_NULL_PTR;
    }

#if TARGET_NUM == 32650
    uint32_t sub_sec;
    if (ChipRevision > 0xA1) {
        sub_sec = ((MXC_PWRSEQ->ctrl >> 12) & 0xF00) | (rtc->ssec & 0xFF);
    } else {
        sub_sec = rtc->ssec;
    }

    *ssec = sub_sec;
#else
    *ssec = rtc->ssec;
#endif

    return E_NO_ERROR;
}

int MXC_RTC_RevA_GetSeconds(mxc_rtc_reva_regs_t *rtc, uint32_t *sec)
{
    if (MXC_RTC_RevA_GetBusyFlag(rtc)) {
        return E_BUSY;
    } else if (sec == NULL) {
        return E_NULL_PTR;
    }

    *sec = rtc->sec;

    return E_NO_ERROR;
}

int MXC_RTC_RevA_GetTime(mxc_rtc_reva_regs_t *rtc, uint32_t *sec, uint32_t *subsec)
{
    int32_t temp_sec = 0;

    if (sec == NULL || subsec == NULL) {
        return E_NULL_PTR;
    }

    do {
        // Check if an update is about to happen.
        if (!(rtc->ctrl & MXC_F_RTC_REVA_CTRL_RDY)) {
            return E_BUSY;
        }

        // Read the seconds count.
        temp_sec = MXC_RTC_RevA_GetSecond(rtc);

        if (temp_sec == E_BUSY) {
            return E_BUSY;
        }

        // Check if an update is about to happen.
        if (!(rtc->ctrl & MXC_F_RTC_REVA_CTRL_RDY)) {
            return E_BUSY;
        }

        // Read the sub-seconds count.
        *subsec = MXC_RTC_RevA_GetSubSecond(rtc);

        // Check if an update is about to happen.
        if (!(rtc->ctrl & MXC_F_RTC_REVA_CTRL_RDY)) {
            return E_BUSY;
        }

        // Read the seconds count.
        *sec = MXC_RTC_RevA_GetSecond(rtc);

        // Repeat until a steady state is reached.
    } while (temp_sec != *sec);

    return E_NO_ERROR;
}

int MXC_RTC_RevA_TrimCrystal(mxc_rtc_reva_regs_t *rtc, mxc_tmr_regs_t *tmr)
{
    int err, ppm = 0;
    uint32_t sec = 0, ssec = 0, ctrl = 0;
    uint32_t sec_sample[MXC_RTC_REVA_TRIM_PERIODS + 1] = { 0 };
    uint32_t ssec_sample[MXC_RTC_REVA_TRIM_PERIODS + 1] = { 0 };
    bool rtc_en = true;

    if (!(rtc->ctrl & MXC_F_RTC_REVA_CTRL_EN)) { // If RTC not enable, initialize it
        rtc_en = false;

        // Save state
        while (MXC_RTC_RevA_GetTime(rtc, &sec, &ssec) < 0) {}
        while (rtc->ctrl & MXC_F_RTC_CTRL_BUSY) {}
        ctrl = rtc->ctrl;

        if ((err = MXC_RTC_Init(0, 0)) != E_NO_ERROR) {
            return err;
        }
        MXC_RTC_Start();
    }

    MXC_TMR_ClearFlags(tmr);
    MXC_TMR_Start(tmr); // Sample the RTC ticks in MXC_RTC_REVA_TRIM_PERIODS number of periods
    while (MXC_RTC_RevA_GetTime(rtc, &sec_sample[0], &ssec_sample[0]) < 0) {}

    for (int i = 1; i < (MXC_RTC_REVA_TRIM_PERIODS + 1); i++) {
        // Wait for time trim period to elapse
        while (!(MXC_TMR_GetFlags(tmr) & MXC_RTC_TRIM_TMR_IRQ)) {}

        // Take time sample
        while (MXC_RTC_RevA_GetTime(rtc, &sec_sample[i], &ssec_sample[i]) < 0) {}

        MXC_TMR_ClearFlags(tmr);
    }

    MXC_TMR_Stop(tmr); // Shutdown timer
    MXC_TMR_Shutdown(tmr);

    if (!rtc_en) { // If RTC wasn't enabled entering the function, restore state
        MXC_RTC_Stop();

        while (rtc->ctrl & MXC_F_RTC_REVA_CTRL_BUSY) {}
        MXC_SETFIELD(rtc->ssec, MXC_F_RTC_REVA_SSEC_SSEC, (ssec << MXC_F_RTC_REVA_SSEC_SSEC_POS));
        while (rtc->ctrl & MXC_F_RTC_REVA_CTRL_BUSY) {}
        MXC_SETFIELD(rtc->sec, MXC_F_RTC_REVA_SEC_SEC, (sec << MXC_F_RTC_REVA_SEC_SEC_POS));
        while (rtc->ctrl & MXC_F_RTC_REVA_CTRL_BUSY) {}
        rtc->ctrl = ctrl;
    }

    for (int i = 0; i < MXC_RTC_REVA_TRIM_PERIODS;
         i++) { // Get total error in RTC ticks over MXC_RTC_REVA_TRIM_PERIODS number of sample periods
        if (sec_sample[i] < sec_sample[i + 1]) {
            ppm += MXC_RTC_REVA_TICKS_PER_PERIOD -
                   ((MXC_RTC_MAX_SSEC - ssec_sample[i]) + ssec_sample[i + 1]);
        } else {
            ppm += MXC_RTC_REVA_TICKS_PER_PERIOD - (ssec_sample[i + 1] - ssec_sample[i]);
        }
    }

    ppm /= MXC_RTC_REVA_TRIM_PERIODS;
    ppm = PPM(ppm); // Convert total error to PPM and set trim
    if (ppm < -128 || ppm > 127) {
        return E_OVERFLOW;
    }

    return MXC_RTC_RevA_Trim(rtc, (int8_t)ppm); // Set Trim
}
