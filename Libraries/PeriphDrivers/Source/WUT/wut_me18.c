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

/* **** Includes **** */
#include <stddef.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "wut.h"
#include "wut_reva.h"
#include "trimsir_regs.h"

/* **** Definitions **** */

/* Clock rate the BLE DBB counter */
#ifndef BB_CLK_RATE_HZ
#define BB_CLK_RATE_HZ 1000000
#endif

/* Higher values will produce a more accurate measurement, but will consume more power */
#define WUT_TRIM_TICKS 0x1000

/* **** Globals **** */

/* **** Local Variables **** */

/* Used for the asynchronous trim procedure */
static uint32_t wutCnt0_async, snapshot0_async, bestTrim_async, bestDiff_async;
static int capAdded_async;
static int trimPending;
static mxc_wut_complete_cb_t cb_async;

/* **** Functions **** */

/* ************************************************************************** */
void MXC_WUT_Init(mxc_wut_regs_t *wut, mxc_wut_pres_t pres)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
#endif
    MXC_WUT_RevA_Init((mxc_wut_reva_regs_t *)wut, (mxc_wut_reva_pres_t)pres);
}

/* ************************************************************************** */
void MXC_WUT_Shutdown(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_Shutdown((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_Enable(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_Enable((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_Disable(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_Disable((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_Config(mxc_wut_regs_t *wut, const mxc_wut_cfg_t *cfg)
{
    MXC_WUT_RevA_Config((mxc_wut_reva_regs_t *)wut, (mxc_wut_reva_cfg_t *)cfg);
}

/* ************************************************************************** */
uint32_t MXC_WUT_GetCompare(mxc_wut_regs_t *wut)
{
    return MXC_WUT_RevA_GetCompare((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
uint32_t MXC_WUT_GetCapture(mxc_wut_regs_t *wut)
{
    return MXC_WUT_RevA_GetCapture((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************* */
uint32_t MXC_WUT_GetCount(mxc_wut_regs_t *wut)
{
    return MXC_WUT_RevA_GetCount((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************* */
void MXC_WUT_ClearFlags(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_IntClear((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************* */
uint32_t MXC_WUT_GetFlags(mxc_wut_regs_t *wut)
{
    return MXC_WUT_RevA_IntStatus((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************* */
void MXC_WUT_SetCompare(mxc_wut_regs_t *wut, uint32_t cmp_cnt)
{
    MXC_WUT_RevA_SetCompare((mxc_wut_reva_regs_t *)wut, cmp_cnt);
}

/* ************************************************************************* */
void MXC_WUT_SetCount(mxc_wut_regs_t *wut, uint32_t cnt)
{
    MXC_WUT_RevA_SetCount((mxc_wut_reva_regs_t *)wut, cnt);
}

/* ************************************************************************* */
int MXC_WUT_GetTicks(mxc_wut_regs_t *wut, uint32_t time, mxc_wut_unit_t units, uint32_t *ticks)
{
    return MXC_WUT_RevA_GetTicks((mxc_wut_reva_regs_t *)wut, ERTCO_FREQ, time,
                                 (mxc_wut_reva_unit_t)units, ticks);
}

/* ************************************************************************* */
int MXC_WUT_GetTime(mxc_wut_regs_t *wut, uint32_t ticks, uint32_t *time, mxc_wut_unit_t *units)
{
    return MXC_WUT_RevA_GetTime((mxc_wut_reva_regs_t *)wut, ERTCO_FREQ, ticks, time,
                                (mxc_wut_reva_unit_t *)units);
}

/* ************************************************************************** */
void MXC_WUT_WaitForEdge(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_Edge((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_StoreCount(mxc_wut_regs_t *wut)
{
    MXC_WUT_RevA_Store((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_RestoreBBClock(mxc_wut_regs_t *wut, uint32_t dbbFreq)
{
    MXC_WUT_RevA_RestoreBBClock((mxc_wut_reva_regs_t *)wut, dbbFreq, ERTCO_FREQ);
}

/* ************************************************************************** */
uint32_t MXC_WUT_GetSleepTicks(mxc_wut_regs_t *wut)
{
    return MXC_WUT_RevA_GetSleepTicks((mxc_wut_reva_regs_t *)wut);
}

/* ************************************************************************** */
void MXC_WUT_Delay_MS(mxc_wut_regs_t *wut, uint32_t waitMs)
{
    MXC_WUT_RevA_Delay_MS((mxc_wut_reva_regs_t *)wut, waitMs, ERTCO_FREQ);
}

/* ************************************************************************** */
static void MXC_WUT_GetWUTSync(mxc_wut_regs_t *wut, uint32_t *wutCnt, uint32_t *snapshot)
{
    MXC_WUT_RevA_Edge((mxc_wut_reva_regs_t *)wut);
    *wutCnt = wut->cnt;
    *snapshot = wut->snapshot;
}

/* ************************************************************************** */
static void MXC_WUT_SetTrim(uint32_t trimValue)
{
    MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_RTCX1,
                 (trimValue << MXC_F_TRIMSIR_RTC_RTCX1_POS));
    MXC_SETFIELD(MXC_TRIMSIR->rtc, MXC_F_TRIMSIR_RTC_RTCX2,
                 (trimValue << MXC_F_TRIMSIR_RTC_RTCX2_POS));
}

/* ************************************************************************** */
static int MXC_WUT_StartTrim(mxc_wut_regs_t *wut)
{
    uint32_t wutCnt0, wutCnt1;
    uint32_t snapshot0, snapshot1;
    uint32_t trimValue;

    /* Make sure the WUT is running in compare mode */
    if (!(wut->ctrl & MXC_F_WUT_REVA_CTRL_TEN)) {
        return E_UNINITIALIZED;
    }

    if ((wut->ctrl & MXC_F_WUT_CTRL_TMODE) != MXC_S_WUT_CTRL_TMODE_COMPARE) {
        return E_UNINITIALIZED;
    }

    /* Make sure that DBB counter is running */
    MXC_WUT_GetWUTSync(wut, &wutCnt0, &snapshot0);
    MXC_WUT_GetWUTSync(wut, &wutCnt1, &snapshot1);
    if (snapshot0 == snapshot1) {
        return E_UNINITIALIZED;
    }

    /* Start with existing trim value */
    trimValue = (MXC_TRIMSIR->rtc & MXC_F_TRIMSIR_RTC_RTCX1) >> MXC_F_TRIMSIR_RTC_RTCX1_POS;
    MXC_WUT_SetTrim(trimValue);

    /* Initialize the variables */
    bestTrim_async = trimValue;
    bestDiff_async = 0xFFFF;

    /* Get the initial snapshot */
    MXC_WUT_GetWUTSync(wut, &wutCnt0_async, &snapshot0_async);

    trimPending = 1;

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_WUT_Handler(mxc_wut_regs_t *wut)
{
    uint32_t wutCnt1;
    uint32_t snapshot1;
    uint32_t trimValue;
    uint32_t snapTicks, wutTicks;
    uint64_t calcTicks;
    int trimComplete;
    mxc_wut_complete_cb_t cbTemp;

    /* Clear the interrupt flags */
    MXC_WUT_ClearFlags(wut);

    if (!trimPending) {
        return E_NO_ERROR;
    }

    /* Store the snapshot */
    MXC_WUT_GetWUTSync(wut, &wutCnt1, &snapshot1);
    snapTicks = snapshot1 - snapshot0_async;
    wutTicks = wutCnt1 - wutCnt0_async;

    /* Calculate the ideal number of DBB ticks in WUT_TRIM_TICKS */
    calcTicks = ((uint64_t)wutTicks * (uint64_t)BB_CLK_RATE_HZ) / (uint64_t)32768;

    trimComplete = 0;
    trimValue = (MXC_TRIMSIR->rtc & MXC_F_TRIMSIR_RTC_RTCX1) >> MXC_F_TRIMSIR_RTC_RTCX1_POS;

    if (snapTicks > calcTicks) {
        /* See if we're closer to the calculated value */
        if ((snapTicks - calcTicks) <= bestDiff_async) {
            bestDiff_async = snapTicks - calcTicks;
            bestTrim_async = trimValue;
        }

        /* Running slow, reduce cap */
        if (trimValue == 0) {
            /* We're maxed out on trim range */
            trimComplete = 1;
        }
        trimValue--;

        if (capAdded_async == 1) {
            /* We've hit an inflection point */
            trimComplete = 1;
        }
        capAdded_async = -1;

    } else if (snapTicks < calcTicks) {
        /* See if we're closer to the calculated value */
        if ((calcTicks - snapTicks) <= bestDiff_async) {
            bestDiff_async = calcTicks - snapTicks;
            bestTrim_async = trimValue;
        }

        /* Running fast, increase cap */
        if (trimValue == 0x1f) {
            /* We're maxed out on trim range */
            trimComplete = 1;
        }
        trimValue++;

        if (capAdded_async == -1) {
            /* We've hit an inflection point */
            trimComplete = 1;
        }
        capAdded_async = 1;

    } else {
        /* Just right */
        bestTrim_async = trimValue;
        trimComplete = 1;
    }

    if (trimComplete) {
        /* Apply the best trim value */
        MXC_WUT_SetTrim(bestTrim_async);

        trimPending = 0;

        /* Call the callback */
        if (cb_async != NULL) {
            cbTemp = cb_async;
            cb_async = NULL;
            cbTemp(E_NO_ERROR);
        }

        return E_NO_ERROR;
    }

    /* Start the next step */
    MXC_WUT_SetTrim(trimValue);
    MXC_WUT_GetWUTSync(wut, &wutCnt0_async, &snapshot0_async);

    if (cb_async != NULL) {
        /* Prime the compare interrupt */
        wut->cmp = wut->cnt + WUT_TRIM_TICKS - 1;
    }

    /* Return E_BUSY to indicate the trim procedure is still running */
    return E_BUSY;
}

/* ************************************************************************** */
int MXC_WUT_TrimCrystal(mxc_wut_regs_t *wut)
{
    int err, i;

    /* Clear the async callback pointer */
    cb_async = NULL;

    /* Start the trim procedure */
    err = MXC_WUT_StartTrim(wut);
    if (err != E_NO_ERROR) {
        return err;
    }
    do {
        for (i = 0; i < (WUT_TRIM_TICKS - 1); i++) {
            MXC_WUT_RevA_Edge((mxc_wut_reva_regs_t *)wut);
        }
    } while (MXC_WUT_Handler(wut) != E_NO_ERROR);

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_WUT_TrimCrystalAsync(mxc_wut_regs_t *wut, mxc_wut_complete_cb_t cb)
{
    int err;

    if (cb == NULL) {
        return E_NULL_PTR;
    }

    /* Save the callback */
    cb_async = cb;

    /* Start the trim procedure */
    err = MXC_WUT_StartTrim(wut);
    if (err != E_NO_ERROR) {
        return err;
    }

    /* Prime the compare interrupt */
    wut->cmp = wut->cnt + WUT_TRIM_TICKS - 1;

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_WUT_TrimPending(mxc_wut_regs_t *wut)
{
    if (trimPending) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}
