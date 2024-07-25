/*************************************************************************************************/
/*!
 * \file
 *
 * \brief  RTC clock implementation.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 *
 * Copyright (c) 2022-2023 Analog Devices, Inc.
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
 */
/*************************************************************************************************/

#include "pal_rtc.h"
#include "pal_led.h"
#include "pal_sys.h"
#include "wut.h"
#include "lp.h"
#include "mxc_device.h"
#include "wut_regs.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/
#define PAL_WUT MXC_WUT0
#define PAL_WUT_IRQn WUT0_IRQn
/**************************************************************************************************
  Global Functions
**************************************************************************************************/

/**************************************************************************************************
  Global Variables
**************************************************************************************************/
static struct {
  PalRtcState_t state;
} palRtcCb;

/*************************************************************************************************/
/*!
 *  \brief  Interrupt service routine.
 *
 *  \return None.
 */
/*************************************************************************************************/



#if PAL_WUT_IRQn == WUT0_IRQn
void WUT0_IRQHandler(void)
{
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);
  MXC_WUT_Handler();

}
#else
void WUT1_IRQHandler(void)
{
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);
  MXC_WUT_Handler();

}
#endif
/*************************************************************************************************/
/*!
 *  \brief  Get the state of the RTC.
 *
 *  \return PAL_RTC_STATE_READY if active, PAL_RTC_STATE_UNINIT if inactive.
 */
/*************************************************************************************************/
PalRtcState_t PalRtcGetState(void)
{
  return palRtcCb.state;
}

/*************************************************************************************************/
/*!
 *  \brief  Set the RTC capture compare value.
 *
 *  \param  channelId   Channel ID Number.
 *  \param  value   Set new value for compare value.
 */
/*************************************************************************************************/
void PalRtcCompareSet(uint8_t channelId, uint32_t value)
{
  MXC_WUT_SetCompare(value);
}

/*************************************************************************************************/
/*!
 *  \brief  Tickless timer initialization routine.
 */
/*************************************************************************************************/
void PalRtcInit(void)
{
  /* Init WUT */
  mxc_wut_cfg_t cfg;
  cfg.mode = MXC_WUT_MODE_COMPARE;
  cfg.cmp_cnt = PAL_MAX_RTC_COUNTER_VAL;

  MXC_WUT_Init(MXC_WUT_PRES_1);
  MXC_WUT_Config(&cfg);
  MXC_LP_EnableWUTAlarmWakeup();

  NVIC_ClearPendingIRQ(PAL_WUT_IRQn);
  NVIC_EnableIRQ(PAL_WUT_IRQn);

  /* Enable WUT */
  MXC_WUT_Enable();

  palRtcCb.state = PAL_RTC_STATE_READY;
}

/*************************************************************************************************/
/*!
 *  \brief  Get current time.
 *
 *  \return RTC time.
 *
 *  \note   Caution must be taken when performing calculations with RTC time. RTC value bit width
 *          and resolution may vary.
 */
/*************************************************************************************************/
uint32_t PalRtcCounterGet(void)
{
  uint32_t count = MXC_WUT_GetCount();

  return count;
}

/*************************************************************************************************/
/*!
 *  \brief  Function for starting the RTC timer.
 *
 *  \param  channelId   Channel ID Number.
 */
/*************************************************************************************************/
void PalRtcEnableCompareIrq(uint8_t channelId)
{
  NVIC_EnableIRQ(PAL_WUT_IRQn);
}

/*************************************************************************************************/
/*!
 *  \brief  Function for stopping the RTC timer.
 *
 *  \param  channelId   Channel ID Number.
 */
/*************************************************************************************************/
void PalRtcDisableCompareIrq(uint8_t channelId)
{
  MXC_WUT_IntClear();
  NVIC_DisableIRQ(PAL_WUT_IRQn);
}
