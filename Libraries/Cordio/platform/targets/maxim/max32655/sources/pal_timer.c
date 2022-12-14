/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      Timer driver
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
 */
/*************************************************************************************************/

#include "pal_timer.h"
#include "pal_led.h"
#include "pal_sys.h"
#include "pal_bb.h"
#include "tmr.h"
#include "lp.h"
#include "gcr_regs.h"
#include "mxc_device.h"
#include "wsf_trace.h"
#include "pal_rtc.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/
#ifdef DEBUG

/*! \brief      Parameter and state check. */
#define PAL_TIMER_CHECK(expr)         { if (!(expr)) { palTimerCb.state = PAL_TIMER_STATE_ERROR; while(1); } }

#else

/*! \brief      Parameter and state check (disabled). */
#define PAL_TIMER_CHECK(expr)

#endif

#ifndef PAL_TMR_IDX
#define PAL_TMR_IDX                     0
#endif

#ifndef PAL_SLEEP_TMR_IDX
#define PAL_SLEEP_TMR_IDX               1
#endif

#if (PAL_TMR_IDX==PAL_SLEEP_TMR_IDX)
#error "Must use a different timer for sleep"
#endif

#define PAL_TMR                         MXC_TMR_GET_TMR(PAL_TMR_IDX)
#define PAL_TMR_IRQn                    MXC_TMR_GET_IRQ(PAL_TMR_IDX)

#define PAL_SLEEP_TMR                   MXC_TMR_GET_TMR(PAL_SLEEP_TMR_IDX)
#define PAL_SLEEP_TMR_IRQn              MXC_TMR_GET_IRQ(PAL_SLEEP_TMR_IDX)

#define PAL_TMR_SETUP_TICKS             9

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      Scheduler timer driver control block. */
static struct {
  PalTimerState_t     state;           /*!< State. */
  PalTimerCompCback_t expCback;        /*!< Timer expiry call back function. */
} palTimerCb;

/**************************************************************************************************
  External Functions: Initialization
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Timer interrupt handler dedicated to scheduler timer.
 *
 *  \return     None.
 */
/*************************************************************************************************/
#if   (PAL_TMR_IDX == 5)
void TMR5_IRQHandler(void)
#elif (PAL_TMR_IDX == 4)
void TMR4_IRQHandler(void)
#elif (PAL_TMR_IDX == 3)
void TMR3_IRQHandler(void)
#elif (PAL_TMR_IDX == 2)
void TMR2_IRQHandler(void)
#elif (PAL_TMR_IDX == 1)
void TMR1_IRQHandler(void)
#else
void TMR0_IRQHandler(void)
#endif
{
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);

  /* Check hardware status */
  PAL_TIMER_CHECK(palTimerCb.state == PAL_TIMER_STATE_BUSY);

  /* Disable and clear PAL_TMR interrupt */
  NVIC_DisableIRQ(PAL_TMR_IRQn);
  MXC_TMR_ClearFlags(PAL_TMR);

  palTimerCb.state = PAL_TIMER_STATE_READY;

  if (palTimerCb.expCback) {
    palTimerCb.expCback();
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Timer interrupt handler dedicated to sleep timer.
 *
 *  \return     None.
 */
/*************************************************************************************************/
#if   (PAL_SLEEP_TMR_IDX == 5)
void TMR5_IRQHandler(void)
#elif (PAL_SLEEP_TMR_IDX == 4)
void TMR4_IRQHandler(void)
#elif (PAL_SLEEP_TMR_IDX == 3)
void TMR3_IRQHandler(void)
#elif (PAL_SLEEP_TMR_IDX == 2)
void TMR2_IRQHandler(void)
#elif (PAL_SLEEP_TMR_IDX == 1)
void TMR1_IRQHandler(void)
#else
void TMR0_IRQHandler(void)
#endif
{
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);

  /* Disable and clear PAL_TMR interrupt */
  NVIC_DisableIRQ(PAL_SLEEP_TMR_IRQn);
  MXC_TMR_ClearFlags(PAL_SLEEP_TMR);
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize timer resources.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerInit(PalTimerCompCback_t expCback)
{
  mxc_tmr_cfg_t tmr_cfg;

  PAL_TIMER_CHECK(palTimerCb.state == PAL_TIMER_STATE_UNINIT);
  PAL_TIMER_CHECK(expCback != NULL);

  tmr_cfg.pres = TMR_PRES_1;
  tmr_cfg.mode = TMR_MODE_ONESHOT;
  tmr_cfg.clock = MXC_TMR_32K_CLK;

  tmr_cfg.bitMode = TMR_BIT_MODE_32;
  tmr_cfg.pol = 0;

  NVIC_ClearPendingIRQ(PAL_TMR_IRQn);
  NVIC_DisableIRQ(PAL_TMR_IRQn);
  MXC_TMR_ClearFlags(PAL_TMR);

  MXC_TMR_Stop(PAL_TMR);
  MXC_TMR_Init(PAL_TMR, &tmr_cfg, FALSE);
  
  palTimerCb.expCback = expCback;
  palTimerCb.state = PAL_TIMER_STATE_READY;
}

/*************************************************************************************************/
/*!
 *  \brief      Restore timer state after deep sleep.
 *
 *  \param      expUsec     Countdown time in microseconds, adjusted for sleep time.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerRestore(uint32_t expUsec)
{
  palTimerCb.state = PAL_TIMER_STATE_UNINIT;
  PalTimerInit(palTimerCb.expCback);
  PalTimerStart(expUsec);
}

/*************************************************************************************************/
/*!
 *  \brief      De-Initialize timer resources.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerDeInit(void)
{
  NVIC_DisableIRQ(PAL_TMR_IRQn);

  MXC_TMR_Shutdown(PAL_TMR);

  palTimerCb.state = PAL_TIMER_STATE_UNINIT;
}

/**************************************************************************************************
  External Functions: Control and Status
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Return scheduler timer state.
 
 */
/*************************************************************************************************/
void PalTimerExecCallback(void)
{
  if (palTimerCb.expCback) {
    palTimerCb.expCback();
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Return scheduler timer state.
 *
 *  \return     state.
 */
/*************************************************************************************************/
PalTimerState_t PalTimerGetState(void)
{
  return palTimerCb.state;
}

/*************************************************************************************************/
/*!
 *  \brief      Register a callback and set a timer.
 *
 *  \param      expUsec     Countdown time in microseconds.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerStart(uint32_t expUsec)
{
  PAL_TIMER_CHECK(palTimerCb.state == PAL_TIMER_STATE_READY);
  PAL_TIMER_CHECK(expUsec != 0);
  uint64_t compareValue;

  /* Make sure we don't wrap the timeout */
  if(expUsec == 0) {
    expUsec = 1;
  }

  MXC_TMR_SetCount(PAL_TMR, 0);

  /* Calculate the compare value */
  compareValue = ((uint64_t)expUsec * (uint64_t)PAL_RTC_TICKS_PER_SEC) / (uint64_t)1000000;

  /* Account for setup time */
  if (compareValue > PAL_TMR_SETUP_TICKS) {
    compareValue -= PAL_TMR_SETUP_TICKS;
  } else {
    compareValue = 1;
  }
  MXC_TMR_SetCompare(PAL_TMR, compareValue);

  /* Clear and enable interrupts */
  MXC_TMR_ClearFlags(PAL_TMR);
  MXC_TMR_EnableInt(PAL_TMR);
  NVIC_ClearPendingIRQ(PAL_TMR_IRQn);
  NVIC_EnableIRQ(PAL_TMR_IRQn);

  palTimerCb.state = PAL_TIMER_STATE_BUSY;

  MXC_TMR_Start(PAL_TMR);
}

/*************************************************************************************************/
/*!
 *  \brief      Cancel a timer operation.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerStop(void)
{
  MXC_TMR_Stop(PAL_TMR);

  /* Disable this interrupt */
  NVIC_DisableIRQ(PAL_TMR_IRQn);
  MXC_TMR_ClearFlags(PAL_TMR);

  palTimerCb.state = PAL_TIMER_STATE_READY;
}

/*************************************************************************************************/
/*!
 *  \brief      Sleep and use the timer to wakeup after expUsec micros seconds.
 *
 *  \param      expUsec     Countdown time in microseconds.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalTimerSleep(uint32_t expUsec)
{
  mxc_tmr_cfg_t tmr_cfg;
  uint64_t compareValue;

  /* Configure timer */
  tmr_cfg.pres = TMR_PRES_1;
  tmr_cfg.mode = TMR_MODE_ONESHOT;
  tmr_cfg.clock = MXC_TMR_APB_CLK;

  tmr_cfg.bitMode = TMR_BIT_MODE_32;
  tmr_cfg.pol = 0;

  NVIC_ClearPendingIRQ(PAL_SLEEP_TMR_IRQn);
  NVIC_DisableIRQ(PAL_SLEEP_TMR_IRQn);
  MXC_TMR_ClearFlags(PAL_SLEEP_TMR);

  MXC_TMR_Stop(PAL_SLEEP_TMR);
  MXC_TMR_Init(PAL_SLEEP_TMR, &tmr_cfg, FALSE);
  MXC_TMR_SetCount(PAL_SLEEP_TMR, 0);

  /* Calculate the compare value */
  compareValue = ((uint64_t)expUsec * (uint64_t)PAL_RTC_TICKS_PER_SEC) / (uint64_t)1000000;

  /* Account for setup time */
  if (compareValue > PAL_TMR_SETUP_TICKS) {
    compareValue -= PAL_TMR_SETUP_TICKS;
  } else {
    compareValue = 1;
  }
  MXC_TMR_SetCompare(PAL_SLEEP_TMR, compareValue);

  /* Clear and enable interrupts */
  MXC_TMR_ClearFlags(PAL_SLEEP_TMR);
  MXC_TMR_EnableInt(PAL_SLEEP_TMR);
  NVIC_ClearPendingIRQ(PAL_SLEEP_TMR_IRQn);
  NVIC_EnableIRQ(PAL_SLEEP_TMR_IRQn);

  MXC_TMR_Start(PAL_SLEEP_TMR);

  PalLedOff(PAL_LED_ID_CPU_ACTIVE);

  MXC_LP_EnterSleepMode();

  PalLedOn(PAL_LED_ID_CPU_ACTIVE);

  MXC_TMR_Shutdown(PAL_SLEEP_TMR);
}

/*************************************************************************************************/
/*!
 *  \brief      Get the time in microseconds before the next PalTimer expiration.
 *
 *  \return     Time to next PalTimer expiration in micro seconds.
 */
/*************************************************************************************************/
uint32_t PalTimerGetExpTime(void)
{
  uint64_t time;

  /* See if the timer is currently running */
  if(palTimerCb.state != PAL_TIMER_STATE_BUSY) {
    return 0;
  }

  /* See if the timer is enabled */
  if(!(PAL_TMR->ctrl0 & MXC_F_TMR_CTRL0_EN_A)) {
    return 0;
  }

  time = MXC_TMR_GetCompare(PAL_TMR) - MXC_TMR_GetCount(PAL_TMR);

  /* Account for the setup time */
  if(time > PAL_TMR_SETUP_TICKS) {
    time -= PAL_TMR_SETUP_TICKS;

    /* Convert back to us */
    time = ((uint64_t)time * (uint64_t)1000000) / (uint64_t)PAL_RTC_TICKS_PER_SEC;
  } else {
    return 0;
  }
  
  return time;
}
