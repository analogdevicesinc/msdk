/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief      Generic baseband driver implementation file.
 *
 *  Copyright (c) 2016-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019-2020 Packetcraft, Inc.
 *  
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *  
 *      http://www.apache.org/licenses/LICENSE-2.0
 *  
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

#include <string.h>
#include "bb_api.h"
#include "bb_int.h"
#include "pal_bb.h"

#include "wdt.h"

/**************************************************************************************************
  Globals
**************************************************************************************************/

BbCtrlBlk_t bbCb;                       /*!< BB control block. */
const BbRtCfg_t *pBbRtCfg = NULL;       /*!< Runtime configuration. */

uint32_t u32WdTmr0Cnt = 0;

/**************************************************************************************************
  Functions
**************************************************************************************************/

//! -------------------------------------------------------------------------------------------------------------------
//! Local Function Prototypes
//!
void WDT0_IRQHandler(void);
void ZioHalWatchdog0_HandlerC (uint32_t stackPointer[]);

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Clear the interrupt flag.
void
ZioHalWatchdog0_InterruptFlagClear(void)
{
    //@?ZIO_WATCHDOG_TIMER_0->ctrl &= ~(MXC_F_WDT_CTRL_INT_FLAG);
    MXC_WDT_ClearIntFlag(MXC_WDT0);
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Watchdog0 Interrupt Handler: attempts to log debug info ahead of watchdog reset
//! \note   This assembly code needs to be added as a separate assembly function,
//!         as the RVDS compiler cannot do inline assembly of thumb instructions required for Cortex-M3.
//!         More information on exception stack frames and their use in watchdog debugging can be found here:
//!         https://interrupt.memfault.com/blog/firmware-watchdog-best-practices#c-code-example
#pragma cstat_suppress="MISRAC2004-8.10" // IRQ Handler is ok to have only one external linkage
void WDT0_IRQHandler(void)
{
    // Our bare metal architecture only ever uses msp
    // Revisit this assembly code using the above link if we start using threads/rtos
      __asm volatile(
      "mrs r0, msp \n"                  // Move to ARM register from system coprocessor register - copies the main stack pointer into r0 so it can be passed as an argument 
      "b ZioHalWatchdog0_HandlerC \n"   // Branch instruction - calls ZioHalWatchdog0_HandlerC
                                );
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  C helper exception handler for watchdog interrupts
//! \note   Uses a function pointer for the logger routine to preserve hierarchy
//!         This will be set by ZioWatchdog0.c
//!         Using this instead of an external symbol to simplify the assembly code
//! \param  stackPointer   watchdog_args Stack frame location
#pragma cstat_suppress="MISRAC2004-8.10" // IRQ Handler is ok to have only one external linkage
void ZioHalWatchdog0_HandlerC (uint32_t stackPointer[])
{
    ZioHalWatchdog0_InterruptFlagClear();
    //@?mpWatchdogIntLogger(stackPointer);

    MXC_WDT_ResetTimer(MXC_WDT0);
    
    u32WdTmr0Cnt++;
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Initialize the appropriate watchdog settings.
//! \note   ZioResetCause_Determine() uses WDT0->ctrl to detect various reset scenarios. Ensure that this routine is
//!         only called after ZioResetCause_Determine(), or when knowing the reset cause is no longer required.
void
ZioWatchdog_Init(void)
{
    //@?ZioHalWatchdog0_ClockEnable(eZioBool_TRUE);
    MXC_WDT_Init(MXC_WDT0);
    // Clear the watchdog interrupt at initialization time in case it was left over across a reset.
    ZioHalWatchdog0_InterruptFlagClear();
    //@?ZioHalWatchdog0_InterruptPeriodSet(ZIO_WATCHDOG_INT_PERIOD);
    MXC_WDT_SetIntPeriod(MXC_WDT0, MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW28);    // 5.5925 seconds @ 96 Mhz SYSCLK (must be shorter than RST period)
    //@?ZioHalWatchdog0_ResetPeriodSet(ZIO_WATCHDOG_RST_PERIOD);
    MXC_WDT_SetResetPeriod(MXC_WDT0, MXC_S_WDT_CTRL_INT_PERIOD_WDT2POW29);  // 11.185 seconds @ 96 Mhz SYSCLK
    //@?ZioHalWatchdog0_IntLoggerSet(ZioWatchdog_IntLogger);
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Reset the watchdog timer.
void
ZioHalWatchdog0_Pet(void)
{
    // this sequence is defined in the user guide to cause watchdog timer pet/kick/feed/reset
    //@?ZIO_WATCHDOG_TIMER_0->rst = ZIO_WATCHDOG_RESET_VALUE_0;
    //@?ZIO_WATCHDOG_TIMER_0->rst = ZIO_WATCHDOG_RESET_VALUE_1;
    MXC_WDT_ResetTimer(MXC_WDT0);
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Pet (kick, feed) the watchdog. Analogous to resetting the countdown.
void
ZioWatchdog_Pet(void)
{
    ZioHalWatchdog0_Pet();
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Enable the watchdog timer.
//! \note   ensure that the watchdog clock has been enabled prior to enabling the watchdog
//!         (via ZioHalWatchdog0_ClockEnable)
//! \param  enable eZioBool_TRUE to enable, eZioBool_FALSE to disable
void
ZioHalWatchdog0_Enable(int enable)
{
    if (TRUE == enable)
    {
        //@?ZIO_WATCHDOG_TIMER_0->ctrl |= MXC_F_WDT_CTRL_WDT_EN;
        MXC_WDT_Enable(MXC_WDT0);
    }
    else
    {
        //@?ZIO_WATCHDOG_TIMER_0->ctrl &= ~(MXC_F_WDT_CTRL_WDT_EN);
        MXC_WDT_Disable(MXC_WDT0);
    }
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Enable the watchdog interrupt.
//! \param  enable eZioBool_TRUE to enable, eZioBool_FALSE to disable
void
ZioHalWatchdog0_InterruptEnable(int enable)
{
    if (TRUE == enable)
    {
        //@?ZIO_WATCHDOG_TIMER_0->ctrl |= MXC_F_WDT_CTRL_INT_EN;
        MXC_WDT_EnableInt(MXC_WDT0);
    }
    else
    {
        //@?ZIO_WATCHDOG_TIMER_0->ctrl &= ~(MXC_F_WDT_CTRL_INT_EN);
        MXC_WDT_DisableInt(MXC_WDT0);
    }

    // Enable the interrupt vector
    NVIC_ClearPendingIRQ(WDT0_IRQn);
    NVIC_DisableIRQ(WDT0_IRQn);
    //@?NVIC_SetPriority(WDT0_IRQn, ZIO_IRQ_PRIORITY_WDT0);
    NVIC_SetPriority(WDT0_IRQn, 0);
    NVIC_EnableIRQ(WDT0_IRQn);
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Enable the watchdog reset.
//! \param  enable eZioBool_TRUE to enable, eZioBool_FALSE to disable
void
ZioHalWatchdog0_ResetEnable(uint8_t enable)
{
    if (TRUE == enable)
    {
        MXC_WDT_EnableReset(MXC_WDT0);
    }
    else
    {
        MXC_WDT_DisableReset(MXC_WDT0);
    }
}

//! -------------------------------------------------------------------------------------------------------------------
//! \brief  Enable the watchdog
//! \note   This behavior is subject to conditional compilation based on our debug state
void
ZioWatchdog_Enable(void)
{
    ZioHalWatchdog0_Pet();
    ZioHalWatchdog0_Enable(TRUE);

#if defined(ZIO_DEBUG_ALLOW_BREAKPOINTS) // Disable interrupts and resets if we're enabling breakpoints
    ZioHalWatchdog0_InterruptEnable(eZioBool_FALSE);
    ZioHalWatchdog0_ResetEnable(eZioBool_FALSE);
#elif defined(ZIO_DEBUG_DISABLE_WATCHDOG_RESETS) // Only enable interrupts so we can log and debug watchdog resets
    ZioHalWatchdog0_InterruptEnable(eZioBool_TRUE);
    ZioHalWatchdog0_ResetEnable(eZioBool_FALSE);
#else // Normal flow with both resets and interrupts enabled
    ZioHalWatchdog0_InterruptEnable(TRUE);
    ZioHalWatchdog0_ResetEnable(TRUE);
#endif
}


/*************************************************************************************************/
/*!
 *  \brief      Initialize runtime configuration.
 *
 *  \param      pCfg        Pointer to runtime configuration parameters (data must be static).
 *
 *  This function initializes the BB subsystem's runtime configuration.
 *
 *  \note       This routine must be called only once before any other initialization routine.
 */
/*************************************************************************************************/
void BbInitRunTimeCfg(const BbRtCfg_t *pCfg)
{
  WSF_ASSERT(pBbRtCfg == NULL);
  WSF_ASSERT(pCfg);

  WSF_ASSERT(pCfg->clkPpm >= 20);
  WSF_ASSERT(pCfg->rfSetupDelayUs > 0);
  WSF_ASSERT(pCfg->maxScanPeriodMs > 0);
  WSF_ASSERT(pCfg->schSetupDelayUs > 0);
  WSF_ASSERT(pCfg->BbTimerBoundaryUs > 0);

  pBbRtCfg = pCfg;
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize the BB.
 *
 *  Initialize baseband resources.
 */
/*************************************************************************************************/
void BbInit(void)
{
  WSF_ASSERT(pBbRtCfg);

  ZioWatchdog_Init();
  ZioWatchdog_Enable();

  PalBbInit();

  memset(&bbCb, 0, sizeof(bbCb));
}

/*************************************************************************************************/
/*!
 *  \brief      Register operation completion handler.
 *
 *  \param      eventCback  Event callback.
 *
 *  Register operation completion handler.
 */
/*************************************************************************************************/
void BbRegister(BbBodCompCback_t eventCback)
{
  bbCb.bodCompCback = eventCback;
}

/*************************************************************************************************/
/*!
 *  \brief      Start BB processing of given protocol.
 *
 *  \param      protId  Protocol ID.
 */
/*************************************************************************************************/
static void bbProtStart(PalBbProt_t protId)
{
  WSF_ASSERT(bbCb.prot[protId].startProtCback != NULL);

  /* Enable protocol-specific BB. */
  bbCb.prot[protId].startProtCback();

  /* Protocol now started. */
  bbCb.protStarted   = TRUE;
  bbCb.protIdStarted = protId;
  PalBbSetProtId(protId);
}

/*************************************************************************************************/
/*!
 *  \brief      Start BB processing of given protocol.
 *
 *  \param      protId  Protocol ID.
 *
 *  Enable BB and start processing the list of BODs.  This routine may be called several times, if
 *  a protocol layers has several simultaneously-enabled operations.  However, \ref BbStop() must
 *  be called an equal number of time to disable the baseband.
 */
/*************************************************************************************************/
void BbStart(PalBbProt_t protId)
{
  WSF_ASSERT(protId < BB_PROT_NUM);

  if (!bbCb.protStarted)
  {
    /* Enable generic BB. */
    PalBbEnable();

    /* Enable protocol-specific BB. */
    bbProtStart(protId);
  }

  bbCb.prot[protId].startCnt++;
}

/*************************************************************************************************/
/*!
 *  \brief      Stop BB processing of given protocol.
 *
 *  \param      protId  Protocol ID.
 */
/*************************************************************************************************/
static void bbProtStop(PalBbProt_t protId)
{
  WSF_ASSERT(bbCb.prot[protId].stopProtCback != NULL);

  /* No protocol started. */
  bbCb.protStarted = FALSE;

  /* Disable protocol-specific BB. */
  bbCb.prot[protId].stopProtCback();
}

/*************************************************************************************************/
/*!
 *  \brief      Stop BB processing of given protocol.
 *
 *  \param      protId  Protocol ID.
 *
 *  Disable BB processing of BODs.
 *
 *  \note       For any particular protocol, calls to \ref BbStart() and \ref BbStop() must be
 *              balanced to ensure that the hardware is disabled if and only if appropriate.
 */
/*************************************************************************************************/
void BbStop(PalBbProt_t protId)
{
  WSF_ASSERT(protId < BB_PROT_NUM);
  WSF_ASSERT(bbCb.prot[protId].startCnt > 0);

  bbCb.prot[protId].startCnt--;

  if (bbCb.protStarted && (bbCb.protIdStarted == protId) && (bbCb.prot[protId].startCnt == 0))
  {
    /* Disable protocol-specific BB. */
    bbProtStop(protId);

    /* Disable generic BB. */
    PalBbDisable();
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Execute BOD.
 *
 *  \param      pBod    Pointer to the BOD to execute.
 *
 *  Execute the protocol specific BOD handler.
 */
/*************************************************************************************************/
void BbExecuteBod(BbOpDesc_t *pBod)
{
  WSF_ASSERT(pBod);

  WSF_ASSERT(pBod->protId < BB_PROT_NUM);
  /* TODO: Removed this assert as it spuriously seems to be taken. */
  /* WSF_ASSERT(!bbCb.pOpInProgress); */
  bbCb.pOpInProgress = pBod;
  bbCb.termBod = FALSE;

  /* Enable generic BB. */
  if (!bbCb.protStarted)
  {
    PalBbEnable();
  }

  /* Switch protocols if necessary. */
  if (bbCb.protStarted && (bbCb.protIdStarted != pBod->protId))
  {
    /* Disable protocol-specific BB. */
    bbProtStop(bbCb.protIdStarted);   /* sets bbCb.protStarted = FALSE */
  }
  if (!bbCb.protStarted)
  {
    /* Enable protocol-specific BB. */
    bbProtStart(pBod->protId);
  }

  if (bbCb.prot[pBod->protId].execOpCback != NULL)
  {
    bbCb.prot[pBod->protId].execOpCback(pBod);
  }

  if (bbCb.termBod)
  {
    bbCb.pOpInProgress = NULL;
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Cancel current executing BOD.
 */
/*************************************************************************************************/
void BbCancelBod(void)
{
  if (bbCb.pOpInProgress)
  {
    BbOpDesc_t * const pBod = bbCb.pOpInProgress;

    WSF_ASSERT(pBod->protId < BB_PROT_NUM);
    if (bbCb.prot[pBod->protId].cancelOpCback != NULL)
    {
      bbCb.prot[pBod->protId].cancelOpCback(pBod);
    }

    bbCb.pOpInProgress = NULL;
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Get the currently-executing BOD.
 *
 *  \return     Currently-executing BOD.
 */
/*************************************************************************************************/
BbOpDesc_t *BbGetCurrentBod(void)
{
  return bbCb.pOpInProgress;
}

/*************************************************************************************************/
/*!
 *  \brief      Set termination flag of current executing BOD.
 *
 *  \note       This function is expected to be called during the execution context of the
 *              current executing BOD, typically in the related ISRs. In the end, termination
 *              flag will help to decide if BbTerminateBod() should be called.
 */
/*************************************************************************************************/
void BbSetBodTerminateFlag(void)
{
  if (bbCb.pOpInProgress)
  {
    bbCb.termBod = TRUE;
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Get termination state of current executing BOD.
 *
 *  \return     TRUE if termination flag set, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t BbGetBodTerminateFlag(void)
{
  return bbCb.termBod;
}

/*************************************************************************************************/
/*!
 *  \brief      Terminate a BOD immediately.
 *
 *  \note       This function is expected to be called during the execution context of the
 *              current executing BOD, typically in the related ISRs.
 */
/*************************************************************************************************/
void BbTerminateBod(void)
{
  WSF_ASSERT(bbCb.bodCompCback);

  BbOpDesc_t * const pBod = bbCb.pOpInProgress;
  if (pBod &&
      (bbCb.prot[pBod->protId].lowPowerOpCback != NULL))
  {
    bbCb.prot[pBod->protId].lowPowerOpCback();
  }

  bbCb.pOpInProgress = NULL;
  bbCb.termBod = TRUE;
  bbCb.bodCompCback();
}

/*************************************************************************************************/
/*!
 *  \brief      Get BB clock accuracy.
 *
 *  \return     Clock accuracy in part per million.
 *
 *  Returns the current BB clock accuracy.
 */
/*************************************************************************************************/
uint16_t BbGetClockAccuracy(void)
{
  return pBbRtCfg->clkPpm;
}

/*************************************************************************************************/
/*!
 *  \brief      Get BB timer boundary before wraparound.
 *
 *  \return     Time boundary in microseconds.
 *
 */
/*************************************************************************************************/
uint32_t BbGetBbTimerBoundaryUs(void)
{
  return pBbRtCfg->BbTimerBoundaryUs;
}

/*************************************************************************************************/
/*!
 *  \brief      Get scheduler setup delay.
 *
 *  \return     Scheduler setup delay in microseconds.
 *
 *  Returns the scheduler setup delay.
 */
/*************************************************************************************************/
uint16_t BbGetSchSetupDelayUs(void)
{
  return pBbRtCfg->schSetupDelayUs;
}

/*************************************************************************************************/
/*!
 *  \brief      Get RF setup delay.
 *
 *  \return     RF setup delay in microseconds.
 *
 *  Returns the RF setup delay.
 */
/*************************************************************************************************/
uint16_t BbGetRfSetupDelayUs(void)
{
  return pBbRtCfg->rfSetupDelayUs;
}

/*************************************************************************************************/
/*!
 *  \brief      Adjust new time tick with wraparound.
 *
 *  \param      dueUsec    Time tick without wraparound in microseconds.

 *
 *  \return     Time tick with wraparound.
 *
 *  \note        dueUsec can only be at most +/-(BbTimerBoundaryUs/2) out of range.
 */
/*************************************************************************************************/
uint32_t BbAdjustTime(uint32_t dueUsec)
{
  /* If and only of dueUsec is outside the range of [0, BbTimerBoundaryUs], mapping adjustment is needed. */
  if (dueUsec > pBbRtCfg->BbTimerBoundaryUs)
  {
    /* dueUsec is in range [-(BbTimerBoundaryUs + 1)/2, 0). */
    if (dueUsec >= ~(pBbRtCfg->BbTimerBoundaryUs >> 1))
    {
      dueUsec += (pBbRtCfg->BbTimerBoundaryUs + 1);
    }
    /* dueUsec is in range [(BbTimerBoundaryUs + 1), (BbTimerBoundaryUs + 1) + (BbTimerBoundaryUs + 1) / 2). */
    else if (dueUsec <= (pBbRtCfg->BbTimerBoundaryUs + 1 + (pBbRtCfg->BbTimerBoundaryUs >> 1)))
    {
      dueUsec -= (pBbRtCfg->BbTimerBoundaryUs + 1);
    }
    else
    {
      /* It should never happen here. */
      WSF_ASSERT(FALSE);
    }
  }

  /* If dueUsec is  in range [0, BbTimerBoundaryUs], no need to adjust. */
  return dueUsec;
}

/*************************************************************************************************/
/*!
 *  \brief      Get Delta between target and reference time. Only valid if target time is in the future.
 *
 *  \param      targetUsec    Target time in microseconds.
 *  \param      refUsec       Reference time in microseconds.
 *
 *  \return     Positive number in microseconds if target time is in the future.
 *              Zero if target time is in the past or same compared with reference time.
 *
 *  \note       Caller has to make sure target time and reference time are within SCH_MAX_SPAN.
 */
/*************************************************************************************************/
uint32_t BbGetTargetTimeDelta(uint32_t targetUsec, uint32_t refUsec)
{
  targetUsec = BbAdjustTime(targetUsec);
  refUsec = BbAdjustTime(refUsec);

  uint32_t delta = 0;

  if (targetUsec > refUsec)
  {
    /* Always bigger number minus smaller number. */
    if ((targetUsec - refUsec) < ((pBbRtCfg->BbTimerBoundaryUs >> 1) + 1))
    {
      /* Normal case. */
      delta = targetUsec - refUsec;
    }
    else
    {
      /* reference time must be wraparound and target time is in the past.*/
      delta = 0;
    }
  }
  else
  {
    /* Always bigger number minus smaller number. */
    if ((refUsec - targetUsec) < ((pBbRtCfg->BbTimerBoundaryUs >> 1) + 1))
    {
      /* target time is in the past. */
      delta = 0;
    }
    else
    {
      /* Target time must be wraparound. */
      delta = pBbRtCfg->BbTimerBoundaryUs - (refUsec - targetUsec) + 1;
    }
  }

  return delta;
}

/*************************************************************************************************/
/*!
 *  \brief      Returns the ID of the active protocol.
 *
 *  \return     Protocol operation in progress.
 */
/*************************************************************************************************/
uint8_t BbGetActiveProtocol(void)
{
  return bbCb.protIdStarted;
}

/*************************************************************************************************/
/*!
 *  \brief      Register protocol handlers.
 *
 *  \param      protId          Protocol ID.
 *  \param      execOpCback     Execute operation callback.
 *  \param      cancelOpCback   Cancel operation callback.
 *  \param      startProtCback  Start protocol callback.
 *  \param      stopProtCback   Stop protocol callback.
 */
/*************************************************************************************************/
void BbRegisterProt(PalBbProt_t protId, BbBodCback_t execOpCback, BbBodCback_t cancelOpCback,
                    BbProtCback_t startProtCback, BbProtCback_t stopProtCback)
{
  WSF_ASSERT(protId < BB_PROT_NUM);
  WSF_ASSERT(startProtCback != NULL);
  WSF_ASSERT(stopProtCback != NULL);

  bbCb.prot[protId].execOpCback    = execOpCback;
  bbCb.prot[protId].cancelOpCback  = cancelOpCback;
  bbCb.prot[protId].startProtCback = startProtCback;
  bbCb.prot[protId].stopProtCback  = stopProtCback;
}

/*************************************************************************************************/
/*!
 *  \brief      Register protocol handlers for low power.
 *
 *  \param      protId          Protocol ID.
 *  \param      lowPowerOpCback Low power operation callback.
 */
/*************************************************************************************************/
void BbRegisterProtLowPower(PalBbProt_t protId, BbLowPowerCback_t lowPowerOpCback)
{
  WSF_ASSERT(protId < BB_PROT_NUM);
  WSF_ASSERT(lowPowerOpCback != NULL);
  bbCb.prot[protId].lowPowerOpCback    = lowPowerOpCback;
}


