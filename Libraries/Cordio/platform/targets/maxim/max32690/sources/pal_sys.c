/*************************************************************************************************/
/*!
 * \file
 *
 * \brief  System hooks.
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
#include <string.h>

#ifdef __ICCARM__  // IAR
#include <stdlib.h>
#pragma section="CSTACK"
#pragma section="HEAP"
#endif

#include "pal_sys.h"
#include "pal_led.h"
#include "pal_rtc.h"
#include "pal_timer.h"
#include "pal_crypto.h"
#include "pal_uart.h"
#include "pal_bb.h"
#include "mxc_device.h"
#include "mcr_regs.h"
#include "gcr_regs.h"
#include "lp.h"
#include "wut.h"
#include "uart.h"
#include "sema.h"
#include "dma.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

#ifndef PAL_SYS_RISCV_LOAD
#define PAL_SYS_RISCV_LOAD                0
#endif

#ifndef PAL_SYS_SLEEP_DISABLE
#define PAL_SYS_SLEEP_DISABLE             0
#endif

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! \brief      Number of assertions. */
static uint32_t palSysAssertCount;

/*! \brief      Trap enabled flag. */
static volatile bool_t PalSysAssertTrapEnable;

/*! \brief      Busy client count. */
static uint32_t palSysBusyCount;

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Enter a critical section.
 */
/*************************************************************************************************/
void PalEnterCs(void)
{
  __disable_irq();
}

/*************************************************************************************************/
/*!
 *  \brief  Exit a critical section.
 */
/*************************************************************************************************/
void PalExitCs(void)
{
  __enable_irq();
}

/*************************************************************************************************/
/*!
 *  \brief  DMA interrupt handlers.
 */
/*************************************************************************************************/
void DMA0_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 0 is associated with DMA instance 0 */
}
void DMA1_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 1 is associated with DMA instance 0 */
}
void DMA2_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 2 is associated with DMA instance 0 */
}
void DMA3_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 3 is associated with DMA instance 0 */
}
void DMA4_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 4 is associated with DMA instance 0 */
}
void DMA5_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 5 is associated with DMA instance 0 */
}
void DMA6_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 6 is associated with DMA instance 0 */
}
void DMA7_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 7 is associated with DMA instance 0 */
}
void DMA8_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 8 is associated with DMA instance 0 */
}
void DMA9_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 9 is associated with DMA instance 0 */
}
void DMA10_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 10 is associated with DMA instance 0 */
}
void DMA11_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 11 is associated with DMA instance 0 */
}
void DMA12_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 12 is associated with DMA instance 0 */
}
void DMA13_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 13 is associated with DMA instance 0 */
}
void DMA14_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 14 is associated with DMA instance 0 */
}
void DMA15_IRQHandler(void)
{
  MXC_DMA_Handler(); /* DMA channel 15 is associated with DMA instance 0 */
}

/*************************************************************************************************/
/*!
 *  \brief      Common platform initialization.
 */
/*************************************************************************************************/
void PalSysInit(void)
{
  /* Delay to prevent lockup when debugging */
#ifdef DEBUG
  volatile int i;
  for(i = 0; i < 0x3FFFF; i++) {}
#endif

  palSysAssertCount = 0;
  PalSysAssertTrapEnable = TRUE;
  palSysBusyCount = 0;

  /* Enable wakeup sources */
  MXC_PWRSEQ->lppwen |= (MXC_F_PWRSEQ_LPPWEN_CPU1 | MXC_F_PWRSEQ_LPPWEN_UART0 | MXC_F_PWRSEQ_LPPWEN_UART1 |
                         MXC_F_PWRSEQ_LPPWEN_UART2 | MXC_F_PWRSEQ_LPPWEN_UART3 | MXC_F_PWRSEQ_LPPWEN_TMR0 | MXC_F_PWRSEQ_LPPWEN_TMR1);

  PalLedInit();
  PalLedOff(PAL_LED_ID_ERROR);
  PalLedOn(PAL_LED_ID_CPU_ACTIVE);
  PalCryptoInit();
  PalRtcInit();

#ifndef __riscv
#if PAL_SYS_RISCV_LOAD

  /* Halt the RISCV */
  MXC_SYS_RISCVShutdown();

#ifdef DEBUG
  /* Enable RISCV debugger GPIO */
  MXC_GPIO_Config(&gpio_cfg_rv_jtag);
#endif

  /* Initialize the Semaphore peripheral */
  MXC_SEMA_Init();
  MXC_SEMA_InitBoxes();

  /* Enable semaphore interrupt and clear state */
  NVIC_ClearPendingIRQ(RISCV_IRQn);
  NVIC_EnableIRQ(RISCV_IRQn);

  /* Start the RISCV core */
  MXC_SYS_RISCVRun();

  /* Give the RISCV time to startup */
  volatile int j;
  for(j = 0; j < 0xFFFFFF; j++) {}

#endif
#endif

#ifdef __riscv
  /* Initialize the Semaphore peripheral */
  MXC_SEMA_Init();

  /* Enable ARM incoming interrupts */
  NVIC_ClearPendingIRQ(PF_IRQn);
  NVIC_EnableIRQ(PF_IRQn);

#endif
}

/*************************************************************************************************/
/*!
 *  \brief      System fault trap.
 */
/*************************************************************************************************/
void PalSysAssertTrap(void)
{
  PalEnterCs();
  PalLedOn(PAL_LED_ID_ERROR);
  palSysAssertCount++;
  while (PalSysAssertTrapEnable);
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Set system trap.
 *
 *  \param      enable    Enable assert trap or not.
 */
/*************************************************************************************************/
void PalSysSetTrap(bool_t enable)
{
  PalSysAssertTrapEnable = enable;
}

/*************************************************************************************************/
/*!
 *  \brief      Get assert count.
 */
/*************************************************************************************************/
uint32_t PalSysGetAssertCount(void)
{
  return palSysAssertCount;
}

/*************************************************************************************************/
/*!
 *  \brief      Count stack usage.
 *
 *  \return     Number of bytes used by the stack.
 */
/*************************************************************************************************/
uint32_t PalSysGetStackUsage(void)
{
  /* Not available; stub routine. */
  return 0;
}

/*************************************************************************************************/
/*!
 *  \brief      System sleep.
 *
 *  \param      nextWakeMs  Next CPU wakeup time.
 *
 *  \note       Caller of this routine must ensure IRQ are disabled before entering this call.
 */
/*************************************************************************************************/
void PalSysSleep(void)
{
  if (palSysBusyCount) {
    /* Work pending; do not sleep yet. */
    return;
  }

  #if PAL_SYS_SLEEP_DISABLE
  return;
  #endif

  #ifdef DEBUG
  /* Stay active to prevent debugger dropout */
  return;
  #endif
  
  MXC_LP_EnterSleepMode();
}

/*************************************************************************************************/
/*!
 *  \brief      Set system busy.
 */
/*************************************************************************************************/
void PalSysSetBusy(void)
{
  PalEnterCs();
  palSysBusyCount++;
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Set system idle.
 */
/*************************************************************************************************/
void PalSysSetIdle(void)
{
  PalEnterCs();
  if (palSysBusyCount) {
    palSysBusyCount--;
  }
  PalExitCs();
}

/*************************************************************************************************/
/*!
 *  \brief      Check if system is busy.
 *
 *  \return     TRUE if system is busy.
 */
/*************************************************************************************************/
bool_t PalSysIsBusy(void)
{
  bool_t sysIsBusy = FALSE;
  PalEnterCs();
  sysIsBusy = ((palSysBusyCount == 0) ? FALSE : TRUE);
  PalExitCs();
  return sysIsBusy;
}
