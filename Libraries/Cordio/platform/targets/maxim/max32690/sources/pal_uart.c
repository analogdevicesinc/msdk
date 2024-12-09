/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      UART driver implementation.
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

#include "pal_uart.h"
#include "pal_led.h"
#include "pal_sys.h"

#include "board.h"

#include "uart.h"
#include "dma.h"
#include "sema.h"

#include "uart_revb.h"

#include "wsf_cs.h"

#include <string.h>

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Number of UARTs this driver will use */
#ifndef PAL_UARTS
#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
#define PAL_UARTS         5
#else
#define PAL_UARTS         4
#endif
#endif

#ifndef TERMINAL_BAUD
#define TERMINAL_BAUD     115200
#endif

/**************************************************************************************************
  Local Variables
**************************************************************************************************/
/*! \brief      Control block. */
static struct {
  PalUartState_t state;
  PalUartCompCback_t rdCback;
  PalUartCompCback_t wrCback;
  int writeCh;
  int readCh;
} palUartCb[PAL_UARTS];

/*************************************************************************************************/
/*!
 *  \brief      Inter-processor communication interrupt handlers.
 *
 *  \return     None.
 */
/*************************************************************************************************/
#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
#ifdef __riscv
/* RISCV */
void PF_IRQHandler(void)
{
  /* Clear interrupt state */
  MXC_SEMA->irq1 &= ~(MXC_F_SEMA_IRQ1_RV32_IRQ);

  PalLedOn(PAL_LED_ID_CPU_ACTIVE);

  /* Handle the semaphore interrupt */
  MXC_SEMA_Handler();
}
#else
/* ARM */
void RISCV_IRQHandler(void)
{
  /* Clear interrupt state */
  MXC_SEMA->irq0 &= ~(MXC_F_SEMA_IRQ0_CM4_IRQ);

  PalLedOn(PAL_LED_ID_CPU_ACTIVE);

  /* Handle the semaphore interrupt */
  MXC_SEMA_Handler();
}
#endif
#endif

/*************************************************************************************************/
/*!
 *  \brief      Callback from the UART driver.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void palUartCallback(int ch, int error)
{
  int i;
  for(i = 0; i < PAL_UARTS; i++) {
    /* Find the corresponding rqeuest and call the callback */
    
     if((ch == palUartCb[i].readCh) && (palUartCb[i].state != PAL_UART_STATE_UNINIT)) {
      palUartCb[i].readCh = -1;
      if(palUartCb[i].rdCback != NULL) {
        palUartCb[i].rdCback();
      }

      MXC_DMA_ReleaseChannel(ch);
      return;
    }

    if((ch == palUartCb[i].writeCh) && (palUartCb[i].state != PAL_UART_STATE_UNINIT)) {
      palUartCb[i].writeCh = -1;
      palUartCb[i].state = PAL_UART_STATE_READY;
      if(palUartCb[i].wrCback != NULL) {
        palUartCb[i].wrCback();
      }

      MXC_DMA_ReleaseChannel(ch);
      return;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \brief      Get UART instance number from UART ID.
 *
 *  \param      uartId           UART ID.
 *
 *  \return     UART instance number.
 */
/*************************************************************************************************/
static uint8_t palUartGetNum(PalUartId_t uartId)
{
  uint8_t uartNum;
  switch (uartId) {
    case PAL_UART_ID_CHCI:
      uartNum = HCI_UART;
      break;
    case PAL_UART_ID_TERMINAL:
      uartNum = TERMINAL_UART;
      break;
    case PAL_UART_ID_USER:
      uartNum = USER_UART;
      break;
    default:
      PalSysAssertTrap();
      return 0;
      break;
  }

  return uartNum;
}

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)

void palMailWriteCb(int result)
{
  if(palUartCb[PAL_UARTS-1].wrCback != NULL) {
    palUartCb[PAL_UARTS-1].state = PAL_UART_STATE_READY;
    palUartCb[PAL_UARTS-1].wrCback();
  }
}

void palMailReadCb(int result)
{
  if(palUartCb[PAL_UARTS-1].rdCback != NULL) {
    palUartCb[PAL_UARTS-1].rdCback();
  }
}

static void PalMailWriteData(const uint8_t *pData, uint16_t len)
{
  palUartCb[PAL_UARTS-1].state = PAL_UART_STATE_BUSY;
  MXC_SEMA_WriteBoxAsync(palMailWriteCb, pData, len);
}

static void PalMailReadData(uint8_t *pData, uint16_t len)
{
  MXC_SEMA_ReadBoxAsync(palMailReadCb, pData, len);
}

static void PalMailInit(const PalUartConfig_t *pCfg)
{
  unsigned uartNum = PAL_UARTS-1;

  /* Save the callback */
  palUartCb[uartNum].rdCback = pCfg->rdCback;
  palUartCb[uartNum].wrCback = pCfg->wrCback;

  palUartCb[uartNum].state = PAL_UART_STATE_READY;
}
#endif


/**************************************************************************************************
  Global Functions
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief      Initialize UART.
 *
 *  \param      id          UART Id.
 *  \param      pCfg        Peripheral configuration.
 *
 *  \return     None.
 *
 *  Initialize UART peripheral with \a pCfg values.
 */
/*************************************************************************************************/
void PalUartInit(PalUartId_t id, const PalUartConfig_t *pCfg)
{
  uint8_t uartNum = palUartGetNum(id);
  mxc_uart_regs_t *uart = MXC_UART_GET_UART(uartNum);

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
  if(id == PAL_UART_ID_CHCI) {
    PalMailInit(pCfg);
    return;
  }
#endif

  PAL_SYS_ASSERT(palUartCb[uartNum].state == PAL_UART_STATE_UNINIT);

  /* Save the callback */
  palUartCb[uartNum].rdCback = pCfg->rdCback;
  palUartCb[uartNum].wrCback = pCfg->wrCback;
  palUartCb[uartNum].readCh = -1;
  palUartCb[uartNum].writeCh = -1;
  

  int result = MXC_UART_Init(uart, pCfg->baud, MXC_UART_IBRO_CLK);
  (void)result;
  PAL_SYS_ASSERT(result == 0);

  /* Disable UART interrupts */
  MXC_UART_DisableInt(uart, 0xFFFFFFFF);
  MXC_UART_ClearFlags(uart, 0xFFFFFFFF);

  MXC_UART_SetDataSize(uart, 8);
  MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);
  MXC_UART_SetParity(uart, MXC_UART_PARITY_DISABLE);
  if(pCfg->hwFlow) {
    MXC_UART_SetFlowCtrl(uart, MXC_UART_FLOW_EN, 1);
  }
  
  MXC_DMA_Init();

  /* Enable the DMA channel interrupts */
  NVIC_EnableIRQ(DMA0_IRQn);
  NVIC_EnableIRQ(DMA1_IRQn);
  NVIC_EnableIRQ(DMA2_IRQn);
  NVIC_EnableIRQ(DMA3_IRQn);
  NVIC_EnableIRQ(DMA4_IRQn);
  NVIC_EnableIRQ(DMA5_IRQn);
  NVIC_EnableIRQ(DMA6_IRQn);
  NVIC_EnableIRQ(DMA7_IRQn);
  NVIC_EnableIRQ(DMA8_IRQn);
  NVIC_EnableIRQ(DMA9_IRQn);
  NVIC_EnableIRQ(DMA10_IRQn);
  NVIC_EnableIRQ(DMA11_IRQn);
  NVIC_EnableIRQ(DMA12_IRQn);
  NVIC_EnableIRQ(DMA13_IRQn);
  NVIC_EnableIRQ(DMA14_IRQn);
  NVIC_EnableIRQ(DMA15_IRQn);
 
  palUartCb[uartNum].state = PAL_UART_STATE_READY;
}

/*************************************************************************************************/
/*!
 *  \brief      De-Initialize UART.
 *
 *  \param      id      UART id.
 *
 *  \return     None.
 *
 *  De-Initialize UART.
 */
/*************************************************************************************************/
void PalUartDeInit(PalUartId_t id)
{
  uint8_t uartNum = palUartGetNum(id);

  int result = MXC_UART_Shutdown(MXC_UART_GET_UART(uartNum));
  (void)result;
  PAL_SYS_ASSERT(result);

  NVIC_DisableIRQ(MXC_UART_GET_IRQ(uartNum));
  MXC_DMA_DeInit();

  palUartCb[uartNum].state = PAL_UART_STATE_UNINIT;
}

/*************************************************************************************************/
/*!
 *  \brief      Get the current state.
 *
 *  \param      id      UART id.
 *
*  \return      Current state.
 *
 *  Return the current state.
 */
/*************************************************************************************************/
PalUartState_t PalUartGetState(PalUartId_t id)
{
  return palUartCb[palUartGetNum(id)].state;
}

/*************************************************************************************************/
/*!
 *  \brief      Read data from Rx FIFO.
 *
 *  \param      id          UART id.
 *  \param      pData       Read buffer.
 *  \param      len         Number of bytes to read.
 *
 *  \return     None.
 *
 *  Store \a len received bytes in \a pData. After \a len is transferred, call
 *  \a UartInitInfo_t::rdCback to signal read completion. Alway call this function to setup buffer
 *  when boot up or after a reading is done
 */
/*************************************************************************************************/
void PalUartReadData(PalUartId_t id, uint8_t *pData, uint16_t len)
{
  uint8_t uartNum = palUartGetNum(id);
  mxc_uart_regs_t* uart = MXC_UART_GET_UART(uartNum);

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
  if(id == PAL_UART_ID_CHCI) {
    PalMailReadData(pData, len);
    return;
  }
#endif

  WsfCsEnter();
  int dmaCh = MXC_DMA_AcquireChannel();
  WsfCsExit();

  /* Save the channel number */
  palUartCb[uartNum].readCh = dmaCh;

  /* Setup the DMA transfer */
  mxc_dma_config_t config = {
    .ch = dmaCh,
    .srcwd = MXC_DMA_WIDTH_BYTE,
    .dstwd = MXC_DMA_WIDTH_BYTE,
    .srcinc_en = 0,
    .dstinc_en = 1
  };

  mxc_dma_srcdst_t srcdst = {
    .ch = dmaCh,
    .dest = (void*)pData,
    .len = len
  };

  switch (uartNum) {
  case 0:
    config.reqsel = MXC_DMA_REQUEST_UART0RX;
    break;

  case 1:
    config.reqsel = MXC_DMA_REQUEST_UART1RX;
    break;

  case 2:
    config.reqsel = MXC_DMA_REQUEST_UART2RX;
    break;

  case 3:
    config.reqsel = MXC_DMA_REQUEST_UART3RX;
    break;

  default:
    PAL_SYS_ASSERT(0);
    return;
  }

  MXC_DMA_ConfigChannel(config, srcdst);
  MXC_DMA_SetCallback(dmaCh, palUartCallback);
  
  /* Enable Count-to-Zero (CTZ) interrupt */
  MXC_DMA_EnableInt(dmaCh);
  MXC_DMA_SetChannelInterruptEn(dmaCh, 0, 1);

  /* Set Rx FIFO threshold */
  uart->dma |= 1 << MXC_F_UART_REVB_DMA_RX_THD_VAL_POS;
  /* Enable channel receiving */ 
  uart->dma |= MXC_F_UART_REVB_DMA_RX_EN;

  /* Start the transfer */
  MXC_DMA_Start(dmaCh);
}




/*************************************************************************************************/
/*!
 *  \brief      Write data to Tx FIFO.
 *
 *  \param      id          UART id.
 *  \param      pData       Write buffer.
 *  \param      len         Number of bytes to write.
 *
 *  \return     None.
 *
 *  Assign buffer and length and transmit data.
 */
/*************************************************************************************************/
void PalUartWriteData(PalUartId_t id, const uint8_t *pData, uint16_t len)
{
  uint8_t uartNum = palUartGetNum(id);
  mxc_uart_regs_t* uart = MXC_UART_GET_UART(uartNum);

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
  if(id == PAL_UART_ID_CHCI) {
    PalMailWriteData(pData, len);
    return;
  }
#endif
  
  WsfCsEnter();
  int dmaCh = MXC_DMA_AcquireChannel();
  WsfCsExit();

  palUartCb[uartNum].writeCh = dmaCh;
  palUartCb[uartNum].state = PAL_UART_STATE_BUSY;

  /* Setup the DMA transfer */
  mxc_dma_config_t config = {
    .ch = dmaCh,
    .srcwd = MXC_DMA_WIDTH_BYTE,
    .dstwd = MXC_DMA_WIDTH_BYTE,
    .srcinc_en = 1,
    .dstinc_en = 0
  };

  mxc_dma_srcdst_t srcdst = {
    .ch = dmaCh,
    .source = (void*)pData,
    .len = len
  };

  switch (uartNum) {
  case 0:
    config.reqsel = MXC_DMA_REQUEST_UART0TX;
    break;

  case 1:
    config.reqsel = MXC_DMA_REQUEST_UART1TX;
    break;

  case 2:
    config.reqsel = MXC_DMA_REQUEST_UART2TX;
    break;

  case 3:
    config.reqsel = MXC_DMA_REQUEST_UART3TX;
    break;

  default:
    PAL_SYS_ASSERT(0);
    return;
  }

  MXC_DMA_ConfigChannel(config, srcdst);
  MXC_DMA_SetCallback(dmaCh, palUartCallback);

  /* Enable Count-to-Zero (CTZ) interrupt */
  MXC_DMA_EnableInt(dmaCh);
  MXC_DMA_SetChannelInterruptEn(dmaCh, 0, 1);

  /* Set Tx FIFO threshold */
  uart->dma |= 2 << MXC_F_UART_REVB_DMA_TX_THD_VAL_POS;
  /* Enable channel transmission */
  uart->dma |= MXC_F_UART_REVB_DMA_TX_EN;

  /* Start the transfer */
  MXC_DMA_Start(dmaCh);
}
