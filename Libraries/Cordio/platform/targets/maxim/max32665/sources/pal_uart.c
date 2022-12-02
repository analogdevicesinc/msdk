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
 */
/*************************************************************************************************/

#include "pal_uart.h"
#include "pal_led.h"
#include "pal_sys.h"

#include "wsf_cs.h"

#include "uart.h"
#include "dma.h"
#include "nvic_table.h"

#include "board.h"

#include <string.h>

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Number of UARTs this driver will use */
#ifndef PAL_UARTS
#define PAL_UARTS 3
#endif

#ifndef TERMINAL_BAUD
#define TERMINAL_BAUD 115200
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

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

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
    for (i = 0; i < PAL_UARTS; i++) {
        /* Find the corresponding rqeuest and call the callback */
        if ((ch == palUartCb[i].readCh) && (palUartCb[i].state != PAL_UART_STATE_UNINIT)) {
            palUartCb[i].readCh = -1;
            if (palUartCb[i].rdCback != NULL) {
                palUartCb[i].rdCback();
            }

            MXC_DMA_ReleaseChannel(ch);
            return;
        }

        if ((ch == palUartCb[i].writeCh) && (palUartCb[i].state != PAL_UART_STATE_UNINIT)) {
            palUartCb[i].writeCh = -1;
            palUartCb[i].state = PAL_UART_STATE_READY;
            if (palUartCb[i].wrCback != NULL) {
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
static int palUartGetNum(PalUartId_t uartId)
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
        PAL_SYS_ASSERT(0);
        return -1;
        break;
    }

    return uartNum;
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
static sys_map_t palUartGetMap(PalUartId_t uartId)
{
    sys_map_t uartMap;
    switch (uartId) {
    case PAL_UART_ID_CHCI:
        uartMap = HCI_UART_MAP;
        break;
    case PAL_UART_ID_TERMINAL:
        uartMap = TERMINAL_UART_MAP;
        break;
    case PAL_UART_ID_USER:
        uartMap = USER_UART_MAP;
        break;
    default:
        PAL_SYS_ASSERT(0);
        return -1;
        break;
    }

    return uartMap;
}

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
    int uartNum = palUartGetNum(id);
    sys_map_t uartMap = palUartGetMap(id);
    int result;
    mxc_uart_regs_t *uart = MXC_UART_GET_UART(uartNum);

    /* Invalid UART num */
    if (uartNum < 0) {
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Save the callback */
    palUartCb[uartNum].rdCback = pCfg->rdCback;
    palUartCb[uartNum].wrCback = pCfg->wrCback;
    palUartCb[uartNum].readCh = -1;
    palUartCb[uartNum].writeCh = -1;

    /* Initialize the UART */
    result = MXC_UART_Init(uart, pCfg->baud, uartMap);
    (void)result;
    PAL_SYS_ASSERT(result >= 0);

    /* Disable UART interrupts */
    MXC_UART_DisableInt(uart, 0xFFFFFFFF);
    MXC_UART_ClearFlags(uart, 0xFFFFFFFF);

    MXC_UART_SetDataSize(uart, 8);
    MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);
    MXC_UART_SetParity(uart, MXC_UART_PARITY_DISABLE);
    if (pCfg->hwFlow) {
        MXC_UART_SetFlowCtrl(uart, MXC_UART_FLOW_EN_LOW, 1, uartMap);
    }

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
    int uartNum = palUartGetNum(id);
    int result;

    /* Invalid UART num */
    if (uartNum < 0) {
        PAL_SYS_ASSERT(0);
        return;
    }

    result = MXC_UART_Shutdown(MXC_UART_GET_UART(uartNum));
    (void)result;
    PAL_SYS_ASSERT(result);

    NVIC_DisableIRQ(MXC_UART_GET_IRQ(uartNum));

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
    int uartNum = palUartGetNum(id);

    /* Invalid UART num */
    if (uartNum < 0) {
        PAL_SYS_ASSERT(0);
        return PAL_UART_STATE_ERROR;
    }

    return palUartCb[uartNum].state;
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
    int uartNum = palUartGetNum(id);
    int dmaCh;
    mxc_uart_regs_t *uart = MXC_UART_GET_UART(uartNum);
    mxc_dma_srcdst_t srcdst;
    mxc_dma_config_t config;

    /* Invalid UART num */
    if (uartNum < 0) {
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Acquire the DMA channel */
    WsfCsEnter();
    dmaCh = MXC_DMA_AcquireChannel(MXC_DMA0);
    WsfCsExit();

    if ((dmaCh < 0) || (dmaCh > 7)) {
        /* DMA unavailable */
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Save the channel number */
    palUartCb[uartNum].readCh = dmaCh;

    /* Setup the DMA transfer */
    config.ch = dmaCh;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = dmaCh;
    srcdst.dest = (void *)pData;
    srcdst.len = len;

    /* Select the appropriate DMA request type */
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

    default:
        PAL_SYS_ASSERT(0);
        return;
    }

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(dmaCh, palUartCallback);
    MXC_DMA_EnableInt(dmaCh);

    MXC_DMA0->ch[dmaCh].cfg |= MXC_F_DMA_CFG_CTZIEN;
    uart->dma |= ((1 << MXC_F_UART_DMA_RXDMA_LEVEL_POS) | MXC_F_UART_DMA_RXDMA_EN);

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
    int uartNum = palUartGetNum(id);
    int dmaCh;
    mxc_uart_regs_t *uart = MXC_UART_GET_UART(uartNum);
    mxc_dma_srcdst_t srcdst;
    mxc_dma_config_t config;

    /* Invalid UART num */
    if (uartNum < 0) {
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Acquire the DMA channel */
    WsfCsEnter();
    dmaCh = MXC_DMA_AcquireChannel(MXC_DMA0);
    WsfCsExit();

    if ((dmaCh < 0) || (dmaCh > 7)) {
        /* DMA unavailable */
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Save the channel number */
    palUartCb[uartNum].writeCh = dmaCh;
    palUartCb[uartNum].state = PAL_UART_STATE_BUSY;

    /* Setup the DMA transfer */
    config.ch = dmaCh;

    config.srcwd = MXC_DMA_WIDTH_BYTE;
    config.dstwd = MXC_DMA_WIDTH_BYTE;

    config.srcinc_en = 1;
    config.dstinc_en = 0;

    srcdst.ch = dmaCh;
    srcdst.source = (void *)pData;
    srcdst.len = len;

    /* Select the appropriate DMA request type */
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

    default:
        PAL_SYS_ASSERT(0);
        return;
    }

    MXC_DMA_ConfigChannel(config, srcdst);
    MXC_DMA_SetCallback(dmaCh, palUartCallback);
    MXC_DMA_EnableInt(dmaCh);

    MXC_DMA0->ch[dmaCh].cfg |= MXC_F_DMA_CFG_CTZIEN;
    uart->dma |= ((2 << MXC_F_UART_DMA_TXDMA_LEVEL_POS) | MXC_F_UART_DMA_TXDMA_EN);

    /* Start the transfer */
    MXC_DMA_Start(dmaCh);
}
