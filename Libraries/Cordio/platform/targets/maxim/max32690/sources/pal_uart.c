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

#include "board.h"

#include "uart.h"
#include "sema.h"

#include <string.h>

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Number of UARTs this driver will use */
#ifndef PAL_UARTS
#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
#define PAL_UARTS 5
#else
#define PAL_UARTS 4
#endif
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
    mxc_uart_req_t readReq;
    mxc_uart_req_t writeReq;
    PalUartCompCback_t rdCback;
    PalUartCompCback_t wrCback;
} palUartCb[PAL_UARTS];

/**************************************************************************************************
  Local Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      UART Interrupt handlers.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void UART0_IRQHandler(void)
{
    int result;
    PalLedOn(PAL_LED_ID_CPU_ACTIVE);
    result = MXC_UART_AsyncHandler(MXC_UART0);
    (void)result;
    PAL_SYS_ASSERT(result == 0);
}
void UART1_IRQHandler(void)
{
    int result;
    PalLedOn(PAL_LED_ID_CPU_ACTIVE);
    result = MXC_UART_AsyncHandler(MXC_UART1);
    (void)result;
    PAL_SYS_ASSERT(result == 0);
}
void UART2_IRQHandler(void)
{
    int result;
    PalLedOn(PAL_LED_ID_CPU_ACTIVE);
    result = MXC_UART_AsyncHandler(MXC_UART2);
    (void)result;
    PAL_SYS_ASSERT(result == 0);
}
void UART3_IRQHandler(void)
{
    int result;
    PalLedOn(PAL_LED_ID_CPU_ACTIVE);
    result = MXC_UART_AsyncHandler(MXC_UART3);
    (void)result;
    PAL_SYS_ASSERT(result == 0);
}

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
void palUartCallback(mxc_uart_req_t *req, int error)
{
    int i;
    for (i = 0; i < PAL_UARTS; i++) {
        /* Find the corresponding rqeuest and call the callback */
        if (req == &palUartCb[i].readReq) {
            if (palUartCb[i].rdCback != NULL) {
                palUartCb[i].rdCback();
            }
            return;
        }

        if (req == &palUartCb[i].writeReq) {
            palUartCb[i].state = PAL_UART_STATE_READY;
            if (palUartCb[i].wrCback != NULL) {
                palUartCb[i].wrCback();
            }
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
    if (palUartCb[PAL_UARTS - 1].wrCback != NULL) {
        palUartCb[PAL_UARTS - 1].state = PAL_UART_STATE_READY;
        palUartCb[PAL_UARTS - 1].wrCback();
    }
}

void palMailReadCb(int result)
{
    if (palUartCb[PAL_UARTS - 1].rdCback != NULL) {
        palUartCb[PAL_UARTS - 1].rdCback();
    }
}

static void PalMailWriteData(const uint8_t *pData, uint16_t len)
{
    palUartCb[PAL_UARTS - 1].state = PAL_UART_STATE_BUSY;
    MXC_SEMA_WriteBoxAsync(palMailWriteCb, pData, len);
}

static void PalMailReadData(uint8_t *pData, uint16_t len)
{
    MXC_SEMA_ReadBoxAsync(palMailReadCb, pData, len);
}

static void PalMailInit(const PalUartConfig_t *pCfg)
{
    unsigned uartNum = PAL_UARTS - 1;

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
    uint8_t uartNum;
    int result;

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
    if (id == PAL_UART_ID_CHCI) {
        PalMailInit(pCfg);
        return;
    }
#endif

    uartNum = palUartGetNum(id);

    /* Save the callback */
    palUartCb[uartNum].rdCback = pCfg->rdCback;
    palUartCb[uartNum].wrCback = pCfg->wrCback;

    /* Initialize the UART */
    if (uartNum == 3) {
        /* Use the IBRO clock for UART3 */
        result = MXC_UART_Init(MXC_UART_GET_UART(uartNum), pCfg->baud, MXC_UART_IBRO_CLK);
    } else {
        /* Use the APB clock for rest of the UARTs */
        result = MXC_UART_Init(MXC_UART_GET_UART(uartNum), pCfg->baud, MXC_UART_APB_CLK);
    }

    (void)result;
    PAL_SYS_ASSERT(result == 0);

    MXC_UART_SetDataSize(MXC_UART_GET_UART(uartNum), 8);
    MXC_UART_SetStopBits(MXC_UART_GET_UART(uartNum), MXC_UART_STOP_1);
    MXC_UART_SetParity(MXC_UART_GET_UART(uartNum), MXC_UART_PARITY_DISABLE);
    if (pCfg->hwFlow) {
        MXC_UART_SetFlowCtrl(MXC_UART_GET_UART(uartNum), MXC_UART_FLOW_EN, 1);
    }

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
    int result;

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
    uint8_t uartNum;
    uint32_t irqn;
    int result;

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
    if (id == PAL_UART_ID_CHCI) {
        PalMailReadData(pData, len);
        return;
    }
#endif

    uartNum = palUartGetNum(id);
    irqn = MXC_UART_GET_IRQ(uartNum);

    palUartCb[uartNum].readReq.uart = MXC_UART_GET_UART(uartNum);
    palUartCb[uartNum].readReq.rxData = pData;
    palUartCb[uartNum].readReq.rxLen = len;
    palUartCb[uartNum].readReq.txLen = 0;
    palUartCb[uartNum].readReq.callback = palUartCallback;

    NVIC_DisableIRQ(irqn);

    /* Start the read */
    result = MXC_UART_TransactionAsync(&palUartCb[uartNum].readReq);
    (void)result;
    PAL_SYS_ASSERT(result == E_SUCCESS);

    /* Enable the interrupt */
    NVIC_EnableIRQ(irqn);
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
    uint8_t uartNum;
    uint32_t irqn;
    int result;

#if defined(HCI_TR_MAIL) && (HCI_TR_MAIL != 0)
    if (id == PAL_UART_ID_CHCI) {
        PalMailWriteData(pData, len);
        return;
    }
#endif

    uartNum = palUartGetNum(id);
    irqn = MXC_UART_GET_IRQ(uartNum);

    NVIC_DisableIRQ(irqn);

    palUartCb[uartNum].state = PAL_UART_STATE_BUSY;

    palUartCb[uartNum].writeReq.uart = MXC_UART_GET_UART(uartNum);
    palUartCb[uartNum].writeReq.txData = pData;
    palUartCb[uartNum].writeReq.txLen = len;
    palUartCb[uartNum].writeReq.rxLen = 0;
    palUartCb[uartNum].writeReq.callback = palUartCallback;

    /* Start the write */
    result = MXC_UART_TransactionAsync(&palUartCb[uartNum].writeReq);
    (void)result;
    PAL_SYS_ASSERT(result == E_SUCCESS);

    /* Enable the interrupt */
    NVIC_EnableIRQ(irqn);
}
