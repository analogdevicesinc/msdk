/*************************************************************************************************/
/*!
 *  \file   hci_tr.c
 *
 *  \brief  HCI transport module.
 *
 *  Copyright (c) 2011-2018 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
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
#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "util/bstream.h"
#include "hci_api.h"
#include "hci_core.h"
#include "hci_tr.h"
#include "hci_core_ps.h"

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
#include "ll_api.h"
#endif

#if defined(HCI_TR_UART) && (HCI_TR_UART == 1)

#include "pal_uart.h"

/*! \brief      Transport packet states. */
typedef enum {
    HCI_TR_COMP = 0, /*!< HCI packet complete. */
    HCI_TR_TYPE = 1, /*!< HCI type. */
    HCI_TR_LEN = 2, /*!< HCI length. */
    HCI_TR_PAYLOAD = 3, /*!< HCI payload. */
} hciTrState_t;

/*! \brief      Transport control block. */
typedef struct {
    hciTrState_t wrState;
    uint16_t wrLen;
    uint8_t *wrBuf;
    uint8_t wrType;
    void *wrContext;
    uint8_t *wrData;

    hciTrState_t rdState;
    uint16_t rdLen;
    uint8_t *rdBuf;
    uint8_t rdType;
    uint8_t rdOpLen[4];
} hciTrCtrlBlk_t;

static hciTrCtrlBlk_t hciTrCtrlBlk;

/*************************************************************************************************/
/*!
 *  \brief  Send a complete ACL buffer to the transport.
 *
 *  \param  pCmdData    WSF msg buffer containing an HCI command.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciTrSendAcl(uint8_t *pAclData)
{
    /* Wait for previous message to complete */
    while (hciTrCtrlBlk.wrState != HCI_TR_COMP) {}

    /* save the length and buffer */
    hciTrCtrlBlk.wrLen = pAclData[2] + (pAclData[3] << 8) + HCI_ACL_HDR_LEN;
    hciTrCtrlBlk.wrBuf = pAclData;
    hciTrCtrlBlk.wrType = HCI_ACL_TYPE;

    /* Start the write */
    hciTrCtrlBlk.wrState = HCI_TR_TYPE;
    PalUartWriteData(PAL_UART_ID_CHCI, &hciTrCtrlBlk.wrType, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  Restart the read state machine.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciTrRdRestart(void)
{
    hciTrCtrlBlk.rdState = HCI_TR_TYPE;
    PalUartReadData(PAL_UART_ID_CHCI, &hciTrCtrlBlk.rdType, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  UART Read callback.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciTrRdCback(void)
{
    switch (hciTrCtrlBlk.rdState) {
    case HCI_TR_TYPE:
        /* Type received, get the length */
        hciTrCtrlBlk.rdState = HCI_TR_LEN;
        switch (hciTrCtrlBlk.rdType) {
        case HCI_EVT_TYPE:
            PalUartReadData(PAL_UART_ID_CHCI, (uint8_t *)&hciTrCtrlBlk.rdOpLen, 2);
            break;
        case HCI_ACL_TYPE:
            PalUartReadData(PAL_UART_ID_CHCI, (uint8_t *)&hciTrCtrlBlk.rdOpLen, 4);
            break;
        case HCI_ISO_TYPE:
            PalUartReadData(PAL_UART_ID_CHCI, (uint8_t *)&hciTrCtrlBlk.rdOpLen, 4);
            break;
        default:
            /* Unknown HCI packet type */
            WSF_ASSERT(0);
            hciTrRdRestart();
            return;
        }
        break;

    case HCI_TR_LEN:
        /* Type and length received, get the payload */
        hciTrCtrlBlk.rdState = HCI_TR_PAYLOAD;
        switch (hciTrCtrlBlk.rdType) {
        case HCI_EVT_TYPE:
            hciTrCtrlBlk.rdLen = hciTrCtrlBlk.rdOpLen[1];
            break;
        case HCI_ACL_TYPE:
            hciTrCtrlBlk.rdLen = hciTrCtrlBlk.rdOpLen[2] | (hciTrCtrlBlk.rdOpLen[3] << 8);
            break;
        case HCI_ISO_TYPE:
            hciTrCtrlBlk.rdLen = hciTrCtrlBlk.rdOpLen[2] | (hciTrCtrlBlk.rdOpLen[3] << 8);
            break;
        }

        /* Add room for the opcode/length bytes */
        hciTrCtrlBlk.rdBuf = WsfMsgAlloc(hciTrCtrlBlk.rdLen + 4);
        if (hciTrCtrlBlk.rdBuf == NULL) {
            WSF_ASSERT(0);
            hciTrRdRestart();
            return;
        }

        /* Fill in the opcode and length, start the read for the payload */
        switch (hciTrCtrlBlk.rdType) {
        case HCI_EVT_TYPE:
            memcpy(&hciTrCtrlBlk.rdBuf[0], hciTrCtrlBlk.rdOpLen, 2);
            PalUartReadData(PAL_UART_ID_CHCI, &hciTrCtrlBlk.rdBuf[2], hciTrCtrlBlk.rdLen);
            break;
        case HCI_ACL_TYPE:
        case HCI_ISO_TYPE:
            memcpy(&hciTrCtrlBlk.rdBuf[0], hciTrCtrlBlk.rdOpLen, 4);
            PalUartReadData(PAL_UART_ID_CHCI, &hciTrCtrlBlk.rdBuf[4], hciTrCtrlBlk.rdLen);
            break;
        }
        break;

    case HCI_TR_PAYLOAD:
        /* Payload received */
        hciCoreRecv(hciTrCtrlBlk.rdType, hciTrCtrlBlk.rdBuf);
        hciTrCtrlBlk.rdBuf = NULL;
        hciTrRdRestart();
        break;

    default:
    case HCI_TR_COMP:
        /* Unexpected state */
        WSF_ASSERT(0);
        hciTrRdRestart();
        return;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  UART Write callback.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void hciTrWrCback(void)
{
    switch (hciTrCtrlBlk.wrState) {
    case HCI_TR_TYPE:
        /* Send the payload */
        hciTrCtrlBlk.wrState = HCI_TR_PAYLOAD;
        PalUartWriteData(PAL_UART_ID_CHCI, hciTrCtrlBlk.wrBuf, hciTrCtrlBlk.wrLen);
        break;

    case HCI_TR_PAYLOAD:
        hciTrCtrlBlk.wrState = HCI_TR_COMP;
        if (hciTrCtrlBlk.wrType == HCI_CMD_TYPE) {
            /* Packet complete, free the buffer */
            WsfMsgFree(hciTrCtrlBlk.wrBuf);
        } else if (hciTrCtrlBlk.wrType == HCI_ACL_TYPE) {
            WsfMsgFree(hciTrCtrlBlk.wrBuf);
            hciCoreTxAclComplete(hciTrCtrlBlk.wrContext, hciTrCtrlBlk.wrData);
        } else {
            WSF_ASSERT(0); /* Undefined packet type */
        }
        break;

    default:
    case HCI_TR_COMP:
        WSF_ASSERT(0); /* Undefined state */
        return;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Send a complete HCI command to the transport.
 *
 *  \param  pCmdData    WSF msg buffer containing an HCI command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciTrSendCmd(uint8_t *pCmdData)
{
    /* Wait for previous command to complete */
    while (hciTrCtrlBlk.wrState != HCI_TR_COMP) {}

    /* save the length and buffer */
    hciTrCtrlBlk.wrLen = pCmdData[2] + HCI_CMD_HDR_LEN;
    hciTrCtrlBlk.wrBuf = pCmdData;

    /* Start the write */
    hciTrCtrlBlk.wrState = HCI_TR_TYPE;
    hciTrCtrlBlk.wrType = HCI_CMD_TYPE;
    PalUartWriteData(PAL_UART_ID_CHCI, &hciTrCtrlBlk.wrType, 1);
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize HCI transport resources.
 *
 *  \param  port        COM port.
 *  \param  baudRate    Baud rate.
 *  \param  flowControl TRUE if flow control is enabled
 *
 *  \return TRUE if initialization succeeds, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciTrInit(uint8_t port, uint32_t baudRate, bool_t flowControl)
{
    PalUartConfig_t uartConfig;

    /* Initialize the read and write states */
    hciTrCtrlBlk.wrState = HCI_TR_COMP;
    hciTrCtrlBlk.rdState = HCI_TR_COMP;

    /* Initialize the transport UART */
    uartConfig.rdCback = hciTrRdCback;
    uartConfig.wrCback = hciTrWrCback;
    uartConfig.baud = baudRate;
    uartConfig.hwFlow = flowControl;

    PalUartInit(PAL_UART_ID_CHCI, &uartConfig);

    /* Start the read */
    hciTrRdRestart();

    return TRUE;
}

#endif

/*************************************************************************************************/
/*!
 *  \fn     hciTrSendAclData
 *
 *  \brief  Send a complete HCI ACL packet to the transport.
 *
 *  \param  pContext Connection context.
 *  \param  pData    WSF msg buffer containing an ACL packet.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciTrSendAclData(void *pContext, uint8_t *pData)
{
    uint16_t len;
    uint8_t *p;

    /* if fragmenting */
    if (hciCoreTxAclDataFragmented(pContext)) {
        /* get 16-bit length */
        BYTES_TO_UINT16(len, (pData + 2))
        len += HCI_ACL_HDR_LEN;

        /* allocate LL buffer */
        if ((p = WsfMsgDataAlloc(len, HCI_TX_DATA_TAILROOM)) != NULL) {
            /* copy data */
            memcpy(p, pData, len);

#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
            /* send to LL  */
            LlSendAclData(p);

            /* free HCI buffer */
            hciCoreTxAclComplete(pContext, pData);
#endif

#if defined(HCI_TR_UART) && (HCI_TR_UART == 1)
            hciTrCtrlBlk.wrContext = pContext;
            hciTrCtrlBlk.wrData = pData;
            hciTrSendAcl(p);
#endif
        }
    } else {
#if defined(HCI_TR_EXACTLE) && (HCI_TR_EXACTLE == 1)
        /* send to LL  */
        LlSendAclData(pData);

        /* LL will free HCI buffer */
        hciCoreTxAclComplete(pContext, NULL);
#endif

#if defined(HCI_TR_UART) && (HCI_TR_UART == 1)
        hciTrCtrlBlk.wrContext = pContext;
        hciTrCtrlBlk.wrData = NULL;
        hciTrSendAcl(pData);
#endif
    }
}
