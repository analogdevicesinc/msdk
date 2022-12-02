/*************************************************************************************************/
/*!
 * \file
 *
 * \brief  System configuration definition.
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

#include "pal_cfg.h"
#include "pal_sys.h"
#include "ll_defs.h"
#include "mxc_sys.h"

/*! \brief  LL configuration. */
typedef struct {
    /* Advertiser */
    uint8_t maxAdvSets; /*!< Maximum number of advertising sets. */
    uint8_t maxAdvReports; /*!< Maximum number of pending legacy or extended advertising reports. */
    uint16_t maxExtAdvDataLen; /*!< Maximum extended advertising data size. */
    uint8_t defExtAdvDataFragLen; /*!< Default extended advertising data fragmentation size. */
    uint16_t auxDelayUsec; /*!< Additional Auxiliary Offset delay above T_MAFS in microseconds. */
    uint16_t
        auxPtrOffsetUsec; /*!< Delay of auxiliary packet in microseconds from the time specified by auxPtr. */
    /* Scanner */
    uint8_t maxScanReqRcvdEvt; /*!< Maximum scan request received events. */
    uint16_t maxExtScanDataLen; /*!< Maximum extended scan data size. */
    /* Connection */
    uint8_t maxConn; /*!< Maximum number of connections. */
    uint8_t numTxBufs; /*!< Default number of transmit buffers. */
    uint8_t numRxBufs; /*!< Default number of receive buffers. */
    uint16_t maxAclLen; /*!< Maximum ACL buffer size. */
    int8_t defTxPwrLvl; /*!< Default Tx power level for connections. */
    uint8_t
        ceJitterUsec; /*!< Allowable CE jitter on a slave (account for master's sleep clock resolution). */
    /* ISO */
    uint8_t numIsoTxBuf; /*!< Default number of ISO transmit buffers. */
    uint8_t numIsoRxBuf; /*!< Default number of ISO receive buffers. */
    uint16_t maxIsoBufLen; /*!< Maximum ISO buffer size between host and controller. */
    uint16_t maxIsoPduLen; /*!< Maximum ISO PDU size between controllers. */

    /* CIS */
    uint8_t maxCig; /*!< Maximum number of CIG. */
    uint8_t maxCis; /*!< Maximum number of CIS, it is shared by the CIGs. */
    uint16_t cisSubEvtSpaceDelay; /*!< Subevent spacing above T_MSS. */

    /* BIS*/
    uint8_t maxBig; /*!< Maximum number of BIG. */
    uint8_t maxBis; /*!< Maximum number of BIS. */
    /* DTM */
    uint16_t dtmRxSyncMs; /*!< DTM Rx synchronization window in milliseconds. */
} PalCfgLl_t;

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief convert uint32_t to little endian byte stream, incrementing four bytes. */
#define PAL_UINT32_TO_BSTREAM(p, n)    \
    {                                  \
        *(p)++ = (uint8_t)(n);         \
        *(p)++ = (uint8_t)((n) >> 8);  \
        *(p)++ = (uint8_t)((n) >> 16); \
        *(p)++ = (uint8_t)((n) >> 24); \
    }

/**************************************************************************************************
  Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Get BLE PHY feature configuration.
 *
 *  \param      phy2mSup            2M PHY supported.
 *  \param      phyCodedSup         Coded PHY supported.
 *  \param      stableModIdxTxSup   Tx stable modulation index supported.
 *  \param      stableModIdxRxSup   Rx stable modulation index supported.
 */
/*************************************************************************************************/
void palCfgGetBlePhyFeatures(uint8_t *pPhy2mSup, uint8_t *pPhyCodedSup, uint8_t *pStableModIdxTxSup,
                             uint8_t *pStableModIdxRxSup)
{
    *pPhy2mSup = TRUE;
    *pPhyCodedSup = TRUE;
    *pStableModIdxTxSup = FALSE;
    *pStableModIdxRxSup = FALSE;
}

/*************************************************************************************************/
/*!
 *  \brief      Load LL advertising configuration.
 *
 *  \param      pConfig                Return configuration values.
 */
/*************************************************************************************************/
void palCfgLoadLlParams(uint8_t *pConfig)
{
    PalCfgLl_t *pCfg = (PalCfgLl_t *)pConfig;

    const uint16_t advDataLen = LL_MAX_ADV_DATA_LEN;
    const uint16_t connDataLen = 512;
    const uint16_t numTxBufs = 16;

    pCfg->maxAdvSets = 6;
    pCfg->maxAdvReports = 16;
    pCfg->maxExtAdvDataLen = advDataLen;
    /* pCfg->defExtAdvDataFragLen */ /* Use default. */
    pCfg->auxDelayUsec = 0;
    pCfg->maxScanReqRcvdEvt = 4;
    pCfg->maxExtScanDataLen = advDataLen;
    pCfg->maxConn = 4;
    pCfg->maxAclLen = connDataLen;
    pCfg->numTxBufs = numTxBufs;
    pCfg->numRxBufs = 8;
    pCfg->numIsoTxBuf = 16;
    pCfg->numIsoRxBuf = 8;
    pCfg->maxIsoBufLen = 512;
    pCfg->maxIsoPduLen = 251;
    pCfg->maxCig = 2;
    pCfg->maxCis = 6;
    pCfg->cisSubEvtSpaceDelay = 0;
    pCfg->maxBig = 2;
    pCfg->maxBis = 6;
}

/*************************************************************************************************/
/*!
 *  \brief      Load device address.
 *
 *  \param      pDevAddr            device address.
 */
/*************************************************************************************************/
void palCfgLoadBdAddress(uint8_t *pDevAddr)
{
    uint8_t id[MXC_SYS_USN_CHECKSUM_LEN];
    uint8_t checksum[MXC_SYS_USN_CHECKSUM_LEN];

    if (MXC_SYS_GetUSN((uint8_t *)id, (uint8_t *)checksum) != E_NO_ERROR) {
        PalSysAssertTrap();
    }

    /* MA-L assigend by IEEE to Maxim Integrated Products */
    pDevAddr[5] = 0x00;
    pDevAddr[4] = 0x18;
    pDevAddr[3] = 0x80;

    pDevAddr[2] = checksum[2];
    pDevAddr[1] = checksum[1];
    pDevAddr[0] = checksum[0];
}

/*************************************************************************************************/
/*!
 *  \brief      Load 15.4 address.
 *
 *  \param      pDevAddr            device address.
 */
/*************************************************************************************************/
void palCfgLoadExtMac154Address(uint8_t *pDevAddr)
{
    unsigned int devAddrLen = 8;
    uint8_t id[MXC_SYS_USN_CHECKSUM_LEN];
    uint8_t checksum[MXC_SYS_USN_CHECKSUM_LEN];

    if (MXC_SYS_GetUSN(id, checksum) != E_NO_ERROR) {
        PalSysAssertTrap();
    }

    /* Set the device address */
    unsigned int i = 0;
    while (i < devAddrLen) {
        pDevAddr[i] = checksum[i];
        i++;
    }
}

/*************************************************************************************************/
/*!
 *  \brief      Set device UUID.
 *
 *  \param      pBuf                Return device UUID.
 */
/*************************************************************************************************/
void PalCfgSetDeviceUuid(uint8_t *pBuf)
{
    /* Not used on this platform. */
    (void)pBuf;
}

/*************************************************************************************************/
/*!
 *  \brief      Load device UUID.
 *
 *  \param      cfgId                Configuration ID.
 *  \param      pBuf                 Buffer.
 *  \param      len                  Buffer length.
 */
/*************************************************************************************************/
void PalCfgLoadData(uint8_t cfgId, uint8_t *pBuf, uint32_t len)
{
    (void)len;

    switch (cfgId) {
    case PAL_CFG_ID_BD_ADDR:
        palCfgLoadBdAddress(pBuf);
        break;

    case PAL_CFG_ID_BLE_PHY:
        palCfgGetBlePhyFeatures(pBuf, pBuf + 1, pBuf + 2, pBuf + 3);
        break;

    case PAL_CFG_ID_LL_PARAM:
        palCfgLoadLlParams(pBuf);
        break;

    case PAL_CFG_ID_MAC_ADDR:
        palCfgLoadExtMac154Address(pBuf);
        break;

    default:
        break;
    }
}
