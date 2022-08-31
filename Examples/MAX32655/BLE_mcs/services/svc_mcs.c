/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Maxim Custom service server.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "svc_mcs.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef MCS_SEC_PERMIT_READ
#define MCS_SEC_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

/*! Characteristic write permissions */
#ifndef MCS_SEC_PERMIT_WRITE
#define MCS_SEC_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/*Service variables declaration*/
const uint8_t attMcsSvcUuid[ATT_128_UUID_LEN] = { ATT_UUID_MCS_SERVICE };

/*Characteristic variables declaration*/
const uint8_t svcMcsButtonUuid[ATT_128_UUID_LEN] = { ATT_UUID_MCS_BUTTON };
const uint8_t svcMcsRUuid[ATT_128_UUID_LEN] = { ATT_UUID_MCS_R };
const uint8_t svcMcsGUuid[ATT_128_UUID_LEN] = { ATT_UUID_MCS_G };
const uint8_t svcMcsBUuid[ATT_128_UUID_LEN] = { ATT_UUID_MCS_B };

static const uint8_t mcsValSvc[] = { ATT_UUID_MCS_SERVICE };
static const uint16_t mcsLenSvc = sizeof(mcsValSvc);

static const uint8_t mcsButtonValCh[]
    = { ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(MCS_BUTTON_HDL), ATT_UUID_MCS_BUTTON };
static const uint16_t mcsButtonLenCh = sizeof(mcsButtonValCh);

static const uint8_t mcsRValCh[]
    = { ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(MCS_R_HDL), ATT_UUID_MCS_R };
static const uint16_t mcsRLenCh = sizeof(mcsRValCh);

static const uint8_t mcsGValCh[]
    = { ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(MCS_G_HDL), ATT_UUID_MCS_G };
static const uint16_t mcsGLenCh = sizeof(mcsGValCh);

static const uint8_t mcsBValCh[]
    = { ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(MCS_B_HDL), ATT_UUID_MCS_B };
static const uint16_t mcsBLenCh = sizeof(mcsBValCh);

/*Characteristic values declaration*/
static uint8_t mcsButtonVal[] = { 0 };
static const uint16_t mcsButtonValLen = sizeof(mcsButtonVal);

static uint8_t mcsButtonValChCcc[] = { UINT16_TO_BYTES(0x0000) };
static const uint16_t mcsButtonLenValChCcc = sizeof(mcsButtonValChCcc);

static uint8_t mcsRVal[] = { 0 };
static const uint16_t mcsRValLen = sizeof(mcsRVal);

static uint8_t mcsGVal[] = { 0 };
static const uint16_t mcsGValLen = sizeof(mcsGVal);

static uint8_t mcsBVal[] = { 0 };
static const uint16_t mcsBValLen = sizeof(mcsBVal);

/**************************************************************************************************
 Maxim Custom Service group
**************************************************************************************************/

/* Attribute list for mcs group */
static const attsAttr_t mcsList[] = {
    /*-----------------------------*/
    /* Service declaration */
    { attPrimSvcUuid, (uint8_t*)mcsValSvc, (uint16_t*)&mcsLenSvc, sizeof(mcsValSvc), 0,
        MCS_SEC_PERMIT_READ },

    /*----------------------------*/
    /* Button characteristic declaration */
    { attChUuid, (uint8_t*)mcsButtonValCh, (uint16_t*)&mcsButtonLenCh, sizeof(mcsButtonValCh), 0,
        MCS_SEC_PERMIT_READ },
    /* Button characteristic value */
    { svcMcsButtonUuid, (uint8_t*)mcsButtonVal, (uint16_t*)&mcsButtonValLen, sizeof(mcsButtonVal),
        0, MCS_SEC_PERMIT_READ },
    /*Button characteristic value descriptor*/
    { attCliChCfgUuid, (uint8_t*)mcsButtonValChCcc, (uint16_t*)&mcsButtonLenValChCcc,
        sizeof(mcsButtonValChCcc), ATTS_SET_CCC, (ATTS_PERMIT_READ | SVC_SEC_PERMIT_WRITE) },

    /*-----------------------------*/
    /* R characteristic declaration */
    { attChUuid, (uint8_t*)mcsRValCh, (uint16_t*)&mcsRLenCh, sizeof(mcsRValCh),
        ATTS_SET_WRITE_CBACK, (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) },
    /* R characteristic characteristic value */
    { svcMcsRUuid, (uint8_t*)mcsRVal, (uint16_t*)&mcsRValLen, sizeof(mcsRVal), ATTS_SET_WRITE_CBACK,
        (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) },

    /*-----------------------------*/
    /* G characteristic declaration */
    { attChUuid, (uint8_t*)mcsGValCh, (uint16_t*)&mcsGLenCh, sizeof(mcsGValCh),
        ATTS_SET_WRITE_CBACK, (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) },
    /* G characteristic characteristic value */
    { svcMcsGUuid, (uint8_t*)mcsGVal, (uint16_t*)&mcsGValLen, sizeof(mcsGVal), ATTS_SET_WRITE_CBACK,
        (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) },

    /*-----------------------------*/
    /*  B characteristic declaration */
    { attChUuid, (uint8_t*)mcsBValCh, (uint16_t*)&mcsBLenCh, sizeof(mcsBValCh),
        ATTS_SET_WRITE_CBACK, (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) },
    /* B characteristic value */
    { svcMcsBUuid, (uint8_t*)mcsBVal, (uint16_t*)&mcsBValLen, sizeof(mcsBVal), ATTS_SET_WRITE_CBACK,
        (MCS_SEC_PERMIT_READ | MCS_SEC_PERMIT_WRITE) }
};

/* Test group structure */
static attsGroup_t svcMcsGroup
    = { NULL, (attsAttr_t*)mcsList, NULL, NULL, MCS_START_HDL, MCS_END_HDL };

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcMcsAddGroup(void)
{
    AttsAddGroup(&svcMcsGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcMcsRemoveGroup(void)
{
    AttsRemoveGroup(MCS_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcMcsCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
    svcMcsGroup.readCback = readCback;
    svcMcsGroup.writeCback = writeCback;
}
