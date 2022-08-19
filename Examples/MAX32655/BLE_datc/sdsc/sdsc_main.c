/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Arm Ltd. proprietary profile client.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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
#include "wsf_assert.h"
#include "util/bstream.h"
#include "app_api.h"
#include "sdsc_api.h"


/**************************************************************************************************
  UUIDs
**************************************************************************************************/
#define ATT_UUID_SEC_DATA_SERVICE \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x42, 0xD9, 0x32, 0x7E, 0x36, 0xFC, 0x42
/* MCS service GATT characteristic UUIDs*/
#define ATT_UUID_SEC_DATA  0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x41, 0xD9, 0x31, 0x3E, 0x56, 0xFC, 0x42
/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*!
 *  Arm Ltd. proprietary service P1
 */

/* UUIDs */
static const uint8_t SecDatSvcUuid[] = {ATT_UUID_SEC_DATA_SERVICE};    /*! Proprietary service P1 */
static const uint8_t SecDatChUuid[] = {ATT_UUID_SEC_DATA};        /*! Proprietary data D1 */

/* Characteristics for discovery */

/*! Proprietary data */
static const attcDiscChar_t secDat =
{
  SecDatChUuid,
  ATTC_SET_REQUIRED | ATTC_SET_UUID_128
};

/*! Proprietary data descriptor */
static const attcDiscChar_t secDatCcc =
{
  attCliChCfgUuid,
  ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR
};

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t *secDatDiscCharList[] =
{
  &secDat,                  /*! Proprietary data */
  &secDatCcc                /*! Proprietary data descriptor */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(SEC_HDL_LIST_LEN == ((sizeof(secDatDiscCharList) / sizeof(attcDiscChar_t *))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Arm Ltd. proprietary service P1.
 *          Parameter pHdlList must point to an array of length WPC_P1_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SecDatSvcDiscover(dmConnId_t connId, uint16_t *pHdlList)
{
  AppDiscFindService(connId, ATT_128_UUID_LEN, (uint8_t *) SecDatSvcUuid,
                     SEC_HDL_LIST_LEN, (attcDiscChar_t **) secDatDiscCharList, pHdlList);
}

