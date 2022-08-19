/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Arm Ltd. proprietary profile client.
 *
 *  Copyright (c) 2012-2018 Arm Ltd. All Rights Reserved.
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
#ifndef SDSC_API_H
#define SDSC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif



/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief Arm Ltd. proprietary service P1 enumeration of handle indexes of characteristics to be discovered */
enum
{
  SEC_DAT_HDL_IDX,           /*!< \brief Proprietary data */
  SEC_DAT_CCC_HDL_IDX,        /*!< \brief Proprietary data client characteristic configuration descriptor */
  SEC_HDL_LIST_LEN           /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Arm Ltd. proprietary service P1.
 *          Parameter pHdlList must point to an array of length \ref WPC_P1_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SecDatSvcDiscover(dmConnId_t connId, uint16_t *pHdlList);

/*! \} */    /* PACKETCRAFT_PROPRIETARY_PROFILE */

#ifdef __cplusplus
};
#endif

#endif /* SDSC_API_H */
