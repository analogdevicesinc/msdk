/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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
 *
 ******************************************************************************/

/*************************************************************************************************/
/*! Secure Data Service Client
*   Implements the necessary handles list to perform service 
*   and characteristic discovery of custom secured service .
*
 */
/*************************************************************************************************/

#ifndef EXAMPLES_MAX32655_BLUETOOTH_BLE_DATC_SDSC_API_H_
#define EXAMPLES_MAX32655_BLUETOOTH_BLE_DATC_SDSC_API_H_

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Secured service  enumeration of handle indexes of characteristics to be discovered */
enum {
    SEC_DAT_HDL_IDX, /*!< \brief Secured data */
    SEC_DAT_CCC_HDL_IDX, /*!< \brief Secured data client characteristic configuration descriptor */
    SEC_HDL_LIST_LEN /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Secured service .
 *          Parameter pHdlList must point to an array of length \ref SEC_HDL_LIST_LEN.
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

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32655_BLUETOOTH_BLE_DATC_SDSC_API_H_
