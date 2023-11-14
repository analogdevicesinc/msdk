/*************************************************************************************************/
/*!
 *  \file
 *
 *  \brief  Wireless Data Exchange profile implementation - File Example.
 *
 *  Copyright (c) 2013-2018 Arm Ltd. All Rights Reserved.
 *
 *  Portions Copyright (c) 2022-2023 Analog Devices, Inc.
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

#ifndef EXAMPLES_MAX32690_BLUETOOTH_BLE_OTAS_WDXS_FILE_H_
#define EXAMPLES_MAX32690_BLUETOOTH_BLE_OTAS_WDXS_FILE_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t fileLen;
    uint32_t fileCRC;
} fileHeader_t;

/*! \addtogroup WIRELESS_DATA_EXCHANGE_PROFILE
 *  \{ */

/**************************************************************************************************
  Constant Definitions
**************************************************************************************************/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Initialize the WDXS File.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WdxsFileInit(void);

/*************************************************************************************************/
/*!
 *  \brief  Get the base address of the WDXS file.
 *
 *  \return Base address of WDXS file.
 */
/*************************************************************************************************/
uint32_t WdxsFileGetBaseAddr(void);

/*************************************************************************************************/
/*!
 *  \brief  Get the length of the last verified WDXS file.
 *
 *  \return Verified length of WDXS file.
 */
/*************************************************************************************************/
uint32_t WdxsFileGetVerifiedLength(void);

/*************************************************************************************************/
/*!
 *  \brief  Get the firmware version of the WDXS file.
 *
 *  \return Firmware version of the WDXS file.
 */
/*************************************************************************************************/
uint16_t WdxsFileGetFirmwareVersion(void);
/*************************************************************************************************/
/*!
 *  \brief  set the length of the expected file
 *
 *  \return None.
 */
/*************************************************************************************************/
void initHeader(fileHeader_t *header);

/*! \} */ /* WIRELESS_DATA_EXCHANGE_PROFILE */

#ifdef __cplusplus
}
#endif

#endif // EXAMPLES_MAX32690_BLUETOOTH_BLE_OTAS_WDXS_FILE_H_
