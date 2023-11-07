/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/*************************************************************************************************/
/*! Secure Data Service 
*   Implementation of a characteristic with elevated security features.
*   The connection must be encrypted with an authenticated key to read/write
*   the attributes value.
*
 */
/*************************************************************************************************/

#ifndef EXAMPLES_MAX32665_BLE_DATS_SERVICES_SVC_SDS_H_
#define EXAMPLES_MAX32665_BLE_DATS_SERVICES_SVC_SDS_H_

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Service and Characteristic UUIDs
**************************************************************************************************/
#define ATT_UUID_SEC_DATA_SERVICE \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x42, 0xD9, 0x32, 0x7E, 0x36, 0xFC, 0x42
/* MCS service GATT characteristic UUIDs*/
#define ATT_UUID_SEC_DATA \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x41, 0xD9, 0x31, 0x3E, 0x56, 0xFC, 0x42

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/
/*! \brief Secured Data Service */
#define SEC_DATA_START_HDL 0x300 /*!< \brief Start handle. */
#define SEC_DATA_END_HDL (SEC_DAT_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Secured Service Handles */
enum {
    SEC_DATA_SVC_HDL = SEC_DATA_START_HDL, /*!< \brief Secured Data service declaration */
    SEC_DAT_CH_HDL, /*!< \brief Secured Data characteristic */
    SEC_DAT_HDL, /*!< \brief Secured Data  */
    SEC_DAT_CH_CCC_HDL, /*!< \brief Secured Data client characteristic configuration */
    SEC_DAT_MAX_HDL /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataRemoveGroup(void);

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
void SvcSecDataCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */ /* WP_SERVICE */

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32665_BLE_DATS_SERVICES_SVC_SDS_H_
