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
/*! Secure Data Service Client
*   Implements the necessary handles list to perform service 
*   and characteristic discovery of custom secured service .
*
 */
/*************************************************************************************************/

#ifndef EXAMPLES_MAX32655_BLE_DATC_SDSC_API_H_
#define EXAMPLES_MAX32655_BLE_DATC_SDSC_API_H_

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

#endif // EXAMPLES_MAX32655_BLE_DATC_SDSC_API_H_
