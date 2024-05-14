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
#ifndef EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_CORDIOLESS_SCAN_H_
#define EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_CORDIOLESS_SCAN_H_

#include <stdint.h>
#include "pal_bb_ble.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
  Definitions
*******************************************************************************/

typedef struct {
    uint32_t accessAddress;
    uint32_t crcInit;
    uint8_t winSize;
    uint16_t winOffset;
    uint16_t interval;
    uint16_t latency;
    uint16_t timeout;
    uint8_t *chM;
    uint8_t hop;
    uint8_t sca;
} ConnDataParams_t;

/*******************************************************************************
  Functions
*******************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief      Initialize pseudo-scanner.
 *
 *  \param      advAddr             Advertising address.
 *  \param      scanAddr            Scanning address.
 *  \param      pChan               Pointer to the channelization parameters.
 *  \param      pConn               Pointer to the connection parameters.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void initScanner(uint8_t *advAddr, uint8_t *scanAddr, PalBbBleChan_t *pChan, ConnDataParams_t *pConn);

/*************************************************************************************************/
/*!
 *  \brief      Start scanning.
 *
 *  \param      scanWin             Scanning window.
 * 
 *  \return     TRUE if a connection was established, else FALSE.
 */
/*************************************************************************************************/
bool_t startScanning(uint32_t scanWin);

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_CORDIOLESS_SCAN_H_
