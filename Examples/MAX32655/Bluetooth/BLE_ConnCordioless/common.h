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
#ifndef EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_COMMON_H_
#define EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "pal_bb_ble.h"

#ifdef __cplusplus
extern "C" {
#endif
/*******************************************************************************
  Definitions
*******************************************************************************/
#define DUE_OFFSET_US           1000
#define RX_TIMEOUT_US           10000
#define HEADER_LEN              2
#define ADDR_LEN                6
#define CHM_LEN                 5

#if DEBUG == 1
#define DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...) ((void)0)
#endif

/*******************************************************************************
  Global Variables
*******************************************************************************/
// extern uint8_t bbTxData;

/*******************************************************************************
  Functions
*******************************************************************************/
/*************************************************************************************************/
/*!
 *  \brief      Get TX data array.
 *
 *  \return     Pointer to the TX data array.
 */
/*************************************************************************************************/
uint8_t* common_getData(void);

/*************************************************************************************************/
/*!
 *  \brief      Clear TIFS in operation.
 *
 *  The BB may choose not to enable TIFS after the next TX or RX.
 */
/*************************************************************************************************/
static inline void common_setTifs(void)
{
    PalBbBleOpParam_t opParams = { .ifsMode=PAL_BB_IFS_MODE_TOGGLE_TIFS, .ifsTime=0, .pIfsChan=NULL };
    PalBbBleSetOpParams(&opParams);
}

/*************************************************************************************************/
/*!
 *  \brief      Set TIFS in operation.
 *
 *  The BB must enable TIFS after the next TX or RX.
 */
/*************************************************************************************************/
static inline void common_clearTifs(void)
{
    PalBbBleOpParam_t opParams = { .ifsMode=PAL_BB_IFS_MODE_CLR, .ifsTime=0, .pIfsChan=NULL };
    PalBbBleSetOpParams(&opParams);
}

#ifdef __cplusplus
};
#endif

#endif // EXAMPLES_MAX32655_BLUETOOTH_BLE_CONNCORDIOLESS_COMMON_H_