/* *****************************************************************************
 * Copyright (C) Analog Devices, All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 **************************************************************************** */

/**
 * @file    pal_bb_afe.h
 * @brief   Function headers to use the analog front end.
 */

#ifndef MAX32665_INCLUDE_PAL_BB_AFE_H_
#define MAX32665_INCLUDE_PAL_BB_AFE_H_

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include "pal_bb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Type Definitions
**************************************************************************************************/

typedef struct {
    uint8_t div_int;
    uint8_t b_osc;
    uint8_t ib_lobuf_dac;
    uint8_t ib_vco_dac;

    /* DIV int and frac adjustments for different power levels */
    uint8_t div_int_4;
    uint8_t div_int_2;
    uint8_t div_int_0;
    uint8_t div_int_n2;

    uint16_t div_frac_4;
    uint16_t div_frac_2;
    uint16_t div_frac_0;
    uint16_t div_frac_n2;

    uint16_t div_frac;
    uint16_t f_dev;
    uint16_t f_dev_2M;
} palBbAfeCh_t;

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

bool_t PalBbAfeTxSetup(void);
bool_t PalBbAfeRxSetup(void);
bool_t PalBbAfeTxDone(void);
bool_t PalBbAfeRxDone(void);

bool_t PalBbAfeSetChannel(uint8_t channel);

bool_t PalBbAfeInit(void);
bool_t PalBbAfeSetup(void);
bool_t PalBbAfeCalibrate(void);
bool_t PalBbAfeRestore(void);
bool_t PalBbAfeEnable(void);
bool_t PalBbAfeDisable(void);

int8_t PalBbAfeGetIndexTxPower(unsigned index);
int8_t PalBbAfeGetTxPower(void);
unsigned PalBbAfeGetTxPowerLevels(void);
uint8_t PalBbAfeGetTxPowerPa(int8_t txPower);
bool_t PalBbAfeSetTxPower(int8_t txPower);
bool_t PalBbAfeSetTxPowerIndex(unsigned index);
bool_t PalBbAfeSaveCal(void);

#ifdef __cplusplus
};
#endif

#endif // MAX32665_INCLUDE_PAL_BB_AFE_H_
