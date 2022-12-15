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
 * @file    pal_bb_seq.h
 * @brief   Function headers to use the afe sequencers.
 */

#ifndef MAX32665_INCLUDE_PAL_BB_SEQ_H_
#define MAX32665_INCLUDE_PAL_BB_SEQ_H_

/**************************************************************************************************
  Includes
**************************************************************************************************/

#include "pal_bb.h"
#include "pal_bb_afe.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/**************************************************************************************************
  Type Definitions
**************************************************************************************************/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

void PalBbSeqInit(void);

void PalBbSeqUpdate(uint8_t addr, uint8_t mask, uint8_t value);

bool_t PalBbSeqSetChannel(palBbAfeCh_t *rxChCfg, palBbAfeCh_t *txChCfg);

void PalBbSeqSetPA(uint8_t reg);

#ifdef __cplusplus
};
#endif

#endif // MAX32665_INCLUDE_PAL_BB_SEQ_H_
