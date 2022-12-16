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
 * @file    pal_bb_dbb.h
 * @brief   Function headers to use the digital base band.
 */

#ifndef MAX32665_INCLUDE_PAL_BB_DBB_H_
#define MAX32665_INCLUDE_PAL_BB_DBB_H_

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

#define PAL_BB_DBB_SPI_WRITE_BIT 0x80
#define PAL_BB_DBB_SPI_ADDR_POS 8
#define PAL_BB_DBB_SPI_ADDR_MASK 0xFF00
#define PAL_BB_DBB_SPI_DATA_MASK 0x00FF

/**************************************************************************************************
  Type Definitions
**************************************************************************************************/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

bool_t PalBbDbbInit(void);
bool_t PalBbDbbRestore(void);

bool_t PalBbDbbSpiWrite(uint8_t address, uint8_t data);
bool_t PalBbDbbSpiRead(uint8_t address, uint8_t *data);
void PalBbDbbDelay(uint32_t us);

void PalBbDbbCmu1M(void);
void PalBbDbbCmu2M(void);

void PalBbDbbAgcSetDefaultGain(void);
void PalBbDbbAgcSetHsGain(void);
void PalBbDbbAgcRestoreDcOffs(uint8_t phy);

void PalBbDbbRxCancel(void);
void PalBbDbbTxCancel(void);
void PalBbDbbSetAccAddr(uint32_t addr);
void PalBbDbbSetCrcInit(uint32_t crcInit);
void PalBbDbbSetChannel(uint32_t channel);
void PalBbDbbSetRxTimeout(uint32_t timeout, uint8_t phy);
void PalBbDbbDisableTIFS(void);
void PalBbDbbEnableTIFS(void);
void PalBbDbbEnableEncryption(uint64_t pktCnt, uint8_t pktLen, uint32_t aad, uint8_t dir);
void PalBbDbbDisableEncryption(void);
void PalBbDbbEnableDecryption(uint64_t pktCnt, uint8_t dir);
void PalBbDbbDisableDecryption(void);

#ifdef __cplusplus
};
#endif

#endif // MAX32665_INCLUDE_PAL_BB_DBB_H_
