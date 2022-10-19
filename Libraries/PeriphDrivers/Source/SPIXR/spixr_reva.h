/* ****************************************************************************
 * Copyright(C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files(the "Software"),
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
 *************************************************************************** */

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXR_SPIXR_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXR_SPIXR_REVA_H_

/****** Includes *******/
#include "mxc_device.h"
#include "mxc_sys.h"
#include "spixr_reva_regs.h"
#include "spixr.h"

/* **** Definitions **** */
/**
 * @brief       Structure type for configuring a SPIXR port.
 */

/**
 * @brief       Data Width, # of data I/O used to rcv data
 */
typedef enum {
    MXC_SPIXR_REVA_SINGLE_SDIO = 0,
    MXC_SPIXR_REVA_DUAL_SDIO,
    MXC_SPIXR_REVA_QUAD_SDIO,
    MXC_SPIXR_REVA_INVALID,
} mxc_spixr_reva_width_t;

/**
 * @brief       SPIXF mode.
 * @note        modes 1 and 2 are not supported
 */
typedef enum {
    MXC_SPIXR_REVA_MODE_0,
    MXC_SPIXR_REVA_MODE_1,
    MXC_SPIXR_REVA_MODE_2,
    MXC_SPIXR_REVA_MODE_3,
} mxc_spixr_reva_mode_t;

/**
 * @brief       Configuration parameters of SPIXR
 */
typedef struct {
    uint32_t
        numbits; ///< Number of Bits per character. In slave mode 9-bit character length is not supported.
    mxc_spixr_reva_width_t data_width; ///< SPI Data width

    uint32_t ssel_act_1; ///< Slave Select Action delay 1
    uint32_t ssel_act_2; ///< Slave Select Action delay 2
    uint32_t ssel_inact; ///< Slave Select Inactive delay

    uint32_t baud_freq; ///< Desired frequency
} mxc_spixr_reva_cfg_t;

/***** Function Prototypes *****/

int MXC_SPIXR_RevA_ReadRXFIFO(mxc_spixr_reva_regs_t *spixr, uint8_t *buf, int len);
int MXC_SPIXR_RevA_WriteTXFIFO(mxc_spixr_reva_regs_t *spixr, uint8_t *buf, int len);
void MXC_SPIXR_RevA_SetSS(mxc_spixr_reva_regs_t *spixr, int ssIdx);
int MXC_SPIXR_RevA_GetSS(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_SetSSCtrl(mxc_spixr_reva_regs_t *spixr, int stayActive);
int MXC_SPIXR_RevA_GetSSCtrl(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_Enable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_Disable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_IsEnabled(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_ThreeWireModeEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_ThreeWireModeDisable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_ThreeWireModeIsEnabled(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_GetTXFIFOCount(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_GetRXFIFOCount(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_SetWidth(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_width_t width);
int MXC_SPIXR_RevA_SetSPIMode(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_mode_t mode);
int MXC_SPIXR_RevA_SetSSPolarity(mxc_spixr_reva_regs_t *spixr, int active);
void MXC_SPIXR_RevA_SetSSTiming(mxc_spixr_reva_regs_t *spixr, unsigned int ssIActDelay,
                                unsigned int postActive, unsigned int preActive);
int MXC_SPIXR_RevA_SetFrequency(mxc_spixr_reva_regs_t *spixr, int hz);
int MXC_SPIXR_RevA_GetFrequency(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_GetIntFlags(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_EnableInt(mxc_spixr_reva_regs_t *spixr, int flags);
void MXC_SPIXR_RevA_DisableInt(mxc_spixr_reva_regs_t *spixr, int flags);
int MXC_SPIXR_RevA_GetWakeUpFlags(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_EnableWakeUp(mxc_spixr_reva_regs_t *spixr, int flags);
void MXC_SPIXR_RevA_DisableWakeUp(mxc_spixr_reva_regs_t *spixr, int flags);
void MXC_SPIXR_RevA_ExMemEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_ExMemDisable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_ExMemUseDummy(mxc_spixr_reva_regs_t *spixr, int delay255);
void MXC_SPIXR_RevA_ExMemSetWriteCommand(mxc_spixr_reva_regs_t *spixr, uint8_t command);
uint8_t MXC_SPIXR_RevA_ExMemGetWriteCommand(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_ExMemSetReadCommand(mxc_spixr_reva_regs_t *spixr, uint8_t command);
uint8_t MXC_SPIXR_RevA_ExMemGetReadCommand(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_Busy(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_Init(mxc_spixr_reva_regs_t *spixr, mxc_spixr_reva_cfg_t *cfg);
int MXC_SPIXR_RevA_Shutdown(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_SendCommand(mxc_spixr_reva_regs_t *spixr, uint8_t *cmd, uint32_t length,
                                uint32_t tx_num_char);
void MXC_SPIXR_RevA_TXFIFOEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_TXFIFODisable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_TXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_DmaTXFIFOEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_DmaTXFIFODisable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_DmaTXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_RXFIFOEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_RXFIFODisable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_RXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_DmaRXFIFOEnable(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_DmaRXFIFODisable(mxc_spixr_reva_regs_t *spixr);
int MXC_SPIXR_RevA_DmaRXFIFOIsEnabled(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_TXFIFOClear(mxc_spixr_reva_regs_t *spixr);
void MXC_SPIXR_RevA_RXFIFOClear(mxc_spixr_reva_regs_t *spixr);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXR_SPIXR_REVA_H_
