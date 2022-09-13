/* *****************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-08-22 15:47:30 -0500 (Wed, 22 Aug 2018) $
 * $Revision: 37251 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "spixr.h"
#include "spixr_reva.h"
#include "tpu.h"
#include "mxc_errors.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Local Function Prototypes **** */

/* ************************************************************************* */
int MXC_SPIXR_ReadRXFIFO(uint8_t *buf, int len)
{
    return MXC_SPIXR_RevA_ReadRXFIFO((mxc_spixr_reva_regs_t *)MXC_SPIXR, buf, len);
}

/* ************************************************************************* */
int MXC_SPIXR_WriteTXFIFO(uint8_t *buf, int len)
{
    return MXC_SPIXR_RevA_WriteTXFIFO((mxc_spixr_reva_regs_t *)MXC_SPIXR, buf, len);
}

/* ************************************************************************* */
void MXC_SPIXR_SetSS(void)
{
    MXC_SPIXR_RevA_SetSS((mxc_spixr_reva_regs_t *)MXC_SPIXR, 0);
}

/* ************************************************************************* */
int MXC_SPIXR_GetSS(void)
{
    return MXC_SPIXR_RevA_GetSS((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_SetSSCtrl(int stayActive)
{
    if (stayActive) {
        MXC_SPIXR_RevA_SetSSCtrl((mxc_spixr_reva_regs_t *)MXC_SPIXR, 1);
    } else {
        MXC_SPIXR_RevA_SetSSCtrl((mxc_spixr_reva_regs_t *)MXC_SPIXR, 0);
    }
}

/* ************************************************************************* */
int MXC_SPIXR_GetSSCtrl(void)
{
    return MXC_SPIXR_RevA_GetSSCtrl((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_Enable(void)
{
    MXC_SPIXR_RevA_Enable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_Disable(void)
{
    MXC_SPIXR_RevA_Disable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_IsEnabled(void)
{
    return MXC_SPIXR_RevA_IsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_TXFIFOEnable(void)
{
    MXC_SPIXR_RevA_TXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_TXFIFODisable(void)
{
    MXC_SPIXR_RevA_TXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_TXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_TXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_DMATXFIFOEnable(void)
{
    MXC_SPIXR_RevA_DmaTXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_DMATXFIFODisable(void)
{
    MXC_SPIXR_RevA_DmaTXFIFODisable((mxc_spixr_reva_regs_t*)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_DMATXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_DmaTXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_RXFIFOEnable(void)
{
    MXC_SPIXR_RevA_RXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_RXFIFODisable(void)
{
    MXC_SPIXR_RevA_RXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_RXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_RXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_DMARXFIFOEnable(void)
{
    MXC_SPIXR_RevA_DmaRXFIFOEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_DMARXFIFODisable(void)
{
    MXC_SPIXR_RevA_DmaRXFIFODisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_DMARXFIFOIsEnabled(void)
{
    return MXC_SPIXR_RevA_DmaRXFIFOIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_ThreeWireModeEnable(void)
{
    MXC_SPIXR_RevA_ThreeWireModeEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_ThreeWireModeDisable(void)
{
    MXC_SPIXR_RevA_ThreeWireModeDisable((mxc_spixr_reva_regs_t*)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_ThreeWireModeIsEnabled(void)
{
    return MXC_SPIXR_RevA_ThreeWireModeIsEnabled((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_GetTXFIFOCount(void)
{
    return MXC_SPIXR_RevA_GetTXFIFOCount((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_GetRXFIFOCount(void)
{
    return MXC_SPIXR_RevA_GetRXFIFOCount((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_TXFIFOClear(void)
{
    MXC_SPIXR_RevA_TXFIFOClear((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_RXFIFOClear(void)
{
    MXC_SPIXR_RevA_RXFIFOClear((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_SetWidth(mxc_spixr_width_t width)
{
    return MXC_SPIXR_RevA_SetWidth((mxc_spixr_reva_regs_t *)MXC_SPIXR, width);
}

/* ************************************************************************* */
int MXC_SPIXR_SetSPIMode(mxc_spixr_mode_t mode)
{
    return MXC_SPIXR_RevA_SetSPIMode((mxc_spixr_reva_regs_t *)MXC_SPIXR, mode);
}

/* ************************************************************************* */
int MXC_SPIXR_SetSSPolarity(int activeLow)
{
    return MXC_SPIXR_RevA_SetSSPolarity((mxc_spixr_reva_regs_t *)MXC_SPIXR, !activeLow);
}

/* ************************************************************************* */
void MXC_SPIXR_SetSSTiming(unsigned int ssIActDelay, unsigned int postActive,
                           unsigned int preActive)
{
    MXC_SPIXR_RevA_SetSSTiming((mxc_spixr_reva_regs_t *)MXC_SPIXR, ssIActDelay, postActive,
                               preActive);
}

/* ************************************************************************* */
int MXC_SPIXR_SetFrequency(int hz)
{
    return MXC_SPIXR_RevA_SetFrequency((mxc_spixr_reva_regs_t *)MXC_SPIXR, hz);
}

/* ************************************************************************* */
int MXC_SPIXR_GetFrequency(void)
{
    return MXC_SPIXR_RevA_GetFrequency((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_GetIntFlags(void)
{
    return MXC_SPIXR_RevA_GetIntFlags((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_EnableInt(int flags)
{
    MXC_SPIXR_RevA_EnableInt((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

/* ************************************************************************* */
void MXC_SPIXR_DisableInt(int flags)
{
    MXC_SPIXR_RevA_DisableInt((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

/* ************************************************************************* */
int MXC_SPIXR_GetWakeUpFlags(void)
{
    return MXC_SPIXR_RevA_GetWakeUpFlags((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_EnableWakeUp(int flags)
{
    MXC_SPIXR_RevA_EnableWakeUp((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

/* ************************************************************************* */
void MXC_SPIXR_DisableWakeUp(int flags)
{
    MXC_SPIXR_RevA_DisableWakeUp((mxc_spixr_reva_regs_t *)MXC_SPIXR, flags);
}

/* ************************************************************************* */
void MXC_SPIXR_ExMemEnable(void)
{
    MXC_SPIXR_RevA_ExMemEnable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_ExMemDisable(void)
{
    MXC_SPIXR_RevA_ExMemDisable((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_ExMemUseDummy(int delay255)
{
    MXC_SPIXR_RevA_ExMemUseDummy((mxc_spixr_reva_regs_t *)MXC_SPIXR, delay255);
}

/* ************************************************************************* */
void MXC_SPIXR_ExMemSetWriteCommand(uint8_t command)
{
    MXC_SPIXR_RevA_ExMemSetWriteCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, command);
}

/* ************************************************************************* */
uint8_t MXC_SPIXR_ExMemGetWriteCommand(void)
{
    return MXC_SPIXR_RevA_ExMemGetWriteCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
void MXC_SPIXR_ExMemSetReadCommand(uint8_t command)
{
    MXC_SPIXR_RevA_ExMemSetReadCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, command);
}

/* ************************************************************************* */
uint8_t MXC_SPIXR_ExMemGetReadCommand(void)
{
    return MXC_SPIXR_RevA_ExMemGetReadCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_Busy(void)
{
    return MXC_SPIXR_RevA_Busy((mxc_spixr_reva_regs_t *)MXC_SPIXR);
}

/* ************************************************************************* */
int MXC_SPIXR_Init(mxc_spixr_cfg_t *cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    MXC_TPU_Init(MXC_SYS_PERIPH_CLOCK_TPU);
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    MXC_GPIO_Config(&gpio_cfg_spixr);

    return MXC_SPIXR_RevA_Init((mxc_spixr_reva_regs_t *)MXC_SPIXR, (mxc_spixr_reva_cfg_t *)cfg);
}

/* ************************************************************************* */
int MXC_SPIXR_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SCACHE);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIPR);
    return E_NO_ERROR;
}

/* ************************************************************************* */
void MXC_SPIXR_SendCommand(uint8_t *cmd, uint32_t length, uint32_t tx_num_char)
{
    MXC_SPIXR_RevA_SendCommand((mxc_spixr_reva_regs_t *)MXC_SPIXR, cmd, length, tx_num_char);
}
