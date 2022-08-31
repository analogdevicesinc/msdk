/* *****************************************************************************
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
 * $Date: 2019-10-25 17:20:09 -0500(Fri, 25 Oct 2019) $
 * $Revision: 48104 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_sys.h"
#include "sdhc.h"
#include "sdhc_reva.h"
#include <string.h>

/* **** Definitions **** */

/* **** Globals **** */

/* ************************************************************************** */
void MXC_SDHC_Set_Clock_Config(unsigned int clk_div)
{
    MXC_SDHC_RevA_Set_Clock_Config((mxc_sdhc_reva_regs_t*)MXC_SDHC, clk_div);
}

/* ************************************************************************** */
unsigned int MXC_SDHC_Get_Clock_Config(void)
{
    return MXC_SDHC_RevA_Get_Clock_Config((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Init(const mxc_sdhc_cfg_t* cfg)
{
    mxc_gpio_regs_t* gpio = gpio_cfg_sdhc.port;

    // Startup the HIRC96M clock if it's not on already
    if (!(MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC96M_EN)) {
        MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC96M_EN;

        if (MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCN_HIRC96M_RDY) != E_NO_ERROR) {
            return E_TIME_OUT;
        }
    }

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SDHC);

    gpio->ds0 |= gpio_cfg_sdhc.mask;

    MXC_GPIO_Config(&gpio_cfg_sdhc);
    return MXC_SDHC_RevA_Init((mxc_sdhc_reva_regs_t*)MXC_SDHC, cfg);
}

/* ************************************************************************** */
void MXC_SDHC_PowerUp(void)
{
    MXC_SDHC_RevA_PowerUp((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_PowerDown(void)
{
    MXC_SDHC_RevA_PowerDown((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Shutdown(void)
{
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SDHC);
    return MXC_SDHC_RevA_Shutdown((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_SendCommand(mxc_sdhc_cmd_cfg_t* sd_cmd_cfg)
{
    return MXC_SDHC_RevA_SendCommand((mxc_sdhc_reva_regs_t*)MXC_SDHC, sd_cmd_cfg);
}

/* ************************************************************************** */
int MXC_SDHC_SendCommandAsync(mxc_sdhc_cmd_cfg_t* sd_cmd_cfg)
{
    return MXC_SDHC_RevA_SendCommandAsync((mxc_sdhc_reva_regs_t*)MXC_SDHC, sd_cmd_cfg);
}

/* ************************************************************************** */
void MXC_SDHC_Handler(void)
{
    MXC_SDHC_RevA_Handler((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_ClearFlags(uint32_t mask)
{
    MXC_SDHC_RevA_ClearFlags((mxc_sdhc_reva_regs_t*)MXC_SDHC, mask);
}

/* ************************************************************************** */
unsigned MXC_SDHC_GetFlags(void)
{
    return MXC_SDHC_RevA_GetFlags((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Card_Inserted(void)
{
    return MXC_SDHC_RevA_Card_Inserted((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Reset(void)
{
    MXC_SDHC_RevA_Reset((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Reset_CMD_DAT(void)
{
    MXC_SDHC_RevA_Reset_CMD_DAT((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Card_Busy(void)
{
    return MXC_SDHC_RevA_Card_Busy((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
unsigned int MXC_SDHC_Get_Host_Cn_1(void)
{
    return MXC_SDHC_RevA_Get_Host_Cn_1((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
uint32_t MXC_SDHC_Get_Response32(void)
{
    return MXC_SDHC_RevA_Get_Response32((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
uint32_t MXC_SDHC_Get_Response32_Auto(void)
{
    return MXC_SDHC_RevA_Get_Response32_Auto((mxc_sdhc_reva_regs_t*)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Get_Response128(unsigned char* response)
{
    MXC_SDHC_RevA_Get_Response128((mxc_sdhc_reva_regs_t*)MXC_SDHC, response);
}
