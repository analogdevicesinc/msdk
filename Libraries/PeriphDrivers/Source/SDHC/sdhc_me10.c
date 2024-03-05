/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* **** Includes **** */
#include <string.h>
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_pins.h"
#include "gpio.h"
#include "sdhc.h"
#include "sdhc_reva.h"

/* **** Definitions **** */

/* **** Globals **** */

/* **** Functions **** */

/* ************************************************************************** */
int MXC_SDHC_Init(const mxc_sdhc_cfg_t *cfg)
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SDHC);

    MXC_GPIO_Config(&gpio_cfg_sdhc_0);
    MXC_GPIO_Config(&gpio_cfg_sdhc_1);
    gpio_cfg_sdhc_1.port->vssel |= gpio_cfg_sdhc_1.mask;
    gpio_cfg_sdhc_0.port->ds_sel0 |= gpio_cfg_sdhc_0.mask;
    gpio_cfg_sdhc_1.port->ds_sel0 |= gpio_cfg_sdhc_1.mask;

    return MXC_SDHC_RevA_Init((mxc_sdhc_reva_regs_t *)MXC_SDHC, cfg);
}

/* ************************************************************************** */
void MXC_SDHC_PowerUp(void)
{
    MXC_SDHC_RevA_PowerUp((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_PowerDown(void)
{
    MXC_SDHC_RevA_PowerDown((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Shutdown(void)
{
    MXC_SDHC_RevA_Shutdown((mxc_sdhc_reva_regs_t *)MXC_SDHC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SDHC);

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SDHC_Set_Clock_Config(unsigned int clk_div)
{
    MXC_SDHC_RevA_Set_Clock_Config((mxc_sdhc_reva_regs_t *)MXC_SDHC, clk_div);
}

/* ************************************************************************** */
unsigned int MXC_SDHC_Get_Clock_Config(void)
{
    /* clk_div is split across two fields in the register.  Build it up accordingly */
    return MXC_SDHC_RevA_Get_Clock_Config((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

unsigned int MXC_SDHC_Get_Input_Clock_Freq(void)
{
    if (MXC_GCR->pclk_div & MXC_F_GCR_PCLK_DIV_SDHCFRQ) {
        return SystemCoreClock >> 1; // Div by 2
    } else {
        return 50000000; // UG specifies a hard-coded 50Mhz value in this case
    }
}

/* ************************************************************************** */
int MXC_SDHC_SendCommand(mxc_sdhc_cmd_cfg_t *sd_cmd_cfg)
{
    return MXC_SDHC_RevA_SendCommand((mxc_sdhc_reva_regs_t *)MXC_SDHC, sd_cmd_cfg);
}

/* ************************************************************************** */
int MXC_SDHC_SendCommandAsync(mxc_sdhc_cmd_cfg_t *sd_cmd_cfg)
{
    return MXC_SDHC_RevA_SendCommandAsync((mxc_sdhc_reva_regs_t *)MXC_SDHC, sd_cmd_cfg);
}

/* ************************************************************************** */
void MXC_SDHC_Handler(void)
{
    MXC_SDHC_RevA_Handler((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Card_Inserted(void)
{
    return MXC_SDHC_RevA_Card_Inserted((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_ClearFlags(uint32_t mask)
{
    MXC_SDHC_RevA_ClearFlags((mxc_sdhc_reva_regs_t *)MXC_SDHC, mask);
}

/* ************************************************************************** */
unsigned MXC_SDHC_GetFlags(void)
{
    return MXC_SDHC_RevA_GetFlags((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Reset(void)
{
    MXC_SDHC_RevA_Reset((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Reset_CMD_DAT(void)
{
    MXC_SDHC_RevA_Reset_CMD_DAT((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
int MXC_SDHC_Card_Busy(void)
{
    return MXC_SDHC_RevA_Card_Busy((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
unsigned int MXC_SDHC_Get_Host_Cn_1(void)
{
    return MXC_SDHC_RevA_Get_Host_Cn_1((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
uint32_t MXC_SDHC_Get_Response32(void)
{
    return MXC_SDHC_RevA_Get_Response32((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
uint32_t MXC_SDHC_Get_Response32_Auto(void)
{
    /* The response for auto commands get set at idx 3 */
    return MXC_SDHC_RevA_Get_Response32_Auto((mxc_sdhc_reva_regs_t *)MXC_SDHC);
}

/* ************************************************************************** */
void MXC_SDHC_Get_Response128(unsigned char *response)
{
    MXC_SDHC_RevA_Get_Response128((mxc_sdhc_reva_regs_t *)MXC_SDHC, response);
}
