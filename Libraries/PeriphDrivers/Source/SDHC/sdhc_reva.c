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
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "sdhc.h"
#include "sdhc_reva.h"

/* **** Definitions **** */

/* **** Globals **** */
mxc_sdhc_callback_fn sdhc_callback = NULL;

/* **** Functions **** */
static void MXC_SDHC_FreeCallback(int error);
static int MXC_SDHC_TransSetup(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg);

/* ************************************************************************** */
void MXC_SDHC_RevA_Set_Clock_Config(mxc_sdhc_reva_regs_t *sdhc, unsigned int clk_div)
{
    sdhc->clk_cn = 0;
    /* clk_div is split across two fields in the register.  Break it up accordingly */
    sdhc->clk_cn = (clk_div & 0xff) << MXC_F_SDHC_REVA_CLK_CN_SDCLK_FREQ_SEL_POS;
    sdhc->clk_cn |= ((clk_div & 0x300) >> 8) << MXC_F_SDHC_REVA_CLK_CN_UPPER_SDCLK_FREQ_SEL_POS;
    sdhc->clk_cn |= MXC_F_SDHC_REVA_CLK_CN_INTERNAL_CLK_EN;

    while (!(sdhc->clk_cn & MXC_F_SDHC_REVA_CLK_CN_INTERNAL_CLK_STABLE)) {}

    sdhc->clk_cn |= MXC_F_SDHC_REVA_CLK_CN_SD_CLK_EN;
}

/* ************************************************************************** */
unsigned int MXC_SDHC_RevA_Get_Clock_Config(mxc_sdhc_reva_regs_t *sdhc)
{
    /* clk_div is split across two fields in the register.  Build it up accordingly */
    return ((((sdhc->clk_cn >> MXC_F_SDHC_REVA_CLK_CN_UPPER_SDCLK_FREQ_SEL_POS) << 8) & 0x300) |
            ((sdhc->clk_cn >> MXC_F_SDHC_REVA_CLK_CN_SDCLK_FREQ_SEL_POS) & 0xff));
}

/* ************************************************************************** */
int MXC_SDHC_RevA_Init(mxc_sdhc_reva_regs_t *sdhc, const mxc_sdhc_cfg_t *cfg)
{
    MXC_ASSERT(cfg);

    if (cfg->clk_div > 0x3FF) {
        return E_BAD_PARAM;
    }

    MXC_SDHC_Reset();

    /* Turn on bus supply and enable clock */
    sdhc->pwr = (cfg->bus_voltage << MXC_F_SDHC_REVA_PWR_BUS_VOLT_SEL_POS) &
                MXC_F_SDHC_REVA_PWR_BUS_VOLT_SEL;

    sdhc->blk_gap = cfg->block_gap;

    sdhc->host_cn_1 = 0x00;

    MXC_SDHC_Set_Clock_Config(cfg->clk_div);

    /* Set TO to max until we know better */
    sdhc->to = MXC_F_SDHC_REVA_TO_DATA_COUNT_VALUE;

    /* Note: This only enables bits to show up in the int_stat register */
    /* The int_signal register is really what you want to generate interrupts out of the IP block */
    sdhc->int_en = 0xffff;
    sdhc->er_int_en = 0xffff;

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SDHC_RevA_PowerUp(mxc_sdhc_reva_regs_t *sdhc)
{
    sdhc->pwr |= MXC_F_SDHC_REVA_PWR_BUS_POWER;
}

/* ************************************************************************** */
void MXC_SDHC_RevA_PowerDown(mxc_sdhc_reva_regs_t *sdhc)
{
    sdhc->pwr &= ~MXC_F_SDHC_REVA_PWR_BUS_POWER;
}

/* ************************************************************************** */
int MXC_SDHC_RevA_Shutdown(mxc_sdhc_reva_regs_t *sdhc)
{
    /* Disable and clear interrupts */
    sdhc->int_en = 0;
    sdhc->er_int_en = 0;
    sdhc->int_stat = sdhc->int_stat;
    sdhc->er_int_stat = sdhc->er_int_stat;

    if (sdhc_callback != NULL) {
        MXC_SDHC_FreeCallback(E_SHUTDOWN);
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
static int MXC_SDHC_TransSetup(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg)
{
    if (!MXC_SDHC_Card_Inserted()) {
        return E_NO_DEVICE;
    }

    if (sdhc->present & MXC_F_SDHC_REVA_PRESENT_CMD) {
        /* Command already in progress */
        return E_BAD_STATE;
    }

    sdhc->clk_cn |= MXC_F_SDHC_REVA_CLK_CN_SD_CLK_EN;

    sdhc->arg_1 = sd_cmd_cfg->arg_1;

    uint32_t hc1 = sd_cmd_cfg->host_control_1;

    if (sd_cmd_cfg->direction == MXC_SDHC_DIRECTION_WRITE ||
        sd_cmd_cfg->direction == MXC_SDHC_DIRECTION_READ) {
        hc1 &=
            ~(MXC_F_SDHC_REVA_HOST_CN_1_DMA_SELECT | MXC_F_SDHC_REVA_HOST_CN_1_CARD_DETECT_SIGNAL);
    }

    sdhc->host_cn_1 = hc1;

    /* Clear all flags */
    sdhc->int_stat = sdhc->int_stat;
    sdhc->er_int_stat = sdhc->er_int_stat;

    /* Set up Transfer registers */
    if (sd_cmd_cfg->direction != MXC_SDHC_DIRECTION_CFG) {
        sdhc->trans = 0;
        sdhc->sdma = sd_cmd_cfg->sdma;

        if (sd_cmd_cfg->dma) {
            sdhc->trans |= MXC_F_SDHC_REVA_TRANS_DMA_EN;
        }

        if (sd_cmd_cfg->direction == MXC_SDHC_DIRECTION_WRITE) {
            sdhc->trans &= ~(MXC_F_SDHC_REVA_TRANS_READ_WRITE);
        } else {
            sdhc->trans |= MXC_F_SDHC_REVA_TRANS_READ_WRITE;
        }

        sdhc->blk_size = MXC_F_SDHC_REVA_BLK_SIZE_HOST_BUFF |
                         ((sd_cmd_cfg->block_size << MXC_F_SDHC_REVA_BLK_SIZE_TRANS_POS) &
                          MXC_F_SDHC_REVA_BLK_SIZE_TRANS);

        /* Determine transfer size and options */
        if (sd_cmd_cfg->block_count > 1) {
            /* Enable multi-block transfers, enable block count register, and automatically issue CMD12 to stop transfer */
            sdhc->trans |= (MXC_F_SDHC_REVA_TRANS_MULTI | MXC_F_SDHC_REVA_TRANS_BLK_CNT_EN |
                            MXC_S_SDHC_REVA_TRANS_AUTO_CMD_EN_CMD12);
            sdhc->blk_cnt = sd_cmd_cfg->block_count;
        }

    } else {
        sdhc->trans = 0;
        sdhc->sdma = 0;
    }

    return E_NO_ERROR;
}

/* ************************************************************************** */
int MXC_SDHC_RevA_SendCommand(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg)
{
    int err;

    if ((err = MXC_SDHC_TransSetup(sdhc, sd_cmd_cfg)) != E_NO_ERROR) {
        return err;
    }

    /* Start transfer */
    sdhc->cmd = sd_cmd_cfg->command;

    /* Block on completion */
    if (sd_cmd_cfg->direction == MXC_SDHC_DIRECTION_CFG) {
        /* No data transfer, just command */
        while (!(sdhc->int_stat & MXC_F_SDHC_REVA_INT_STAT_CMD_COMP) &&
               !(sdhc->int_stat & MXC_F_SDHC_REVA_INT_STAT_ERR_INTR)) {}
    } else {
        while (!(sdhc->int_stat & MXC_F_SDHC_REVA_INT_STAT_TRANS_COMP) &&
               !(sdhc->int_stat & MXC_F_SDHC_REVA_INT_STAT_ERR_INTR)) {}
    }

    /* Determine if transfer was successful or not */
    if (sdhc->int_stat & MXC_F_SDHC_REVA_INT_STAT_ERR_INTR) {
        if (sdhc->er_int_stat &
            (MXC_F_SDHC_REVA_ER_INT_STAT_CMD_TO | MXC_F_SDHC_REVA_ER_INT_STAT_DATA_TO)) {
            return E_TIME_OUT;
        } else {
            return E_COMM_ERR;
        }
    } else {
        return E_NO_ERROR;
    }
}

/* ************************************************************************** */
int MXC_SDHC_RevA_SendCommandAsync(mxc_sdhc_reva_regs_t *sdhc, mxc_sdhc_cmd_cfg_t *sd_cmd_cfg)
{
    int err;

    if ((err = MXC_SDHC_TransSetup(sdhc, sd_cmd_cfg)) != E_NO_ERROR) {
        return err;
    }

    sdhc_callback = sd_cmd_cfg->callback;

    if (sd_cmd_cfg->direction == MXC_SDHC_DIRECTION_CFG) {
        sdhc->int_signal = MXC_F_SDHC_REVA_INT_SIGNAL_CMD_COMP;
    } else {
        sdhc->int_signal = MXC_F_SDHC_REVA_INT_SIGNAL_TRANS_COMP;
    }

    /* Start transfer */
    sdhc->cmd = sd_cmd_cfg->command;

    return E_NO_ERROR;
}

/* ************************************************************************** */
void MXC_SDHC_RevA_Handler(mxc_sdhc_reva_regs_t *sdhc)
{
    int signal = sdhc->int_signal;
    int flag = MXC_SDHC_GetFlags() & signal;

    // Need to check if there is anything to do in case this function is called
    //  in a polling fashion instead of from the interrupt handler.
    if (!flag) {
        return;
    }

    // Command complete interrupt
    if ((signal & MXC_F_SDHC_REVA_INT_SIGNAL_CMD_COMP) &&
        (flag & MXC_F_SDHC_REVA_INT_STAT_CMD_COMP)) {
        MXC_SDHC_ClearFlags(MXC_F_SDHC_REVA_INT_STAT_CMD_COMP);
        sdhc->int_signal &= ~MXC_F_SDHC_REVA_INT_SIGNAL_CMD_COMP;
        MXC_SDHC_FreeCallback(E_NO_ERROR);
        return;
    }

    // Transfer complete interrupt
    if ((signal & MXC_F_SDHC_REVA_INT_SIGNAL_TRANS_COMP) &&
        (flag & MXC_F_SDHC_REVA_INT_STAT_TRANS_COMP)) {
        MXC_SDHC_ClearFlags(MXC_F_SDHC_REVA_INT_STAT_TRANS_COMP);
        sdhc->int_signal &= ~MXC_F_SDHC_REVA_INT_SIGNAL_TRANS_COMP;
        MXC_SDHC_FreeCallback(E_NO_ERROR);
        return;
    }

    MXC_SDHC_ClearFlags(flag);
    sdhc->int_signal = 0;
    MXC_SDHC_FreeCallback(E_UNKNOWN);
}

/* ************************************************************************** */
void MXC_SDHC_RevA_ClearFlags(mxc_sdhc_reva_regs_t *sdhc, uint32_t mask)
{
    sdhc->int_stat = mask;
}

/* ************************************************************************** */
unsigned MXC_SDHC_RevA_GetFlags(mxc_sdhc_reva_regs_t *sdhc)
{
    return sdhc->int_stat;
}

/* ************************************************************************** */
int MXC_SDHC_RevA_Card_Inserted(mxc_sdhc_reva_regs_t *sdhc)
{
    unsigned int detect, inserted, stable;

    detect = !!(sdhc->present & MXC_F_SDHC_REVA_PRESENT_CARD_DETECT);
    inserted = !!(sdhc->present & MXC_F_SDHC_REVA_PRESENT_CARD_INSERTED);
    stable = !!(sdhc->present & MXC_F_SDHC_REVA_PRESENT_CARD_STATE);

    return (detect & inserted & stable);
}

/* ************************************************************************** */
void MXC_SDHC_RevA_Reset(mxc_sdhc_reva_regs_t *sdhc)
{
    sdhc->sw_reset = MXC_F_SDHC_REVA_SW_RESET_RESET_ALL;

    /* Reset takes non-zero time, so wait for completion */
    while (sdhc->sw_reset & MXC_F_SDHC_REVA_SW_RESET_RESET_ALL) {}
}

/* ************************************************************************** */
void MXC_SDHC_RevA_Reset_CMD_DAT(mxc_sdhc_reva_regs_t *sdhc)
{
    sdhc->sw_reset = MXC_F_SDHC_REVA_SW_RESET_RESET_CMD | MXC_F_SDHC_REVA_SW_RESET_RESET_DAT;

    /* Reset takes non-zero time, so wait for completion */
    while (sdhc->sw_reset &
           (MXC_F_SDHC_REVA_SW_RESET_RESET_CMD | MXC_F_SDHC_REVA_SW_RESET_RESET_DAT)) {}
}

/* ************************************************************************** */
int MXC_SDHC_RevA_Card_Busy(mxc_sdhc_reva_regs_t *sdhc)
{
    /* Response type 1b uses the DAT[0] line low to indicate busy */
    return (!((sdhc->present >> MXC_F_SDHC_REVA_PRESENT_DAT_SIGNAL_LEVEL_POS) & 1));
}

/* ************************************************************************** */
unsigned int MXC_SDHC_RevA_Get_Host_Cn_1(mxc_sdhc_reva_regs_t *sdhc)
{
    return sdhc->host_cn_1;
}

/* ************************************************************************** */
uint32_t MXC_SDHC_RevA_Get_Response32(mxc_sdhc_reva_regs_t *sdhc)
{
    return sdhc->resp[0];
}

/* ************************************************************************** */
uint32_t MXC_SDHC_RevA_Get_Response32_Auto(mxc_sdhc_reva_regs_t *sdhc)
{
    /* The response for auto commands get set at idx 3 */
    return sdhc->resp[3];
}

/* ************************************************************************** */
void MXC_SDHC_RevA_Get_Response128(mxc_sdhc_reva_regs_t *sdhc, unsigned char *response)
{
    uint32_t tmp;

    tmp = sdhc->resp[0];
    response[0] = tmp;
    response[1] = tmp >> 8;
    response[2] = tmp >> 16;
    response[3] = tmp >> 24;

    tmp = sdhc->resp[1];
    response[4] = tmp;
    response[5] = tmp >> 8;
    response[6] = tmp >> 16;
    response[7] = tmp >> 24;

    tmp = sdhc->resp[2];
    response[8] = tmp;
    response[9] = tmp >> 8;
    response[10] = tmp >> 16;
    response[11] = tmp >> 24;

    tmp = sdhc->resp[3];
    response[12] = tmp;
    response[13] = tmp >> 8;
    response[14] = tmp >> 16;
    response[15] = tmp >> 24;
}

/* ************************************************************************** */
static void MXC_SDHC_FreeCallback(int error)
{
    /* Save the request so the callback can be NULLed out and still be called. */
    mxc_sdhc_callback_fn temp_callback = sdhc_callback;

    sdhc_callback = NULL;

    temp_callback(error);
}
