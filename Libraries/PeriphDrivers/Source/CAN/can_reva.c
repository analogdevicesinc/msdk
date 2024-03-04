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

#include "can_reva.h"

/*********** Macros ***********/
#define MAJOR_VERSION_SHIFT 8
#define MAJOR_VERSION_CMSIS 2
#define MINOR_VERSION_CMSIS 8
#define MAJOR_VERSION_MSDK 1
#define MINOR_VERSION_MSDK 0

// Prefix these with MXC_CAN_ and put in can.h?
#define SINGLE_FILT_STD_ID_RTR_SHIFT (4U)
#define SINGLE_FILT_STD_ID_SHIFT (5U)
#define SINGLE_FILT_STD_ID_MASK (0x7FFU)
#define SINGLE_FILT_STD_ID(id) ((SINGLE_FILT_STD_ID_MASK & (id)) << SINGLE_FILT_STD_ID_SHIFT)
#define SINGLE_FILT_EXT_ID_RTR_SHIFT (2U)
#define SINGLE_FILT_EXT_ID_SHIFT (3U)
#define SINGLE_FILT_EXT_ID_MASK (0x1FFFFFFFU)
#define SINGLE_FILT_EXT_ID(id) ((SINGLE_FILT_EXT_ID_MASK & (id)) << SINGLE_FILT_EXT_ID_SHIFT)

#define DUAL_FILT_STD_ID_RTR_SHIFT (4U)
#define DUAL_FILT_STD_ID_SHIFT (5U)
#define DUAL_FILT_STD_ID_MASK (0x7FFU)
#define DUAL_FILT_STD_ID(id) ((DUAL_FILT_STD_ID_MASK & (id)) << DUAL_FILT_STD_ID_SHIFT)
#define DUAL_FILT_EXT_ID_SHIFT (13U)
#define DUAL_FILT_EXT_ID_MASK (0x1FFFE000U)
#define DUAL_FILT_EXT_ID(id) ((DUAL_FILT_EXT_ID_MASK & (id)) >> DUAL_FILT_EXT_ID_SHIFT);

#define HALFWORD_BYTESWAP(half_wd) ((half_wd & 0xFF00) >> 8) | ((half_wd & 0xFF) << 8)
#define WORD_BYTESWAP(wd) \
    ((wd & 0xFF000000) >> 24) | ((wd & 0xFF0000) >> 8) | ((wd & 0xFF00) << 8) | ((wd & 0xFF) << 24)

/*********** Global Variables ***********/
mxc_can_object_event_cb_t obj_evt_cb[MXC_CAN_INSTANCES] = { NULL };
mxc_can_unit_event_cb_t unit_evt_cb[MXC_CAN_INSTANCES] = { NULL };

static uint32_t filt_in_use[MXC_CAN_INSTANCES] = { 0 };
static mxc_can_obj_cfg_t obj_state[MXC_CAN_INSTANCES] = { MXC_CAN_OBJ_CFG_INACTIVE };

static mxc_can_req_t *rx_req[MXC_CAN_INSTANCES];
static uint32_t rx_lock[MXC_CAN_INSTANCES] = { 0 };
static uint32_t tx_lock[MXC_CAN_INSTANCES] = { 0 };

static uint32_t dma_rx0[18] = { 0 };
static uint32_t dma_rx1[18] = { 0 };
static uint32_t rx_dma_lock[MXC_CAN_INSTANCES] = { 0 };

/*********** Functions ***********/
static int getNumBytes(uint32_t dlc, uint32_t fdf, uint32_t rtr)
{
    int num_bytes = 0;
    if (rtr) {
        return 0;
    } else if (dlc > 8 && fdf) {
        // CAN FD message with more than 8 bytes of data
        switch (dlc & 0xF) {
        case 9:
        case 10:
        case 11:
        case 12:
            num_bytes = 8 + (dlc & 0x7) * 4;
            break;
        case 13:
            num_bytes = 32;
            break;
        case 14:
            num_bytes = 48;
            break;
        case 15:
            num_bytes = 64;
            break;
        }
    } else if (dlc > 8 && !fdf) {
        // Normal CAN message with DLC greater than maximum number of data bytes, set to maximum
        num_bytes = 8;
    } else {
        // Normal CAN or CAN FD message with less than or equal to 8 bytes
        num_bytes = dlc;
    }

    return num_bytes;
}

/**********************************************************************************************************************************************************************/
mxc_can_drv_version_t MXC_CAN_RevA_GetVersion(void)
{
    mxc_can_drv_version_t version;
    version.api = (MAJOR_VERSION_CMSIS << MAJOR_VERSION_SHIFT) | MINOR_VERSION_CMSIS;
    version.drv = (MAJOR_VERSION_MSDK << MAJOR_VERSION_SHIFT) | MINOR_VERSION_MSDK;
    return version;
}

/**********************************************************************************************************************************************************************/
mxc_can_capabilities_t MXC_CAN_RevA_GetCapabilities(void)
{
    mxc_can_capabilities_t capabilities;
    capabilities.num_objects = MXC_CAN_INSTANCES;
    capabilities.reentrant_operation = 1;
    capabilities.fd_mode = 1;
    capabilities.restricted_mode = 1;
    capabilities.monitor_mode = 1;
    capabilities.internal_loopback = 1;
    capabilities.external_loopback = 0;
    capabilities.rsv = 0;
    return capabilities;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_Init(mxc_can_reva_regs_t *can, mxc_can_unit_event_cb_t unit_cb,
                      mxc_can_object_event_cb_t obj_cb)
{
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    obj_evt_cb[can_idx] = obj_cb; // Set callback function pointers
    unit_evt_cb[can_idx] = unit_cb;

    MXC_FreeLock(&tx_lock[can_idx]); // Free CAN resources
    MXC_FreeLock(&rx_lock[can_idx]);
    MXC_FreeLock(&rx_dma_lock[can_idx]);

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_UnInit(mxc_can_reva_regs_t *can)
{
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    obj_evt_cb[can_idx] = NULL; // Clear callback function pointers
    unit_evt_cb[can_idx] = NULL;

    MXC_FreeLock(&tx_lock[can_idx]); // Free CAN resources
    MXC_FreeLock(&rx_lock[can_idx]);
    MXC_FreeLock(&rx_dma_lock[can_idx]);

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_PowerControl(mxc_can_reva_regs_t *can, mxc_can_pwr_ctrl_t pwr)
{
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (pwr == MXC_CAN_PWR_CTRL_SLEEP) {
        // CAN periph able to go into sleep mode?
        if (can->stat & MXC_F_CAN_REVA_STAT_TX || can->intfl || can->eintfl || !can->wupclkdiv) {
            return E_BAD_STATE;
        }

        MXC_CAN_SetMode(can_idx, MXC_CAN_MODE_NORMAL);
        can->mode |= MXC_F_CAN_REVA_MODE_SLP; // Enable CAN sleep mode
        while (!(can->mode & MXC_F_CAN_REVA_MODE_SLP)) {}

        // wait for any pending transactions to finish
        MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_INACTIVE);
    } else if (pwr == MXC_CAN_PWR_CTRL_FULL) {
        can->mode &= ~MXC_F_CAN_REVA_MODE_SLP; // Disable CAN sleep mode
        MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_ACTIVE);
    }

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_EnableInt(mxc_can_reva_regs_t *can, uint8_t en, uint8_t ext_en)
{
    // Enable interrupts in INTFL register
    can->inten |= en;
    // Enable interrupts in extended INTFL register
    can->einten |= (ext_en & (MXC_F_CAN_REVA_EINTEN_RX_TO | MXC_F_CAN_REVA_EINTEN_RX_THD));

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_DisableInt(mxc_can_reva_regs_t *can, uint8_t dis, uint8_t ext_dis)
{
    // Disable interrupts in INTFL register
    can->inten &= ~dis;
    // Disable interrupts in extended INTFL register
    can->einten &= ~(ext_dis & (MXC_F_CAN_REVA_EINTEN_RX_TO | MXC_F_CAN_REVA_EINTEN_RX_THD));

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_GetFlags(mxc_can_reva_regs_t *can, uint8_t *flags, uint8_t *ext_flags)
{
    *flags = can->intfl; // Get flags in INTFL register
    *ext_flags = can->eintfl; // Get flags in extended INTFL register

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_ClearFlags(mxc_can_reva_regs_t *can, uint8_t flags, uint8_t ext_flags)
{
    // Clear flags in INTFL register
    can->intfl = flags;
    // Clear flags in Extended INTFL register
    can->eintfl = (ext_flags & (MXC_F_CAN_REVA_EINTFL_RX_TO | MXC_F_CAN_REVA_EINTFL_RX_THD));

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_GetBitRate(mxc_can_reva_regs_t *can, mxc_can_bitrate_sel_t sel, int can_clk)
{
    int num_tq = 1;
    int prescaler;

    // Get prescaler and total number of time quata
    switch (sel) {
    case MXC_CAN_BITRATE_SEL_NOMINAL:
        prescaler = ((can->nbt & MXC_F_CAN_REVA_NBT_NBRP) >> MXC_F_CAN_REVA_NBT_NBRP_POS) + 1;
        num_tq += ((can->nbt & MXC_F_CAN_REVA_NBT_NSEG1) >> MXC_F_CAN_REVA_NBT_NSEG1_POS) + 1;
        num_tq += ((can->nbt & MXC_F_CAN_REVA_NBT_NSEG2) >> MXC_F_CAN_REVA_NBT_NSEG2_POS) + 1;
        break;
    case MXC_CAN_BITRATE_SEL_FD_DATA:
        prescaler =
            ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DBRP) >> MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS) +
            1;
        num_tq +=
            ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DSEG1) >> MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS) +
            1;
        num_tq +=
            ((can->dbt_sspp & MXC_F_CAN_REVA_DBT_SSPP_DSEG2) >> MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS) +
            1;
        break;
    default:
        return E_BAD_PARAM;
    }

    return (can_clk / prescaler) /
           num_tq; // bitrate = (src_clk_freq / prescaler) / total_num_time_quanta
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_SetBitRate(mxc_can_reva_regs_t *can, int can_clk, mxc_can_bitrate_sel_t sel,
                            uint32_t bitrate, uint8_t seg1, uint8_t seg2, uint8_t sjw)
{
    uint32_t num_tq = 1;
    uint32_t prescaler = 1;
    int rv = E_NO_ERROR;

    can->mode |= MXC_F_CAN_REVA_MODE_RST;
    switch (sel) {
    case MXC_CAN_BITRATE_SEL_NOMINAL:
        // Check that desired time quatum in each segment doesn't exceed the maximum
        if (seg1 > MXC_CAN_NOMINAL_MAX_SEG1TQ || seg2 > MXC_CAN_NOMINAL_MAX_SEG2TQ ||
            sjw > MXC_CAN_NOMINAL_MAX_SJWTQ) {
            rv = E_BAD_PARAM;
            break;
        }

        num_tq += seg1 + seg2;
        prescaler = can_clk / (bitrate * num_tq);

        // Check that prescaler doesn't exceed maximum value
        if (prescaler > MXC_CAN_NOMINAL_MAX_PRESCALER) {
            rv = E_INVALID;
            break;
        }

        // Set prescaler and number of time quata in each segment
        MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NBRP,
                     ((prescaler - 1) << MXC_F_CAN_REVA_NBT_NBRP_POS));
        MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSEG1,
                     ((seg1 - 1) << MXC_F_CAN_REVA_NBT_NSEG1_POS));
        MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSEG2,
                     ((seg2 - 1) << MXC_F_CAN_REVA_NBT_NSEG2_POS));
        MXC_SETFIELD(can->nbt, MXC_F_CAN_REVA_NBT_NSJW, ((sjw - 1) << MXC_F_CAN_REVA_NBT_NSJW_POS));
        break;
    case MXC_CAN_BITRATE_SEL_FD_DATA:
        if (seg1 > MXC_CAN_FD_DATA_MAX_SEG1TQ || seg2 > MXC_CAN_FD_DATA_MAX_SEG2TQ ||
            sjw > MXC_CAN_FD_DATA_MAX_SJWTQ) {
            rv = E_BAD_PARAM;
            break;
        }

        num_tq += seg1 + seg2;
        prescaler = can_clk / (bitrate * num_tq);
        if (prescaler > MXC_CAN_FD_DATA_MAX_PRESCALER) {
            rv = E_INVALID;
            break;
        }

        MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DBRP,
                     ((prescaler - 1) << MXC_F_CAN_REVA_DBT_SSPP_DBRP_POS));
        MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSEG1,
                     ((seg1 - 1) << MXC_F_CAN_REVA_DBT_SSPP_DSEG1_POS));
        MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSEG2,
                     ((seg2 - 1) << MXC_F_CAN_REVA_DBT_SSPP_DSEG2_POS));
        MXC_SETFIELD(can->dbt_sspp, MXC_F_CAN_REVA_DBT_SSPP_DSJW,
                     ((sjw - 1) << MXC_F_CAN_REVA_DBT_SSPP_DSJW_POS));
        break;
    default:
        rv = E_BAD_PARAM;
    }
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return rv;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_SetMode(mxc_can_reva_regs_t *can, mxc_can_mode_t mode)
{
    // Ensure valid mode
    if (mode > MXC_CAN_MODE_LOOPBACK_W_TXD) {
        return E_BAD_PARAM;
    }

    can->mode |= MXC_F_CAN_REVA_MODE_RST;

    // Clear any existing operating modes
    can->fdctrl &= ~MXC_F_CAN_REVA_FDCTRL_REOM;
    can->mode &= ~MXC_F_CAN_REVA_MODE_LOM;
    can->test &= ~(MXC_F_CAN_REVA_TEST_LBEN | MXC_F_CAN_REVA_TEST_TXC);

    switch (mode) {
    case MXC_CAN_MODE_INITIALIZATION: // Reset mode
        return E_NO_ERROR;
    case MXC_CAN_MODE_NORMAL: // Normal mode
        break;
    case MXC_CAN_MODE_RESTRICTED: // Restriced mode
        can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_REOM;
        break;
    case MXC_CAN_MODE_MONITOR: // Listen-only mode
        can->mode |= MXC_F_CAN_REVA_MODE_LOM;
        break;
    case MXC_CAN_MODE_LOOPBACK: // Internal loopback mode
        can->test |= MXC_F_CAN_REVA_TEST_LBEN;
        break;
    case MXC_CAN_MODE_LOOPBACK_W_TXD: // Internal loopback mode with TX pin disconnected
        can->test |= MXC_F_CAN_REVA_TEST_LBEN | MXC_F_CAN_REVA_TEST_TXC;
        break;
    default:
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        return E_BAD_PARAM;
    }

    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
mxc_can_obj_capabilities_t MXC_CAN_RevA_ObjectGetCapabilities(mxc_can_reva_regs_t *can)
{
    mxc_can_obj_capabilities_t obj_cap;
    obj_cap.tx = 1; // Transmitting messages supported
    obj_cap.rx = 1; // Receiveiving messages supported
    obj_cap.rx_rtr_tx_data = 0; // Auto TX on RTR reception not supported
    obj_cap.tx_rtr_rx_data = 1; // Auto RX on RTR transmit supported
    obj_cap.multiple_filters = 1; // Multiple filters supported
    obj_cap.exact_filtering = 1; // Exact filtering supported
    obj_cap.mask_filtering = 1; // Mask filtering supported
    obj_cap.range_filtering = 0; // Range filtering supported
    obj_cap.message_depth = 1; // MXC_CAN TX FIFO depth is 1
    obj_cap.reserved = 0;
    return obj_cap;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_ObjectSetFilter(mxc_can_reva_regs_t *can, mxc_can_filt_cfg_t cfg, uint32_t id,
                                 uint32_t arg)
{
    uint32_t op_type = (cfg & MXC_CAN_FILT_OP_TYPE_MASK);
    uint32_t filt_sel = (cfg & MXC_CAN_FILT_SEL_MASK);
    uint32_t dual_filt_sel;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (filt_sel ==
        MXC_CAN_FILT_CFG_DUAL_GEN) { // If using middleware dual filter select, figure out which filter to set
        if (op_type == MXC_CAN_FILT_CFG_EXACT_ADD || op_type == MXC_CAN_FILT_CFG_MASK_ADD) {
            filt_sel = ((filt_in_use[can_idx] + 1) * 2) << MXC_CAN_FILT_SEL_SHIFT;
        } else if (op_type == MXC_CAN_FILT_CFG_EXACT_DEL || op_type == MXC_CAN_FILT_CFG_MASK_DEL) {
            filt_sel = (filt_in_use[can_idx] * 2) << MXC_CAN_FILT_SEL_SHIFT;
        }
    }

    can->mode |= MXC_F_CAN_REVA_MODE_RST;

    if (op_type == MXC_CAN_FILT_CFG_EXACT_ADD || op_type == MXC_CAN_FILT_CFG_MASK_ADD) {
        // Create filter selected
        if (filt_in_use[can_idx] >= MXC_CAN_FILT_PER_OBJ) {
            // Filter not available --> return error
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_NONE_AVAIL;
        }

        if (op_type == MXC_CAN_FILT_CFG_EXACT_ADD) {
            arg = 0xFFFFFFFF;
        }

        dual_filt_sel = 1;
        switch (filt_sel) {
        case MXC_CAN_FILT_CFG_DUAL1_STD_ID: // Set dual filter with standard, 11-bit ID length
            dual_filt_sel = 0; //Fall through
        case MXC_CAN_FILT_CFG_DUAL2_STD_ID:
            can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
            id = DUAL_FILT_STD_ID(id);
            can->acr16[dual_filt_sel] = HALFWORD_BYTESWAP(id);
            arg = DUAL_FILT_STD_ID((~arg)) | (1 << DUAL_FILT_STD_ID_RTR_SHIFT);
            can->amr16[dual_filt_sel] = HALFWORD_BYTESWAP(arg);
            can->amr8[1] |= 0x0F; // Set data byte filtering to don't care
            can->amr8[3] |= 0x0F;
            break;

        case MXC_CAN_FILT_CFG_DUAL1_EXT_ID: // Set dual filter with extended, 29-bit ID
            dual_filt_sel = 0; //Fall through
        case MXC_CAN_FILT_CFG_DUAL2_EXT_ID:
            can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
            id = DUAL_FILT_EXT_ID(id);
            can->acr16[dual_filt_sel] = HALFWORD_BYTESWAP(id);
            arg = DUAL_FILT_EXT_ID((~arg));
            can->amr16[dual_filt_sel] = HALFWORD_BYTESWAP(arg);
            break;

        case MXC_CAN_FILT_CFG_SINGLE_STD_ID: // Set single filter wit standard, 11-bit ID
            can->mode |= MXC_F_CAN_REVA_MODE_AFM;
            id = SINGLE_FILT_STD_ID(id);
            can->acr16[0] = HALFWORD_BYTESWAP(id);
            arg = SINGLE_FILT_STD_ID((~arg)) | (1 << SINGLE_FILT_STD_ID_RTR_SHIFT);
            can->amr16[0] = HALFWORD_BYTESWAP(arg);
            can->acr16[1] = 0xFFFF; // Set data byte filtering to don't care.
            can->amr16[1] = 0xFFFF;
            break;

        case MXC_CAN_FILT_CFG_SINGLE_EXT_ID: // Set single filter for extended, 29-bit ID
            can->mode |= MXC_F_CAN_REVA_MODE_AFM;
            id = SINGLE_FILT_EXT_ID(id);
            can->acr32 = WORD_BYTESWAP(id);
            arg = SINGLE_FILT_EXT_ID(~arg) | (1 << SINGLE_FILT_EXT_ID_RTR_SHIFT);
            can->amr32 = WORD_BYTESWAP(arg);
            break;

        default:
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_BAD_PARAM;
        }

        filt_in_use[can_idx]++;
    } else if (op_type == MXC_CAN_FILT_CFG_EXACT_DEL || op_type == MXC_CAN_FILT_CFG_MASK_DEL) {
        // Deleting filter
        if (filt_in_use[can_idx] == 0) {
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_NONE_AVAIL;
        }

        switch (filt_sel) {
        case MXC_CAN_FILT_CFG_DUAL1_STD_ID: // Clear dual filter 1
        case MXC_CAN_FILT_CFG_DUAL1_EXT_ID:
            can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
            can->acr16[0] = 0;
            can->amr16[0] = 0;
            break;
        case MXC_CAN_FILT_CFG_DUAL2_STD_ID: // Clear dual filter 2
        case MXC_CAN_FILT_CFG_DUAL2_EXT_ID:
            can->mode &= ~MXC_F_CAN_REVA_MODE_AFM;
            can->acr16[1] = 0;
            can->amr16[1] = 0;
            break;
        case MXC_CAN_FILT_CFG_SINGLE_STD_ID: // Clear single filter
        case MXC_CAN_FILT_CFG_SINGLE_EXT_ID:
            can->mode |= MXC_F_CAN_REVA_MODE_AFM;
            can->acr32 = 0;
            can->amr32 = 0;
            break;
        default:
            can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
            return E_BAD_PARAM;
        }

        filt_in_use[can_idx]--;
    } else {
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        return E_BAD_PARAM;
    }
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_ObjectConfigure(mxc_can_reva_regs_t *can, mxc_can_obj_cfg_t cfg)
{
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    switch (cfg) {
    case MXC_CAN_OBJ_CFG_INACTIVE: // Configure object as inactive
        MXC_CAN_PowerControl(can_idx, MXC_CAN_PWR_CTRL_SLEEP);
        break;
    case MXC_CAN_OBJ_CFG_TXRX: // Configure object to transmit and/or receive data
    case MXC_CAN_OBJ_CFG_RSV:
    case MXC_CAN_OBJ_CFG_TX_RTR_RX_DATA: // Configure object to receive message after sending RTR frames (RX always enabled when CAN is on, as long as MSG ID is accepted)
        MXC_CAN_PowerControl(can_idx, MXC_CAN_PWR_CTRL_FULL);
        can->mode |= MXC_F_CAN_REVA_MODE_RST;
        can->mode |= MXC_F_CAN_REVA_MODE_DMA;
        can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_EXTBT;
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
        break;
    case MXC_CAN_OBJ_CFG_RX_RTR_TX_DATA: // Configure object to auto TX on RTR frames not allowed
    default:
        return E_BAD_PARAM;
    }
    obj_state[can_idx] = cfg;
    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_WriteTXFIFO(mxc_can_reva_regs_t *can, mxc_can_msg_info_t *info,
                             const uint8_t *data, uint8_t size)
{
    uint8_t txbuf_idx = 0;
    uint32_t txbuf_data = 0;
    int num_bytes = getNumBytes(info->dlc, info->fdf, info->rtr);

    if (info == NULL) { // Check if desired format is valid
        return E_NULL_PTR;
    } else if (info->rtr && (info->fdf || info->brs || info->esi)) {
        return E_INVALID;
    } else if (!info->fdf && (info->brs || info->esi)) {
        return E_INVALID;
    } else if (can->txscnt < (num_bytes + 5)) {
        return E_BAD_STATE;
    }

    txbuf_data = MXC_CAN_BUF_CFG_RTR(info->rtr) | MXC_CAN_BUF_CFG_FDF(info->fdf) |
                 MXC_CAN_BUF_CFG_BRS(info->brs) | MXC_CAN_BUF_CFG_DLC(info->dlc);
    if (info->msg_id & MXC_CAN_MSG_INFO_IDE_BIT) { // Configure message for extended ID
        txbuf_data |= MXC_CAN_BUF_CFG_IDE;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX1(info->msg_id) << 8;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX2(info->msg_id) << 16;
        txbuf_data |= MXC_CAN_BUF_CFG_EXT_ID_TX3(info->msg_id) << 24;
        can->txfifo32 = txbuf_data;

        txbuf_data = MXC_CAN_BUF_CFG_EXT_ID_TX4(info->msg_id) |
                     MXC_CAN_BUF_CFG_EXT_ID_ESI(info->esi);
        txbuf_idx = 1;
    } else { // Configure message for standard ID
        txbuf_data |= MXC_CAN_BUF_CFG_STD_ID_TX1(info->msg_id) << 8;
        txbuf_data |=
            (MXC_CAN_BUF_CFG_STD_ID_TX2(info->msg_id) | MXC_CAN_BUF_CFG_STD_ID_ESI(info->esi))
            << 16;
        txbuf_idx = 3;
    }

    if (info->rtr || data == NULL) { // No data bytes in RTR frames
        can->txfifo32 = txbuf_data;

        return E_NO_ERROR;
    }

    for (int i = 0; i < num_bytes; i++) { // Add data bytes to FIFO
        if (txbuf_idx == 0) {
            can->txfifo32 = txbuf_data;
            txbuf_data = 0;
        }

        if (i < size) {
            // We have exceeded the size of the data buffer passed in, write zeros to remaining bytes
            txbuf_data |= data[i] << (txbuf_idx << 3);
        }

        txbuf_idx = (txbuf_idx + 1) & 0x3;
    }
    can->txfifo32 = txbuf_data;

    return num_bytes;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_ReadRXFIFO(mxc_can_reva_regs_t *can, mxc_can_msg_info_t *info, uint8_t *data,
                            uint8_t size, bool dma)
{
    uint8_t rxfifo_idx = 0, rxbuf_parse = 0, dma_buf_idx = 0;
    uint32_t rx_data = 0;
    uint32_t *dma_buf = dma_rx0;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (info == NULL || data == NULL) {
        return E_NULL_PTR;
    } else if (!dma && can->rxdcnt == 0) {
        // RX FIFO empty for non-DMA transaction
        return E_BAD_STATE;
    }

    // Select appropriate DMA buffer to read from if DMA read
    if (dma && can_idx == 0) {
        dma_buf = dma_rx0;
    } else if (dma && can_idx == 1) {
        dma_buf = dma_rx1;
    }

    if (dma) { // Get next word in RX FIFO
        rx_data = dma_buf[dma_buf_idx++];
    } else {
        rx_data = can->rxfifo32;
    }
    rxbuf_parse = rx_data & 0xFF;
    info->dlc = rxbuf_parse & MXC_CAN_BUF_CFG_DLC(0xF);
    info->brs = !!(rxbuf_parse & MXC_CAN_BUF_CFG_BRS(1));
    info->fdf = !!(rxbuf_parse & MXC_CAN_BUF_CFG_FDF(1));

    if (rx_data & MXC_CAN_BUF_CFG_IDE) {
        // Parse Message Header for extended ID format
        info->msg_id = MXC_CAN_MSG_INFO_IDE_BIT;

        rxbuf_parse = (rx_data & 0xFF00) >> 8;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX1(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF0000) >> 16;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX2(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF000000) >> 24;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX3(rxbuf_parse);

        if (dma) {
            rx_data = dma_buf[dma_buf_idx++];
        } else {
            rx_data = can->rxfifo32;
        }
        rxbuf_parse = rx_data & 0xFF;
        info->msg_id |= MXC_CAN_BUF_CFG_EXT_ID_RX4(rxbuf_parse);
        info->rtr = !!(rxbuf_parse & MXC_CAN_BUF_CFG_EXT_ID_RTR(1));
        info->esi = !!(rxbuf_parse & MXC_CAN_BUF_CFG_EXT_ID_ESI(1));
        rxfifo_idx = 1;
    } else {
        // Parse Header for Standard ID Format
        info->rtr = !!(rxbuf_parse & MXC_CAN_BUF_CFG_STD_ID_RTR(1));

        rxbuf_parse = (rx_data & 0xFF00) >> 8;
        info->msg_id = MXC_CAN_BUF_CFG_STD_ID_RX1(rxbuf_parse);

        rxbuf_parse = (rx_data & 0xFF0000) >> 16;
        info->msg_id |= MXC_CAN_BUF_CFG_STD_ID_RX2(rxbuf_parse);
        info->esi = !!(rxbuf_parse & MXC_CAN_BUF_CFG_STD_ID_ESI(1));
        rxfifo_idx = 3;
    }

    if (info->rtr) { // No data bytes in RTR frames
        return E_NO_ERROR;
    }

    // Get number of data bytes to read
    int num_bytes = getNumBytes(info->dlc, info->fdf, info->rtr);
    int rv = num_bytes;

    // Read data bytes
    for (int i = 0; i < num_bytes; i++) {
        if (rxfifo_idx == 0) {
            if (dma) {
                rx_data = dma_buf[dma_buf_idx++];
            } else {
                rx_data = can->rxfifo32;
            }
        }

        if (i < size) {
            // Mask off next byte in word
            data[i] = (rx_data & (0xFF << (rxfifo_idx << 3))) >> (rxfifo_idx << 3);
        } else {
            rv = size;
        }

        rxfifo_idx = (rxfifo_idx + 1) & 0x3;
    }

    return rv;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageSend(mxc_can_reva_regs_t *can, mxc_can_req_t *req)
{
    int err;
    uint8_t flags, ext_flags;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (req == NULL) {
        return E_NULL_PTR;
    } else if (MXC_GetLock(&tx_lock[can_idx], 1)) {
        // TX lock already taken
        return E_BAD_STATE;
    }

    // Format and write message to FIFO
    if ((err = MXC_CAN_WriteTXFIFO(can_idx, req->msg_info, req->data, req->data_sz)) < E_NO_ERROR) {
        return err;
    }

    MXC_CAN_ClearFlags(
        can_idx,
        (MXC_F_CAN_REVA_INTFL_ERWARN | MXC_F_CAN_REVA_INTFL_ERPSV | MXC_F_CAN_REVA_INTFL_TX), 0);
    can->cmd = MXC_F_CAN_REVA_CMD_TXREQ; // Send TX request

    // Wait for TX to complete
    do {
        // Check for errors
        if ((err = MXC_CAN_RevA_Handler(can, &flags, &ext_flags)) != E_NO_ERROR) {
            MXC_FreeLock(&tx_lock[can_idx]);
            return err;
        }
    } while (!(flags & MXC_F_CAN_REVA_INTFL_TX));

    if (flags & MXC_F_CAN_REVA_INTFL_TX) { // If TX complete call callback function
        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_TX_COMPLETE);
    }

    MXC_FreeLock(&tx_lock[can_idx]); // Free TX lock

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageSendAsync(mxc_can_reva_regs_t *can, mxc_can_req_t *req)
{
    int err;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (req == NULL) {
        return E_NULL_PTR;
    } else if (MXC_GetLock(&tx_lock[can_idx], 1)) {
        // TX lock already acquired
        return E_BAD_STATE;
    }

    // Format and write message to TX FIFO
    if ((err = MXC_CAN_WriteTXFIFO(can_idx, req->msg_info, req->data, req->data_sz)) < E_NO_ERROR) {
        MXC_FreeLock(&tx_lock[can_idx]);
        return err;
    }

    MXC_CAN_ClearFlags(
        can_idx,
        (MXC_F_CAN_REVA_INTFL_ERWARN | MXC_F_CAN_REVA_INTFL_ERPSV | MXC_F_CAN_REVA_INTFL_TX), 0);
    MXC_CAN_EnableInt(
        can_idx,
        (MXC_F_CAN_REVA_INTEN_ERWARN | MXC_F_CAN_REVA_INTEN_ERPSV | MXC_F_CAN_REVA_INTEN_TX), 0);

    can->cmd = MXC_F_CAN_REVA_CMD_TXREQ; // Assert TX request

    return err;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageSendDMA(mxc_can_reva_regs_t *can, mxc_can_req_t *req)
{
    // No performance improvements using DMA transmit since CAN FIFO can only hold one message to transmit at a time
    return MXC_CAN_RevA_MessageSendAsync(can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageRead(mxc_can_reva_regs_t *can, mxc_can_req_t *req)
{
    int err = 0;
    uint8_t flags = 0, ext_flags = 0;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    // Check for bad parameters
    if (req == NULL) {
        return E_NULL_PTR;
    } else if (MXC_GetLock(&rx_lock[can_idx], 1)) {
        // RX lock already taken
        return E_BUSY;
    }

    // Wait to receive message
    do {
        // Check for errors
        if ((err = MXC_CAN_RevA_Handler(can, &flags, &ext_flags)) != E_NO_ERROR) {
            break;
        }
    } while (!(flags & (MXC_F_CAN_REVA_INTFL_RX | MXC_F_CAN_REVA_INTFL_DOR)));

    if (flags & MXC_F_CAN_REVA_INTFL_DOR) {
        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_RX_OVERRUN);
    }

    if (flags & MXC_F_CAN_REVA_INTFL_RX) {
        // Read data from FIFO
        if ((err = MXC_CAN_ReadRXFIFO(can_idx, req->msg_info, req->data, req->data_sz)) <
            E_NO_ERROR) {
            MXC_FreeLock(&rx_lock[can_idx]);
            return err;
        }
        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_RX);
    }

    MXC_FreeLock(&rx_lock[can_idx]); // Free RX lock

    return err;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageReadAsync(mxc_can_reva_regs_t *can, mxc_can_req_t *req)
{
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (req == NULL) {
        return E_NULL_PTR;
    } else if (MXC_GetLock(&rx_lock[can_idx], 1)) {
        // RX lock already taken
        return E_BUSY;
    }

    rx_req[can_idx] = req; // Store message read request

    MXC_CAN_ClearFlags(can_idx,
                       (MXC_F_CAN_REVA_INTFL_ERWARN | MXC_F_CAN_REVA_INTFL_ERPSV |
                        MXC_F_CAN_REVA_INTFL_RX | MXC_F_CAN_REVA_INTFL_DOR),
                       0);
    MXC_CAN_EnableInt(can_idx,
                      (MXC_F_CAN_REVA_INTEN_ERWARN | MXC_F_CAN_REVA_INTEN_ERPSV |
                       MXC_F_CAN_REVA_INTEN_RX | MXC_F_CAN_REVA_INTEN_DOR),
                      0);

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_MessageReadDMA(mxc_can_reva_regs_t *can, mxc_can_req_t *req,
                                mxc_dma_reqsel_t reqsel, void (*dma_cb)(int, int))
{
    int ch;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    if (req == NULL) {
        return E_NULL_PTR;
    } else if (MXC_GetLock(&rx_lock[can_idx], 1)) {
        // RX lock already taken
        return E_BUSY;
    } else if (MXC_GetLock(&rx_dma_lock[can_idx], 1)) {
        // DMA lock already taken
        MXC_FreeLock(&rx_lock[can_idx]);
        return E_BUSY;
    }

    rx_req[can_idx] = req; // Save RX request

    if ((ch = MXC_DMA_AcquireChannel()) < E_NO_ERROR) { // Acquire DMA Channel
        return ch;
    }

    if (dma_cb != NULL) { // Set DMA callback
        MXC_DMA_SetCallback(ch, dma_cb);
    }

    if (can_idx == 0) { // Configure DMA Channel
        srcdst.dest = dma_rx0;
    } else if (can_idx == 1) {
        srcdst.dest = dma_rx1;
    }

    config.reqsel = reqsel;
    config.ch = ch;
    config.srcwd = MXC_DMA_WIDTH_WORD;
    config.dstwd = MXC_DMA_WIDTH_WORD;
    config.srcinc_en = 0;
    config.dstinc_en = 1;

    srcdst.ch = ch;
    srcdst.len = getNumBytes((req->msg_info)->dlc, (req->msg_info)->fdf, (req->msg_info)->rtr) +
                 MXC_CAN_DMA_LEN((req->msg_info)->msg_id);
    if (srcdst.len & 0x3) {
        srcdst.len += (4 - (srcdst.len & 0x3));
    }
    MXC_DMA_ConfigChannel(config, srcdst);

    mxc_dma_adv_config_t advConfig;
    advConfig.ch = ch;
    advConfig.prio = MXC_DMA_PRIO_HIGH;
    advConfig.reqwait_en = 0;
    advConfig.tosel = MXC_DMA_TIMEOUT_4_CLK; // 0
    advConfig.pssel = MXC_DMA_PRESCALE_DISABLE; // 0
    advConfig.burst_size = 4;
    MXC_DMA_AdvConfigChannel(advConfig);

    // Set RX DMA trigger to one word
    MXC_SETFIELD(can->mode, MXC_F_CAN_REVA_MODE_RXTRIG, MXC_S_CAN_REVA_MODE_RXTRIG_1W);

    MXC_DMA_EnableInt(ch); // Enable DMA
    MXC_DMA_SetChannelInterruptEn(ch, 0, 1);
    MXC_DMA_Start(ch);

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_Handler(mxc_can_reva_regs_t *can, uint8_t *intfl, uint8_t *eintfl)
{
    uint8_t flg, ext_flg;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    MXC_CAN_GetFlags(can_idx, &flg, &ext_flg); // Get and clear flags
    MXC_CAN_ClearFlags(can_idx, flg, ext_flg);

    if (intfl != NULL && eintfl != NULL) { // Return flags if able to
        *intfl = flg;
        *eintfl = ext_flg;
    }

    if (flg & MXC_F_CAN_REVA_INTFL_ERWARN) {
        if (can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
            // Bus entered bus off state
            MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_BUS_OFF);

            if (tx_lock[can_idx]) {
                MXC_FreeLock(&tx_lock[can_idx]);
            }
            return E_COMM_ERR;
        } else if (can->stat & MXC_F_CAN_REVA_STAT_ERR) {
            // Bus entered err warning state (> 127 errs)
            MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_WARNING);
        }
    }

    if (flg & MXC_F_CAN_REVA_INTFL_ERPSV) {
        if (can->txerr > MXC_CAN_ERRPSV_THRESH || can->rxerr > MXC_CAN_ERRPSV_THRESH) {
            // Bus entered error passive state
            MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_PASSIVE);
        } else {
            // Bus exited from error passive state
            MXC_CAN_SignalUnitEvent(can_idx, MXC_CAN_UNIT_EVT_ACTIVE);
        }
    }

    if ((flg & MXC_F_CAN_REVA_INTFL_TX) && (can->inten & MXC_F_CAN_REVA_INTEN_TX)) {
        // TX completed
        MXC_FreeLock(&tx_lock[can_idx]);
        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_TX_COMPLETE);
    }

    if ((flg & MXC_F_CAN_REVA_INTFL_RX) &&
        ((can->inten & MXC_F_CAN_REVA_INTEN_RX) || rx_dma_lock[can_idx])) {
        // RX completed
        mxc_can_req_t *msg_req = rx_req[can_idx];

        if (rx_dma_lock[can_idx]) {
            // Read from DMA RX Buffers for transactions
            MXC_CAN_RevA_ReadRXFIFO(can, msg_req->msg_info, msg_req->data, msg_req->data_sz, true);
            MXC_FreeLock(&rx_dma_lock[can_idx]);
        } else {
            // Read CAN message
            MXC_CAN_RevA_ReadRXFIFO(can, msg_req->msg_info, msg_req->data, msg_req->data_sz, false);
        }
        MXC_FreeLock(&rx_lock[can_idx]);

        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_RX);
    }

    // Data overrun
    if (flg & MXC_F_CAN_REVA_INTFL_DOR) {
        MXC_CAN_SignalObjectEvent(can_idx, MXC_CAN_OBJ_EVT_RX_OVERRUN);
    }

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_Control(mxc_can_reva_regs_t *can, mxc_can_ctrl_t ctrl, uint32_t ctrl_arg)
{
    if (ctrl == MXC_CAN_CTRL_SET_FD_MODE) { // Set FD Mode
        can->mode |= MXC_F_CAN_REVA_MODE_RST;
        can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_FDEN | MXC_F_CAN_REVA_FDCTRL_BRSEN |
                       MXC_F_CAN_REVA_FDCTRL_ISO;
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
    } else if (ctrl == MXC_CAN_CTRL_ABORT_TX) { // Abort transmission
        can->cmd = MXC_F_CAN_REVA_CMD_ABORT;
    } else if (ctrl == MXC_CAN_CTRL_RETRANSMISSION) { // Enable/disable auto retransmission
        can->mode |= MXC_F_CAN_REVA_MODE_RST;
        if (ctrl_arg) { // Enable
            can->fdctrl &= ~MXC_F_CAN_REVA_FDCTRL_DAR;
        } else { // Disable
            can->fdctrl |= MXC_F_CAN_REVA_FDCTRL_DAR;
        }
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
    } else if (ctrl == MXC_CAN_CTRL_TRANSCEIVER_DLY) { // Set transceiver delay
        can->mode |= MXC_F_CAN_REVA_MODE_RST;
        can->txdecmp = MXC_F_CAN_REVA_TXDECMP_TDCEN | (ctrl_arg & MXC_F_CAN_REVA_TXDECMP_TDCO);
        can->mode &= ~MXC_F_CAN_REVA_MODE_RST;
    } else {
        return E_BAD_PARAM;
    }

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_RevA_SetWakeupTimer(mxc_can_reva_regs_t *can, uint8_t prescaler, uint16_t wup_filter_tm,
                                uint32_t wup_expire_tm)
{
    can->mode |= MXC_F_CAN_REVA_MODE_RST;

    // Set wake up values: clock divider, filter time, expire time
    MXC_SETFIELD(can->wupclkdiv, MXC_F_CAN_REVA_WUPCLKDIV_WUPDIV, prescaler);
    MXC_SETFIELD(can->wupft, MXC_F_CAN_REVA_WUPFT_WUPFT, wup_filter_tm);
    MXC_SETFIELD(can->wupet, MXC_F_CAN_REVA_WUPET_WUPET, wup_expire_tm);
    can->mode &= ~MXC_F_CAN_REVA_MODE_RST;

    return E_NO_ERROR;
}

/**********************************************************************************************************************************************************************/
mxc_can_stat_t MXC_CAN_RevA_GetStatus(mxc_can_reva_regs_t *can)
{
    mxc_can_stat_t stat;
    int can_idx;

    can_idx = MXC_CAN_GET_IDX((mxc_can_regs_t *)can);
    MXC_ASSERT(can_idx >= 0);

    // Get unit state
    if (obj_state[can_idx] == MXC_CAN_OBJ_CFG_INACTIVE) {
        stat.unit_state = MXC_CAN_UNIT_STATE_INACTIVE;
    } else if (can->stat & MXC_F_CAN_REVA_STAT_BUS_OFF) {
        stat.unit_state = MXC_CAN_UNIT_STATE_BUS_OFF;
    } else if ((can->mode & MXC_F_CAN_REVA_MODE_LOM) ||
               (can->fdctrl & MXC_F_CAN_REVA_FDCTRL_REOM)) {
        stat.unit_state = MXC_CAN_UNIT_STATE_PASSIVE;
    } else {
        stat.unit_state = MXC_CAN_UNIT_STATE_ACTIVE;
    }

    stat.tx_err_cnt = can->txerr; // Get TX error count
    stat.rx_err_cnt = can->rxerr; // Get RX error count

    stat.last_error_code = can->ecc; // Get last error code
    switch (stat.last_error_code & MXC_CAN_ECC_ERROR_CODE_MASK) {
    case MXC_F_CAN_REVA_ECC_BER:
        stat.last_error_code = MXC_CAN_LEC_BIT_ERR;
        break;
    case MXC_F_CAN_REVA_ECC_STFER:
        stat.last_error_code = MXC_CAN_LEC_STUFF_ERR;
        break;
    case MXC_F_CAN_REVA_ECC_CRCER:
        stat.last_error_code = MXC_CAN_LEC_CRC_ERR;
        break;
    case MXC_F_CAN_REVA_ECC_FRMER:
        stat.last_error_code = MXC_CAN_LEC_FORM_ERR;
        break;
    case MXC_F_CAN_REVA_ECC_ACKER:
        stat.last_error_code = MXC_CAN_LEC_ACK_ERR;
        break;
    default:
        stat.last_error_code = MXC_CAN_LEC_NO_ERR;
    }

    return stat;
}

/**********************************************************************************************************************************************************************/
void MXC_CAN_RevA_SignalUnitEvent(uint32_t can_idx, mxc_can_unit_evt_t event)
{
    mxc_can_object_event_cb_t evt_cb = unit_evt_cb[can_idx];
    if (evt_cb != NULL) {
        evt_cb(can_idx, event); // Call unit event callback
    }
}

/**********************************************************************************************************************************************************************/
void MXC_CAN_RevA_SignalObjectEvent(uint32_t can_idx, mxc_can_obj_evt_t event)
{
    mxc_can_object_event_cb_t evt_cb = obj_evt_cb[can_idx];
    if (evt_cb != NULL) {
        evt_cb(can_idx, event); // Call object event callback
    }
}
