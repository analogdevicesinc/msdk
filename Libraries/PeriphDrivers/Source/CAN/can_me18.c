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

#include "can.h"
#include "can_reva.h"

#define MXC_CAN_OBJCAPABILITIES_ERROR      \
    {                                      \
        -1, -1, -1, -1, -1, -1, -1, -1, -1 \
    }

static int8_t last_stat_update = -1;

/**********************************************************************************************************************************************************************/
mxc_can_drv_version_t MXC_CAN_GetVersion(void)
{
    return MXC_CAN_RevA_GetVersion();
}

/**********************************************************************************************************************************************************************/
mxc_can_capabilities_t MXC_CAN_GetCapabilities(void)
{
    mxc_can_capabilities_t ret = MXC_CAN_RevA_GetCapabilities();
    ret.fd_mode = 0;
    return ret;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_Init(uint32_t can_idx, mxc_can_obj_cfg_t cfg, mxc_can_unit_event_cb_t unit_cb,
                 mxc_can_object_event_cb_t obj_cb)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    MXC_CAN_RevA_Init((mxc_can_reva_regs_t *)can, unit_cb, obj_cb);

    return MXC_CAN_ObjectConfigure(can_idx, cfg);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_UnInit(uint32_t can_idx)
{
    switch (can_idx) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_CAN0);
#ifndef MSDK_NO_GPIO_CLK_INIT
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CAN0);
#endif // MSDK_NO_GPIO_CLK_INIT
        break;
    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_CAN1);
#ifndef MSDK_NO_GPIO_CLK_INIT
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CAN1);
#endif // MSDK_NO_GPIO_CLK_INIT
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_UnInit((mxc_can_reva_regs_t *)MXC_CAN_GET_CAN(can_idx));
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_PowerControl(uint32_t can_idx, mxc_can_pwr_ctrl_t pwr)
{
    mxc_sys_periph_clock_t periph_clk;
    if (can_idx == 0) {
        periph_clk = MXC_SYS_PERIPH_CLOCK_CAN0;
    } else if (can_idx == 1) {
        periph_clk = MXC_SYS_PERIPH_CLOCK_CAN1;
    } else {
        return E_BAD_PARAM;
    }

    switch (pwr) {
    case MXC_CAN_PWR_CTRL_OFF:
#ifndef MSDK_NO_GPIO_CLK_INIT
        MXC_SYS_ClockDisable(periph_clk);
#endif // MSDK_NO_GPIO_CLK_INIT
        return E_NO_ERROR;
    case MXC_CAN_PWR_CTRL_SLEEP: //Fall through
    case MXC_CAN_PWR_CTRL_FULL:
#ifndef MSDK_NO_GPIO_CLK_INIT
        MXC_SYS_ClockEnable(periph_clk);
#endif // MSDK_NO_GPIO_CLK_INIT
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_PowerControl((mxc_can_reva_regs_t *)MXC_CAN_GET_CAN(can_idx), pwr);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_EnableInt(uint32_t can_idx, uint8_t en, uint8_t ext_en)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_EnableInt((mxc_can_reva_regs_t *)can, en, ext_en);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_DisableInt(uint32_t can_idx, uint8_t dis, uint8_t ext_dis)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_DisableInt((mxc_can_reva_regs_t *)can, dis, ext_dis);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_GetFlags(uint32_t can_idx, uint8_t *flags, uint8_t *ext_flags)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_GetFlags((mxc_can_reva_regs_t *)can, flags, ext_flags);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_ClearFlags(uint32_t can_idx, uint8_t flags, uint8_t ext_flags)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_ClearFlags((mxc_can_reva_regs_t *)can, flags, ext_flags);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_GetClock(uint32_t can_idx)
{
    return PeripheralClock;
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_GetBitRate(uint32_t can_idx, mxc_can_bitrate_sel_t sel)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (sel == MXC_CAN_BITRATE_SEL_FD_DATA) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_GetBitRate((mxc_can_reva_regs_t *)can, sel, MXC_CAN_GetClock(can_idx));
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_SetBitRate(uint32_t can_idx, mxc_can_bitrate_sel_t sel, uint32_t bitrate,
                       uint32_t bit_segments)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (sel == MXC_CAN_BITRATE_SEL_FD_DATA) {
        return E_NOT_SUPPORTED;
    }

    uint8_t seg1 = (bit_segments & (0xFF << MXC_CAN_SEG1_SHIFT)) >> MXC_CAN_SEG1_SHIFT;
    uint8_t seg2 = (bit_segments & (0xFF << MXC_CAN_SEG2_SHIFT)) >> MXC_CAN_SEG2_SHIFT;
    uint8_t sjw = (bit_segments & (0xFF << MXC_CAN_SJW_SHIFT)) >> MXC_CAN_SJW_SHIFT;

    return MXC_CAN_RevA_SetBitRate((mxc_can_reva_regs_t *)can, MXC_CAN_GetClock(can_idx), sel,
                                   bitrate, seg1, seg2, sjw);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_SetMode(uint32_t can_idx, mxc_can_mode_t mode)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_SetMode((mxc_can_reva_regs_t *)can, mode);
}

/**********************************************************************************************************************************************************************/
mxc_can_obj_capabilities_t MXC_CAN_ObjectGetCapabilities(uint32_t can_idx)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return (mxc_can_obj_capabilities_t)MXC_CAN_OBJCAPABILITIES_ERROR;
    }

    return MXC_CAN_RevA_ObjectGetCapabilities((mxc_can_reva_regs_t *)can);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_ObjectSetFilter(uint32_t can_idx, mxc_can_filt_cfg_t cfg, uint32_t id, uint32_t arg)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_ObjectSetFilter((mxc_can_reva_regs_t *)can, cfg, id, arg);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_ObjectConfigure(uint32_t can_idx, mxc_can_obj_cfg_t cfg)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    switch (can_idx) {
    case 0:
        MXC_GPIO_Config(&gpio_cfg_can0);
        break;
    case 1:
        MXC_GPIO_Config(&gpio_cfg_can1);
        break;
    }
#endif // MSDK_NO_GPIO_CLK_INIT

    return MXC_CAN_RevA_ObjectConfigure((mxc_can_reva_regs_t *)can, cfg);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_WriteTXFIFO(uint32_t can_idx, mxc_can_msg_info_t *info, const uint8_t *data,
                        uint8_t size)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_WriteTXFIFO((mxc_can_reva_regs_t *)can, info, data, size);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_ReadRXFIFO(uint32_t can_idx, mxc_can_msg_info_t *info, uint8_t *data, uint8_t size)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_ReadRXFIFO((mxc_can_reva_regs_t *)can, info, data, size, false);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageSend(uint32_t can_idx, mxc_can_req_t *req)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (req->msg_info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_MessageSend((mxc_can_reva_regs_t *)can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageSendAsync(uint32_t can_idx, mxc_can_req_t *req)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (req->msg_info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_MessageSendAsync((mxc_can_reva_regs_t *)can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageSendDMA(uint32_t can_idx, mxc_can_req_t *req)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (req->msg_info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_MessageSendDMA((mxc_can_reva_regs_t *)can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageRead(uint32_t can_idx, mxc_can_req_t *req)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_MessageRead((mxc_can_reva_regs_t *)can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageReadAsync(uint32_t can_idx, mxc_can_req_t *req)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_MessageReadAsync((mxc_can_reva_regs_t *)can, req);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_MessageReadDMA(uint32_t can_idx, mxc_can_req_t *req, void (*dma_cb)(int, int))
{
    mxc_dma_reqsel_t reqsel;

    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (req->msg_info->fdf != 0) {
        return E_NOT_SUPPORTED;
    }

    // Set Appropriate DMA Request Select
    switch (can_idx) {
    case 0:
        reqsel = MXC_DMA_REQUEST_CAN0RX;
        break;
    case 1:
        reqsel = MXC_DMA_REQUEST_CAN1RX;
        break;
    default:
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_MessageReadDMA((mxc_can_reva_regs_t *)can, req, reqsel, dma_cb);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_Handler(uint32_t can_idx)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_Handler((mxc_can_reva_regs_t *)can, NULL, NULL);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_Control(uint32_t can_idx, mxc_can_ctrl_t ctrl, uint32_t ctrl_arg)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    } else if (ctrl == MXC_CAN_CTRL_SET_FD_MODE) {
        return E_NOT_SUPPORTED;
    }

    return MXC_CAN_RevA_Control((mxc_can_reva_regs_t *)can, ctrl, ctrl_arg);
}

/**********************************************************************************************************************************************************************/
int MXC_CAN_SetWakeupTimer(uint32_t can_idx, uint8_t prescaler, uint16_t wup_filter_tm,
                           uint32_t wup_expire_tm)
{
    mxc_can_regs_t *can = MXC_CAN_GET_CAN(can_idx);
    if (can == 0) {
        return E_BAD_PARAM;
    }

    return MXC_CAN_RevA_SetWakeupTimer((mxc_can_reva_regs_t *)can, prescaler, wup_filter_tm,
                                       wup_expire_tm);
}

/**********************************************************************************************************************************************************************/
mxc_can_stat_t MXC_CAN_GetStatus(uint32_t can_idx)
{
    mxc_can_stat_t obj_stat[MXC_CAN_INSTANCES];
    for (int i = 0; i < MXC_CAN_INSTANCES; i++) {
        obj_stat[i] = MXC_CAN_RevA_GetStatus((mxc_can_reva_regs_t *)MXC_CAN_GET_CAN(i));
        obj_stat[i].can_idx = i;
    }

    if (obj_stat[0].last_error_code && !obj_stat[1].last_error_code) {
        return obj_stat[0];
    } else if (!obj_stat[0].last_error_code && obj_stat[1].last_error_code) {
        return obj_stat[1];
    } else {
        last_stat_update = (last_stat_update + 1) % MXC_CAN_INSTANCES;
        return obj_stat[last_stat_update];
    }
}

/**********************************************************************************************************************************************************************/
void MXC_CAN_SignalUnitEvent(uint32_t can_idx, mxc_can_unit_evt_t event)
{
    if (can_idx >= MXC_CAN_INSTANCES) {
        return;
    } else if (event > MXC_CAN_UNIT_EVT_BUS_OFF) {
        return;
    }

    MXC_CAN_RevA_SignalUnitEvent(can_idx, event);
}

/**********************************************************************************************************************************************************************/
void MXC_CAN_SignalObjectEvent(uint32_t can_idx, mxc_can_obj_evt_t event)
{
    if (can_idx >= MXC_CAN_INSTANCES) {
        return;
    } else if (event > MXC_CAN_OBJ_EVT_RX_OVERRUN) {
        return;
    }

    MXC_CAN_RevA_SignalObjectEvent(can_idx, event);
}
