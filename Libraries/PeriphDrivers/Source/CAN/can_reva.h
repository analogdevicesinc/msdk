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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_CAN_CAN_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_CAN_CAN_REVA_H_

#include <stdio.h>
#include "can.h"
#include "can_reva_regs.h"
#include "dma.h"
#include "dma_reva_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"

/***** FUNCTIONS *****/
mxc_can_drv_version_t MXC_CAN_RevA_GetVersion(void);

mxc_can_capabilities_t MXC_CAN_RevA_GetCapabilities(void);

int MXC_CAN_RevA_Init(mxc_can_reva_regs_t *can, mxc_can_unit_event_cb_t unit_cb,
                      mxc_can_object_event_cb_t obj_cb);

int MXC_CAN_RevA_UnInit(mxc_can_reva_regs_t *can);

int MXC_CAN_RevA_PowerControl(mxc_can_reva_regs_t *can, mxc_can_pwr_ctrl_t pwr);

int MXC_CAN_RevA_EnableInt(mxc_can_reva_regs_t *can, uint8_t en, uint8_t ext_en);

int MXC_CAN_RevA_DisableInt(mxc_can_reva_regs_t *can, uint8_t dis, uint8_t ext_dis);

int MXC_CAN_RevA_GetFlags(mxc_can_reva_regs_t *can, uint8_t *flags, uint8_t *ext_flags);

int MXC_CAN_RevA_ClearFlags(mxc_can_reva_regs_t *can, uint8_t flags, uint8_t ext_flags);

int MXC_CAN_RevA_GetBitRate(mxc_can_reva_regs_t *can, mxc_can_bitrate_sel_t sel, int can_clk);

int MXC_CAN_RevA_SetBitRate(mxc_can_reva_regs_t *can, int can_clk, mxc_can_bitrate_sel_t sel,
                            uint32_t bitrate, uint8_t seg1, uint8_t seg2, uint8_t sjw);

int MXC_CAN_RevA_SetMode(mxc_can_reva_regs_t *can, mxc_can_mode_t mode);

mxc_can_obj_capabilities_t MXC_CAN_RevA_ObjectGetCapabilities(mxc_can_reva_regs_t *can);

int MXC_CAN_RevA_ObjectSetFilter(mxc_can_reva_regs_t *can, mxc_can_filt_cfg_t cfg, uint32_t id,
                                 uint32_t arg);

int MXC_CAN_RevA_ObjectConfigure(mxc_can_reva_regs_t *can, mxc_can_obj_cfg_t cfg);

int MXC_CAN_RevA_WriteTXFIFO(mxc_can_reva_regs_t *can, mxc_can_msg_info_t *info,
                             const uint8_t *data, uint8_t size);

int MXC_CAN_RevA_ReadRXFIFO(mxc_can_reva_regs_t *can, mxc_can_msg_info_t *info, uint8_t *data,
                            uint8_t size, bool dma);

int MXC_CAN_RevA_MessageSend(mxc_can_reva_regs_t *can, mxc_can_req_t *req);

int MXC_CAN_RevA_MessageSendAsync(mxc_can_reva_regs_t *can, mxc_can_req_t *req);

int MXC_CAN_RevA_MessageSendDMA(mxc_can_reva_regs_t *can, mxc_can_req_t *req);

int MXC_CAN_RevA_MessageRead(mxc_can_reva_regs_t *can, mxc_can_req_t *req);

int MXC_CAN_RevA_MessageReadAsync(mxc_can_reva_regs_t *can, mxc_can_req_t *req);

int MXC_CAN_RevA_MessageReadDMA(mxc_can_reva_regs_t *can, mxc_can_req_t *req,
                                mxc_dma_reqsel_t reqsel, void (*dma_cb)(int, int));

int MXC_CAN_RevA_Handler(mxc_can_reva_regs_t *can, uint8_t *intfl, uint8_t *eintfl);

int MXC_CAN_RevA_Control(mxc_can_reva_regs_t *can, mxc_can_ctrl_t ctrl, uint32_t ctrl_arg);

int MXC_CAN_RevA_SetWakeupTimer(mxc_can_reva_regs_t *can, uint8_t prescaler, uint16_t wup_filter_tm,
                                uint32_t wup_expire_tm);

mxc_can_stat_t MXC_CAN_RevA_GetStatus(mxc_can_reva_regs_t *can);

void MXC_CAN_RevA_SignalUnitEvent(uint32_t can_idx, mxc_can_unit_evt_t event);

void MXC_CAN_RevA_SignalObjectEvent(uint32_t can_idx, mxc_can_obj_evt_t event);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_CAN_CAN_REVA_H_
