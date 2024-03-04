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

/****** Includes *******/
#include <stddef.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "spixf.h"
#include "spixf_reva.h"

/****** Functions ******/

int MXC_SPIXF_Init(uint32_t cmdval, uint32_t frequency)
{
#ifndef MSDK_NO_GPIO_CLK_INIT
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXIP); // SPIX
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIXFC); // SPIXFC
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_ICACHEXIP); // XIP ICACHE
    MXC_GPIO_Config(&gpio_cfg_spixf);
#endif

    return MXC_SPIXF_RevA_Init((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                               (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, cmdval, frequency);
}

void MXC_SPIXF_Shutdown(void)
{
    MXC_SPIXF_RevA_Shutdown((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXFC); // SPIXFC
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPIXIP); // SPIX
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_ICACHEXIP); // XIP ICACHE
}

void MXC_SPIXF_IOCtrl(mxc_spixf_ds_t sclk_ds, mxc_spixf_ds_t ss_ds, mxc_spixf_ds_t sdio_ds,
                      mxc_spixf_pup_t pupdctrl)
{
    MXC_SPIXF_RevA_IOCtrl((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, sclk_ds, ss_ds, sdio_ds, pupdctrl);
}

int MXC_SPIXF_Clocks(uint32_t len, uint8_t deass)
{
    return MXC_SPIXF_RevA_Clocks((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                 (mxc_spixfm_reva_regs_t *)MXC_SPIXFM,
                                 (mxc_spixfc_fifo_reva_regs_t *)MXC_SPIXFC_FIFO, len, deass);
}

int MXC_SPIXF_Transaction(mxc_spixf_req_t *req)
{
    return MXC_SPIXF_RevA_Transaction((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                      (mxc_spixfc_fifo_reva_regs_t *)MXC_SPIXFC_FIFO, req);
}

int MXC_SPIXF_TransactionAsync(mxc_spixf_req_t *req)
{
    return MXC_SPIXF_RevA_TransactionAsync((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                           (mxc_spixfc_fifo_reva_regs_t *)MXC_SPIXFC_FIFO, req);
}

int MXC_SPIXF_AbortAsync(mxc_spixf_req_t *req)
{
    return MXC_SPIXF_RevA_AbortAsync((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, req);
}

void MXC_SPIXF_Handler(void)
{
    MXC_SPIXF_RevA_Handler((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                           (mxc_spixfc_fifo_reva_regs_t *)MXC_SPIXFC_FIFO);
}

int MXC_SPIXF_ReadyForSleep(void)
{
    return MXC_SPIXF_RevA_ReadyForSleep((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_EnableInt(uint32_t mask)
{
    return MXC_SPIXF_RevA_EnableInt((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

int MXC_SPIXF_DisableInt(uint32_t mask)
{
    return MXC_SPIXF_RevA_DisableInt((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

int MXC_SPIXF_ClearFlags(uint32_t flags)
{
    return MXC_SPIXF_RevA_ClearFlags((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, flags);
}

int MXC_SPIXF_GetFlags(void)
{
    return MXC_SPIXF_RevA_GetFlags((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

//Low level

int MXC_SPIXF_SetMode(mxc_spixf_mode_t mode)
{
    return MXC_SPIXF_RevA_SetMode((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                  (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, mode);
}

mxc_spixf_mode_t MXC_SPIXF_GetMode(void)
{
    return MXC_SPIXF_RevA_GetMode((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetSSPolActiveHigh(void)
{
    return MXC_SPIXF_RevA_SetSSPolActiveHigh((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                             (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetSSPolActiveLow(void)
{
    return MXC_SPIXF_RevA_SetSSPolActiveLow((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                            (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_GetSSPolarity(void)
{
    return MXC_SPIXF_RevA_GetSSPolarity((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetSPIFrequency(unsigned int hz)
{
    return MXC_SPIXF_RevA_SetSPIFrequency((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                          (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, hz);
}

int MXC_SPIXF_SetSPIFrequencyRead(unsigned int hz)
{
    return MXC_SPIXF_RevA_SetSPIFrequency(NULL, (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, hz);
}

int MXC_SPIXF_SetSPIFrequencyWrite(unsigned int hz)
{
    return MXC_SPIXF_RevA_SetSPIFrequency((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, NULL, hz);
}

uint32_t MXC_SPIXF_GetSPIFrequency(void)
{
    return MXC_SPIXF_RevA_GetSPIFrequency((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

uint32_t MXC_SPIXF_GetSPIFrequencyRead(void)
{
    return MXC_SPIXF_RevA_GetSPIFrequency((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

uint32_t MXC_SPIXF_GetSPIFrequencyWrite(void)
{
    return MXC_SPIXF_RevA_GetSPIFrequencyWrite((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetSSActiveTime(mxc_spixf_ssact_t ssact)
{
    return MXC_SPIXF_RevA_SetSSActiveTime((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                          (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, ssact);
}

mxc_spixf_ssact_t MXC_SPIXF_GetSSActiveTime(void)
{
    return MXC_SPIXF_RevA_GetSSActiveTime((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetSSInactiveTime(mxc_spixf_ssiact_t ssiact)
{
    return MXC_SPIXF_RevA_SetSSInactiveTime((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                            (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, ssiact);
}

mxc_spixf_ssiact_t MXC_SPIXF_GetSSInactiveTime(void)
{
    return MXC_SPIXF_RevA_GetSSInactiveTime((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetCmdWidth(mxc_spixf_spiwidth_t width)
{
    return MXC_SPIXF_RevA_SetCmdWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, width);
}

mxc_spixf_spiwidth_t MXC_SPIXF_GetCmdWidth(void)
{
    return MXC_SPIXF_RevA_GetCmdWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetAddrWidth(mxc_spixf_spiwidth_t width)
{
    return MXC_SPIXF_RevA_SetAddrWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, width);
}

mxc_spixf_spiwidth_t MXC_SPIXF_GetAddrWidth(void)
{
    return MXC_SPIXF_RevA_GetAddrWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetDataWidth(mxc_spixf_spiwidth_t width)
{
    return MXC_SPIXF_RevA_SetDataWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, width);
}

mxc_spixf_spiwidth_t MXC_SPIXF_GetDataWidth(void)
{
    return MXC_SPIXF_RevA_GetDataWidth((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_Set4ByteAddr(void)
{
    return MXC_SPIXF_RevA_Set4ByteAddr((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_Set3ByteAddr(void)
{
    return MXC_SPIXF_RevA_Set3ByteAddr((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

unsigned int MXC_SPIXF_GetBytesPerAddr(void)
{
    return MXC_SPIXF_RevA_GetBytesPerAddr((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetModeClk(uint8_t mdclk)
{
    return MXC_SPIXF_RevA_SetModeClk((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, mdclk);
}

uint8_t MXC_SPIXF_GetModeClk(void)
{
    return MXC_SPIXF_RevA_GetModeClk((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetCmdValue(uint8_t cmdval)
{
    return MXC_SPIXF_RevA_SetCmdValue((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, cmdval);
}

int MXC_SPIXF_SetCmdModeEveryTrans(void)
{
    return MXC_SPIXF_RevA_SetCmdModeEveryTrans((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetCmdModeFirstTrans(void)
{
    return MXC_SPIXF_RevA_SetCmdModeFirstTrans((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

mxc_spixf_cmd_t MXC_SPIXF_GetCmdMode(void)
{
    return MXC_SPIXF_RevA_GetCmdMode((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_BBDataOutputEnable(uint8_t mask)
{
    return MXC_SPIXF_RevA_BBDataOutputEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

int MXC_SPIXF_BBDataOutputDisable(uint8_t mask)
{
    return MXC_SPIXF_RevA_BBDataOutputDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

uint8_t MXC_SPIXF_BBDataOutputIsEnabled(void)
{
    return MXC_SPIXF_RevA_BBDataOutputIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

uint8_t MXC_SPIXF_GetBBDataOutputValue(void)
{
    return MXC_SPIXF_RevA_GetBBDataOutputValue((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

uint8_t MXC_SPIXF_GetBBDataInputValue(void)
{
    return MXC_SPIXF_RevA_GetBBDataInputValue((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SetModeData(uint16_t data)
{
    return MXC_SPIXF_RevA_SetModeData((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                      (mxc_spixfm_reva_regs_t *)MXC_SPIXFM, data);
}

uint16_t MXC_SPIXF_GetModeData(void)
{
    return MXC_SPIXF_RevA_GetModeData((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetSCKInverted(void)
{
    return MXC_SPIXF_RevA_SetSCKInverted((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                         (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetSCKNonInverted(void)
{
    return MXC_SPIXF_RevA_SetSCKNonInverted((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                            (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_GetSCKInverted(void)
{
    return MXC_SPIXF_RevA_GetSCKInverted((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SCKFeedbackEnable(void)
{
    return MXC_SPIXF_RevA_SCKFeedbackEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                            (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SCKFeedbackDisable(void)
{
    return MXC_SPIXF_RevA_SCKFeedbackDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC,
                                             (mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SCKFeedbackIsEnabled(void)
{
    return MXC_SPIXF_RevA_SCKFeedbackIsEnabled((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetSCKSampleDelay(uint8_t delay)
{
    return MXC_SPIXF_RevA_SetSCKSampleDelay((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, delay);
}

uint8_t MXC_SPIXF_GetSCKSampleDelay(void)
{
    return MXC_SPIXF_RevA_GetSCKSampleDelay((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetPageSize(mxc_spixf_page_size_t size)
{
    MXC_SPIXF_RevA_SetPageSize((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, size);
}

mxc_spixf_page_size_t MXC_SPIXF_GetPageSize(void)
{
    return MXC_SPIXF_RevA_GetPageSize((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleRXEnabled(void)
{
    return MXC_SPIXF_RevA_SimpleRXEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleRXDisable(void)
{
    return MXC_SPIXF_RevA_SimpleRXDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleRXIsEnabled(void)
{
    return MXC_SPIXF_RevA_SimpleRXIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleModeEnable(void)
{
    return MXC_SPIXF_RevA_SimpleModeEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleModeDisable(void)
{
    return MXC_SPIXF_RevA_SimpleModeDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SimpleModeIsEnabled(void)
{
    return MXC_SPIXF_RevA_SimpleModeIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_SampleOutputEnable(uint8_t mask)
{
    return MXC_SPIXF_RevA_SampleOutputEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

int MXC_SPIXF_SampleOutputDisable(uint8_t mask)
{
    return MXC_SPIXF_RevA_SampleOutputDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC, mask);
}

uint8_t MXC_SPIXF_SampleOutputIsEnabled(void)
{
    return MXC_SPIXF_RevA_SampleOutputIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

uint8_t MXC_SPIXF_GetSampleOutputValue(void)
{
    return MXC_SPIXF_RevA_GetSampleOutputValue((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetSCKDriveHigh(void)
{
    MXC_SPIXF_RevA_SetSCKDriveHigh((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetSCKDriveLow(void)
{
    MXC_SPIXF_RevA_SetSCKDriveLow((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

uint8_t MXC_SPIXF_GetSCKDrive(void)
{
    return MXC_SPIXF_RevA_GetSCKDrive((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetSSDriveOutputHigh(void)
{
    MXC_SPIXF_RevA_SetSSDriveOutputHigh((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetSSDriveOutputLow(void)
{
    MXC_SPIXF_RevA_SetSSDriveOutputLow((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

uint8_t MXC_SPIXF_GetSSDriveOutput(void)
{
    return MXC_SPIXF_RevA_GetSSDriveOutput((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_BitBangModeEnable(void)
{
    return MXC_SPIXF_RevA_BitBangModeEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_BitBangModeDisable(void)
{
    return MXC_SPIXF_RevA_BitBangModeDisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_BitBangModeIsEnabled(void)
{
    return MXC_SPIXF_RevA_BitBangModeIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_RXFIFOEnable(void)
{
    return MXC_SPIXF_RevA_RXFIFOEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_RXFIFODisable(void)
{
    return MXC_SPIXF_RevA_RXFIFODisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_RXFIFOIsEnabled(void)
{
    return MXC_SPIXF_RevA_RXFIFOIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_TXFIFOEnable(void)
{
    return MXC_SPIXF_RevA_TXFIFOEnable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_TXFIFODisable(void)
{
    return MXC_SPIXF_RevA_TXFIFODisable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_TXFIFOIsEnabled(void)
{
    return MXC_SPIXF_RevA_TXFIFOIsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_Enable(void)
{
    return MXC_SPIXF_RevA_Enable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_Disable(void)
{
    return MXC_SPIXF_RevA_Disable((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

int MXC_SPIXF_IsEnabled(void)
{
    return MXC_SPIXF_RevA_IsEnabled((mxc_spixfc_reva_regs_t *)MXC_SPIXFC);
}

void MXC_SPIXF_SetIoctrlSDIODriveHigh(void)
{
    MXC_SPIXF_RevA_SetIoctrlSDIODriveHigh((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetIoctrlSDIODriveLow(void)
{
    MXC_SPIXF_RevA_SetIoctrlSDIODriveLow((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

uint8_t MXC_SPIXF_GetIoctrlSDIODrive(void)
{
    return MXC_SPIXF_RevA_GetIoctrlSDIODrive((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetIoctrlSCLKDriveHigh(void)
{
    MXC_SPIXF_RevA_SetIoctrlSCLKDriveHigh((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetIoctrlSCLKDriveLow(void)
{
    MXC_SPIXF_RevA_SetIoctrlSCLKDriveLow((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

uint8_t MXC_SPIXF_GetIoctrlSCLKDrive(void)
{
    return MXC_SPIXF_RevA_GetIoctrlSCLKDrive((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetIoctrlSSDriveHigh(void)
{
    MXC_SPIXF_RevA_SetIoctrlSSDriveHigh((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetIoctrlSSDriveLow(void)
{
    MXC_SPIXF_RevA_SetIoctrlSSDriveLow((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

uint8_t MXC_SPIXF_GetIoctrlSSDrive(void)
{
    return MXC_SPIXF_RevA_GetIoctrlSSDrive((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

void MXC_SPIXF_SetPuPdCtrl(mxc_spixf_pup_t pupd)
{
    MXC_SPIXF_RevA_SetPuPdCtrl((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, pupd);
}

uint8_t MXC_SPIXF_GetPuPdCtrl(void)
{
    return MXC_SPIXF_RevA_GetPuPdCtrl((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}

int MXC_SPIXF_SetBusIdle(unsigned int busidle)
{
    return MXC_SPIXF_RevA_SetBusIdle((mxc_spixfm_reva_regs_t *)MXC_SPIXFM, busidle);
}

unsigned int MXC_SPIXF_GetBusIdle(void)
{
    return MXC_SPIXF_RevA_GetBusIdle((mxc_spixfm_reva_regs_t *)MXC_SPIXFM);
}
