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

#ifndef LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXF_SPIXF_REVA_H_
#define LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXF_SPIXF_REVA_H_

/****** Includes *******/
#include "mxc_device.h"
#include "mxc_sys.h"
#include "spixf.h"
#include "spixfc_reva_regs.h"
#include "spixfm_reva_regs.h"
#include "spixfc_fifo_reva_regs.h"

/* **** Definitions **** */
/**
 * @brief      Active levels for slave select lines.
 */
typedef enum {
    MXC_SPIXF_REVA_SSEL0_HIGH = (0x1 << 0),
    MXC_SPIXF_REVA_SSEL0_LOW = 0,
    MXC_SPIXF_REVA_SSEL1_HIGH = (0x1 << 1),
    MXC_SPIXF_REVA_SSEL1_LOW = 0,
    MXC_SPIXF_REVA_SSEL2_HIGH = (0x1 << 2),
    MXC_SPIXF_REVA_SSEL2_LOW = 0,
    MXC_SPIXF_REVA_SSEL3_HIGH = (0x1 << 3),
    MXC_SPIXF_REVA_SSEL3_LOW = 0
} mxc_spixf_reva_ssel_t;

/**
 * @brief      Header direction.
 */
typedef enum {
    MXC_SPIXF_REVA_HEADER_DIR_NONE,
    MXC_SPIXF_REVA_HEADER_DIR_TX,
    MXC_SPIXF_REVA_HEADER_DIR_RX,
    MXC_SPIXF_REVA_HEADER_DIR_BOTH,
} mxc_spixf_reva_hdr_direction_t;

/**
 * @brief      SPIXF Pin mode.
 */
typedef enum {
    MXC_SPIXF_REVA_ACTIVE_LOW,
    MXC_SPIXF_REVA_ACTIVE_HIGH,
} mxc_spixf_reva_sspol_t;

/**
 * @brief      SPIXF set command.
 */
typedef enum {
    MXC_SPIXF_REVA_CMD_EVERY_TRANS,
    MXC_SPIXF_REVA_CMD_FIRST_TRANS,
} mxc_spixf_reva_cmd_t;

/**
 * @brief      SPIXF mode.
 * @note       modes 1 and 2 are not supported
 */
typedef enum {
    MXC_SPIXF_REVA_MODE_0 = 0,
    MXC_SPIXF_REVA_MODE_3 = 3,
} mxc_spixf_reva_mode_t;

/**
 * @brief      Select page size.
 */
typedef enum {
    MXC_SPIXF_REVA_4B,
    MXC_SPIXF_REVA_8B,
    MXC_SPIXF_REVA_16B,
    MXC_SPIXF_REVA_32B,
} mxc_spixf_reva_age_size_t;

/**
 * @brief      Header units.
 */
typedef enum {
    MXC_SPIXF_REVA_HEADER_UNITS_BITS,
    MXC_SPIXF_REVA_HEADER_UNITS_BYTES,
    MXC_SPIXF_REVA_HEADER_UNITS_PAGES,
} mxc_spixf_reva_hdr_units_t;

/**
 * @brief      Number of data lines to use.
 */
typedef enum {
    MXC_SPIXF_REVA_WIDTH_1, ///< 1 Data Line.
    MXC_SPIXF_REVA_WIDTH_2, ///< 2 Data Lines(x2).
    MXC_SPIXF_REVA_WIDTH_4, ///< 4 Data Lines(x4).
} mxc_spixf_reva_width_t;

/**
 * @brief      MXC_SPIXF configuration type.
 */
typedef struct {
    mxc_spixf_reva_mode_t mode; ///< MXC_SPIXF mode to use, 0-3.
    mxc_spixf_reva_sspol_t
        ssel_pol; ///< Mask of active levels for slave select signals, use mxc_spixf_ssel_t.
    uint32_t hz; ///< SPI Frequency in Hz.
} mxc_spixf_reva_cfg_t;

/**
 * @brief      Slave select active timing
 */
typedef enum {
    MXC_SPIXF_REVA_SYS_CLOCKS_0,
    MXC_SPIXF_REVA_SYS_CLOCKS_2,
    MXC_SPIXF_REVA_SYS_CLOCKS_4,
    MXC_SPIXF_REVA_SYS_CLOCKS_8,
} mxc_spixf_reva_ssact_t;

/**
 * @brief      Slave select Inactive timing
 */
typedef enum {
    MXC_SPIXF_REVA_SYS_CLOCKS_1, ///< 1 system clocks
    MXC_SPIXF_REVA_SYS_CLOCKS_3, ///< 3 system clocks
    MXC_SPIXF_REVA_SYS_CLOCKS_5, ///< 5 system clocks
    MXC_SPIXF_REVA_SYS_CLOCKS_9, ///< 9 system clocks
} mxc_spixf_reva_ssiact_t;

/**
 * @brief      Data Width, # of data I/O used to rcv data
 */
typedef enum {
    MXC_SPIXF_REVA_SINGLE_SDIO,
    MXC_SPIXF_REVA_DUAL_SDIO,
    MXC_SPIXF_REVA_QUAD_SDIO,
    MXC_SPIXF_REVA_INVALID,
} mxc_spixf_reva_spiwidth_t;

/**
 * @brief      IO pullup/pulldown Control
 */
typedef enum {
    MXC_SPIXF_REVA_TRISTATE = MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_TRI_STATE,
    MXC_SPIXF_REVA_PULL_UP = MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_UP,
    MXC_SPIXF_REVA_PULL_DOWN = MXC_S_SPIXFM_REVA_IO_CTRL_PU_PD_CTRL_PULL_DOWN,
} mxc_spixf_reva_pup_t;

/**
 * @brief       SPIXF drive strentgh
 * 
 */
typedef enum { MXC_SPIXF_REVA_LOW = 0, MXC_SPIXF_REVA_HIGH } mxc_spixf_reva_ds_t;

/**
 * @brief      MXC_SPIXF Transaction request.
 */
typedef struct mxc_spixf_reva_req mxc_spixf_reva_req_t;

struct mxc_spixf_reva_req {
    uint8_t deass; ///< De-assert slave select at the end of the transaction.
    uint8_t wait_tx; ///< Wait for the TX FIFO to be empty before returning.
    const uint8_t *tx_data; ///< TX buffer.
    uint8_t *rx_data; ///< RX buffer.
    mxc_spixf_reva_width_t width; ///< Number of data lines to use
    unsigned len; ///< Number of bytes to send.
    unsigned read_num; ///< Number of bytes read.
    unsigned write_num; ///< Number of bytes written.
    spixr_complete_cb_t callback; ///< callback function
};

typedef mxc_spixf_pup_t mxc_spixf_padctrl_t;

/***** Function Prototypes *****/
int MXC_SPIXF_RevA_Init(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                        uint32_t cmdval, uint32_t frequency);
void MXC_SPIXF_RevA_Shutdown(mxc_spixfc_reva_regs_t *spixfc);
void MXC_SPIXF_RevA_IOCtrl(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_ds_t sclk_ds,
                           mxc_spixf_ds_t ss_ds, mxc_spixf_ds_t sdio_ds,
                           mxc_spixf_padctrl_t padctrl);
int MXC_SPIXF_RevA_Clocks(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                          mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, uint32_t len, uint8_t deass);
int MXC_SPIXF_RevA_Transaction(mxc_spixfc_reva_regs_t *spixfc,
                               mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req);
int MXC_SPIXF_RevA_TransactionAsync(mxc_spixfc_reva_regs_t *spixfc,
                                    mxc_spixfc_fifo_reva_regs_t *spixfc_fifo, mxc_spixf_req_t *req);
int MXC_SPIXF_RevA_AbortAsync(mxc_spixfc_reva_regs_t *spixfc, mxc_spixf_req_t *req);
void MXC_SPIXF_RevA_Handler(mxc_spixfc_reva_regs_t *spixfc,
                            mxc_spixfc_fifo_reva_regs_t *spixfc_fifo);
int MXC_SPIXF_RevA_ReadyForSleep(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_EnableInt(mxc_spixfc_reva_regs_t *spixfc, uint32_t mask);
int MXC_SPIXF_RevA_DisableInt(mxc_spixfc_reva_regs_t *spixfc, uint32_t mask);
int MXC_SPIXF_RevA_ClearFlags(mxc_spixfc_reva_regs_t *spixfc, uint32_t flags);
int MXC_SPIXF_RevA_GetFlags(mxc_spixfc_reva_regs_t *spixfc);

//Low level
int MXC_SPIXF_RevA_SetMode(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                           mxc_spixf_mode_t mode);
mxc_spixf_mode_t MXC_SPIXF_RevA_GetMode(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetSSPolActiveHigh(mxc_spixfc_reva_regs_t *spixfc,
                                      mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetSSPolActiveLow(mxc_spixfc_reva_regs_t *spixfc,
                                     mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_GetSSPolarity(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetSPIFrequency(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                   unsigned int hz);
uint32_t MXC_SPIXF_RevA_GetSPIFrequency(mxc_spixfm_reva_regs_t *spixfm);
uint32_t MXC_SPIXF_RevA_GetSPIFrequencyWrite(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetSSActiveTime(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                   mxc_spixf_ssact_t ssact);
mxc_spixf_ssact_t MXC_SPIXF_RevA_GetSSActiveTime(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetSSInactiveTime(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                                     mxc_spixf_ssiact_t ssiact);
mxc_spixf_ssiact_t MXC_SPIXF_RevA_GetSSInactiveTime(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetCmdWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width);
mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetCmdWidth(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetAddrWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width);
mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetAddrWidth(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetDataWidth(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_spiwidth_t width);
mxc_spixf_spiwidth_t MXC_SPIXF_RevA_GetDataWidth(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_Set4ByteAddr(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_Set3ByteAddr(mxc_spixfm_reva_regs_t *spixfm);
unsigned int MXC_SPIXF_RevA_GetBytesPerAddr(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetModeClk(mxc_spixfm_reva_regs_t *spixfm, uint8_t mdclk);
uint8_t MXC_SPIXF_RevA_GetModeClk(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetCmdModeEveryTrans(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetCmdModeFirstTrans(mxc_spixfm_reva_regs_t *spixfm);
mxc_spixf_cmd_t MXC_SPIXF_RevA_GetCmdMode(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_BBDataOutputEnable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask);
int MXC_SPIXF_RevA_BBDataOutputDisable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask);
uint8_t MXC_SPIXF_RevA_BBDataOutputIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
uint8_t MXC_SPIXF_RevA_GetBBDataOutputValue(mxc_spixfc_reva_regs_t *spixfc);
uint8_t MXC_SPIXF_RevA_GetBBDataInputValue(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetModeData(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm,
                               uint16_t data);
uint16_t MXC_SPIXF_RevA_GetModeData(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetSCKInverted(mxc_spixfc_reva_regs_t *spixfc, mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetSCKNonInverted(mxc_spixfc_reva_regs_t *spixfc,
                                     mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_GetSCKInverted(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SCKFeedbackEnable(mxc_spixfc_reva_regs_t *spixfc,
                                     mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SCKFeedbackDisable(mxc_spixfc_reva_regs_t *spixfc,
                                      mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SCKFeedbackIsEnabled(mxc_spixfm_reva_regs_t *spixfm);
int MXC_SPIXF_RevA_SetSCKSampleDelay(mxc_spixfc_reva_regs_t *spixfc, uint8_t delay);
uint8_t MXC_SPIXF_RevA_GetSCKSampleDelay(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetCmdValue(mxc_spixfm_reva_regs_t *spixfm, uint8_t cmdval);
uint8_t MXC_SPIXF_RevA_GetCmdValue(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetPageSize(mxc_spixfc_reva_regs_t *spixfc, mxc_spixf_page_size_t size);
mxc_spixf_page_size_t MXC_SPIXF_RevA_GetPageSize(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleRXEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleRXDisable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleRXIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleModeEnable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleModeDisable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SimpleModeIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SampleOutputEnable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask);
int MXC_SPIXF_RevA_SampleOutputDisable(mxc_spixfc_reva_regs_t *spixfc, uint8_t mask);
uint8_t MXC_SPIXF_RevA_SampleOutputIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
uint8_t MXC_SPIXF_RevA_GetSampleOutputValue(mxc_spixfc_reva_regs_t *spixfc);
void MXC_SPIXF_RevA_SetIoctrlSDIODriveHigh(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetIoctrlSDIODriveLow(mxc_spixfm_reva_regs_t *spixfm);
uint8_t MXC_SPIXF_RevA_GetIoctrlSDIODrive(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetIoctrlSCLKDriveHigh(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetIoctrlSCLKDriveLow(mxc_spixfm_reva_regs_t *spixfm);
uint8_t MXC_SPIXF_RevA_GetIoctrlSCLKDrive(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetIoctrlSSDriveHigh(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetIoctrlSSDriveLow(mxc_spixfm_reva_regs_t *spixfm);
uint8_t MXC_SPIXF_RevA_GetIoctrlSSDrive(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetPuPdCtrl(
    mxc_spixfm_reva_regs_t *spixfm,
    mxc_spixf_padctrl_t pad); // Legacy Name. Will start using SetPadCtrl going forward. 1-23-2023
uint8_t MXC_SPIXF_RevA_GetPuPdCtrl(
    mxc_spixfm_reva_regs_t
        *spixfm); // Legacy Name. Will start using GetPadCtrl going forward. 1-23-2023
void MXC_SPIXF_RevA_SetPadCtrl(mxc_spixfm_reva_regs_t *spixfm, mxc_spixf_padctrl_t pad);
uint8_t MXC_SPIXF_RevA_GetPadCtrl(mxc_spixfm_reva_regs_t *spixfm);
void MXC_SPIXF_RevA_SetSCKDriveHigh(mxc_spixfc_reva_regs_t *spixfc);
void MXC_SPIXF_RevA_SetSCKDriveLow(mxc_spixfc_reva_regs_t *spixfc);
uint8_t MXC_SPIXF_RevA_GetSCKDrive(mxc_spixfc_reva_regs_t *spixfc);
void MXC_SPIXF_RevA_SetSSDriveOutputHigh(mxc_spixfc_reva_regs_t *spixfc);
void MXC_SPIXF_RevA_SetSSDriveOutputLow(mxc_spixfc_reva_regs_t *spixfc);
uint8_t MXC_SPIXF_RevA_GetSSDriveOutput(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_BitBangModeEnable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_BitBangModeDisable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_BitBangModeIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_RXFIFOEnable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_RXFIFODisable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_RXFIFOIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_TXFIFOEnable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_TXFIFODisable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_TXFIFOIsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_Enable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_Disable(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_IsEnabled(mxc_spixfc_reva_regs_t *spixfc);
int MXC_SPIXF_RevA_SetBusIdle(mxc_spixfm_reva_regs_t *spixfm, unsigned int busidle);
unsigned int MXC_SPIXF_RevA_GetBusIdle(mxc_spixfm_reva_regs_t *spixfm);

#endif // LIBRARIES_PERIPHDRIVERS_SOURCE_SPIXF_SPIXF_REVA_H_
