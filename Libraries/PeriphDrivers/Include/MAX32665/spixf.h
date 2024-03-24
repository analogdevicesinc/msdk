/**
 * @file    spixf.h
 * @brief   SPI Flash Controller driver header file.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SPIXF_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SPIXF_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_sys.h"
#include "spixfc_regs.h"
#include "spixfm_regs.h"
#include "spixfc_fifo_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spixf SPI External Flash (SPIXF)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 * @brief      Active levels for slave select lines.
 */
typedef enum {
    MXC_SPIXF_SSEL0_HIGH = (0x1 << 0),
    MXC_SPIXF_SSEL0_LOW = 0,
    MXC_SPIXF_SSEL1_HIGH = (0x1 << 1),
    MXC_SPIXF_SSEL1_LOW = 0,
    MXC_SPIXF_SSEL2_HIGH = (0x1 << 2),
    MXC_SPIXF_SSEL2_LOW = 0,
    MXC_SPIXF_SSEL3_HIGH = (0x1 << 3),
    MXC_SPIXF_SSEL3_LOW = 0
} mxc_spixf_ssel_t;

/**
 * @brief      Header direction.
 */
typedef enum {
    MXC_SPIXF_HEADER_DIR_NONE,
    MXC_SPIXF_HEADER_DIR_TX,
    MXC_SPIXF_HEADER_DIR_RX,
    MXC_SPIXF_HEADER_DIR_BOTH,
} mxc_spixf_hdr_direction_t;

/**
 * @brief      SPIXF Pin mode.
 */
typedef enum {
    MXC_SPIXF_ACTIVE_LOW,
    MXC_SPIF_ACTIVE_HIGH,
} mxc_spixf_sspol_t;

/**
 * @brief      SPIXF set command.
 */
typedef enum {
    MXC_SPIXF_CMD_EVERY_TRANS,
    MXC_SPIXF_CMD_FIRST_TRANS,
} mxc_spixf_cmd_t;

/**
 * @brief      SPIXF mode.
 * @note       modes 1 and 2 are not supported
 */
typedef enum {
    MXC_SPIXF_MODE_0 = 0,
    MXC_SPIXF_MODE_3 = 3,
} mxc_spixf_mode_t;

/**
 * @brief      Select page size.
 */
typedef enum {
    MXC_SPIXF_4B,
    MXC_SPIXF_8B,
    MXC_SPIXF_16B,
    MXC_SPIXF_32B,
} mxc_spixf_page_size_t;

/**
 * @brief      Header units.
 */
typedef enum {
    MXC_SPIXF_HEADER_UNITS_BITS,
    MXC_SPIXF_HEADER_UNITS_BYTES,
    MXC_SPIXF_HEADER_UNITS_PAGES,
} mxc_spixf_hdr_units_t;

/**
 * @brief      Number of data lines to use.
 */
typedef enum {
    MXC_SPIXF_WIDTH_1, ///< 1 Data Line.
    MXC_SPIXF_WIDTH_2, ///< 2 Data Lines (x2).
    MXC_SPIXF_WIDTH_4, ///< 4 Data Lines (x4).
} mxc_spixf_width_t;

/**
 * @brief      MXC_SPIXF configuration type.
 */
typedef struct {
    mxc_spixf_mode_t mode; ///< MXC_SPIXF mode to use, 0-3.
    mxc_spixf_sspol_t
        ssel_pol; ///< Mask of active levels for slave select signals, use mxc_spixf_ssel_t.
    uint32_t hz; ///< SPI Frequency in Hz.
} mxc_spixf_cfg_t;

/**
 * @brief      Slave select active timing
 */
typedef enum {
    MXC_SPIXF_SYS_CLOCKS_0,
    MXC_SPIXF_SYS_CLOCKS_2,
    MXC_SPIXF_SYS_CLOCKS_4,
    MXC_SPIXF_SYS_CLOCKS_8,
} mxc_spixf_ssact_t;

/**
 * @brief      Slave select Inactive timing
 */
typedef enum {
    MXC_SPIXF_SYS_CLOCKS_1, ///< 1 system clocks
    MXC_SPIXF_SYS_CLOCKS_3, ///< 3 system clocks
    MXC_SPIXF_SYS_CLOCKS_5, ///< 5 system clocks
    MXC_SPIXF_SYS_CLOCKS_9, ///< 9 system clocks
} mxc_spixf_ssiact_t;

/**
 * @brief      Data Width, # of data I/O used to rcv data
 */
typedef enum {
    MXC_SPIXF_SINGLE_SDIO,
    MXC_SPIXF_DUAL_SDIO,
    MXC_SPIXF_QUAD_SDIO,
    MXC_SPIXF_INVALID,
} mxc_spixf_spiwidth_t;

/**
 * @brief      IO pullup/pulldown Control
 */
typedef enum {
    MXC_SPIXF_TRISTATE = MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_TRI_STATE,
    MXC_SPIXF_PULL_UP = MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_UP,
    MXC_SPIXF_PULL_DOWN = MXC_S_SPIXFM_IO_CTRL_PU_PD_CTRL_PULL_DOWN,
} mxc_spixf_pup_t;

/**
 * @brief       SPIXF drive strentgh
 * 
 */
typedef enum { MXC_SPIXF_LOW = 0, MXC_SPIXF_HIGH } mxc_spixf_ds_t;

/**
 * @brief      MXC_SPIXF Transaction request.
 */
typedef struct mxc_spixf_req mxc_spixf_req_t;

/**
 * @brief   Callback for asynchronous request.
 * @param   mxc_spixf_req_t*  Pointer to the transaction request.
 * @param   int         Error code.
 */
typedef void (*spixr_complete_cb_t)(mxc_spixf_req_t *, int);

struct mxc_spixf_req {
    uint8_t deass; ///< De-assert slave select at the end of the transaction.
    uint8_t wait_tx; ///< Wait for the TX FIFO to be empty before returning.
    const uint8_t *tx_data; ///< TX buffer.
    uint8_t *rx_data; ///< RX buffer.
    mxc_spixf_width_t width; ///< Number of data lines to use
    unsigned len; ///< Number of bytes to send.
    unsigned read_num; ///< Number of bytes read.
    unsigned write_num; ///< Number of bytes written.
    spixr_complete_cb_t callback; ///< callback function
};

/***** Globals *****/

/***** Function Prototypes *****/
/**
 * @brief      Setup MXC_SPIXF for Execute in Place
 *
 * @param      cmdval       command for the external flash chip.
 * @param      frequency    Frequency of transmission
 *
 * @return     None
 */
int MXC_SPIXF_Init(uint32_t cmdval, uint32_t frequency);

/**
 * @brief      Shutdown MXC_SPIXF module.
 */
void MXC_SPIXF_Shutdown(void);

/**
 * @brief      Setup Drive Strength on the I/O pins
 *
 * @param      sclk_ds   SCLK drive strength
 * @param      ss_ds     Slave Select Drive Strength
 * @param      sdio_ds   SDIO Drive Strength
 * @param      pupdctrl  IO Pullup/Pulldown Control
 */
void MXC_SPIXF_IOCtrl(mxc_spixf_ds_t sclk_ds, mxc_spixf_ds_t ss_ds, mxc_spixf_ds_t sdio_ds,
                      mxc_spixf_pup_t pupdctrl);

/**
 * @brief      Send Clock cycles on SCK without reading or writing.
 *
 * @param      len    Number of clock cycles to send.
 * @param      deass  De-assert slave select at the end of the transaction.
 *
 * @return     Cycles transacted if everything is successful, error if
 *             unsuccessful.
 */
int MXC_SPIXF_Clocks(uint32_t len, uint8_t deass);

/**
 * @brief      Read/write MXC_SPIXF data. Will block until transaction is
 *             complete.
 *
 * @param      req   Request for a MXC_SPIXF transaction.
 * @note       Callback is ignored.
 *
 * @return     Bytes transacted if everything is successful, error if
 *             unsuccessful.
 */
int MXC_SPIXF_Transaction(mxc_spixf_req_t *req);

/**
 * @brief      Asynchronously read/write MXC_SPIXF data.
 *
 * @param      req   Request for a MXC_SPIXF transaction.
 * @note       Request structure must remain allocated until callback.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_TransactionAsync(mxc_spixf_req_t *req);

/**
 * @brief      Abort asynchronous request.
 *
 * @param      req   Pointer to request for a MXC_SPIXF transaction.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_AbortAsync(mxc_spixf_req_t *req);

/**
 * @brief      MXC_SPIXF interrupt handler.
 * @details    This function should be called by the application from the
 *             interrupt handler if MXC_SPIXF interrupts are enabled.
 *             Alternately, this function can be periodically called by the
 *             application if MXC_SPIXF interrupts are disabled.
 */
void MXC_SPIXF_Handler(void);

/**
 * @brief      Attempt to prepare the MXC_SPIXF for sleep.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_ReadyForSleep(void);

/**
 * @brief      Enable Interrupts
 *
 * @param[in]  mask  The mask for the interrupts to enable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_EnableInt(uint32_t mask);

/**
 * @brief      Disable Interrupts
 *
 * @param[in]  mask  The mask for the interrupts to enable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_DisableInt(uint32_t mask);

/**
 * @brief      Clear Flags that have been set
 *
 * @param[in]  flags  The flags to be cleared
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_ClearFlags(uint32_t flags);

/**
 * @brief      Get Flags of Interrupts of Flags that have been set.
 *
 * @return     the Flags that are set
 */
int MXC_SPIXF_GetFlags(void);

//Low level

/**
 * @brief      Sets the SPI Mode
 *
 * @param[in]  mode  The enum that corresponds to the SPI Mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetMode(mxc_spixf_mode_t mode);

/**
 * @brief      Get which mode that spixf is currently set for
 *
 * @return     returns the enum for spi 0 or spi3
 */
mxc_spixf_mode_t MXC_SPIXF_GetMode(void);

/**
 * @brief      Set the slave select polarity to high or low
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSSPolActiveHigh(void);

/**
 * @brief      Set the slave select polarity to high or low
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSSPolActiveLow(void);

/**
 * @brief      Get the current slave select polarity
 *
 * @return     Active High is (0), Active Low is (1)
 */
int MXC_SPIXF_GetSSPolarity(void);

/**
 * @brief      Sets both the read and write SPI clock frequency to the given value.
 *
 * @param[in]  hz   The frequency the spi will communicating at.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSPIFrequency(unsigned int hz);

/**
 * @brief      Sets the SPI clock frequency for read operations.
 *
 * @param[in]  hz   The frequency the spi will communicating at.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSPIFrequencyRead(unsigned int hz);

/**
 * @brief      Sets the SPI clock frequency for write operations.
 *
 * @param[in]  hz   The frequency the spi will communicating at.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSPIFrequencyWrite(unsigned int hz);

/**
 * @brief      Get the current SPI clock frequency.  If the read and write clock
 *             frequencies have been set to different values, this function will
 *             return the read frequency.
 *
 * @return     The current frequency that the SPI is communicating at.
 */
uint32_t MXC_SPIXF_GetSPIFrequency(void);

/**
 * @brief      Get the current SPI clock frequency for read operations.
 *
 * @return     The current frequency that the SPI is communicating at.
 */
uint32_t MXC_SPIXF_GetSPIFrequencyRead(void);

/**
 * @brief      Get the current SPI clock frequency for write operations.
 *
 * @return     The current frequency that the SPI is communicating at.
 */
uint32_t MXC_SPIXF_GetSPIFrequencyWrite(void);

/**
 * @brief      Set the slave select active timing. This will control the delay from assertion of slave select to start
 *             of the SCK pulse and delay from the end of SCK pulse to de-assertion of slave select.
 *
 * @param[in]  ssact  The enum that corresponds with 0,2,4,8 clocks of delay
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 *
 */
int MXC_SPIXF_SetSSActiveTime(mxc_spixf_ssact_t ssact);

/**
 * @brief      Get the slave select active timing setting.
 *
 * @return     gives the enum value that corresponds to 0,2,4,8 clocks of delay
 *             that ss is currently set at.
 */
mxc_spixf_ssact_t MXC_SPIXF_GetSSActiveTime(void);

/**
 * @brief      Set slave select inactive timing to delay from de-assertion of
 *             slave select to re-assertion of slave select for another spi
 *             transaction.
 *
 * @param[in]  ssiact  The enum that corresponds to a 1,3,5,9 clock delay
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSSInactiveTime(mxc_spixf_ssiact_t ssiact);

/**
 * @brief      Get slave select inactive timing to delay from de-assertion of slave select to re-assertion of
 *             slave select for another spi transaction.
 *
 * @return     The enum that corresponds to a 1,3,5,9 clock delay
 */
mxc_spixf_ssiact_t MXC_SPIXF_GetSSInactiveTime(void);

/**
 * @brief      Set Number of data I/O used to send commands
 *
 * @param[in]  width  Enum that corresponds to single, dual, quad SDIO.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetCmdWidth(mxc_spixf_spiwidth_t width);

/**
 * @brief      Get number of data I/O expected to be used for commands
 *
 * @return     Enum that corresponds to single, dual, quad SDIO
 */
mxc_spixf_spiwidth_t MXC_SPIXF_GetCmdWidth(void);

/**
 * @brief      Set Number of data I/O used to send address
 *
 * @param[in]  width  Enum that corresponds to single, dual, quad SDIO.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetAddrWidth(mxc_spixf_spiwidth_t width);

/**
 * @brief      Get number of data I/O expected to be used for address
 *
 * @return     Enum that corresponds to single, dual, quad SDIO
 */
mxc_spixf_spiwidth_t MXC_SPIXF_GetAddrWidth(void);

/**
 * @brief      Set Number of data I/O used to send data
 *
 * @param[in]  width  Enum that corresponds to single, dual, quad SDIO.
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetDataWidth(mxc_spixf_spiwidth_t width);

/**
 * @brief      Get number of data I/O expected to be used for data
 *
 * @return     Enum that corresponds to single, dual, quad SDIO
 */
mxc_spixf_spiwidth_t MXC_SPIXF_GetDataWidth(void);

/**
 * @brief      Set address mode to be 4 byte address
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_Set4ByteAddr(void);

/**
 * @brief      Set address mode to be 3 byte address
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_Set3ByteAddr(void);

/**
 * @brief      Get number of bytes in address
 *
 * @return     3-byte address (3) or 4-byte address (4)
 */
unsigned int MXC_SPIXF_GetBytesPerAddr(void);

/**
 * @brief      Set number of SPI clocks needed during the mode/dummy phase of
 *             fetch
 *
 * @param[in]  mdclk  The number of clocks from 0-15
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetModeClk(uint8_t mdclk);

/**
 * @brief      Get number of SPI clocks being used for the mode/dummy phase of
 *             fetch
 *
 * @return     the number of clocks mode clock is set at.
 */
uint8_t MXC_SPIXF_GetModeClk(void);

/**
 * @brief      Set command value for spi transaction
 *
 * @param[in]  cmdval
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetCmdValue(uint8_t cmdval);

/**
 * @brief      Set command mode to send read a command every time a spi
 *             transaction is initiated
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetCmdModeEveryTrans(void);

/**
 * @brief      Set command mode to send a read a command the first transaction.
 */
int MXC_SPIXF_SetCmdModeFirstTrans(void);

/**
 * @brief      Get Command mode
 *
 * @return     When the read command is sent.  It will be either every time (0) or First transaction only (1).
 */
mxc_spixf_cmd_t MXC_SPIXF_GetCmdMode(void);

/**
 * @brief      Enable bits to be in bit bang output mode
 *
 * @param[in]  mask  Enable output bit bang on specific bits so bit 3 set will
 *                   enable bit 3
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_BBDataOutputEnable(uint8_t mask);

/**
 * @brief      Disable bits to not be in bit bang output mode
 *
 * @param[in]  mask  Disable output bit bang on specific bits so bit 3 set will
 *                   disable bit 3
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_BBDataOutputDisable(uint8_t mask);

/**
 * @brief      Get if bit bang output mode is enabled or disabled for all bits
 *
 * @return     Returns if Output bit bang is enabled (1) or disabled (0) in bits
 *             0:3 corresponding with sdio0:sdio3
 */
uint8_t MXC_SPIXF_BBDataOutputIsEnabled(void);

/**
 * @brief      Get Output data value
 *
 * @return     Returns output data value of a 1 or 0 in bits 0:3 corresponding with sdio0:sdio3
 */
uint8_t MXC_SPIXF_GetBBDataOutputValue(void);

/**
 * @brief      Get Input data value
 *
 * @return     Returns input data value of a 1 or 0 in bits 0:3 corresponding with sdio0:sdio3
 */
uint8_t MXC_SPIXF_GetBBDataInputValue(void);

/**
 * @brief      Set the data to send with the dummy clocks
 *
 * @param[in]  data  The data to send
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetModeData(uint16_t data);

/**
 * @brief      Get the data that is supposed to send with the dummy clocks
 *
 * @return     the data being sent with dummy clocks
 */
uint16_t MXC_SPIXF_GetModeData(void);

/**
 * @brief      Set SCK to be inverted
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSCKInverted(void);

/**
 * @brief      Set SCK to be non-inverted
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSCKNonInverted(void);

/**
 * @brief      Get weather SCK is inverted or non-inverted
 *
 * @return     Enum that corresponds to Inverted (1) or non-inverted (0)
 */
int MXC_SPIXF_GetSCKInverted(void);

/**
 * @brief      Enable SCK Feedback
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SCKFeedbackEnable(void);

/**
 * @brief      Disable SCK Feedback
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SCKFeebackDisable(void);

/**
 * @brief      Get if SCK Feedback is enabled or disabled
 *
 * @return     Returns if feedback mode is Enabled (1) or disabled (0).
 */
int MXC_SPIXF_SCKFeebackIsEnabled(void);

/**
 * @brief      Set SPI clock periods to delay before sampling SDIO input.  This
 *             value must be less than or equal to HICLK
 *
 * @param[in]  delay  The number of clocks to delay between 0-15
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetSCKSampleDelay(uint8_t delay);

/**
 * @brief      Get the Number of clocks being delayed before sampling SDIO
 *
 * @return     get number of clocks between 0-15.
 */
uint8_t MXC_SPIXF_GetSCKSampleDelay(void);

/**
 * @brief      Set Page Size for transactions
 *
 * @param[in]  size  The size of a transaction in bytes
 *
 */
void MXC_SPIXF_SetPageSize(mxc_spixf_page_size_t size);

/**
 * @brief      Get the current page size for a transaction
 *
 * @return     The enum that corresponds with the current page size.
 */
mxc_spixf_page_size_t MXC_SPIXF_GetPageSize(void);

/**
 * @brief      Set to Receive only in simple mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SimpleRXEnabled(void);

/**
 * @brief      Set to Receive only in simple mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SimpleRXDisable(void);

/**
 * @brief      Get Receive only mode setting
 *
 * @return     Returns if simple receive is enabled or disabled
 */
int MXC_SPIXF_SimpleRXIsEnabled(void);

/**
 * @brief      Enable Simple Mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SimpleModeEnable(void);

/**
 * @brief      Disable Simple Mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SimpleModeDisable(void);

/**
 * @brief      Get if Simple mode is enabled or disabled
 *
 * @return     Returns if simple mode is enabled or disabled
 */
int MXC_SPIXF_SimpleModeIsEnabled(void);

/**
 * @brief      Enable bit bang sample output mode on spefic bits
 *
 * @param[in]  mask  Enable sample output bit bang mode. so bit 3 set will
 *                   enable bit 3
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SampleOutputEnable(uint8_t mask);

/**
 * @brief      Disable bit bang sample output mode on spefic bits
 *
 * @param[in]  mask  Disable sample output bit bang mode. so bit 3 set will
 *                   disable bit 3
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SampleOutputDisable(uint8_t mask);

/**
 * @brief      Get if sample output bit bang is enabled or disabled
 *
 * @return     Returns sample output bit bang mode if a 1 (enabled) or 0
 *             (disabled) in bits 0:3 corresponding with sdio0:sdio3
 */
uint8_t MXC_SPIXF_SampleOutputIsEnabled(void);

/**
 * @brief      Get bit bang sample output value
 *
 * @return     Returns sample output bit bang value of a 1 or 0 in bits 0:3
 *             corresponding with sdio0:sdio3
 */
uint8_t MXC_SPIXF_GetSampleOutputValue(void);

/**
 * @brief      Drive SCK High
 */
void MXC_SPIXF_SetSCKDriveHigh(void);

/**
 * @brief      Drive SCK Low
 */
void MXC_SPIXF_SetSCKDriveLow(void);

/**
 * @brief      Get if SCK is high or low
 *
 * @return     returns 1 if High, 0 if  Low
 */
uint8_t MXC_SPIXF_GetSCKDrive(void);

/**
 * @brief      Set Slave select in bit bang mode to a 1
 */
void MXC_SPIXF_SetSSDriveOutputHigh(void);

/**
 * @brief      Set Slave select in bit bang mode to a 0
 */
void MXC_SPIXF_SetSSDriveOutputLow(void);

/**
 * @brief      Get current slave select output
 *
 * @return     0 or 1 based on output of line.
 */
uint8_t MXC_SPIXF_GetSSDriveOutput(void);

/**
 * @brief      Enable bit bang mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_BitBangModeEnable(void);

/**
 * @brief      Disable bit bang mode
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_BitBangModeDisable(void);

/**
 * @brief      Is Bit bang mode enabled or disabled
 *
 * @return     Bit bang mode is enabled (1) or disabled (0)
 */
int MXC_SPIXF_BitBangModeIsEnabled(void);

/**
 * @brief      RX FIFO (results fifo) enable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_RXFIFOEnable(void);

/**
 * @brief      RX FIFO (results fifo) disable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_RXFIFODisable(void);

/**
 * @brief      Is RX FIFO (results fifo) enabled or disabled
 *
 * @return     RX FIFO is enabled (1) or disabled (0)
 */
int MXC_SPIXF_RXFIFOIsEnabled(void);

/**
 * @brief      TX FIFO (Transaction FIFO) enable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_TXFIFOEnable(void);

/**
 * @brief      TX FIFO (Transaction FIFO) disable
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_TXFIFODisable(void);

/**
 * @brief      Is TX FIFO (Transaction FIFO) Enabled or Disabled
 *
 * @return     TX FIFO is enabled (1) or disabled (0)
 */
int MXC_SPIXF_TXFIFOIsEnabled(void);

/**
 * @brief      Enables SPIXF but doesnt change any configurations
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_Enable(void);

/**
 * @brief      Disables SPIXF but doesnt change any configurations
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_Disable(void);

/**
 * @brief      Is SPIXF Enabled or Disabled
 *
 * @return     SPIXF is enabled (1) or disabled (0)
 */
int MXC_SPIXF_IsEnabled(void);

/**
 * @brief      Set IOCTRL SDIO Drive to High
 */
void MXC_SPIXF_SetIoctrlSDIODriveHigh(void);

/**
 * @brief      Set IOCTRL SDIO Drive to Low
 */
void MXC_SPIXF_SetIoctrlSDIODriveLow(void);

/**
 * @brief      Get IOCTRL SDIO Drive
 *
 * @return     1 = high or 0 = low
 */
uint8_t MXC_SPIXF_GetIoctrlSDIODrive(void);

/**
 * @brief      Set IOCTRL SCLK Drive to High
 */
void MXC_SPIXF_SetIoctrlSCLKDriveHigh(void);

/**
 * @brief      Set IOCTRL SCLK Drive to Low
 */
void MXC_SPIXF_SetIoctrlSCLKDriveLow(void);

/**
 * @brief      Get IOCTRL SCLK Drive
 *
 * @return     1= high or 0 = low
 */
uint8_t MXC_SPIXF_GetIoctrlSCLKDrive(void);

/**
 * @brief      Set IOCTRL SS Drive to High
 */
void MXC_SPIXF_SetIoctrlSSDriveHigh(void);

/**
 * @brief      Set IOCTRL SS Drive to Low
 */
void MXC_SPIXF_SetIoctrlSSDriveLow(void);

/**
 * @brief      Get IOCTRL SS Drive
 *
 * @return     1 = high or 0 = low
 */
uint8_t MXC_SPIXF_GetIoctrlSSDrive(void);

/**
 * @brief      Set pull up pull down
 *
 * @param[in]  pupd  The enum corresponding to Pull up pull down states
 */
void MXC_SPIXF_SetPuPdCtrl(mxc_spixf_pup_t pupd);

/**
 * @brief      Get what setting the pull up pull down is set to
 *
 * @return     the enum value for pull up pull down state
 */
uint8_t MXC_SPIXF_GetPuPdCtrl(void);

/**
 * @brief      Set bus idle to a time where ss will be deactivated if timer runs out
 *
 * @param[in]  busidle  The time before the timer runs out
 *
 * @return     See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPIXF_SetBusIdle(unsigned int busidle);

/**
 * @brief      Get Bus Idle time
 *
 * @return     the time the bus will idle before deactivating slave select.
 */
unsigned int MXC_SPIXF_GetBusIdle(void);

/**
 * @defgroup spixfm SPI External Flash Master (SPIXFM)
 * @ingroup spixf
 * @{
 */
/**@} end of group spixfm */

/**
 * @defgroup spixfc SPI External Flash Controller (SPIXFC)
 * @ingroup spixf
 * @{
 */
/**@} end of group spixfc */
/**
 * @defgroup spixfc_fifo SPI External Flash Controller FIFO (SPIXFC_FIFO)
 * @ingroup spixf
 * @{
 */
/**@} end of group spixfc_fifo */

/**@} end of group spixf */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_SPIXF_H_
