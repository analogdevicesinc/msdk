/**
 * @file    spixr.h
 * @brief   Registers, Bit Masks and Bit Positions for the SPI RAM XIP Data module.
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
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_SPIXR_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_SPIXR_H_

/* **** Includes **** */
#include "spixr_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spixr SPI External Ram (SPIXR)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 * @brief       Enum to define SPIXR data width.
 */
typedef enum {
    MXC_SPIXR_SINGLE_SDIO = 0,
    MXC_SPIXR_DUAL_SDIO,
    MXC_SPIXR_QUAD_SDIO,
} mxc_spixr_width_t;

/**
 * @brief       SPIXR mode.
 * @note        modes 1 and 2 are not supported
 */
typedef enum {
    MXC_SPIXR_MODE_0,
    MXC_SPIXR_MODE_1,
    MXC_SPIXR_MODE_2,
    MXC_SPIXR_MODE_3,
} mxc_spixr_mode_t;

/**
 * @brief       Configuration parameters of SPIXR
 */
typedef struct {
    uint32_t
        numbits; /**< Number of Bits per character. In slave mode 9-bit character length is not supported. */
    mxc_spixr_width_t data_width; /**< SPI Data width */

    uint32_t ssel_act_1; /**< Slave Select Action delay 1 */
    uint32_t ssel_act_2; /**< Slave Select Action delay 2 */
    uint32_t ssel_inact; /**< Slave Select Inactive delay */

    uint32_t baud_freq; /**< Desired baud rate duty cycle control */
} mxc_spixr_cfg_t;

/* **** Function Prototypes **** */

/**
 * @brief       Unloads bytes from the FIFO
 *
 * @param       buf   The buffer to read the data into
 * @param       len     The number of bytes to read
 *
 * @return     #E_NULL_PTR if NULL buffer passed, #E_NO_ERROR otherwise
 */
int MXC_SPIXR_ReadRXFIFO(uint8_t *buf, int len);

/**
 * @brief       Loads bytes into the FIFO
 *
 * @param       buf   The buffer containing data to write
 * @param       len     The number of bytes to write
 *
 * @return     #E_NULL_PTR if NULL buffer passed, #E_NO_ERROR otherwise
 */
int MXC_SPIXR_WriteTXFIFO(uint8_t *buf, int len);

/**
 * @brief       Select which SS pin is used in SPIXR
 */
void MXC_SPIXR_SetSS(void);

/**
 * @brief       Returns the SS line selected
 *
 * @return      The index of the SS pin to use
 */
int MXC_SPIXR_GetSS(void);

/**
 * @brief       Control the deassertion of the SS line
 *
 * @param       stayActive  Keep the SS line asserted between
 *                          sequential transmissions
 */
void MXC_SPIXR_SetSSCtrl(int stayActive);

/**
 * @brief       Get the setting that controls deassertion of the SS line
 *
 * @return      1 to keep the SS line asserted between sequential transmissions
 */
int MXC_SPIXR_GetSSCtrl(void);

/**
 * @brief       Enable the SPI RAM XIP Data module.
 */
void MXC_SPIXR_Enable(void);

/**
 * @brief       Disable the SPI RAM XIP Data module.
 */
void MXC_SPIXR_Disable(void);

/**
 * @brief       Get if SPIXR is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_IsEnabled(void);

/**
 * @brief       Enable the TXFIFO
 *
 */
void MXC_SPIXR_TXFIFOEnable(void);

/**
 * @brief       Disable the TXFIFO
 *
 */
void MXC_SPIXR_TXFIFODisable(void);

/**
 * @brief       Get if TXFIFO is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_TXFIFOIsEnabled(void);

/**
 * @brief       Enable the TX DMA
 *
 */
void MXC_SPIXR_DMATXFIFOEnable(void);

/**
 * @brief       Disable the TX DMA
 *
 */
void MXC_SPIXR_DMATXFIFODisable(void);

/**
 * @brief       Get if TX DMA is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_DMATXFIFOIsEnabled(void);

/**
 * @brief       Enable the RXFIFO
 *
 */
void MXC_SPIXR_RXFIFOEnable(void);

/**
 * @brief       Disable the RXFIFO
 *
 */
void MXC_SPIXR_RXFIFODisable(void);

/**
 * @brief       Get if RXFIFO is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_RXFIFOIsEnabled(void);

/**
 * @brief       Enable the RX DMA
 *
 */
void MXC_SPIXR_DMARXFIFOEnable(void);

/**
 * @brief       Disable the RX DMA
 *
 */
void MXC_SPIXR_DMARXFIFODisable(void);

/**
 * @brief       Get if RX DMA is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_DMARXFIFOIsEnabled(void);

/**
 * @brief       Enable three wire mode
 *
 */
void MXC_SPIXR_ThreeWireModeEnable(void);

/**
 * @brief       Disable three wire mode
 *
 */
void MXC_SPIXR_ThreeWireModeDisable(void);

/**
 * @brief       Get if three wire mode is enabled
 *
 * @return      1 = enabled, 0 = disabled
 */
int MXC_SPIXR_ThreeWireModeIsEnabled(void);

/**
 * @brief       Get the number of bytes currently in the TX FIFO
 *
 * @return      The number of bytes currently in the TX FIFO
 */
int MXC_SPIXR_GetTXFIFOCount(void);

/**
 * @brief       Get the number of bytes currently in the RX FIFO
 *
 * @return      The number of bytes currently in the RX FIFO
 */
int MXC_SPIXR_GetRXFIFOCount(void);

/**
 * @brief       Clear TX FIFO
 *
 */
void MXC_SPIXR_TXFIFOClear(void);

/**
 * @brief       Clear RX FIFO
 *
 */
void MXC_SPIXR_RXFIFOClear(void);

/**
 * @brief       Set the SPI Width used
 *
 * @param       width   The width to be used
 *
 * @return      E_NO_ERROR if successful, E_BAD_PARAM otherwise.
 */
int MXC_SPIXR_SetWidth(mxc_spixr_width_t width);

/**
 * @brief       Set the SPI Mode used
 *
 * @param       mode   The mode to be used
 *
 * @return      E_NO_ERROR if successful, E_BAD_PARAM otherwise.
 */
int MXC_SPIXR_SetSPIMode(mxc_spixr_mode_t mode);

/**
 * @brief       Set the active state of the SS line
 *
 * @param       activeLow   Make the slave select line active low
 *
 * @return      E_NO_ERROR
 */
int MXC_SPIXR_SetSSPolarity(int activeLow);

/**
 * @brief       Set the SS Timing Parameters
 * @note        All timing is in units of system clocks
 *
 * @param       ssIActDelay Delay between end of transaction and start of next
 * @param       postActive  Time after last SCLK  that SS remains active
 * @param       preActive   Time after SS becomes active until first SCLK
 *
 */
void MXC_SPIXR_SetSSTiming(unsigned int ssIActDelay, unsigned int postActive,
                           unsigned int preActive);

/**
 * @brief       Set the SPI Frequency
 *
 * @param       hz  The requested SCLK frequency
 *
 * @return      The actual speed set in Hz
 */
int MXC_SPIXR_SetFrequency(int hz);

/**
 * @brief       Get the SPI Frequency
 *
 * @return      The current speed in Hz
 */
int MXC_SPIXR_GetFrequency(void);

/**
 * @brief       Get the active interrupt flags
 * @note        See \ref mxc_spixr_regs_t for a detailed list of flags
 *
 * @return      The SPIXR interrupt flags
 */
int MXC_SPIXR_GetIntFlags(void);

/**
 * @brief       Enable SPIXR interrupts
 *
 * @param       flags   The flags to enable
 */
void MXC_SPIXR_EnableInt(int flags);

/**
 * @brief       Disable SPIXR interrupts
 *
 * @param       flags   The flags to disable
 */
void MXC_SPIXR_DisableInt(int flags);

/**
 * @brief       Get the active wake up flags
 * @note        See \ref mxc_spixr_regs_t for a detailed list of flags
 *
 * @return      The SPIXR wake up flags
 */
int MXC_SPIXR_GetWakeUpFlags(void);

/**
 * @brief       Enable Wake up for SPIXR
 *
 * @param       flags   The flags to disable
 */
void MXC_SPIXR_EnableWakeUp(int flags);

/**
 * @brief       Enable Wake up for SPIXR
 *
 * @param       flags   The flags to disable
 */
void MXC_SPIXR_DisableWakeUp(int flags);

/**
 * @brief       Enable the external memory mode
 */
void MXC_SPIXR_ExMemEnable(void);

/**
 * @brief       Disable the SPI RAM XIP Data module.
 */
void MXC_SPIXR_ExMemDisable(void);

/**
 * @brief       Put 255 characters worth of clocks between address
 *              and read phase of external memory transactions
 *
 * @param       delay255 add the delay
 */
void MXC_SPIXR_ExMemUseDummy(int delay255);

/**
 * @brief       Set the write command used for external memory mode
 *
 * @param       command The command to be used
 */
void MXC_SPIXR_ExMemSetWriteCommand(uint8_t command);

/**
 * @brief       Get the write command used for external memory mode
 *
 * @return      the command to be used
 */
uint8_t MXC_SPIXR_ExMemGetWriteCommand(void);

/**
 * @brief       Set the read command used for external memory mode
 *
 * @param       command The command to be used
 */
void MXC_SPIXR_ExMemSetReadCommand(uint8_t command);

/**
 * @brief       Get the read command used for external memory mode
 *
 * @return      the command to be used
 */
uint8_t MXC_SPIXR_ExMemGetReadCommand(void);

/**
 * @brief   SPI active status.
 *
 * In Master mode, set when transaction starts, cleared when last bit of last
 * character is acted upon and Slave Select de-assertion would occur.
 * @return  0 if inactive, 1 if active
 */
int MXC_SPIXR_Busy(void);

/**
 * @brief       Initialize the SPI RAM XIP Data module.
 * @param       cfg   initialize SPIXR parameters
 * @return      #E_NO_ERROR if the SPIXR is initialized successfully,
 *              @ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIXR_Init(mxc_spixr_cfg_t *cfg);

/**
 * @brief       Shut Down the SPI RAM XIP Data Module
 *
 * @return      #E_NO_ERROR if the SPIXR is shutdown successfully,
 *              @ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_SPIXR_Shutdown(void);

/**
 * @brief       Send a SPI formatted instruction to external RAM
 * @param       cmd         Array of instructions to send
 * @param       length      number of bytes to send
 * @param       tx_num_char number of bytes to send
 */
void MXC_SPIXR_SendCommand(uint8_t *cmd, uint32_t length, uint32_t tx_num_char);

/**@} end of group spixr */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_SPIXR_H_
