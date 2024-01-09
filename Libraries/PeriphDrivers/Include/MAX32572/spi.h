/**
 * @file
 * @brief   Serial Peripheral Interface (SPI) communications driver.
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
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
 ******************************************************************************/

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SPI_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SPI_H_

/***** includes *******/
#include <stdbool.h>
#include "mxc_assert.h"
#include "mxc_device.h"
#include "mxc_lock.h"
#include "mxc_pins.h"
#include "mxc_sys.h"
#include "gpio.h"
#include "spi_regs.h"
#include "dma_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup spi SPI
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/

// clang-format off

/**
 * @brief   The list of types for the SPI peripheral.
 */
typedef enum {
    MXC_SPI_TYPE_SLAVE = 0,
    MXC_SPI_TYPE_TARGET = 0,
    MXC_SPI_TYPE_MASTER = 1,
    MXC_SPI_TYPE_CONTROLLER = 1
} mxc_spi_type_t;

/**
 * @brief   The list of Target Select Control Scheme Options for
 *          target assertion/deassertion.
 */
typedef enum {
    MXC_SPI_TSCONTROL_HW_AUTO = 0, // Automatically by hardware
    MXC_SPI_TSCONTROL_SW_APP = 1   // Through software in the application
} mxc_spi_tscontrol_t;

/**
 * @brief   The list of possible states for an SPI instance.
 */
typedef enum {
    MXC_SPI_STATE_READY = 0, // Ready for transaction
    MXC_SPI_STATE_BUSY = 1   // Busy transferring
} mxc_spi_state_t;

/**
 * @brief   The list of supported SPI Interface Modes.
 */
typedef enum {
    MXC_SPI_INTERFACE_STANDARD = 0,
    MXC_SPI_INTERFACE_4WIRE = 0,
    MXC_SPI_INTERFACE_QUAD = 1,
    MXC_SPI_INTERFACE_3WIRE = 2,
    MXC_SPI_INTERFACE_DUAL = 3
} mxc_spi_interface_t;

/**
 * @brief The list of SPI clock modes
 * 
 * SPI supports four combinations of clock and phase polarity.
 * 
 * Clock polarity is controlled using the bit SPIn_CTRL2.cpol 
 * and determines if the clock is active high or active low
 * 
 * Clock phase determines when the data must be stable for sampling
 */
typedef enum {
    MXC_SPI_CLKMODE_0 = 0, // CPOL: 0    CPHA: 0
    MXC_SPI_CLKMODE_1 = 1, // CPOL: 0    CPHA: 1
    MXC_SPI_CLKMODE_2 = 2, // CPOL: 1    CPHA: 0
    MXC_SPI_CLKMODE_3 = 3  // CPOL: 1    CPHA: 1
} mxc_spi_clkmode_t;

///>>> @deprecated
/**
 * @brief   The list of SPI Widths supported
 * 
 * @deprecated.
 */
typedef enum {
    SPI_WIDTH_3WIRE,    ///< 1 Data line, half duplex
    SPI_WIDTH_STANDARD, ///< MISO/MOSI, full duplex
    SPI_WIDTH_DUAL,     ///< 2 Data lines, half duplex
    SPI_WIDTH_QUAD,     ///< 4 Data lines, half duplex
} mxc_spi_width_t;

/**
 * @brief The list of SPI modes
 * 
 * SPI supports four combinations of clock and phase polarity
 * 
 * Clock polarity is controlled using the bit SPIn_CTRL2.cpol 
 * and determines if the clock is active high or active low
 * 
 * Clock phase determines when the data must be stable for sampling
 *  
 */
typedef enum {
    SPI_MODE_0, ///< clock phase = 0, clock polarity = 0
    SPI_MODE_1, ///< clock phase = 0, clock polarity = 1
    SPI_MODE_2, ///< clock phase = 1, clock polarity = 0
    SPI_MODE_3, ///< clock phase = 1, clock polarity = 1
} mxc_spi_mode_t;
///<<< Deprecated

typedef struct _mxc_spi_req_t mxc_spi_req_t;

/**
 * @brief   The callback routine used to indicate the transaction has terminated.
 *
 * @param   req         The details of the transaction.
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_spi_callback_t)(void *, int result);
typedef mxc_spi_callback_t spi_complete_cb_t; // Support SPI v1 name.

typedef struct _mxc_spi_pins_t mxc_spi_pins_t;
struct _mxc_spi_pins_t {
    bool ss0;                   ///< Target select pin 0.
    bool ss1;                   ///< Target select pin 1.
    bool ss2;                   ///< Target select pin 2.

    bool vddioh;                ///< VDDIOH Select

    bool clock;                 ///< Clock pin
    bool miso;                  ///< miso pin
    bool mosi;                  ///< mosi pin
    bool sdio2;                 ///< SDIO2 pin
    bool sdio3;                 ///< SDIO3 pin

    mxc_gpio_drvstr_t drvstr;   ///< Drive strength setting
    bool ss3;
};

typedef struct {
    // SPI Settings.
    mxc_spi_regs_t *spi;                // Selected SPI Instance
    mxc_spi_clkmode_t clk_mode;         // Clock modes
    uint8_t frame_size;                 // Number of bits per character sent

    // DMA Settings.
    bool use_dma_tx;                    // Enable DMA TX.
    bool use_dma_rx;                    // Enable DMA RX. (use_dma_tx must be true to use DMA RX).
    mxc_dma_regs_t *dma;                // Select DMA instance for SPI DMA.
} mxc_spi_cfg_t;

// Suppport names for backwards compatibility.
struct _mxc_spi_req_t {
    mxc_spi_regs_t *spi;     // Pointer to SPI registers
    int ssIdx;
    int ssDeassert;
    uint8_t *txData;
    uint8_t *rxData;
    uint32_t txLen; // Number of frames to be stored in txData
    uint32_t rxLen; // Number of frames to be stored in rxData
    uint32_t txCnt; // Number of bytes transmitted from txData (Unused for SPI v2)
    uint32_t rxCnt; // Number of bytes stored in rxData (Unused for SPI v2)
    mxc_spi_callback_t completeCB; // completeCB
    uint16_t txDummyValue; // Value of dummy bytes to be sent
};
// clang-format on

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

/**
 * @brief   Initialize and enable SPI peripheral.
 * 
 * This function does not set the Clock Mode (defaults to Clock Mode 0) and 
 * only two interface modes are selectable (Quad Mode or 4-Wire Standard Mode).
 * To change the clock mode, call MXC_SPI_SetClkMode(..)..
 * To select another interface mode, call MXC_SPI_SetInterface(..)..
 *
 * These parameters can be modified after cfgialization using low level functions
 *
 * @param   spi                 Pointer to SPI instance's registers.
 * @param   controller_target   Whether to put the device in controller or target mode. Use
 *                              non-zero for controller mode, and zero for target mode.
 *                                  MXC_SPI_TYPE_CONTROLLER - 1
 *                                  MXC_SPI_TYPE_TARGET - 0
 * @param   if_mode             Set the interface mode.
 *                                  MXC_SPI_INTERFACE_STANDARD - 0 (4 wire)
 *                                  MXC_SPI_INTERFACE_QUAD - 1
 *                                  MXC_SPI_INTERFACE_3WIRE - 2
 *                                  MXC_SPI_INTERFACE_DUAL - 3
 * @param   numTargets          This parameter is UNUSED for SPI v2.
 *                              The number of target used, if in controller mode. This
 *                              is used to obtain control of the necessary TS pins.
 *                              In target mode this is ignored and TS1 is used.
 * @param   ts_active_pol_mask  This field sets the TS active polarity for each
 *                              target, each bit position corresponds to each TS line.
 *                                  ts_active_pol_mask[0] - TS0
 *                                  ts_active_pol_mask[1] - TS1
 *                                  ts_active_pol_mask[n] - TSn
 * @param   freq                The requested clock frequency. The actual clock frequency
 *                              will be returned by the function if successful. Used in
 *                              master mode only.
 * @param   pins                SPI pin structure. Pins selected as true will be initialized 
 *                              for the requested SPI block.            
 *
 * @return  If successful, the actual clock frequency is returned. Otherwise, see
 *          \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_Init(mxc_spi_regs_t *spi, mxc_spi_type_t controller_target, mxc_spi_interface_t if_mode,
                 int numTargets, uint8_t ts_active_pol_mask, uint32_t freq, mxc_spi_pins_t pins);

/**
 * @brief   Configure the SPI peripheral.
 *
 * List of Setting that will be updated:
 *      Clock Mode.
 *      Frame Size (bits).
 *      Interface Mode (Dual, Quad, Standard, 3-Wire).
 *      If DMA selections are true, configure and acquire DMA channels.
 * 
 * @param   cfg         Pointer to SPI configuration struct.         
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_Config(mxc_spi_cfg_t *cfg);

/**
 * @brief   Overwrite the cfg struct with default values.
 *
 * Settings:
 *      SPI APB (SPI1) instance
 *      Default, predefined SPI pins at VDDIO
 *      Controller Mode
 *      Standard 4-wire mode
 *      100KHz speed
 *      CPOL: 0, CPHA: 0
 *      Automatic Hardware mode for TS Control
 *      TS0 pin
 *      Target active polarity is LOW (0)
 *      If use_dma = true, set DMATX and RX to true and set the DMA0 as the default instance.
 *
 * @param   cfg         Pointer to SPI configuration struct.    
 * @param   use_dma_tx  True/False option to configure DMA TX. 
 * @param   use_dma_rx  True/False option to configure DMA RX.  
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.odes.
 */
int MXC_SPI_ConfigStruct(mxc_spi_cfg_t *cfg, bool use_dma_tx, bool use_dma_rx);

/**
 * @brief   Disable and shutdown the SPI instance.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_Shutdown(mxc_spi_regs_t *spi);

/**
 * @brief   Gets the interrupt flags that are currently set
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA)
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 *
 * @return The interrupt flags.
 */
unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t *spi);

/**
 * @brief   Clears the interrupt flags that are currently set.
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA).
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 */
void MXC_SPI_ClearFlags(mxc_spi_regs_t *spi);

/**
 * @brief   Enables specific interrupts
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA).
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 * @param   intEn       The interrupts to be enabled.
 */
void MXC_SPI_EnableInt(mxc_spi_regs_t *spi, unsigned int intEn);

/**
 * @brief   Disables specific interrupts
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA)
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 * @param   intDis      The interrupts to be disabled
 */
void MXC_SPI_DisableInt(mxc_spi_regs_t *spi, unsigned int intDis);

/**
 * @brief   Returns the frequency of the clock used as the bit rate generator for a given SPI instance.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Frequency of the clock used as the bit rate generator
 */
int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t *spi);

/**
 * @brief   Configures the Pre-defined SPI Target Select pins for a specific instance.
 * 
 * @param   spi                 Pointer to SPI instance's registers.
 * @param   ts_control          Target Select Control Scheme (\ref mxc_spi_tscontrol_t).
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetTSControl(mxc_spi_regs_t *spi, mxc_spi_tscontrol_t ts_control);

/**
 * @brief   Set the frequency of the SPI interface.
 *
 * This function is applicable in Master mode only
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   hz          The desired frequency in Hertz.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, unsigned int hz);

/**
 * @brief   Get the frequency of the SPI interface.
 *
 * This function is applicable in Master mode only
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  If successful, the SPI instance's set frequency value is returned. 
 *          Otherwise, see \ref MXC_Error_Codes for a list of return codes.
 */
unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the number of bits per frame.
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   frame_size  The number of bits per frame.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetFrameSize(mxc_spi_regs_t *spi, int frame_size);

/**
 * @brief   Gets the number of bits per frame.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  If successful, the SPI instance's set data size is returned. 
 *          Otherwise, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_GetFrameSize(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the SPI interface mode used for transmissions.
 * 
 * 3-Wire, Standard (4-Wire), Quad, Dual Modes
 * 
 * @param   spi         Pointer to SPI instance's registers.
 * @param   if_mode     SPI interface mode (3-Wire, Standard, Dual SPI, Quad SPI).
 *                      See \ref mxc_spi_datawidth_t
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetInterface(mxc_spi_regs_t *spi, mxc_spi_interface_t if_mode);

/**
 * @brief   Gets the SPI interface mode used for transmissions.
 * 
 * 3-Wire, Standard (4-Wire), Quad, Dual Modes
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  The selected SPI instance's data line width. See \ref mxc_spi_datawidth_t.
 */
mxc_spi_interface_t MXC_SPI_GetInterface(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the SPI clock mode (clock polarity and clock phase).
 * 
 * @param spi           Pointer to SPI instance's registers.
 * @param clk_mode      SPI clock mode. See \ref mxc_spi_clkmode_t.
 *  
 * @return Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_SPI_SetClkMode(mxc_spi_regs_t *spi, mxc_spi_clkmode_t clk_mode);

/**
 * @brief   Gets the SPI clock mode (clock polarity and clock phase).
 * 
 * @param spi           Pointer to SPI instance's registers.
 * @param clk_mode      SPI clock mode. See \ref mxc_spi_clkmode_t
 *  
 * @return The selected SPI instance's clock mode. See \ref mxc_spi_clkwidth_t.
 */
mxc_spi_clkmode_t MXC_SPI_GetClkMode(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the SPI instance's callback function.
 * 
 * @param   spi         Pointer to SPI instance's registers.
 * @param   callback    Pointer to callback function called when transaction is complete.
 * @param   data        Pointer for data to pass through callback funciton.
 *  
 * @return Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetCallback(mxc_spi_regs_t *spi, mxc_spi_callback_t callback, void *data);

/**
 * @brief   Checks the SPI instance for an ongoing transmission
 *
 * This function is applicable in Controller mode only.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Active/Inactive, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_GetActive(mxc_spi_regs_t *spi);

/**
 * @brief   Checks whether the SPI instance is ready for sleep.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Busy/Ready, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_ReadyForSleep(mxc_spi_regs_t *spi);

/**
 * @brief   Starts a SPI Transmission
 *
 * This function is applicable in Master mode only
 *
 * The user must ensure that there are no ongoing transmissions before
 * calling this function
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_StartTransmission(mxc_spi_regs_t *spi);

/**
 * @brief   Aborts an ongoing SPI Transmission
 *
 * This function is applicable in Master mode only
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_AbortTransmission(mxc_spi_regs_t *spi);

/**
 * @brief   Abort any asynchronous requests in progress.
 *
 * Abort any asynchronous requests in progress. Any callbacks associated with
 * the active transaction will be executed to indicate when the transaction
 * has been terminated.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_AbortAsync(mxc_spi_regs_t *spi);

/**
 * @brief   Get the amount of free space available in the transmit FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  The number of bytes available.
 */
unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t *spi);

/**
 * @brief   Get the number of bytes currently available in the receive FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  The number of bytes available.
 */
unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t *spi);

/**
 * @brief   Removes and discards all bytes currently in the transmit FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t *spi);

/**
 * @brief   Removes and discards all bytes currently in the receive FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t *spi);

/**
 * @brief   Set the transmit threshold level.
 *
 * TX FIFO threshold. Smaller values will cause interrupts
 * to occur more often, but reduce the possibility of terminating
 * a transaction early in master mode, or transmitting invalid data
 * in slave mode. Larger values will reduce the time required by
 * the ISR, but increase the possibility errors occurring. Passing
 * an invalid value will cause the driver to use the value already
 * set in the appropriate register.
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   numBytes    The threshold level to set.  This value must be
 *                      between 0 and 8 inclusive.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetTXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes);

/**
 * @brief   Set the receive threshold level.
 *
 * RX FIFO Receive threshold. Smaller values will cause
 * interrupts to occur more often, but reduce the possibility
 * of losing data because of a FIFO overflow. Larger values
 * will reduce the time required by the ISR, but increase the
 * possibility of data loss. Passing an invalid value will
 * cause the driver to use the value already set in the
 * appropriate register.
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   numBytes    The threshold level to set. This value must be
 *                      between 0 and 8 inclusive.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetRXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes);

/**
 * @brief   Get the current transmit threshold level.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  The transmit threshold value (in bytes).
 */
unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t *spi);

/**
 * @brief   Get the current receive threshold level.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  The receive threshold value (in bytes).
 */
unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t *spi);

//////>>> Previous Implementation
/**
 * @brief   Sets the number of bits per character
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 * @param   dataSize    The number of bits per character
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int dataSize);

/**
 * @brief   Gets the number of bits per character
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the SPI width used for transmissions
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 * @param   spiWidth    SPI Width (3-Wire, Standard, Dual SPI, Quad SPI)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_width_t spiWidth);

/**
 * @brief   Gets the SPI width used for transmissions
 *
 * @param   spi         Pointer to SPI registers (selects the SPI block used).
 *
 * @return  Spi Width
 */
mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the slave select (SS) line used for transmissions
 *
 * This function is applicable in Master mode only
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   ssIdx       Slave select index
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetSlave(mxc_spi_regs_t *spi, int ssIdx);

/**
 * @brief   Gets the slave select (SS) line used for transmissions
 *
 * This function is applicable in Master mode only
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  slave slect
 */
int MXC_SPI_GetSlave(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the spi mode using clock polarity and clock phase
 * 
 * @param   spi           Pointer to SPI registers (selects the SPI block used).
 * @param   spiMode       \ref mxc_spi_mode_t
 *  
 * @return Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode);

/**
 * @brief   Gets the spi mode
 * 
 * @param   spi           Pointer to SPI registers (selects the SPI block used).
 * 
 * @return mxc_spi_mode_t   \ref mxc_spi_mode_t
 */
mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi);

/**
 * @brief   Loads bytes into the transmit FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   bytes       The buffer containing the bytes to write
 * @param   len         The number of bytes to write.
 *
 * @return  The number of bytes actually written.
 */
unsigned int MXC_SPI_WriteTXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len);

/**
 * @brief   Unloads bytes from the receive FIFO.
 *
 * @param   spi         Pointer to SPI instance's registers.
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 *
 * @return  The number of bytes actually read.
 */
unsigned int MXC_SPI_ReadRXFIFO(mxc_spi_regs_t *spi, unsigned char *bytes, unsigned int len);

/**
 * @brief   Sets the TX data to transmit as a 'dummy' byte
 *
 * In single wire master mode, this data is transmitted on MOSI when performing
 * an RX (MISO) only transaction. This defaults to 0.
 *
 * @param   spi             Pointer to SPI registers (selects the SPI block used).
 * @param   defaultTXData   Data to shift out in RX-only transactions
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t *spi, unsigned int defaultTXData);
//////<<< Previous Implementation

/* ** DMA Functions ** */

/**
 * @brief   This function initializes the DMA for SPI DMA transactions.
 * 
 * @note    This function must run before any SPI DMA transactions.
 *
 * @param   spi             Pointer to SPI registers (selects the SPI block used).
 * @param   dma             Pointer to DMA registers (selects the DMA block used).
 * @param   use_dma_tx      True/False setting to initialize SPI DMA TX. Acquire DMA TX channel.
 * @param   use_dma_rx      True/False setting to initialize SPI DMA RX. Acquire DMA RX channel.
 *                              use_dma_tx must be true even if use_dma_rx is false.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_DMA_Init(mxc_spi_regs_t *spi, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx);

/**
 * @brief   Helper function that checks whether the MXC_SPI_Init function
 *          cfgalized DMA for SPI DMA transactons.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
bool MXC_SPI_DMA_GetInitialized(mxc_spi_regs_t *spi);

/**
 * @brief   Retreive the DMA TX Channel associated with SPI instance.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  If successful, the DMA TX Channel number is returned. Otherwise, see
 *          \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_DMA_GetTXChannel(mxc_spi_regs_t *spi);

/**
 * @brief   Retreive the DMA RX Channel associated with SPI instance.
 *
 * @param   spi         Pointer to SPI instance's registers.
 *
 * @return  If successful, the DMA RX Channel number is returned. Otherwise, see
 *          \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_DMA_GetRXChannel(mxc_spi_regs_t *spi);

/**
 * @brief   Sets the SPI instance's DMA TX/RX request select.
 * 
 * @param   spi         Pointer to SPI instance's registers.
 * @param   use_dma_tx  True/False setting to set SPI DMA TX request select.
 * @param   use_dma_rx  True/False setting to set SPI DMA RX request select.
 *  
 * @return Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_SPI_DMA_SetRequestSelect(mxc_spi_regs_t *spi, bool use_dma_tx, bool use_dma_rx);

/* ** Transaction Functions ** */

/**
 * @brief   Performs a blocking SPI transaction.
 *
 * Performs a blocking SPI transaction.
 * These actions will be performed in Master Mode:
 * 1. Assert the specified SS
 * 2. In Full Duplex Modes, send TX data while receiving RX Data
 *      if rxLen > txLen, pad txData with DefaultTXData
 *      if txLen > rxLen, discard rxData where rxCnt > rxLen
 * 3. In Half Duplex Modes, send TX Data, then receive RX Data
 * 4. Deassert the specified SS
 *
 * These actions will be performed in Slave Mode:
 * 1. Fill FIFO with txData
 * 2. Wait for SS Assert
 * 3. If needed, pad txData with DefaultTXData
 * 4. Unload RX FIFO as needed
 * 5. On SS Deassert, return
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_MasterTransaction(mxc_spi_req_t *req);

/**
 * @brief   Setup an interrupt-driven SPI transaction
 *
 * The TX FIFO will be filled with txData, padded with DefaultTXData if necessary
 * Relevant interrupts will be enabled, and relevant registers set (SS, Width, etc)
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req);

/**
 * @brief   Setup a DMA driven SPI transaction
 *
 * The TX FIFO will be filled with txData, padded with DefaultTXData if necessary
 * Relevant interrupts will be enabled, and relevant registers set (SS, Width, etc)
 *
 * The lowest-indexed unused DMA channel will be acquired (using the DMA API) and
 * set up to load/unload the FIFOs with as few interrupt-based events as
 * possible. The channel will be reset and returned to the system at the end of
 * the transaction.
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req);

/**
 * @brief   Set up a blocking, non-interrupt-driven SPI controller transaction.
 * 
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_ControllerTransaction(mxc_spi_req_t *req);

/**
 * @brief   Set up a non-blocking, interrupt-driven SPI controller transaction.
 * 
 * The MXC_SPI_Handler function must be called in the selected SPI instance's
 * interrupt handler to process the transaction.
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_ControllerTransactionAsync(mxc_spi_req_t *req);

/**
 * @brief   Set up a non-blocking, DMA-driven SPI controller transaction.
 * 
 * The MXC_SPI_Config(...) or MXC_SPI_DMA_Init(..). functions must be
 * called before calling this DMA transaction function. This function
 * does not initialize the DMA.
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_ControllerTransactionDMA(mxc_spi_req_t *req);

/**
 * @brief   Set up a blocking, DMA-driven SPI controller transaction.
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_ControllerTransactionDMAB(mxc_spi_req_t *req);

/**
 * @brief   Performs a blocking SPI transaction.
 *
 * @param   req         Pointer to details of the transaction
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req);

/**
 * @brief   Setup an interrupt-driven SPI transaction
 *
 * @param   req         Pointer to details of the transactionz
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req);

/**
 * @brief   Setup a DMA driven SPI transaction
 *
 * The TX FIFO will be filled with txData, padded with DefaultTXData if necessary
 * Relevant interrupts will be enabled, and relevant registers set (SS, Width, etc)
 *
 * The lowest-indexed unused DMA channel will be acquired (using the DMA API) and
 * set up to load/unload the FIFOs with as few interrupt-based events as
 * possible. The channel will be reset and returned to the system at the end of
 * the transaction.
 *
 * @param   req             Pointer to details of the transaction
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t *req);

/**
 * @brief   Setup a blocking SPI Target transaction.
 * 
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_TargetTransaction(mxc_spi_req_t *req);

/**
 * @brief   Setup an interrupt-driven, non-blocking SPI Target transaction.
 * 
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_TargetTransactionAsync(mxc_spi_req_t *req);

/**
 * @brief   Setup a DMA-driven SPI Target transaction.
 *
 * The MXC_SPI_Config(...) or MXC_SPI_DMA_Init(..). functions must be
 * called before calling this DMA transaction function. This function
 * does not initialize the DMA.
 *
 * @param   req         Pointer to details of the transaction.
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_SPI_TargetTransactionDMA(mxc_spi_req_t *req);

/* ** Handler Functions ** */

/**
 * @brief   The processing function for asynchronous transactions.
 *
 * When using the asynchronous functions, the application must call this
 * function periodically. This can be done from within the SPI interrupt
 * handler or periodically by the application if SPI interrupts are disabled.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_AsyncHandler(mxc_spi_regs_t *spi);

/**
 * @brief   The processing function for asynchronous transactions.
 *
 * When using the asynchronous functions, the application must call this
 * function periodically. This can be done from within the SPI interrupt
 * handler or periodically by the application if SPI interrupts are disabled.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_Handler(mxc_spi_regs_t *spi);

/**
 * @brief   The processing function for DMA TX transactions.
 * 
 * This function calls the callback function if only TX transaction was made.
 * 
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_DMA_TX_Handler(mxc_spi_regs_t *spi);

/**
 * @brief   The processing function for DMA RX transactions.
 * 
 *  This function calls the callback function at the end of a TX/RX transaction.
 *
 * @param   spi         Pointer to SPI instance's registers.
 */
void MXC_SPI_DMA_RX_Handler(mxc_spi_regs_t *spi);

/**@} end of group spi */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32572_SPI_H_
