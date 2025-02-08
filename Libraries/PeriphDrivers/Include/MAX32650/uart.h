/**
 * @file
 * @brief   This files defines the driver API including definitions, data types
 *          and function prototypes.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_UART_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_UART_H_

/***** Includes *****/
#include <stdbool.h>
#include <stdint.h>
#include "uart_regs.h"
#include "mxc_sys.h"
#include "mxc_errors.h"

#ifdef __cplusplus
extern "C" {
#endif

/***** Definitions *****/

/**
 * @brief      Alternate clock rate. (7.3728MHz) */
#define UART_ALTERNATE_CLOCK_HZ 7372800
/**
 * @defgroup uart UART
 * @ingroup periphlibs
 * @{
 */

/**
 * @brief   The list of UART Parity options supported
 *
 */
typedef enum {
    MXC_UART_PARITY_DISABLE, ///< UART Parity Disabled
    MXC_UART_PARITY_EVEN, ///< UART Parity Even
    MXC_UART_PARITY_ODD, ///< UART Parity Odd
    MXC_UART_PARITY_MARK, ///< UART Parity Mark
    MXC_UART_PARITY_SPACE, ///< UART Parity Space
    MXC_UART_PARITY_EVEN_0, ///< UART Parity Even, 0 based
    MXC_UART_PARITY_EVEN_1, ///< UART Parity Even, 1 based
    MXC_UART_PARITY_ODD_0, ///< UART Parity Odd, 0 based
    MXC_UART_PARITY_ODD_1, ///< UART Parity Odd, 1 based
    MXC_UART_PARITY_MARK_0, ///< UART Parity Mark, 0 based
    MXC_UART_PARITY_MARK_1, ///< UART Parity Mark, 1 based
    MXC_UART_PARITY_SPACE_0, ///< UART Parity Space, 0 based
    MXC_UART_PARITY_SPACE_1, ///< UART Parity Space, 1 based
} mxc_uart_parity_t;

/**
 * @brief      Stop bit settings */
typedef enum {
    MXC_UART_STOP_1, /**< UART Stop 1 clock cycle */
    MXC_UART_STOP_2, /**< UART Stop 2 clock cycle */
} mxc_uart_stop_t;

/**
 * @brief      Flow control */
typedef enum {
    MXC_UART_FLOW_DIS, /**< RTS/CTS flow is disabled */
    MXC_UART_FLOW_EN_LOW, /**< RTS/CTS flow is enabled, active low */
    MXC_UART_FLOW_EN_HIGH, /**< RTS/CTS flow is enabled, active high */
} mxc_uart_flow_t;

typedef struct _mxc_uart_req_t mxc_uart_req_t;

/**
 * @brief   The callback routine used to indicate the transaction has terminated.
 *
 * @param   req         The details of the transaction.
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_uart_complete_cb_t)(mxc_uart_req_t *req, int result);

/**
 * @brief   The callback routine used to indicate the transaction has terminated.
 *
 * @param   req         The details of the transaction.
 * @param   num         The number of characters actually copied
 * @param   result      See \ref MXC_Error_Codes for the list of error codes.
 */
typedef void (*mxc_uart_dma_complete_cb_t)(mxc_uart_req_t *req, int num, int result);

/**
 * @brief      Non-blocking UART transaction request. 
 */
struct _mxc_uart_req_t {
    mxc_uart_regs_t *uart; ///<Point to UART registers
    uint8_t *txData; ///< Buffer containing transmit data. For character sizes
    ///< < 8 bits, pad the MSB of each byte with zeros. For
    ///< character sizes > 8 bits, use two bytes per character
    ///< and pad the MSB of the upper byte with zeros
    uint8_t *rxData; ///< Buffer to store received data For character sizes
    ///< < 8 bits, pad the MSB of each byte with zeros. For
    ///< character sizes > 8 bits, use two bytes per character
    ///< and pad the MSB of the upper byte with zeros
    uint32_t txLen; ///< Number of bytes to be sent from txData
    uint32_t rxLen; ///< Number of bytes to be stored in rxData
    volatile uint32_t txCnt; ///< Number of bytes actually transmitted from txData
    volatile uint32_t rxCnt; ///< Number of bytes stored in rxData

    mxc_uart_complete_cb_t callback; ///< Pointer to function called when transaction is complete
};

/***** Functions Prototypes *****/

/**
 * @brief   Initialize and enable UART peripheral.
 *
 * This function initializes everything necessary to call a UART transaction function.
 * Some parameters are set to defaults as follows:
 * UART Data Size    - 8 bits
 * UART Stop Bits    - 1 bit
 * UART Parity       - None
 * UART Flow Control - None
 * UART Clock        - 7.37MHz Clock (for baud > 7372800, PCLK is used)
 *
 * These parameters can be modified after initialization using low level functions
 *
 * @note    On default this function enables UART peripheral clock.
 *          if you wish to manage clock and gpio related things in upper level instead of here.
 *          Define MSDK_NO_GPIO_CLK_INIT flag in project.mk file. 
 *          By this flag this function will remove clock and gpio related codes from file.
 * 
 * @param   uart            Pointer to UART registers (selects the UART block used.)
 * @param   baud            The requested clock frequency. The actual clock frequency
 *                          will be returned by the function if successful.
 *
 * @return  If successful, the actual clock frequency is returned. Otherwise, see
 *          \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_Init(mxc_uart_regs_t *uart, unsigned int baud);

/**
 * @brief   Shutdown UART module.
 * @param   uart    Pointer to the UART registers.
 * @returns #E_NO_ERROR UART shutdown successfully, @ref MXC_Error_Codes "error" if
 *             unsuccessful.
 */
int MXC_UART_Shutdown(mxc_uart_regs_t *uart);

/**
 * @brief   Checks if the given UART bus can be placed in sleep more.
 *
 * This functions checks to see if there are any on-going UART transactions in
 * progress. If there are transactions in progress, the application should
 * wait until the UART bus is free before entering a low-power state.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  #E_NO_ERROR if ready, and non-zero if busy or error. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_ReadyForSleep(mxc_uart_regs_t *uart);

/**
 * @brief   Set the frequency of the UART interface.
 *
 *
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   baud        The desired baud rate
 *
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_SetFrequency(mxc_uart_regs_t *uart, unsigned int baud);

/**
 * @brief   Get the frequency of the UART interface.
 *
 * This function is applicable in Master mode only
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  The UART baud rate
 */
int MXC_UART_GetFrequency(mxc_uart_regs_t *uart);

/**
 * @brief   Sets the number of bits per character
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   dataSize    The number of bits per character (5-8 bits/character are valid)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetDataSize(mxc_uart_regs_t *uart, int dataSize);

/**
 * @brief   Sets the number of stop bits sent at the end of a character
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   stopBits    The number of stop bits used
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetStopBits(mxc_uart_regs_t *uart, mxc_uart_stop_t stopBits);

/**
 * @brief   Sets the type of parity generation used
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   parity      see \ref mxc_uart_parity_t UART Parity Types for details
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetParity(mxc_uart_regs_t *uart, mxc_uart_parity_t parity);

/**
 * @brief   Sets the flow control used
 *
 * @param   uart           Pointer to UART registers (selects the UART block used.)
 * @param   flowCtrl       see \ref mxc_uart_flow_t UART Flow Control Types for details
 * @param   rtsThreshold   Number of bytes remaining in the RX FIFO when RTS is asserted
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetFlowCtrl(mxc_uart_regs_t *uart, mxc_uart_flow_t flowCtrl, int rtsThreshold);

/**
 * @brief   Sets the clock source for the baud rate generator
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   usePCLK     Non-zero values will use the PCLK as the bit clock instead
 *                      of the default 7.37MHz clock source. The baud rate generator
 *                      will automatically be reconfigured to the closest possible
 *                      baud rate.
 *
 * @return  Actual baud rate if successful, otherwise see \ref MXC_Error_Codes
 *          for a list of return codes.
 */
int MXC_UART_SetClockSource(mxc_uart_regs_t *uart, int usePCLK);

/**
 * @brief   Enables or Disables the built-in null modem
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   nullModem   Non-zero values will enable the null modem function,
 *                      which swaps TXD/RXD and also swaps RTS/CTS, if used.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetNullModem(mxc_uart_regs_t *uart, int nullModem);

/**
 * @brief   Transmits a Break Frame (all bits 0)
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SendBreak(mxc_uart_regs_t *uart);

/**
 * @brief   Checks the UART Peripheral for an ongoing transmission
 *
 * This function is applicable in Master mode only
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  Active/Inactive, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_GetActive(mxc_uart_regs_t *uart);

/**
 * @brief         Abort an ongoing UART transmission.
 *
 * @param   uart  Pointer to UART registers (selects the UART block used.)
 *
 * @return     #E_NO_ERROR if the asynchronous request aborted successfully started, @ref
 *             MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_UART_AbortTransmission(mxc_uart_regs_t *uart);

/**
 * @brief      Read a single byte from the UART.
 * @note       This function will block until a character is available.
 *
 * @param      uart  Pointer to the UART registers.
 * @return     The byte read.
 */
int MXC_UART_ReadCharacter(mxc_uart_regs_t *uart);

/**
 * @brief      Write one byte at a time to the UART.
 * @note       This function will block until the character has been placed in the transmit FIFO.
 *             It may return before the character is actually transmitted.
 *
 * @param      uart  Pointer to the UART registers.
 * @param      data  The byte to write.
 */
int MXC_UART_WriteCharacter(mxc_uart_regs_t *uart, uint8_t data);

/**
 * @brief   Reads the next available character. If no character is available, this function
 *          will return an error.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  The character read, otherwise see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_ReadCharacterRaw(mxc_uart_regs_t *uart);

/**
 * @brief   Writes a character on the UART. If the character cannot be written because the
 *          transmit FIFO is currently full, this function returns an error.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   character         The character to write
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_WriteCharacterRaw(mxc_uart_regs_t *uart, uint8_t character);

/**
 * @brief      Read UART data, <em>blocking</em> until transaction is complete.
 *
 * @param      uart    Pointer to the UART registers.
 * @param      buffer  Pointer to buffer to save the data read.
 * @param      len     Number of bytes to read.
 *
 * @return     Number of bytes read, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_UART_Read(mxc_uart_regs_t *uart, uint8_t *buffer, int *len);

/**
 * @brief      Write UART data. This function blocks until the write transaction
 *             is complete.
 * @param      uart    Pointer to the UART registers.
 * @param      buffer  Pointer to buffer for write data.
 * @param      len     Number of bytes to write.
 * @note       This function will return once data has been put into FIFO, not necessarily
 *             transmitted.
 * @return     Number of bytes written if successful, @ref MXC_Error_Codes "error" if unsuccessful.
 */
int MXC_UART_Write(mxc_uart_regs_t *uart, uint8_t *buffer, int *len);

/**
 * @brief   Unloads bytes from the receive FIFO.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 *
 * @return  The number of bytes actually read.
 */
unsigned int MXC_UART_ReadRXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len);

/**
 * @brief   Unloads bytes from the receive FIFO user DMA for longer reads.
 *
 * @param   uart        Pointer to UART registers (selects the UART block used.)
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 * @param   callback    The function to call when the read is complete
 *
 * @return  See \ref MXC_Error_Codes for a list of return values
 */
int MXC_UART_ReadRXFIFODMA(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len,
                           mxc_uart_dma_complete_cb_t callback);

/**
 * @brief      Returns the number of bytes available to be read from the RX FIFO.
 *
 * @param      uart  Pointer to the UART registers.
 *
 * @return     The number of bytes available to read in the RX FIFO.
 */
unsigned MXC_UART_GetRXFIFOAvailable(mxc_uart_regs_t *uart);

/**
 * @brief   Loads bytes into the transmit FIFO.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   bytes       The buffer containing the bytes to write
 * @param   len         The number of bytes to write.
 *
 * @return  The number of bytes actually written.
 */
unsigned int MXC_UART_WriteTXFIFO(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len);

/**
 * @brief   Loads bytes into the transmit FIFO using DMA for longer writes
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   bytes       The buffer containing the bytes to write
 * @param   len         The number of bytes to write.
 * @param   callback    The function to call when the write is complete
 *
 * @return  See \ref MXC_Error_Codes for a list of return values
 */
int MXC_UART_WriteTXFIFODMA(mxc_uart_regs_t *uart, unsigned char *bytes, unsigned int len,
                            mxc_uart_dma_complete_cb_t callback);

/**
 * @brief      Returns the number of bytes still pending transmission in the UART TX FIFO.
 *
 * @param      uart  Pointer to the UART registers.
 *
 * @return     Number of unused bytes in the TX FIFO.
 */
unsigned MXC_UART_GetTXFIFOAvailable(mxc_uart_regs_t *uart);

/**
 * @brief      Drains/empties and data in the RX FIFO, discarding any bytes not yet consumed.
 *
 * @param      uart  Pointer to the UART registers.
 */
void MXC_UART_ClearRXFIFO(mxc_uart_regs_t *uart);

/**
 * @brief      Drains/empties any data in the TX FIFO, discarding any bytes not yet transmitted.
 *
 * @param      uart  Pointer to the UART registers.
 */
void MXC_UART_ClearTXFIFO(mxc_uart_regs_t *uart);

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
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   numBytes    The threshold level to set. This value must be
 *                      between 0 and 8 inclusive.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetRXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes);

/**
 * @brief   Get the current receive threshold level.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  The receive threshold value (in bytes).
 */
unsigned int MXC_UART_GetRXThreshold(mxc_uart_regs_t *uart);

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
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   numBytes    The threshold level to set.  This value must be
 *                      between 0 and 8 inclusive.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_UART_SetTXThreshold(mxc_uart_regs_t *uart, unsigned int numBytes);

/**
 * @brief   Get the current transmit threshold level.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  The transmit threshold value (in bytes).
 */
unsigned int MXC_UART_GetTXThreshold(mxc_uart_regs_t *uart);

/**
 * @brief      Get the UART interrupt flags.
 *
 * @param      uart  Pointer to the UART registers.
 *
 * @return     Mask of active flags.
 */
unsigned MXC_UART_GetFlags(mxc_uart_regs_t *uart);

/**
 * @brief      Clears the specified interrupt flags.
 *
 * @param      uart   Pointer to the UART registers.
 * @param      flags  Mask of the UART interrupts to clear, see
 *                   @ref UART_INT_FL Register.
 */
void MXC_UART_ClearFlags(mxc_uart_regs_t *uart, unsigned int mask);

/**
 * @brief   Enables specific interrupts
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA)
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   intEn       The interrupts to be enabled
 */
void MXC_UART_EnableInt(mxc_uart_regs_t *uart, unsigned int intEn);

/**
 * @brief   Disables specific interrupts
 *
 * These functions should not be used while using non-blocking Transaction Level
 * functions (Async or DMA)
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * @param   intDis      The interrupts to be disabled
 */
void MXC_UART_DisableInt(mxc_uart_regs_t *uart, unsigned int intDis);

/**
 * @brief   Gets the status flags that are currently set
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 *
 * @return  The status flags
 */
unsigned int MXC_UART_GetStatus(mxc_uart_regs_t *uart);

/**
 * @brief   Performs a blocking UART transaction.
 *
 * Performs a blocking UART transaction as follows.
 *
 * If tx_len is non-zero, transmit TX data
 * Once tx_len has been sent, if rx_len is non-zero, receive data
 *
 * @param   req         Pointer to details of the transaction
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_Transaction(mxc_uart_req_t *req);

/**
 * @brief   Setup an interrupt-driven UART transaction
 *
 * The TX FIFO will be filled with txData if necessary
 * Relevant interrupts will be enabled
 *
 * @param   req         Pointer to details of the transaction
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_TransactionAsync(mxc_uart_req_t *req);

/**
 * @brief   Setup a DMA driven UART transaction
 *
 * The TX FIFO will be filled with txData if necessary
 * Relevant interrupts will be enabled
 *
 * The DMA channel indicated by the request will be set up to load/unload the FIFOs
 * with as few interrupt-based events as possible. The channel will be reset and
 * returned to the system at the end of the transaction.
 *
 * @param   req             Pointer to details of the transaction
 *
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_TransactionDMA(mxc_uart_req_t *req);

/**
 * @brief   The processing function for DMA transactions.
 * 
 * When using the DMA functions, the application must call this
 * function periodically. This can be done from within the DMA Interrupt Handler.
 *
 * @param   ch          DMA channel
 * @param   error       Error status
 */
void MXC_UART_DMACallback(int ch, int error);

/**
 * @brief      Async callback
 *
 * @param      uart    The uart
 * @param[in]  retVal  The ret value
 * 
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_AsyncCallback(mxc_uart_regs_t *uart, int retVal);
int MXC_UART_TxAsyncCallback(mxc_uart_regs_t *uart, int retVal);
int MXC_UART_RxAsyncCallback(mxc_uart_regs_t *uart, int retVal);

/**
 * @brief   stop any async callbacks
 *
 * @param   uart  The uart
 * 
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_AsyncStop(mxc_uart_regs_t *uart);
int MXC_UART_TxAsyncStop(mxc_uart_regs_t *uart);
int MXC_UART_RxAsyncStop(mxc_uart_regs_t *uart);

/**
 * @brief   Abort any asynchronous requests in progress.
 *
 * Abort any asynchronous requests in progress. Any callbacks associated with
 * the active transaction will be executed to indicate when the transaction
 * has been terminated.
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * 
 * @return  See \ref MXC_Error_Codes for the list of error return codes.
 */
int MXC_UART_AbortAsync(mxc_uart_regs_t *uart);
int MXC_UART_TxAbortAsync(mxc_uart_regs_t *uart);
int MXC_UART_RxAbortAsync(mxc_uart_regs_t *uart);

/**
 * @brief      UART interrupt handler.
 * @details    This function should be called by the application from the
 *             interrupt handler if UART interrupts are enabled. Alternately,
 *             this function can be periodically called by the application if
 *             UART interrupts are disabled. It is only necessary to call this
 *             when using asynchronous functions.
 *
 * @param      uart  Pointer to the UART registers.
 */
void MXC_UART_AsyncHandler(mxc_uart_regs_t *uart);

/**
 * @brief   Provide TXCount for asynchronous transactions..
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * 
 * @return  Returns transmit bytes (in FIFO).
 */
uint32_t MXC_UART_GetAsyncTXCount(mxc_uart_req_t *req);

/**
 * @brief   Provide RXCount for asynchronous transactions..
 *
 * @param   uart         Pointer to UART registers (selects the UART block used.)
 * 
 * @return  Returns receive bytes (in FIFO).
 */
uint32_t MXC_UART_GetAsyncRXCount(mxc_uart_req_t *req);

/**
 * @brief Enable or disable automatic DMA interrupt handlers for the UART module.
 * 
 * The @ref MXC_UART_TransactionDMA functions require special interrupt handlers to work.
 *  
 * When "Auto" DMA handlers are enabled, the UART drivers will acquire DMA channels
 * and assign the appropriate handlers automatically.  The acquired channels are
 * released after each transaction.
 * 
 * If "Auto" DMA handlers are disabled, the user must acquire DMA channels manually
 * and assign them to the drivers with the @ref MXC_UART_SetTXDMAChannel and
 * @ref MXC_UART_SetRXDMAChannel functions.
 *
 * @param uart Pointer to the UART module's registers.
 * @param enable true to enable Auto DMA handlers, false to disable.
 * @return 0 on success, or a non-zero error code on failure.
 */
int MXC_UART_SetAutoDMAHandlers(mxc_uart_regs_t *uart, bool enable);

/**
 * @brief Set the TX (Transmit) DMA channel for a UART module.
 *
 * This function assigns the DMA channel for transmitting data
 * when @ref is MXC_UART_SetAutoDMAHandlers disabled.
 *
 * @param uart Pointer to the UART module's registers.
 * @param channel The DMA channel number to be used for @ref MXC_UART_TransactionDMA.
 */
int MXC_UART_SetTXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel);

/**
 * @brief Get the TX (Transmit) DMA channel for a UART module.
 *
 * This function retrieves the currently assigned DMA channel for transmitting data
 * when @ref is MXC_UART_SetAutoDMAHandlers disabled.
 *
 * @param uart Pointer to the UART module's registers.
 * @return The currently assigned TX DMA channel.
 */
int MXC_UART_GetTXDMAChannel(mxc_uart_regs_t *uart);

/**
 * @brief Set the RX (Receive) DMA channel for a UART module.
 *
 * This function assigns the DMA channel for receiving data
 * when @ref is MXC_UART_SetAutoDMAHandlers disabled.
 *
 * @param uart Pointer to the UART module's registers.
 * @param channel The DMA channel number to be used for @ref MXC_UART_TransactionDMA.
 */
int MXC_UART_SetRXDMAChannel(mxc_uart_regs_t *uart, unsigned int channel);

/**
 * @brief Get the RX (Receive) DMA channel for a UART module.
 *
 * This function retrieves the currently configured DMA channel for receiving data
 * when @ref is MXC_UART_SetAutoDMAHandlers disabled.
 *
 * @param uart Pointer to the UART module's registers.
 * @return The currently configured RX DMA channel.
 */
int MXC_UART_GetRXDMAChannel(mxc_uart_regs_t *uart);

/**@} end of group uart */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_UART_H_
