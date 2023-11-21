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
#ifndef EXAMPLES_MAX78002_QSPI_FASTSPI_H_
#define EXAMPLES_MAX78002_QSPI_FASTSPI_H_

/**
 * @file    fastspi.h
 * @brief   "fast" and optimized DMA SPI drivers for use alongside external SRAM drivers
 */

#include "fastspi_config.h"
// NOTE: "fastspi_config.h should be implemented for each Board Support Package and placed in
//       its include folder in Libraries/Boards

#ifndef SPI
#error Missing fastspi_config.h definition 'SPI' to select SPI instance
#endif

#ifndef SPI_SPEED
#error Missing fastspi_config.h definition 'SPI_SPEED' to set SPI clock frequency
#endif

// These pin definitions should also be provided in the "fastspi_config.h" file
extern const mxc_gpio_cfg_t fastspi_ss_pin;
extern const mxc_gpio_cfg_t fastspi_spi_pins;

/**
 * @brief Initializes the SPI (Serial Peripheral Interface) module.
 *
 * This function initializes the SPI module with default settings. It must be
 * called before any SPI transactions are performed.
 *
 * @return An integer status code. 0 indicates success, while a non-zero value
 *         indicates an error during initialization.
 */
extern int spi_init();

/**
 * @brief Transmits and receives data using the SPI module.
 *
 * This function performs a full-duplex SPI transaction. It transmits data from
 * the source buffer and receives data into the destination buffer. The lengths
 * of the transmit and receive buffers are specified by txlen and rxlen
 * parameters, respectively.
 *
 * @param[in]  src       Pointer to the source buffer containing data to be transmitted.
 * @param[in]  txlen     Length of the data to be transmitted, in bytes.
 * @param[out] dest      Pointer to the destination buffer to store received data.
 * @param[in]  rxlen     Length of the data to be received, in bytes.
 * @param[in]  deassert  Boolean indicating whether to deassert the CS (Chip Select) line
 *                      after the transaction (true) or keep it asserted (false).
 *
 * @return An integer status code. 0 indicates success, while a non-zero value
 *         indicates an error during the SPI transaction.
 */
extern int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert);

/**
 * @brief Exits the quad mode for SPI (Serial Peripheral Interface) communication.
 *
 * This function is used to exit the quad mode in SPI, if previously enabled.
 * Quad mode typically allows for faster data transfer rates by utilizing
 * multiple data lines for both input and output.
 *
 * @return An integer status code. 0 indicates success, while a non-zero value
 *         indicates an error during the quad mode exit process.
 */
extern int spi_exit_quadmode();

/**
 * @brief Enters the quad mode for SPI (Serial Peripheral Interface) communication.
 *
 * This function is used to enter the quad mode in SPI, enabling the use of
 * multiple data lines for both input and output, which can result in faster
 * data transfer rates.
 *
 * @return An integer status code. 0 indicates success, while a non-zero value
 *         indicates an error during the quad mode entry process.
 */
extern int spi_enter_quadmode();

#endif // EXAMPLES_MAX78002_QSPI_FASTSPI_H_
