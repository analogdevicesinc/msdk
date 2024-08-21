/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_SRAM_FASTSPI_H_
#define LIBRARIES_MISCDRIVERS_SRAM_FASTSPI_H_

/**
 * @file    fastspi.h
 * @brief   "fast" and optimized DMA SPI drivers for use alongside external SRAM drivers
 */

#include "fastspi_config.h"
// NOTE: "fastspi_config.h should be implemented for each Board Support Package and placed in
//       its include folder in Libraries/Boards

#ifndef FASTSPI_INSTANCE
#error Missing fastspi_config.h definition 'FASTSPI_INSTANCE' to select SPI instance
#endif

#ifndef FASTSPI_SPEED
#error Missing fastspi_config.h definition 'FASTSPI_SPEED' to set SPI clock frequency
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
int spi_init();

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
int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert);

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
int spi_exit_quadmode();

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
int spi_enter_quadmode();

#endif // LIBRARIES_MISCDRIVERS_SRAM_FASTSPI_H_
