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

#ifndef LIBRARIES_MISCDRIVERS_SRAM_N01S830HA_H_
#define LIBRARIES_MISCDRIVERS_SRAM_N01S830HA_H_

#include <stdint.h>
#include <stdbool.h>
#include "N01S830HA_config.h"

#define CMD_READ 0x03
#define CMD_WRITE 0x02
#define CMD_ENABLE_QUAD_IO 0x38
#define CMD_ENABLE_DUAL_IO 0x3B
#define CMD_RESET_IO 0xFF
#define CMD_READ_MODE_REG 0x05
#define CMD_WRITE_MODE_REG 0x01

// =======================================================================================

// EXTERNAL PIN DEFINITIONS
// - Must be supplied externally (!) via N01S830HA_config.h for each BSP

#ifndef N01S830HA_HOLD_PIN_PORT
#error Missing definition 'N01S830HA_HOLD_PIN_PORT'
#endif

#ifndef N01S830HA_HOLD_PIN_MASK
#error Missing definition 'N01S830HA_HOLD_PIN_MASK'
#endif

#ifndef N01S830HA_VSSEL
#error Missing definition 'N01S830HA_VSSEL'
#endif

extern const mxc_gpio_cfg_t N01S830HA_hold_pin;

// =======================================================================================

// SPI TRANSPORT LAYER
// - Must be implemented externally (!)

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

// =======================================================================================

// DRIVER FUNCTIONS

/**
 * @brief Initialize the N01S830HA SRAM.
 *
 * This function initializes the N01S830HA SRAM, setting up necessary configurations
 * for proper operation.  It should be called before any other driver functions.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_init();

/**
 * @brief Enter Quad Mode for the N01S830HA SRAM.
 *
 * This function puts the N01S830HA SRAM into Quad Mode for enhanced data transfer
 * capabilities.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_enter_quadmode();

/**
 * @brief Exit Quad Mode for the N01S830HA SRAM.
 *
 * This function exits Quad Mode for the N01S830HA SRAM, returning to normal operation.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_exit_quadmode();

/**
 * @brief Read data from the N01S830HA SRAM.
 *
 * @param address The 24-bit memory address to read from.
 * @param out Pointer to the buffer where the read data will be stored.
 * @param len Number of bytes to read.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_read(uint32_t address, uint8_t *out, unsigned int len);

/**
 * @brief Write data to the N01S830HA SRAM.
 *
 * This function writes data to the specified address in the N01S830HA SRAM.
 *
 * @param address The 24-bit memory address to write to.
 * @param data Pointer to the buffer containing the data to be written.
 * @param len Number of bytes to write.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_write(uint32_t address, uint8_t *data, unsigned int len);

/**
 * @brief Write to the Mode Register of the N01S830HA SRAM.
 *
 * @param val The value to be written to the Mode Register.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_write_mode_reg(uint8_t val);

/**
 * @brief Read the Mode Register of the N01S830HA SRAM.
 *
 * @param[out] out Pointer to the variable where the read value will be stored.
 *
 * @return 0 on success, or an error code on failure.
 */
int N01S830HA_read_mode_reg(uint8_t *out);

#endif // LIBRARIES_MISCDRIVERS_SRAM_N01S830HA_H_
