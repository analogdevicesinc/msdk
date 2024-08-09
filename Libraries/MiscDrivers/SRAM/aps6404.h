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
#ifndef LIBRARIES_MISCDRIVERS_SRAM_APS6404_H_
#define LIBRARIES_MISCDRIVERS_SRAM_APS6404_H_

#include <stdint.h>
#include <stdbool.h>

#define MFID_EXPECTED 0x0D
#define KGD_EXPECTED 0x5D
#define DENSITY_EXPECTED 0b010

typedef struct {
    uint8_t MFID;
    uint8_t KGD;
    uint8_t density;
    int EID;
} aps6404_id_t;

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
extern int spi_init(void);

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
extern int spi_exit_quadmode(void);

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
extern int spi_enter_quadmode(void);

// =======================================================================================

// DRIVER FUNCTIONS

/**
 * @brief Initialize the APS6404 SRAM.
 *
 * This function initializes the APS6404 SRAM, setting up necessary configurations
 * for proper operation.  It should be called before any other driver functions.
 * 
 * @return 0 on success, or an error code on failure.
 */
int aps6404_init(void);

/**
 * @brief Reset the SRAM.
 *
 * This function issues a software reset sequence to the SRAM.  It is called automatically
 * as the final step of the initialization sequence.  Resetting the SRAM will wipe
 * all memory contents.
 *
 * @return 0 on success, or an error code on failure.
 */
int aps6404_reset(void);

/**
 * @brief Enter Quad Mode.
 *
 * This function puts the APS6404 into Quad SPI Mode for improved data transfer
 * throughput.  It will remain in quad mode until 
 *
 * @return 0 on success, or an error code on failure.
 */
int aps6404_enter_quadmode(void);

/**
 * @brief Exit Quad Mode.
 *
 * This function exits Quad Mode for the APS6404 SRAM, returning to standard SPI operation.
 *
 * @return 0 on success, or an error code on failure.
 */
int aps6404_exit_quadmode(void);

/**
 * @brief Read the ID fields from the APS6404
 *
 * This function reads the manufacturer and other ID fields and validates them against
 * expected values from the datasheet.
 * 
 * @param out Pointer to an output struct.  ID information will be saved into this struct.
 *
 * @return 0 on success, or an error code if the ID fields do not match what is expected.
 */
int aps6404_read_id(aps6404_id_t *out);

/**
 * @brief Read data from the APS6404.
 * 
 * Operations are initialized to standard SPI by default.  To perform QSPI
 * operations call @ref aps6404_enter_quadmode first.
 *
 * @param address The 24-bit memory address to read from.
 * @param out Pointer to the buffer where the read data will be stored.
 * @param len Number of bytes to read.
 *
 * @return 0 on success, or an error code on failure.
 */
int aps6404_read(uint32_t address, uint8_t *out, unsigned int len);

/**
 * @brief Write data to the APS6404.
 * 
 * Operations are initialized to standard SPI by default.  To perform QSPI
 * operations call @ref aps6404_enter_quadmode first.
 *
 * @param address The 24-bit memory address to write to.
 * @param data Pointer to the buffer containing the data to be written.
 * @param len Number of bytes to read.
 *
 * @return 0 on success, or an error code on failure.
 */
int aps6404_write(uint32_t address, uint8_t *data, unsigned int len);

#endif // LIBRARIES_MISCDRIVERS_SRAM_APS6404_H_
