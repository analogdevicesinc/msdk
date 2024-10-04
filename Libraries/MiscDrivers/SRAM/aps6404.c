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
#include <stdint.h>
#include <string.h> // For memset
#include "aps6404.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "fastspi.h"
#include "tmr.h"

enum MODE { STANDARD_MODE, QUAD_MODE };
typedef enum MODE MODE_t;
MODE_t g_current_mode;

// Utility macro for validating an error code.  Assumes an 'err' variable
// of type int exists in the macro's context
#define ERR_CHECK(x)             \
    {                            \
        err = (x);               \
        if (err != E_NO_ERROR) { \
            return x;            \
        }                        \
    }

inline void _parse_spi_header(uint8_t cmd, uint32_t address, uint8_t *out)
{
    out[0] = cmd;
    out[1] = (address >> 16) & 0xFF; // MSB first
    out[2] = (address >> 8) & 0xFF;
    out[3] = (address & 0xFF);
}

int _transmit_spi_header(uint8_t cmd, uint32_t address)
{
    int err = E_NO_ERROR;
    // SPI reads and writes will always start with 4 bytes.
    // A command byte, then a 24-bit address (MSB first)
    uint8_t header[4];
    _parse_spi_header(cmd, address, header);

    // Transmit header, but keep Chip Select asserted.
    spi_transmit(header, 4, NULL, 0, false);
    return err;
}

int aps6404_init()
{
    int err = E_NO_ERROR;
    ERR_CHECK(spi_init());
    ERR_CHECK(aps6404_reset());
    return err;
}

int aps6404_reset()
{
    int err = E_NO_ERROR;

    ERR_CHECK(aps6404_exit_quadmode()); // Protect against quad-mode lock-up

    uint8_t data[2] = { 0x66, 0x99 };
    ERR_CHECK(spi_transmit(&data[0], 1, NULL, 0, true));
    ERR_CHECK(spi_transmit(&data[1], 1, NULL, 0, true));

    return err;
}

int aps6404_enter_quadmode()
{
    int err = E_NO_ERROR;
    uint8_t tx_data = 0x35;

    ERR_CHECK(MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_STANDARD));
    ERR_CHECK(spi_transmit(&tx_data, 1, NULL, 0, true));
    ERR_CHECK(MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_QUAD));

    g_current_mode = QUAD_MODE;

    return err;
}

int aps6404_exit_quadmode()
{
    int err = E_NO_ERROR;
    uint8_t tx_data = 0xF5;

    ERR_CHECK(MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_QUAD));
    ERR_CHECK(spi_transmit(&tx_data, 1, NULL, 0, true));
    ERR_CHECK(MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_STANDARD));

    g_current_mode = STANDARD_MODE;

    return err;
}

int aps6404_read_id(aps6404_id_t *out)
{
    int err = E_NO_ERROR;
    uint8_t tx_data = 0x9F;
    uint8_t rx_data[12];
    bool back_to_quad_mode = false;

    if (g_current_mode != STANDARD_MODE) {
        // ID fields seem not to support quad mode.
        // If we're currently in quad mode, exit it but re-enable it later
        aps6404_exit_quadmode();
        back_to_quad_mode = true;
    }

    ERR_CHECK(spi_transmit(&tx_data, 1, NULL, 0, false));
    ERR_CHECK(spi_transmit(NULL, 0, rx_data, 12, true));

    out->MFID = rx_data[3];
    out->KGD = rx_data[4];
    out->density = (rx_data[5] & 0xe0) >> 5; // Density is just top 3 bits

    // Formulate 44-bit EID from remaining bytes
    int tmp = rx_data[5] & 0x1F;
    for (int i = 0; i <= 6; i++) {
        tmp = tmp << 8;
        tmp |= rx_data[5 + i];
    }
    out->EID = tmp;

    if (back_to_quad_mode) {
        aps6404_enter_quadmode();
    }

    // Validate against expected values
    if (out->MFID != MFID_EXPECTED)
        return E_INVALID;
    if (out->KGD != KGD_EXPECTED)
        return E_INVALID;
    if (out->density != DENSITY_EXPECTED)
        return E_INVALID;

    return err;
}

int aps6404_read(uint32_t address, uint8_t *out, unsigned int len)
{
    int err = E_NO_ERROR;

    if (g_current_mode == STANDARD_MODE) {
        ERR_CHECK(_transmit_spi_header(0x03, address));
        ERR_CHECK(spi_transmit(NULL, 0, out, len, true));
    } else if (g_current_mode == QUAD_MODE) {
        uint8_t header[7];
        memset(header, 0xFF, 7);
        // ^ Sending dummy bytes with value 0x00 seems to break QSPI reads...  Sending 0xFF works
        _parse_spi_header(0xEB, address, header);
        ERR_CHECK(spi_transmit(&header[0], 7, NULL, 0, false));
        ERR_CHECK(spi_transmit(NULL, 0, out, len, true));
    }
    return err;
}

int aps6404_write(uint32_t address, uint8_t *data, unsigned int len)
{
    int err = E_NO_ERROR;

    if (g_current_mode == STANDARD_MODE) {
        ERR_CHECK(_transmit_spi_header(0x02, address));
        ERR_CHECK(spi_transmit(data, len, NULL, 0, true));
    } else if (g_current_mode == QUAD_MODE) {
        ERR_CHECK(_transmit_spi_header(0x38, address));
        ERR_CHECK(spi_transmit(data, len, NULL, 0, true));
    }
    return err;
}
