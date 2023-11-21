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
#include <stdint.h>
#include <stdio.h>
#include <string.h> // For memset
#include "N01S830HA.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "tmr.h"

enum MODE { STANDARD_MODE, QUAD_MODE };
typedef enum MODE MODE_t;
MODE_t g_current_mode;

// =============================================================================
// GLOBALS

// global error variable
int _g_err = E_NO_ERROR;
// Error checking macro that can be used inside functions with an 'int' return
// type
#define ERR_CHECK(x)                    \
    if ((_g_err = (x)) != E_NO_ERROR) { \
        return (_g_err);                \
    }

// =============================================================================

inline void _parse_spi_header(uint8_t cmd, uint32_t address, uint8_t *out)
{
    out[0] = cmd;
    out[1] = (address >> 16) & 0xFF; // MSB first
    out[2] = (address >> 8) & 0xFF;
    out[3] = (address & 0xFF);
}

inline int _transmit_spi_header(uint8_t cmd, uint32_t address)
{
    // SPI reads and writes will always start with 4 bytes.
    // A command byte, then a 24-bit address (MSB first)
    uint8_t header[4];
    _parse_spi_header(cmd, address, header);

    // Transmit header, but keep Chip Select asserted.
    ERR_CHECK(spi_transmit(header, 4, NULL, 0, false));
    return E_NO_ERROR;
}

int N01S830HA_init()
{
    ERR_CHECK(spi_init());
    ERR_CHECK(N01S830HA_exit_quadmode()); // Protect against quad-mode lock-up

    // The first thing we need to do is disable the HOLD function, which
    // is enabled by default.  There is a hardware (hold pin) and software
    // (hold bit) component to this.

    // Set the hold pin to the HIGH state.
    ERR_CHECK(MXC_GPIO_Config(&N01S830HA_hold_pin));
    MXC_GPIO_OutSet(N01S830HA_hold_pin.port, N01S830HA_hold_pin.mask);

    ERR_CHECK(N01S830HA_write_mode_reg(0b1)); // Disable hold function

    // Now, validate that we were able to write to the mode register
    // This is the closest thing we have to a "read id" or
    // communication verification for this SRAM chip.
    uint8_t mode_reg;
    ERR_CHECK(N01S830HA_read_mode_reg(&mode_reg));
    if (mode_reg != 0x01) {
        return E_NO_DEVICE;
    }

    // Set burst mode (mode reg bits [7:6])
    // [7:6] = 0b00 -> word mode
    // [7:6] = 0b10 -> page mode
    // [7:6] = 0b01 -> burst mode (*)
    // [7:6] = 0b11 - > reserved

    // Burst mode allows writing across the 8-bit word and
    // 32-word page boundaries by automatically incrementing and
    // wrapping the address pointer.
    mode_reg &= ~(0b11 << 6);
    mode_reg |= (0b01 << 6);
    ERR_CHECK(N01S830HA_write_mode_reg(mode_reg));

    // Validate the mode reg is updated successfully.
    uint8_t validate = 0;
    ERR_CHECK(N01S830HA_read_mode_reg(&validate));
    if (validate != mode_reg) {
        printf("Expected %i, received %i\n", mode_reg, validate);
        return E_COMM_ERR;
    }

    // Re-configure SPI pins now that the hold function has been disabled.
    // This allows us to use QSPI.
    ERR_CHECK(spi_init());

    return E_NO_ERROR;
}

int N01S830HA_enter_quadmode()
{
    uint8_t tx_data = CMD_ENABLE_QUAD_IO;

    ERR_CHECK(spi_exit_quadmode());
    ERR_CHECK(spi_transmit(&tx_data, 1, NULL, 0, true));
    ERR_CHECK(spi_enter_quadmode());

    g_current_mode = QUAD_MODE;

    return E_NO_ERROR;
}

int N01S830HA_exit_quadmode()
{
    uint8_t tx_data = CMD_RESET_IO;

    ERR_CHECK(spi_enter_quadmode());
    ERR_CHECK(spi_transmit(&tx_data, 1, NULL, 0, true));
    ERR_CHECK(spi_exit_quadmode());

    g_current_mode = STANDARD_MODE;

    return E_NO_ERROR;
}

int N01S830HA_read(uint32_t address, uint8_t *out, unsigned int len)
{
    if (g_current_mode == STANDARD_MODE) {
        _transmit_spi_header(CMD_READ, address);
    } else if (g_current_mode == QUAD_MODE) {
        // QUAD mode requires 1 dummy byte.
        uint8_t header[5]; // (1 byte cmd + 3 byte address + 1 dummy)
        memset(header, 0xFF, 5);
        _parse_spi_header(CMD_READ, address, header);
        ERR_CHECK(spi_transmit(header, 5, NULL, 0, false));
    }

    return spi_transmit(NULL, 0, out, len, true);
}

int N01S830HA_write(uint32_t address, uint8_t *data, unsigned int len)
{
    _transmit_spi_header(CMD_WRITE, address);
    return spi_transmit(data, len, NULL, 0, true);
}

int N01S830HA_write_mode_reg(uint8_t val)
{
    uint8_t data[2] = { CMD_WRITE_MODE_REG, val };
    ERR_CHECK(spi_transmit(data, 2, NULL, 0, true));
    MXC_Delay(MXC_DELAY_USEC(
        100)); // Some small delay after updating the mode reg appears to be necessary.
    return E_NO_ERROR;
}

int N01S830HA_read_mode_reg(uint8_t *out)
{
    uint8_t cmd = CMD_READ_MODE_REG;
    ERR_CHECK(spi_transmit(&cmd, 1, NULL, 0, false));
    return spi_transmit(NULL, 0, out, 1, true);
}
