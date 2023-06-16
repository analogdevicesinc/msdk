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
#include <string.h> // For memset
#include "N01S830HA.h"
#include "mxc_errors.h"
#include "mxc_delay.h"
#include "fastspi.h"
#include "tmr.h"
#include <stdio.h>

enum MODE { STANDARD_MODE, QUAD_MODE };
typedef enum MODE MODE_t;
MODE_t g_current_mode;

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
    spi_transmit(header, 4, NULL, 0, false, true, true);
    return err;
}

int ram_init()
{    
    int err = E_NO_ERROR;
    err = spi_init();
    if (err)
        return err;

    err = ram_exit_quadmode(); // Protect against quad-mode lock-up
    if (err)
        return err;

    // Set the hold pin to the HIGH state.
    // This is required for normal SPI operations to work reliably
    MXC_GPIO_Config(&hold_pin);
    MXC_GPIO_OutSet(hold_pin.port, hold_pin.mask);

    err = ram_write_mode_reg(0x01); // Disable hold function
    if (err)
        return err;

    // Validate that we were able to write to the mode register
    // This is the closest thing we have to a "read id" or 
    // communication verification for this SRAM chip.
    uint8_t mode_reg;
    err = ram_read_mode_reg(&mode_reg);
    if (err)
        return err;
    
    if (mode_reg != 0x01) {
        return E_NO_DEVICE;
    }

    // Re-configure SPI pins now that the hold function has been disabled.
    err = MXC_GPIO_Config(&spi_pins);
    return err;
}

int ram_enter_quadmode()
{
    int err = E_NO_ERROR;
    uint8_t tx_data = CMD_ENABLE_QUAD_IO;

    MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
    spi_transmit(&tx_data, 1, NULL, 0, true, true, true);
    MXC_SPI_SetWidth(SPI, SPI_WIDTH_QUAD);

    g_current_mode = QUAD_MODE;

    return err;
}

int ram_exit_quadmode()
{
    int err = E_NO_ERROR;
    uint8_t tx_data = CMD_RESET_IO;

    MXC_SPI_SetWidth(SPI, SPI_WIDTH_QUAD);
    spi_transmit(&tx_data, 1, NULL, 0, true, true, true);
    MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);    

    g_current_mode = STANDARD_MODE;

    return err;
}

int ram_read(uint32_t address, uint8_t *out, unsigned int len)
{
    if (g_current_mode == STANDARD_MODE) {
        _transmit_spi_header(CMD_READ, address);
    } else if (g_current_mode == QUAD_MODE) {
        // QUAD mode requires 1 dummy byte.
        uint8_t header[5]; // (1 byte cmd + 3 byte address + 1 dummy)
        memset(header, 0xFF, 5);
        _parse_spi_header(CMD_READ, address, header);
        spi_transmit(header, 5, NULL, 0, false, true, true);
    }

    return spi_transmit(NULL, 0, out, len, true, true, true);
}

int ram_write(uint32_t address, uint8_t *data, unsigned int len)
{
    _transmit_spi_header(CMD_WRITE, address);
    return spi_transmit(data, len, NULL, 0, true, true, true);
}

int ram_write_mode_reg(uint8_t val)
{
    uint8_t data[2] = { CMD_WRITE_MODE_REG, val };
    return spi_transmit(data, 2, NULL, 0, true, true, true);
}

int ram_read_mode_reg(uint8_t* out)
{
    int err = E_NO_ERROR;
    uint8_t cmd = CMD_READ_MODE_REG;
    err = spi_transmit(&cmd, 1, NULL, 0, false, true, true);
    if (err)
        return err;
    return spi_transmit(NULL, 0, out, 1, true, true, true);
}
