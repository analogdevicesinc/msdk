/**
 * @file    mx25.c
 * @brief   Board layer Driver for the Micron MX25 Serial Multi-I/O Flash Memory.
 */
/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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

/* **** Includes **** */
#include <string.h>
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mx25.h"
// #include "spi.h"
#include "board.h"
#include "spixf.h"

/**
 * @ingroup mx25
 * @{
 */

/* **** Definitions **** */

/* **** Globals **** */

/***** Globals *****/

/* **** Static Functions **** */

/* ************************************************************************* */
static int flash_busy()
{
    uint8_t buf;

    MX25_Read_SR(&buf);

    if (buf & MX25_WIP_MASK) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

/* ************************************************************************* */
static int write_enable()
{
    uint8_t cmd = MX25_CMD_WRITE_EN;
    uint8_t buf;

    // Send the command
    if (MX25_Board_Write(&cmd, 1, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    MX25_Read_SR(&buf);

    if (buf & MX25_WEL_MASK) {
        return E_NO_ERROR;
    }

    return E_BAD_STATE;
}

/* ************************************************************************* */
static int inline read_reg(uint8_t cmd, uint8_t* buf)
{
    // Send the command
    if (MX25_Board_Write(&cmd, 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    // Read the data
    if (MX25_Board_Read(buf, 1, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
static int inline write_reg(uint8_t* buf, unsigned len)
{
    if (write_enable() != 0) {
        return E_BAD_STATE;
    }

    // Send the command and data
    if (MX25_Board_Write(buf, len, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* **** Functions **** */

/* ************************************************************************* */

int MX25_Init(void)
{
    return MX25_Board_Init();
}

/* ************************************************************************* */
int MX25_Reset(void)
{
    int busy_count = 0;

    // Send the Reset command
    uint8_t cmd;
    cmd = MX25_CMD_RST_EN;

    if (MX25_Board_Write(&cmd, 1, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    cmd = MX25_CMD_RST_MEM;

    if (MX25_Board_Write(&cmd, 1, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    while (flash_busy()) {
        busy_count++;

        if (busy_count > 10000) {
            return E_TIME_OUT;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
uint32_t MX25_ID(void)
{
    uint8_t cmd = MX25_CMD_ID;
    uint8_t id[3];

    // Send the command
    if (MX25_Board_Write(&cmd, 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return 0;
    }

    // Read the data
    if (MX25_Board_Read(id, 3, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return 0;
    }

    return ((uint32_t)(id[2] | (id[1] << 8) | (id[0] << 16)));
}

/* ************************************************************************* */
int MX25_Quad(int enable)
{
    // Enable QSPI mode
    uint8_t pre_buf;
    uint8_t post_buf;

    MX25_Read_SR(&pre_buf);

    while (flash_busy()) {
    }

    if (enable) {
        pre_buf |= MX25_QE_MASK;
    } else {
        pre_buf &= ~MX25_QE_MASK;
    }

    if (write_enable() != 0) {
        return E_BAD_STATE;
    }

    MX25_Write_SR(pre_buf);

    while (flash_busy()) {
    }

    MX25_Read_SR(&post_buf);

    while (flash_busy()) {
    }

    if (enable) {
        if (!(post_buf & MX25_QE_MASK)) {
            return E_UNKNOWN;
        }
    } else {
        if (post_buf & MX25_QE_MASK) {
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Write_Protect(int enable)
{
    uint8_t pre_buf;
    uint8_t post_buf;

    MX25_Read_SR(&pre_buf);

    if (enable) {
        pre_buf |= MX25_WP_MASK;
    } else {
        pre_buf &= ~MX25_WP_MASK;
    }

    if (write_enable() != E_NO_ERROR) {
        return E_BAD_STATE;
    }

    MX25_Write_SR(pre_buf);

    while (flash_busy()) {
    }

    MX25_Read_SR(&post_buf);

    if (enable) {
        if (!(post_buf & MX25_WP_MASK)) {
            return E_UNKNOWN;
        }
    } else {
        if (post_buf & MX25_WP_MASK) {
            return E_UNKNOWN;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Read(uint32_t address, uint8_t* rx_buf, uint32_t rx_len, mxc_spixf_width_t width)
{
    uint8_t cmd[4];

    if (flash_busy()) {
        return E_BUSY;
    }

    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Send the command and dummy bits
    if (width == MXC_SPIXF_WIDTH_1) {
        cmd[0] = MX25_CMD_READ;

        if (MX25_Board_Write(&cmd[0], 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send the address
        if (MX25_Board_Write(&cmd[1], 3, 0, width) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send dummy bits
        MX25_Clock(MX25_Read_DUMMY, 0);

    } else if (width == MXC_SPIXF_WIDTH_2) {
        cmd[0] = MX25_CMD_DREAD;

        if (MX25_Board_Write(&cmd[0], 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send the address
        if (MX25_Board_Write(&cmd[1], 3, 0, width) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send dummy bits
        MX25_Clock(MX25_DREAD_DUMMY, 0);

    } else {
        cmd[0] = MX25_CMD_QREAD;

        if (MX25_Board_Write(&cmd[0], 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send the address
        if (MX25_Board_Write(&cmd[1], 3, 0, width) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        // Send dummy bits
        MX25_Clock(MX25_QREAD_DUMMY, 0);
    }

    // Receive the data
    if (MX25_Board_Read(rx_buf, rx_len, 1, width) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Program_Page(uint32_t address, uint8_t* tx_buf, uint32_t tx_len, mxc_spixf_width_t width)
{
    int timeout = 0;
    uint8_t cmd[4];
    unsigned len;
    uint32_t next_page;
    uint8_t* pWrite_Data;

    if (flash_busy()) {
        return E_BUSY;
    }

    // if flash address is out-of-range
    if ((address >= MX25_DEVICE_SIZE) || ((address + tx_len) >= MX25_DEVICE_SIZE)) {
        return E_BAD_PARAM; // attempt to write outside flash memory size
    }

    pWrite_Data = tx_buf; // note our starting source data address

    // Now write out as many pages of flash as required to fulfil the request
    while (tx_len > 0) {
        while (write_enable()) {
            timeout++;

            if (timeout > 100) {
                return E_TIME_OUT;
            }
        }

        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] = address & 0xFF;

        // Send the command and dummy bits
        if (width != MXC_SPIXF_WIDTH_4) {
            cmd[0] = MX25_CMD_PPROG;

            if (MX25_Board_Write(&cmd[0], 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
                return E_COMM_ERR;
            }

            // Send the address
            if (MX25_Board_Write(&cmd[1], 3, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
                return E_COMM_ERR;
            }
        } else {
            cmd[0] = MX25_CMD_QUAD_PROG;

            if (MX25_Board_Write(&cmd[0], 1, 0, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
                return E_COMM_ERR;
            }

            // Send the address
            if (MX25_Board_Write(&cmd[1], 3, 0, width) != E_NO_ERROR) {
                return E_COMM_ERR;
            }
        }

        // calculate the next flash page boundary from our starting address
        next_page = ((address & ~(MX25_PAGE_SIZE - 1)) + MX25_PAGE_SIZE);

        // Now check for how much data to write on this page of flash
        if ((address + tx_len) < next_page)
            len = tx_len; // no page boundary is crossed
        else
            len = next_page - address; // adjust length of this write to say within the current page

        // Write the data
        if (MX25_Board_Write(pWrite_Data, len, 1, width) != E_NO_ERROR) {
            return E_COMM_ERR;
        }

        if (tx_len >= len) {
            tx_len -= len; // what's left to write
        }

        // if there is more to write
        if (tx_len > 0) {
            address += len;     // calculate new starting flash_address
            pWrite_Data += len; // and source data address
        }

        timeout = 0;
        while (flash_busy()) {
            timeout++;

            if (timeout > 10000) {
                return E_TIME_OUT;
            }
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Bulk_Erase(void)
{
    uint8_t cmd;
    int timeout = 0;

    if (flash_busy()) {
        return E_BUSY;
    }

    if (write_enable() != 0) {
        return E_BAD_STATE;
    }

    cmd = MX25_CMD_BULK_ERASE;

    // Send the command
    if (MX25_Board_Write(&cmd, 1, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    while (flash_busy()) {
        timeout++;

        if (timeout > 10000) {
            return E_TIME_OUT;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Erase(uint32_t address, MX25_Erase_t size)
{
    uint8_t cmd[4];
    int timeout = 0;

    if (flash_busy()) {
        return E_BUSY;
    }

    if (write_enable() != 0) {
        return E_BAD_STATE;
    }

    switch (size) {
        case MX25_Erase_4K:
        default:
            cmd[0] = MX25_CMD_4K_ERASE;
            break;

        case MX25_Erase_32K:
            cmd[0] = MX25_CMD_32K_ERASE;
            break;

        case MX25_Erase_64K:
            cmd[0] = MX25_CMD_64K_ERASE;
            break;
    }

    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Send the command and the address
    if (MX25_Board_Write(&cmd[0], 4, 1, MXC_SPIXF_WIDTH_1) != E_NO_ERROR) {
        return E_COMM_ERR;
    }

    while (flash_busy()) {
        timeout++;

        if (timeout > 10000) {
            return E_TIME_OUT;
        }
    }

    return E_NO_ERROR;
}

/* ************************************************************************* */
int MX25_Read_SR(uint8_t* buf)
{
    uint8_t cmd = MX25_CMD_READ_SR;

    return read_reg(cmd, buf);
}

/* ************************************************************************* */
int MX25_Write_SR(uint8_t value)
{
    uint8_t cmd[2] = {MX25_CMD_WRITE_SR, value};

    return write_reg(cmd, 2);
}
/**@} end of ingroup mx25 */
