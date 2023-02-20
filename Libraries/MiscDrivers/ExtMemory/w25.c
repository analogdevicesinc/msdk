/**
 * @file    w25.c
 * @brief   Board layer Driver for the Micron W25 Serial Multi-I/O Flash Memory.
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

/* **** Includes **** */
#include <stdint.h>
#include <stddef.h>
#include "Ext_Flash.h"

/**
 * @ingroup w25
 * @{
 */

/* **** Definitions **** */
#define W25_ID_LEN (3)

#define W25_WIP_MASK 0x01 /**< Status Reg-1: Work In Progress          */
#define W25_WEL_MASK 0x02 /**< Status Reg-1: Write Enable Latch mask   */
#define W25_QE_MASK 0x02 /**< Status Reg-2: Quad-SPI enable mask      */

#define W25_TB_POS 5
#define W25_TB_MASK (1 << W25_TB_POS) /**< Top/Bottom Select mask         */
#define W25_BP_POS 2
#define W25_BP_MASK (0x7 << W25_BP_POS) /**< Block Protect mask             */
#define W25_SR1_FP_MASK \
    (W25_TB_MASK | W25_BP_MASK) /**< Mask of all flash block protect bits in status register 1 */
#define W25_CMP_POS 6
#define W25_CMP_MASK (1 << W25_CMP_POS) /**< Flash protect complement bit mask */

#define W25_GET_BP_IN_FIRST_HALF(pg) \
    ((pg < 4)              ? 1 :     \
     (pg > 3 && pg < 8)    ? 2 :     \
     (pg > 7 && pg < 16)   ? 3 :     \
     (pg > 15 && pg < 32)  ? 4 :     \
     (pg > 31 && pg < 64)  ? 5 :     \
     (pg > 63 && pg < 128) ? 6 :     \
                             -1)
#define W25_GET_BP_IN_SECOND_HALF(pg) \
    ((pg < 192)             ? 5 :     \
     (pg > 191 && pg < 224) ? 4 :     \
     (pg > 223 && pg < 240) ? 3 :     \
     (pg > 239 && pg < 248) ? 2 :     \
     (pg > 247 && pg < 252) ? 1 :     \
                              -1)

#define W25_DEVICE_SIZE 0x1000000
#define W25_BLOCK_SIZE 0x10000
#define W25_PAGE_SIZE 256

#define W25_CMD_RST_EN 0x66 /**< Reset Enable                   */
#define W25_CMD_RST_MEM 0x99 /**< Reset Memory                   */
#define W25_CMD_ID 0x9F /**< ID                             */
#define W25_CMD_WRITE_EN 0x06 /**< Write Enable                   */
#define W25_CMD_WRITE_DIS 0x04 /**< Write Disable                  */

#define W25_CMD_READ_SR1 0x05 /**< Read Status Register 1         */
#define W25_CMD_WRITE_SR1 0x01 /**< Write Status Register 1        */

#define W25_CMD_READ_SR2 0x35 /**< Read Status Register 2         */
#define W25_CMD_WRITE_SR2 0x31 /**< Write Status Register 2        */

#define W25_CMD_READ_SR3 0x15 /**< Read Status Register 3         */
#define W25_CMD_WRITE_SR3 0x11 /**< Write Status Register 3        */

#define W25_CMD_PPROG 0x02 /**< Page Program                   */
#define W25_CMD_QUAD_PROG 0X32 /**< Quad (4 x I/O) Page Program    */

#define W25_CMD_4K_ERASE 0x20 /**< Page Erase                     */
#define W25_CMD_32K_ERASE 0x52 /**< Sector Type 2 (32KB) Erase     */
#define W25_CMD_64K_ERASE 0xD8 /**< Sector Type 3 (64KB) Erase     */
#define W25_CMD_BULK_ERASE 0xC7 /**< Bulk Erase                     */

/* **** Globals **** */

static Ext_Flash_Config_t g_cfg;
static uint8_t g_is_configured = 0;

/* **** Static Functions **** */

/* ************************************************************************* */
static int flash_busy()
{
    uint8_t buf;

    Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1);

    if (buf & W25_WIP_MASK) {
        return EF_E_BUSY;
    } else {
        return EF_E_SUCCESS;
    }
}

/* ************************************************************************* */
static int write_enable()
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = W25_CMD_WRITE_EN;
    uint8_t buf = 0;

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    if ((err = Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    if (buf & W25_WEL_MASK) {
        return EF_E_SUCCESS;
    }

    return EF_E_BAD_STATE;
}

/* ************************************************************************* */
static int inline read_reg(uint8_t cmd, uint8_t *buf)
{
    int err = EF_E_SUCCESS;

    if (!buf) {
        return EF_E_BAD_PARAM;
    }

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(buf, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
static int inline write_reg(uint8_t *buf, unsigned len)
{
    int err = EF_E_SUCCESS;

    if (!buf || (len == 0)) {
        return EF_E_BAD_PARAM;
    }

    if ((err = write_enable()) != EF_E_SUCCESS) {
        return err;
    }

    // Send the command and data
    if ((err = g_cfg.write(buf, len, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* **** Functions **** */

int Ext_Flash_Configure(Ext_Flash_Config_t *cfg)
{
    int err = EF_E_SUCCESS;

    if (cfg == NULL) {
        return EF_E_BAD_PARAM;
    }

    g_cfg = *cfg;
    g_is_configured = 1;

    return err;
}

/* ************************************************************************* */

int Ext_Flash_Init(void)
{
    if (!g_is_configured) {
        return EF_E_BAD_STATE;
    }

    return g_cfg.init();
}

/* ************************************************************************* */
int Ext_Flash_Reset(void)
{
    int err = EF_E_SUCCESS;
    int busy_count = 0;
    uint8_t cmd = W25_CMD_RST_EN;

    // Send the Reset command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    cmd = W25_CMD_RST_MEM;
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {
        busy_count++;
        if (busy_count > 20000) {
            return EF_E_TIME_OUT;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
uint32_t Ext_Flash_ID(void)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = W25_CMD_ID;
    uint8_t id[W25_ID_LEN] = { 0 };

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(id, W25_ID_LEN, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return ((uint32_t)(id[2] | (id[1] << 8) | (id[0] << 16)));
}

/* ************************************************************************* */
int Ext_Flash_Quad(int enable)
{
    int err = EF_E_SUCCESS;
    uint8_t pre_buf = 0;
    uint8_t post_buf = 0;

    // Enable QSPI mode
    if ((err = Ext_Flash_Read_SR(&pre_buf, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (pre_buf & W25_QE_MASK) {
            return EF_E_SUCCESS;
        }
        pre_buf |= W25_QE_MASK;
    } else {
        if (!(pre_buf & W25_QE_MASK)) {
            return EF_E_SUCCESS;
        }
        pre_buf &= ~W25_QE_MASK;
    }

    if (write_enable() != EF_E_SUCCESS) {
        return EF_E_BAD_STATE;
    }

    if ((err = Ext_Flash_Write_SR(pre_buf, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if ((err = Ext_Flash_Read_SR(&post_buf, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (!(post_buf & W25_QE_MASK)) {
            return EF_E_ERROR;
        }
    } else {
        if (post_buf & W25_QE_MASK) {
            return EF_E_ERROR;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
int Ext_Flash_Read(uint32_t address, uint8_t *rx_buf, uint32_t rx_len, Ext_Flash_DataLine_t d_line)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd[4] = { 0 };
    uint8_t dummy_bits = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Select approriate command for the desired read mode
    if (d_line == Ext_Flash_DataLine_Single) {
        cmd[0] = EXT_FLASH_CMD_READ;
        dummy_bits = EXT_FLASH_Read_DUMMY;
    } else if (d_line == Ext_Flash_DataLine_Dual) {
        cmd[0] = EXT_FLASH_CMD_DREAD;
        dummy_bits = EXT_FLASH_DREAD_DUMMY;
    } else {
        cmd[0] = EXT_FLASH_CMD_QREAD;
        dummy_bits = EXT_FLASH_QREAD_DUMMY;
    }

    // Send command
    if ((err = g_cfg.write(&cmd[0], 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Send starting address of the read
    if ((err = g_cfg.write(&cmd[1], 3, 0, d_line)) != EF_E_SUCCESS) {
        return err;
    }

    // Send dummy bits
    g_cfg.clock(dummy_bits, 0);

    // Receive the data
    if ((err = g_cfg.read(rx_buf, rx_len, 1, d_line)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
int Ext_Flash_Program_Page(uint32_t address, uint8_t *tx_buf, uint32_t tx_len,
                           Ext_Flash_DataLine_t d_line)
{
    int err = EF_E_SUCCESS;
    int timeout = 0;
    uint8_t cmd[4] = { 0 };
    uint32_t len = 0;
    uint32_t next_page = 0;
    uint8_t *pWrite_Data = NULL;

    if (tx_buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    // if flash address is out-of-range
    if ((address >= W25_DEVICE_SIZE) || ((address + tx_len) >= W25_DEVICE_SIZE)) {
        return EF_E_BAD_PARAM; // attempt to write outside flash memory size
    }

    // Device only supports page program in Standard and Quad modes
    if (d_line == Ext_Flash_DataLine_Dual) {
        return EF_E_ERROR;
    }

    pWrite_Data = tx_buf; // note our starting source data address

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    // Now write out as many pages of flash as required to fulfil the request
    while (tx_len > 0) {
        while (write_enable()) {
            timeout++;
            if (timeout > 100) {
                return EF_E_TIME_OUT;
            }
        }

        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >> 8) & 0xFF;
        cmd[3] = address & 0xFF;

        // Send the command and dummy bits
        if (d_line == Ext_Flash_DataLine_Quad) {
            cmd[0] = W25_CMD_QUAD_PROG;
        } else {
            cmd[0] = W25_CMD_PPROG;
        }

        if ((err = g_cfg.write(&cmd[0], 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }

        // Send the address
        if ((err = g_cfg.write(&cmd[1], 3, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }

        // calculate the next flash page boundary from our starting address
        next_page = ((address & ~(W25_PAGE_SIZE - 1)) + W25_PAGE_SIZE);

        // Now check for how much data to write on this page of flash
        if ((address + tx_len) < next_page) {
            len = tx_len; // no page boundary is crossed
        } else {
            len = next_page - address; // adjust length of this write to say within the current page
        }

        // Write the data
        if ((err = g_cfg.write(pWrite_Data, len, 1, d_line)) != EF_E_SUCCESS) {
            return err;
        }

        if (tx_len >= len) {
            tx_len -= len; // what's left to write
        }

        // if there is more to write
        if (tx_len > 0) {
            address += len; // calculate new starting flash_address
            pWrite_Data += len; // and source data address
        }

        timeout = 0;
        while (flash_busy()) {
            timeout++;
            if (timeout > 10000) {
                return EF_E_TIME_OUT;
            }
        }
    }
    return EF_E_SUCCESS;
}

/* ************************************************************************* */
int Ext_Flash_Bulk_Erase(void)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = W25_CMD_BULK_ERASE;
    int timeout = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    if (write_enable() != 0) {
        return EF_E_BAD_STATE;
    }

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {
        timeout++;
        if (timeout > 100000000) {
            return EF_E_TIME_OUT;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
int Ext_Flash_Erase(uint32_t address, Ext_Flash_Erase_t size)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd[4] = { 0 };
    int timeout = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    if (write_enable() != 0) {
        return EF_E_BAD_STATE;
    }

    switch (size) {
    case Ext_Flash_Erase_4K:
    default:
        cmd[0] = W25_CMD_4K_ERASE;
        break;
    case Ext_Flash_Erase_32K:
        cmd[0] = W25_CMD_32K_ERASE;
        break;
    case Ext_Flash_Erase_64K:
        cmd[0] = W25_CMD_64K_ERASE;
        break;
    }

    cmd[1] = (address >> 16) & 0xFF;
    cmd[2] = (address >> 8) & 0xFF;
    cmd[3] = address & 0xFF;

    // Send the command and the address
    if ((err = g_cfg.write(&cmd[0], 4, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {
        timeout++;
        if (timeout > 1000000000) {
            return EF_E_TIME_OUT;
        }
    }
    return EF_E_SUCCESS;
}

/* ************************************************************************* */
int Ext_Flash_Read_SR(uint8_t *buf, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd = 0;

    if (buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
        cmd = W25_CMD_READ_SR1;
        break;
    case Ext_Flash_StatusReg_2:
        cmd = W25_CMD_READ_SR2;
        break;
    case Ext_Flash_StatusReg_3:
        cmd = W25_CMD_READ_SR3;
        break;
    default:
        return EF_E_BAD_PARAM;
    }

    return read_reg(cmd, buf);
}

/* ************************************************************************* */
int Ext_Flash_Write_SR(uint8_t value, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd = 0;

    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
        cmd = W25_CMD_WRITE_SR1;
        break;
    case Ext_Flash_StatusReg_2:
        cmd = W25_CMD_WRITE_SR2;
        break;
    case Ext_Flash_StatusReg_3:
        cmd = W25_CMD_WRITE_SR3;
        break;
    default:
        return EF_E_BAD_PARAM;
    }

    uint8_t cmd_seq[2] = { cmd, value };

    return write_reg(cmd_seq, 2);
}

/* ************************************************************************* */
int Ext_Flash_Block_WP(uint32_t addr, uint32_t begin)
{
    int err = EF_E_SUCCESS;
    uint8_t sr1, sr2, page_num, bp, cmp;

    if (addr >= W25_DEVICE_SIZE) { // Check address valid
        return EF_E_ERROR;
    }

    page_num = addr / W25_BLOCK_SIZE; // Get page number in which "addr" is located
    if ((begin && page_num > 251) || (!begin && page_num < 4)) { // Entire memory array to be locked
        begin = 0;
        cmp = 1;
        bp = 0;
    } else if ((begin && addr == 0) ||
               (!begin && addr == W25_DEVICE_SIZE)) { // Entire memory array to be unlocked
        begin = 0;
        cmp = 0;
        bp = 0;
    } else if ((begin && page_num < 128) ||
               (!begin &&
                page_num > 127)) { // If address in the same half of flash as the starting point
        cmp = 0;

        if (!begin) {
            page_num = 255 - page_num;
        }

        bp = W25_GET_BP_IN_FIRST_HALF(page_num);
    } else { // If address in the opposite half of flash as the starting point
        cmp = 1;

        if (!begin) {
            page_num = 255 - page_num;
        }

        bp = W25_GET_BP_IN_SECOND_HALF(page_num);
    }

    if ((err = Ext_Flash_Read_SR(&sr1, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Read current value of flash protect bits
        return err;
    }
    if ((err = Ext_Flash_Read_SR(&sr2, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        return err;
    }

    sr1 = (sr1 & ~W25_SR1_FP_MASK) | (!!begin << W25_TB_POS) |
          (bp << W25_BP_POS); // Modify flash protect bits
    sr2 = (sr2 & ~W25_CMP_MASK) | (cmp << W25_CMP_POS);

    if ((err = Ext_Flash_Write_SR(sr1, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Write flash protect settings back to W25
        return err;
    }
    if ((err = Ext_Flash_Write_SR(sr2, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        return err;
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
Ext_Flash_Unblk_t Ext_Flash_GetAvailableFlash(void)
{
    int err = 0;
    uint8_t sr1, sr2;
    uint32_t page_addr;
    Ext_Flash_Unblk_t free_flash;

    if ((err = Ext_Flash_Read_SR(&sr1, Ext_Flash_StatusReg_1)) !=
        EF_E_SUCCESS) { // Get current value of flash protect bits
        free_flash.start_addr = err;
        free_flash.end_addr = err;
        return (free_flash);
    }
    if ((err = Ext_Flash_Read_SR(&sr2, Ext_Flash_StatusReg_2)) != EF_E_SUCCESS) {
        free_flash.start_addr = err;
        free_flash.end_addr = err;
        return (free_flash);
    }

    // Use CMP, TB, and BP bits to find start and end addresses
    if (((sr1 & W25_BP_MASK) >> W25_BP_POS) == 0) { // Special case: block protect == 0
        if (sr2 & W25_CMP_MASK) {
            free_flash.start_addr = 0; // CMP == 1 ---> All mem protected
            free_flash.end_addr = 0;
        } else {
            free_flash.start_addr = 0; // CMP == 0 ---> All mem un-protected
            free_flash.end_addr = W25_DEVICE_SIZE;
        }
    } else if (((sr1 & W25_BP_MASK) >> W25_BP_POS) == 7) { // Special case: block protect == 7
        if (sr2 & W25_CMP_MASK) {
            free_flash.start_addr = 0; // CMP == 1 ---> All mem un-protected
            free_flash.end_addr = W25_DEVICE_SIZE;
        } else { // CMP == 0 ---> All mem protected
            free_flash.start_addr = 0;
            free_flash.end_addr = 0;
        }
    } else { // 0 < Block Protect < 7
        page_addr = 4 << (((sr1 & W25_BP_MASK) >> W25_BP_POS) -
                          1); // Find page offset from value of block protect field

        if ((sr1 & W25_TB_MASK) && (sr2 & W25_CMP_MASK)) { // TB == 1 && CMP == 1
            free_flash.start_addr = 0;
            free_flash.end_addr = page_addr * W25_BLOCK_SIZE;
        } else if ((sr1 & W25_TB_MASK) && !(sr2 & W25_CMP_MASK)) { // TB == 1 && CMP == 0
            free_flash.start_addr = page_addr * W25_BLOCK_SIZE;
            free_flash.end_addr = W25_DEVICE_SIZE;
        } else if (!(sr1 & W25_TB_MASK) && (sr2 & W25_CMP_MASK)) { // TB == 0 && CMP == 1
            free_flash.start_addr = 0;
            free_flash.end_addr = (256 - page_addr) * W25_BLOCK_SIZE;
        } else { // TB == 0 && CMP == 0
            free_flash.start_addr = (256 - page_addr) * W25_BLOCK_SIZE;
            free_flash.end_addr = W25_DEVICE_SIZE;
        }
    }

    return free_flash;
}
/**@} end of ingroup w25 */
