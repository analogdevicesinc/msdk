/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc. All Rights Reserved. This software
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

/**
 * @file    is25.c
 * @brief   Board layer Driver for the ISSI IS25 Serial Multi-I/O Flash Memory.
*/

/* **** Includes **** */
#include <stdint.h>
#include <stddef.h>
#include "Ext_Flash.h"

/**
 * @ingroup is25
 * @{
 */

/* **** Definitions **** */
#define IS25_ID_LEN (3)

#define IS25_WIP_MASK 0x01 /**< Status Reg-1: Work In Progress          */
#define IS25_WEL_MASK 0x02 /**< Status Reg-1: Write Enable Latch mask   */
#define IS25_QE_MASK 0x40 /**< Status Reg-2: Quad-SPI enable mask      */

#define IS25_DEVICE_SIZE 0x1000000
#define IS25_BLOCK_SIZE 0x10000
#define IS25_PAGE_SIZE 256

#define IS25_CMD_RST_EN 0x66 /**< Reset Enable                   */
#define IS25_CMD_RST_MEM 0x99 /**< Reset Memory                   */
#define IS25_CMD_ID 0x9F /**< ID                             */
#define IS25_CMD_WRITE_EN 0x06 /**< Write Enable                   */
#define IS25_CMD_WRITE_DIS 0x04 /**< Write Disable                  */

#define IS25_CMD_READ_SR 0x05 /**< Read Status Register 1         */
#define IS25_CMD_WRITE_SR 0x01 /**< Write Status Register 1        */

#define IS25_CMD_PPROG 0x02 /**< Page Program                   */
#define IS25_CMD_QUAD_PROG 0X32 /**< Quad (4 x I/O) Page Program    */

#define IS25_CMD_4K_ERASE 0x20 /**< Page Erase                     */
#define IS25_CMD_32K_ERASE 0x52 /**< Sector Type 2 (32KB) Erase     */
#define IS25_CMD_64K_ERASE 0xD8 /**< Sector Type 3 (64KB) Erase     */
#define IS25_CMD_BULK_ERASE 0xC7 /**< Bulk Erase                     */

/* **** Globals **** */

static Ext_Flash_Config_t g_cfg;
static uint8_t g_is_configured = 0;

/* **** Static Functions **** */

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int flash_busy()
{
    uint8_t buf;

    Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1);

    if (buf & IS25_WIP_MASK) {
        return EF_E_BUSY;
    } else {
        return EF_E_SUCCESS;
    }
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int write_enable()
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = IS25_CMD_WRITE_EN;
    uint8_t buf = 0;

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    if ((err = Ext_Flash_Read_SR(&buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    if (buf & IS25_WEL_MASK) {
        return EF_E_SUCCESS;
    }

    return EF_E_BAD_STATE;
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Init(void)
{
    if (!g_is_configured) {
        return EF_E_BAD_STATE;
    }

    return g_cfg.init();
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Reset(void)
{
    int err = EF_E_SUCCESS;
    int busy_count = 0;
    uint8_t cmd = IS25_CMD_RST_EN;

    // Send the Reset command
    if ((err = g_cfg.write(&cmd, 1, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    cmd = IS25_CMD_RST_MEM;
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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
uint32_t Ext_Flash_ID(void)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = IS25_CMD_ID;
    uint8_t id[IS25_ID_LEN];

    // Send the command
    if ((err = g_cfg.write(&cmd, 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    // Read the data
    if ((err = g_cfg.read(id, IS25_ID_LEN, 1, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
        return err;
    }

    return ((uint32_t)(id[2] | (id[1] << 8) | (id[0] << 16)));
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Quad(int enable)
{
    int err = EF_E_SUCCESS;
    uint8_t pre_buf = 0;
    uint8_t post_buf = 0;

    // Enable QSPI mode
    if ((err = Ext_Flash_Read_SR(&pre_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (pre_buf & IS25_QE_MASK) {
            return EF_E_SUCCESS;
        }
        pre_buf |= IS25_QE_MASK;
    } else {
        if (!(pre_buf & IS25_QE_MASK)) {
            return EF_E_SUCCESS;
        }
        pre_buf &= ~IS25_QE_MASK;
    }

    if (write_enable() != EF_E_SUCCESS) {
        return EF_E_BAD_STATE;
    }

    if ((err = Ext_Flash_Write_SR(pre_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if ((err = Ext_Flash_Read_SR(&post_buf, Ext_Flash_StatusReg_1)) != EF_E_SUCCESS) {
        return err;
    }

    while (flash_busy()) {}

    if (enable) {
        if (!(post_buf & IS25_QE_MASK)) {
            return EF_E_ERROR;
        }
    } else {
        if (post_buf & IS25_QE_MASK) {
            return EF_E_ERROR;
        }
    }

    return EF_E_SUCCESS;
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
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
        dummy_bits = EXT_FLASH_READ_DUMMY;
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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Program_Page(uint32_t address, uint8_t *tx_buf, uint32_t tx_len,
                           Ext_Flash_DataLine_t d_line)
{
    int err = EF_E_SUCCESS;
    volatile int timeout = 0;
    uint8_t cmd[4] = { 0 };
    uint32_t len = 0;
    uint32_t next_page = 0;
    uint8_t *pWrite_Data = NULL;

    if (tx_buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    // if flash address is out-of-range
    if ((address >= IS25_DEVICE_SIZE) || ((address + tx_len) > IS25_DEVICE_SIZE)) {
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
            cmd[0] = IS25_CMD_QUAD_PROG;
        } else {
            cmd[0] = IS25_CMD_PPROG;
        }

        if ((err = g_cfg.write(&cmd[0], 1, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }

        // Send the address
        if ((err = g_cfg.write(&cmd[1], 3, 0, Ext_Flash_DataLine_Single)) != EF_E_SUCCESS) {
            return err;
        }

        // calculate the next flash page boundary from our starting address
        next_page = ((address & ~(IS25_PAGE_SIZE - 1)) + IS25_PAGE_SIZE);

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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Bulk_Erase(void)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd = IS25_CMD_BULK_ERASE;
    volatile int timeout = 0;

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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Erase(uint32_t address, Ext_Flash_Erase_t size)
{
    int err = EF_E_SUCCESS;
    uint8_t cmd[4] = { 0 };
    volatile int timeout = 0;

    if (flash_busy()) {
        return EF_E_BUSY;
    }

    while (write_enable()) {
        timeout++;
        if (timeout > 100) {
            return EF_E_BAD_STATE;
        }
    }

    switch (size) {
    case Ext_Flash_Erase_4K:
    default:
        cmd[0] = IS25_CMD_4K_ERASE;
        break;
    case Ext_Flash_Erase_32K:
        cmd[0] = IS25_CMD_32K_ERASE;
        break;
    case Ext_Flash_Erase_64K:
        cmd[0] = IS25_CMD_64K_ERASE;
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
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Read_SR(uint8_t *buf, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd = 0;

    if (buf == NULL) {
        return EF_E_BAD_PARAM;
    }

    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
        cmd = IS25_CMD_READ_SR;
        break;
    default:
        return EF_E_BAD_PARAM;
    }

    return read_reg(cmd, buf);
}

/* ************************************************************************* */
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int Ext_Flash_Write_SR(uint8_t value, Ext_Flash_StatusReg_t reg_num)
{
    uint8_t cmd = 0;

    switch (reg_num) {
    case Ext_Flash_StatusReg_1:
        cmd = IS25_CMD_WRITE_SR;
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
    // not implemented yet
    return EF_E_BAD_PARAM;
}

/* ************************************************************************* */
Ext_Flash_Unblk_t Ext_Flash_GetAvailableFlash(void)
{
    // not implemented yet
    Ext_Flash_Unblk_t free_flash;
    free_flash.start_addr = 0;
    free_flash.end_addr = 0;

    return free_flash;
}
/**@} end of ingroup is25 */
