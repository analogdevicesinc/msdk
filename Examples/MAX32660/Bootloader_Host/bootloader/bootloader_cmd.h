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

#ifndef EXAMPLES_MAX32660_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_CMD_H_
#define EXAMPLES_MAX32660_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_CMD_H_

/*******************************      INCLUDES    ****************************/

/*******************************      DEFINES     ****************************/
// Bootloader Return Values
#define BL_RET_SUCCESS 0xAA
#define BL_RET_PARTIAL_ACK 0xAB
#define BL_RET_ERR_UNAVAIL_CMD 0x01
#define BL_RET_ERR_UNAVAIL_FUNC 0x02
#define BL_RET_ERR_ERR_DATA_FORMAT 0x03
#define BL_RET_ERR_INPUT_VALUE 0x04
#define BL_RET_ERR_BTLDR_GENERAL 0x80
#define BL_RET_ERR_BTLDR_CHECKSUM 0x81
#define BL_RET_ERR_BTLDR_AUTH 0x82
#define BL_RET_ERR_BTLDR_INVALID_APP 0x83
#define BL_RET_ERR_BTLDR_APP_NOT_ERASED 0x84
#define BL_RET_ERR_TRY_AGAIN 0xFE
#define BL_RET_ERR_UNKNOWN 0xFF

//
#define AES_KEY_SIZE 24
#define AES_AAD_SIZE 32
#define AES_AUTH_SIZE 16
#define AES_IV_SIZE 11

/******************************* Type Definitions ****************************/
typedef enum _BLExitMode {
    BL_EXIT_IMMEDIATE = 0,
    BL_EXIT_TIMEOUT = 1,
    BL_EXIT_INDEFINITE = 2,
} BLExitMode_t;

typedef enum _BLCmdDevSetMode {
    BLCmdDevSetMode_MAIN_CMD = 0x01,
    //sub commands
    BLCmdDevSetMode_SET_MODE = 0x00,
} BLCmdDevSetMode_t;

typedef enum _BLCmdDevGetMode {
    BLCmdDevGetMode_MAIN_CMD = 0x02,
    //sub commands
    BLCmdDevGetMode_GET_MODE = 0x00,
} BLCmdDevGetMode_t;

typedef enum _BLCmdFlash {
    BLCmdFlash_MAIN_CMD = 0x80,
    // sub commands
    BLCmdFlash_SET_IV = 0x00,
    BLCmdFlash_SET_AUTH = 0x01,
    BLCmdFlash_SET_NUM_PAGES = 0x02,
    BLCmdFlash_ERASE_APP_MEMORY = 0x03,
    BLCmdFlash_WRITE_PAGE = 0x04,
    BLCmdFlash_ERASE_PAGE_MEMORY = 0x05,
    BLCmdFlash_SET_PARTIAL_PAGE_SIZE = 0x06
} BLCmdFlash_t;

typedef enum _BLCmdInfo {
    BLCmdInfo_MAIN_CMD = 0x81,
    // sub commands
    BLCmdInfo_GET_VERSION = 0x00,
    BLCmdInfo_GET_PAGE_SIZE = 0x01,
    BLCmdInfo_GET_USN = 0x02,
} BLCmdInfo_t;

typedef enum _BLCmdConfigWrite {
    BLCmdConfigWrite_MAIN_CMD = 0x82,
    // sub commands
    BLCmdConfigWrite_SAVE_SETTINGS = 0x00,
    BLCmdConfigWrite_ENTRY_CONFIG = 0x01,
    BLCmdConfigWrite_EXIT_CONFIG = 0x02,
} BLCmdConfigWrite_t;

typedef enum _BLCmdConfigRead {
    BLCmdConfigRead_MAIN_CMD = 0x83,
    // sub commands
    BLCmdConfigRead_ENTRY_CONFIG = 0x01,
    BLCmdConfigRead_EXIT_CONFIG = 0x02,
    BLCmdConfigRead_ALL_CFG = 0xFF,
} BLCmdConfigRead_t;

typedef enum _BLCmdDeviceInfo {
    BLCmdDeviceInfo_MAIN_CMD = 0xFF,
    // sub commands
    BLCmdDeviceInfo_GET_PLATFORM_TYPE = 0x00
} BLCmdDeviceInfo_t;

typedef enum _BLConfigItems {
    BL_CFG_EBL_CHECK = 0x00,
    BL_CFG_EBL_PIN = 0x01,
    BL_CFG_EBL_POL = 0x02,
    BL_CFG_VALID_MARK_CHK = 0x03,
    BL_CFG_UART_ENABLE = 0x04,
    BL_CFG_I2C_ENABLE = 0x05,
    BL_CFG_SPI_ENABLE = 0x06,
    BL_CFG_I2C_SLAVE_ADDR = 0x07,
    BL_CFG_CRC_CHECK = 0x08,
    BL_CFG_LOCK_SWD = 0x09,
} BLConfigItems_t;

typedef union {
    struct {
        uint32_t enter_bl_check : 1; // MFIO pin check enable
        uint32_t ebl_pin : 4; // MFIO Pin
        uint32_t ebl_polarity : 1; // 0 - active low, 1 active high
        uint32_t res0 : 2; // Reserved for future usage
        uint32_t uart_enable : 1; // Enable UART
        uint32_t i2c_enable : 1; // Enable I2C
        uint32_t spi_enable : 1; // Enable SPI
        uint32_t res1 : 5; // Reserved for future usage
        uint32_t ebl_timeout : 4; // Programmable timeout to enter bootloader
        uint32_t exit_bl_mode : 2; // Timeout mode
        uint32_t res2 : 2; // Reserved for future usage
        uint32_t crc_check : 1; // Enable CRC Check
        uint32_t valid_mark_check : 1; // Enable Valid Mark Check
        uint32_t lock_swd : 1; // Lock SWD
        uint32_t res3 : 5; // Reserved for future usage
        uint32_t i2c_addr : 7; // I2C Slave Address
        uint32_t res4 : 25; // Reserved for future usage
    };
    uint64_t cfg;
    uint8_t v[8];
} boot_config_t;

typedef union {
    struct {
        uint32_t enter_bl_check : 1; // MFIO pin check enable
        uint32_t ebl_pin : 4; // MFIO Pin
        uint32_t ebl_polarity : 1; // 0 - active low, 1 active high
        uint32_t ebl_port : 2; //
        uint32_t uart_enable : 1; // Enable UART
        uint32_t i2c_enable : 1; // Enable I2C
        uint32_t spi_enable : 1; // Enable SPI
        uint32_t i2c_addr : 2; // I2C Slave Address
        uint32_t res1 : 3; // Reserved for future usage
        uint32_t ebl_timeout : 4; // Programmable timeout to enter bootloader
        uint32_t exit_bl_mode : 2; // Timeout mode
        uint32_t res2 : 2; // Reserved for future usage
        uint32_t crc_check : 1; // Enable CRC Check
        uint32_t valid_mark_check : 1; // Enable Valid Mark Check
        uint32_t lock_swd : 1; // Lock SWD
        uint32_t res3 : 5; // Reserved for future usage
    };
    uint32_t cfg;
    uint8_t v[4];
} boot_config_t_before_v342;

/******************************* Public Functions ****************************/

#endif // EXAMPLES_MAX32660_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_CMD_H_
