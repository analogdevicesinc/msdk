/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "spi.h"

#define MASTER_SPI                  MXC_SPI1
#define MASTER_SPI_SPEED            100000      // Bit Rate

#define SFE_CMD_ID                  0xFFFFFF9F

#define SFE_CMD_RST_EN              0x66
#define SFE_CMD_RST_MEM             0x99

#define SFE_4BYTE_ENTER             0xB7
#define SFE_4BYTE_EXIT              0xE9

#define SFE_BUSY                    0x01         /* status register */

#define SFE_WRITE                   0x02
#define SFE_4BYTE_WRITE             0x12

#define SFE_READ                    0x03
#define SFE_4BYTE_READ              0x13
#define SFE_FAST_READ               0x0B
#define SFE_4BYTE_FAST_READ         0x0C
#define SFE_TPM_READ                0xFE

#define SFE_DUAL_FAST_WRITE         0xA2
#define SFE_4BYTE_DUAL_FAST_READ    0x3C

#define SFE_QUAD_FAST_WRITE         0x32
#define SFE_4BYTE_QUAD_FAST_WRITE   0x34
#define SFE_QUAD_FAST_READ          0x6B
#define SFE_4BYTE_QUAD_FAST_READ    0x6C

#define SFE_CMD_READ_SR             0x05

#define SFE_READ_SFDP               0x5A

//FLASH COMMANDS
#define FLASH_WRITE                 0x02

#define FLASH_PAGE_ERASE            0x55

#define RAM_SBA            0x00330000
#define FLASH_SBA          0x00990000

#define FLASH_WRITE_SBA    0x00330008
#define FLASH_ERASE_SBA    0x00330000

typedef enum {
    SPI_WIDTH_01 = SPI_WIDTH_STANDARD,
    SPI_WIDTH_02 = SPI_WIDTH_DUAL,
    SPI_WIDTH_04 = SPI_WIDTH_QUAD,
} spi_width_t;

typedef enum {
    SFE_3BYTE = 0,
    SFE_4BYTE = 1,
} spi_address_t;


/**
 * @brief
 *
 */
void SFE_Reset();

/**
 * @brief
 *
 * @param id
 */
void SFE_ID(uint8_t* id);

/**
 * @brief
 *
 * @param txdata
 * @param length
 * @param address
 * @param command
 * @param width
 * @param addrMode
 */
void SFE_FlashWrite(uint8_t* txdata, uint32_t length, uint32_t address, uint32_t command, spi_width_t width, spi_address_t addrMode);

/**
 * @brief
 *
 * @param txdata
 * @param length
 * @param address
 * @param width
 * @param addrMode
 */
void SFE_RAMWrite(uint8_t* txdata, uint32_t length, uint32_t address, spi_width_t width, spi_address_t addrMode);

/**
 * @brief
 *
 * @param rxdata
 * @param length
 * @param address
 * @param width
 * @param addrMode
 */
void SFE_Read(uint8_t* rxdata, uint32_t length, uint32_t address, spi_width_t width, spi_address_t addrMode);

/**
 * @brief
 *
 */
void SFE_4ByteModeEnable();

/**
 * @brief
 *
 */
void SFE_4ByteModeDisable();