/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "sfe_host.h"


/***** Globals *****/
mxc_spi_req_t master_req;

//******************************************************************************
void SFE_Reset()
{
    uint8_t cmd = SFE_CMD_RST_EN;
    
    // Initialize spi_reqest struct for Master
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0x0;
    master_req.ssDeassert = 1;
    master_req.txData = &cmd;
    master_req.rxData = NULL;
    master_req.txLen = 1;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("\nError resetting chip\r\n");
    }
}

void SFE_ID(uint8_t* id)
{
    uint32_t cmd = SFE_CMD_ID;
    
    // Initialize spi_reqest struct for Master
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0x0;
    master_req.ssDeassert = 1;
    master_req.txData = (uint8_t*) &cmd;
    master_req.rxData = id;
    master_req.txLen = 4;
    master_req.rxLen = 1;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
}

void SFE_FlashWrite(uint8_t* txdata, uint32_t length, uint32_t address, uint32_t command, spi_width_t width, spi_address_t addrMode)
{
    uint8_t cmd[5] = {0};
    uint8_t flashCmd[4] = {0};
    uint8_t flashAddr[4] = {0};
    uint8_t dataLength[4] = {0};
    
    switch (width) {
    case SPI_WIDTH_01 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_WRITE;
        }
        else {
            cmd[0] = SFE_WRITE;
        }
        
        break;
        
    case SPI_WIDTH_02 :
        cmd[0] = SFE_DUAL_FAST_WRITE;
        break;
        
    case SPI_WIDTH_04 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_QUAD_FAST_WRITE;
            
        }
        else {
            cmd[0] = SFE_QUAD_FAST_WRITE;
        }
        
        break;
        
    default:
        printf("Invalid SPI width\r\n");
    }
    
    if (command == FLASH_WRITE) {
        if (addrMode) {
            cmd[1] = (FLASH_WRITE_SBA >> 24) & 0xFF;
            cmd[2] = (FLASH_WRITE_SBA >> 16) & 0xFF;
            cmd[3] = (FLASH_WRITE_SBA >>  8) & 0xFF;
            cmd[4] = FLASH_WRITE_SBA & 0xFF;
        }
        else {
            cmd[1] = (FLASH_WRITE_SBA >> 16) & 0xFF;
            cmd[2] = (FLASH_WRITE_SBA >>  8) & 0xFF;
            cmd[3] = FLASH_WRITE_SBA & 0xFF;
        }
    }
    else if (command == FLASH_PAGE_ERASE) {
        if (addrMode) {
            cmd[1] = (FLASH_ERASE_SBA >> 24) & 0xFF;
            cmd[2] = (FLASH_ERASE_SBA >> 16) & 0xFF;
            cmd[3] = (FLASH_ERASE_SBA >>  8) & 0xFF;
            cmd[4] = FLASH_ERASE_SBA & 0xFF;
        }
        else {
            cmd[1] = (FLASH_ERASE_SBA >> 16) & 0xFF;
            cmd[2] = (FLASH_ERASE_SBA >>  8) & 0xFF;
            cmd[3] = FLASH_ERASE_SBA & 0xFF;
        }
    }
    else {
        printf("\nInvalid Command\n");
    }
    
    if (addrMode) {
        master_req.txLen = 5;
    }
    else {
        master_req.txLen = 4;
    }
    
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0;
    master_req.ssDeassert = 0;
    master_req.txData = cmd;
    master_req.rxData = NULL;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    flashCmd[0] = command & 0xFF;
    flashCmd[1] = (command >>  8) & 0xFF;
    flashCmd[2] = (command >> 16) & 0xFF;
    flashCmd[3] = (command >> 24) & 0xFF;
    master_req.txLen = 4;
    master_req.rxData = NULL;
    master_req.txData = flashCmd;
    
    status = MXC_SPI_SetWidth(MASTER_SPI, width);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    flashAddr[0] = address & 0xFF;
    flashAddr[1] = (address >>  8) & 0xFF;
    flashAddr[2] = (address >> 16) & 0xFF;
    flashAddr[3] = (address >> 24) & 0xFF;
    master_req.txLen = 4;
    master_req.txData = flashAddr;
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    dataLength[0] = length & 0xFF;
    dataLength[1] = (length >>  8) & 0xFF;
    dataLength[2] = (length >> 16) & 0xFF;
    dataLength[3] = (length >> 24) & 0xFF;
    master_req.txLen = 4;
    master_req.txData = dataLength;
    
    if (command == FLASH_PAGE_ERASE) {
        master_req.ssDeassert = 1;
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    if (command == FLASH_WRITE) {
        // Initialize spi_reqest struct for Master
        master_req.ssDeassert = 1;
        master_req.txData = txdata;
        master_req.rxData = NULL;
        master_req.txLen = length;
        status  =  MXC_SPI_MasterTransaction(&master_req);
        
        if (status != E_NO_ERROR) {
            printf("Error Writing to the chip\r\n");
        }
        else {
            printf("\nWrite Cmd and Data Sent");
        }
    }
}

void SFE_RAMWrite(uint8_t* txdata, uint32_t length, uint32_t address, spi_width_t width, spi_address_t addrMode)
{
    uint8_t cmd[5] = {0};
    
    /*if(address_4byte_enable && !addr)    // Invalid settings
        return E_INVALID;*/
    
    // if(flash_busy(SPI_WIDTH_01))
    //     return;
    
    switch (width) {
    case SPI_WIDTH_01 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_WRITE;
            cmd[1] = (address >> 24) & 0xFF;
            cmd[2] = (address >> 16) & 0xFF;
            cmd[3] = (address >>  8) & 0xFF;
            cmd[4] = address & 0xFF;
        }
        else {
            cmd[0] = SFE_WRITE;
            cmd[1] = (address >> 16) & 0xFF;
            cmd[2] = (address >>  8) & 0xFF;
            cmd[3] = address & 0xFF;
        }
        
        break;
        
    case SPI_WIDTH_02 :
        cmd[0] = SFE_DUAL_FAST_WRITE;
        cmd[1] = (address >> 16) & 0xFF;
        cmd[2] = (address >>  8) & 0xFF;
        cmd[3] = address & 0xFF;
        break;
        
    case SPI_WIDTH_04 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_QUAD_FAST_WRITE;
            cmd[1] = (address >> 24) & 0xFF;
            cmd[2] = (address >> 16) & 0xFF;
            cmd[3] = (address >>  8) & 0xFF;
            cmd[4] = address & 0xFF;
        }
        else {
            cmd[0] = SFE_QUAD_FAST_WRITE;
            cmd[1] = (address >> 16) & 0xFF;
            cmd[2] = (address >>  8) & 0xFF;
            cmd[3] = address & 0xFF;
        }
        
        break;
        
    default:
        printf("Invalid SPI width\r\n");
        //return -1;
    }
    
    printf("\nWrite cmd: 0x%x\n", cmd[0]);
    
    // Initialize spi_reqest struct for Master
    if (addrMode) {
        master_req.txLen = 5;
    }
    else {
        master_req.txLen = 4;
    }
    
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0;
    master_req.ssDeassert = 0;
    master_req.txData = cmd;
    master_req.rxData = NULL;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    
    // Initialize spi_reqest struct for Master
    master_req.ssDeassert = 1;
    master_req.txData = txdata;
    master_req.rxData = NULL;
    master_req.txLen = length;
    
    status = MXC_SPI_SetWidth(MASTER_SPI, width);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  =  MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error writing to the chip\r\n");
    }
    else {
        printf("Write Command and Data Sent\n");
    }
}

void SFE_Read(uint8_t* rxdata, uint32_t length, uint32_t address, spi_width_t width, spi_address_t addrMode)
{
    uint8_t cmd[5] = {0};
    
    switch (width) {
    case SPI_WIDTH_01 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_READ;
            cmd[1] = (address >> 24) & 0xFF;
            cmd[2] = (address >> 16) & 0xFF;
            cmd[3] = (address >>  8) & 0xFF;
            cmd[4] = address & 0xFF;
        }
        else {
            cmd[0] = SFE_READ;
            cmd[1] = (address >> 16) & 0xFF;
            cmd[2] = (address >>  8) & 0xFF;
            cmd[3] = address & 0xFF;
        }
        
        break;
        
    case SPI_WIDTH_02 :
        cmd[0] = SFE_4BYTE_DUAL_FAST_READ;
        cmd[1] = (address >> 24) & 0xFF;
        cmd[2] = (address >> 16) & 0xFF;
        cmd[3] = (address >>  8) & 0xFF;
        cmd[4] = address & 0xFF;
        break;
        
    case SPI_WIDTH_04 :
        if (addrMode) {
            cmd[0] = SFE_4BYTE_QUAD_FAST_READ;
            cmd[1] = (address >> 24) & 0xFF;
            cmd[2] = (address >> 16) & 0xFF;
            cmd[3] = (address >>  8) & 0xFF;
            cmd[4] = address & 0xFF;
        }
        else {
            cmd[0] = SFE_QUAD_FAST_READ;
            cmd[1] = (address >> 16) & 0xFF;
            cmd[2] = (address >>  8) & 0xFF;
            cmd[3] = address & 0xFF;
        }
        
        break;
        
    default:
        printf("Invalid SPI width\r\n");
    }
    
    printf("\nRead cmd: 0x%x\n", cmd[0]);
    
    // Initialize spi_reqest struct for Master
    if (addrMode) {
        master_req.txLen = 5;
    }
    else {
        master_req.txLen = 4;
    }
    
    // Initialize spi_reqest struct for Master
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0;
    master_req.ssDeassert = 0;
    master_req.txData = cmd;
    master_req.rxLen = 0;
    master_req.rxData = NULL;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error transmitting data\r\n");
    }
    
    // Send the dummy cycles
    if (width != SPI_WIDTH_01) {
        if (width == SPI_WIDTH_02) {
            // Send the dummy cycles
            master_req.spi = MASTER_SPI;
            master_req.rxLen = 2;
            master_req.txData = NULL;
            master_req.rxData = rxdata;
            master_req.ssDeassert = 0;
        }
        else if (width == SPI_WIDTH_04) {
            // Send the dummy cycles
            master_req.spi = MASTER_SPI;
            master_req.rxLen = 4;
            master_req.txData = NULL;
            master_req.rxData = rxdata;
            master_req.ssDeassert = 0;
        }
        
        status = MXC_SPI_SetWidth(MASTER_SPI, width);
        
        if (status != E_NO_ERROR) {
            printf("\nError Setting data width\r\n");
        }
        
        status  = MXC_SPI_MasterTransaction(&master_req);
        
        if (status != E_NO_ERROR) {
            printf("Error reading from the chip\r\n");
        }
    }
    
    // Initialize spi_reqest struct for Master
    master_req.ssDeassert = 1;
    master_req.txData = NULL;
    master_req.rxData = rxdata;
    master_req.rxLen = length;
    
    status = MXC_SPI_SetWidth(MASTER_SPI, width);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("Error reading from the chip\r\n");
    }
    else {
        printf("Data read from SFE slave\r\n");
    }
}

// *****************************************************************************
void SFE_4ByteModeEnable()
{

    uint8_t cmd = SFE_4BYTE_ENTER;
    
    // Initialize spi_reqest struct for Master
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0x0;
    master_req.ssDeassert = 1;
    master_req.txData = &cmd;
    master_req.rxData = NULL;
    master_req.txLen = 1;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("\nError configuring 4 byte mode\r\n");
    }
    else {
        printf("\n4 byte mode configured \r\n");
    }
}

// *****************************************************************************
void SFE_4ByteModeDisable()
{
    uint8_t cmd = SFE_4BYTE_EXIT;
    
    // Initialize spi_reqest struct for Master
    master_req.spi = MASTER_SPI;
    master_req.ssIdx = 0x0;
    master_req.ssDeassert = 1;
    master_req.txData = &cmd;
    master_req.rxData = NULL;
    master_req.txLen = 1;
    master_req.rxCnt = 0;
    master_req.txCnt = 0;
    master_req.completeCB = NULL;
    
    int status = MXC_SPI_SetWidth(MASTER_SPI, SPI_WIDTH_01);
    
    if (status != E_NO_ERROR) {
        printf("\nError Setting data width\r\n");
    }
    
    status  = MXC_SPI_MasterTransaction(&master_req);
    
    if (status != E_NO_ERROR) {
        printf("\nError exiting 4-byte mode\r\n");
    }
    else {
        printf("\nExit from 4-Byte mode \r\n");
    }
}