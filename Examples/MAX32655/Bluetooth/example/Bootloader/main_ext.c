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

/**
 * @file    main.c
 * @brief   Bootloader
 * @details Simple bootloader to verify and replace images.
 */

/***** Includes *****/
#include <stdio.h>
#include "gpio.h"
#include "mxc_device.h"
/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"
#include "flc.h"
#include "Ext_Flash.h"
/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Addresses for the flash sections, defined in the linker file */
extern uint32_t _flash0;
extern uint32_t _flash1;

#define FLASH0_START ((uint32_t)&_flash0)
#define FLASH1_START ((uint32_t)&_flash1)
#define FLASH_ERASED_WORD 0xFFFFFFFF
#define CRC32_LEN 4
#define EXT_FLASH_BLOCK_SIZE 224

#define DELAY(loopCount) \
    for (i = 0; i < loopCount; i++) {}

/**************************************************************************************************
  Local Variables
**************************************************************************************************/
typedef struct {
    uint32_t fileLen;
    uint32_t fileCRC;
} fileHeader_t;
fileHeader_t fileHeader;

typedef enum { COPY_FILE_OP, CALC_CRC32_OP } externFileOp_t;
/**************************************************************************************************
  Functions
**************************************************************************************************/

/* Defined in boot_lower.S */
extern void Boot_Lower(void);

void led_On(unsigned int idx)
{
    MXC_GPIO_OutClr(led_pin[idx].port, led_pin[idx].mask);
}

void led_Off(unsigned int idx)
{
    MXC_GPIO_OutSet(led_pin[idx].port, led_pin[idx].mask);
}

void led_Toggle(unsigned int idx)
{
    MXC_GPIO_OutToggle(led_pin[idx].port, led_pin[idx].mask);
}

void ledSuccessPattern(void)
{
    /* Green LED blinks */
    volatile int i, j;
    for (j = 0; j < 10; j++) {
        led_Toggle(1);
        DELAY(0xFFFFF);
    }
}
void ledFailPattern(void)
{
    /* Red LED blinks */
    volatile int i, j;
    for (j = 0; j < 10; j++) {
        led_Toggle(0);
        DELAY(0xFFFFF);
    }
}

// http://home.thep.lu.se/~bjorn/crc/
/*************************************************************************************************/
/*!
 *  \brief  Create the CRC32 table.
 *
 *  \param  r       Index into the table
 *
 *  \return None.
 */
/*************************************************************************************************/
uint32_t crc32_for_byte(uint32_t r)
{
    for (int j = 0; j < 8; ++j) r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
    return r ^ (uint32_t)0xFF000000L;
}

/*************************************************************************************************/
/*!
 *  \brief  Calculate the CRC32 value for the given buffer.
 *
 *  \param  data    Pointer to the data.
 *  \param  n_bytes Number of bytes in the buffer.
 *  \param  crc     Pointer to store the result.
 *
 *  \return None.
 */
/*************************************************************************************************/
static uint32_t table[0x100] = { 0 };
void crc32(const void *data, size_t n_bytes, uint32_t *crc)
{
    if (!*table) {
        for (size_t i = 0; i < 0x100; ++i) {
            table[i] = crc32_for_byte(i);
        }
    }

    for (size_t i = 0; i < n_bytes; ++i) {
        *crc = table[(uint8_t)*crc ^ ((uint8_t *)data)[i]] ^ *crc >> 8;
    }
}

void bootError(void)
{
    /* Flash the failure LED */
    ledFailPattern();
    NVIC_SystemReset();
}

int flashPageErased(uint32_t *addr)
{
    /* Determine if this page is totally erased */
    int i;
    for (i = 0; i < (MXC_FLASH_PAGE_SIZE / 4); i++) {
        if (*(addr + i) != FLASH_ERASED_WORD) {
            return 0;
        }
    }

    return 1;
}

static int multiPageErase(uint8_t *address, uint32_t pages)
{
    int err;
    volatile uint32_t address32 = (uint32_t)address;
    address32 &= 0xFFFFF;

    while (pages) {
        err = MXC_FLC_PageErase((uint32_t)address);
        if (err != E_NO_ERROR) {
            return err;
        }

        address += MXC_FLASH_PAGE_SIZE;
        pages--;
    }

    return E_NO_ERROR;
}

static int flashWrite(uint32_t *address, uint32_t *data, uint32_t len)
{
    int err;

    while ((len / 16) > 0) {
        err = MXC_FLC_Write128((uint32_t)address, data);
        if (err != E_NO_ERROR) {
            return err;
        }
        len -= 16;
        address += 4;
        data += 4;
    }
    while (len) {
        err = MXC_FLC_Write32((uint32_t)address, *data);
        if (err != E_NO_ERROR) {
            return err;
        }
        len -= 4;
        address += 1;
        data += 1;
    }
    return E_NO_ERROR;
}

uint32_t externFileOperation(externFileOp_t fileOperation)
{
    uint32_t internalFlashStartingAddress = FLASH0_START;
    uint8_t extFlashBlockBuff[EXT_FLASH_BLOCK_SIZE] = { 0 };
    uint32_t startingAddress = 0x00000000;
    uint32_t fileLen = 0x40000;
    uint32_t crcResult = 0;
    uint32_t err = 0;
    /* Read blocks from ext flash and perform desired fileOperation */
    while (fileLen >= EXT_FLASH_BLOCK_SIZE) {
        Ext_Flash_Read(startingAddress, extFlashBlockBuff, EXT_FLASH_BLOCK_SIZE,
                       Ext_Flash_DataLine_Quad);
            err += flashWrite((uint32_t *)internalFlashStartingAddress,
                              (uint32_t *)extFlashBlockBuff, EXT_FLASH_BLOCK_SIZE);
            internalFlashStartingAddress += EXT_FLASH_BLOCK_SIZE;
        
        fileLen -= EXT_FLASH_BLOCK_SIZE;
        startingAddress += EXT_FLASH_BLOCK_SIZE;
    }
    /* Read remaining data that did not fill a block */
    if (fileLen) {
        Ext_Flash_Read(startingAddress, extFlashBlockBuff, fileLen, Ext_Flash_DataLine_Quad);
  
            err += flashWrite((uint32_t *)internalFlashStartingAddress,
                              (uint32_t *)extFlashBlockBuff, fileLen);
        
    }
    if (fileOperation == COPY_FILE_OP)
        return err;

    return crcResult;
}
extern void Boot_Lower(void);
#define DELAY(loopCount) \
    for (i = 0; i < loopCount; i++) {}

#define MXC_GPIO_PORT_OUT MXC_GPIO0

#define MXC_GPIO_PIN_OUT MXC_GPIO_PIN_25
#define MXC_GPIO_PORT_IN MXC_GPIO0
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_19
#define LED1_Pin 24
#define LED2_Pin 25
int main(void)
{
    volatile int i;
     int err = 0x00000000;
    uint32_t startingAddress = 0x00000000;
    uint32_t crcResult = 0x00000000;
    mxc_gpio_cfg_t gpio_out;
    mxc_gpio_cfg_t gpio_in;
    gpio_out.port = MXC_GPIO_PORT_OUT;
    gpio_out.mask = MXC_GPIO_PIN_OUT;
    gpio_out.pad = MXC_GPIO_PAD_NONE;
    gpio_out.func = MXC_GPIO_FUNC_OUT;
    gpio_out.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_out.drvstr = MXC_GPIO_DRVSTR_0;

    gpio_in.port = MXC_GPIO_PORT_IN;
    gpio_in.mask = MXC_GPIO_PIN_IN;
    gpio_in.pad = MXC_GPIO_PAD_PULL_UP;
    gpio_in.func = MXC_GPIO_FUNC_IN;
    gpio_in.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_in.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_in);
    MXC_GPIO_Config(&gpio_out);
    /* Boot from lower image */


    if (!MXC_GPIO_InGet(gpio_in.port, gpio_in.mask)) {
            /* Input pin was high, set the output pin. */
        MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);
     
        __disable_irq();

 
        err += Ext_Flash_Init();
        err += Ext_Flash_Quad(1);


   
        Ext_Flash_Read(startingAddress, (uint8_t *)&fileHeader, sizeof(fileHeader_t),
                       Ext_Flash_DataLine_Quad);
        uint32_t pagesToErase = (0x40000 / MXC_FLASH_PAGE_SIZE) + 1;
        
        multiPageErase((uint8_t *)FLASH0_START, pagesToErase);
        
        externFileOperation(COPY_FILE_OP);

        //Ext_Flash_Erase(0x00000000, Ext_Flash_Erase_64K);

    }
    else
    {
        MXC_GPIO_OutClr(gpio_out.port, gpio_out.mask);
    }
        
 
    Boot_Lower();

    while (1) {}

    return 0;
}
