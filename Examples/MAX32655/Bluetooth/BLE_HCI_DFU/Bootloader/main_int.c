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
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "pb.h"
#include "board.h"
#include "mxc_delay.h"
#include "flc.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Addresses for the flash sections, defined in the linker file */
extern uint32_t _flash0;
extern uint32_t _flash1;

#define FLASH0_START ((uint32_t)&_flash0)
#define FLASH1_START ((uint32_t)&_flash1)

/* 
adjust this FLASH_LEN according to the linker script 
in both Bootloader and BLT folder
*/
#define FLASH_LEN 0x38000

#define FLASH_ERASED_WORD 0xFFFFFFFF
#define MXC_GPIO_PORT_IN MXC_GPIO0
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_19
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

/**************************************************************************************************
  Functions
**************************************************************************************************/

/* Defined in boot_lower.S */
extern void Boot_Lower(void);

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

void bootError(void)
{
    /* Flash the failure LED */
    int j;
    volatile int i;
    for (j = 0; j < 10; j++) {
        LED_Toggle(0);
        for (i = 0; i < 0xFFFFF; i++) {}
    }
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

static int multiPageErase(uint8_t *address, uint32_t size)
{
    int err;
    volatile uint32_t address32 = (uint32_t)address;
    address32 &= 0xFFFFF;

    /* Page align the size */
    size += MXC_FLASH_PAGE_SIZE - (size % MXC_FLASH_PAGE_SIZE);

    while (size) {
        err = MXC_FLC_PageErase((uint32_t)address);
        if (err != E_NO_ERROR) {
            return err;
        }

        address += MXC_FLASH_PAGE_SIZE;
        size -= MXC_FLASH_PAGE_SIZE;
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

int main(void)
{
    volatile int i;
    int numLedsBlink;

    /* Limit the number of LED blinks */
    if (num_leds > 2) {
        numLedsBlink = 2;
    } else {
        numLedsBlink = num_leds;
    }

    /* Prevent bricks */
    if (numLedsBlink == 0) {
        DELAY(0x3FFFFF);
    }

    LED_Init();
    for (int led = 0; led < numLedsBlink; led++) {
        LED_On(led);
        DELAY(0x1FFFFF);
        LED_Off(led);
        DELAY(0x1FFFFF);
    }

    if (((uint32_t)(*(uint32_t *)FLASH1_START)) != 0xFFFFFFFF) {
        multiPageErase((uint8_t *)FLASH0_START, FLASH_LEN);

        flashWrite((uint32_t *)FLASH0_START, (uint32_t *)FLASH1_START, FLASH_LEN);

        multiPageErase((uint8_t *)FLASH1_START, 0x40000);
    }

    Boot_Lower();

    while (1) {}
}
