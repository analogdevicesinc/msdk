/*************************************************************************************************/
/*!
 * @file    main.c
 * @brief   Bluetooth data server that advertises as "OTAS" and accepts connection requests. 
 *          Demonstrates Over-the-Air (OTA) firmware updates.
 *
 *  Copyright (c) 2013-2019 Arm Ltd. All Rights Reserved.
 *
 *  Copyright (c) 2019 Packetcraft, Inc.
 *
 *  Portions Copyright (c) 2022-2023 Analog Devices, Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
/*************************************************************************************************/

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


typedef struct {
    uint32_t fileLen;
    uint32_t fileCRC;
} fileHeader_t;
fileHeader_t fileHeader;
extern uint32_t _flash0;
extern uint32_t _flash1;
#define MXC_GPIO_PORT_IN MXC_GPIO0
#define MXC_GPIO_PIN_IN MXC_GPIO_PIN_18
#define EXT_FLASH_BLOCK_SIZE 224

#define FLASH0_START ((uint32_t)0x10004000)
#define FLASH1_START ((uint32_t)0x10040000)
#define FLASH0_LEN ((uint32_t)0x25000)
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

int main(void)
{
    mxc_gpio_cfg_t gpio_in;
    //int err = 0x00000000;
    //uint32_t startingAddress = 0x00000000;

    /* init external flash */
    Ext_Flash_Init();
    Ext_Flash_Quad(1);
    gpio_in.port = MXC_GPIO_PORT_IN;
    gpio_in.mask = MXC_GPIO_PIN_IN;
    gpio_in.pad = MXC_GPIO_PAD_PULL_UP;
    gpio_in.func = MXC_GPIO_FUNC_IN;
    gpio_in.vssel = MXC_GPIO_VSSEL_VDDIO;
    gpio_in.drvstr = MXC_GPIO_DRVSTR_0;
    MXC_GPIO_Config(&gpio_in);

    printf("First Program\n");


    if (!MXC_GPIO_InGet(gpio_in.port, gpio_in.mask))
    {
        Ext_Flash_Erase(0x00000000, Ext_Flash_Erase_64K);
        uint32_t pagesToErase = (0x40000 / MXC_FLASH_PAGE_SIZE) + 1;
        Ext_Flash_Program_Page(0x00000000, (uint8_t *)FLASH0_START, 0x40000, Ext_Flash_DataLine_Quad);
        printf("start update the program\n");
        

    }
    else{

        printf("not update the program\n");
    }

         uint32_t * a = (uint32_t *) 0x10004000;
         
        printf("%x: %x\n",a,*a);
       
        a = (uint32_t *) 0x10004004;
        printf("%x: %x\n",a,(uint8_t *)*a);
        
       a = (uint32_t *) 0x10004008;
       printf("%x: %x\n",a,*a);
      uint8_t extFlashBlockBuff[EXT_FLASH_BLOCK_SIZE] = { 0 };
      Ext_Flash_Read(0x00000000, extFlashBlockBuff, EXT_FLASH_BLOCK_SIZE,
                       Ext_Flash_DataLine_Quad);
        
        printf("%x %x %x %x\n",extFlashBlockBuff[0],extFlashBlockBuff[1],extFlashBlockBuff[2],extFlashBlockBuff[3]);
        printf("%x %x %x %x\n",extFlashBlockBuff[4],extFlashBlockBuff[5],extFlashBlockBuff[6],extFlashBlockBuff[7]);
        printf("%x %x %x %x\n",extFlashBlockBuff[8],extFlashBlockBuff[9],extFlashBlockBuff[10],extFlashBlockBuff[11]);
        printf("on\n");
        
       
        return 0;
       
        return 0;
        
}
        
        
    
    