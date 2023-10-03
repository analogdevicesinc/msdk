/**
 * @file        main.c
 * @brief       24LC256 EEPROM I2C Communication Example
 * @details     This example uses the I2C Master to read/write from/to the EEPROM.
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "mxc_device.h"

#include "eeprom_24lc256_driver.h"

/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 ///< I2C instance
#define I2C_FREQ 100000 ///< I2C clock frequency

#define EEPROM_24LC256_I2C_SLAVE_ADDR0 0x50//(0xA0 >> 1)

#define EEPROM_DEMO_BUFFER_PAGE_COUNT	4
#define EEPROM_DEMO_BUFFER_SIZE			_24LC256_EEPROM_PAGE_SIZE * EEPROM_DEMO_BUFFER_PAGE_COUNT		// Bytes
// *****************************************************************************

int main(void)
{
    int err = E_NO_ERROR;
    int i = 0, j = 0;
	uint16_t eeprom_memory_addr = 0x0000;
	uint8_t written_val = 0;
	uint8_t readed_val = 0;
	uint8_t eeprom_demo_buffer[EEPROM_DEMO_BUFFER_SIZE];
	uint32_t page_offset = 0;

    printf("\n****************** 24LC256 EEPROM DEMO *******************\n\n");
    printf("This example communicates with an external 24LC256 EEPROM using the I2C.\n");
    printf("One 24LC256 EEPROM is required to implement this example.\n");
    printf("Press ENTER key to Continue\n\n");
    getchar();

    err = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (err != E_NO_ERROR) {
        printf("EEPROM configure failed with error %i\n", err);
        return err;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    eeprom_24LC256_driver_t eeprom_24LC256 = eeprom_24LC256_Open();

    eeprom_24LC256.init(I2C_MASTER, EEPROM_24LC256_I2C_SLAVE_ADDR0); // init the EEPROM

    printf("EEPROM DEMO - Test 1: Writing and reading 1 byte:\n\n");
    for(i = 0; i < 5; i++)
    {
    	err = eeprom_24LC256.write(eeprom_memory_addr, &written_val, 1);
		if (err != E_NO_ERROR) {
			printf("EEPROM write error, error code = %d\n", err);
		}else{
			printf("The value: 0x%02X is written to the address: 0x%04X\n", written_val, eeprom_memory_addr);
		}
    	err = eeprom_24LC256.read(eeprom_memory_addr, &readed_val, 1);
		if (err != E_NO_ERROR) {
			printf("EEPROM read error, error code = %d\n", err);
		}else{
			printf("The value: 0x%02X is read from the address: 0x%04X\n", readed_val, eeprom_memory_addr);
		}
		if(readed_val != written_val){
			printf("EEPROM error; written and read values are different\n", readed_val, eeprom_memory_addr);
		}

    	written_val++;
    	eeprom_memory_addr++;
	}

    printf("\nEEPROM DEMO - Test2: Writing and Reading %d Bytes:\n\n", EEPROM_DEMO_BUFFER_SIZE);
    eeprom_memory_addr = 0x0000;
    page_offset = eeprom_memory_addr / _24LC256_EEPROM_PAGE_SIZE;

    for(i = 0; i < EEPROM_DEMO_BUFFER_PAGE_COUNT; i++)
    {
        for(j = 0; j < _24LC256_EEPROM_PAGE_SIZE; j++)
        {
        	eeprom_demo_buffer[i*_24LC256_EEPROM_PAGE_SIZE + j] = i;
        }
    }

	err = eeprom_24LC256.write(eeprom_memory_addr, &eeprom_demo_buffer[0], EEPROM_DEMO_BUFFER_SIZE);
	if (err != E_NO_ERROR) {
		printf("EEPROM write error, error code = %d\n", err);
	}else{
		printf("%d Bytes written to the EEPROM starting from the address: 0x%04X. Each page (64 byte) filled with its own page number.\n", EEPROM_DEMO_BUFFER_SIZE, eeprom_memory_addr);
	}

    for(i = 0; i < EEPROM_DEMO_BUFFER_SIZE; i++)
    {
    	eeprom_demo_buffer[i] = 0;
    }

    err = eeprom_24LC256.read(eeprom_memory_addr, &eeprom_demo_buffer[0], EEPROM_DEMO_BUFFER_SIZE);
	if (err != E_NO_ERROR) {
		printf("EEPROM read %d bytes error, error code = %d\n", EEPROM_DEMO_BUFFER_SIZE, err);
	}else{
		printf("\n%d bytes bytes read from the memory. The start address: 0x%04X. The values read: ", EEPROM_DEMO_BUFFER_SIZE, eeprom_memory_addr);
	    for(i = 0; i < EEPROM_DEMO_BUFFER_SIZE; i++)
	    {
	        if(i % _24LC256_EEPROM_PAGE_SIZE  == 0)
	        {
	    		printf("\nPage %4d: ", (i/_24LC256_EEPROM_PAGE_SIZE + page_offset));
	        }
	        printf("0x%02X, ", eeprom_demo_buffer[i]);

		}
		printf("\n");
	}

	printf("\nEEPROM DEMO - Test 3: Write and read test for writing multiple pages starting from random adress(not page start):\n\n");

	uint32_t test_size = 80;
	uint16_t test_val = 0xA5;
    eeprom_memory_addr = 0x003A;

	printf("Test size : %d Bytes \n", test_size);
	printf("Test value : 0x%02X \n", test_val);
	printf("Test start address : 0x%04X \n", eeprom_memory_addr);

    for(i = 0; i < test_size; i++)
    {
    	eeprom_demo_buffer[i] = test_val;
    }

	err = eeprom_24LC256.write(eeprom_memory_addr, &eeprom_demo_buffer[0], test_size);
	if (err != E_NO_ERROR) {
		printf("EEPROM write error, error code = %d\n", err);
	}else{
		printf("The value: 0x%02X is written to EEPROM for %d Bytes starting from the address: 0x%04X\n", test_val, test_size, eeprom_memory_addr);
	}

    for(i = 0; i < test_size; i++)
    {
    	eeprom_demo_buffer[i] = 0;
    }

	err = eeprom_24LC256.read(eeprom_memory_addr, &eeprom_demo_buffer[0], test_size);
	if (err != E_NO_ERROR) {
		printf("EEPROM read error, error code = %d\n", err);
	}else{
		printf("\n%d bytes read from the memory. The start address: 0x%04X. The values read: \n", test_size, eeprom_memory_addr);
		for(i = 0; i < test_size; i++)
		{
			printf("0x%02X, ", eeprom_demo_buffer[i]);
		}
		printf("\n");
	}

    return E_NO_ERROR;
}
