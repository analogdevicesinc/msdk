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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "spi.h"
#include "tmr.h"
#include "dma.h"
#include "nvic_table.h"
#include "fastspi.h"
#include "aps6404.h"

/***** Definitions *****/
#define TEST_SIZE 640
#define TEST_COUNT 480
#define TEST_VALUE 0x00
#define TEST_ADDR 0x000

/***** Globals *****/

/***** Functions *****/

// A more in-depth (but less readable) SRAM test.  Validates full functionality
// and demonstrates speed improvements of QSPI over standard SPI.  For a simpler
// example, see the main function.
int test(void)
{
    printf("QSPI SRAM Test:\n");
    printf("\tTest Address: 0x%x\n", TEST_ADDR);
    printf("\tTest size: %i bytes\n", TEST_SIZE);
    printf("\tTest count: %i rows\n", TEST_COUNT);
    printf("\tTest speed: %i Hz\n", FASTSPI_SPEED);

    int err = E_NO_ERROR;
    unsigned int elapsed = 0;
    int fail_count = 0;

    // Time the measurement overhead of our measurement functions
    MXC_TMR_SW_Start(MXC_TMR0);
    int sw_overhead = MXC_TMR_SW_Stop(MXC_TMR0);

    uint8_t tx_buffer[TEST_SIZE];
    uint8_t rx_buffer[TEST_SIZE];
    memset(rx_buffer, 0, TEST_SIZE);

    // Time tx_buffer initialization as benchmark
    MXC_TMR_SW_Start(MXC_TMR0);
    memset(tx_buffer, TEST_VALUE, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("(Benchmark) Wrote %i bytes to internal SRAM in %ius\n", TEST_SIZE, elapsed);

    // Benchmark standard-width SPI write to external SRAM
    printf("Test 1: Standard SPI write...\n");
    aps6404_exit_quadmode();
    MXC_TMR_SW_Start(MXC_TMR0);
    aps6404_write(TEST_ADDR, tx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tDone (%i bytes in %ius)\n", TEST_SIZE, elapsed);

    // Read and validate
    printf("Test 2: Validate w/ standard SPI...\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    aps6404_read(TEST_ADDR, rx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            fail_count++;
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", TEST_ADDR + i,
                   tx_buffer[i], rx_buffer[i]);
        }
    }
    printf("\tDone\n");

    printf("Test 3: Validate w/ QSPI...\n");
    aps6404_enter_quadmode();
    MXC_TMR_SW_Start(MXC_TMR0);
    aps6404_read(TEST_ADDR, rx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            fail_count++;
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", TEST_ADDR + i,
                   tx_buffer[i], rx_buffer[i]);
        }
    }
    printf("\tDone\n");

    // Invert test pattern - this ensures every bit has to be updated in the next write
    memset(tx_buffer, ~(TEST_VALUE), TEST_SIZE);
    // memset(tx_buffer, 0x00, TEST_SIZE);
    memset(rx_buffer, 0, TEST_SIZE);

    // Benchmark QSPI write to external SRAM
    printf("Test 4: QSPI Write...\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    err = aps6404_write(TEST_ADDR, tx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tDone (%i bytes in %ius)\n", TEST_SIZE, elapsed);

    // Read and validate
    printf("Test 5: Validate w/ standard SPI...\n");
    aps6404_exit_quadmode();
    MXC_TMR_SW_Start(MXC_TMR0);
    aps6404_read(TEST_ADDR, rx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            fail_count++;
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", TEST_ADDR + i,
                   tx_buffer[i], rx_buffer[i]);
        }
    }
    printf("\tDone\n");

    memset(rx_buffer, 0, TEST_SIZE);

    // Read and validate
    printf("Test 6: Validate w/ QSPI...\n");
    aps6404_enter_quadmode();
    MXC_TMR_SW_Start(MXC_TMR0);
    aps6404_read(TEST_ADDR, rx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            fail_count++;
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", TEST_ADDR + i,
                   tx_buffer[i], rx_buffer[i]);
        }
    }
    printf("\tDone\n");

    // Generate a new more interesting test pattern
    for (int i = 0; i < TEST_SIZE; i++) {
        tx_buffer[i] = i % 256;
    }
    memset(rx_buffer, 0, TEST_SIZE);

    // Benchmark writing across multiple pages boundaries.
    int address = TEST_ADDR;
    printf("Test 7: QSPI Writing across page boundaries...\n", TEST_SIZE, TEST_COUNT);
    MXC_TMR_SW_Start(MXC_TMR0);
    for (int i = 0; i < TEST_COUNT; i++) {
        aps6404_write(address, tx_buffer, TEST_SIZE);
        address += TEST_SIZE;
    }
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0) - sw_overhead;
    printf("\tWrote %i bytes in %ius\n", TEST_SIZE * TEST_COUNT, elapsed);

    printf("Test 8: Validating with standard SPI...\n");
    aps6404_exit_quadmode();
    address = TEST_ADDR;
    int temp = fail_count;
    for (int i = 0; i < TEST_COUNT; i++) {
        aps6404_read(address, rx_buffer, TEST_SIZE);
        for (int j = 0; j < TEST_SIZE; j++) {
            if (rx_buffer[j] != tx_buffer[j]) {
                fail_count++;
            }
        }
        address += TEST_SIZE;
        memset(rx_buffer, 0, TEST_SIZE);
    }
    if (fail_count != temp) {
        printf("\tFailed (%i) mismatches\n", fail_count - temp);
    } else {
        printf("\tSuccess\n");
    }

    // Validate
    printf("Test 9: Validating with QSPI...\n");
    aps6404_enter_quadmode();
    address = TEST_ADDR;
    temp = fail_count;
    for (int i = 0; i < TEST_COUNT; i++) {
        aps6404_read(address, rx_buffer, TEST_SIZE);
        for (int j = 0; j < TEST_SIZE; j++) {
            if (rx_buffer[j] != tx_buffer[j]) {
                fail_count++;
            }
        }
        address += TEST_SIZE;
        memset(rx_buffer, 0, TEST_SIZE);
    }
    if (fail_count != temp) {
        printf("\tFailed (%i) mismatches\n", fail_count - temp);
    } else {
        printf("\tSuccess\n");
    }

    if (fail_count > 0) {
        printf("\nFailed with %i mismatches (%.2f%%)!\n", fail_count,
               (double)(100 * (((float)fail_count) / (TEST_SIZE * TEST_COUNT))));
        return E_FAIL;
    }

    printf("Success!\n");

    return err;
}

// *****************************************************************************
int main(void)
{
    int err = E_NO_ERROR;
    MXC_Delay(MXC_DELAY_SEC(2));

    // Set the Internal Primary Oscillator for the fastest system clock speed
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);

    printf("Initializing SRAM...\n");
    err = aps6404_init();
    if (err) {
        printf("Initalization failed! (error code %i)\n", err);
        return err;
    }

    // Reading the ID out is a good first step to validate that hardware
    // is connected and working properly.
    printf("Reading ID...\n");
    aps6404_id_t id;
    err = aps6404_read_id(&id);
    if (err) {
        printf("Failed to read expected SRAM ID!\n");
        return err;
    } else {
        printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", id.MFID,
               id.KGD, id.density, id.EID);
    }

    uint8_t tx_data[4] = { 1, 2, 3, 4 };
    uint8_t rx_buffer[4] = { 0, 0, 0, 0 };

    aps6404_enter_quadmode(); // Quad mode is faster
    aps6404_write(TEST_ADDR, tx_data, 4);
    aps6404_read(TEST_ADDR, rx_buffer, 4);

    for (int i = 0; i < 4; i++) {
        if (tx_data[i] != rx_buffer[i]) {
            printf("Simple R/W test failed at index %i!  Expected %i but got %i\n", i, tx_data[i],
                   rx_buffer[i]);
            return E_FAIL;
        }
    }

    // Run a more advanced test and validation.  SRAM is fully functional if this passes.
    err = test();
    return err;
}
