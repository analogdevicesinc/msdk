/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
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
 * @file    main.c
 * @brief   Demonstrates SRAM drivers for the N01S830HA on the MAX78000FTHR
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
#include "N01S830HA.h"

/***** Definitions *****/
#define TEST_SIZE 4096
#define TEST_ADDR 0x00000
// ^ Max address is 0x1FFFF

/***** Globals *****/
int g_sw_overhead = 0;
#define TIME(x, output)                                        \
    {                                                          \
        MXC_TMR_SW_Start(MXC_TMR0);                            \
        (x);                                                   \
        (elapsed) = MXC_TMR_SW_Stop(MXC_TMR0) - g_sw_overhead; \
    }

/***** Functions *****/

bool validate(uint8_t *expected, uint8_t *received, int len)
{
    return memcmp(expected, received, len) == 0;
}

// *****************************************************************************
int main(void)
{
    int err = E_NO_ERROR;
    unsigned int elapsed = 0;

    uint8_t tx_buffer[TEST_SIZE];
    uint8_t rx_buffer[TEST_SIZE];

    MXC_Delay(MXC_DELAY_SEC(2));

    // Set the Internal Primary Oscillator for the fastest system clock speed
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);

    printf("QSPI SRAM Test:\n");
    printf("\tTest Address: 0x%x\n", TEST_ADDR);
    printf("\tTest size: %i bytes\n", TEST_SIZE);
    printf("\tTest speed: %i Hz\n", FASTSPI_SPEED);

    // Time the measurement overhead of our measurement functions
    MXC_TMR_SW_Start(MXC_TMR0);
    g_sw_overhead = MXC_TMR_SW_Stop(MXC_TMR0);

    // Benchmark internal memory write
    TIME(memset(tx_buffer, 0, TEST_SIZE), elapsed);
    printf("(Benchmark) Wrote %i bytes to internal SRAM in %ius\n", TEST_SIZE, elapsed);

    // Initialize test pattern
    for (int i = 0; i < TEST_SIZE; i++) {
        tx_buffer[i] = i % 256;
    }
    memset(rx_buffer, 0, TEST_SIZE);

    // =====================================================
    // SRAM Init
    if ((err = N01S830HA_init()) != E_NO_ERROR) {
        printf("RAM init failed!  Error %i\n", err);
        return err;
    }

    // =====================================================
    // Standard SPI
    printf("Test 1: Standard SPI write...\n");
    TIME(N01S830HA_write(TEST_ADDR, tx_buffer, TEST_SIZE), // SRAM Write
         elapsed);
    printf("\tDone (%i bytes in %ius)\n", TEST_SIZE, elapsed);

    // Read and validate
    printf("Test 2: Standard SPI read...\n");
    TIME(N01S830HA_read(TEST_ADDR, rx_buffer, TEST_SIZE), // SRAM Read
         elapsed)
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    if (!validate(rx_buffer, tx_buffer, TEST_SIZE)) {
        printf("\tValidation failed!\n");
        return E_FAIL;
    }
    printf("\tSuccess.\n");
    // =====================================================

    // Invert test pattern - this ensures every bit has to be updated in the next write
    for (int i = 0; i < TEST_SIZE; i++) {
        tx_buffer[i] = ~tx_buffer[i];
    }

    // =====================================================
    // QSPI
    printf("Test 3: QSPI write...\n");

    N01S830HA_enter_quadmode(); // Enter quad mode

    TIME(N01S830HA_write(TEST_ADDR, tx_buffer, TEST_SIZE), // SRAM Write
         elapsed);
    printf("\tDone (%i bytes in %ius)\n", TEST_SIZE, elapsed);

    // Read and validate
    printf("Test 4: QSPI read...\n");
    TIME(N01S830HA_read(TEST_ADDR, rx_buffer, TEST_SIZE), // SRAM Read
         elapsed)
    printf("\tRead finished (%i bytes in %ius)\n", TEST_SIZE, elapsed);
    printf("\tChecking for mismatches...\n");
    if (!validate(rx_buffer, tx_buffer, TEST_SIZE)) {
        printf("\tValidation failed!\n");
        return E_FAIL;
    }
    printf("\tSuccess.\n");
    // =====================================================

    return E_NO_ERROR;
}
