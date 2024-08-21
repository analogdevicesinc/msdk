/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
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
 * @brief   read and write sdhc
 * @details This example uses the sdhc and ffat to read/write the file system on
 *          an SD card. The Fat library used supports long filenames (see ffconf.h)
 *          the max length is 256 characters. It uses the CLI library for taking user
 *          user commands.
 *
 *          You must connect an sd card to the sd card slot.
 */

/***** Includes *****/
#include "board.h"
#include "cli.h"
#include "nvic_table.h"
#include "sdhc.h"
#include "uart.h"
#include "user-cli.h"
#include "sdhc_lib.h"
#include "gpio.h"
#include "mxc_sys.h"

mxc_gpio_cfg_t SDPowerEnablePin = { MXC_GPIO1, MXC_GPIO_PIN_15, MXC_GPIO_FUNC_OUT,
                                    MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO };

/******************************************************************************/
int main(void)
{
    mxc_sdhc_cfg_t cfg;
    int err;
    printf("\n\n***** MAX78000 SDHC FAT Filesystem Example *****\n");

    // Enable Power To Card
    printf("Enabling card power...\n");
    MXC_GPIO_Config(&SDPowerEnablePin);
    MXC_GPIO_OutClr(MXC_GPIO1, SDPowerEnablePin.mask);

    // Initialize SDHC peripheral
    printf("Initializing SDHC peripheral...\n");
    cfg.bus_voltage = MXC_SDHC_Bus_Voltage_3_3;
    cfg.block_gap = 0;
    cfg.clk_div =
        0x0b0; // Maximum divide ratio, frequency must be >= 400 kHz during Card Identification phase
    if (MXC_SDHC_Init(&cfg) != E_NO_ERROR) {
        printf("Unable to initialize SDHC driver.\n");
        return 1;
    }

    // wait for card to be inserted
    printf("Waiting for card to be inserted...\n");
    while (!MXC_SDHC_Card_Inserted()) {}
    printf("Card inserted.\n");

    // set up card to get it ready for a transaction
    printf("Initializing card...\n");
    if ((err = MXC_SDHC_Lib_InitCard(10)) == E_NO_ERROR) {
        printf("Card Initialized.\n");
    } else {
        printf("SDHC Library initialization failed with error %i\n", err);

        return -1;
    }

    if (MXC_SDHC_Lib_Get_Card_Type() == CARD_SDHC) {
        printf("Card type: SDHC\n");
    } else {
        printf("Card type: MMC/eMMC\n");
    }

    // Wait for any printfs to complete
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART))) {}

    // Initialize CLI
    if ((err = MXC_CLI_Init(MXC_UART_GET_UART(CONSOLE_UART), user_commands, num_user_commands)) !=
        E_NO_ERROR) {
        return err;
    }

    // Run CLI
    while (1) {}
}
