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

/**
 * @file    main.c
 * @brief   HART UART Example
 * @details This example configures the HART UART to transmit some data, loopback
 */

/***** Includes *****/
#include "afe.h"
#include "board.h"
#include "hart_uart.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "uart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h> // For rand()
#include <string.h>

/***** Definitions *****/

#define HART_BANNER_LEN 26
#define HART_ECHO_BAN_LEN 21
#define HART_BUFFER_MAX_LEN 512

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    int retval = 0;
    uint8_t hart_message[HART_BANNER_LEN] = { "\nHello from MAX32675 HART\n" };
    uint8_t hart_echo_ban[HART_ECHO_BAN_LEN] = { "\nEcho from MAX32675: " };

    uint8_t hart_rx_buffer[HART_BUFFER_MAX_LEN];
    uint8_t hart_tx_buffer[HART_BUFFER_MAX_LEN];

    uint32_t rx_length = 0;
    int i = 0;
    int prev_cd_stat = 1;

    printf("\n\n\n\n\nMAX32675 HART UART Example\n\n");

    printf("This example transmits a basic welcome message via the HART uart\n");
    printf("Then it enters a HART loop back mode, waiting to receive a message.\n");
    printf("This examples requires use of another HART modem such as the HCF-Tool-35\n");
    printf("From the HART Physical Layer test kit.\n");
    printf("The modems may be connected across a 250Ohm load resistor.\n");
    printf("For basic operation of the other HART modem, a terminal emulator is required.\n");
    printf("it should be able to toggle RTS manually.  This example was tested with\n");
    printf("Roger Meier's CoolTerm.\n\n");
    printf("Setup the terminal at 1200 baud, with 1 stop bit, ODD parity, and connect.\n");
    printf("Upon execution of this example a hello message is sent via HART then loopback.\n");
    printf(
        "To send via CoolTerm SET RTS via mouse click, type message, then CLEAR RTS via mouse\n");
    printf("NOTE: Set is indicated by the RTS button color as light green, or lit.\n");
    printf("The message will be returned with \"Echo from MAX32675\" appended.\n\n\n");

    retval = hart_uart_setup(NORMAL_HART_TRANSCEIVE_MODE);
    if (retval != E_NO_ERROR) {
        printf("Failed to setup HART UART Error: %d. Halting...\n", retval);
        while (1) { }
    }

    // Note banner strings are NOT null terminated
    printf("Sending HART Banner Now...\n");

    retval = hart_uart_send(hart_message, HART_BANNER_LEN);
    if (retval != E_NO_ERROR) {
        printf("Failed to send HART Banner Error: %d. Halting...\n", retval);
        while (1) { }
    }

    printf("\nReady to Echo HART Messages...\n");

    //
    // Infinite HART UART loopback
    //
    while (1) {
        retval = hart_uart_get_received_packet(hart_rx_buffer, &rx_length);

        if (retval == E_BUSY) {
            if (prev_cd_stat != retval) {
                // HART Carrier Detected
                printf("HART Carrier ACTIVE\n");
                prev_cd_stat = retval;
            }
        } else if (retval == E_NONE_AVAIL) {
            // No HART Carrier Detected
            if (prev_cd_stat != retval) {
                printf("No HART Carrier Detected\n");
                prev_cd_stat = retval;
            }
        } else if (retval == E_SUCCESS) {
            if (rx_length > 0) {
                // Got a response
                printf("HEX received:\n");
                for (i = 0; i < rx_length; i++) {
                    printf("0x%02X ", hart_rx_buffer[i]);
                }
                printf("\n");

                printf("\nASCII received: ");
                for (i = 0; i < rx_length; i++) {
                    printf("%c", hart_rx_buffer[i]);
                }
                printf("\n");

                //
                // Now reply to sender (Mostly Echo)
                //
                memcpy(hart_tx_buffer, hart_echo_ban, HART_ECHO_BAN_LEN);
                memcpy(hart_tx_buffer + HART_ECHO_BAN_LEN, hart_rx_buffer, rx_length);

                retval = hart_uart_send(hart_tx_buffer, HART_ECHO_BAN_LEN + rx_length);
                if (retval != E_SUCCESS) {
                    printf("Failed to send HART message. Error: %d\n", retval);
                } else {
                    printf("HART echo sent\n");
                }
            }
        }
    }
}
