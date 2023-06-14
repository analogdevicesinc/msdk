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

/**
 * @file    main.c
 * @brief   HART UART Example
 * @details This example configures the HART UART to transmit some data, loopback
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h> // For rand()
#include "mxc_device.h"
#include "board.h"
#include "mxc_delay.h"
#include "afe.h"
#include "hart_uart.h"
#include "uart.h"

/***** Definitions *****/
// Uses hart_uart_get_received_packet and hart_uart_send to manage HART messages
// If not defined, example uses SAP(Service Access Points) functions and callbacks
#define INTERRUPT_DRIVEN_LOOPBACK

#define HART_BANNER_LEN 28
#define HART_ECHO_BAN_LEN 22
#define HART_BUFFER_MAX_LEN 512

#define AFE_TIMER_INSTANCE MXC_TMR1

/***** Globals *****/

uint8_t g_hart_rx_buffer[HART_BUFFER_MAX_LEN];
uint8_t g_hart_tx_buffer[HART_BUFFER_MAX_LEN];

uint32_t g_rx_buffer_index = 0;
uint32_t g_tx_buffer_index = 0;

uint32_t g_rx_len = 0;
uint32_t g_tx_len = 0;

uint32_t g_rx_complete = 0;
uint32_t g_tx_complete = 0;

uint32_t g_rx_active = 0;
uint32_t g_tx_active = 0;

/***** Functions *****/
// NOTE: Need this strong reference for SysTick_Handler.
//  There is a __weak one defined in mxc_delay.c but it may not be selected
//  by the compiler as there are multiple __weak definitions of it.
void SysTick_Handler(void)
{
    MXC_DelayHandler();
}

void afe_spi_timer_irq_cb_test(void)
{
    printf("SPI Timer CB\n");
}

void hart_timer_irq_cb_test(void)
{
    printf("HART Timer CB\n");
}

void hart_sap_reset_confirm_callback(void)
{
    // Confirms a reset has been completed
    printf("RESET.confirm CB.\n");
}

void hart_sap_enable_confirm_callback(uint32_t state)
{
    // Indicates a change in RTS
    printf("ENABLE.confirm CB. State: 0x%02X\n", state);

#ifndef INTERRUPT_DRIVEN_LOOPBACK
    // Update our global active boolean
    if (state == HART_STATE_TRANSMIT) {
        // Indicates that RTS is Low
        g_tx_active = 1;
        g_tx_buffer_index = 0;
        g_tx_len = 0;
    } else {
        g_tx_active = 0;
        g_tx_complete = 1;
    }
#endif
}

void hart_sap_enable_indicate_callback(uint32_t state)
{
    // Indicates a change in Carrier Detect
    printf("ENABLE.Indicate CB. State: 0x%02X\n", state);

#ifndef INTERRUPT_DRIVEN_LOOPBACK
    // Update our global active boolean
    if (state == HART_STATE_RECEIVE_ACTIVE) {
        // Indicates that CD is high
        g_rx_active = 1;
        g_rx_buffer_index = 0;
        g_rx_len = 0;
    } else {
        g_rx_active = 0;
        g_rx_complete = 1;
    }
#endif
}

void hart_sap_data_confirm_callback(uint8_t data)
{
    // Indicates this data byte was written to fifo
    //  Generates too much debug noise therefore disabled.
    // printf("DATA.confirm Data: 0x%02X\n", data);

    // Currently doing nothing here
}

void hart_sap_data_indicate_callback(uint8_t data)
{
    // This is a properly received data byte
    printf("DATA.indicate CB. Data: 0x%02X\n", data);

#ifndef INTERRUPT_DRIVEN_LOOPBACK
    // Add this data to our receive buffer
    if (g_rx_active) {
        if (g_rx_buffer_index < HART_BUFFER_MAX_LEN) {
            g_hart_rx_buffer[g_rx_buffer_index++] = data;
            g_rx_len++;
        }
    }
#endif
}

void hart_sap_error_indicate_callback(uint8_t status, uint8_t data)
{
    // This or a byte was received with a communications error (Parity, OV, Framing)
    printf("Error.indicate CB. ST: 0x%02X Data: 0x%02X\n", status, data);

    // Just dropping any error bytes for now
}

// Strong definitions of GPIO irq handlers to ensure proper vector installation
void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}

// *****************************************************************************
int main(void)
{
    int status = 0;
    uint8_t hart_message[HART_BANNER_LEN] = { "\n\rHello from MAX32675 HART\n\r" };
    uint8_t hart_echo_ban[HART_ECHO_BAN_LEN] = { "\n\rEcho from MAX32675: " };

    int i = 0;
    uint32_t buff_index = 0;

#ifdef INTERRUPT_DRIVEN_LOOPBACK
    int prev_cd_stat = 1;
    uint32_t comm_errors = 0;
    uint32_t rx_length = 0;
#endif

    hart_uart_callbacks_t hart_sap_cbs;

    hart_sap_cbs.reset_confirm_cb = (reset_confirm_callback_t)hart_sap_reset_confirm_callback;
    hart_sap_cbs.enable_confirm_cb = (enable_confirm_callback_t)hart_sap_enable_confirm_callback;
    hart_sap_cbs.enable_indicate_cb = (enable_indicate_callback_t)hart_sap_enable_indicate_callback;
    hart_sap_cbs.data_confirm_cb = (data_confirm_callback_t)hart_sap_data_confirm_callback;
    hart_sap_cbs.data_indicate_cb = (data_indicate_callback_t)hart_sap_data_indicate_callback;
    hart_sap_cbs.error_indicate_cb = (error_indicate_callback_t)hart_sap_error_indicate_callback;

    status = afe_load_trims(AFE_TIMER_INSTANCE);
    if (status != E_NO_ERROR) {
        printf("Error during afe load trims: %d\n", status);
        while (1) {}
    }

    status = hart_uart_setup(NORMAL_HART_TRANSCEIVE_MODE);

    if (status != E_NO_ERROR) {
        printf("Failed to setup HART UART Error: %d. Halting...\n", status);
        while (1) {}
    }

    hart_uart_setup_saps(hart_sap_cbs);

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

    // Note banner strings are NOT null terminated
    printf("Sending HART Banner Now...\n");

    status = hart_uart_send(hart_message, HART_BANNER_LEN);
    if (status != E_NO_ERROR) {
        printf("Failed to send HART Banner Error: %d. Halting...\n", status);
        while (1) {}
    }

    while (!hart_uart_check_transmit_complete()) {}

    printf("\nWaiting 2 seconds..\n");
    MXC_Delay(SEC(2));

    printf("\nReady to Echo HART Messages...\n");

#ifdef INTERRUPT_DRIVEN_LOOPBACK
    //
    // Infinite HART UART loopback using interrupts
    //
    while (1) {
        status = hart_uart_get_received_packet(g_hart_rx_buffer, &rx_length, &comm_errors);

        if (status == E_BUSY) {
            if (prev_cd_stat != status) {
                // HART Carrier Detected
                printf("HART Carrier ACTIVE\n");
                prev_cd_stat = status;
            }
        } else if (status == E_NONE_AVAIL) {
            // No HART Carrier Detected
            if (prev_cd_stat != status) {
                printf("No HART Carrier Detected\n");
                prev_cd_stat = status;
            }
        } else if (status != E_SUCCESS) {
            printf("HART Error during reception: %d\n", status);

            if (status == E_COMM_ERR) {
                printf("E_COMM_ERROR: 0x%02X\n", comm_errors);
            }
        } else if (status == E_SUCCESS) {
            if (comm_errors != 0) {
                printf("HART UART communications errors during reception: 0x%02X\n", comm_errors);
            }

            if (rx_length > 0) {
                // Got a response
                printf("HEX received:\n");
                for (i = 0; i < rx_length; i++) {
                    printf("0x%02X ", g_hart_rx_buffer[i]);
                }
                printf("\n");

                printf("\nASCII received: ");
                for (i = 0; i < rx_length; i++) {
                    printf("%c", g_hart_rx_buffer[i]);
                }
                printf("\n");

                //
                // Now reply to sender (Mostly Echo)
                //

                // NOTE: incoming may or may not already have preamble
                buff_index = 0;
                memcpy(g_hart_tx_buffer + buff_index, hart_echo_ban, HART_ECHO_BAN_LEN);
                buff_index += HART_ECHO_BAN_LEN;
                memcpy(g_hart_tx_buffer + buff_index, g_hart_rx_buffer, rx_length);

                status = hart_uart_send(g_hart_tx_buffer, HART_ECHO_BAN_LEN + rx_length);

                if (status != E_SUCCESS) {
                    printf("Failed to send HART message. Error: %d\n", status);
                } else {
                    while (!hart_uart_check_transmit_complete()) {}
                    printf("HART echo sent\n");
                }
            }
        }
    }
#else // END of INTERRUPT_DRIVEN_LOOPBACK
    //
    // Infinite HART UART loopback using SAP (Service Access Points)
    //
    while (1) {
        if (g_rx_active) {
            // Active carrier detected, attempting to receive data
            // Allow SAP callback to handle
        } else if (g_tx_active) {
            // Actively sending data out

            // This means there may be room for more data
            //  We are driving the transmission from here for now.
            if (g_tx_buffer_index < g_tx_len) {
                // Not finished transmitting, load next byte
                // NOTE: Assuming the byte passed fits in the fifo,
                status = hart_sap_data_request(g_hart_tx_buffer[g_tx_buffer_index]);

                if (status == E_SUCCESS) {
                    // Next byte successfully loaded in FIFO, update index
                    g_tx_buffer_index++;
                } else {
                    printf("TX FIFO Full\n");
                }
            }
            // Else no more bytes to send
        } else if (g_tx_complete) {
            // Transmission is complete
            printf("TX completed.\n");
            g_tx_complete = 0;

            // NOTE: HART UART driver automatically releases RTS after TX is complete
        } else if (g_rx_complete) {
            // Reception is complete
            printf("RX completed.\n");
            g_rx_complete = 0;

            if (g_rx_len > 0) {
                // Got a response
                printf("HEX received:\n");
                for (i = 0; i < g_rx_len; i++) {
                    printf("0x%02X ", g_hart_rx_buffer[i]);
                }
                printf("\n");

                printf("\nASCII received: ");
                for (i = 0; i < g_rx_len; i++) {
                    printf("%c", g_hart_rx_buffer[i]);
                }
                printf("\n");

                //
                // Now reply to sender (Mostly Echo)
                //

                // NOTE: incoming may or may not already have preamble
                buff_index = 0;
                memcpy(g_hart_tx_buffer + buff_index, hart_echo_ban, HART_ECHO_BAN_LEN);
                buff_index += HART_ECHO_BAN_LEN;
                memcpy(g_hart_tx_buffer + buff_index, g_hart_rx_buffer, g_rx_len);

                hart_sap_enable_request(HART_STATE_TRANSMIT);

                // Set length to transmit
                g_tx_len = buff_index + g_rx_len;

                // Initiate transmission
                status = hart_sap_data_request(g_hart_tx_buffer[g_tx_buffer_index++]);
                if (status != E_SUCCESS) {
                    printf("Got an error for first transmission byte: %d\n");
                    while (1) {}
                }
            }
        } else {
            // Not receiving or sending, just idling
        }
    } // End of while (1)
#endif // END of INTERRUPT_DRIVEN_LOOPBACK ELSE
}
