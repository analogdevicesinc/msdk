/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-09-05 16:46:11 -0500 (Wed, 05 Sep 2018) $
 * $Revision: 37695 $
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   FreeRTOSDemo
 * @details This example demonstrates FreeRTOS.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board.h"
#include "i2c_mngr.h"
#include "led.h"
#include "mxc_device.h"
#include "task.h"

/******************************************************************************/
/* Definitions */
/******************************************************************************/
#define I2C_BUS MXC_I2C0  // SCL P0_6; SDA P0_7
#define RX_BUF_LEN 100
#define TX_BUF_LEN 2
#define BUSY_WAIT_TIME_MS 5

// EEPROM0 Transaction Parameters
#define EEPROM0_SLAVE_ADDR 0x50					// EEPROM slave address
#define EEPROM0_BUS_SPEED 100000				// Frequency to drive SCL Clock at
#define EEPROM0_TIMEOUT_US 10000				// I2C Timeout
#define EEPROM0_TRANSACTION_INTERVAL_MS 100		// Transaction interval
#define EEPROM0_READ_ADDR 200					// Starting address of data read in EEPROM memory

// EEPROM1 Transaction Parameters
#define EEPROM1_SLAVE_ADDR 0x51
#define EEPROM1_BUS_SPEED 400000
#define EEPROM1_TIMEOUT_US 5000
#define EEPROM1_TRANSACTION_INTERVAL_MS 50
#define EEPROM1_READ_ADDR 1000

/******************************************************************************/
/* Globals */
/******************************************************************************/
i2c_mngr_slv_config_t eeprom0_config = {.i2c_instance = I2C_BUS,
                                         .slave_addr = EEPROM0_SLAVE_ADDR,
                                         .freq = EEPROM0_BUS_SPEED,
                                         .timeout = EEPROM0_TIMEOUT_US,
                                         .clock_stretching = 0};

i2c_mngr_slv_config_t eeprom1_config = {.i2c_instance = I2C_BUS,
                                         .slave_addr = EEPROM1_SLAVE_ADDR,
                                         .freq = EEPROM1_BUS_SPEED,
                                         .timeout = EEPROM1_TIMEOUT_US,
                                         .clock_stretching = 0};
uint8_t eeprom0_rx_buf[RX_BUF_LEN];
uint8_t eeprom1_rx_buf[RX_BUF_LEN];
uint8_t eeprom0_tx_buf[TX_BUF_LEN];
uint8_t eeprom1_tx_buf[TX_BUF_LEN];


/******************************************************************************/
/* Functions */
/******************************************************************************/
void execute_transaction(i2c_mngr_txn_t *t, uint32_t loop_interval) {
	int error, cnt = 0;

	// Spin until I2C transaction performed
	do {
		error = I2C_MNGR_Transact(t);
		vTaskDelay(BUSY_WAIT_TIME_MS);
		cnt += BUSY_WAIT_TIME_MS;
	} while (error == E_BUSY);

	// Check for errors in transaction
	if (error != E_NO_ERROR) {
		printf("Transaction failed with error: %i\n", error);
	}

	// Delay task if wait time hasn't exceeded task execution period
	if (cnt < loop_interval) {
		vTaskDelay(loop_interval - cnt);  // Delay for time remaining in task execution period
	}
}


void vEEPROM0_Task(void *pvParameters) {
	while (1) {
		LED_Toggle(0);

		// Write EEPROM address
		eeprom0_tx_buf[0] = EEPROM0_READ_ADDR >> 8;
		eeprom0_tx_buf[1] = EEPROM0_READ_ADDR & 0xFF;

		// Set transaction parameters for EEPROM0
		i2c_mngr_txn_t txn = {.slave_config = &eeprom0_config,
                              .p_tx_data = eeprom0_tx_buf,
						      .tx_len = TX_BUF_LEN,
						      .p_rx_data = eeprom0_rx_buf,
						      .rx_len = RX_BUF_LEN};

		// Do EEPROM0 read
		execute_transaction(&txn, EEPROM0_TRANSACTION_INTERVAL_MS);
	}
}


void vEEPROM1_Task(void *pvParameters) {
	while (1) {
		LED_Toggle(1);

		// Write EEPROM address
		eeprom1_tx_buf[0] = EEPROM1_READ_ADDR >> 8;
		eeprom1_tx_buf[1] = EEPROM1_READ_ADDR & 0xFF;

		// Set transaction parameters for EEPROM1
		i2c_mngr_txn_t txn = {.slave_config = &eeprom1_config,
                              .p_tx_data = eeprom1_tx_buf,
							  .tx_len = TX_BUF_LEN,
							  .p_rx_data = eeprom1_rx_buf,
							  .rx_len = RX_BUF_LEN};

		// Do EEPROM1 read
		execute_transaction(&txn, EEPROM1_TRANSACTION_INTERVAL_MS);
	}
}


int main(void) {
	printf("\n\n***************** I2C Transaction Manager Demo *****************\n");
	printf("Data is read from EEPROM0 every %dms with an I2C bus frequency\n", EEPROM0_TRANSACTION_INTERVAL_MS);
	printf("of %dHz. And data is read from EEPROM1 every %dms with an\n", EEPROM0_BUS_SPEED, EEPROM1_TRANSACTION_INTERVAL_MS);
	printf("I2C bus frequency of %dHz.\n\n", EEPROM1_BUS_SPEED);

	printf("LED0 is toggled each time the read from EEPROM0 is executed and\n");
	printf("LED1 is toggled each time the read from EEPROM1 is executed.\n\n");


	// Initialize I2C Manager
	I2C_MNGR_Init();

	/* Configure tasks */
	if ((xTaskCreate(vEEPROM0_Task, (const char *)"EEPROM0", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) ||
			(xTaskCreate(vEEPROM1_Task, (const char *)"EEPROM1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS)) {
		printf("xTaskCreate() failed to create a task.\n");
	} else {
		/* Start scheduler */
		printf("Starting scheduler.\n");
		vTaskStartScheduler();
	}

	/* This code is only reached if the scheduler failed to start */
	printf("ERROR: FreeRTOS did not start due to above error!\n");
	while (1) {
		__NOP();
	}

	/* Quiet GCC warnings */
	return -1;
}
