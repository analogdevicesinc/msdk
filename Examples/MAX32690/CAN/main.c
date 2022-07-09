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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_delay.h"
#include "mxc_device.h"
#include "board.h"
#include "can.h"
#include "dma.h"
#include "led.h"
#include "pb.h"

/***** Definitions *****/
// Select one of these
#define SYNC 			1	// Use blocking send and receive
#define ASYNC			0	// Use non-blocking send and receive
#define DMA				0	// Use DMA send and receive

#if (!(SYNC || ASYNC || DMA))
#error "You must select a send and receive method."
#elif ((SYNC && ASYNC) || (SYNC && DMA) || (ASYNC && DMA))
#error "Select only one send and receive method."
#endif

// Enable which functions are demonstrated
#define SEND_CAN 		1  	// Demonstrates sending a standard CAN 2.0 message
#define SEND_RTR		1	// Demonstrates sending an RTR message, disable by setting to 0
#define SEND_CANFD		1	// Demonstrates sending a CAN FD message, disable by setting to 0
#define RECEIVE_CAN 	1	// Demonstrates receiving a CAN message (any type)

/***** Globals *****/
int wait_for_msg;
uint8_t data_rx[64];
mxc_can_msg_info_t msg_rx;

/***** Functions *****/
void dma_callback(int ch, int err);
int numBytes(uint32_t dlc, uint32_t fdf, uint32_t rtr);
void printMsgInfo(void);
void msgRead(void);
void msgSend(uint32_t can_idx, mxc_can_msg_info_t* msg, uint8_t* data, uint8_t size);
void my_unit_cb(uint32_t can_idx, uint32_t event);
void my_obj_cb(uint32_t can_idx, uint32_t event);

// *****************************************************************************************************************************************************
int main(void)
{
	int err;

	printf("\n\n************************** CAN Example *********************************\n");
	printf("This example demonstrates how to perform various CAN transactions,\n");
	printf("namely, sending a CAN 2.0 message, an RTR message, and/or a CAN FD message,\n");
	printf("as well as receiving a CAN message (any type). These operations can be\n");
	printf("performed with either blocking, non-blocking or DMA methods (selectable\n");
	printf("with the macros defined above.)\n\n");
	printf("Connect CAN signals on header JH8 to CAN bus.\n\n");

  #if ASYNC || DMA
	NVIC_EnableIRQ(CAN0_IRQn);
  #endif //ASYNC
  #if DMA
	MXC_DMA_Init();
	NVIC_EnableIRQ(DMA0_IRQn);
	MXC_DMA_ReleaseChannel(0);
  #endif //DMA

	// Enable MXC_CAN0 to send and receive CAN messages
 	MXC_CAN_Init(0, MXC_CAN_OBJ_CFG_TXRX, my_unit_cb, my_obj_cb);
 	MXC_CAN_ObjectSetFilter(0, (MXC_CAN_FILT_CFG_MASK_ADD | MXC_CAN_FILT_CFG_SINGLE_EXT_ID), 0x1FFFFFFF, 0);	// Set receive filter to accept all extended IDs

	// Set bitrate
	MXC_CAN_SetBitRate(0, MXC_CAN_BITRATE_SEL_NOMINAL, 500000, MXC_CAN_BIT_SEGMENTS(7, 2, 2));		// Nominal bitrate 500kHz, TSEG1 - 7, TSEG2 - 2
	if((err = MXC_CAN_GetBitRate(0, MXC_CAN_BITRATE_SEL_NOMINAL)) != 500000) {
		LED_On(0);
		while(1);
	}
	MXC_CAN_SetBitRate(0, MXC_CAN_BITRATE_SEL_FD_DATA, 2000000, MXC_CAN_BIT_SEGMENTS(7, 2, 2));		// FD bitrate 2MHz, TSEG1 - 7, TSEG2 - 2
	if((err = MXC_CAN_GetBitRate(0, MXC_CAN_BITRATE_SEL_FD_DATA)) != 2000000) {
		LED_On(0);
		while(1);
	}

	mxc_can_msg_info_t msg_tx;
	uint8_t data_tx[64];
	for(int i = 0; i < 64; i++) {
		data_tx[i] = i;
	}

	printf("Press button SW2 to begin example.\n");
	while(!PB_Get(0));
	MXC_Delay(MXC_DELAY_MSEC(500));

  #if SEND_CAN
	/* Send CAN message with 6 data bytes */
	printf("\nSending standard CAN message...\n");
	msg_tx.msg_id = MXC_CAN_STANDARD_ID(0x123);
	msg_tx.rtr = 0;
	msg_tx.fdf = 0;
	msg_tx.brs = 0;
	msg_tx.esi = 0;
	msg_tx.dlc = 6;
	msgSend(0, &msg_tx, data_tx, 6);
  #endif

  #if SEND_RTR
	/* Send RTR message requesting 8 bytes of data */
	printf("Sending RTR message...\n");

	msg_tx.msg_id = MXC_CAN_STANDARD_ID(0x456);
	msg_tx.rtr = 1;
	msg_tx.fdf = 0;
	msg_tx.brs = 0;
	msg_tx.esi = 0;
	msg_tx.dlc = 8;
	msgSend(0, &msg_tx, data_tx, 8);
  #endif // SEND_RTR

  #if SEND_CANFD
	/* Send CAN FD message with 48 data bytes */
	printf("Sending CAN FD message...\n");

	msg_tx.msg_id = MXC_CAN_EXTENDED_ID(0x12345678);
	msg_tx.rtr = 0;
	msg_tx.fdf = 1;
	msg_tx.brs = 1;
	msg_tx.esi = 0;
	msg_tx.dlc = 14;
	msgSend(0, &msg_tx, data_tx, 48);
  #endif // SEND_CANFD

  #if RECEIVE_CAN
	/* Demonstrate message receive. */
	memset(&msg_rx, 0, sizeof(mxc_can_msg_info_t));
	memset(data_rx, 0, sizeof(data_rx));

	printf("\nReady to receive message.\n");
	msgRead();
	printMsgInfo();
  #endif

	printf("\n\nExample complete.\n");
	while(1);
}

// ************************************************************ Message Send ***************************************************************************
#if (SEND_CAN || SEND_RTR || SEND_CANFD)
void msgSend(uint32_t can_idx, mxc_can_msg_info_t* msg, uint8_t* data, uint8_t size)
{
	int err;
	mxc_can_req_t req;
	req.msg_info = msg;
	req.data = data;
	req.data_sz = size;

  #if SYNC
	if((err = MXC_CAN_MessageSend(0, &req)) < E_NO_ERROR) {
		LED_On(0);
		while(1);
	}
  #elif ASYNC
	wait_for_msg = 1;
	if((err = MXC_CAN_MessageSendAsync(0, &req)) < E_NO_ERROR) {
		LED_On(0);
		while(1);
	}

	while(wait_for_msg);
  #elif DMA
	wait_for_msg = 1;
	if((err = MXC_CAN_MessageSendDMA(0, &req)) < E_NO_ERROR) {
		LED_On(0);
		while(1);
	}

	while(wait_for_msg);
  #endif
}
#endif //SEND

// ************************************************************ Message Receive ************************************************************************
#if RECEIVE_CAN
int numBytes(uint32_t dlc, uint32_t fdf, uint32_t rtr)
{
    int num_bytes = 0;
    if(rtr) {
        return 0;
    }
    else if(dlc > 8 && fdf) {       // CAN FD message with more than 8 bytes of data
        switch(dlc & 0xF) {
            case 9:
            case 10:
            case 11:
            case 12:
                num_bytes = 8 + (dlc & 0x7) * 4;
                break;
            case 13:
                num_bytes = 32;
                break;
            case 14:
                num_bytes = 48;
                break;
            case 15:
                num_bytes = 64;
                break;
        }
    }
    else if(dlc > 8 && !fdf) {      // Normal CAN message with DLC greater than maximum number of data bytes, set to maximum
        num_bytes = 8;
    }
    else {                          // Normal CAN or CAN FD message with less than or equal to 8 bytes
        num_bytes = dlc;
    }

    return num_bytes;
}

void printMsgInfo() {
	printf("MSG ID: %s - 0x%x\n", (msg_rx.msg_id & MXC_CAN_MSG_INFO_IDE_BIT ? "29-bit ID" : "11-bit ID"),
								  (msg_rx.msg_id & MXC_CAN_MSG_INFO_IDE_BIT ? msg_rx.msg_id & ~MXC_CAN_MSG_INFO_IDE_BIT : msg_rx.msg_id));
	printf("RTR: %d\n", !!msg_rx.rtr);
	printf("FDF: %d\n", !!msg_rx.fdf);
	printf("BRS: %d\n", !!msg_rx.brs);
	printf("ESI: %d\n", !!msg_rx.esi);
	printf("DLC: %d\n", msg_rx.dlc);
	printf("Data Received:");

	int bytes_received = numBytes(msg_rx.dlc, msg_rx.fdf, msg_rx.rtr);
	for(int i = 0; i < bytes_received; i++) {
		if(!(i & 0x7)) {
			printf("\n");
		}
		printf("%x ", data_rx[i]);
	}
}

void msgRead()
{
	mxc_can_req_t req;
	req.msg_info = &msg_rx;
	req.data = data_rx;
	req.data_sz = sizeof(data_rx);

  #if SYNC
	if(MXC_CAN_MessageRead(0, &req) < E_NO_ERROR) {
		LED_On(0);
		while(1);
	}
  #elif ASYNC
	wait_for_msg = 1;
	if(MXC_CAN_MessageReadAsync(0, &req) < E_NO_ERROR) {
		LED_On(0);
		while(1);
	}

	while(wait_for_msg);
  #elif DMA
	//*** IMPORTANT ***: Set these fields to the expected values of the message to be received. The DMA transaction will be configured improperly if you fail to do this.
	msg_rx.msg_id =  MXC_CAN_EXTENDED_ID(0x255);
	msg_rx.rtr = 0;
	msg_rx.fdf = 0;
	msg_rx.brs = 0;
	msg_rx.esi = 0;
	msg_rx.dlc = 8;

	wait_for_msg = 1;
	if(MXC_CAN_MessageReadDMA(0, &req, dma_callback) != E_NO_ERROR) {
		LED_On(0);
		while(1);
	}

	while(wait_for_msg);
  #endif
}
#endif //RECEIVE

// ************************************************************ CAN Object Event Callback **************************************************************
void my_obj_cb(uint32_t can_idx, uint32_t event)
{
	if(can_idx == 0 && event == MXC_CAN_OBJ_EVT_TX_COMPLETE) {		// Message send complete
		printf("Message sent!\n");
		wait_for_msg = 0;
	}

	if(can_idx == 0 && event == MXC_CAN_OBJ_EVT_RX) {				// Message received
		printf("Message received!\n");
		LED_On(1);
		wait_for_msg = 0;
	}

	if(event == MXC_CAN_OBJ_EVT_RX_OVERRUN) {						// RX overrun
		LED_On(0);
		while(1);
	}
}

// ************************************************************ CAN Unit Event Callback ****************************************************************
void my_unit_cb(uint32_t can_idx, uint32_t event)
{
	if(can_idx == 0 && event != MXC_CAN_UNIT_EVT_ACTIVE) {			// Any unit event except for DMA becoming active
		LED_On(0);
		while(1);
	}
}

// ************************************************************ CAN Interrupt Handler ******************************************************************
#if ASYNC || DMA						// Need CAN handler for MessageSendDMA
void CAN0_IRQHandler(void)
{
	MXC_CAN_Handler(0);					// Moves message from RX FIFO to request structure
}
#endif // ASYNC

// ************************************************************ DMA Handlers ***************************************************************************
#if DMA
void DMA0_IRQHandler(void)				// Message ready in temporary buffer
{
	MXC_DMA_Handler();
}

void dma_callback(int ch, int err) 		// Need CAN handler for MessageReadDMA
{
	MXC_CAN_Handler(0);					// Moves message received from temporary buffer to the request structure
}
#endif //DMA
