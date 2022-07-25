/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
 * @brief   MAX32572 FreeRTOS Demo Example!
 *
 * @details
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "MAX32xxx.h"

#include "state.h"
#include "message.h"

//
#define mainQUEUE_SIZE (4)

/********************************* 		VARIABLES	 *************************/
//
extern void vAnimTask(void* pvParameter);
extern void vGetATRTask(void* pvParameter);
extern void vGetMSRTask(void* pvParameter);
extern void vGetTSTask(void* pvParameter);
extern void vGetKEYTask(void* pvParameter);
extern void vGetNFCTask(void* pvParameter);

/* The queue used to send strings to the print task for display on the LCD. */
xQueueHandle xQueueMain;

/******************************   STATIC FUNCTIONS  **************************/
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    while (1)
        ;
}

static int system_init(void)
{
    int ret = 0;

    TFT_Init();

    TFT_SetBackGroundColor(0);
    TFT_ShowImage(52, 87, maxim_integrated_large_bmp);

    //
    MXC_TS_Init();
    MXC_TS_Start();

    return ret;
}

static void vMainTask(void* pvParameters)
{
    (void)pvParameters;

    message_t mMessage;
    State* state;
    int ret;
    unsigned int wait_time       = 0;
    unsigned int total_idle_time = 0;
    unsigned int max_idle_time   = 15000;

    ret = xTaskCreate(vGetTSTask, "GetTS", 500, NULL, 4, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

    ret = xTaskCreate(vGetATRTask, "GetATR", 1000, NULL, 4, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

#ifndef MN_EvKit_V1
    ret = xTaskCreate(vGetKEYTask, "GetKey", 500, NULL, 4, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

    ret = xTaskCreate(vGetNFCTask, "GetNFC", 1000, NULL, 4, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

    ret = xTaskCreate(vGetMSRTask, "GetMSR", 1000, NULL, 4, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }
#endif

    state_init();

    while (1) {
        state = state_get_current();

        wait_time = state->timeout / 10;
        if (wait_time == 0) {
            wait_time = 100; //100 * 10ms
        }

        if (xQueueReceive(xQueueMain, &mMessage, (wait_time * configTICK_RATE_10ms))) {
            switch (mMessage.pcType) {
                case 'T': // Touch screen
                    if (state->prcss_key)
                        state->prcss_key(mMessage.pcMessage[0]); // process touch screen keys
                    break;
                case 'K': // Keyboard
                    if (state->prcss_key)
                        state->prcss_key(mMessage.pcMessage[0]); // process keypad keys
                    break;
                case 'M': // MSR
                    if (state->prcss_msr)
                        state->prcss_msr(mMessage.pcMessage, mMessage.len);
                    break;
                case 'N': // NFC
                    if (state->prcss_nfc)
                        state->prcss_nfc(mMessage.pcMessage, mMessage.len);
                    break;
                default:
                    break;
            }
            // reset total idle time
            total_idle_time = 0;
        } else {
            // check tick function
            if (state->tick) {
                ret = state->tick();
                if (ret == 0) { // means tick function is used, do not switch idle state
                    total_idle_time = 0;
                }
            }

            // check total idle time
            total_idle_time += (wait_time * 10);
            if (total_idle_time >= max_idle_time) {
                state_set_current(get_idle_state());
                total_idle_time = 0;
            }
        }
    }
}

/******************************   PUBLIC FUNCTIONS  **************************/
int main(void)
{
    long ret;

    printf("\n************************** MAX32572 Demo Example **************************\n\n");
    printf("This example interact with user\n");
    printf(
        "Depend on the user selection on TFT display, some functionality of EvKit can be tested\n");
    printf("Please follow instruction on TFT Display\n");
    printf("Note:\n"
           "\tMSR: VBAT_SEL need to be connected to 3.3V, VDD_MSR need to be connected\n"
           "\tSmartCard can be configured to 5V mode (Class A) or 3V mode (Class B),\n"
           "\t		To configure 5V mode:\n"
           "\t			1- On EvKit connect SC_PWR_SEL jumper to 5V\n"
           "\t			2- In demo_config_h file update SMARTCARD_EXT_AFE_Voltage to 5V\n"
           "\t			3- Rebuild project and load it\n"
           "\t		To configure 3V mode:\n"
           "\t			1- On EvKit connect SC_PWR_SEL jumper to 3V\n"
           "\t			2- In demo_config_h file update SMARTCARD_EXT_AFE_Voltage to 3V\n"
           "\t			3- Rebuild project and load it\n");

    /* Configure the clocks, UART and GPIO. */
    ret = system_init();
    if (ret) {
        while (1)
            ;
    }

    /* Initialize SKBD port with default configurations */
    xQueueMain = xQueueCreate(mainQUEUE_SIZE, sizeof(message_t));

    /* Start the tasks defined within the 5000. */
    ret = xTaskCreate(vMainTask, "Main", 1000, NULL, 2, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

    /* Start the tasks defined within the file. */
    ret = xTaskCreate(vAnimTask, "Logo_Anim", 500, NULL, 1, NULL);
    if (ret != pdPASS) {
        while (1)
            ;
    }

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient heap to start the scheduler.*/
    return 0;
}
