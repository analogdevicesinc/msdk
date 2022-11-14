/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
 * 
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 * @file        main.c
 * @brief       Configures and starts the RTC and demonstrates the use of the alarms.
 * @details     The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
 *              P2.17 (LED0) is toggled each time the sub-second alarm triggers.  The
 *              time-of-day alarm is set to 2 seconds.  When the time-of-day alarm
 *              triggers, the rate of the sub-second alarm is switched to 500 ms.  The
 *              time-of-day alarm is then rearmed for another 2 sec.  Pressing SW2 will
 *              output the current value of the RTC to the console UART.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

#include "bitmap.h"
#include "tft_ssd2119.h"

/***** Definitions *****/
#define LED_FLASH 0
#define LED_ALARM 0

#define BUTTON_SIZE_X 42 //
#define BUTTON_SIZE_Y 41 //

#define KEY_1 1
#define KEY_2 2

/***** Globals *****/
static int button1Pressed = 0;

/***** Functions *****/
void SysTick_Handler(void)
{
    // Do nothing here
}

void HTMR0_IRQHandler(void)
{
    if (MXC_HTMR_GetFlags(MXC_HTMR0) & MXC_F_HTMR_CTRL_ALDF) {
        LED_On(LED_ALARM);
        MXC_HTMR_ClearFlags(MXC_HTMR0, MXC_F_HTMR_CTRL_ALDF);
    }

    if (MXC_HTMR_GetFlags(MXC_HTMR0) & MXC_F_HTMR_CTRL_ALSF) {
        if (!button1Pressed) {
            LED_Toggle(LED_FLASH);
        }

        MXC_HTMR_ClearFlags(MXC_HTMR0, MXC_F_HTMR_CTRL_ALSF);
    }

    return;
}

static void setAlarm(void)
{
    MXC_HTMR_Stop(MXC_HTMR0);
    button1Pressed = 1;
    LED_Off(LED_ALARM);

    if (MXC_HTMR_SetLongAlarm(MXC_HTMR0, MXC_HTMR_GetLongCount(MXC_HTMR0) + 10000) != E_NO_ERROR) {
        printf("Failed to set Long Interval Alarm.\n");
    }

    MXC_TFT_Printf("Alarm Set\n");

    MXC_HTMR_Start(MXC_HTMR0);
}

static void resetAlarm(void)
{
    button1Pressed = 0;
    LED_Off(LED_ALARM);
    MXC_TFT_Printf("Alarm Cleared\n");
}

static void printTime(void)
{
    double count;

    /* The short count register is 20 bits long, however
     * the long count counter is incremented every 4096 (12 bits) short counts
     * so we need to take short count mod 4096 and add long count to
     * get a float representing the total number of long counts
     */
    count = (MXC_HTMR_GetShortCount(MXC_HTMR0) % 4096) / 4096.0;
    count += MXC_HTMR_GetLongCount(MXC_HTMR0);

    printf("\nCurrent Count %f\n", count);
}

static int setup_display(void)
{
    int x = 100;
    int y = 150;

    MXC_TFT_Init();
    MXC_TS_Init();
    //
    MXC_TS_Start();

    MXC_TFT_ShowImage(11, 7, logo_white_bg_white_bmp);

    MXC_TFT_ShowImage(x, y, key_1_bg_white_bmp);
    MXC_TS_AddButton(x, y, x + BUTTON_SIZE_X, y + BUTTON_SIZE_Y, KEY_1);

    x += BUTTON_SIZE_X + 40; // add 40pixel space
    MXC_TFT_ShowImage(x, y, key_2_bg_white_bmp);
    MXC_TS_AddButton(x, y, x + BUTTON_SIZE_X, y + BUTTON_SIZE_Y, KEY_2);

    // set up font
    MXC_TFT_SetFont(urw_gothic_13_grey_bg_white);

    // Set print area
    area_t print = { 20, 40, 300, 30 };
    MXC_TFT_ConfigPrintf(&print);

    return 0;
}

// *****************************************************************************
int main(void)
{
    int key;

    printf("\n******************** High Speed Timer Example ******************\n\n");
    printf("This example enables the HTMR and sets the short interval\n");
    printf("alarm to trigger every ~2^22 short interval counts (0.5sec)\n");
    printf("Pressing PB0 (P3.06) will print the current count to the console\n");
    printf("Pressing Button 1 will set the Long Interval alarm to light LED1 in 10000 counts\n\n");
    printf("Pressing Button 2 will clear the Long Interval alarm LED1\n\n");

    setup_display();

    MXC_NVIC_SetVector(HTMR0_IRQn, HTMR0_IRQHandler);
    NVIC_EnableIRQ(HTMR0_IRQn);

    /* Turn LED off initially */
    LED_Off(LED_ALARM);

    if (MXC_HTMR_Init(MXC_HTMR0, 0, 0) != E_NO_ERROR) {
        printf("Failed HTMR Initialization.\n");

        while (1) {}
    }

    if (MXC_HTMR_SetShortAlarm(MXC_HTMR0, 0xFFC7BFFF) != E_NO_ERROR) {
        printf("Failed to set short interval alarm\n");
    }

    MXC_HTMR_Start(MXC_HTMR0);
    printf("\nTimer started.\n\n");
    printTime();

    while (1) {
        // check touch screen key
        key = MXC_TS_GetKey();

        if (key > 0) {
            /* Show the time elapsed. */
            printTime();
            /* Delay for switch debouncing. */
            MXC_TMR_Delay(MXC_TMR0, MSEC(100));

            switch (key) {
            case KEY_1:
                setAlarm();
                break;

            case KEY_2:
                resetAlarm();
                break;

            default:
                break;
            }
        }
    }
}
