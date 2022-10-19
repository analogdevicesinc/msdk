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

// ARM wrapper code

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "led.h"
#include "tmr.h"
#include "sema_regs.h"
#include "mxc.h"
#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif
#ifdef BOARD_EVKIT_V1
#include "tft_ssd2119.h"
#include "bitmap.h"
#endif

#define VERSION "3.0.1 (01/21/21)"

#define TFT_BUFF_SIZE 50 // TFT buffer size
#define NUM_OUTPUTS 21 // number of classes

extern volatile void const *__FlashStart_; // Defined in linker file
char buff[TFT_BUFF_SIZE];

#ifdef ENABLE_TFT
void TFT_Intro(void);
void TFT_Print(char *str, int x, int y, int font, int length);
void TFT_End(uint16_t words);
#ifdef BOARD_EVKIT_V1
int image_bitmap = ADI_256_bmp;
int font_1 = urw_gothic_12_white_bg_grey;
int font_2 = urw_gothic_13_white_bg_grey;
#endif
#ifdef BOARD_FTHR_REVA
int image_bitmap = (int)&img_1_rgb565[0];
int font_1 = (int)&SansSerif16x16[0];
int font_2 = (int)&SansSerif16x16[0];
#endif
#endif

#define MAILBOX_SIZE 16
__attribute__((section(".shared__at__mailbox"))) volatile uint32_t mail_box[MAILBOX_SIZE];

/* Set of detected words */
const char keywords[NUM_OUTPUTS][10] = { "UP",    "DOWN", "LEFT",   "RIGHT", "STOP",  "GO",
                                         "YES",   "NO",   "ON",     "OFF",   "ONE",   "TWO",
                                         "THREE", "FOUR", "FIVE",   "SIX",   "SEVEN", "EIGHT",
                                         "NINE",  "ZERO", "Unknown" };

void WakeISR(void)
{
    MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN & ~MXC_F_SEMA_IRQ0_CM4_IRQ;
}

int main(void)
{
#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
#endif
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    /* Switch to 100 MHz clock */
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

#if defined(BOARD_FTHR_REVA)
    /* Enable microphone power on Feather board */
    Microphone_Power(POWER_ON);
#endif

    printf("Analog Devices \nKeyword Spotting Demo\nVer. %s \n", VERSION);

#ifdef ENABLE_TFT
    MXC_Delay(500000);
    printf("\n*** Init TFT ***\n");
#ifdef BOARD_EVKIT_V1
    MXC_TFT_Init();
    MXC_TFT_ClearScreen();

    MXC_TFT_ShowImage(0, 0, image_bitmap);
    MXC_Delay(1000000);

    MXC_TFT_SetPalette(logo_white_bg_darkgrey_bmp);
    MXC_TFT_SetBackGroundColor(4);
    printf("if RED LED is not on, disconnect PICO SWD and powercycle!\n");
#endif
#ifdef BOARD_FTHR_REVA
    /* Initialize TFT display */
    MXC_TFT_Init(MXC_SPI0, 1, NULL, NULL);
    MXC_TFT_SetRotation(ROTATE_90);
    MXC_TFT_ShowImage(0, 0, image_bitmap);
    MXC_Delay(1000000);
    MXC_TFT_SetBackGroundColor(4);
    MXC_TFT_SetForeGroundColor(WHITE); // set chars to white
#endif

    printf("Waiting for PB1 press\n");
    TFT_Intro();
#else

    for (int i = 0; i < (1 << 27); i++) {}
    // Let debugger interrupt if needed

#endif

    MXC_FCR->urvbootaddr = (uint32_t)&__FlashStart_; // Set RISC-V boot address
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SMPHR); // Enable Sempahore clock
    MXC_NVIC_SetVector(RISCV_IRQn, WakeISR); // Set wakeup ISR
    MXC_GCR->pclkdis1 &= ~MXC_F_GCR_PCLKDIS1_CPU1; // Enable RISC-V clock

    mail_box[MAILBOX_SIZE - 1] = 0; // set it to zero to show it is clear
        //  printf("mailbox ARM: %x\n",&mail_box[0]);

    while (1) {
        int16_t max = 0; // soft_max output is 0->32767
        int16_t max_index = -1;
        int i = 0;

        //LED_On(0);
        __WFI(); // Let RISC-V run
        //LED_Off(0);

        max_index = mail_box[0];

        /* is there anything in the mail box? */
        if (mail_box[MAILBOX_SIZE - 1]) {
#ifdef ENABLE_TFT
            // update TFT message
            MXC_TFT_ClearScreen();
            memset(buff, 32, TFT_BUFF_SIZE);

            /* read mail box, Top 1 class*/
            max_index = mail_box[0];
            max = mail_box[1];
            TFT_Print(buff, 20, 30, font_2,
                      snprintf(buff, sizeof(buff), "%s (%0.1f%%)", keywords[max_index],
                              (double)100.0 * max / 32768.0));
            TFT_Print(buff, 1, 50, font_1, snprintf(buff, sizeof(buff), "__________________________ "));
            TFT_Print(buff, 1, 80, font_1, snprintf(buff, sizeof(buff), "Top classes:"));

            /* read mail box, Top 2-4 class*/
            for (i = 1; i < 5; i++) {
                max_index = mail_box[2 * i];
                max = mail_box[2 * i + 1];
                TFT_Print(buff, 20, 80 + 20 * i, font_1,
                          snprintf(buff, sizeof(buff), "%s (%0.1f%%)", keywords[max_index],
                                  (double)100.0 * max / 32768.0));
            }

            TFT_Print(buff, 20, 200, font_1,
                      snprintf(buff, sizeof(buff), "Sample Min: %d    Max: %d", (int32_t)mail_box[2 * i] - 128,
                              (int32_t)mail_box[2 * i + 1] - 128));
#endif
            /* clear mail box */
            mail_box[MAILBOX_SIZE - 1] = 0;
        }
    }

    return 0;
}

/************************************************************************************/
#ifdef ENABLE_TFT
void TFT_Intro(void)
{
    char buff[TFT_BUFF_SIZE];
    memset(buff, 32, TFT_BUFF_SIZE);
    TFT_Print(buff, 55, 10, font_2, snprintf(buff, sizeof(buff), "ANALOG DEVICES"));
    TFT_Print(buff, 35, 40, font_1, snprintf(buff, sizeof(buff), "Keyword Spotting Demo"));
    TFT_Print(buff, 70, 70, font_1, snprintf(buff, sizeof(buff), "Ver. %s", VERSION));
    TFT_Print(buff, 5, 110, font_1, snprintf(buff, sizeof(buff), "Following keywords can be"));
    TFT_Print(buff, 5, 135, font_1, snprintf(buff, sizeof(buff), "detected:"));
    TFT_Print(buff, 35, 160, font_1, snprintf(buff, sizeof(buff), "0...9, up, down, left, right"));
    TFT_Print(buff, 35, 185, font_1, snprintf(buff, sizeof(buff), "stop, go, yes, no, on, off"));
    TFT_Print(buff, 5, 210, font_2, snprintf(buff, sizeof(buff), "PRESS PB1 TO START!"));

    while (!PB_Get(0)) {}

    MXC_TFT_ClearScreen();
#ifdef BOARD_EVKIT_V1
    TFT_Print(buff, 20, 20, font_1, snprintf(buff, sizeof(buff), "Wait for RED LED to turn on"));
    TFT_Print(buff, 20, 50, font_1, snprintf(buff, sizeof(buff), "and start saying keywords..."));
    TFT_Print(buff, 20, 110, font_1, snprintf(buff, sizeof(buff), "If RED LED didn't turn on in"));
    TFT_Print(buff, 20, 140, font_1, snprintf(buff, sizeof(buff), "2 sec, disconnect SWD and"));
    TFT_Print(buff, 20, 170, font_1, snprintf(buff, sizeof(buff), "power cycle."));
#else
    TFT_Print(buff, 20, 50, font_1, snprintf(buff, sizeof(buff), "Start saying keywords..."));
#endif
}

/* **************************************************************************** */
void TFT_Print(char *str, int x, int y, int font, int length)
{
    // fonts id
    text_t text;
    text.data = str;
    text.len = length;
    MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

/* **************************************************************************** */
void TFT_End(uint16_t words)
{
    char buff[TFT_BUFF_SIZE];
    memset(buff, 32, TFT_BUFF_SIZE);
    MXC_TFT_ClearScreen();
    TFT_Print(buff, 70, 30, font_2, snprintf(buff, sizeof(buff), "Demo Stopped!"));
    TFT_Print(buff, 10, 60, font_1, snprintf(buff, sizeof(buff), "Number of words: %d ", words));
    TFT_Print(buff, 20, 180, font_1, snprintf(buff, sizeof(buff), "PRESS RESET TO TRY AGAIN!"));
}
#endif
