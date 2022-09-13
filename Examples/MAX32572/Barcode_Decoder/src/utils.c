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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "rtc.h"
#include "utils.h"

#pragma GCC optimize("-O0")

#define DEBUG_COMPORT MXC_UART1

/***************************** VARIABLES *************************************/

/************************    PUBLIC FUNCTIONS  *******************************/
void utils_delay_ms(unsigned int ms)
{
    MXC_Delay(ms * 1000UL);
}

unsigned int utils_get_time_ms(void)
{
    int sec;
    double subsec;
    unsigned int ms;

    subsec = MXC_RTC_GetSubSecond() / 4096.0;
    sec = MXC_RTC_GetSecond();

    ms = (sec * 1000) + (int)(subsec * 1000);

    return ms;
}

void utils_hexDump(const char *title, unsigned char *buf, unsigned int len)
{
    unsigned int i;

    /* Print title */
    if (title) {
        printf("%s", title);
    }

    /* Print buffer bytes */
    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
            printf("\n");
        }

        printf("%02X ", buf[i]);
    }

    printf("\n");
}

int utils_send_img_to_pc(uint8_t *img, uint32_t imgLen, int w, int h, uint8_t *pixelformat)
{
    int len;

    // start
    len = 5;
    MXC_UART_Write(DEBUG_COMPORT, (uint8_t *)"*STR*", &len);

    // w
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (w >> 8) & 0xff); // high byte
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (w >> 0) & 0xff); // low byte
    // h
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (h >> 8) & 0xff); // high byte
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (h >> 0) & 0xff); // low byte

    // format
    len = strlen((char *)pixelformat);
    MXC_UART_WriteCharacter(DEBUG_COMPORT, len & 0xff);
    MXC_UART_Write(DEBUG_COMPORT, pixelformat, &len);

    // imagelen
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (imgLen >> 24) & 0xff); // high byte
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (imgLen >> 16) & 0xff); // low byte
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (imgLen >> 8) & 0xff); // low byte
    MXC_UART_WriteCharacter(DEBUG_COMPORT, (imgLen >> 0) & 0xff); // low byte

#if 1

    while (imgLen) {
        len = imgLen;
        MXC_UART_Write(DEBUG_COMPORT, img, &len);
        img += len;
        imgLen -= len;
    }

#else // provide a slow communication if need

    while (imgLen) {
        int step_size = (imgLen > 2048) ? 2048 : imgLen;

        imgLen -= step_size;

        while (step_size) {
            len = step_size;
            MXC_UART_Write(DEBUG_COMPORT, img, &len);
            img += len;
            step_size -= len;
        }

        //MXC_Delay(1);
    }

#endif
    return 0;
}
