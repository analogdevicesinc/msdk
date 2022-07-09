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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "board.h"
#include "mxc_delay.h"
#include "uart.h"
#include "rtc.h"
#include "utils.h"

/***************************** VARIABLES *************************************/


/************************    PUBLIC FUNCTIONS  *******************************/

uint32_t utils_get_time_ms(void)
{
    int sec;
    double subsec;
    uint32_t ms;

    subsec = MXC_RTC_GetSubSecond() / 4096.0;
    sec = MXC_RTC_GetSecond();

    ms = (sec * 1000) + (int)(subsec * 1000);

    return ms;
}

static void utils_send_byte(mxc_uart_regs_t* uart, uint8_t value)
{
    while (MXC_UART_WriteCharacter(uart, value) == E_OVERFLOW) { }
}

void utils_send_bytes(mxc_uart_regs_t* uart, uint8_t* ptr, int length)
{
    int i;

    for (i = 0; i < length; i++) {
        utils_send_byte(uart, ptr[i]);
    }
}


#pragma GCC optimize ("-O0")

#define DEBUG_COMPORT   MXC_UART0

/***************************** VARIABLES *************************************/


/************************    PUBLIC FUNCTIONS  *******************************/

void utils_hexDump(const char* title, uint8_t* buf, uint32_t len)
{
    uint32_t i;

    // Display the title
    if (title) {
        printf("%s", title);
    }

    // Display the buffer bytes
    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
            printf("\n");
        }

        printf("%02X ", buf[i]);
    }

    printf("\n");
}


int utils_send_img_to_pc(uint8_t* img, uint32_t imgLen, int w, int h, uint8_t* pixelformat)
{
    int len;

    // Transmit the start token
    len = 5;
    utils_send_bytes(DEBUG_COMPORT, (uint8_t*)"*STR*", len);

    // Transmit the width of the image
    utils_send_byte(DEBUG_COMPORT, (w >> 8) & 0xff); // high byte
    utils_send_byte(DEBUG_COMPORT, (w >> 0) & 0xff); // low byte
    // Transmit the height of the image
    utils_send_byte(DEBUG_COMPORT, (h >> 8) & 0xff); // high byte
    utils_send_byte(DEBUG_COMPORT, (h >> 0) & 0xff); // low byte

    // Transmit the pixel format of the image
    len = strlen((char*)pixelformat);
    utils_send_byte(DEBUG_COMPORT, len & 0xff);
    utils_send_bytes(DEBUG_COMPORT, pixelformat, len);

    // Transmit the image length in bytes
    utils_send_byte(DEBUG_COMPORT, (imgLen >> 24) & 0xff); // high byte
    utils_send_byte(DEBUG_COMPORT, (imgLen >> 16) & 0xff); // low byte
    utils_send_byte(DEBUG_COMPORT, (imgLen >> 8)  & 0xff); // low byte
    utils_send_byte(DEBUG_COMPORT, (imgLen >> 0)  & 0xff); // low byte

    // Send the image pixel bytes
    while (imgLen) {
        len = imgLen;
        utils_send_bytes(DEBUG_COMPORT, img, len);
        img       += len;
        imgLen    -= len;
    }

    return 0;
}
