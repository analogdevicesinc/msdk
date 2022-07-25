/**
 * @file
 * @brief      CLCD Example
 * @details    Shows the Maxim Integrated Logo and then shows a gray screen.
 *             When running this example make sure none of the pins used have
 *             jumpers connected to them otherwise the color may be wrong or may
 *             not work properly.
 */

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
#include "mxc_errors.h"
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "mxc_pins.h"
#include "gpio.h"
#include "clcd.h"
#include "frame.h"
#include "mxc_sys.h"
#include "mxc_delay.h"

/* **** Definitions **** */
#define IRAM_BUFFER_SIZE 0x00012C00
#define PANEL_WIDTH      320
#define PANEL_HEIGHT     240

/* **** Globals **** */
uint8_t framebuffer[IRAM_BUFFER_SIZE + ((PANEL_WIDTH * PANEL_HEIGHT * LOGO_BPP) >> 3) + 31];

/* **** Functions **** */

/**
 * @brief      Function to configure the CLCD and display an image to the CLCD
 *             display.
 */
void display_logo(void)
{
    unsigned char* source;
    unsigned char* dest;
    int xoffs;
    int yoffs;
    unsigned int i;
    volatile uint32_t base_addr = 0;
    mxc_clcd_cfg_t panel; /**! Panel configuration structure */

    panel.width       = PANEL_WIDTH;  /**! Set the size of the panel in pixels */
    panel.height      = PANEL_HEIGHT; /**! Set the width of the panel in pixels */
    panel.frequency   = 6400000;      /**! minimum panel supported frequency */
    panel.vfrontporch = 2;            /**! the vertical front porch for the display */
    panel.vbackporch  = 2;            /**! the vertical back porch for the display */
    panel.vsyncwidth  = 10;
    panel.hfrontporch = 12; /**! Set the horizontal front porch width */
    panel.hbackporch  = 2;  /**! Set the horizontal back porch width */
    panel.hsyncwidth  = 70; /**! Set the horizontal sync width */
    panel.palette     = palette;
    panel.paletteSize = sizeof(palette) / sizeof(uint32_t); /** set the palette size in words */

    /** Enable the CLCD panel based on the number of bits per pixel */
    switch (LOGO_BPP) {
        case 1:
            panel.bpp = MXC_BPP1;
            break;
        case 8:
        default:
            panel.bpp = MXC_BPP8;
            break;
    }

    /** Init and config CLCD */
    MXC_CLCD_Init(&panel);

    /** Align */
    base_addr = ((unsigned int)framebuffer + 31) & ~0x1f;

    /** Clear to background color */
    memset((void*)base_addr, 0, (panel.width * panel.height * LOGO_BPP) >> 3);

    /** Center logo in frame buffer */
    xoffs = (panel.width - LOGO_W) >> 1;
    yoffs = (panel.height - LOGO_H) >> 1;

    source = image;
    dest   = (unsigned char*)(base_addr + (((panel.width * yoffs + xoffs) * LOGO_BPP) >> 3));
    /** Copy the image to the frame buffer */
    for (i = 0; i < LOGO_H; i++) {
        memcpy(dest, source, (LOGO_W * LOGO_BPP) >> 3);
        source += (LOGO_W * LOGO_BPP) >> 3;
        dest += (panel.width * LOGO_BPP) >> 3;
    }

    /** Set image to Maxim logo */
    MXC_CLCD_SetFrameAddr((void*)base_addr);

    /** Enable the CLCD to display the image */
    MXC_CLCD_Enable();

    /** Leave the display on 3 seconds */
    MXC_Delay(MXC_DELAY_SEC(3));

    /** Set image to all gray on the display */
    memset((void*)base_addr, 0, (panel.width * panel.height * LOGO_BPP) >> 3);
}

/**
 * @brief      System Tick Interrupt Handler to perform the required delay.
 */
void SysTick_Handler(void)
{
    MXC_DelayHandler();
}

/**
 * @brief      Main function that drives the CLCD display.
 *
 * @details    This example displays the Maxim logo on the CLCD display of the
 *             evaluation kit for 3 seconds, disables the CLCD display for
 *             3 seconds and then shuts down the CLCD display and then repeats
 *             the process.
 *
 *
 * @return     never returns.
 */
int main(void)
{
    printf("\n\n********** CLCD Example **********\n");
    while (1) {
        /**
         *  This function includes a 3 second delay for showing the image and
         *  then sets the CLCD image to solid gray.
         */
        display_logo();

        /** Disable the CLCD display. */
        MXC_CLCD_Disable();
        /** Delay is not required, this is for the demo flow */
        MXC_Delay(MXC_DELAY_SEC(3));
        /** Shut down the CLCD */
        MXC_CLCD_Shutdown();
        MXC_Delay(MXC_DELAY_SEC(1));
    }
}
