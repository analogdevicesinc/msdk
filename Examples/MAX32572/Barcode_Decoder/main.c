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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

#include "camera.h"
#include "app_loop.h"

// *****************************************************************************
int main(void)
{
    int ret = 0;
    
    printf("\n************** Barcode Decoder on MAX32572 !*************\n");
#ifdef BRCD_RDR_IMG_TO_PC
    // To send image to PC
    MXC_UART_Init(MXC_UART1, 460800);
#endif
    // enable catch
    MXC_ICC_Enable();
    // initialize timer
    ret = MXC_RTC_Init(0, 0);
    
    if (ret == 0) {
        ret = MXC_RTC_Start();
    }
    
    if (ret == 0) {
        // initialize loop
        app_loop_init();
        app_loop_endless();
    }
    
    return ret;
}
