/*
********************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
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
*******************************************************************************
*/
#include <stdio.h>

#include "MAX32xxx.h"
#include "utils.h"

/***************************** VARIABLES *************************************/

/************************   STATIC FUNCTIONS  *******************************/

/************************   PUBLIC FUNCTIONS  *******************************/
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

void utils_delay_ms(unsigned int ms)
{
    MXC_Delay(ms * 1000UL);
}

void utils_hex2char(char chr, char* msg)
{
    int i;
    char c;

    c = chr >> 4;
    for (i = 0; i < 2; i++) {
        if (c < 10) {
            *msg = '0' + c;
        } else {
            *msg = 'A' + c - 10;
        }
        c = chr & 0x0F;
        msg++;
    }
}
