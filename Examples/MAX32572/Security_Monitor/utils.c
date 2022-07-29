/*
 * utils.c
 *
 ******************************************************************************
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
 ******************************************************************************
 */

#include <stdio.h>
#include <string.h>
#include <MAX32xxx.h>
#include "utils.h"

/****************************    DEFINES        ******************************/
#define SECONDSONEDAY (24 * 3600)
#define STARTYEAR     1970

/****************************    Variables      ******************************/
static unsigned char daysOfMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

/**************************** Static Functions *******************************/

/**************************** Public Functions *******************************/
void utils_hex_dump(const char* title, unsigned char* buf, unsigned int len)
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

void utils_seconds_to_date(DateTime_t* dt, unsigned int sec)
{
    int startYear            = STARTYEAR;
    unsigned int oneYearSecs = 365 * SECONDSONEDAY, oneMonthSecs;
    unsigned char leapYear = 0, i, mon = 0, day, hour, min;

    while (1) {
        if ((startYear % 4 == 0 && startYear % 100 != 0) || (startYear % 400 == 0)) {
            leapYear = 1;
        } else {
            leapYear = 0;
        }

        if (sec >= (oneYearSecs + (leapYear * SECONDSONEDAY))) {
            startYear++;
            sec -= (oneYearSecs + (leapYear * SECONDSONEDAY));
        } else {
            //month
            for (i = 0; i < 12; i++) {
                oneMonthSecs = (unsigned int)daysOfMonth[i] * SECONDSONEDAY;

                if (leapYear && i == 1) { // february leap year error
                    oneMonthSecs += SECONDSONEDAY;
                }

                if (sec < oneMonthSecs) {
                    mon = i;
                    break;
                } else {
                    sec -= oneMonthSecs;
                }
            }

            //day
            day = sec / SECONDSONEDAY;
            sec = sec % SECONDSONEDAY;

            //hour
            hour = sec / 3600;
            sec  = sec % 3600;

            //min
            min = sec / 60;
            sec = sec % 60;

            dt->year = startYear;
            dt->mon  = mon;
            dt->day  = day;
            dt->hour = hour;
            dt->min  = min;
            dt->sec  = sec;
            return;
        }
    }
}

void utils_delay_ms(unsigned int ms)
{
    MXC_Delay(ms * 1000UL);
}
