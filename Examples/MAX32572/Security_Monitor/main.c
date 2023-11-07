/*
 * @file main.c
/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc., All Rights Reserved.
 * (now owned by Analog Devices, Inc.)
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
 ******************************************************************************
 *
 * Copyright 2023 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

#include <stdio.h>
#include <MAX32xxx.h>

#include "keypad.h"
#include "security_monitor.h"

typedef struct {
    const char *name;
    int (*callback)(void);
    const char *detail;
} list_t;

static list_t list[] = {
    { "Check AES Key", smon_check_aes_key, "Can be used to check Battery Backed key status" },
    { "Check RTC Status", smon_rtc_status,
      "Is used to check whether RTC is running or not. When battery being removed RTC will stop." },
    { "Start RTC", smon_start_rtc, "To start RTC" },
    { "Set Ext Sensors", smon_set_ext_sensors, "To activated external sensors" },
    { "Set Int Sensors", smon_set_int_sensors, "To activated internal sensors" },
    { "Load AES Crypto Key", smon_load_aes_key, "To create and load Battery Backed key" },
    { "Check Tamper", smon_check_tamper, "To check tamper status" },
    { "Get Tamper Time", smon_check_tamper_time, "To read tamper time if it is exist" },
    { "Clear Tamper", smon_clear_tamper, "To clear tamper" },
    { "Clear Tamper Time", smon_clear_tamper_time, "To clear tamper time" },
    { "Set KeyWipe", smon_secalm_keywipe, "To manually remove secret keys" },
    { "Create DRS", smon_create_DRS, "To manually create DRS" }
};

static int system_init(void)
{
    int ret = 0;

    ret = smon_init();

    if (ret == 0) {
        ret = kb_init();
    }

    return ret;
}

int main(void)
{
    int ret = 0;
    int i;
    const char *items[32];
    int nb_of_items = sizeof(list) / sizeof(list[0]);

    ret = system_init();

    if (ret != 0) {
        printf("\n\nError during initialization!!!\n");
        return ret;
    }

    printf("\n\n Step by Step Security Monitor Example \n");
    printf("-------------------------------------------------\n");

    printf("With this example you can run security monitor feature step by step"
           "and the test result can be seen easily.\n");

    for (i = 0; i < nb_of_items; i++) {
        printf("\t%02d- %-20s:%s\n", (i + 1), list[i].name, list[i].detail);
    }

    printf("-------------------------------------------------\n");
    printf("To activate system on t=0 time (after battery remove)\n"
           "    - Start RTC\n"
           "    - Activate Sensors\n"
           "    - Load AES Key\n");
    printf("\nInput device can be EvKit keypad or PC Keyboard (Default). To change it please check "
           "keypad.c file\n");

    for (i = 0; i < nb_of_items; i++) {
        items[i] = list[i].name;
    }

    // test loop
    while (1) {
        i = kb_select_from_list_xcol("Security Monitor", items, nb_of_items, 2);

        if ((i > 0) && (i <= nb_of_items)) {
            --i; // index start from 0
            list[i].callback();
        } else {
            printf("ERR:Invalid function number\n");
            printf("-------------------------------------\n");
        }
    }

    return ret;
}
