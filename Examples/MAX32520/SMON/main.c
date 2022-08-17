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

/**
 * @file    main.c
 * @brief   Security MOnitor
 *
 * @details
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "smon.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    uint32_t flags = 1;
    printf("\n***********Security Monitor***********\n");
    printf("\nPull the ext sensor jumper to set off the alarm that the sensor has been tampered "
           "with. \n");
    flags              = MXC_SMON_GetFlags();
    flags              = flags & MXC_F_SMON_SECALM_EXTSTAT0;
    mxc_gpio_cfg_t ext = {MXC_GPIO1, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9), MXC_GPIO_FUNC_ALT1,
                          MXC_GPIO_PAD_PULL_DOWN, MXC_GPIO_VSSEL_VDDIO};
    MXC_GPIO_Config(&ext);

    MXC_SMON_Init();
    MXC_SMON_ClearFlags(MXC_F_SMON_SECALM_EXTSTAT0 | MXC_F_SMON_SECALM_EXTSWARN0 |
                        MXC_F_SMON_SECALM_EXTF);

    flags = MXC_SMON_GetFlags();
    flags = flags & MXC_F_SMON_SECALM_EXTSTAT0;
    printf("\nArming external sensor\n");

    mxc_smon_ext_cfg_t cfg;
    cfg.sensorNumber = SMON_EXTSENSOR_0;
    cfg.clockDivide  = SMON_CLK_DIVIDE_1;
    cfg.freqDivide   = SMON_FREQ_DIVIDE_4;
    cfg.errorCount   = 2;
    MXC_SMON_ExtSensorEnable(&cfg, 5000);

    while (!(flags)) {
        flags = MXC_SMON_GetFlags();
        flags = flags & MXC_F_SMON_SECALM_EXTSTAT0;
    }

    printf("External Sensor has Alarmed\n\n");
    MXC_SMON_ClearFlags(MXC_F_SMON_SECALM_EXTSTAT0);
    flags = MXC_SMON_GetFlags();
    flags = flags & MXC_F_SMON_SECALM_EXTSTAT0;

    printf("Example Successful\n");

    return 0;
}
