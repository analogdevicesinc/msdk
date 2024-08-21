/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

/**
 * @file    main.c
 * @brief   Security Monitor example
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
    flags = MXC_SMON_GetFlags();
    flags = flags & MXC_F_SMON_SECALM_EXTSTAT0;
    mxc_gpio_cfg_t ext = { MXC_GPIO1, (MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9), MXC_GPIO_FUNC_ALT1,
                           MXC_GPIO_PAD_PULL_DOWN, MXC_GPIO_VSSEL_VDDIO };
    MXC_GPIO_Config(&ext);

    MXC_SMON_Init();
    MXC_SMON_ClearFlags(MXC_F_SMON_SECALM_EXTSTAT0 | MXC_F_SMON_SECALM_EXTSWARN0 |
                        MXC_F_SMON_SECALM_EXTF);

    flags = MXC_SMON_GetFlags();
    flags = flags & MXC_F_SMON_SECALM_EXTSTAT0;
    printf("\nArming external sensor\n");

    mxc_smon_ext_cfg_t cfg;
    cfg.sensorNumber = SMON_EXTSENSOR_0;
    cfg.clockDivide = SMON_CLK_DIVIDE_1;
    cfg.freqDivide = SMON_FREQ_DIVIDE_4;
    cfg.errorCount = 2;
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
