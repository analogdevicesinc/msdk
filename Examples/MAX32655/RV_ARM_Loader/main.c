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
 * @brief   A basic getting started program for the RISCV, run from the ARM core.
 * @details RV_ARM_Loader runs on the ARM core to load the RISCV code space, setup the RISCV debugger pins,
            and start the RISCV core.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_sys.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
int main(void)
{
    printf("ARM: RV_ARM_Loader\n");

    MXC_Delay(MXC_DELAY_SEC(2));

    /* Enable RISCV debugger GPIO */
    MXC_GPIO_Config(&gpio_cfg_rv_jtag);

    /* Start the RISCV core */
    MXC_SYS_RISCVRun();

    /* Enter LPM */
    while (1) {}
}
