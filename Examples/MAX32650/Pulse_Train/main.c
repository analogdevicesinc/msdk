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
 * @file main.c
 * @brief Configures and starts four different pulse trains on GPIO LEDs.
 * @details LED0 - PT11 setup as 2Hz continuous signal that outputs 10110b
 *          LED1 - PT12 setup as 10Hz continuous square wave
 *          SW1 - push button setup to stop/start pulse trains
 *
 * @note Interrupts for pulse trains are enabled but the interrupt handler only clears the flags.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "board.h"
#include "pb.h"
#include "pt.h"

/***** Definitions *****/
#define ALL_PT 0xFFFF
/***** Globals *****/
#define PTG MXC_PTG

/***** Functions *****/

// *****************************************************************************
static void PB_Start_Stop_handler()
{
    uint32_t enablePTMask;
    for (int i = 0; i < 20000; i++) {}

    //Check if any pulse trains are running
    if (MXC_PT_IsActive(PTG, ALL_PT)) {
        //stop all pulse trains
        MXC_PT_Stop(PTG, ALL_PT);
    } else {
        //start PT14 and PT15
        enablePTMask = MXC_F_PTG_ENABLE_PT14 | MXC_F_PTG_ENABLE_PT15;
        MXC_PT_Start(PTG, enablePTMask);
    }
}

// *****************************************************************************
void PT_IRQHandler(void)
{
    printf("flags = 0x%08x\n", MXC_PT_GetFlags(PTG));

    MXC_PT_ClearFlags(PTG, ALL_PT);
}

// *****************************************************************************
void ContinuousPulseTrain(void)
{
    //Setup GPIO to PT output function
    //GPIO P2.28 uses PT14

    //setup PT configuration
    mxc_pt_cfg_t ptConfig;
    ptConfig.channel = 14; //PT14
    ptConfig.bps = 2; //bit rate
    ptConfig.ptLength = 5; //bits
    ptConfig.pattern = 0x16;
    ptConfig.loop = 0; //continuous loop
    ptConfig.loopDelay = 0;
    ptConfig.outputSelect = 0;

    MXC_PT_Config(PTG, &ptConfig);
}

// *****************************************************************************
void SquareWave(void)
{
    //Setup GPIO to PT output function
    //GPIO P0.23 uses PT15

    uint32_t freq = 10; //Hz
    MXC_PT_SqrWaveConfig(PTG, 15, freq, 0); //PT15
}

// *****************************************************************************
int main(void)
{
    printf("\n*************** Pulse Train Demo ****************\n");
    printf("\nPlease make the following connections: P2.28->P2.25 and P0.23->P2.26\n");
    printf("LED0 = Outputs continuous pattern of 10110b at 2bps\n");
    printf("LED1 = Outputs 10Hz continuous square wave\n");
    printf("SW3  = Stop/Start all pulse trains\n");

    //Setup push button to start/stop All pulse train
    PB_RegisterCallback(1, (pb_callback)PB_Start_Stop_handler);

    NVIC_EnableIRQ(PT_IRQn); //enabled default interrupt handler
    MXC_PT_EnableInt(PTG, ALL_PT); //enabled interrupts for all PT
    MXC_PT_Init(PTG, MXC_PT_CLK_DIV1); //initialize pulse trains

    //configure and start pulse trains
    ContinuousPulseTrain();
    SquareWave();

    while (1) {}
}
