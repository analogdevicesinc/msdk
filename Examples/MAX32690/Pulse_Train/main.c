/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
 * @details LED0 - PT2 setup as 2Hz continuous signal that outputs 10110b
 *          LED1 - PT3 setup as 10Hz continuous square wave
 *
 * @note Interrupts for pulse trains are enabled but the interrupt handler only clears the flags.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "board.h"
#include "pt.h"

/***** Definitions *****/
#define ALL_PT 0x0C

// PT Selection
#define ContPulse_PT 2
#define SquareWave_PT 3
#define ContPulse_Pin 8
#define SquareWave_Pin 9

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
void PT_IRQHandler(void)
{
    printf("flags = 0x%08x\n", MXC_PT_GetStopFlags());

    MXC_PT_ClearStopFlags(ALL_PT);
}

// *****************************************************************************
void ContinuousPulseTrain(void)
{
    //Setup GPIO to PT output function

    //setup PT configuration
    mxc_pt_cfg_t ptConfig;

    ptConfig.channel = ContPulse_PT; // PT channel
    ptConfig.bps = 2; //bit rate
    ptConfig.ptLength = 5; //bits
    ptConfig.pattern = 0x16;
    ptConfig.loop = 0; //continuous loop
    ptConfig.loopDelay = 0;

    MXC_PT_Config(&ptConfig);

    //start PT
    MXC_PT_Start(1 << ContPulse_PT);
}

// *****************************************************************************
void SquareWave(void)
{
    //Setup GPIO to PT output function

    uint32_t freq = 10; //Hz
    MXC_PT_SqrWaveConfig(SquareWave_PT, freq); //PT Channel

    //start PT
    MXC_PT_Start(1 << SquareWave_PT);
}

// *****************************************************************************
int main(void)
{
    printf("\n*************** Pulse Train Demo ****************\n");
    printf("PT%d (P1.%d) = Outputs continuous pattern of 10110b at 2bps\n", ContPulse_PT,
           ContPulse_Pin);
    printf("PT%d (P1.%d) = Outputs 10Hz continuous square wave\n", SquareWave_PT, SquareWave_Pin);

    NVIC_EnableIRQ(PT_IRQn); //enabled default interrupt handler
    MXC_PT_EnableStopInt(ALL_PT); //enabled interrupts for all PT
    MXC_PT_Init(MXC_PT_CLK_DIV1); //initialize pulse trains

    //configure and start pulse trains
    ContinuousPulseTrain();
    SquareWave();

    while (1) {}
}
