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
#include <mxc.h>

/***** Definitions *****/
#define ALL_PT 0x0C

// PT Selection
#ifdef FTHR_Apps_P1 // Defined in board.h
#define ContPulse_PT   0
#define SquareWave_PT  1
#define ContPulse_Pin  18
#define SquareWave_Pin 19

#else
#define ContPulse_PT   2
#define SquareWave_PT  3
#define ContPulse_Pin  16
#define SquareWave_Pin 17

#endif

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
void PT_IRQHandler(void)
{
    printf("flags = 0x%08x\n", MXC_PT_GetFlags());

    MXC_PT_ClearFlags(ALL_PT);
}

// *****************************************************************************
void ContinuousPulseTrain(void)
{
    //Setup GPIO to PT output function

    //setup PT configuration
    mxc_pt_cfg_t ptConfig;

    ptConfig.channel   = ContPulse_PT; // PT channel
    ptConfig.bps       = 2;            //bit rate
    ptConfig.ptLength  = 5;            //bits
    ptConfig.pattern   = 0x16;
    ptConfig.loop      = 0; //continuous loop
    ptConfig.loopDelay = 0;

    MXC_PT_Config(&ptConfig);

    //start PT
    MXC_PT_Start(1 << ContPulse_PT);
}

// *****************************************************************************
void SquareWave(void)
{
    //Setup GPIO to PT output function

    uint32_t freq = 10;                        //Hz
    MXC_PT_SqrWaveConfig(SquareWave_PT, freq); //PT Channel

    //start PT
    MXC_PT_Start(1 << SquareWave_PT);
}

// *****************************************************************************
int main(void)
{
    printf("\n*************** Pulse Train Demo ****************\n");
    printf("PT%d (P0.%d) = Outputs continuous pattern of 10110b at 2bps\n", ContPulse_PT,
           ContPulse_Pin);
    printf("PT%d (P0.%d) = Outputs 10Hz continuous square wave\n", SquareWave_PT, SquareWave_Pin);

    NVIC_EnableIRQ(PT_IRQn);      //enabled default interrupt handler
    MXC_PT_EnableInt(ALL_PT);     //enabled interrupts for all PT
    MXC_PT_Init(MXC_PT_CLK_DIV1); //initialize pulse trains

    //configure and start pulse trains
    ContinuousPulseTrain();
    SquareWave();

    while (1) {}
}
