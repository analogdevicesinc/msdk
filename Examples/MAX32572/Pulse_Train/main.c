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

/**
 * @file main.c
 * @brief Configures and starts four different pulse trains on GPIO LEDs.
 * @details LED0 - PT6 setup as 2Hz continuous signal that outputs 10110b
 *          LED1 - PT7 setup as 10Hz continuous square wave
 *          SW1 - push button setup to stop/start pulse trains
 *
 * @note Interrupts for pulse trains are enabled but the interrupt handler only clears the flags.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

/***** Definitions *****/
#define ALL_PT 0x7F
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
    //GPIO P0.13 uses PT4

    //setup PT configuration
    mxc_pt_cfg_t ptConfig;
    ptConfig.channel   = 4; //PT4
    ptConfig.bps       = 2; //bit rate
    ptConfig.ptLength  = 5; //bits
    ptConfig.pattern   = 0x16;
    ptConfig.loop      = 0; //continuous loop
    ptConfig.loopDelay = 0;

    MXC_PT_Config(&ptConfig);

    //start PT4
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT4);
}

// *****************************************************************************
void SquareWave(void)
{
    //Setup GPIO to PT output function
    //GPIO P0.12 uses PT3

    uint32_t freq = 10;            //Hz
    MXC_PT_SqrWaveConfig(3, freq); //PT3

    //start PT3
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT3);
}

// *****************************************************************************
int main(void)
{
    printf("\n*************** Pulse Train Demo ****************\n");
    printf("LED0 = Outputs continuous pattern of 10110b at 2bps\n");
    printf("LED1 = Outputs 10Hz continuous square wave\n");

    printf("\nConnect external LEDS to see the demo.\n");
    printf("LED0 to P0.13 and LED1 to P0.12");

    NVIC_EnableIRQ(PT_IRQn);      //enabled default interrupt handler
    MXC_PT_EnableInt(ALL_PT);     //enabled interrupts for all PT
    MXC_PT_Init(MXC_PT_CLK_DIV1); //initialize pulse trains

    //configure and start pulse trains
    ContinuousPulseTrain();
    SquareWave();

    while (1) {
    }
}
