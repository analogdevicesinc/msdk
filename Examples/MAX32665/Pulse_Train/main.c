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
 * @details LED0 (P1.14) - PT14 setup as 2Hz continuous signal that outputs 10110b
 *          LED1 (P1.15) - PT15 setup as 10Hz continuous square wave
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
#define PTG MXC_PTG_BUS0

/***** Functions *****/

// *****************************************************************************
static void PB_Start_Stop_handler()
{
    uint32_t enablePTMask;

    //Check if any pulse trains are running
    if (MXC_PT_IsActive(PTG, ALL_PT)) {
        //stop all pulse trains
        MXC_PT_Stop(PTG, ALL_PT);
    } else {
        //start PT14 and PT15
        enablePTMask = MXC_F_PTG_ENABLE_PT14 | MXC_F_PTG_ENABLE_PT15;
        MXC_PT_Start(PTG, enablePTMask);
        MXC_PT_Start(PTG, MXC_F_PTG_ENABLE_PT14);
        MXC_PT_Start(PTG, MXC_F_PTG_ENABLE_PT15);
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
    //GPIO P1.14 uses PT14

    //setup PT configuration
    mxc_pt_cfg_t ptConfig;
    ptConfig.channel = 14; //PT14
    ptConfig.bps = 2; //bit rate
    ptConfig.ptLength = 5; //bits
    ptConfig.pattern = 0x16;
    ptConfig.loop = 0; //continuous loop
    ptConfig.loopDelay = 0;

    MXC_PT_Config(PTG, &ptConfig);

    //start PT14
    MXC_PT_Start(PTG, MXC_F_PTG_ENABLE_PT14);
}

// *****************************************************************************
void SquareWave(void)
{
    //Setup GPIO to PT output function
    //GPIO P1.15 uses PT15

    uint32_t freq = 10; //Hz
    MXC_PT_SqrWaveConfig(PTG, 15, freq); //PT15

    //start PT15
    MXC_PT_Start(PTG, MXC_F_PTG_ENABLE_PT15);
}

// *****************************************************************************
int main(void)
{
    printf("\n*************** Pulse Train Demo ****************\n");

#if defined(BOARD_FTHR2)
    printf("P1.14 = Outputs continuous pattern of 10110b at 2bps\n");
    printf("P1.15 = Outputs 10Hz continuous square wave\n");
    printf("Push button 0 = Stop/Start all pulse trains\n");
#else
    printf("LED0 = Outputs continuous pattern of 10110b at 2bps\n");
    printf("LED1 = Outputs 10Hz continuous square wave\n");
    printf("Push button 0 = Stop/Start all pulse trains\n");
#endif
    //Setup push button to start/stop All pulse train
    PB_RegisterCallback(0, (pb_callback)PB_Start_Stop_handler);

    NVIC_EnableIRQ(PT_IRQn); //enabled default interrupt handler
    MXC_PT_EnableInt(PTG, ALL_PT); //enabled interrupts for all PT
    MXC_PT_Init(PTG, MXC_PT_CLK_DIV1); //initialize pulse trains

    //configure and start pulse trains
    ContinuousPulseTrain();
    SquareWave();

    while (1) {}
}
