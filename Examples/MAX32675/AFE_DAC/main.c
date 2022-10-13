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
 * @brief   AFE_DAC Example
 * @details This example configures the AFE DAC 12 to output 1V
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_device.h"
#include "board.h"
#include "mxc_delay.h"
#include "afe.h"
#include "afe_dac_regs.h"

/***** Definitions *****/

/***** Globals *****/

/***** Functions *****/

static void dump_dac_regs(void)
{
    uint32_t read_val = 0;

    afe_read_register(MXC_R_AFE_DAC_CTRL, &read_val);
    printf("AFE DAC CTRL: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_RATE, &read_val);
    printf("AFE DAC RATE: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_INT, &read_val);
    printf("AFE DAC INT: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_REG, &read_val);
    printf("AFE DAC REG: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_TRIM, &read_val);
    printf("AFE DAC TRIM: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_CTRL, &read_val);
    printf("AFE DAC VREF CTRL: \t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_FIFO, &read_val);
    printf("AFE DAC FIFO: \t\t0x%08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_TRIM, &read_val);
    printf("AFE DAC VREF TRIM: \t0x%08X\n", read_val);
}

// *****************************************************************************
int main(void)
{
    uint32_t read_val = 0;

    printf("\n\n\n\n\nMAX32675 AFE DAC Example\n\n");
    printf("This example configures the AFE's DAC to output a static 1.0V\n");

    //
    // Reset the DAC
    //
    printf("Reseting DAC\n");
    afe_write_register(MXC_R_AFE_DAC_CTRL, MXC_F_AFE_DAC_CTRL_RESET);

    // Wait till reset is done
    while (1) {
        afe_read_register(MXC_R_AFE_DAC_CTRL, &read_val);

        if (!(read_val & MXC_F_AFE_DAC_CTRL_RESET)) {
            // Reset is cleared.  Done
            break;
        }
    }

    //
    // Configure to use Internal DAC Reference
    //
    afe_read_register(MXC_R_AFE_DAC_VREF_CTRL, &read_val);
    read_val |= MXC_F_AFE_DAC_VREF_CTRL_REF_PU | MXC_F_AFE_DAC_VREF_CTRL_REFDAC_OUTEN;
    // Clear out ref selections bits.
    read_val &= ~MXC_F_AFE_DAC_VREF_CTRL_DACREFSEL;
    read_val |= MXC_S_AFE_DAC_VREF_CTRL_DACREFSEL_VOLTS_2_048;

    afe_write_register(MXC_R_AFE_DAC_VREF_CTRL, read_val);

    //
    // Configure the DAC
    // Set to output values when written to FIFO. DAC will hold last value
    // Assuming default power level of 48uA is adequate
    //
    afe_read_register(MXC_R_AFE_DAC_CTRL, &read_val);

    read_val |= MXC_F_AFE_DAC_CTRL_POWER_ON;
    read_val |= MXC_F_AFE_DAC_CTRL_CLOCK_GATE_EN;

    read_val &= ~MXC_F_AFE_DAC_CTRL_OP_MODE;
    read_val |= MXC_S_AFE_DAC_CTRL_OP_MODE_OUTPUT_WHEN_FIFO_AVAIL;

    // Enable DAC output P and N channel FETs
    // NOTE: These bits [18 and 19] are currently undocumented, and always read as 0
    read_val |= 0x000C0000;

    read_val &= ~MXC_F_AFE_DAC_CTRL_START_MODE;
    read_val |= MXC_S_AFE_DAC_CTRL_START_MODE_START_WHEN_FIFO_NOT_EMPTY;

    afe_write_register(MXC_R_AFE_DAC_CTRL, read_val);

    //
    // Set output voltage
    // Ref is 2.048V and DAC is 12 bits.
    // 2^12 is 4096 LSBs. Each LSB is 2.048/4096 = 500uV
    // To get 1V 1/.0005 = 2000
    //
    // Write the FIFO one time to set to static voltage.
    //
    // NOTE: values written to the FIFO must be LEFT justified, so shift over 4 bits
    //

    afe_write_register(MXC_R_AFE_DAC_FIFO, (2000 << 4));

    printf("DAC Configured for 1.0V output. Current AFE DAC registers:\n");
    dump_dac_regs();

    printf("Done. Halting...\n");

    return 0;
}
