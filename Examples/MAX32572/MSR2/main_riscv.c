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

/**
 * @file    main_riscv.c
 * @brief   Magnetic Stripe Reader RISC-V
 *
 * @details This example demonstrates MSR reading 2-track (bank) cards
 */

/***** Includes *****/
#include "stdio.h"
#include "max32572.h"
#include "fcr_regs.h"
#include "gcr_regs.h"
#include "sema_regs.h"
#include "riscv_rom.h"

/***** Definitions *****/

/* Sending IRQ signal to ARM core if enabled */
#define SEND_IRQ_TO_CM4 {if (msr_cfg & MSR_CFG_CM4IRQ_ENABLE) \
            MXC_SEMA->irq1 |= MXC_F_SEMA_IRQ1_RV32_IRQ + MXC_F_SEMA_IRQ1_EN;}

/* RISC-V related memory wait state control bits */
#define MEMCTRL_RISCV_WS (MXC_F_GCR_MEMCTRL_SRAM5_WS \
        | MXC_F_GCR_MEMCTRL_SRAM6_WS | MXC_F_GCR_MEMCTRL_ROM1_WS)

/***** Globals *****/

uint32_t tmp, tmp1, tmp2, tmp3;

/* Define data to replace default parameters with */

/* ADC frame sequence */
uint8_t adc_seq[8] = {1, 1, 2, 1, 1, 2, 1, 2};
uint8_t tmp_adc_seq[8];

/* ADC input common mode setup */
hw_register_setup_t adc_cm = {
    (uint32_t)&MXC_FCR->msradc9,
    MXC_S_FCR_MSRADC9_R1_0K | MXC_S_FCR_MSRADC9_R2_6K,
    0,
    5 };
hw_register_setup_t tmp_adc_cm;

/* Track 1 filtering setup */
track_flt_setup_t  flt_in_1 = {
    {2,4,8,16},     /* osr */
    {256/4, 256/16, 256/64, 256/256}, /* scale_up */
    {90, 180, 360,    210},    /* speed thr [3], BPI */
    {128, 128, 128, 128},      /* thr_fct (Q8.8) */
    1,  /* scale dn rshift */
    2   /* init idx [0-3]*/
};

/* Track 2 filtering setup */
track_flt_setup_t  flt_in_2 = {
    {4,7,14,28},     /* osr */
    {784/16, 784/49, 784/196, 784/784}, /* scale_up */
    {90, 180, 360,    75},    /* speed thr [3], BPI */
    {128, 128, 128, 128},      /* thr_fct (Q8.8) */
    3,  /* scale dn rshift */
    2   /* init idx [0-3]*/
};

track_flt_setup_t  flt_out;



/***** Functions *****/


/*  Will use interrupt vector table in SRAM 
    However need MSRADC and CM4 interrupts to be serviced by ISR functions in ROM
    Redirect MSRADC and CM4 interrupts to ROM here
*/
__attribute__((naked))  /* no prologue code inserted */
void MSRADC_IRQHandler(void)
{
    goto *pADC9_IRQHandler;
}
__attribute__((naked))  /* no prologue code inserted */
void CM4_IRQHandler(void)
{
  goto *pCM4_IRQHandler;
}


// *****************************************************************************
int main(void)
{

    /* signal to host that MSR is not ready (0 = busy) */
    msr_ctrl = 0;

    /* set default MSR parameters */
    pset_default_parameters();


    /*  App may change default MSR parameters here
        Tune MSR parameters to get better performance on 2-track card
        - speed up ADC sampling with more aggressive (faster) ADC clock
        - only sample track 1 and track 2
        - set more aggressive filtering parameters (larger OSR setting)
        - tweak detection thresholds

        NOTE: printing MSR parameters is commented out below (#if 0) since each 
        printf() adds extra code and will overflow limited RISCV code space (16KB)
        at some point
        You may uncomment few at a time (#if 1) as long as code fits, but not all
    */

    printf("\n");

    /* PCLK to ACLK divider ratio [2..255] */
    /* assume fast wait state setting */
    pset_pclk2aclk_div(15);
#if 1
    tmp = pget_pclk2aclk_div();
    printf("RISCV: pclk2aclk_div = %d\n", tmp);
#endif

    /* ADC frame length [1..8] */
    pset_adc_frame_len(8);
#if 1
    tmp = pget_adc_frame_len();
    printf("RISCV: adc_frame_len = %d\n", tmp);
#endif

    /* ADC frame sequence (sequence of tracks 1, 2, or 3 to sample) */
    pset_adc_frame_seq(adc_seq);
#if 1
    pget_adc_frame_seq(tmp_adc_seq);
    printf("RISCV: adc_frame_seq = %d %d %d %d %d %d %d %d\n"
          , tmp_adc_seq[0], tmp_adc_seq[1], tmp_adc_seq[2], tmp_adc_seq[3]
          , tmp_adc_seq[4], tmp_adc_seq[5], tmp_adc_seq[6], tmp_adc_seq[7] );
#endif

    // /* ADC common mode trim */
    pset_adc_cm_setup(&adc_cm);
#if 1
    pget_adc_cm_setup(&tmp_adc_cm);
    printf("RISCV: adc cm trim : register = 0x%x pos = %d len = %d value = 0x%x\n"
          , tmp_adc_cm.address, tmp_adc_cm.pos, tmp_adc_cm.len, tmp_adc_cm.value);
#endif

    /* Noise acquisition time (ms) and termination timeout (ms), per track */
    pset_trk_timo(1, 100, 200);
    pset_trk_timo(2, 100, 200);
// #if 0
//     pget_trk_timo(1, &tmp1, &tmp2);
//     printf("RISCV: tr1 noise acq = %d ms, termination timo = %d ms\n", tmp1, tmp2);
//     pget_trk_timo(2, &tmp1, &tmp2);
//     printf("RISCV: tr2 noise acq = %d ms, termination timo = %d ms\n", tmp1, tmp2);
// #endif

    /* Peak detection threshold factors (Q8.8), per track */
    pset_trk_trh_setup(1, 0x160, 0x20);
    pset_trk_trh_setup(2, 0x160, 0x20);
// #if 0
//     pget_trk_trh_setup(1, &tmp1, &tmp2);
//     printf("RISCV: tr1 thr_fct = 0x%x 0x%x\n", tmp1, tmp2);
//     pget_trk_trh_setup(2, &tmp1, &tmp2);
//     printf("RISCV: tr2 thr_fct = 0x%x 0x%x\n", tmp1, tmp2);
// #endif

    /* Voltage waveform filtering parameters, per track */
    pset_trk_flt_setup(1, &flt_in_1);
    pset_trk_flt_setup(2, &flt_in_2);
#if 0
    pget_trk_flt_setup(1, &flt_out);
    printf("RISCV: tr1 filtering:\n");
    printf("  osr = %d %d %d %d\n"
              , flt_in_1.flens[0], flt_in_1.flens[1]
              , flt_in_1.flens[2], flt_in_1.flens[3] );
    printf("  scale_up = %d %d %d %d\n"
              , flt_in_1.fscale_up[0], flt_in_1.fscale_up[1]
              , flt_in_1.fscale_up[2], flt_in_1.fscale_up[3] );
    printf("  speed_thr = %d %d %d %d\n"
              , flt_in_1.spd_thr[0], flt_in_1.spd_thr[1]
              , flt_in_1.spd_thr[2], flt_in_1.spd_thr[3] );
    printf("  thr_fct = %d %d %d %d\n"
              , flt_in_1.thr_fct[0], flt_in_1.thr_fct[1]
              , flt_in_1.thr_fct[2], flt_in_1.thr_fct[3] );
    printf("  scale_dn = %d\n", flt_in_1.fscale_dn);
    printf("  init_idx = %d\n", flt_in_1.ilen);
#endif
#if 0
    pget_trk_flt_setup(2, &flt_out);
    printf("RISCV: tr2 filtering:\n");
    printf("  osr = %d %d %d %d\n"
              , flt_in_2.flens[0], flt_in_2.flens[1]
              , flt_in_2.flens[2], flt_in_2.flens[3] );
    printf("  scale_up = %d %d %d %d\n"
              , flt_in_2.fscale_up[0], flt_in_2.fscale_up[1]
              , flt_in_2.fscale_up[2], flt_in_2.fscale_up[3] );
    printf("  speed_thr = %d %d %d %d\n"
              , flt_in_2.spd_thr[0], flt_in_2.spd_thr[1]
              , flt_in_2.spd_thr[2], flt_in_2.spd_thr[3] );
    printf("  thr_fct = %d %d %d %d\n"
              , flt_in_2.thr_fct[0], flt_in_2.thr_fct[1]
              , flt_in_2.thr_fct[2], flt_in_2.thr_fct[3] );
    printf("  scale_dn = %d\n", flt_in_2.fscale_dn);
    printf("  init_idx = %d\n", flt_in_2.ilen);
    pget_trk_flt_setup(3, &flt_out);
#endif

    printf("\n");
    /*  End of change default MSR parameters  */


    /* initialize magnetic stripe read */
    pmsr_init();

    /* signal to host that MSR is ready (non-0) */
    msr_ctrl = 0xff;
    /* IRQ to CM4 if enabled */
    SEND_IRQ_TO_CM4

	/* swipe loop */
	while (1) {
        pGetSwipe();
        pProcessSwipe();
        /* IRQ to CM4 if enabled */
        SEND_IRQ_TO_CM4
	}

}
