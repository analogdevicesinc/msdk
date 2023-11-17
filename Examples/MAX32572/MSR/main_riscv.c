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
 * @details This example demonstrates basic MSR functionality
 */

/***** Includes *****/
#include "stdio.h"
#include "max32572.h"
#include "fcr_regs.h"
#include "gcr_regs.h"
#include "sema_regs.h"
#include "riscv_rom.h"


/***** Definitions *****/

/*  Select interrupt vector table location
    Options: ROM table (1) or SRAM table (0)
    RISC-V code running from SRAM may optionally (re)use
    interrupt vector table existing in MSR ROM code
    If the case, no interrupt will be serviced except MSRADC and CM4,
    so UART and other interrupt-dependent peripherals won't work on RISCV
*/
#define USE_ROM_INTERRUPT_VECTOR_TABLE (0)

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
uint8_t adc_seq[8] = {2, 1, 3, 1, 3, 1, 3, 0};
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
    {2,4,8,12},     /* osr */
    {1728/4, 1728/16, 1728/64, 1728/144}, /* scale_up */
    {100, 200, 400,    210},    /* speed thr [3], BPI */
    {64, 80, 96, 112},      /* thr_fct (Q8.8) */
    4,  /* scale dn rshift */
    2   /* init idx [0-3]*/
};

/* Track 2 filtering setup */
track_flt_setup_t  flt_in_2 = {
    {2,4,8,12},     /* osr */
    {1728/4, 1728/16, 1728/64, 1728/144}, /* scale_up */
    {95, 190, 380,    75},    /* speed thr [3], BPI */
    {64, 80, 96, 112},      /* thr_fct (Q8.8) */
    4,  /* scale dn rshift */
    2   /* init idx [0-3]*/
};

/* Track 3 filtering setup */
track_flt_setup_t  flt_in_3 = {
    {2,4,8,12},     /* osr */
    {1728/4, 1728/16, 1728/64, 1728/144}, /* scale_up */
    {100, 200, 400,    210},    /* speed thr [3], BPI */
    {64, 80, 96, 112},      /* thr_fct (Q8.8) */
    4,  /* scale dn rshift */
    2   /* init idx [0-3]*/
};

track_flt_setup_t  flt_out;

/* Raw MSR data arrays placement and size */

/*  Select raw arrays placement
    Options: combined (1) or separate (0)
*/
#define  MSR_RAW_ARRAYS_COMBINED (1)

#if MSR_RAW_ARRAYS_COMBINED
    /* Size and Addresses of combined arrays */
    #define  MSR_RAW_ALL_ORG (0x20062800)
    #define  MSR_RAW_ALL_LEN (1200)
    #define  MSR_RAW_ALL_X   (MSR_RAW_ALL_ORG)
    #define  MSR_RAW_ALL_Y   (MSR_RAW_ALL_ORG + 4 * MSR_RAW_ALL_LEN * 3)
#else
    /*  Sizes and Addresses of individual arrays
        NOTE: memory region 0x20050000-0x20056000 "belongs" to ARM core and
        should not be used by RISCV MSR code to avoid memory corruption.
        It is used here for the sake of example since example ARM core code
        is known to not use this region.
        MSR RISCV code should safely use region 0x20062800-0x20068000 for raw data 
    */
    #define  MSR_RAW_TR1_ORG (0x20050000)
    #define  MSR_RAW_TR1_LEN (1200)
    #define  MSR_RAW_TR1_X   (MSR_RAW_TR1_ORG)
    #define  MSR_RAW_TR1_Y   (MSR_RAW_TR1_ORG + 4 * MSR_RAW_TR1_LEN)

    #define  MSR_RAW_TR2_ORG (0x20052000)
    #define  MSR_RAW_TR2_LEN (440)
    #define  MSR_RAW_TR2_X   (MSR_RAW_TR2_ORG)
    #define  MSR_RAW_TR2_Y   (MSR_RAW_TR2_ORG + 4 * MSR_RAW_TR2_LEN)

    #define  MSR_RAW_TR3_ORG (0x20054000)
    #define  MSR_RAW_TR3_LEN (1200)
    #define  MSR_RAW_TR3_X   (MSR_RAW_TR3_ORG)
    #define  MSR_RAW_TR3_Y   (MSR_RAW_TR3_ORG + 4 * MSR_RAW_TR3_LEN)
#endif


/* Track 1 encoding setup */
parse_bits_t trk_par_in_1 = {
    0x45,   /* start sentinel */
    0x1F,   /* end sentinel */
    7,      /* # of bits per char */
    210,    /* bit density used for swipe rate calculation, bit/in */
    
    (uint8_t *)
    /*  correct characters must have odd parity, place '~' at even parity index within array */
    /*  00 000000000000001111111111111111222222222222222233333333333333334444444444444444555555555555555566666666666666667777777777777 777 */
    /*  01 23456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABC DEF */
       "~!\"~$~~'(~~+~-.~0~~3~56~~9:~<~~?@~~C~EF~~IJ~L~~O~QR~T~~WX~~[~]^~ ~~#~%&~~)*~,~~/~12~4~~78~~;~=>~~AB~D~~GH~~K~MN~P~~S~UV~~YZ~\\~~_"
};

/* Track 2 encoding setup */
parse_bits_t trk_par_in_2 = {
    0x0B,   /* start sentinel */
    0x1F,   /* end sentinel */
    5,      /* # of bits per char */
    75,     /* bit density used for swipe rate calculation, bit/in */
    
    (uint8_t *)
    /*  correct characters must have odd parity, place '~' at even parity index within array */
    /*  00000000000000001111111111111111 */
    /*  0123456789ABCDEF0123456789ABCDEF */
       "~12~4~~78~~;~=>~0~~3~56~~9:~<~~?"
};

/* Track 3 encoding setup */
parse_bits_t trk_par_in_3 = {
    0x0B,   /* start sentinel */
    0x1F,   /* end sentinel */
    5,      /* # of bits per char */
    210,     /* bit density used for swipe rate calculation, bit/in */
    
    (uint8_t *)
    /*  correct characters must have odd parity, place '~' at even parity index within array */
    /*  00000000000000001111111111111111 */
    /*  0123456789ABCDEF0123456789ABCDEF */
       "~12~4~~78~~;~=>~0~~3~56~~9:~<~~?"
};

parse_bits_t trk_par_out;




/***** Functions *****/

#if !USE_ROM_INTERRUPT_VECTOR_TABLE

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

#endif

// *****************************************************************************
int main(void)
{

#if USE_ROM_INTERRUPT_VECTOR_TABLE
    /* This will set MTVEC register to point to interrupt vector table in ROM */
    pReset_Handler_Call(MSR_ISR_VECTOR);
#endif

    /* signal to host that MSR is not ready (0 = busy) */
    msr_ctrl = 0;

    /* set default MSR parameters */
    pset_default_parameters();



    /*  App may change default MSR parameters here
        Following code demonstartes use of every MSR API function,
        even though most parameters are not changed (set to the same default values)

        NOTE: printing MSR parameters is commented out below (#if 0) since each 
        printf() adds extra code and will overflow limited RISCV code space (16KB)
        at some point
        You may uncomment few at a time (#if 1) as long as code fits, but not all
    */

    printf("\n");

    /* Periph Clock (Hz) */
    pset_periph_clock_hz(50000000);
#if 1
    tmp = pget_periph_clock_hz();
    printf("RISCV: pclk = %d Hz\n", tmp);
#endif

    /* PCLK to ACLK divider ratio [2..255] */
    if (MXC_GCR -> memctrl & MEMCTRL_RISCV_WS) {
        /* slow wait state setting */
        pset_pclk2aclk_div(26);
    } else {
        /* fast wait state setting */
        pset_pclk2aclk_div(19);
    }
#if 0
    tmp = pget_pclk2aclk_div();
    printf("RISCV: pclk2aclk_div = %d\n", tmp);
#endif

    /* ACLK to SMPL divider ratio [3,5,4,8,16,32,64,128] */
    pset_aclk2smpl_div(8);
#if 0
    tmp = pget_aclk2smpl_div();
    printf("RISCV: aclk2smpl_div = %d\n", tmp);
#endif

    /* PCLK to TMR divider ratio [1,2,4,8,..., 4096] */
    pset_pclk2tmr_div(8);
#if 0
    tmp = pget_pclk2tmr_div();
    printf("RISCV: pclk2tmr_div = %d\n", tmp);
#endif

    /* ADC frame length [1..8] */
    pset_adc_frame_len(7);
#if 0
    tmp = pget_adc_frame_len();
    printf("RISCV: adc_frame_len = %d\n", tmp);
#endif

    /* ADC frame sequence (sequence of tracks 1, 2, or 3 to sample) */
    pset_adc_frame_seq(adc_seq);
#if 0
    pget_adc_frame_seq(tmp_adc_seq);
    printf("RISCV: adc_frame_seq = %d %d %d %d %d %d %d %d\n"
          , tmp_adc_seq[0], tmp_adc_seq[1], tmp_adc_seq[2], tmp_adc_seq[3]
          , tmp_adc_seq[4], tmp_adc_seq[5], tmp_adc_seq[6], tmp_adc_seq[7] );
#endif

    /* Swipe timeout (sec) */
    pset_getswipe_timout(45);
#if 1
    tmp = pget_getswipe_timout();
    printf("RISCV: swipe timeout = %d sec\n", tmp);
#endif

    /* Threshold scale factors UP,DN (Q8.8). Must satisfy UP*DN = 256*256 */
    pset_thr_scale(331, 198);
#if 0
    pget_thr_scale(&tmp1, &tmp2);
    printf("RISCV: thr_scale up/dn = %d %d\n", tmp1, tmp2);
#endif

    /* Noise limit threshold (ADC LSB) */
    pset_maxnoise(5);
#if 0
    tmp = pget_maxnoise();
    printf("RISCV: max_noise = %d lsb\n", tmp);
#endif

    /* Preamble detection filter factor NUM, DEN */
    pset_pream_fct(13, 2);
#if 0
    pget_pream_fct(&tmp1, &tmp2);
    printf("RISCV: pream_fct num,den = %d %d\n", tmp1, tmp2);
#endif

    /* ADC common mode trim */
    pset_adc_cm_setup(&adc_cm);
#if 0
    pget_adc_cm_setup(&tmp_adc_cm);
    printf("RISCV: adc cm trim : register = 0x%x pos = %d len = %d value = 0x%x\n"
          , tmp_adc_cm.address, tmp_adc_cm.pos, tmp_adc_cm.len, tmp_adc_cm.value);
#endif

    /* Noise acquisition time (ms) and termination timeout (ms), per track */
    pset_trk_timo(1, 100, 300);
    pset_trk_timo(2, 100, 300);
    pset_trk_timo(3, 100, 300);
#if 0
    pget_trk_timo(1, &tmp1, &tmp2);
    printf("RISCV: tr1 noise acq = %d ms, termination timo = %d ms\n", tmp1, tmp2);
    pget_trk_timo(2, &tmp1, &tmp2);
    printf("RISCV: tr2 noise acq = %d ms, termination timo = %d ms\n", tmp1, tmp2);
    pget_trk_timo(3, &tmp1, &tmp2);
    printf("RISCV: tr3 noise acq = %d ms, termination timo = %d ms\n", tmp1, tmp2);
#endif

    /* Leading and trailing zeros count threshold, per track */
    pset_trk_tcnt(1, 15, 13);
    pset_trk_tcnt(2, 5, 9);
    pset_trk_tcnt(3, 15, 9);
#if 0
    pget_trk_tcnt(1, &tmp1, &tmp2);
    printf("RISCV: tr1 leading zcnt = %d, trailing zcnt = %d\n", tmp1, tmp2);
    pget_trk_tcnt(2, &tmp1, &tmp2);
    printf("RISCV: tr2 leading zcnt = %d, trailing zcnt = %d\n", tmp1, tmp2);
    pget_trk_tcnt(3, &tmp1, &tmp2);
    printf("RISCV: tr3 leading zcnt = %d, trailing zcnt = %d\n", tmp1, tmp2);
#endif

    /* Peak detection threshold factors (Q8.8), per track */
    pset_trk_trh_setup(1, 0x180, 0x80);
    pset_trk_trh_setup(2, 0x180, 0x80);
    pset_trk_trh_setup(3, 0x180, 0x80);
#if 0
    pget_trk_trh_setup(1, &tmp1, &tmp2);
    printf("RISCV: tr1 thr_fct = 0x%x 0x%x\n", tmp1, tmp2);
    pget_trk_trh_setup(2, &tmp1, &tmp2);
    printf("RISCV: tr2 thr_fct = 0x%x 0x%x\n", tmp1, tmp2);
    pget_trk_trh_setup(3, &tmp1, &tmp2);
    printf("RISCV: tr3 thr_fct = 0x%x 0x%x\n", tmp1, tmp2);
#endif

    /* Voltage waveform filtering parameters, per track */
    pset_trk_flt_setup(1, &flt_in_1);
    pset_trk_flt_setup(2, &flt_in_2);
    pset_trk_flt_setup(3, &flt_in_3);
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
#if 0
    printf("RISCV: tr3 filtering:\n");
    printf("  osr = %d %d %d %d\n"
              , flt_in_3.flens[0], flt_in_3.flens[1]
              , flt_in_3.flens[2], flt_in_3.flens[3] );
    printf("  scale_up = %d %d %d %d\n"
              , flt_in_3.fscale_up[0], flt_in_3.fscale_up[1]
              , flt_in_3.fscale_up[2], flt_in_3.fscale_up[3] );
    printf("  speed_thr = %d %d %d %d\n"
              , flt_in_3.spd_thr[0], flt_in_3.spd_thr[1]
              , flt_in_3.spd_thr[2], flt_in_3.spd_thr[3] );
    printf("  thr_fct = %d %d %d %d\n"
              , flt_in_3.thr_fct[0], flt_in_3.thr_fct[1]
              , flt_in_3.thr_fct[2], flt_in_3.thr_fct[3] );
    printf("  scale_dn = %d\n", flt_in_3.fscale_dn);
    printf("  init_idx = %d\n", flt_in_3.ilen);
#endif

    /* Arrays for storing raw MSR data, combined or per track */
#if MSR_RAW_ARRAYS_COMBINED
    pset_trk_arrays(0, MSR_RAW_ALL_X, MSR_RAW_ALL_Y, MSR_RAW_ALL_LEN);
#else
    pset_trk_arrays(1, MSR_RAW_TR1_X, MSR_RAW_TR1_Y, MSR_RAW_TR1_LEN);
    pset_trk_arrays(2, MSR_RAW_TR2_X, MSR_RAW_TR2_Y, MSR_RAW_TR2_LEN);
    pset_trk_arrays(3, MSR_RAW_TR3_X, MSR_RAW_TR3_Y, MSR_RAW_TR3_LEN);
#endif

#if 0
    pget_trk_arrays(1, &tmp1, &tmp2, &tmp3);
    printf("RISCV: tr1 raw_x @ 0x%x  raw_y @ 0x%x len = %d\n", tmp1, tmp2, tmp3);
    pget_trk_arrays(2, &tmp1, &tmp2, &tmp3);
    printf("RISCV: tr2 raw_x @ 0x%x  raw_y @ 0x%x len = %d\n", tmp1, tmp2, tmp3);
    pget_trk_arrays(3, &tmp1, &tmp2, &tmp3);
    printf("RISCV: tr3 raw_x @ 0x%x  raw_y @ 0x%x len = %d\n", tmp1, tmp2, tmp3);
#endif

    /* Track encoding parameters (app should not change these) */
    pset_trk_params(1, &trk_par_in_1);
    pset_trk_params(2, &trk_par_in_2);
    pset_trk_params(3, &trk_par_in_3);
#if 0
    pget_trk_params(1, &trk_par_out);
    printf("RISCV: tr1 parameters:\n");
    printf("  start sentinel = 0x%x\n", trk_par_out.startsen);
    printf("  end sentinel = 0x%x\n", trk_par_out.endsen);
    printf("  char width = %d bit\n", trk_par_out.nbits);
    printf("  bit density = %d bit/in\n", trk_par_out.density);
    printf("  char set = %s\n", trk_par_out.charset);
#endif
#if 0
    pget_trk_params(2, &trk_par_out);
    printf("RISCV: tr2 parameters:\n");
    printf("  start sentinel = 0x%x\n", trk_par_out.startsen);
    printf("  end sentinel = 0x%x\n", trk_par_out.endsen);
    printf("  char width = %d bit\n", trk_par_out.nbits);
    printf("  bit density = %d bit/in\n", trk_par_out.density);
    printf("  char set = %s\n", trk_par_out.charset);
#endif
#if 0
    pget_trk_params(3, &trk_par_out);
    printf("RISCV: tr3 parameters:\n");
    printf("  start sentinel = 0x%x\n", trk_par_out.startsen);
    printf("  end sentinel = 0x%x\n", trk_par_out.endsen);
    printf("  char width = %d bit\n", trk_par_out.nbits);
    printf("  bit density = %d bit/in\n", trk_par_out.density);
    printf("  char set = %s\n", trk_par_out.charset);
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
