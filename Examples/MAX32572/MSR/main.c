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
 * @file    main.c
 * @brief   Magnetic Stripe Reader ARM
 *
 * @details This example demonstrates basic MSR functionality
 */

/***** Includes *****/
#include <MAX32xxx.h>
#include "fcr_regs.h"
#include "sema_regs.h"
#include "riscv_rom.h"

/***** Definitions *****/

/*  Select RISC-V memory access wait state setting
    Options: slow (0) or fast (1)
*/
#define FAST_WS  (0)

/*  Select RISC-V code starting point
    Options: execute ROM code (1) or execute SRAM code (0)
    If ROM code selected then RISC-V code in SRAM will be ignored
    If SRAM code selected, it may call functions in ROM code
*/
#define RUN_RISCV_ROM_ONLY (0)

/*  Select communication channel between RISCV and ARM
    Options: use IRQ_RISCV (1) or use shared SRAM location (0)
*/
#define USE_IRQ_FOR_READY_SIGNAL_FROM_RISCV (0)

/* RISC-V related memory wait state control bits */
#define MEMCTRL_RISCV_WS (MXC_F_GCR_MEMCTRL_SRAM5_WS \
        | MXC_F_GCR_MEMCTRL_SRAM6_WS | MXC_F_GCR_MEMCTRL_ROM1_WS)

/***** Globals *****/

/* Symbols defined when loading RISCV image */
extern uint32_t _binary_riscv_bin_start;
extern uint32_t _binary_riscv_bin_end;
uint32_t riscv_load_addr = { 
#include "riscv_load.addr"
};
uint32_t riscv_text_addr = { 
#include "riscv_text.addr"
};

int swipe_complete;
int swp_cnt, tr1_cnt, tr2_cnt, tr3_cnt;


/***** Functions *****/
void print_swipe(volatile msr_decoded_track_t *decoded_swipe);

void RISCV_IRQHandler(void)
{
    /* Clear RISC-V interrupt flags */
    MXC_SEMA -> irq1 &= ~MXC_F_SEMA_IRQ1_RV32_IRQ;
    NVIC_ClearPendingIRQ(RISCV_IRQn);

    /* indicate swipe complete */
    swipe_complete = TRUE;
}

// *****************************************************************************
int main(void)
{
    printf("\nARM: *********** MSR Example ***********\n");

    /* Load RISCV image from FLASH to SRAM */
    {
        volatile uint32_t * psrc = &_binary_riscv_bin_start;
        volatile uint32_t * pdst = (uint32_t *)riscv_load_addr;
        while (psrc < &_binary_riscv_bin_end) {
            *pdst++ = *psrc++;
        }
    }

#if FAST_WS
    /* Set fast wait state (_WS = 0) */
    MXC_GCR -> memctrl &= ~MEMCTRL_RISCV_WS;
#else
    /* Set slow wait state (_WS = 1) */
    MXC_GCR -> memctrl |=  MEMCTRL_RISCV_WS;
#endif


#if RUN_RISCV_ROM_ONLY
    /* Run RV ROM code */
    MXC_FCR->urvbootaddr = RV_START_ROM; /* Set RISC-V boot address */
#else
    /* Run RV code in SRAM */
    MXC_FCR->urvbootaddr = riscv_text_addr; /* Set RISC-V boot address */
#endif


#if USE_IRQ_FOR_READY_SIGNAL_FROM_RISCV
    msr_cfg |= MSR_CFG_CM4IRQ_ENABLE;
#else
    msr_cfg &= ~MSR_CFG_CM4IRQ_ENABLE;
#endif

    /* Init ARM side application */
    msr_ctrl = 0;
    swipe_complete = FALSE;
    swp_cnt = 0;
    tr1_cnt = 0;
    tr2_cnt = 0;
    tr3_cnt = 0;

    printf("ARM: Starting RISC-V at %08x, %s %s \n", MXC_FCR->urvbootaddr,__DATE__,__TIME__);
    NVIC_EnableIRQ(RISCV_IRQn);

    /* delay starting RISCV until printf() complete */
    while (MXC_UART_GetActive(MXC_UART_GET_UART(CONSOLE_UART)) == E_BUSY) {}

    /* Start RISC-V core */
    MXC_GCR->pclkdis1 &= ~MXC_F_GCR_PCLKDIS1_CPU1; /* enable RISCV clock */

    /* wait for MSR READY signal from RV */
#if USE_IRQ_FOR_READY_SIGNAL_FROM_RISCV
    while (!swipe_complete) { MXC_Delay(MSEC(10)); }
    swipe_complete = FALSE;
#else
    while (!msr_ctrl) { MXC_Delay(MSEC(10)); } /* RISCV will set non-0 when ready */
#endif

    printf("ARM: RISC-V MSR ROM version %d.%d started and ready\n"
            , msr_version_hi, msr_version_lo);
    printf("ARM: Memory control at %s Wait State\n"
            , (MXC_GCR -> memctrl & MEMCTRL_RISCV_WS) ? "Slow" : "Fast");


    /* swipe loop */
    while (1) {

        swp_cnt++;
        printf("\nARM: Waiting for swipe %d...\n", swp_cnt);

        /*  This will demonstrate ARM-to-RISCV communication options:
            ARM can initiate swipe by IRQ_CM4 or by writing shared SRAM
            RISCV can signal swipe complete by IRQ_RISCV or by writing shared SRAM
            Following code will rotate through these four options
            Real application need only one of the four options
         */
        switch (swp_cnt & 0x03) {

        case 0:
            /*  ARM initiates swipe by writing shared SRAM
                RISCV signals swipe complete by writing shared SRAM
            */
            msr_cfg &= ~MSR_CFG_CM4IRQ_ENABLE;  /* disable IRQ when swipe complete */
            msr_ctrl = 0;                       /* initiate swipe */
            /* Wait for swipe complete signal. RISCV will set non-zero value */
            while (!msr_ctrl) { MXC_Delay(MSEC(10)); }
            print_swipe(msr_decoded_track);
            printf(" Swipe was initiated by SRAM, complete by SRAM\n");
            break;

        case 1:
            /*  ARM initiates swipe by writing shared SRAM
                RISCV signals swipe complete by IRQ_RISCV
            */
            msr_cfg |= MSR_CFG_CM4IRQ_ENABLE;   /* enable IRQ when swipe complete */
            msr_ctrl = 0;                       /* initiate swipe */
            /* Wait for swipe complete signal. RISCV will fire IRQ */
            while (!swipe_complete) { MXC_Delay(MSEC(10)); }
            swipe_complete = FALSE;
            print_swipe(msr_decoded_track);
            printf(" Swipe was initiated by SRAM, complete by IRQ_RISCV\n");
            break;

        case 2:
            /*  ARM initiates swipe by IRQ_CM4
                RISCV signals swipe complete by writing shared SRAM
            */
            msr_cfg &= ~MSR_CFG_CM4IRQ_ENABLE;  /* disable IRQ when swipe complete */
            MXC_SEMA -> irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ + MXC_F_SEMA_IRQ0_EN;   /* initiate swipe */
            while (msr_ctrl) { MXC_Delay(MSEC(10)); } /* make sure RISCV is busy swiping */
            /* Wait for swipe complete signal. RISCV will set non-zero value */
            while (!msr_ctrl) { MXC_Delay(MSEC(10)); }
            print_swipe(msr_decoded_track);
            printf(" Swipe was initiated by IRQ_CM4, complete by SRAM\n");
            break;

        case 3:
            /*  ARM initiates swipe by IRQ_CM4
                RISCV signals swipe complete by IRQ_RISCV
            */
            msr_cfg |= MSR_CFG_CM4IRQ_ENABLE;   /* enable IRQ when swipe complete */
            MXC_SEMA -> irq0 |= MXC_F_SEMA_IRQ0_CM4_IRQ + MXC_F_SEMA_IRQ0_EN;   /* initiate swipe */
            while (msr_ctrl) { MXC_Delay(MSEC(10)); } /* make sure RISCV is busy swiping */
            /* Wait for swipe complete signal. RISCV will fire IRQ */
            while (!swipe_complete) { MXC_Delay(MSEC(10)); }
            swipe_complete = FALSE;
            print_swipe(msr_decoded_track);
            printf(" Swipe was initiated by IRQ_CM4, complete by IRQ_RISCV\n");
            break;
    }

    MXC_Delay(MSEC(100)); /* delay (to avoid back to back swipes) */
  }
  return 0;
}


/* This will print swipe results */
void print_swipe(volatile msr_decoded_track_t *decoded_swipe)
{
    uint16_t err;

    printf("ARM printing swipe %d results:\n", swp_cnt);

    if (msr_ctrl_exit_code == GETSWIPE_OK) {
        /* swipe complete OK, print track information */

        for (int ii = 0; ii < 3; ii++) {
            printf(" === Track %u === ", ii+1);
            err = decoded_swipe[ii].error_code;

            /* Print data decoding errors if any detected */
            if (err != MSR_ERR_OK) {
                printf("Error: 0x%02x ", err);
                if (err & MSR_ERR_BAD_LEN)   { printf(" BAD_LEN"); }
                if (err & MSR_ERR_START_SEN) { printf(" START_SEN"); }
                if (err & MSR_ERR_END_SEN)   { printf(" END_SEN"); }
                if (err & MSR_ERR_OUTLIER)   { printf(" OUTLIER"); }
                if (err & MSR_ERR_PARITY)    { printf(" PARITY"); }
                if (err & MSR_ERR_LRC)       { printf(" LRC"); }
            }
            printf("\n");
            printf(" LRC check %s\n", decoded_swipe[ii].lrc ? "failed" : "passed");

            printf(" Decoded %d chars, ", decoded_swipe[ii].len);
            if ( decoded_swipe[ii].speed  &&  decoded_swipe[ii].len ) {
                printf("%s, ", decoded_swipe[ii].direction == MSR_FORWARD ?
                            "Forward" : "Reverse");
                printf("Rate %u.%u in/sec\n"
                        , decoded_swipe[ii].speed / 10,  decoded_swipe[ii].speed % 10);
            } else {
                printf("Direction and Rate invalid\n");
            }

            /* Print track data characters */
            printf(" %s\n", decoded_swipe[ii].data);
        }

    } else if (msr_ctrl_exit_code == GETSWIPE_ERROR) {
        /* swipe complete with errors */
        err = msr_ctrl_err_status;

        printf(" Errors: ");
        if (err & ADCERR_OVERRUN)    printf("ADC Overrun ");
        if (err & ADCERR_SHIFT)      printf("ADC Shift ");
        if (err & ADCERR_INCOMPLETE) printf("ADC Incomplete ");
        if (err & HIGH_NOISE)        printf("High Noise ");
        printf("\n");

    } else if (msr_ctrl_exit_code == GETSWIPE_TIMO) {
        /* no swipe detected within specified time */
        printf(" Swipe timed out.\n");
    }

    /* count successfully decoded tracks */
    if (decoded_swipe[0].error_code == MSR_ERR_OK) tr1_cnt++;
    if (decoded_swipe[1].error_code == MSR_ERR_OK) tr2_cnt++;
    if (decoded_swipe[2].error_code == MSR_ERR_OK) tr3_cnt++;
    printf(" Total (%d-%d-%d) successful tracks in %d swipes\n",
                tr1_cnt, tr2_cnt, tr3_cnt, swp_cnt);

    fflush(stdout);
}
