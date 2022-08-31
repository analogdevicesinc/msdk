/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
 *
 * This software is protected by copyright laws of the United States and
 * of foreign countries. This material may also be protected by patent laws
 * and technology transfer regulations of the United States and of foreign
 * countries. This software is furnished under a license agreement and/or a
 * nondisclosure agreement and may only be used or reproduced in accordance
 * with the terms of those agreements. Dissemination of this information to
 * any party or parties not specified in the license agreement and/or
 * nondisclosure agreement is expressly prohibited.
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
 *******************************************************************************
 */

/* Global includes */
#include <stdio.h>

#include "MAX32xxx.h"
#include "msr.h"

/* Select SDMA instance to run MSR on {0,1} */
#define MSR_SDMA_INSTANCE (0)

#if MSR_SDMA_INSTANCE == 0
#define MSR_SDMA MXC_SDMA0
#define MSR_SDMA_IRQn HA0_IRQn
#define SDMAx_IRQHandler HA0_IRQHandler
#define CLK_DIS_SDMAxD MXC_SYS_PERIPH_CLOCK_HA0
#else
#define MSR_SDMA MXC_SDMA1
#define MSR_SDMA_IRQn HA1_IRQn // SDMA1_IRQn
#define SDMAx_IRQHandler HA1_IRQHandler
#define CLK_DIS_SDMAxD MXC_SYS_PERIPH_CLOCK_HA1
#endif

/* Global swipe statistics data */
static int Processed = 0;
static int Discarded = 0;
static int Discarded_track[3] = { 0, 0, 0 };

/* SDMA Code */
extern unsigned char msr_sdma_code[];

/* MSR SDMA ISR */
static volatile uint32_t sdma_irq_flag;

/* SHARED section layout */
volatile uint32_t* msr_version = (uint32_t*)0x20000000;
volatile uint16_t* msr_ctrl_ptr = (uint16_t*)0x20000004;
volatile uint16_t* adc9_err_ptr = (uint16_t*)0x20000006;
volatile uint32_t* swipe_timeout_sec_ptr = (uint32_t*)0x20000008;
volatile mcr_decoded_track_t* decoded_track = (mcr_decoded_track_t*)0x2000000C;

#define msr_version (*msr_version)
#define msr_ctrl (*msr_ctrl_ptr)
#define adc9_err (*adc9_err_ptr)
#define swipe_timeout_sec (*swipe_timeout_sec_ptr)

void SDMAx_IRQHandler(void)
{
    MSR_SDMA->irq_flag = 1;
    sdma_irq_flag = 1;
}

/* This will start MSR SDMA */
static void start_msr_sdma(void)
{
    /* Enable SDMA clock */
    MXC_SYS_ClockEnable(CLK_DIS_SDMAxD);

    /* Disable interrupt from SDMA */
    /* will wait for irq_flag=1 but dont want IRQHandler() to run */
    NVIC_DisableIRQ(MSR_SDMA_IRQn);
    MSR_SDMA->irq_ie = 0;

#define ADC9_IRQ_VECTOR 22

    /* Enable interrupt mux */
    MSR_SDMA->int_mux_ctrl0 = (ADC9_IRQ_VECTOR << MXC_F_SDMA_INT_MUX_CTRL0_INTSEL16_POS);
    MSR_SDMA->int_mux_ctrl1 = 0;
    MSR_SDMA->int_mux_ctrl2 = 0;
    MSR_SDMA->int_mux_ctrl3 = 0;
    /* Set org address and enable sdma */
    MSR_SDMA->ip_addr = (uint32_t)&msr_sdma_code;
    MSR_SDMA->ctrl |= MXC_F_SDMA_CTRL_EN;
    /* Wait for the SDMA to finish re-initialization */
    while (!MSR_SDMA->irq_flag) {
        {
        }
    }
    /* Clear irq_flag */
    MSR_SDMA->irq_flag = 1;
    NVIC_ClearPendingIRQ(MSR_SDMA_IRQn);
    /* Re-Enable interrupt from SDMA */
    NVIC_EnableIRQ(MSR_SDMA_IRQn);
    MSR_SDMA->irq_ie |= MXC_F_SDMA_IRQ_IE_IRQ_EN;
    MSR_SDMA->int_in_ie = 1;
}

/* This will stop MSR SDMA */
static void stop_msr_sdma(void)
{
    /* Disable interrupt from SDMA */
    NVIC_DisableIRQ(MSR_SDMA_IRQn);
    MSR_SDMA->irq_ie = 0;

    /* stop SDMA */
    MSR_SDMA->ctrl &= ~(MXC_F_SDMA_CTRL_EN);

    /* Disable interrupt mux */
    MSR_SDMA->int_mux_ctrl0 = 0;
    MSR_SDMA->int_mux_ctrl1 = 0;
    MSR_SDMA->int_mux_ctrl2 = 0;
    MSR_SDMA->int_mux_ctrl3 = 0;

    /* Disable SDMA clock */
    MXC_SYS_ClockDisable(CLK_DIS_SDMAxD);
}

/* This will print decoded swipe data */
static void ProcessSwipe(void)
{
    unsigned int tidx;

    for (tidx = 0; tidx < MCR_NUM_TRACKS; tidx++) {
        /* Display the decoding results */
        printf("\r\n--=  Track %u  =--\r\n", tidx + 1);

        if (decoded_track[tidx].error_code != MCR_ERR_OK) {
            printf("Error: 0x%02x", decoded_track[tidx].error_code);
            if (decoded_track[tidx].error_code & MCR_ERR_BAD_LEN) {
                printf(" BAD_LEN");
            }
            if (decoded_track[tidx].error_code & MCR_ERR_START_SEN) {
                printf(" START_SEN");
            }
            if (decoded_track[tidx].error_code & MCR_ERR_END_SEN) {
                printf(" END_SEN");
            }
            if (decoded_track[tidx].error_code & MCR_ERR_OUTLIER) {
                printf(" OUTLIER");
            }
            if (decoded_track[tidx].error_code & MCR_ERR_PARITY) {
                printf(" PARITY");
            }
            if (decoded_track[tidx].error_code & MCR_ERR_LRC) {
                printf(" LRC");
            }
            printf("\r\n");
        }

        if (decoded_track[tidx].lrc == 0) {
            printf("LRC check passed\r\n");
        } else {
            printf("LRC check failed\r\n");
        }

        printf("Decoded %d chars, ", decoded_track[tidx].len);
        if (decoded_track[tidx].speed && decoded_track[tidx].len) {
            if (decoded_track[tidx].direction == MCR_FORWARD) {
                printf("Forward, ");
            } else {
                printf("Reverse, ");
            }
            printf("Rate %u.%u in/sec\r\n", decoded_track[tidx].speed / 10,
                decoded_track[tidx].speed % 10);
        } else {
            printf("Direction and Rate invalid\r\n");
        }

        printf("%s\r\n", decoded_track[tidx].data);

        /* Capture Statistics */
        Processed++;
        if ((decoded_track[tidx].len == 0) || (decoded_track[tidx].lrc != 0)) {
            Discarded++;
            Discarded_track[tidx]++;
        }
    }

    printf("\r\nDecode Stats:\r\n");
    printf("  Tracks %d, Discarded %d  (%d %d %d)\r\n", Processed, Discarded, Discarded_track[0],
        Discarded_track[1], Discarded_track[2]);
}

int main(void)
{
    //
    printf("\nWelcome to Magnetic Stripe Reader Application\n");

    while (1) {
        printf("\n\n\n---------------------------------------------------------------\n");

        swipe_timeout_sec = 30; /* Set swipe timeout */

        printf("Start MSR on SDMA%1d \n", MSR_SDMA_INSTANCE);
        start_msr_sdma();
        printf("MSR SDMA Verion %d.%d\n", msr_version >> 16, msr_version & 0xffff);

        printf("\nWaiting for swipe ...\n");

        msr_ctrl = GETSWIPE_BUSY; /* signal to SDMA to get swipe data */
        sdma_irq_flag = 0;
        while (!sdma_irq_flag) {
            {
            }
        }

        /* check exit code */
        switch (msr_ctrl) {
        case GETSWIPE_OK:
            ProcessSwipe(); /* Print Swipe results */
            break;
        case GETSWIPE_ADCERR:
            /* report ADC errors */
            if (adc9_err & ADCERR_OVERRUN)
                printf("Error: ADC9 overrun\r\n");
            else if (adc9_err & ADCERR_SHIFT)
                printf("Error: ADC9 channels shifted\r\n");
            else if (adc9_err & ADCERR_INCOMPLETE)
                printf("Error: ADC9 conversion incomplete\r\n");
            break;
        case GETSWIPE_TIMO: /* Timeout: no swipe */
            printf("No swipe for %d seconds\r\n", swipe_timeout_sec);
            break;
        }

        //
        stop_msr_sdma();
    }

    return 0;
}
