/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "spi.h"
#include "afe.h"
#include "hart_uart.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "mxc_device.h"
#include "nvic_table.h"

#include "afe_gpio.h"
#include "afe_timer.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"
#include "uart.h"
#include "uart_revb.h"
#include "uart_common.h"
#include "mcr_regs.h"
#include "dma.h"

#if (TARGET_NUM == 32680)
#include "ptg_regs.h"
#include "pt_regs.h"
#endif

// Defines
// #define HART_CLK_4MHZ_CHECK

// 20 Preamble, 1 Delimiter, 5 address, 3 Expansion, 1 Command, 1 Byte Count, 255 Max Data, 1 Check Byte
#define HART_UART_MAX_PACKET_LEN 286

// This is used to guarantee at least 2 bytes in UART transmit FIFO, which are
//  necessary to trigger the MXC_F_UART_INT_FL_TX_OB interrupt.  That is, one
//  byte left in FIFO, which occurs when FIFO level goes from 2 to 1.
//  Since first write to FIFO will immediately drop through to shift register
//  we need 3 bytes.
// NOTE: Due to all real HART messages containing at least 5 preambles etc. this
//  should never be an issues unless caller specifies incorrect length.
#define HART_UART_MIN_PACKET_LENGTH 3

#define HART_UART_FIFO_SIZE 8

// Only arm the transmission complete callback when FIFO level is this or less
#define HART_UART_FIFO_LEVEL_TO_ARM_CALLBACK 1

// This is used while waiting for the current HART transmission to finish.
//  The primary timeout is calculated in hart_uart_transmit_irq, but if we
//  arrive in the callback and transmission is still in progress we will
//  schedule another callback this many microseconds later to recheck.
//  Each bit in the 1200 baud transmission is 1/1200 834us.
//  Currently delaying 2 characters more.
#define HART_TRANSMIT_COMPLETE_RETRY_IN_US (834 * 2)

// Use 128 times oversampling of demodulated HART serial data for optimal bit decoding.
#define UART_RX_BIT_OVERSAMPLING_128 0

// Detect and handle UART Framing, Parity and Overrun Errors
#define MXC_UART_ERRINT_EN \
    (MXC_F_UART_INT_EN_RX_FERR | MXC_F_UART_INT_EN_RX_PAR | MXC_F_UART_INT_EN_RX_OV)

#define MXC_UART_ERRINT_FL \
    (MXC_F_UART_INT_FL_RX_FERR | MXC_F_UART_INT_FL_RX_PAR | MXC_F_UART_INT_FL_RX_OV)

// Enables for the interrupts we care about
#define HART_UART_INTERRUPT_ENABLES                                                   \
    ((MXC_F_UART_INT_EN_TX_HE | MXC_F_UART_INT_EN_TX_OB | MXC_F_UART_INT_EN_RX_THD) | \
     (MXC_UART_ERRINT_EN))

// Note, this is internally bonded, but is Y bonded to MAX32675 package as well, as pin: 51, aka P1.8
#if (TARGET_NUM == 32675)
#define HART_UART_INSTANCE MXC_UART2

#define HART_RTS_GPIO_PORT MXC_GPIO1
#define HART_RTS_GPIO_PIN MXC_GPIO_PIN_8

#define HART_CD_GPIO_PORT MXC_GPIO0
#define HART_CD_GPIO_PIN MXC_GPIO_PIN_16

#define HART_IN_GPIO_PORT MXC_GPIO0
#define HART_IN_GPIO_PIN MXC_GPIO_PIN_15

#define HART_OUT_GPIO_PORT MXC_GPIO0
#define HART_OUT_GPIO_PIN MXC_GPIO_PIN_14

#define HART_CLK_GPIO_PORT MXC_GPIO0
#define HART_CLK_GPIO_PIN MXC_GPIO_PIN_10

#define PCLKDIV_DIV_BY_4 2

#elif (TARGET_NUM == 32680)
#define HART_UART_INSTANCE MXC_UART0

#define HART_RTS_GPIO_PORT MXC_GPIO0
#define HART_RTS_GPIO_PIN MXC_GPIO_PIN_3

#define HART_CD_GPIO_PORT MXC_GPIO0
#define HART_CD_GPIO_PIN MXC_GPIO_PIN_2

#define HART_IN_GPIO_PORT MXC_GPIO0
#define HART_IN_GPIO_PIN MXC_GPIO_PIN_1

#define HART_OUT_GPIO_PORT MXC_GPIO0
#define HART_OUT_GPIO_PIN MXC_GPIO_PIN_0

#define HART_CLK_GPIO_PORT MXC_GPIO0
#define HART_CLK_GPIO_PIN MXC_GPIO_PIN_18
#define HART_CLK_GPIO_ALT_FUNC MXC_GPIO_FUNC_ALT1

#endif

// Globals
volatile uint8_t hart_buf[HART_UART_MAX_PACKET_LEN]; // used for transmit and receive
volatile uint32_t hart_uart_reception_len = 0;
volatile uint32_t hart_uart_transmission_len = 0;
volatile uint32_t hart_uart_transmission_byte_index = 0;
volatile uint32_t hart_uart_transmission_complete = 0;
volatile int32_t hart_uart_reception_avail = 0;
volatile uint32_t hart_receive_active = 0;
volatile uint32_t hart_uart_errors = 0;

/**
 * Global callback structure
 */
hart_uart_callbacks_t sap_callbacks = { NULL, NULL, NULL, NULL, NULL };

#if (TARGET_NUM == 32680)
mxc_pt_regs_t *pPT0 = MXC_PT0;
mxc_ptg_regs_t *pPTG = MXC_PTG;
#endif

#ifdef HART_CLK_4MHZ_CHECK
mxc_pt_regs_t *pPT2 = MXC_PT2;
#endif

// Prototypes
void hart_cd_isr(void *cbdata);
void hart_uart_irq_handler(void);

static int hart_uart_init(mxc_uart_regs_t *uart, unsigned int baud, mxc_uart_clock_t clock)
{
    int retval = E_NO_ERROR;

    retval = MXC_UART_Shutdown(uart);
    if (retval) {
        return retval;
    }

    switch (clock) {
    case MXC_UART_EXT_CLK:
        MXC_AFE_GPIO_Config(&gpio_cfg_extclk);
        break;

    case MXC_UART_ERTCO_CLK:
        return E_BAD_PARAM;
        break;

    case MXC_UART_IBRO_CLK:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        break;

    case MXC_UART_ERFO_CLK:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERFO);
        break;

    default:
        break;
    }

    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_AFE_GPIO_Config(&gpio_cfg_uart0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;

    case 2:
        MXC_AFE_GPIO_Config(&gpio_cfg_uart2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;

    default:
        return E_NOT_SUPPORTED;
    }

    retval = MXC_UART_RevB_Init((mxc_uart_revb_regs_t *)uart, baud, (mxc_uart_revb_clock_t)clock);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Set RX threshold to 1 byte
    retval = MXC_UART_SetRXThreshold(uart, 1);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Set Datasize to 8 bits
    retval = MXC_UART_SetDataSize(uart, 8);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // HART messages use ODD parity
    retval = MXC_UART_SetParity(uart, MXC_UART_PARITY_ODD_0);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_UART_SetStopBits(uart, MXC_UART_STOP_1);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // IMPORTANT: by default the SDK uart driver uses only 4 samples per bit
    //  This can cause issues for HART as the bit times are not always
    //  perfectly sized at 833.3uS.  Up to 15% inaccuracy observed.
    //  Increase oversampling to 128x to improve receiver accuracy
    uart->osr = UART_RX_BIT_OVERSAMPLING_128;

    // NOTE: RTS is handled by software, so the gpio_cfg_uart2_flow structure in pins_me15.c
    // only describes the CTS pin.
    // OCD from HART modem is hooked up to CTS, But CD doesn't function like CTS, So this
    // is handled by software as well.

    // Clear any flags
    MXC_UART_ClearFlags(uart, MXC_UART_GetFlags(uart));

    // Enable FIFO threshold exceeded so we can drain it into receive buffer
    retval = MXC_UART_EnableInt(uart, HART_UART_INTERRUPT_ENABLES);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Overwrite default UART IRQ Vector with ours
    MXC_NVIC_SetVector(MXC_UART_GET_IRQ(MXC_UART_GET_IDX(uart)), hart_uart_irq_handler);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(MXC_UART_GET_IDX(uart)));

    return E_NO_ERROR;
}

static int setup_rts_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_rts;

    hart_rts.port = HART_RTS_GPIO_PORT;
    hart_rts.mask = HART_RTS_GPIO_PIN;
    hart_rts.pad = MXC_GPIO_PAD_NONE;
    hart_rts.func = MXC_GPIO_FUNC_OUT;
    hart_rts.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_rts);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_GPIO_OutSet(HART_RTS_GPIO_PORT, HART_RTS_GPIO_PIN);

    return retval;
}

static int idle_rts_pin(void)
{
    int retval = 0;
    mxc_gpio_cfg_t hart_rts;

    hart_rts.port = HART_RTS_GPIO_PORT;
    hart_rts.mask = HART_RTS_GPIO_PIN;
    hart_rts.pad = MXC_GPIO_PAD_PULL_DOWN;
    hart_rts.func = MXC_GPIO_FUNC_OUT;
    hart_rts.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_rts);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int setup_cd_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_cd;

    hart_cd.port = HART_CD_GPIO_PORT;
    hart_cd.mask = HART_CD_GPIO_PIN;
    hart_cd.pad = MXC_GPIO_PAD_NONE;
    hart_cd.func = MXC_GPIO_FUNC_IN;
    hart_cd.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_cd);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_GPIO_RegisterCallback(&hart_cd, hart_cd_isr, NULL);

    retval = MXC_GPIO_IntConfig(&hart_cd, MXC_GPIO_INT_BOTH);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_GPIO_EnableInt(hart_cd.port, hart_cd.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(hart_cd.port)));

    return retval;
}

static int idle_cd_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_cd;

    hart_cd.port = HART_CD_GPIO_PORT;
    hart_cd.mask = HART_CD_GPIO_PIN;
    hart_cd.pad = MXC_GPIO_PAD_PULL_DOWN;
    hart_cd.func = MXC_GPIO_FUNC_IN;
    hart_cd.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_cd);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static uint32_t get_hart_cd_state(void)
{
    return MXC_GPIO_InGet(HART_CD_GPIO_PORT, HART_CD_GPIO_PIN);
}

static int setup_hart_in_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_in;

    hart_in.port = HART_IN_GPIO_PORT;
    hart_in.mask = HART_IN_GPIO_PIN;
    hart_in.pad = MXC_GPIO_PAD_NONE;
    hart_in.func = MXC_GPIO_FUNC_OUT;
    hart_in.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_in);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Clear (0) means 2200 signal
    MXC_GPIO_OutClr(HART_IN_GPIO_PORT, HART_IN_GPIO_PIN);

    return retval;
}

static int idle_hart_in_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_in;

    hart_in.port = HART_IN_GPIO_PORT;
    hart_in.mask = HART_IN_GPIO_PIN;
    hart_in.pad = MXC_GPIO_PAD_PULL_DOWN;
    hart_in.func = MXC_GPIO_FUNC_OUT;
    hart_in.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_in);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int setup_hart_out_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_out;

    hart_out.port = HART_OUT_GPIO_PORT;
    hart_out.mask = HART_OUT_GPIO_PIN;
    hart_out.pad = MXC_GPIO_PAD_NONE;
    hart_out.func = MXC_GPIO_FUNC_IN;
    hart_out.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_out);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int idle_hart_out_pin(void)
{
    int retval = E_NO_ERROR;
    mxc_gpio_cfg_t hart_out;

    hart_out.port = HART_OUT_GPIO_PORT;
    hart_out.mask = HART_OUT_GPIO_PIN;
    hart_out.pad = MXC_GPIO_PAD_PULL_DOWN;
    hart_out.func = MXC_GPIO_FUNC_OUT;
    hart_out.vssel = MXC_GPIO_VSSEL_VDDIOH;

    retval = MXC_AFE_GPIO_Config(&hart_out);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int hart_uart_pins_idle_mode(void)
{
    // To avoid floating inputs, when HART is NOT enabled, pull the HART UART pins low
    // NOTE: this only handles the HART uart pins, not the HART clock output
    int retval = E_NO_ERROR;

    retval = idle_hart_in_pin();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = idle_rts_pin();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = idle_cd_pin();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = idle_hart_out_pin();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

void hart_rts_transmit_mode(void)
{
    MXC_GPIO_OutClr(HART_RTS_GPIO_PORT, HART_RTS_GPIO_PIN);
}

void hart_rts_receive_mode(void)
{
    MXC_GPIO_OutSet(HART_RTS_GPIO_PORT, HART_RTS_GPIO_PIN);
}

void hart_sap_enable_request(uint32_t state)
{
    if (state == HART_STATE_TRANSMIT) {
        hart_rts_transmit_mode();
    } else {
        hart_rts_receive_mode();
    }

    // Call Enable.confirm(state)
    if (sap_callbacks.enable_confirm_cb) {
        if (state == HART_STATE_TRANSMIT) {
            sap_callbacks.enable_confirm_cb(HART_STATE_TRANSMIT);
        } else {
            sap_callbacks.enable_confirm_cb(HART_STATE_IDLE);
        }
    }
}

// TODO(ADI): Consider adding some parameters to this function to specify
//  clock divider settings. Complicated due to differences in MAX32675 and
//  MAX32680 divider output methodologies.
int hart_clock_enable(void)
{
    int retval = E_NO_ERROR;

#if (TARGET_NUM == 32675)

    // ERFO Crystal is required for the HART device in the AFE
    retval = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERFO);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // Put clock output pin in correct mode
    //

    // To avoid any glitches on the clock output first configure the in I/O mode
    HART_CLK_GPIO_PORT->en0_set = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->out_clr = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->outen_set = HART_CLK_GPIO_PIN;

    // Transition to requested alternate function clearing en0 last
    HART_CLK_GPIO_PORT->en2_set = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->en1_set = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->en0_clr = HART_CLK_GPIO_PIN;

    // Leave GPIO pad pull down enabled, pulls are overridden in IO and alternate function modes.
    //  But will take over if clock is later disabled.
    HART_CLK_GPIO_PORT->padctrl0 |= HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->ps &= ~HART_CLK_GPIO_PIN;

    // Since we have 16Mhz External RF Oscillator (ERFO)
    // We need to divide it by 4 to get desired 4Mhz output (DIV_CLK_OUT_CTRL of 2)
    MXC_GCR->pclkdiv &= ~(MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_CTRL | MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_EN);
    MXC_GCR->pclkdiv |= ((PCLKDIV_DIV_BY_4 << MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_CTRL_POS) &
                         MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_CTRL);
    MXC_GCR->pclkdiv |= MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_EN;

#elif (TARGET_NUM == 32680)
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PT);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PT);

    //set clock scale, DIV1
    MXC_GCR->clkctrl &= ~MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128;
    MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1;

    //disable all PT0
    pPTG->enable = ~0x01;

    //clear PT0 interrupt flag
    pPTG->intfl = 0x01;

    //enable ISO before enabling ERFO
    MXC_GCR->btleldoctrl |= (MXC_F_GCR_BTLELDOCTRL_LDOTXEN | MXC_F_GCR_BTLELDOCTRL_LDORXEN |
                             MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL0 | MXC_F_GCR_BTLELDOCTRL_LDOTXVSEL1 |
                             MXC_F_GCR_BTLELDOCTRL_LDORXVSEL0 | MXC_F_GCR_BTLELDOCTRL_LDORXVSEL1);

    MXC_GCR->clkctrl |= MXC_F_GCR_CLKCTRL_ISO_EN;

    MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ISO_RDY);

    retval = MXC_SYS_Clock_Timeout(MXC_F_GCR_CLKCTRL_ISO_RDY);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERFO);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //change to ERFO before starting PT.
    retval = MXC_SYS_Clock_Select(MXC_SYS_CLOCK_ERFO);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    MXC_SYS_SetClockDiv(MXC_SYS_CLOCK_DIV_1);

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_PT);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_PT);

    //set clock scale, DIV1
    MXC_GCR->clkctrl &= ~MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128;
    MXC_GCR->clkctrl |= MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1;

    //disable all PT0
    pPTG->enable = ~0x01;

    //clear PT0 interrupt flag
    pPTG->intfl = 0x01;

    //4MHz frequency
    pPT0->rate_length =
        ((2 << MXC_F_PT_RATE_LENGTH_RATE_CONTROL_POS) & MXC_F_PT_RATE_LENGTH_RATE_CONTROL) |
        ((2 << MXC_F_PT_RATE_LENGTH_MODE_POS) & MXC_F_PT_RATE_LENGTH_MODE);

    //50% duty cycle
    pPT0->train = 1;

    //number of cycles (infinite)
    pPT0->loop = ((0 << MXC_F_PT_LOOP_COUNT_POS) & MXC_F_PT_LOOP_COUNT) |
                 ((0 << MXC_F_PT_LOOP_DELAY_POS) & MXC_F_PT_LOOP_DELAY);

    retval = MXC_AFE_GPIO_Config(&gpio_cfg_pt0);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#ifdef HART_CLK_4MHZ_CHECK
    //4MHz frequency
    pPT2->rate_length =
        ((2 << MXC_F_PT_RATE_LENGTH_RATE_CONTROL_POS) & MXC_F_PT_RATE_LENGTH_RATE_CONTROL) |
        ((2 << MXC_F_PT_RATE_LENGTH_MODE_POS) & MXC_F_PT_RATE_LENGTH_MODE);

    //50% duty cycle
    pPT2->train = 1;

    //number of cycles (infinite) for PT2 (P0_16)
    pPT2->loop = ((0 << MXC_F_PT_LOOP_COUNT_POS) & MXC_F_PT_LOOP_COUNT) |
                 ((0 << MXC_F_PT_LOOP_DELAY_POS) & MXC_F_PT_LOOP_DELAY);

    retval = MXC_AFE_GPIO_Config(&gpio_cfg_pt2);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    pPTG->enable |= MXC_F_PTG_ENABLE_PT0 | MXC_F_PTG_ENABLE_PT2;
#else
    pPTG->enable |= MXC_F_PTG_ENABLE_PT0;
#endif // End of HART_CLK_4MHZ_CHECK

    //wait for PT to start
    while ((pPTG->enable & (MXC_F_PTG_ENABLE_PT0)) != MXC_F_PTG_ENABLE_PT0) {}

#endif // End of (TARGET_NUM == 32680)

    return retval;
}

static void idle_hart_clock_pin(void)
{
    // To avoid potential glitches when changing from Alternate functions
    //  First return to IO mode before clearing AF selections
    HART_CLK_GPIO_PORT->en0_set = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->out_clr = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->outen_set = HART_CLK_GPIO_PIN;

    HART_CLK_GPIO_PORT->en1_clr = HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->en2_clr = HART_CLK_GPIO_PIN;

    // GPIO pad pull down enabled
    HART_CLK_GPIO_PORT->padctrl0 |= HART_CLK_GPIO_PIN;
    HART_CLK_GPIO_PORT->ps &= ~HART_CLK_GPIO_PIN;

    return;
}

void hart_clock_disable(void)
{
#if (TARGET_NUM == 32675)
    MXC_GCR->pclkdiv &= ~MXC_F_GCR_PCLKDIV_DIV_CLK_OUT_EN;
#elif (TARGET_NUM == 32680)
    pPTG->enable &= ~MXC_F_PTG_ENABLE_PT0;
#endif // End of (TARGET_NUM == 32675)

    // Before disabling the HART clock output, configure pin pull-down
    idle_hart_clock_pin();

    return;
}

int hart_uart_enable(void)
{
    int retval = E_NO_ERROR;
    uint32_t read_val = 0;

    retval = afe_read_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, &read_val);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    read_val |= MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN;

    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, read_val);

    return retval;
}

int hart_uart_disable(void)
{
    int retval = E_NO_ERROR;
    uint32_t read_val = 0;

    retval = afe_read_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, &read_val);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    read_val &= ~MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN;

    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_SYS_CTRL, read_val);

    return retval;
}

int hart_uart_takedown(void)
{
    int retval = E_NO_ERROR;

    // Set the HART uart pins in correct mode for idling
    retval = hart_uart_pins_idle_mode();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = hart_uart_disable();

    if (retval != E_NO_ERROR) {
        return retval;
    }

    hart_clock_disable();

    return retval;
}

void hart_uart_setup_saps(hart_uart_callbacks_t callbacks)
{
    // Set global callback structure
    sap_callbacks = callbacks;
}

int hart_uart_setup(uint32_t test_mode)
{
    int retval = E_NO_ERROR;
    uint32_t read_val = 0;

    //
    // NOTE: enable HART Clock output last to avoid any unexpected behaviors by
    //  HART state machine.
    //

    //
    // Any HART configuration modifications must be made before enabling HART.
    //

    //
    // Utilize character based re-timing on received HART data.
    //  Provides enhanced noise tolerance.
    //
    retval = afe_read_register(MXC_R_AFE_HART_RX_TX_CTL, &read_val);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    read_val |= MXC_F_AFE_HART_RX_TX_CTL_RX_DOUT_UART_EN;

    retval = afe_write_register(MXC_R_AFE_HART_RX_TX_CTL, read_val);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // Enable HART mode on ME19/AFE.
    //  Puts AFE gpios 2-5 into UART mode
    //

    retval = hart_uart_enable();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // HART UART gpio configuration
    //
    //  Note, all four UART pins on the ME19/AFE are just floating inputs
    //  until HART_EN in SYS_CTRL is enabled.
    //
    // Seems safer to enable the ME19 HART UART side first, and endure
    //  contention until the ME15 side is configured.  This means a small
    //  known current drawn between the ME19 HART OUT high and the ME15 HART OUT
    //  pin pull down. Better than floating inputs on the 19 that may consume mAs.

    // The HART OUT pin will have contention, configure it first to minimize current.
    retval = setup_hart_out_pin();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = setup_rts_pin();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = setup_cd_pin();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    if (test_mode) {
        // Test mode is required for physical layer test to output constant bit
        // Frequencies.

        // Force mode for constant transmit.
        retval = setup_hart_in_pin();
        if (retval != E_NO_ERROR) {
            return retval;
        }

        if (test_mode == HART_TEST_MODE_TX_1200) {
            // Set (1) means 1200 signal
            MXC_GPIO_OutSet(HART_IN_GPIO_PORT, HART_IN_GPIO_PIN);
        } else {
            // Clear (0) means 2200 signal
            MXC_GPIO_OutClr(HART_IN_GPIO_PORT, HART_IN_GPIO_PIN);
        }

        hart_rts_transmit_mode(); // start transmit
    } else {
        hart_rts_receive_mode();

        retval = hart_uart_init(HART_UART_INSTANCE, 1200, MXC_UART_ERFO_CLK);
        if (retval != E_NO_ERROR) {
            return E_COMM_ERR;
        }
    }

    //
    // Finally turn on HART 4Mhz Clock
    //
    retval = hart_clock_enable();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

void hart_sap_reset_request(void)
{
    // First disable HART mode, idle pins etc.
    hart_uart_takedown();

    // Now set it back up
    // NOTE: Assuming there is no reason to for any test modes while using
    //  this SAP, defaulting to normal hart transceive mode.
    hart_uart_setup(NORMAL_HART_TRANSCEIVE_MODE);

    // Call RESET.confirm()
    if (sap_callbacks.reset_confirm_cb) {
        sap_callbacks.reset_confirm_cb();
    }
}

void hart_uart_test_transmit_1200(void)
{
    // Set (1) means 1200 signal
    MXC_GPIO_OutSet(HART_IN_GPIO_PORT, HART_IN_GPIO_PIN);
}

void hart_uart_test_transmit_2200(void)
{
    // Clear (0) means 2200 signal
    MXC_GPIO_OutClr(HART_IN_GPIO_PORT, HART_IN_GPIO_PIN);
}

int hart_uart_load_tx_fifo(void)
{
    uint32_t num_written = 0;
    uint32_t load_num = 0;
    uint32_t avail = MXC_UART_GetTXFIFOAvailable(HART_UART_INSTANCE);
    int32_t left_to_send = hart_uart_transmission_len - hart_uart_transmission_byte_index;

    // Figure out how many more bytes we need to send
    if (left_to_send > 0) {
        if (left_to_send > avail) {
            load_num = avail;
        } else {
            load_num = left_to_send;
        }
    } else {
        // Nothing to send
        return 0;
    }

    num_written = MXC_UART_WriteTXFIFO(
        HART_UART_INSTANCE, (const unsigned char *)(hart_buf + hart_uart_transmission_byte_index),
        load_num);

    // Advance index based on bytes actually written.
    // This should always equal load_num though.
    hart_uart_transmission_byte_index += num_written;

    // Return the number actually written in case someone is interested.
    return num_written;
}

int hart_uart_send(uint8_t *data, uint32_t length)
{
    // Ensure the line is quiet before beginning transmission
    if (hart_receive_active) {
        return E_BUSY;
    }

    if (!data) {
        return E_NULL_PTR;
    }

    if (length < HART_UART_MIN_PACKET_LENGTH) {
        // Message is too short to engage proper UART interrupts
        // Or if length is 0, there is nothing to do, do not toggle
        //  RTS disabling receive mode.
        return E_BAD_PARAM;
    }

    if (length > HART_UART_MAX_PACKET_LEN) {
        // Message is longer than allowed, do not overflow our buffer
        return E_BAD_PARAM;
    }

    // Set length to transmit, reset transmit index, clear complete
    hart_uart_transmission_len = length;
    hart_uart_transmission_byte_index = 0;
    hart_uart_transmission_complete = 0;

    // Copy data to our internal buffer
    memcpy((uint8_t *)hart_buf, data, hart_uart_transmission_len);

    // NOTE: we are not forcing preamble
    hart_rts_transmit_mode();

    // Load up the FIFO and allow interrupts handle the rest
    hart_uart_load_tx_fifo();

    return E_SUCCESS;
}

int hart_sap_data_request(uint8_t data)
{
    int retval = E_NO_ERROR;

    retval = MXC_UART_WriteCharacter(HART_UART_INSTANCE, data);

    if (retval != E_SUCCESS) {
        // data was not written correctly
        // Most likely, FIFO is full, let caller know
        return retval;
    }

    // Call Enable.confirm(state)
    if (sap_callbacks.data_confirm_cb) {
        sap_callbacks.data_confirm_cb(data);
    }

    return retval;
}

static uint8_t convert_comm_errors(uint32_t int_flags)
{
    // Convert from UART peripheral error status into formate used by TPDLL
    uint8_t comm_status = TPDLL_COMM_ERROR_INDICATOR;

    if (int_flags & MXC_F_UART_INT_FL_RX_FERR) {
        comm_status |= TPDLL_FRAMING_ERROR;
    }

    if (int_flags & MXC_F_UART_INT_FL_RX_PAR) {
        comm_status |= TPDLL_VERTICAL_PARITY_ERROR;
    }

    if (int_flags & MXC_F_UART_INT_FL_RX_OV) {
        comm_status |= TPDLL_OVERRUN_ERROR;
    }

    return comm_status;
}

void hart_uart_receive_irq(uint32_t uart_err, uint32_t uart_flags)
{
    int retval = E_NO_ERROR;
    uint8_t sap_comm_error_status = 0;

    // Communications Errors, (Parity, Framing, Overrun)
    // Note: Continue attempting to receive, even if error occurs
    if (uart_err) {
        __disable_irq();
        // NOTE: This global error gets cleared in CD ISR
        //  Or in new errors without clearing any previous
        hart_uart_errors |= uart_err;
        sap_comm_error_status = convert_comm_errors(uart_err);
        __enable_irq();
    }

    // Data Available, Threshold should be set to 1
    if (uart_flags & MXC_F_UART_INT_FL_RX_THD) {
        // Should only be one byte available in FIFO, but handle more anyway
        while (1) {
            // Read out any available chars
            retval = MXC_UART_ReadCharacterRaw(HART_UART_INSTANCE);

            if (retval >= 0) {
                // Only place into our buffer if Carrier Detect is set
                // Look at other IRQ flag in critical section for safety
                __disable_irq();

                if (hart_receive_active) {
                    if (hart_uart_reception_len < HART_UART_MAX_PACKET_LEN) {
                        hart_buf[hart_uart_reception_len++] = retval;
                    } else {
                        hart_uart_errors |= UART_FLAG_BUFFER_OVERFLOW_ERROR;
                        sap_comm_error_status |= TPDLL_BUFFER_OVERFLOW_ERROR;
                    }

                    // SAP
                    // If any error occurred during this reception, send ERROR.Indicate
                    if (sap_comm_error_status) {
                        // Call ERROR.Indicate(status, data)
                        if (sap_callbacks.error_indicate_cb) {
                            sap_callbacks.error_indicate_cb(sap_comm_error_status, retval);
                        }
                    } else {
                        // Call DATA.Indicate(data)
                        if (sap_callbacks.data_indicate_cb) {
                            sap_callbacks.data_indicate_cb(retval);
                        }
                    }
                }

                __enable_irq();

            } else {
                // Otherwise nothing available
                break;
            }
        }
    } else {
        // No data in the FIFO, but if we do have an error report it.
        // NOTE: in some cases error flags will be set but data will be dropped
        if (sap_comm_error_status) {
            // Call ERROR.Indicate(status, data)
            if (sap_callbacks.error_indicate_cb) {
                sap_callbacks.error_indicate_cb(sap_comm_error_status, 0);
            }
        }
    }
}

void hart_uart_trans_complete_callback(void)
{
    // Check to see if the HART UART is actually complete, since this is called
    //  via calculated delay.
    if (MXC_UART_GetStatus(HART_UART_INSTANCE) & MXC_F_UART_STATUS_TX_BUSY) {
        // Still busy transmitting.
        // We don't want to spend time here polling for completion so register
        // another callback and try again.

        // Start another wait for SPI TX to complete
        afe_timer_delay_async(HART_TIMER, HART_TRANSMIT_COMPLETE_RETRY_IN_US,
                              (mxc_delay_complete_t)hart_uart_trans_complete_callback);
    } else {
        // Transmission is DONE, release RTS to return to receive mode
        hart_uart_transmission_complete = 1;
        hart_rts_receive_mode();

        // Call Enable.confirm(state)
        if (sap_callbacks.enable_confirm_cb) {
            sap_callbacks.enable_confirm_cb(HART_STATE_IDLE);
        }
    }
}

void hart_uart_transmit_irq(void)
{
    // This ISR should only be called when Transmit FIFO is Half or Almost Empty
    // NOTE: Need to use Almost Empty here as well, in the case that a short
    //  HART message is send.
    //
    //  However, smallest HART packet would still contain 9 Bytes. So this is
    //  just for safety
    //  5 Preambles (0xFF), 1 Delimiter, 1 Command, 1 Byte Count, 1 Check Byte
    uint32_t bytes_yet_to_load = 0;
    uint32_t bytes_in_fifo = 0;
    uint32_t transmit_complete_in_us = 0;

    // Update pointers etc in critical section
    __disable_irq();

    // Send more bytes.  Also updates transmit index
    hart_uart_load_tx_fifo();

    // Check how many more bytes left to load in FIFO
    if (hart_uart_transmission_byte_index < hart_uart_transmission_len) {
        bytes_yet_to_load = hart_uart_transmission_len - hart_uart_transmission_byte_index;
    } else {
        bytes_yet_to_load = 0;
    }

    if (bytes_yet_to_load == 0) {
        //
        // NOTE: due to lack of interrupt source for transmit complete we will arm
        //  a timer callback to terminate the transmission and switch to receive.
        //

        //
        // To avoid arming multiple timer callbacks, ONLY arm when one byte
        //  aka TX FIFO Almost Empty interrupt occurs, AND no more bytes are
        //  left to place in FIFO
        //
        bytes_in_fifo = HART_UART_FIFO_SIZE - MXC_UART_GetTXFIFOAvailable(HART_UART_INSTANCE);

        if (bytes_in_fifo <= HART_UART_FIFO_LEVEL_TO_ARM_CALLBACK) {
            // Assume that this interrupt is called shortly after next byte leaves the
            //  FIFO and is now going out from the shift register.  So number
            //  of bytes in FIFO + 1 (shift register) * 11 bits at 1200 baud.
            //  1/1200 = 834us per bit.
            transmit_complete_in_us = (bytes_in_fifo + 1) * 11 * 834;

            // Start timeout, wait for SPI TX to complete
            afe_timer_delay_async(HART_TIMER, transmit_complete_in_us,
                                  (mxc_delay_complete_t)hart_uart_trans_complete_callback);
        }
    }

    __enable_irq();
}

void hart_uart_irq_handler(void)
{
    uint32_t uart_flags = MXC_UART_GetFlags(HART_UART_INSTANCE);
    uint32_t uart_err = 0;

    // Clear any flags
    MXC_UART_ClearFlags(HART_UART_INSTANCE, uart_flags);

    // Check for reception error
    uart_err = uart_flags & MXC_UART_ERRINT_FL & HART_UART_INSTANCE->int_en;

    // Handle receive events
    if ((uart_err) || (uart_flags & MXC_F_UART_INT_FL_RX_THD)) {
        hart_uart_receive_irq(uart_err, uart_flags);
    }

    // Handle transmit events
    if (uart_flags & (MXC_F_UART_INT_FL_TX_HE | MXC_F_UART_INT_FL_TX_OB)) {
        hart_uart_transmit_irq();
    }
}

void hart_cd_isr(void *cbdata)
{
    // NOTE: cbdata is setup to be null

    if (get_hart_cd_state()) {
        // HART CD is high, reception active
        // NOTE: could be carrier without data though
        hart_receive_active = 1;
        // Reset global error status
        hart_uart_errors = 0;

    } else {
        // HART CD is low, NO reception active
        hart_receive_active = 0;

        // TODO(ADI): If RX threshold is ever increased: drain rx fifo here
        if (hart_uart_reception_len > 0) {
            // Got some chars
            hart_uart_reception_avail = 1;
        }
    }

    // Call Enable.Indicate(state)
    if (sap_callbacks.enable_indicate_cb) {
        if (hart_receive_active) {
            sap_callbacks.enable_indicate_cb(HART_STATE_RECEIVE_ACTIVE);
        } else {
            sap_callbacks.enable_indicate_cb(HART_STATE_IDLE);
        }
    }
}

int hart_uart_check_for_receive()
{
    // NOTE: RTS is placed into receive mode by hart_uart_send
    // Receive mode is default operation for the HART UART

    // The HART modem output Carrier Detect (CD) is connected to the UART CTS pin
    // reception is handled in interrupt context

    return hart_uart_reception_avail;
}

int hart_uart_check_transmit_complete()
{
    return hart_uart_transmission_complete;
}

int hart_uart_get_received_packet(uint8_t *buffer, uint32_t *packet_length, uint32_t *comm_errors)
{
    if (!buffer) {
        return E_NULL_PTR;
    }

    if (!packet_length) {
        return E_NULL_PTR;
    }

    if (!comm_errors) {
        return E_NULL_PTR;
    }

    if (hart_receive_active) {
        *packet_length = 0;
        *comm_errors = 0;
        return E_BUSY;
    }

    if (!hart_uart_reception_avail) {
        // No reception available
        *packet_length = 0;
        *comm_errors = 0;
        return E_NONE_AVAIL;
    }

    // Update interrupt variables in critical section
    __disable_irq();

    *packet_length = hart_uart_reception_len;
    *comm_errors = hart_uart_errors;
    hart_uart_reception_avail = 0;
    hart_uart_reception_len = 0;

    // Otherwise, copy received data for return
    memcpy(buffer, (uint8_t *)hart_buf, *packet_length);

    __enable_irq();

    // Finally report any communications errors
    if (hart_uart_errors) {
        return E_COMM_ERR;
    }

    return E_SUCCESS;
}
