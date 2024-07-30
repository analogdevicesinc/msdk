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

#include <stdint.h>
#include <stdio.h>
#include "spi.h"
#include "afe.h"
#include "mxc_sys.h"
#include "hart_uart.h"
#include "gcr_regs.h"

#include "afe_gpio.h"
#include "afe_timer.h"
#include "gpio.h"
#include "gpio_reva.h"
#include "gpio_common.h"

// Known Device revision defines
#define MXC_REVISION_MASK (0x00FF)
#define MXC_MAX32675_REV_A1 (0x00A1)
#define MXC_MAX32675_REV_A2 (0x00A2)
#define MXC_MAX32675_REV_A3 (0x00A3)
#define MXC_MAX32675_REV_B2 (0x00B2)
#define MXC_MAX32675_REV_B3 (0x00B3)
#define MXC_MAX32675_REV_B4 (0x00B4)

#define MXC_MAX32680_REV_A1 (0x00A1)
#define MXC_MAX32680_REV_B1 (0x00B1)

#define MXC_AFE_VERSION_ORIGINAL 0
#define MXC_AFE_VERSION_PRE_RESET 1
#define MXC_AFE_VERSION_POST_RESET 2

// AFE SPI Port Configuration
#if (TARGET_NUM == 32675)
#define AFE_SPI_PORT MXC_SPI0
#elif (TARGET_NUM == 32680)
#define AFE_SPI_PORT MXC_SPI1
#endif

#define AFE_SPI_BAUD 1000000 // Can only run up to PCLK speed
#define AFE_SPI_BIT_WIDTH 8
#define AFE_SPI_SSEL_PIN 1
#define AFE_SPI_ADDRESS_LEN 1

// AFE Trim Storage Defines
//#define DUMP_TRIM_DATA

//
// Enabling this will use the afe timer to timeout polling of the AFE.
//  This avoid any chance of the blocking function afe_spi_transceive from never returning.
//  However, as this low level function can be called several times for even a single
//  AFE register read, it decreases throughput to and from the AFE.
//  When running at lower system clock frequencies this can be more of a concern.
//
//#define AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER

#ifdef AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER
//
// This timeout prevents infinite loop if AFE is not responsive via SPI
//
//  Each read from AFE is 8 bits, unless CRC is enabled.
//  maximum of 4 reads.
//      1/AFE_SPI_BAUD * 4 * 8 should be a reasonable timeout for this.
//      1/1000000 = 1us * 4 * 8 = 32us
//
//  Since this is just for catastrophic lockup, lets round it up to 100
//      this will also be a even multiple for MXC_AFE_SPI_RESET_TIMEOUT_ITERATES
//
#if (AFE_SPI_BAUD != 1000000)
#warning "Recalculate MXC_AFE_SPI_READ_TIMEOUT, since baud rate was modified.\n"
#endif
#endif // End of AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER

#define MXC_AFE_SPI_READ_TIMEOUT USEC(100)

//
// Initial AFE RESET time, up to 10 milliseconds
//  Due to nature of AFE_TIMER, it cannot support nested Timeouts
//  Therefore, count a number of read timeouts instead.
//
#define MXC_AFE_SPI_RESET_TIMEOUT_ITERATES (MSEC(10) / MXC_AFE_SPI_READ_TIMEOUT)

#if (TARGET_NUM == 32675)
#define AFE_TRIM_OTP_OFFSET_LOW 0x280
#define AFE_TRIM_OTP_OFFSET_HIGH 0x288

#define AFE_SPI_MISO_GPIO_PORT MXC_GPIO0
#define AFE_SPI_MISO_GPIO_PIN MXC_GPIO_PIN_2

#define AFE_SPI_MOSI_GPIO_PORT MXC_GPIO0
#define AFE_SPI_MOSI_GPIO_PIN MXC_GPIO_PIN_3

#define AFE_SPI_SCK_GPIO_PORT MXC_GPIO0
#define AFE_SPI_SCK_GPIO_PIN MXC_GPIO_PIN_4

#define AFE_SPI_SSEL_GPIO_PORT MXC_GPIO0
#define AFE_SPI_SSEL_GPIO_PIN MXC_GPIO_PIN_5

#elif (TARGET_NUM == 32680)
#define AFE_TRIM_OTP_OFFSET_LOW 0x0E10
#define AFE_TRIM_OTP_OFFSET_HIGH 0x0E18

#define AFE_SPI_MISO_GPIO_PORT MXC_GPIO0
#define AFE_SPI_MISO_GPIO_PIN MXC_GPIO_PIN_20

#define AFE_SPI_MOSI_GPIO_PORT MXC_GPIO0
#define AFE_SPI_MOSI_GPIO_PIN MXC_GPIO_PIN_21

#define AFE_SPI_SCK_GPIO_PORT MXC_GPIO0
#define AFE_SPI_SCK_GPIO_PIN MXC_GPIO_PIN_22

#define AFE_SPI_SSEL_GPIO_PORT MXC_GPIO0
#define AFE_SPI_SSEL_GPIO_PIN MXC_GPIO_PIN_23

//
// Register defines for new AFE reset.
// TODO(ADI): This bit does not yet exist for MAX32680.
//  This is just a placeholder for now to prevent compilation errors.
//
#define MXC_F_GCR_RST1_AFE_POS 26 /**< RST1_AFE Position */
#define MXC_F_GCR_RST1_AFE ((uint32_t)(0x1UL << MXC_F_GCR_RST1_AFE_POS)) /**< RST1_AFE Mask */
#endif

#define AFE_TRIM0_ADC0_MASK 0x7FFFF
#define AFE_TRIM0_ADC0_BIT_WIDTH 19

#define AFE_TRIM1_ADC0_MASK 0x3FFF
#define AFE_TRIM1_ADC0_BIT_WIDTH 14

#define AFE_TRIM_DAC_MASK 0x0FFFF
#define AFE_TRIM_DAC_BIT_WIDTH 16

#define AFE_TRIM_ANA_ADC0_MASK 0x7FFF
#define AFE_TRIM_ANA_ADC0_BIT_WIDTH 15

#define AFE_TRIM0_ADC1_MASK 0x7FFFF
#define AFE_TRIM0_ADC1_BIT_WIDTH 19

#define AFE_TRIM1_ADC1_MASK 0x3FFF
#define AFE_TRIM1_ADC1_BIT_WIDTH 14

#define AFE_TRIM_HART_MASK 0x0FFFFF
#define AFE_TRIM_HART_BIT_WIDTH 20

// NOTE: These two bits are embedded inside the HART trim
#define AFE_TRIM_ANA_ADC1_MASK 0x060
#define AFE_TRIM_ANA_ADC1_BIT_WIDTH 2
#define AFE_TRIM_ANA_ADC1_OFFSET_1 5 // bit position in HART trim
#define AFE_TRIM_ANA_ADC1_OFFSET_2 10 // bit position in ANA TRIM ADC1

#define AFE_TRIM_VREF_MASK 0x7FF
#define AFE_TRIM_VREF_BIT_WIDTH 11

// Largest Possible AFE SPI transaction (BYTES): Address 1, Max Data 4, CRC 2
#define AFE_SPI_MAX_DATA_LEN 7

//
// Masks and values used for validation of AFE SPI out of reset
//  Uses entire bottom nibble match, AND por_flag
//
// Revisions AFTER Reset added to AFE
#define ME16_AFE_POST_RST_SYS_CTRL_TRUE_POR_MASK                                  \
    (MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL | MXC_F_AFE_ADC_ZERO_SYS_CTRL_CRC5 | \
     MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS | MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG)
#define ME16_AFE_POST_RST_SYS_CTRL_TRUE_POR_VALUE \
    (MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS | MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG)
#define ME16_AFE_POST_RST_SYS_CTRL_RESET_VALUE (MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS)

// Revisions BEFORE Reset added to AFE
#define ME16_AFE_PRE_RST_SYS_CTRL_TRUE_POR_MASK (MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG)
#define ME16_AFE_PRE_RST_SYS_CTRL_TRUE_POR_VALUE (MXC_F_AFE_ADC_ZERO_SYS_CTRL_POR_FLAG)
#define ME16_AFE_PRE_RST_SYS_CTRL_RESET_VALUE (0)

/***** Globals *****/
uint8_t afe_data[AFE_SPI_MAX_DATA_LEN];
mxc_spi_regs_t *pSPIm = AFE_SPI_PORT;
int check_done = 0;

typedef struct {
    uint32_t adc_trim0_adc0;
    uint32_t adc_trim0_adc1;
    uint32_t adc_trim1_adc0;
    uint32_t adc_trim1_adc1;
    uint32_t ana_trim_adc0;
    uint32_t vref_trim;
    uint32_t hart_trim;
    uint32_t ana_trim_adc1;
    uint32_t dac_trim;
} trim_data_t;

trim_data_t trim_data;

uint32_t current_register_bank = 0;
uint32_t adc0_conversion_active = 0;
uint32_t adc1_conversion_active = 0;
uint32_t device_version = 0;

#if (TARGET_NUM == 32675)

static mxc_gpio_cfg_t gpio_cfg_spi0 = {
    MXC_GPIO0, (MXC_GPIO_PIN_2 | MXC_GPIO_PIN_3 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_5),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE
};

#elif (TARGET_NUM == 32680)

static mxc_gpio_cfg_t gpio_cfg_spi1 = {
    MXC_GPIO0, (MXC_GPIO_PIN_20 | MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23),
    MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE
};
#endif

/***** Private Prototypes *****/
static int raw_afe_write_register(uint8_t reg_address, uint32_t value, uint8_t reg_length);
static int raw_afe_read_register(uint8_t reg_address, uint32_t *value, uint8_t reg_length);

/***** Functions *****/

// Probe and determine silicon version of AFE micro
static int afe_micro_version_probe(void)
{
    // Due to changes in the latest AFE revision, different initialization procedures are required.

    uint32_t rev = MXC_GCR->revision;

    // Mask off metal option details, only care about revision here
    rev &= MXC_REVISION_MASK;

    //
    // Decode AFE version from micro version
    //
#if (TARGET_NUM == 32675)
    switch (rev) {
    // Known and supported versions
    case MXC_MAX32675_REV_A1:
    case MXC_MAX32675_REV_A2:
    case MXC_MAX32675_REV_A3:
    case MXC_MAX32675_REV_B2:
    case MXC_MAX32675_REV_B3:
        device_version = MXC_AFE_VERSION_PRE_RESET;
        return E_NO_ERROR;

    case MXC_MAX32675_REV_B4:
        device_version = MXC_AFE_VERSION_POST_RESET;
        return E_NO_ERROR;

    default:
        // Unknown or unsupported version
        return E_INVALID;
    }
#elif (TARGET_NUM == 32680)
    switch (rev) {
    // Known and supported versions
    case MXC_MAX32680_REV_A1:
        device_version = MXC_AFE_VERSION_PRE_RESET;
        return E_NO_ERROR;

    case MXC_MAX32680_REV_B1:
        // TODO(ADI): Validate this revision is correct when updated MAX32680 is released
        device_version = MXC_AFE_VERSION_POST_RESET;
        return E_NO_ERROR;

    default:
        // Unknown or unsupported version
        return E_INVALID;
    }
#else
#error "Selected TARGET is not known to have an AFE\n"
#endif

    return E_NO_ERROR;
}

// Puts the SPI interface to the AFE into controlled inactive state
static void afe_spi_idle_interface(void)
{
    // Make sure SSEL is inactive first
    // SSEL should output 1 (inactive)
    AFE_SPI_SSEL_GPIO_PORT->out_set = AFE_SPI_SSEL_GPIO_PIN;
    AFE_SPI_SSEL_GPIO_PORT->outen_set = AFE_SPI_SSEL_GPIO_PIN;
    AFE_SPI_SSEL_GPIO_PORT->en0_set = AFE_SPI_SSEL_GPIO_PIN;

    // SCK output 0
    AFE_SPI_SCK_GPIO_PORT->out_clr = AFE_SPI_SCK_GPIO_PIN;
    AFE_SPI_SCK_GPIO_PORT->outen_set = AFE_SPI_SCK_GPIO_PIN;
    AFE_SPI_SCK_GPIO_PORT->en0_set = AFE_SPI_SCK_GPIO_PIN;

    // MISO will always be strong driven from AFE.
    //  Therefore, put in tristate input mode
    AFE_SPI_MISO_GPIO_PORT->outen_clr = AFE_SPI_MISO_GPIO_PIN;
    AFE_SPI_MISO_GPIO_PORT->padctrl0 &= ~AFE_SPI_MISO_GPIO_PIN;
    AFE_SPI_MISO_GPIO_PORT->padctrl1 &= ~AFE_SPI_MISO_GPIO_PIN;

    // Only setting en0 here, to avoid any chance of glitch when set to AF mode later
    AFE_SPI_MISO_GPIO_PORT->en0_set = AFE_SPI_MISO_GPIO_PIN;

    // MOSI output 0
    AFE_SPI_MOSI_GPIO_PORT->out_clr = AFE_SPI_MOSI_GPIO_PIN;
    AFE_SPI_MOSI_GPIO_PORT->outen_set = AFE_SPI_MOSI_GPIO_PIN;
    AFE_SPI_MOSI_GPIO_PORT->en0_set = AFE_SPI_MOSI_GPIO_PIN;

    return;
}

static int afe_spi_setup(void)
{
    int retval = 0;

    // Enable SPI Periph clock, and reset it
#if (TARGET_NUM == 32675)
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI0);

#elif (TARGET_NUM == 32680)
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
#endif

    // Initialize global check for previous SPI transaction finished
    check_done = 0;

    // NOTE: AFE uses SPI Mode 0, which is reset default

    // Use SDK SPI driver to set the baud rate, as these depends what periph clock we are at
    retval = MXC_SPI_SetFrequency(AFE_SPI_PORT, AFE_SPI_BAUD);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    pSPIm->ctrl2 |= ((AFE_SPI_BIT_WIDTH << MXC_F_SPI_CTRL2_NUMBITS_POS) & MXC_F_SPI_CTRL2_NUMBITS);

    pSPIm->ctrl0 |= (AFE_SPI_SSEL_PIN << MXC_F_SPI_CTRL0_SS_ACTIVE_POS);

    pSPIm->ctrl2 |= MXC_S_SPI_CTRL2_DATA_WIDTH_MONO;

    pSPIm->sstime = ((1 << MXC_F_SPI_SSTIME_PRE_POS) | (1 << MXC_F_SPI_SSTIME_POST_POS) |
                     (1 << MXC_F_SPI_SSTIME_INACT_POS));

    pSPIm->dma = (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    // FIFO Threshold reset defaults are fine
    pSPIm->dma |= MXC_F_SPI_DMA_TX_FIFO_EN;
    pSPIm->dma |= MXC_F_SPI_DMA_RX_FIFO_EN;

    // Don't enable the SPI block until the FIFOs are enabled, to avoid floating any of the SPI interface pins.
    pSPIm->ctrl0 |= (MXC_F_SPI_CTRL0_EN | MXC_F_SPI_CTRL0_MST_MODE);

    // Clear any existing interrupt status
    pSPIm->intfl = pSPIm->intfl;

    //
    // Now that the ME15 side of SPI is setup, configure gpio pads.
    //  Doing this after turning on the SPI periph to avoid floating AFE inputs.
    //
#if (TARGET_NUM == 32675)
    retval = MXC_AFE_GPIO_Config(&gpio_cfg_spi0);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#elif (TARGET_NUM == 32680)
    retval = MXC_AFE_GPIO_Config(&gpio_cfg_spi1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
#endif

    if (device_version < MXC_AFE_VERSION_POST_RESET) {
        //
        // MISO needs a pull down when SPI interface is idle, (this is changed on new silicon)
        //
        AFE_SPI_MISO_GPIO_PORT->padctrl0 |= AFE_SPI_MISO_GPIO_PIN;

        // Ensure the second pull enable is disabled, so we only have to write one register later
        AFE_SPI_MISO_GPIO_PORT->padctrl1 &= ~AFE_SPI_MISO_GPIO_PIN;

        // Pull down
        AFE_SPI_MISO_GPIO_PORT->ps &= ~AFE_SPI_MISO_GPIO_PIN;
    }

    return E_NO_ERROR;
}

#ifdef AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER
//
// This function blocks until transceive is completed, or times out
//  To avoid possibility of getting stuck in a infinite loop, it uses the afe timer
//  to timeout polling.
//
static int afe_spi_transceive(uint8_t *data, int byte_length)
{
    int status = E_NO_ERROR;
    int i = 0;

    if (byte_length > AFE_SPI_MAX_DATA_LEN) {
        return E_OVERFLOW;
    }

    //
    // Ensure transmit FIFO is finished
    // NOTE: if the AFE was reset during transaction this may not finish, so using a timeout
    //

    // Start timeout, wait for SPI TX to complete
    afe_timer_delay_async(AFE_SPI_TIMER, MXC_AFE_SPI_READ_TIMEOUT, NULL);

    do {
        status = afe_timer_delay_check(AFE_SPI_TIMER);

        if ((pSPIm->dma & MXC_F_SPI_DMA_TX_LVL) == 0) {
            // TX completed
            afe_timer_delay_abort(AFE_SPI_TIMER);
            break;
        }
    } while (status == E_BUSY);

    if (status != E_BUSY) {
        return E_TIME_OUT;
    }

    //
    // If a transaction has been started, verify it completed before continuing
    //
    if (check_done) {
        // Start timeout, wait for SPI MST DONE to set
        afe_timer_delay_async(AFE_SPI_TIMER, MXC_AFE_SPI_READ_TIMEOUT, NULL);

        do {
            status = afe_timer_delay_check(AFE_SPI_TIMER);

            if (pSPIm->intfl & MXC_F_SPI_INTFL_MST_DONE) {
                // MST Done
                afe_timer_delay_abort(AFE_SPI_TIMER);
                break;
            }
        } while (status == E_BUSY);

        if (status != E_BUSY) {
            return E_TIME_OUT;
        }
    }

    check_done = 1;

    pSPIm->intfl = pSPIm->intfl;
    pSPIm->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    while (pSPIm->dma & (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH)) {}

    pSPIm->ctrl1 = ((((byte_length) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS)) |
                    (byte_length << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS));

    if (device_version < MXC_AFE_VERSION_POST_RESET) {
        //
        // Legacy: Disable pull down on MISO while transmitting.
        //
        AFE_SPI_MISO_GPIO_PORT->padctrl0 |= AFE_SPI_MISO_GPIO_PIN;
    }

    pSPIm->ctrl0 |= MXC_F_SPI_CTRL0_START;

    // NOTE: At most we will read 32 bits before returning to processing, no streaming data

    //
    // Transmit the data
    //
    for (i = 0; i < byte_length; i++) {
        pSPIm->fifo8[0] = data[i];
    }

    //
    // Receive the data
    //

    // Reset byte counter
    i = 0;

    // Start timeout, wait for SPI receive to complete
    afe_timer_delay_async(AFE_SPI_TIMER, MXC_AFE_SPI_READ_TIMEOUT, NULL);

    do {
        status = afe_timer_delay_check(AFE_SPI_TIMER);

        if ((pSPIm->dma & MXC_F_SPI_DMA_RX_LVL)) {
            data[i] = pSPIm->fifo8[0];
            i++;
        }
    } while ((i < byte_length) && (status == E_BUSY));

    afe_timer_delay_abort(AFE_SPI_TIMER);

    if (device_version < MXC_AFE_VERSION_POST_RESET) {
        //
        // Legacy: Enable pull down on MISO while idle.
        //
        AFE_SPI_MISO_GPIO_PORT->padctrl0 |= AFE_SPI_MISO_GPIO_PIN;
    }

    if ((i < byte_length) || (status != E_BUSY)) {
        return E_TIME_OUT;
    }

    // Got all bytes, and we did NOT timeout
    return E_NO_ERROR;
}
#else // Not AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER

//
// This function blocks until transceive is completed, or times out
//  This version does not use the afe timer to interrupt potential infinite polling.
//
static int afe_spi_transceive(uint8_t *data, int byte_length)
{
    int i = 0;

    if (byte_length > AFE_SPI_MAX_DATA_LEN) {
        return E_OVERFLOW;
    }

    //
    // Ensure transmit FIFO is finished
    // NOTE: if the AFE was reset during transaction this may not finish, so using a timeout
    //
    while ((pSPIm->dma & MXC_F_SPI_DMA_TX_LVL) != 0) {}

    //
    // If a transaction has been started, verify it completed before continuing
    //
    if (check_done) {
        while (!(pSPIm->intfl & MXC_F_SPI_INTFL_MST_DONE)) {}
    }

    check_done = 1;

    pSPIm->intfl = pSPIm->intfl;
    pSPIm->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    while (pSPIm->dma & (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH)) {}

    pSPIm->ctrl1 = ((((byte_length) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS)) |
                    (byte_length << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS));

    if (device_version < MXC_AFE_VERSION_POST_RESET) {
        //
        // Legacy: Disable pull down on MISO while transmitting.
        //
        AFE_SPI_MISO_GPIO_PORT->padctrl0 |= AFE_SPI_MISO_GPIO_PIN;
    }

    pSPIm->ctrl0 |= MXC_F_SPI_CTRL0_START;

    // NOTE: At most we will read 32 bits before returning to processing, no streaming data

    //
    // Transmit the data
    //
    for (i = 0; i < byte_length; i++) {
        pSPIm->fifo8[0] = data[i];
    }

    //
    // Receive the data
    //

    // Reset byte counter
    i = 0;

    do {
        if ((pSPIm->dma & MXC_F_SPI_DMA_RX_LVL)) {
            data[i] = pSPIm->fifo8[0];
            i++;
        }
    } while (i < byte_length);

    if (device_version < MXC_AFE_VERSION_POST_RESET) {
        //
        // Legacy: Enable pull down on MISO while idle.
        //
        AFE_SPI_MISO_GPIO_PORT->padctrl0 |= AFE_SPI_MISO_GPIO_PIN;
    }

    // Got all bytes, and we did NOT timeout
    return E_NO_ERROR;
}
#endif // End of else not AFE_SPI_TRANSCEIVE_SAFE_BUT_SLOWER

static int afe_spi_poll_for_ready_post_reset_change(uint32_t *true_por)
{
    int retval = E_NO_ERROR;
    int read_timeout_count = 0;
    uint32_t read_val = 0;

    //
    // New for ME16A-0D, Validate that SPI interface on AFE is responding
    //  AFE will not correctly respond after a reset for up to 10ms.
    //
    // It is ASSUMED that during the time wih the SPI on AFE is in reset the read output will be 0xFF or 0x00.
    //
    // The sys_ctrl register is modified on this revision so that the following is true:
    //  On a true POR: por_flag (bit 6) and st_dis (bit 3) will BOTH be set to 1, all other bits will be 0
    //      That is 0x48 == POR
    //
    //  Any other reset: por_flag (bit 6) will NOT be set ASSUMING user code has already cleared it to 0
    //    st_dis (bit 3) will be set, and hart_en (bit 4) maybe set or not. Other bits will be 0.
    //

    // NOTE: Only allow 10ms for reset init.  raw_afe_read_register calls afe_spi_transceive
    //  which uses afe_timer, so we cannot also use it here. So we count calls to read_reg
    //  instead.
    do {
        retval = raw_afe_read_register(
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

        // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

        if (retval == E_TIME_OUT) {
            // The low level read timed out. We are counting these if it happens
            //  to many times, will error out and return
            continue;
        }

        if (retval != E_NO_ERROR) {
            return retval;
        }

        // Test mask for TRUE POR
        read_val &= ME16_AFE_POST_RST_SYS_CTRL_TRUE_POR_MASK;

        // Check for True POR
        if (read_val == ME16_AFE_POST_RST_SYS_CTRL_TRUE_POR_VALUE) {
            // This appears to be a true POR
            *true_por = 1;
            break;
        }

        // Check for a normal reset
        if (read_val == ME16_AFE_POST_RST_SYS_CTRL_RESET_VALUE) {
            // This appears to be a normal reset
            *true_por = 0;
            break;
        }
        // Keep counting timeouts, if we get too many, bail out
    } while (read_timeout_count++ < MXC_AFE_SPI_RESET_TIMEOUT_ITERATES);

    if (read_timeout_count >= MXC_AFE_SPI_RESET_TIMEOUT_ITERATES) {
        // Failed to initiate communications with AFE within time limit
        return E_TIME_OUT;
    }

    return E_NO_ERROR;
}

static int afe_spi_poll_for_ready_pre_reset_change(uint32_t *true_por)
{
    int retval = E_NO_ERROR;
    int read_timeout_count = 0;
    uint32_t read_val = 0;

    // Legacy initialization method, in this case we can only detect true POR.
    //  Reset has no effect, we can only look at bit6 POR_FLAG

    // NOTE: Only allow 10ms for reset init.  raw_afe_read_register calls afe_spi_transceive
    //  which uses afe_timer, so we cannot also use it here. So we count calls to read_reg
    //  instead.
    do {
        retval = raw_afe_read_register(
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

        // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

        if (retval == E_TIME_OUT) {
            // The low level read timed out. We are counting these if it happens
            //  to many times, will error out and return
            continue;
        }

        if (retval != E_NO_ERROR) {
            return retval;
        }

        // Test mask for TRUE POR
        read_val &= ME16_AFE_PRE_RST_SYS_CTRL_TRUE_POR_MASK;

        // Check for True POR
        if (read_val == ME16_AFE_PRE_RST_SYS_CTRL_TRUE_POR_VALUE) {
            // This appears to be a true POR
            *true_por = 1;
            break;
        }

        // Check for a normal reset
        if (read_val == ME16_AFE_PRE_RST_SYS_CTRL_RESET_VALUE) {
            // This appears to be a normal reset
            *true_por = 0;
            break;
        }
    } while (read_timeout_count++ < MXC_AFE_SPI_RESET_TIMEOUT_ITERATES);

    if (read_timeout_count >= MXC_AFE_SPI_RESET_TIMEOUT_ITERATES) {
        // Failed to initiate communications with AFE within time limit
        return E_TIME_OUT;
    }

    return E_NO_ERROR;
}

static int afe_spi_poll_for_ready(uint32_t *true_por)
{
    if (device_version >= MXC_AFE_VERSION_POST_RESET) {
        return afe_spi_poll_for_ready_post_reset_change(true_por);
    } else {
        // Treat all earlier revs the same
        return afe_spi_poll_for_ready_pre_reset_change(true_por);
    }
}

static int afe_setup_true_por(void)
{
    //
    // If this is a true POR, then set state as expected
    //  Clear por_flag, st_dis, hart_en, crc_en
    //  Also sets crc_inv to normal aka non-inverted
    //   and forces selection of ADC0 register bank
    //
    // SYS_CTRL should be 0
    //
    int retval = 0;
    uint32_t read_val = 0;

    retval = raw_afe_write_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Read it back as an additional validation of proper SPI operation
    retval = raw_afe_read_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Mask SPI_ABORT_DIS bit to 0 for the following expected value comparison
    read_val &= ~MXC_F_AFE_ADC_ZERO_SYS_CTRL_SPI_ABORT_DIS;

    if (read_val != 0) {
        // Response does NOT matches written and expected value
        return E_COMM_ERR;
    }

    return retval;
}

static int afe_setup_non_por(void)
{
    //
    // This is a non POR reset
    // Do NOT clear st_dis, or hart_en bits.
    //  Clear por_flag, crc_en
    //  Also sets crc_inv to normal aka non-inverted
    //   and forces selection of ADC0 register bank
    //
    // st_dis and hart_en must remain on to avoid disabling HART pin biases once enabled

    int retval = 0;
    uint32_t read_val = 0;

    // Doing a read modify write
    retval = raw_afe_read_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // mask all bits but st_dis, or hart_en bits.
    read_val &= (MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN | MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS);

    retval = raw_afe_write_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Read it back as an additional validation of proper SPI operation
    retval = raw_afe_read_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    // NOTE: Remember that reading sys_ctrl also SETS the SPI_ABORT_DIS bit

    if (retval != E_NO_ERROR) {
        return retval;
    }

    // mask all bits but st_dis, or hart_en bits.
    read_val &= (MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN | MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS);

    if (device_version >= MXC_AFE_VERSION_POST_RESET) {
        // ST_DIS MUST be set, and HART_EN MAY be set
        if ((read_val !=
             (MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN | MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS)) &&
            (read_val != (MXC_F_AFE_ADC_ZERO_SYS_CTRL_ST_DIS))) {
            // Response does NOT matches written and expected value
            return E_COMM_ERR;
        }
    } else {
        // LEGACY silicon mode, the ST_DIS bit CANNOT be set as is doesn't exist, so look for 0
        // However, HART_EN could be set, since we cannot reset the ME19
        if ((read_val != 0) && (read_val != MXC_F_AFE_ADC_ZERO_SYS_CTRL_HART_EN)) {
            // Response does NOT matches written and expected value
            return E_COMM_ERR;
        }
    }

    // Finally before continuing, we disable the HART clock, to avoid any unexpected behaviors.
    // Should already be disabled by System Reset, or afe_reset, so this is just for safety.

    hart_clock_disable();

    return retval;
}

int afe_setup(mxc_tmr_regs_t *tmr)
{
    int retval = 0;
    uint32_t true_por = 0;

    // First setup the AFE
    retval = afe_timer_config(tmr);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Probe for Version to determine initialization procedure
    retval = afe_micro_version_probe();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // NEW on ME16-0D, ensure reset is released for the AFE via GRC
    //  Reset could be active due to SW error recovery.
    //
    if (device_version >= MXC_AFE_VERSION_POST_RESET) {
        MXC_GCR->rst1 &= ~MXC_F_GCR_RST1_AFE;
    }

    retval = afe_spi_setup();
    if (retval != E_NO_ERROR) {
        return retval;
    }

    retval = afe_spi_poll_for_ready(&true_por);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    if (true_por) {
        return afe_setup_true_por();
    } else {
        return afe_setup_non_por();
    }
}

void afe_reset(void)
{
    //
    // Before resetting AFE, insure SPI pins are in known good state
    //
    afe_spi_idle_interface();

    //
    // Software controlled reset of the AFE is controlled via GCR
    //  Note, after calling this function afe_load_trims should be called to restore AFE functionality
    //
    MXC_GCR->rst1 |= MXC_F_GCR_RST1_AFE;

    //
    // Disable HART clock to avoid any unexpected behaviors by the HART block when reset is released
    //
    hart_clock_disable();
}

static int raw_afe_write_register(uint8_t reg_address, uint32_t value, uint8_t reg_length)
{
    int i = 0;
    int retval = 0;

    int txLen = AFE_SPI_ADDRESS_LEN + reg_length;

    // First comes address
    // AFE requires bit 7 of byte 0 CLEAR to indicate write command
    afe_data[0] = reg_address & ~AFE_REG_ADDR_READ_BIT;

    // Next data to write MSB first
    for (i = reg_length; i > 0; i--) {
        afe_data[i] = value & 0x0FF;
        value >>= 8;
    }

    // Blocking SPI Transceive
    retval = afe_spi_transceive(afe_data, txLen);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return retval;
}

static int raw_afe_read_register(uint8_t reg_address, uint32_t *value, uint8_t reg_length)
{
    int i = 0;
    int retval = 0;
    int txLen = AFE_SPI_ADDRESS_LEN + reg_length;

    // First comes address
    // AFE requires bit 7 of byte 0 SET to indicate read command
    afe_data[0] = reg_address | AFE_REG_ADDR_READ_BIT;

    // Blocking SPI Transceive
    retval = afe_spi_transceive(afe_data, txLen);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    *value = 0;

    // Extract value
    for (i = 0; i < reg_length; i++) {
        *value <<= 8;
        *value |= afe_data[i + AFE_SPI_ADDRESS_LEN] & 0x0FF;
    }

    // TODO(ADI): If optional CRC5 is added in the future, verify CRC here

    return retval;
}

static int afe_bank_select(uint8_t bank_num)
{
    uint32_t read_val = 0;
    int retval = 0;

    // First, No need to check for current bank if we are already in it.
    if (current_register_bank == bank_num) {
        return E_NO_ERROR;
    }

    // Read current value, and or in our bits
    // NOTE: SYC_CTRL exists in all register banks
    //
    // BIG NOTE: READING of this register arms the SPI_ABORT_DIS feature
    //
    retval = raw_afe_read_register(
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, &read_val,
        (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // BIG NOTE: writing a register on the AFE during a ADC conversion will ABORT it.
    // If we are already in the correct bank, then do NOT write the SYS_CTRL register
    //

    //
    // NOTE: As a workaround to allow for multiple ADC conversions, one in ADC0 and simultaneous one in ADC1
    //  The SPI_ABORT_DIS feature in the sys_ctrl register can help.  Note that there is only 1 sys_cntl register
    //  but it is available from every register bank, ADC0, ADC1, DAC, and HART.
    //
    // Reading the sys_ctrl register arms the HW to set this bit automatically, it doesn't need to be set by SW.
    //  WARNING: It blocks the abort signal. This means, if a conversion is active on the selected ADC, and
    //  after reading sys_ctrl, a write occurs to ADC bank with conversion in progress, the ADC data will
    //  be corrupted.
    //

    if ((read_val & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL) !=
        (bank_num & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL)) {
        // Need to change the bank
        read_val = (read_val & ~MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL) |
                   (bank_num & MXC_F_AFE_ADC_ZERO_SYS_CTRL_ANA_SRC_SEL);

        retval = raw_afe_write_register(
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR) >> AFE_REG_ADDR_POS, read_val,
            (MXC_R_AFE_ADC_ZERO_SYS_CTRL & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS);

        if (retval != E_NO_ERROR) {
            return retval;
        }

        // Update bank tracker
        current_register_bank = bank_num;
    }
    // Else already in the correct bank

    return retval;
}

int afe_write_register(uint32_t target_reg, uint32_t value)
{
    uint8_t reg_bank = 0;
    uint8_t reg_address = 0;
    uint8_t reg_length = 0;
    int retval = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into
    // ana_src_sel[1:0] as bank select. Bottom byte of address encodes the
    // register width in bytes 1 - 4 (8bits to 32bits)

    reg_length = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_bank = (target_reg & AFE_REG_ADDR_BANK) >> AFE_REG_ADDR_BANK_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return raw_afe_write_register(reg_address, value, reg_length);
}

int afe_bank_write_register(uint32_t target_reg, uint8_t reg_bank, uint32_t value)
{
    uint8_t reg_address = 0;
    uint8_t reg_length = 0;
    int retval = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into
    // ana_src_sel[1:0] as bank select. Bottom byte of address encodes the
    // register width in bytes 1 - 4 (8bits to 32bits)

    reg_length = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return raw_afe_write_register(reg_address, value, reg_length);
}

int afe_read_register(uint32_t target_reg, uint32_t *value)
{
    uint8_t reg_bank = 0;
    uint8_t reg_address = 0;
    uint8_t reg_length = 0;
    int retval = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into
    // ana_src_sel[1:0] as bank select. Bottom byte of address encodes the
    // register width in bytes 1 - 4 (8bits to 32bits)

    reg_length = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_bank = (target_reg & AFE_REG_ADDR_BANK) >> AFE_REG_ADDR_BANK_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return raw_afe_read_register(reg_address, value, reg_length);
}

int afe_bank_read_register(uint32_t target_reg, uint8_t reg_bank, uint32_t *value)
{
    uint8_t reg_address = 0;
    uint8_t reg_length = 0;
    int retval = 0;

    //
    // Parse register parameters from register offset
    //

    // Top 2 bytes of address encodes the register address
    // Address Bits 23&24 are MSB address bits, which must be written into
    // ana_src_sel[1:0] as bank select. Bottom byte of address encodes the
    // register width in bytes 1 - 4 (8bits to 32bits)

    reg_length = (target_reg & AFE_REG_ADDR_LEN) >> AFE_REG_ADDR_LEN_POS;
    reg_address = (target_reg & AFE_REG_ADDR) >> AFE_REG_ADDR_POS;

    retval = afe_bank_select(reg_bank);

    if (retval != E_NO_ERROR) {
        return retval;
    }

    return raw_afe_read_register(reg_address, value, reg_length);
}

int afe_load_trims(mxc_tmr_regs_t *tmr)
{
    int retval = 0;
    uint8_t info_buf[INFOBLOCK_LINE_SIZE];
    uint64_t afe_trim_low = 0;
    uint64_t afe_trim_high = 0;
#ifdef DUMP_TRIM_DATA
    uint32_t read_val = 0;
#endif

    // setup the interface before we begin
    retval = afe_setup(tmr);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // Before Trimming, reset AFE to known state
    //

    //
    // SYS_CTRL will be in a known state after a successful call to afe_setup()
    //

    // Reset ADCs, as the MAX32675 Reset has no effect on them
    // After restoring POR defaults, ADCS enter Standby mode
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_PD, MXC_S_AFE_ADC_ZERO_PD_PD_RESET);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_PD, MXC_S_AFE_ADC_ONE_PD_PD_RESET);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    //
    // Read in AFE Trims
    //
    retval = infoblock_read(AFE_TRIM_OTP_OFFSET_LOW, info_buf, INFOBLOCK_LINE_SIZE);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    for (int i = 7; i >= 0; i--) {
        afe_trim_low <<= 8;
        afe_trim_low |= info_buf[i];
    }

    retval = infoblock_read(AFE_TRIM_OTP_OFFSET_HIGH, info_buf, INFOBLOCK_LINE_SIZE);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    for (int i = 7; i >= 0; i--) {
        afe_trim_high <<= 8;
        afe_trim_high |= info_buf[i];
    }

    //
    // Parse out individual trims
    //
    // afe_trim_low
    trim_data.adc_trim0_adc0 = afe_trim_low & AFE_TRIM0_ADC0_MASK;
    afe_trim_low >>= AFE_TRIM0_ADC0_BIT_WIDTH;

    trim_data.adc_trim1_adc0 = afe_trim_low & AFE_TRIM1_ADC0_MASK;
    afe_trim_low >>= AFE_TRIM1_ADC0_BIT_WIDTH;

    trim_data.dac_trim = afe_trim_low & AFE_TRIM_DAC_MASK;
    afe_trim_low >>= AFE_TRIM_DAC_BIT_WIDTH;

    trim_data.ana_trim_adc0 = afe_trim_low & AFE_TRIM_ANA_ADC0_MASK;

    // afe_trim_high
    trim_data.adc_trim0_adc1 = afe_trim_high & AFE_TRIM0_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM0_ADC1_BIT_WIDTH;

    trim_data.adc_trim1_adc1 = afe_trim_high & AFE_TRIM1_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM1_ADC1_BIT_WIDTH;

    // Now got to take care of the ANA_TRIM_ADC1 which is embedded inside the HART trim
    trim_data.ana_trim_adc1 = afe_trim_high & AFE_TRIM_ANA_ADC1_MASK;
    trim_data.ana_trim_adc1 >>= AFE_TRIM_ANA_ADC1_OFFSET_1;
    trim_data.ana_trim_adc1 <<= AFE_TRIM_ANA_ADC1_OFFSET_2;

    trim_data.hart_trim = afe_trim_high & AFE_TRIM_HART_MASK;
    // Force the embedded ANA trim to zeros.
    trim_data.hart_trim &= ~AFE_TRIM_ANA_ADC1_MASK;
    afe_trim_high >>= AFE_TRIM_HART_BIT_WIDTH;

    trim_data.vref_trim = afe_trim_high & AFE_TRIM_VREF_MASK;

    //
    // Dump Trims
    //
#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  Trim Values Read from info block\n");
    printf("\n****************************************************\n");

    printf("ADC0 adc trim 0: %08X\n", trim_data.adc_trim0_adc0);
    printf("ADC0 adc trim 1: %08X\n", trim_data.adc_trim1_adc0);
    printf("DAC trim: %08X\n", trim_data.dac_trim);
    printf("ANA ADC0 trim: %08X\n", trim_data.ana_trim_adc0);

    printf("ADC1 adc trim 0: %08X\n", trim_data.adc_trim0_adc1);
    printf("ADC1 adc trim 1: %08X\n", trim_data.adc_trim1_adc1);
    printf("HART trim: %08X\n", trim_data.hart_trim);
    printf("ANA ADC1 trim: %08X\n", trim_data.ana_trim_adc1);
    printf("VREF trim: %08X\n", trim_data.vref_trim);
#endif

    //
    // Write Trims
    //

    // Unlock trim for ADC0
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD,
                                MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD,
                                MXC_V_AFE_ADC_ZERO_FT_PWORD_FT_PWORD_PWORD_2);
    if (retval != E_NO_ERROR) {
        return retval;
    }

    // Unlock trim for ADC1
    retval =
        afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, MXC_V_AFE_ADC_ONE_FT_PWORD_FT_PWORD_PWORD_1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval =
        afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, MXC_V_AFE_ADC_ONE_FT_PWORD_FT_PWORD_PWORD_2);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  AFE Trim Register before Writing (FT UNLOCKED)\n");
    printf("\n****************************************************\n");

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, &read_val);
    printf("ADC0 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, &read_val);
    printf("ADC0 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_TRIM, &read_val);
    printf("DAC trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, &read_val);
    printf("ANA ADC0 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, &read_val);
    printf("ADC1 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, &read_val);
    printf("ADC1 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_HART_TRIM, &read_val);
    printf("HART trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, &read_val);
    printf("ANA ADC1 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_TRIM, &read_val);
    printf("VREF trim: %08X\n", read_val);

    printf("\n****************************************************\n");
    printf("  Writing Trim Values Into AFE\n");
    printf("\n****************************************************\n");
#endif

    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, trim_data.adc_trim0_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, trim_data.adc_trim1_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_DAC_TRIM, trim_data.dac_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, trim_data.ana_trim_adc0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, trim_data.adc_trim0_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, trim_data.adc_trim1_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_HART_TRIM, trim_data.hart_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, trim_data.ana_trim_adc1);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_DAC_VREF_TRIM, trim_data.vref_trim);
    if (retval != E_NO_ERROR) {
        return retval;
    }

#ifdef DUMP_TRIM_DATA
    printf("\n****************************************************\n");
    printf("  AFE Trim Register after Writing (FT UNLOCKED)\n");
    printf("\n****************************************************\n");

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM0, &read_val);
    printf("ADC0 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ADC_TRIM1, &read_val);
    printf("ADC0 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_TRIM, &read_val);
    printf("DAC trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ZERO_ANA_TRIM, &read_val);
    printf("ANA ADC0 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM0, &read_val);
    printf("ADC1 adc trim 0: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ADC_TRIM1, &read_val);
    printf("ADC1 adc trim 1: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_HART_TRIM, &read_val);
    printf("HART trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_ADC_ONE_ANA_TRIM, &read_val);
    printf("ANA ADC1 trim: %08X\n", read_val);

    afe_read_register(MXC_R_AFE_DAC_VREF_TRIM, &read_val);
    printf("VREF trim: %08X\n", read_val);
#endif

    // Lock Trims
    retval = afe_write_register(MXC_R_AFE_ADC_ONE_FT_PWORD, 0);
    if (retval != E_NO_ERROR) {
        return retval;
    }
    retval = afe_write_register(MXC_R_AFE_ADC_ZERO_FT_PWORD, 0);

    return retval;
}

void afe_dump_registers(uint32_t reg_bank)
{
    uint32_t reg_add = 0;
    uint32_t reg_len = 0;
    uint32_t read_val = 0;

    if (reg_bank == AFE_ADC0_BANK) {
        printf("Dumping registers of AFE ADC0 Bank\n");
    } else if (reg_bank == AFE_ADC1_BANK) {
        printf("Dumping registers of AFE ADC1 Bank\n");
    } else if (reg_bank == AFE_DAC_BANK) {
        printf("Dumping registers of AFE DAC Bank\n");
    } else if (reg_bank == AFE_HART_BANK) {
        printf("Dumping registers of AFE HART Bank\n");
    } else {
        printf("Unknown AFE bank number.\n");
        return;
    }

    if ((reg_bank == AFE_ADC0_BANK) || (reg_bank == AFE_ADC1_BANK)) {
        reg_len = 1;

        // 8 Bit Registers
        printf("\n** 8-bit Control Registers 0x00 - 0x10 **\n");
        for (reg_add = 0; reg_add < 0x11; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%02X\n", reg_add, read_val);
        }

        // 24 bit Registers
        printf("\n** 24-bit Control, Data, Status Registers 0x11 - 0x39 **\n");
        reg_len = 3;
        for (reg_add = 0x11; reg_add < 0x3A; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }

        // 16 bit Registers
        printf("\n** 16-bit Sequencer Registers 0x3A - 0x6E **\n");
        reg_len = 2;
        for (reg_add = 0x3A; reg_add < 0x6F; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%04X\n", reg_add, read_val);
        }

        // UCADDR is only 8 bits
        printf("\n** 8-bit UCADDR Register **\n");
        reg_len = 1;
        reg_add = 0x6F;
        afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                              (reg_len << AFE_REG_ADDR_LEN_POS),
                          &read_val);
        printf("Reg 0x%02X is 0x%02X\n", 0x6F, read_val);
    } // End of ADC Register Dump

    if (reg_bank == AFE_DAC_BANK) {
        reg_len = 4; // 32 Bit registers
        printf("\n** 32-bit DAC Registers 0x00 - 0x04 **\n");
        for (reg_add = 0x00; reg_add < 0x05; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%08X\n", reg_add, read_val);
        }

        reg_len = 3; // 24 Bit Registers
        printf("\n** 24-bit DAC Registers 0x05 - 0x07 **\n");
        for (reg_add = 0x05; reg_add < 0x08; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }
    } // End of ADC DAC Register Dump

    if (reg_bank == AFE_HART_BANK) {
        reg_len = 3; // 24 Bit registers
        printf("\n** 24-bit HART Registers 0x00 - 0x0B **\n");
        for (reg_add = 0x00; reg_add < 0x0C; reg_add++) {
            afe_read_register((reg_add << AFE_REG_ADDR_POS) | (reg_bank << AFE_REG_ADDR_BANK_POS) |
                                  (reg_len << AFE_REG_ADDR_LEN_POS),
                              &read_val);
            printf("Reg 0x%02X is 0x%06X\n", reg_add, read_val);
        }
    } // End of ADC HART Register Dump

    printf("\n\n");
}
