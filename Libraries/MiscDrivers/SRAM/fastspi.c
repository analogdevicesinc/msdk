/******************************************************************************
 *
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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
#include <stdbool.h>
#include <stdio.h>
#include "fastspi.h"
#include "gpio.h"
#include "mxc_sys.h"
#include "spi.h"
#include "dma.h"
#include "nvic_table.h"
#include "mxc_delay.h"

int g_tx_channel;
int g_rx_channel;
int g_fill_dummy_bytes = 0;
int g_dummy_len = 0;
uint8_t g_dummy_byte = 0xFF;

bool g_dma_initialized = false;

uint8_t *g_rx_buffer;
uint8_t *g_tx_buffer;
uint32_t g_rx_len;
uint32_t g_tx_len;

static volatile bool g_tx_done = 0;
static volatile bool g_rx_done = 0;
static volatile bool g_master_done = 0;

void DMA_TX_IRQHandler()
{
    volatile mxc_dma_ch_regs_t *ch =
        &MXC_DMA->ch[g_tx_channel]; // Cast the pointer for readability in this ISR
    uint32_t status = ch->status;

    if (status & MXC_F_DMA_STATUS_CTZ_IF) { // Count-to-Zero (DMA TX complete)
        g_tx_done = 1;
        ch->status |= MXC_F_DMA_STATUS_CTZ_IF;
    }

    if (status & MXC_F_DMA_STATUS_BUS_ERR) { // Bus Error
        ch->status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}

void DMA_RX_IRQHandler()
{
    volatile mxc_dma_ch_regs_t *ch =
        &MXC_DMA->ch[g_rx_channel]; // Cast the pointer for readability in this ISR
    uint32_t status = ch->status;

    if (status & MXC_F_DMA_STATUS_CTZ_IF) { // Count-to-Zero (DMA RX complete)
        g_rx_done = 1;
        ch->status |= MXC_F_DMA_STATUS_CTZ_IF;
    }

    if (status & MXC_F_DMA_STATUS_BUS_ERR) { // Bus Error
        ch->status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}

void processSPI()
{
    // Unload any SPI data that has come in
    while (g_rx_buffer && (FASTSPI_INSTANCE->dma & MXC_F_SPI_DMA_RX_LVL) && g_rx_len > 0) {
        *g_rx_buffer++ = FASTSPI_INSTANCE->fifo8[0];
        g_rx_len--;
    }

    if (g_rx_len <= 0) {
        g_rx_done = 1;
    }

    // Write any pending bytes out.
    while (g_tx_buffer &&
           (((FASTSPI_INSTANCE->dma & MXC_F_SPI_DMA_TX_LVL) >> MXC_F_SPI_DMA_TX_LVL_POS) <
            MXC_SPI_FIFO_DEPTH) &&
           g_tx_len > 0) {
        FASTSPI_INSTANCE->fifo8[0] = *g_tx_buffer++;
        g_tx_len--;
    }

    if (g_tx_len <= 0) {
        g_tx_done = 1;
    }
}

void FastSPI_IRQHandler()
{
    uint32_t status = FASTSPI_INSTANCE->intfl;

    if (status & MXC_F_SPI_INTFL_MST_DONE) { // Master done (TX complete)
        g_master_done = 1;
        FASTSPI_INSTANCE->intfl |= MXC_F_SPI_INTFL_MST_DONE; // Clear flag
    }

    if (status & MXC_F_SPI_INTFL_RX_THD) {
        FASTSPI_INSTANCE->intfl |= MXC_F_SPI_INTFL_RX_THD;
    }

    if (status & MXC_F_SPI_INTFL_TX_THD) {
        FASTSPI_INSTANCE->intfl |= MXC_F_SPI_INTFL_TX_THD;
    }
}

int _dma_init()
{
    if (g_dma_initialized)
        return E_NO_ERROR;

    int err = MXC_DMA_Init();
    // If we get a bad state error here it means DMA has already been
    // initialized.  The drivers do a good job of handling this case,
    // so it's safe to continue
    if (err && (err != E_BAD_STATE))
        return err;

    g_tx_channel = MXC_DMA_AcquireChannel();
    g_rx_channel = MXC_DMA_AcquireChannel();
    if (g_tx_channel < 0 || g_rx_channel < 0) {
        return E_NONE_AVAIL; // Failed to acquire DMA channels
    }

    // TX Channel
    MXC_DMA->ch[g_tx_channel].ctrl &= ~(MXC_F_DMA_CTRL_EN);
    MXC_DMA->ch[g_tx_channel].ctrl =
        MXC_F_DMA_CTRL_SRCINC |
        (0x2F
         << MXC_F_DMA_CTRL_REQUEST_POS); // Enable incrementing the src address pointer, set destination to SPI0 TX FIFO (REQSEL = 0x2F)
    MXC_DMA->ch[g_tx_channel].ctrl |=
        (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE); // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_tx_channel); // Enable DMA interrupts

    // RX Channel
    MXC_DMA->ch[g_rx_channel].ctrl &= ~(MXC_F_DMA_CTRL_EN);
    MXC_DMA->ch[g_rx_channel].ctrl =
        MXC_F_DMA_CTRL_DSTINC |
        (0x0F
         << MXC_F_DMA_CTRL_REQUEST_POS); // Enable incrementing the dest address pointer, set to source to SPI0 RX FIFO (REQSEL = 0x0F)
    MXC_DMA->ch[g_rx_channel].ctrl |=
        (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE); // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_rx_channel); // Enable DMA interrupts

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(g_tx_channel), DMA_TX_IRQHandler);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(g_tx_channel));
    NVIC_SetPriority(MXC_DMA_CH_GET_IRQ(g_tx_channel), 0);

    MXC_NVIC_SetVector(MXC_DMA_CH_GET_IRQ(g_rx_channel), DMA_RX_IRQHandler);
    NVIC_EnableIRQ(MXC_DMA_CH_GET_IRQ(g_rx_channel));
    NVIC_SetPriority(MXC_DMA_CH_GET_IRQ(g_tx_channel), 0);

    g_dma_initialized = true;

    return E_NO_ERROR;
}

int spi_init()
{
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);

    int err = MXC_GPIO_Config(&fastspi_spi_pins);
    if (err)
        return err;

    err = MXC_GPIO_Config(&fastspi_ss_pin);
    if (err)
        return err;

    // Set strongest possible drive strength for SPI pins
    fastspi_spi_pins.port->ds0 |= fastspi_spi_pins.mask;
    fastspi_spi_pins.port->ds1 |= fastspi_spi_pins.mask;

    FASTSPI_INSTANCE->ctrl0 =
        (0b0100
         << MXC_F_SPI_CTRL0_SS_ACTIVE_POS) | // Set SSEL = SS2 <-- TODO(Jake): Improve this when other drivers are added
        MXC_F_SPI_CTRL0_MST_MODE | // Select controller mode
        MXC_F_SPI_CTRL0_EN; // Enable SPI

    FASTSPI_INSTANCE->ctrl2 = (8 << MXC_F_SPI_CTRL2_NUMBITS_POS); // Set 8 bits per character

    FASTSPI_INSTANCE->sstime =
        (1 << MXC_F_SPI_SSTIME_PRE_POS) | // Remove any delay time between SSEL and SCLK edges
        (128 << MXC_F_SPI_SSTIME_POST_POS) | (1 << MXC_F_SPI_SSTIME_INACT_POS);

    FASTSPI_INSTANCE->dma = MXC_F_SPI_DMA_TX_FIFO_EN | // Enable TX FIFO
                            (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS) | // Set TX threshold to 31
                            MXC_F_SPI_DMA_DMA_TX_EN; // Enable DMA for the TX FIFO

    FASTSPI_INSTANCE->inten |=
        MXC_F_SPI_INTFL_MST_DONE; // Enable the "Transaction complete" interrupt

    FASTSPI_INSTANCE->intfl =
        FASTSPI_INSTANCE->intfl; // Clear any any interrupt flags that may already be set

    err = MXC_SPI_SetFrequency(FASTSPI_INSTANCE, FASTSPI_SPEED);
    if (err)
        return err;

    NVIC_EnableIRQ(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(FASTSPI_INSTANCE)));
    MXC_NVIC_SetVector(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(FASTSPI_INSTANCE)), FastSPI_IRQHandler);
    NVIC_SetPriority(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(FASTSPI_INSTANCE)), 0);

    err = _dma_init();

    return err;
}

int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert)
{
    g_tx_done = 0;
    g_rx_done = 0;
    g_master_done = 0;
    mxc_spi_width_t width = MXC_SPI_GetWidth(FASTSPI_INSTANCE);

    // Set the number of bytes to transmit/receive for the SPI transaction
    if (width == SPI_WIDTH_STANDARD) {
        if (rxlen > txlen) {
            /*
            In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            because the hardware always assume full duplex.  Therefore extra
            dummy bytes must be transmitted to support half duplex. 
            */
            g_dummy_len = rxlen - txlen;
            FASTSPI_INSTANCE->ctrl1 = ((txlen + g_dummy_len) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
        } else {
            FASTSPI_INSTANCE->ctrl1 = txlen << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;
        }
    } else { // width != SPI_WIDTH_STANDARD
        FASTSPI_INSTANCE->ctrl1 = (txlen << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS) |
                                  (rxlen << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS);
    }

    FASTSPI_INSTANCE->dma &=
        ~(MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN | MXC_F_SPI_DMA_RX_FIFO_EN |
          MXC_F_SPI_DMA_DMA_RX_EN); // Disable FIFOs before clearing as recommended by UG
    FASTSPI_INSTANCE->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH); // Clear the FIFOs

    // TX
    if (txlen > 1) {
        // Configure TX DMA channel to fill the SPI TX FIFO
        FASTSPI_INSTANCE->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN |
                                  (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS));
        FASTSPI_INSTANCE->fifo8[0] = src[0];
        // ^ Hardware requires writing the first byte into the FIFO manually.
        MXC_DMA->ch[g_tx_channel].src = (uint32_t)(src + 1);
        MXC_DMA->ch[g_tx_channel].cnt = txlen - 1;
        MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_SRCINC;
        MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN; // Start the DMA
    } else if (txlen == 1) {
        // Workaround for single-length transactions not triggering CTZ
        FASTSPI_INSTANCE->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
        FASTSPI_INSTANCE->fifo8[0] = src[0]; // Write first byte into FIFO
        g_tx_done = 1;
    } else if (txlen == 0 && width == SPI_WIDTH_STANDARD) {
        // Configure TX DMA channel to retransmit a dummy byte
        FASTSPI_INSTANCE->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
        MXC_DMA->ch[g_tx_channel].src = (uint32_t)&g_dummy_byte;
        MXC_DMA->ch[g_tx_channel].cnt = rxlen;
        MXC_DMA->ch[g_tx_channel].ctrl &= ~MXC_F_DMA_CTRL_SRCINC;
        MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN; // Start the DMA
    }

    // RX
    if (rxlen > 0) {
        // Configure RX DMA channel to unload the SPI RX FIFO
        FASTSPI_INSTANCE->dma |= (MXC_F_SPI_DMA_RX_FIFO_EN | MXC_F_SPI_DMA_DMA_RX_EN);
        MXC_DMA->ch[g_rx_channel].dst = (uint32_t)dest;
        MXC_DMA->ch[g_rx_channel].cnt = rxlen;
        MXC_DMA->ch[g_rx_channel].ctrl |= MXC_F_DMA_CTRL_EN; // Start the DMA
    }

    // Start the SPI transaction
    FASTSPI_INSTANCE->ctrl0 |= MXC_F_SPI_CTRL0_START;

    /*
    Handle slave-select (SS) deassertion.  This must be done AFTER launching the transaction
    to avoid a glitch on the SS line if:
    - The SS line is asserted
    - We want to deassert the line as part of this transaction

    As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the SS pin based
    on the value of CTRL->SS_CTRL, which causes the glitch.
    */
    if (deassert)
        FASTSPI_INSTANCE->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    else
        FASTSPI_INSTANCE->ctrl0 |= MXC_F_SPI_CTRL0_SS_CTRL;

    // Wait for the transaction to complete.
    while (!((g_tx_done && g_master_done) && (src != NULL && txlen > 0)) &&
           !(g_rx_done && (dest != NULL && rxlen > 0))) {
        /*
        The following polling is a safety fallback to catch any missed interrupts.
        This is especially common with extremely short transactions, where all 3 
        interrupts may fire almost simultaneously.
        */
        if ((src != NULL && txlen > 0) && FASTSPI_INSTANCE->intfl & MXC_F_SPI_INTFL_MST_DONE)
            g_master_done = 1;
        if ((src != NULL && txlen > 0) &&
            MXC_DMA->ch[g_tx_channel].status & MXC_F_DMA_STATUS_CTZ_IF)
            g_tx_done = 1;
        if ((dest != NULL && rxlen > 0) &&
            MXC_DMA->ch[g_rx_channel].status & MXC_F_DMA_STATUS_CTZ_IF)
            g_rx_done = 1;
    }

    return E_SUCCESS;
}

int spi_exit_quadmode()
{
    return MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_STANDARD);
}

int spi_enter_quadmode()
{
    return MXC_SPI_SetWidth(FASTSPI_INSTANCE, SPI_WIDTH_QUAD);
}
