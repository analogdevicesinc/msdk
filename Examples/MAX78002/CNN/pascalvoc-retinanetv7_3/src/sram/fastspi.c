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
bool g_use_dma = false;

bool g_dma_initialized = false;

uint8_t *g_rx_buffer;
uint8_t *g_tx_buffer;
uint32_t g_rx_len;
uint32_t g_tx_len;

// A macro to convert a DMA channel number to an IRQn number
#define GetIRQnForDMAChannel(x) ((IRQn_Type)(((x) == 0 ) ? DMA0_IRQn  : \
                                             ((x) == 1 ) ? DMA1_IRQn  : \
                                             ((x) == 2 ) ? DMA2_IRQn  : \
                                                           DMA3_IRQn))

void DMA_TX_IRQHandler()
{
    volatile mxc_dma_ch_regs_t *ch = &MXC_DMA->ch[g_tx_channel];  // Cast the pointer for readability in this ISR
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
    volatile mxc_dma_ch_regs_t *ch = &MXC_DMA->ch[g_rx_channel];  // Cast the pointer for readability in this ISR
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
    while(g_rx_buffer && (SPI->dma & MXC_F_SPI_DMA_RX_LVL) && g_rx_len > 0) {
        *g_rx_buffer++ = SPI->fifo8[0];
        g_rx_len--;
    }

    if (g_rx_len <= 0) {
        g_rx_done = 1;
    }

    // Write any pending bytes out.
    while(g_tx_buffer && (((SPI->dma & MXC_F_SPI_DMA_TX_LVL) >> MXC_F_SPI_DMA_TX_LVL_POS) < MXC_SPI_FIFO_DEPTH) && g_tx_len > 0) {
        SPI->fifo8[0] = *g_tx_buffer++;
        g_tx_len--;
    }

    if (g_tx_len <= 0) {
        g_tx_done = 1;
    }
}

void SPI_IRQHandler()
{
    uint32_t status = SPI->intfl;

    if (status & MXC_F_SPI_INTFL_MST_DONE) { // Master done (TX complete)
        g_master_done = 1;
        SPI->intfl |= MXC_F_SPI_INTFL_MST_DONE;  // Clear flag
    }

    if (status & MXC_F_SPI_INTFL_RX_THD) {
        SPI->intfl |= MXC_F_SPI_INTFL_RX_THD;
        if (!g_use_dma) {
            // RX threshold has been crossed, there's data to unload from the FIFO
            processSPI();
        }
    }

    if (status & MXC_F_SPI_INTFL_TX_THD) {
        SPI->intfl |= MXC_F_SPI_INTFL_TX_THD;
        if (!g_use_dma) {
            // TX threshold has been crossed, we need to refill the FIFO
            processSPI();
        }
    }

    
}

int dma_init()
{
    if (g_dma_initialized) return E_NO_ERROR;

    int err = MXC_DMA_Init();
    // If we get a bad state error here it means DMA has already been
    // initialized.  The drivers do a good job of handling this case,
    // so it's safe to continue
    if (err && (err != E_BAD_STATE))
        return err;
    
    g_tx_channel = MXC_DMA_AcquireChannel();
    g_rx_channel = MXC_DMA_AcquireChannel();
    if (g_tx_channel < 0 || g_rx_channel < 0) {
        return E_NONE_AVAIL;  // Failed to acquire DMA channels
    }

    // TX Channel
    MXC_DMA->ch[g_tx_channel].ctrl &= ~(MXC_F_DMA_CTRL_EN);
    MXC_DMA->ch[g_tx_channel].ctrl = MXC_F_DMA_CTRL_SRCINC | (0x2F << MXC_F_DMA_CTRL_REQUEST_POS);  // Enable incrementing the src address pointer, set destination to SPI0 TX FIFO (REQSEL = 0x2F)
    MXC_DMA->ch[g_tx_channel].ctrl |= (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE);              // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_tx_channel);                                                          // Enable DMA interrupts

    // RX Channel
    MXC_DMA->ch[g_rx_channel].ctrl &= ~(MXC_F_DMA_CTRL_EN);
    MXC_DMA->ch[g_rx_channel].ctrl = MXC_F_DMA_CTRL_DSTINC | (0x0F << MXC_F_DMA_CTRL_REQUEST_POS);  // Enable incrementing the dest address pointer, set to source to SPI0 RX FIFO (REQSEL = 0x0F)
    MXC_DMA->ch[g_rx_channel].ctrl |= (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE);              // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_rx_channel);                                                          // Enable DMA interrupts

    MXC_NVIC_SetVector(GetIRQnForDMAChannel(g_tx_channel), DMA_TX_IRQHandler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(g_tx_channel));
    NVIC_SetPriority(GetIRQnForDMAChannel(g_tx_channel), 0);

    MXC_NVIC_SetVector(GetIRQnForDMAChannel(g_rx_channel), DMA_RX_IRQHandler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(g_rx_channel));
    NVIC_SetPriority(GetIRQnForDMAChannel(g_tx_channel), 0);

    g_dma_initialized = true;

    return E_NO_ERROR;
}

int spi_init()
{
    // TODO: Add software-controlled slave select functionality
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);

    int err = MXC_GPIO_Config(&spi_pins);
    if (err)
        return err;

    err = MXC_GPIO_Config(&spi_ss_pin);
    if (err)
        return err;

    // Set strongest possible drive strength for SPI pins
    spi_pins.port->ds0 |= spi_pins.mask;
    spi_pins.port->ds1 |= spi_pins.mask;
    
    // TODO: Expose some of the config options below
    // TODO: Move QSPI-SRAM specific options into aps6404.c

    SPI->ctrl0 = (1 << MXC_F_SPI_CTRL0_SS_ACTIVE_POS) |    // Set SSEL = SS0
                        MXC_F_SPI_CTRL0_MST_MODE |         // Select controller mode
                        MXC_F_SPI_CTRL0_EN;                // Enable SPI

    SPI->ctrl2 = (8 << MXC_F_SPI_CTRL2_NUMBITS_POS);       // Set 8 bits per character
    
    SPI->sstime = (1 << MXC_F_SPI_SSTIME_PRE_POS) |        // Remove any delay time between SSEL and SCLK edges
                        (128 << MXC_F_SPI_SSTIME_POST_POS) |
                        (1 << MXC_F_SPI_SSTIME_INACT_POS);

    SPI->dma = MXC_F_SPI_DMA_TX_FIFO_EN |                  // Enable TX FIFO
                    (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS) | // Set TX threshold to 31
                    MXC_F_SPI_DMA_DMA_TX_EN;               // Enable DMA for the TX FIFO

    SPI->inten |= MXC_F_SPI_INTFL_MST_DONE;                 // Enable the "Transaction complete" interrupt

    SPI->intfl = SPI->intfl;                                // Clear any any interrupt flags that may already be set

    err = MXC_SPI_SetFrequency(SPI, SPI_SPEED);
    if (err)
        return err;

    NVIC_EnableIRQ(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(SPI)));
    MXC_NVIC_SetVector(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(SPI)), SPI_IRQHandler);
    NVIC_SetPriority(MXC_SPI_GET_IRQ(MXC_SPI_GET_IDX(SPI)), 0);

    err = dma_init();

    return err;
}

int spi_transmit(uint8_t *src, uint32_t txlen, uint8_t *dest, uint32_t rxlen, bool deassert, bool use_dma, bool block)
{
    g_tx_done = 0;
    g_rx_done = 0;
    g_master_done = 0;
    mxc_spi_width_t width = MXC_SPI_GetWidth(SPI); 

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
            SPI->ctrl1 = ((txlen + g_dummy_len) << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
        } else {
            SPI->ctrl1 = txlen << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;
        }
    } else { // width != SPI_WIDTH_STANDARD
        SPI->ctrl1 = (txlen << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS) |
                     (rxlen << MXC_F_SPI_CTRL1_RX_NUM_CHAR_POS);
    }

    SPI->dma &= ~(MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN | MXC_F_SPI_DMA_RX_FIFO_EN | MXC_F_SPI_DMA_DMA_RX_EN);  // Disable FIFOs before clearing as recommended by UG
    SPI->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);  // Clear the FIFOs

    if (use_dma) {
        g_use_dma = true;
        // TX
        if (txlen > 1) {
            // Configure TX DMA channel to fill the SPI TX FIFO
            SPI->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN | (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS));
            SPI->fifo8[0] = src[0];
            // ^ Hardware requires writing the first byte into the FIFO manually.
            MXC_DMA->ch[g_tx_channel].src = (uint32_t)(src + 1);
            MXC_DMA->ch[g_tx_channel].cnt = txlen - 1;
            MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_SRCINC;
            MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN;  // Start the DMA
        } else if (txlen == 1) {
            // Workaround for single-length transactions not triggering CTZ
            SPI->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
            SPI->fifo8[0] = src[0]; // Write first byte into FIFO
            g_tx_done = 1;
        } else if (txlen == 0 && width == SPI_WIDTH_STANDARD) {
            // Configure TX DMA channel to retransmit a dummy byte
            SPI->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN | MXC_F_SPI_DMA_DMA_TX_EN);
            MXC_DMA->ch[g_tx_channel].src = (uint32_t)&g_dummy_byte;
            MXC_DMA->ch[g_tx_channel].cnt = rxlen;
            MXC_DMA->ch[g_tx_channel].ctrl &= ~MXC_F_DMA_CTRL_SRCINC;
            MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN;  // Start the DMA
        }

        // RX
        if (rxlen > 0) {
            // Configure RX DMA channel to unload the SPI RX FIFO
            SPI->dma |= (MXC_F_SPI_DMA_RX_FIFO_EN | MXC_F_SPI_DMA_DMA_RX_EN);
            MXC_DMA->ch[g_rx_channel].dst = (uint32_t)dest;
            MXC_DMA->ch[g_rx_channel].cnt = rxlen;
            MXC_DMA->ch[g_rx_channel].ctrl |= MXC_F_DMA_CTRL_EN;  // Start the DMA
        }

    } else { // !use_dma
        g_use_dma = false;
        g_rx_buffer = dest;
        g_tx_buffer = src;
        g_rx_len = rxlen;
        g_tx_len = txlen;

        SPI->inten |= MXC_F_SPI_INTEN_MST_DONE;

        if (txlen > 0) {
            // Enable TX FIFO & TX Threshold crossed interrupt 
            SPI->dma |= (MXC_F_SPI_DMA_TX_FIFO_EN);
            SPI->inten |= MXC_F_SPI_INTEN_TX_THD;
        }

        if (rxlen > 0) {
            // Enable RX FIFO & RX Threshold crossed interrupt
            SPI->dma |= (MXC_F_SPI_DMA_RX_FIFO_EN);
            SPI->inten |= MXC_F_SPI_INTEN_RX_THD;
        }

        /*
        This processSPI call fills the TX FIFO as much as possible
        before launching the transaction.  Subsequent FIFO management will
        be handled from the SPI_IRQHandler. 
        */
        processSPI();
    }

    // Start the SPI transaction
    SPI->ctrl0 |= MXC_F_SPI_CTRL0_START;

    /*
    Handle slave-select (SS) deassertion.  This must be done AFTER launching the transaction
    to avoid a glitch on the SS line if:
    - The SS line is asserted
    - We want to deassert the line as part of this transaction

    As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the SS pin based
    on the value of CTRL->SS_CTRL, which causes the glitch.
    */
    if (deassert)
        SPI->ctrl0 &= ~MXC_F_SPI_CTRL0_SS_CTRL;
    else
        SPI->ctrl0 |= MXC_F_SPI_CTRL0_SS_CTRL;


    if (block)
        while(!((g_tx_done && g_master_done) && (src != NULL && txlen > 0)) && !(g_rx_done && (dest != NULL && rxlen > 0))) {
            /*
            The following polling is a safety fallback to catch any missed interrupts.
            This is especially common with extremely short transactions, where all 3 
            interrupts may fire almost simultaneously.
            */
            if ((src != NULL && txlen > 0) && SPI->intfl & MXC_F_SPI_INTFL_MST_DONE)
                g_master_done = 1;
            if ((src != NULL && txlen > 0) && MXC_DMA->ch[g_tx_channel].status & MXC_F_DMA_STATUS_CTZ_IF)
                g_tx_done = 1;
            if ((dest != NULL && rxlen > 0) && MXC_DMA->ch[g_rx_channel].status & MXC_F_DMA_STATUS_CTZ_IF)
                g_rx_done = 1;
        }

    return E_SUCCESS;
}