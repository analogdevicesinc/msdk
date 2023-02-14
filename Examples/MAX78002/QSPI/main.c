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
 * @brief   Hello World!
 * @details This example uses the UART to print to a terminal and flashes an LED.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "mxc_device.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "spi.h"
#include "tmr.h"
#include "dma.h"
#include "nvic_table.h"
#include "fastSPI.h"

/***** Definitions *****/
#define SPI MXC_SPI0
// #define SPI_SPEED 8000000
#define SPI_SPEED 1000
#define TEST_SIZE 800

// A macro to convert a DMA channel number to an IRQn number
#define GetIRQnForDMAChannel(x) ((IRQn_Type)(((x) == 0 ) ? DMA0_IRQn  : \
                                             ((x) == 1 ) ? DMA1_IRQn  : \
                                             ((x) == 2 ) ? DMA2_IRQn  : \
                                                           DMA3_IRQn))

/***** Globals *****/
int g_done = 0;
int g_dma_done = 0;
int g_tx_channel;
int g_rx_channel;

void spi_done(void *req, int result) 
{
    g_done = 1;
}

void DMA_TX_IRQHandler()
{
    uint32_t status = MXC_DMA->ch[g_tx_channel].status;

    if (status & MXC_F_DMA_STATUS_CTZ_IF) { // Count-to-Zero (TX complete)
        printf("CTZ!\n");
        g_done = 1;
        MXC_DMA->ch[g_tx_channel].status |= MXC_F_DMA_STATUS_CTZ_IF;  // Clear flag
    }

    if (status & MXC_F_DMA_STATUS_BUS_ERR) { // Bus Error
        printf("Bus error!\n");
        MXC_DMA->ch[g_tx_channel].status |= MXC_F_DMA_STATUS_BUS_ERR;  // Clear flag
    }
}

// void DMA0_IRQHandler()
// {
//     MXC_DMA_Handler();
// }

// void DMA1_IRQHandler()
// {
//     MXC_DMA_Handler();
// }

mxc_spi_req_t g_req = {
    .spi = SPI,
    .ssDeassert = 1,
    .ssIdx = 0,
    .completeCB = spi_done
};

#define MFID_EXPECTED 0x0D
#define KGD_EXPECTED 0x5D
#define DENSITY_EXPECTED 0b010

typedef struct {
    uint8_t MFID;
    uint8_t KGD;
    uint8_t density;
    int EID;
} ram_id_t;

/***** Functions *****/
int spi_init() 
{
    int err = 0;

    mxc_spi_pins_t pins = {
        .clock = TRUE,
        .miso = TRUE,
        .mosi = TRUE,
        .sdio2 = TRUE,
        .sdio3 = TRUE,
        .ss0 = TRUE,
        .ss1 = FALSE,
        .ss2 = FALSE,
        .vddioh = TRUE
    };

    err = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, pins);
    if (err) 
        return err;

    SPI->sstime |= 10<<MXC_F_SPI_SSTIME_POST_POS; // ss release immediately after last clk is confusing SALEAE
    SPI->sstime |= 10<<MXC_F_SPI_SSTIME_PRE_POS; // ss release to next block assert is extremelt short, confusing SALEAE

    err = MXC_SPI_SetDataSize(SPI, 8);
    if (err) return err;

    err = MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
    if (err) return err;

    NVIC_EnableIRQ(SPI0_IRQn);

    return err;
}

int spi_init_qspi()
{
    int err = 0;
    mxc_spi_pins_t pins = {
        .clock = TRUE,
        .miso = TRUE,
        .mosi = TRUE,
        .sdio2 = TRUE,
        .sdio3 = TRUE,
        .ss0 = TRUE,
        .ss1 = FALSE,
        .ss2 = FALSE,
        .vddioh = TRUE
    };

    err = MXC_SPI_Init(SPI, 1, 1, 1, 0, SPI_SPEED, pins);
    if (err) 
        return err;

    SPI->sstime |= 10<<MXC_F_SPI_SSTIME_POST_POS; // ss release immediately after last clk is confusing SALEAE
    SPI->sstime |= 10<<MXC_F_SPI_SSTIME_PRE_POS; // ss release to next block assert is extremelt short, confusing SALEAE

    err = MXC_SPI_SetDataSize(SPI, 8);
    if (err) return err;

    err = MXC_SPI_SetWidth(SPI, SPI_WIDTH_QUAD);
    if (err) return err;

    NVIC_EnableIRQ(SPI0_IRQn);
    NVIC_EnableIRQ(DMA0_IRQn);
    NVIC_EnableIRQ(DMA1_IRQn);
    NVIC_EnableIRQ(DMA2_IRQn);
    NVIC_EnableIRQ(DMA3_IRQn);

    return err;
}

int ram_reset(mxc_spi_req_t *req) {
    int err = 0;
    uint8_t data[2] = { 0x66, 0x99 };
    
    req->txData = &data[0];
    req->txLen = 1;
    req->rxLen = 0;
    req->ssDeassert = 1;
    err = MXC_SPI_MasterTransaction(req);
    if (err) return err;
    
    req->txData = &data[1];
    return MXC_SPI_MasterTransaction(req);
}

int ram_enter_quadmode(mxc_spi_req_t *req)
{
    int err = 0;
    uint8_t tx_data = 0x35;
    req->txData = &tx_data;
    req->txLen = 1;
    req->rxData = NULL;
    req->rxLen = 0;
    req->ssDeassert = 1;
    MXC_SPI_SetWidth(req->spi, SPI_WIDTH_STANDARD);
    err = MXC_SPI_MasterTransaction(req);
    if (err) return err;

    err = spi_init_qspi();
    return err;
}

int ram_exit_quadmode(mxc_spi_req_t *req)
{
    int err = 0;
    uint8_t tx_data = 0xF5;
    req->txData = &tx_data;
    req->txLen = 1;
    req->rxData = NULL;
    req->rxLen = 0;
    req->ssDeassert = 1;
    MXC_SPI_SetWidth(req->spi, SPI_WIDTH_QUAD);
    err = MXC_SPI_MasterTransaction(req);
    MXC_SPI_SetWidth(req->spi, SPI_WIDTH_STANDARD);
    return err;
}

int ram_read_id(mxc_spi_req_t *req, ram_id_t *out) {
    int err = 0;
    uint8_t tx_data = 0x9F;
    uint8_t rx_data[12] = { 0 };
    req->txData = &tx_data;
    req->txLen = 1;
    req->rxData = NULL;
    req->ssDeassert = 0;
    err = MXC_SPI_MasterTransaction(req);
    if (err) return err;

    req->txData = NULL;
    req->rxData = rx_data;
    req->rxLen = 12;
    req->ssDeassert = 1;
    err = MXC_SPI_MasterTransaction(req);

    out->MFID = rx_data[3];
    out->KGD = rx_data[4];
    out->density = (rx_data[5] & 0xe0) >> 5;  // Density is just top 3 bits

    // Formulate 44-bit EID from remaining bytes
    int tmp = rx_data[5] & 0x1F;
    for (int i = 0; i <= 6; i++) {
        tmp = tmp << 8;
        tmp |= rx_data[5 + i];
    }
    out->EID = tmp;

    // Validate against expected values
    if (out->MFID != MFID_EXPECTED) return E_INVALID;
    if (out->KGD != KGD_EXPECTED) return E_INVALID;
    if (out->density != DENSITY_EXPECTED) return E_INVALID;

    return err;
}

inline void _parse_spi_header(uint8_t cmd, uint32_t address, uint8_t *out)
{
    out[0] = cmd;
    out[1] = (address >> 16) & 0xFF;  // MSB first
    out[2] = (address >> 8) & 0xFF;
    out[3] = (address & 0xFF);
}

int _transmit_spi_header(mxc_spi_req_t *req, uint8_t cmd, uint32_t address)
{
    int err = 0;
    // SPI reads and writes will always start with 4 bytes.
    // A command byte, then a 24-bit address (MSB first)
    // However, the MXC_SPI drivers don't seem to support
    // standard half-duplex transactions, so breaking
    // the transaction up is necessary.
    // TODO: Support standard half-duplex in drivers
    uint8_t header[4];
    _parse_spi_header(cmd, address, header);

    // Transmit header, but keep Chip Select asserted
    req->txData = header;
    req->txLen = 4;
    req->rxData = NULL;
    req->rxLen = 0;
    req->ssDeassert = 0;
    err = MXC_SPI_MasterTransaction(req);
    
    req->txData = NULL; // Reset TX struct members here for convenience
    req->txLen = 0;
    req->ssDeassert = 1;
    return err;
}

int ram_read_slow(mxc_spi_req_t *req, uint32_t address, uint8_t *out, unsigned int len) 
{
    int err = 0;
    err = _transmit_spi_header(req, 0x03, address);
    if (err) return err;

    req->rxData = out;
    req->rxLen = len;
    g_done = 0;
    return MXC_SPI_MasterTransaction(req);
}

int ram_read_quad(mxc_spi_req_t *req, uint32_t address, uint8_t *out, unsigned int len) 
{
    int err = 0;
    uint8_t header[6];
    _parse_spi_header(0x0B, address, header);
    header[4] = 0xFF; // Extra dummy bytes to satisfy wait cycle
    header[5] = 0xFF;

    // Transmit header, but keep Chip Select asserted
    req->txData = header;
    req->txLen = 6;
    req->rxData = NULL;
    req->rxLen = 0;
    req->ssDeassert = 0;
    err = MXC_SPI_MasterTransaction(req);
    if (err) return err;

    req->txData = NULL; // Reset TX struct members here for convenience
    req->txLen = 0;
    req->ssDeassert = 1;
    req->rxData = out;
    req->rxLen = len;
    g_done = 0;
    return MXC_SPI_MasterTransaction(req);
}

int ram_read_quad_dma(mxc_spi_req_t *req, uint32_t address, uint8_t *out, unsigned int len)
{
    int err = 0;
    uint8_t header[6];
    _parse_spi_header(0x0B, address, header);
    header[4] = 0xFF; // Extra dummy bytes to satisfy wait cycle
    header[5] = 0xFF;

    // Transmit header, but keep Chip Select asserted
    req->txData = header;
    req->txLen = 6;
    req->rxData = NULL;
    req->rxLen = 0;
    req->ssDeassert = 0;
    err = MXC_SPI_MasterTransaction(req);
    if (err) return err;

    req->txData = NULL;
    req->txLen = 0;
    req->ssDeassert = 1;
    req->rxData = out;
    req->rxLen = len;
    g_done = 0;
    return MXC_SPI_MasterTransactionDMA(req);
}

int ram_write(mxc_spi_req_t *req, uint32_t address, uint8_t * data, unsigned int len) 
{
    int err = 0;
    err = _transmit_spi_header(req, 0x02, address);
    if (err) return err;

    req->txData = data;
    req->txLen = len;
    req->rxData = NULL;
    req->rxLen = 0;
    g_done = 0;
    return MXC_SPI_MasterTransaction(req);
}

int ram_write_quad(mxc_spi_req_t *req, uint32_t address, uint8_t * data, unsigned int len) 
{
    int err = 0;
    err = _transmit_spi_header(req, 0x38, address);
    if (err) return err;

    req->txData = data;
    req->txLen = len;
    req->rxData = NULL;
    req->rxLen = 0;
    g_done = 0;
    return MXC_SPI_MasterTransaction(req);
}

int ram_write_quad_dma(mxc_spi_req_t *req, uint32_t address, uint8_t * data, unsigned int len) 
{
    req->spi->dma |= (MXC_F_SPI_DMA_TX_FLUSH | MXC_F_SPI_DMA_RX_FLUSH);

    uint8_t header[4];
    _parse_spi_header(0x38, address, header);

    req->spi->ctrl1 = 4 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS;

    MXC_DMA->ch[g_tx_channel].src = (uint32_t)&header[0];
    MXC_DMA->ch[g_tx_channel].cnt = sizeof(header) / sizeof(header[0]);
    MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN;

    req->spi->ctrl0 |= MXC_F_SPI_CTRL0_START;

    while(!g_done) {}

    MXC_DMA->ch[g_tx_channel].src = (uint32_t)data;
    MXC_DMA->ch[g_tx_channel].cnt = len;
    MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN;
    req->spi->ctrl0 |= MXC_F_SPI_CTRL0_START;

    while(!g_done) {}

    return E_SUCCESS;
}

int ram_dma_init(mxc_spi_req_t *req)
{
    req->spi->dma |= (MXC_F_SPI_DMA_DMA_TX_EN); // Enable TX DMA FIFO
    req->spi->dma |= (0 << MXC_F_SPI_DMA_RX_LVL_POS | 31 << MXC_F_SPI_DMA_TX_LVL_POS); // Set FIFO levels

    int err = MXC_DMA_Init();
    if (err)
        return err;
    
    g_tx_channel = MXC_DMA_AcquireChannel();
    g_rx_channel = MXC_DMA_AcquireChannel();
    if (g_tx_channel < 0 || g_rx_channel < 0) {
        return E_NONE_AVAIL;  // Failed to acquire DMA channels
    }

    MXC_DMA->ch[g_tx_channel].ctrl &= ~(MXC_F_DMA_CTRL_EN | MXC_F_DMA_CTRL_RLDEN);
    MXC_DMA->ch[g_tx_channel].status &= ~(MXC_F_DMA_STATUS_CTZ_IF);
    MXC_DMA->ch[g_tx_channel].src = 0;  // Initialize DMA source to empty
    MXC_DMA->ch[g_tx_channel].ctrl |= (0x2F << MXC_F_DMA_CTRL_REQUEST_POS);  // Set DMA destination = SPI0 TX FIFO
    MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_SRCINC;  // Auto-increment the tx source buffer pointer during DMA transfers
    MXC_DMA->ch[g_tx_channel].ctrl |= (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE);  // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_tx_channel);  // Enable DMA interrupts for channel

    MXC_NVIC_SetVector(GetIRQnForDMAChannel(g_tx_channel), DMA_TX_IRQHandler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(g_tx_channel));

    return E_SUCCESS;
}

// *****************************************************************************
int main(void)
{
    int err = 0;

    MXC_Delay(MXC_DELAY_SEC(2));

#if 1
    // Initialize SPI

    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);

    const mxc_gpio_cfg_t spi_pins = {
        .port = MXC_GPIO0,
        .mask = MXC_GPIO_PIN_5 | MXC_GPIO_PIN_4 | MXC_GPIO_PIN_6 | MXC_GPIO_PIN_7 | MXC_GPIO_PIN_8 | MXC_GPIO_PIN_9,
        .func = MXC_GPIO_FUNC_ALT1,
        .pad = MXC_GPIO_PAD_NONE,
        .vssel = MXC_GPIO_VSSEL_VDDIOH
    };

    MXC_GPIO_Config(&spi_pins);
    
    MXC_SPI0->ctrl0 = (1 << MXC_F_SPI_CTRL0_SS_ACTIVE_POS) |    // Set SSEL = SS0
                        MXC_F_SPI_CTRL0_MST_MODE |              // Select controller mode
                        MXC_F_SPI_CTRL0_EN;                     // Enable SPI

    MXC_SPI0->ctrl2 = (8 << MXC_F_SPI_CTRL2_NUMBITS_POS);       // Set 8 bits per character
    
    MXC_SPI0->sstime = (1 << MXC_F_SPI_SSTIME_PRE_POS) |        // Remove any delay time between SSEL and SCLK edges
                        (1 << MXC_F_SPI_SSTIME_POST_POS) |
                        (1 << MXC_F_SPI_SSTIME_INACT_POS);

    MXC_SPI0->dma = MXC_F_SPI_DMA_TX_FIFO_EN |                  // Enable TX FIFO, set TX threshold to 31 
                    (31 << MXC_F_SPI_DMA_TX_THD_VAL_POS) |
                    MXC_F_SPI_DMA_DMA_TX_EN; 

    MXC_SPI0->intfl = MXC_SPI0->intfl;                           // Clear any any interrupt flags that may already be set

    MXC_SPI_SetFrequency(MXC_SPI0, SPI_SPEED);
    NVIC_EnableIRQ(SPI0_IRQn);

    // Initialize DMA

    err = MXC_DMA_Init();
    if (err)
        return err;
    
    g_tx_channel = MXC_DMA_AcquireChannel();
    g_rx_channel = MXC_DMA_AcquireChannel();
    if (g_tx_channel < 0 || g_rx_channel < 0) {
        return E_NONE_AVAIL;  // Failed to acquire DMA channels
    }

    MXC_DMA->ch[g_tx_channel].ctrl = MXC_F_DMA_CTRL_SRCINC | (0x2F << MXC_F_DMA_CTRL_REQUEST_POS);  // Enable incrementing the src address pointer, set to SPI0 TX destination
    MXC_DMA->ch[g_tx_channel].ctrl |= (MXC_F_DMA_CTRL_CTZ_IE | MXC_F_DMA_CTRL_DIS_IE);              // Enable CTZ and DIS interrupts
    MXC_DMA->inten |= (1 << g_tx_channel);                                                          // Enable DMA interrupts for channel

    MXC_NVIC_SetVector(GetIRQnForDMAChannel(g_tx_channel), DMA_TX_IRQHandler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(g_tx_channel));

    uint8_t test[5];
    for (int i = 0; i < 5; i++) {
        test[i] = i;
    }

    MXC_SPI0->ctrl1 = (5 << MXC_F_SPI_CTRL1_TX_NUM_CHAR_POS);
    MXC_DMA->ch[g_tx_channel].src = (uint32_t)test;
    MXC_DMA->ch[g_tx_channel].cnt = 5;
    MXC_DMA->ch[g_tx_channel].ctrl |= MXC_F_DMA_CTRL_EN;

    MXC_SPI0->ctrl0 |= MXC_F_SPI_CTRL0_START;

    while(!g_done) {}

    printf("Complete!  dma_tx_en: %i\n", (MXC_SPI0->dma >> MXC_F_SPI_DMA_DMA_TX_EN_POS) & 0b1);

#else

    err = spi_init_qspi();
    ram_exit_quadmode(&g_req);

    printf("Initializing SPI...\n");
    err = spi_init();
    if (err) return err;

    printf("Resetting SRAM...\n");
    err = ram_reset(&g_req);
    if (err) return err;

    printf("Reading ID...\n");
    ram_id_t id;
    err = ram_read_id(&g_req, &id);
    if (err) {
        printf("Failed to read expected SRAM ID!\n");
        return err;
    }
    printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", id.MFID, id.KGD, id.density, id.EID);

    uint8_t tx_buffer[TEST_SIZE];
    uint8_t rx_buffer[TEST_SIZE];
    memset(rx_buffer, 0, TEST_SIZE);

    // Time tx_buffer initialization as benchmark
    MXC_TMR_SW_Start(MXC_TMR0);
    memset(tx_buffer, 0xA5, TEST_SIZE);
    int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("(Benchmark) Wrote %i bytes to internal SRAM in %ius\n", TEST_SIZE, elapsed);

    // Benchmark standard-width SPI write to external SRAM
    MXC_TMR_SW_Start(MXC_TMR0);
    int address = 0;
    ram_write(&g_req, address, tx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Wrote %i bytes in %ius\n", TEST_SIZE, elapsed);

    // Validate test pattern
    uint8_t buffer = 0;
    ram_read_slow(&g_req, address, rx_buffer, TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", i, 0xA5, buffer);
        }
    }

    // Invert test pattern
    memset(tx_buffer, ~(0x5A), TEST_SIZE);
    memset(rx_buffer, 0, TEST_SIZE);

    // Benchmark QSPI write to external SRAM
    err = ram_enter_quadmode(&g_req);
    MXC_TMR_SW_Start(MXC_TMR0);
    err = ram_write_quad(&g_req, 1024, tx_buffer, TEST_SIZE);
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Wrote %i bytes w/ QSPI in %ius\n", TEST_SIZE, elapsed);

    MXC_Delay(MXC_DELAY_MSEC(1));

    // Validate
    MXC_Delay(MXC_DELAY_MSEC(1));
    ram_read_quad(&g_req, 1024, rx_buffer, TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", i, 0xA5, rx_buffer[i]);
        }
    }

    // Invert test pattern
    memset(tx_buffer, ~(0x5A), TEST_SIZE);
    memset(rx_buffer, 0, TEST_SIZE);

    // Benchmark QSPI write + DMA to external SRAM
    err = ram_dma_init(&g_req);
    if (err) {
        printf("Failed to initialize DMA!\n");
        return err;
    }

    MXC_TMR_SW_Start(MXC_TMR0);
    err = ram_write_quad_dma(&g_req, 1024, tx_buffer, TEST_SIZE);
    if (err) return err;
    while(!g_done) {}
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Wrote %i bytes w/ QSPI in %ius (DMA)\n", TEST_SIZE, elapsed);

    MXC_Delay(MXC_DELAY_MSEC(1));

    // Validate
    MXC_Delay(MXC_DELAY_MSEC(1));
#if 1
    ram_read_quad(&g_req, 1024, rx_buffer, TEST_SIZE);
#else
    ram_read_quad_dma(&g_req, 1024, rx_buffer, TEST_SIZE);
    while(!g_done) {}
#endif
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", i, 0xA5, rx_buffer[i]);
        }
    }

    ram_exit_quadmode(&g_req);

#endif

    printf("Success!\n");
    return err;
}
