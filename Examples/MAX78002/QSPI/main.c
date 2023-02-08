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

/***** Definitions *****/
#define SPI MXC_SPI0
// #define SPI_SPEED 8000000
#define SPI_SPEED 100000
#define TEST_SIZE 800

/***** Globals *****/
int g_done = 0;
void spi_done(void *req, int result) 
{
    g_done = 1;
}

void DMA0_IRQHandler()
{
    MXC_DMA_Handler();
}

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
    return MXC_SPI_MasterTransaction(req);
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
    int err = 0;
    err = _transmit_spi_header(req, 0x38, address);
    if (err) return err;

    req->txData = data;
    req->txLen = len;
    req->rxData = NULL;
    req->rxLen = 0;
    return MXC_SPI_MasterTransactionDMA(req);
}

// *****************************************************************************
int main(void)
{
    int err = 0;

    MXC_Delay(MXC_DELAY_SEC(2));

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
    MXC_TMR_SW_Start(MXC_TMR0);
    err = ram_write_quad_dma(&g_req, 1024, tx_buffer, TEST_SIZE);
    if (err) return err;
    while(!g_done) {}
    elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Wrote %i bytes w/ QSPI in %ius (DMA)\n", TEST_SIZE, elapsed);

    MXC_Delay(MXC_DELAY_MSEC(1));

    // Validate
    MXC_Delay(MXC_DELAY_MSEC(1));
    ram_read_quad(&g_req, 1024, rx_buffer, TEST_SIZE);
    for (int i = 0; i < TEST_SIZE; i++) {
        if (rx_buffer[i] != tx_buffer[i]) {
            printf("Value mismatch at addr %i, expected 0x%x but got 0x%x\n", i, 0xA5, rx_buffer[i]);
        }
    }

    ram_exit_quadmode(&g_req);

    printf("Success!\n");
    return err;
}
