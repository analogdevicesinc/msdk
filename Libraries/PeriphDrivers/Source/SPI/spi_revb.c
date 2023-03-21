/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi.h"
#include "spi_reva.h"
#include "dma_reva.h"

/* **** Definitions **** */

static mxc_spi_callback_t spi_cb = NULL;
static void* spi_cb_data = NULL;

typedef struct {
    mxc_spi_regs_t *spi; 
    int ssIdx; ///< Slave select line to use (Master only, ignored in slave mode)
    int ssDeassert; ///< 1 - Deassert SS at end of transaction, 0 - leave SS asserted
    uint8_t *txData; ///< Buffer containing transmit data. For character sizes
    ///< < 8 bits, pad the MSB of each byte with zeros. For
    ///< character sizes > 8 bits, use two bytes per character
    ///< and pad the MSB of the upper byte with zeros
    uint8_t *rxData; ///< Buffer to store received data For character sizes
    ///< < 8 bits, pad the MSB of each byte with zeros. For
    ///< character sizes > 8 bits, use two bytes per character
    ///< and pad the MSB of the upper byte with zeros
    uint32_t txLen; ///< Number of bytes to be sent from txData
    uint32_t rxLen; ///< Number of bytes to be stored in rxData
    uint32_t txCnt; ///< Number of bytes actually transmitted from txData
    uint32_t rxCnt; ///< Number of bytes stored in rxData

    spi_complete_cb_t completeCB; ///< Pointer to function called when transaction is complete

} mxc_spi_req_t;

typedef struct {
    mxc_spi_req_t *req;
    int txDMAChannel;
    int rxDMAChannel;
    mxc_spi_callback_t callBack;
    void *callBack_data;
    mxc_spi_state_t state;
    uint32_t DMA_REQSEL_SPInTX;
    uint32_t DMA_REQSEL_SPInRX;
    bool deassert;
} mxc_spi_handle_data_t;

static mxc_dma_reva_regs_t *MXC_SPIDMA;

// Enables clock, gpio pins
int MXC_SPI_RevA_Init(mxc_spi_reva_regs_t *spi, mxc_spi_init_t *init)
{
    int error;
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPIx);     // Enable Clock for selected SPI block
    MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPIx);          // Reset selected SPI block

    // TODO: SPI modes (Quad/3wire/etc)
    MXC_GPIO_Config(&SPIMainPins);                      // Configure appropriate GPIO pins for SPI mode
    MXC_GPIO_Config(&SPISlaveSelectPinOrPins);  

    // Set up CS Control Scheme if not controlled by Application code
    if(init->csControl != MXC_SPI_CSCONTROL_SW_APP) {
        MXC_GPIO_Config(&SPISlaveSelectPinOrPins);

        if(init->csControl == MXC_SPI_CSCONTROL_HW_AUTO) {
            spi->ctrl0 |= (MXC_V_SPI_REVA_CTRL0_SS_ACTIVE_SS0 << init->ssIdx);
        }
    }

    // Enable SPI
    spi->ctrl0 = MXC_F_SPI_REVA_CTRL0_EN;

    // Set Master or Slave Mode.
    if(init->msMode) {
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
    } else {
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_MST_MODE);
    }

    // Set Data Size
    MXC_SPI_SetDataSize((mxc_spi_regs_t *)spi, init->dataSize);

    // Remove any delay between SSEL and SCLK edges
    spi->sstime =
        ((0x1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) | (0x1 << MXC_F_SPI_REVA_SSTIME_POST_POS) |
         (0x1 << MXC_F_SPI_REVA_SSTIME_INACT_POS));

    // Set bit rate
    MXC_SPI_SetFrequency((mxc_spi_regs_t *)spi, init->freq);

    // Clear all interrupt flags
    spi->intfl = spi->intfl;

    // Set up DMA if used.
    if(init->useDMA) {
        // TODO: Update for peripherals with more than two DMA instances
        MXC_SPIDMA = init->dma;

        error = MXC_DMA_Init();
        if(error != E_NO_ERROR) {
            return error;
        }

        handle->txDMAChannel = MXC_DMA_AcquireChannel();
        handle->rxDMAChannel = MXC_DMA_AcquireChannel();
        if((handle->txDMAChannel < 0) || (handle->rxDMAChannel < 0)) {
            return E_BAD_STATE;
        }

        MXC_SPIDMA->ch[handle->txDMAChannel].ctrl = MXC_F_DMA_REVA_CTRL_SRCINC;  // Auto-increment the tx source buffer pointer during DMA transfers
        MXC_SPIDMA->ch[handle->rxDMAChannel].ctrl = MXC_F_DMA_REVA_CTRL_DIS_IE |  // Enable the DMA finished interrupt for the receive channel
                                  MXC_F_DMA_REVA_CTRL_DSTINC;                         // Auto-increment the rx destination buffer pointer during DMA transfers                       

        // Set DMA destination and source to SPInTX and SPInRX
        MXC_SPIDMA->ch[handle->txDMAChannel].ctrl |= handle->DMA_REQSEL_SPInTX; // Set the DMA destination to the SPI's transmit FIFO
        MXC_SPIDMA->ch[handle->rxDMAChannel].ctrl |= handle->DMA_REQSEL_SPInRX; // Set the DMA source to the SPI's receive FIFO

        // Enable DMA interrupts.  (We are only enabling interrupts on the receive channel.  By
        //  the nature of SPI, if the receive operation is complete, the transmit operation is
        //  guaranteed to have already completed.)
        MXC_SPIDMA->inten |= (1 << rxChannel);
    }
}

void MXC_SPI_RevA_Handler()
{
    // Clear interrupt flags

    // Read Data

    // Write Data
}

void MXC_SPI_RevA_DMA_Handler()
{
    MXC_SPIDMA->ch[handle->rxDMAChannel].status |= MXC_F_DMA_STATUS_CTZ_IF;

    if(handle->callBack) {
        handle->callBack(handle->callBack_data);
    }
}

int MXC_SPI_RevA_MasterTransaction(mxc_spi_req_t *req)
{
    if (w)




    SPI_INST->dma &= ~(MXC_F_SPI_DMA_TX_EN | FASTSPI_DMA_RX_EN);                  // Disable DMA for SPI transmit and receive operations
    SPI_INST->inten |= FASTSPI_INTEN_RX_THD | FASTSPI_INTEN_MST_DONE;           // Enable SPI receive threshold and transaction complete interrupts

    txBytes = nbBytes;                                                          // Save the number of bytes to be transmitted
    txBuff = pBufferTx;                                                         // Save the transmit buffer pointer
    rxBytes = nbBytes;                                                          // Save the number of bytes to be received
    rxBuff = pBufferRx;                                                         // Save the receive buffer pointer
    processSPI();                                                               // Configure the SPI transaction

}

int MXC_SPI_RevA_MasterReceiveDMA()
{

}

int MXC_SPI_RevA_MasterTransactionDMA(mxc_spi_req_t *req)
{
    mxc_spi_reva_regs_t *spi = (mxc_spi_reva_regs_t *)(req->spi);

    spi->ctrl1 = (req->txLen << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR) |                  // Configure the transfer size
                      (req->rxLen << MXC_F_SPI_REVA_CTRL1_X_NUM_CHAR);

    spi->dma |= (FASTSPI_DMA_TX_EN | FASTSPI_DMA_RX_EN);                   // Enable DMA for SPI transmit and receive operations
    spi->inten &= ~(FASTSPI_INTEN_RX_THD | FASTSPI_INTEN_MST_DONE);        // Disable SPI interrupts, we'll use the DMA interrupts for notificataion
        
    MXC_SPIDMA->ch[handle->txDMAChannel].src = (uint32_t)pBufferTx;    // Set up the source pointer for the transmit DMA
    MXC_SPIDMA->ch[handle->txDMAChannel].cnt = nbBytes;                // Set up the number of bytes to transfer
    MXC_SPIDMA->ch[handle->txDMAChannel].ctrl |= FASTSPI_DMA_CTRL_EN;  // Start the transmit DMA

    MXC_SPIDMA->ch[rxChannel].dst = (uint32_t)pBufferRx;    // Set up the destination pointer for the receive DMA
    MXC_SPIDMA->ch[rxChannel].cnt = nbBytes;                // Set up the number of bytes to transfer
    MXC_SPIDMA->ch[rxChannel].ctrl |= FASTSPI_DMA_CTRL_EN;  // Start the receive DMA

    // Start the SPI Transaction
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;
}



static FastSPI_Callback spi_cb = NULL;  // The callback function to use when a SPI transaction completes
static void* spi_cb_data = NULL;        // Data to pass along with the callback function

static uint8_t* txBuff;                 // The transmit buffer to use with SPI transactions
static uint8_t* rxBuff;                 // The receive buffer to use with SPI transactions
static uint32_t txBytes;                // The number of bytes to send over SPI
static uint32_t rxBytes;                // The number of bytes to receive over SPI

static int txChannel;                   // The DMA channel assigned to SPI transmit
static int rxChannel;                   // The DMA channel assinged to SPI receive

static void processSPI()
{
    // Clear interrupt flags
    SPI_INST->intfl |= FASTSPI_INTFL_RX_THD | FASTSPI_INTFL_MST_DONE;

    // Unload any SPI data that has come in
    while(rxBytes && (SPI_INST->dma & FASTSPI_DMA_RX_LVL)) {
        *rxBuff++ = SPI_INST->fifo8[0];
        rxBytes--;
    }

    // Write any pending bytes out.
    while(txBytes && (((SPI_INST->dma & FASTSPI_DMA_TX_LVL) >> FASTSPI_DMA_TX_LVL_POS) < MXC_SPI_FIFO_DEPTH)) {
        SPI_INST->fifo8[0] = *txBuff++;
        txBytes--;
    }
}

static void SPI_IRQHandler()
{
    // Two SPI interrupts are enabled: "rx threshold crossed" and "spi complete".
    //  Both indicate there is SPI data in the RX FIFO ready to be consumed.
    processSPI();

    // If we've received all the bytes that were asked for, the SPI transaction
    //  is complete and we can call the registered callback function.
    if(!rxBytes) {
        if(spi_cb) {
            spi_cb(spi_cb_data);
        }
    }
}

static void MXC_SPI_RevB_DMAHandler()
{
    // The only DMA interrupt that is enabled is "transaction complete".  If
    //  this interrupt occurs it means the pending SPI transaction has completed.
  
    // Clear the DMA interrupt flag
    ((fastspi_dma_regs_t*)MXC_DMA)->ch[rxChannel].status |= FASTSPI_DMA_STATUS_CTZ_IF;
    
    // See if a SPI callback has been registered and call it.
    if(spi_cb) {
        spi_cb(spi_cb_data);
    }
}

int FastSPI_Init()
{
    MXC_SYS_ClockEnable(FASTSPI_CFG_CLOCK_EN_BIT);                              // Enable clock to selected SPI block
    MXC_SYS_Reset_Periph(FASTSPI_CFG_RESET_BIT);                                // Reset selected SPI block
    MXC_GPIO_Config(&SPIMainPins);                                              // Put appropriate GPIO pins in SPI mode
    MXC_GPIO_Config(&SPISSelPin);                                               // Put appropriate GPIO pins in SPI mode

    ((fastspi_gpio_regs_t*)SPIMainPins.port)->ds0 |= SPIMainPins.mask;          // Set strongest possible drive strength for the SPI pins
    ((fastspi_gpio_regs_t*)SPIMainPins.port)->ds1 |= SPIMainPins.mask;
    ((fastspi_gpio_regs_t*)SPISSelPin.port)->ds0 |= SPISSelPin.mask;                                      
    ((fastspi_gpio_regs_t*)SPISSelPin.port)->ds1 |= SPISSelPin.mask;
    SPI_INST->ctrl0 = (FASTSPI_CTRL0_SS_ACTIVE_SS0 << FASTSPI_CFG_SSEL_IDX) |   // Configure which SSEL pin is used
                      FASTSPI_CTRL0_MST_MODE |                                  // Select controller mode
                      FASTSPI_CTRL0_EN;                                         // Enable SPI
    SPI_INST->ctrl2 = FASTSPI_CTRL2_NUMBITS_8;                                  // Perform 8-bit transfers
    SPI_INST->sstime = (1 << FASTSPI_SSTIME_PRE_POS) |                          // Remove any delay time betweeen SSEL and SCLK edges
                       (1 << FASTSPI_SSTIME_POST_POS) | 
                       (1 << FASTSPI_SSTIME_INACT_POS);
    SPI_INST->clkctrl = (FASTSPI_CFG_CLOCK_LOW << FASTSPI_CLKCTRL_LO_POS) | 
                        (FASTSPI_CFG_CLOCK_HIGH << FASTSPI_CLKCTRL_HI_POS);     // Set the SPI clock rate
    SPI_INST->dma = FASTSPI_RX_FIFO_EN |                                        // Enable the receive FIFO
                    FASTSPI_TX_FIFO_EN |                                        // Enable the transmit FIFO
                    (FASTSPI_CFG_RX_THRESHOLD << FASTSPI_DMA_RX_THD_VAL_POS) |  // Configure the receive threshold that triggers another DMA transfer
                    (FASTSPI_CFG_TX_THRESHOLD << FASTSPI_DMA_TX_THD_VAL_POS);   // Configure the transmit treshold that triggers another DMA transfer
    SPI_INST->intfl = SPI_INST->intfl;                                          // Clear any interrupt flags that may already be set

    // Enable SPI interrupts
    // Set in application.
    //MXC_NVIC_SetVector(MXC_SPI_GET_IRQ(FASTSPI_CFG_INST), SPI_IRQHandler);
    //NVIC_EnableIRQ(MXC_SPI_GET_IRQ(FASTSPI_CFG_INST));

    // Intialize DMA and reserve a couple of channels
    if(MXC_DMA_Init() != E_NO_ERROR) {
        return 0;
    }
    txChannel = MXC_DMA_AcquireChannel();
    rxChannel = MXC_DMA_AcquireChannel();
    if((txChannel < 0) || (rxChannel < 0)) {
        return 0;
    }
    
    ((fastspi_dma_regs_t*)MXC_DMA)->ch[txChannel].ctrl = FASTSPI_DMA_CTRL_SRCINC |  // Auto-increment the tx source buffer pointer during DMA transfers
                                  FASTSPI_CFG_TX_DMA_REQSEL;                        // Set the DMA destination to the SPI's transmit FIFO
    ((fastspi_dma_regs_t*)MXC_DMA)->ch[rxChannel].ctrl = FASTSPI_DMA_CTRL_DIS_IE |  // Enable the DMA finished interrupt for the receive channel
                                  FASTSPI_DMA_CTRL_DSTINC |                         // Auto-increment the rx destination buffer pointer during DMA transfers
                                  FASTSPI_CFG_RX_DMA_REQSEL;                        // Set the DMA source to the SPI's receive FIFO

    // Enable DMA interrupts.  (We are only enabling interrupts on the receive channel.  By
    //  the nature of SPI, if the receive operation is complete, the transmit operation is
    //  guaranteed to have already completed.)
    ((fastspi_dma_regs_t*)MXC_DMA)->inten |= (1 << rxChannel);
    MXC_NVIC_SetVector(GetIRQnForDMAChannel(rxChannel), DMA_IRQHandler);
    NVIC_EnableIRQ(GetIRQnForDMAChannel(rxChannel));

    return 1;
}

void MXC_SPI_RevB_ReadWrite(uint8_t *pBufferTx, uint8_t *pBufferRx, uint32_t nbBytes, int useDma)
{
    SPI_INST->ctrl1 = (nbBytes << FASTSPI_CTRL1_TX_NUM_CHAR_POS) |                  // Configure the transfer size
                      (nbBytes << FASTSPI_CTRL1_RX_NUM_CHAR_POS);

    if(useDma) {
        SPI_INST->dma |= (FASTSPI_DMA_TX_EN | FASTSPI_DMA_RX_EN);                   // Enable DMA for SPI transmit and receive operations
        SPI_INST->inten &= ~(FASTSPI_INTEN_RX_THD | FASTSPI_INTEN_MST_DONE);        // Disable SPI interrupts, we'll use the DMA interrupts for notificataion
        
        ((fastspi_dma_regs_t*)MXC_DMA)->ch[txChannel].src = (uint32_t)pBufferTx;    // Set up the source pointer for the transmit DMA
        ((fastspi_dma_regs_t*)MXC_DMA)->ch[txChannel].cnt = nbBytes;                // Set up the number of bytes to transfer
        ((fastspi_dma_regs_t*)MXC_DMA)->ch[txChannel].ctrl |= FASTSPI_DMA_CTRL_EN;  // Start the transmit DMA

        ((fastspi_dma_regs_t*)MXC_DMA)->ch[rxChannel].dst = (uint32_t)pBufferRx;    // Set up the destination pointer for the receive DMA
        ((fastspi_dma_regs_t*)MXC_DMA)->ch[rxChannel].cnt = nbBytes;                // Set up the number of bytes to transfer
        ((fastspi_dma_regs_t*)MXC_DMA)->ch[rxChannel].ctrl |= FASTSPI_DMA_CTRL_EN;  // Start the receive DMA
    } else {
        SPI_INST->dma &= ~(FASTSPI_DMA_TX_EN | FASTSPI_DMA_RX_EN);                  // Disable DMA for SPI transmit and receive operations
        SPI_INST->inten |= FASTSPI_INTEN_RX_THD | FASTSPI_INTEN_MST_DONE;           // Enable SPI receive threshold and transaction complete interrupts

        txBytes = nbBytes;                                                          // Save the number of bytes to be transmitted
        txBuff = pBufferTx;                                                         // Save the transmit buffer pointer
        rxBytes = nbBytes;                                                          // Save the number of bytes to be received
        rxBuff = pBufferRx;                                                         // Save the receive buffer pointer
        processSPI();                                                               // Configure the SPI transaction
    }

    SPI_INST->ctrl0 |= FASTSPI_CTRL0_START;                                         // Start the SPI transaction
}

void MXC_SPI_RevB_SetRegisterCallback(FastSPI_Callback spiCallback, void * hDevice)
{
    // Save the callback information for later use.
    spi_cb = spiCallback;
    spi_cb_data = hDevice;
}






//************************* Old

typedef struct {
    mxc_spi_reva_req_t *req;
    int started;
    unsigned last_size;
    unsigned ssDeassert;
    unsigned defaultTXData;
    int channelTx;
    int channelRx;
    int mtMode;
    int mtFirstTrans;
    bool txrx_req;
    uint8_t req_done;
    uint8_t async;
} spi_req_reva_state_t;

static spi_req_reva_state_t states[MXC_SPI_INSTANCES];

static uint32_t MXC_SPI_RevA_MasterTransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req);
static uint32_t MXC_SPI_RevA_TransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req);
static uint32_t MXC_SPI_RevA_SlaveTransHandler(mxc_spi_reva_req_t *req);
static void MXC_SPI_RevA_SwapByte(uint8_t *arr, size_t length);
static int MXC_SPI_RevA_TransSetup(mxc_spi_reva_req_t *req);


int MXC_SPI_RevA_Init(mxc_spi_reva_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                      unsigned ssPolarity, unsigned int hz)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    states[spi_num].req = NULL;
    states[spi_num].last_size = 0;
    states[spi_num].ssDeassert = 1;
    states[spi_num].defaultTXData = 0;
    states[spi_num].mtMode = 0;
    states[spi_num].mtFirstTrans = 0;
    states[spi_num].channelTx = E_NO_DEVICE;
    states[spi_num].channelRx = E_NO_DEVICE;

    spi->ctrl0 = (MXC_F_SPI_REVA_CTRL0_EN);
    spi->sstime =
        ((0x1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) | (0x1 << MXC_F_SPI_REVA_SSTIME_POST_POS) |
         (0x1 << MXC_F_SPI_REVA_SSTIME_INACT_POS));

    //set master
    if (masterMode) {
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
    } else {
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_MST_MODE);
    }

    MXC_SPI_SetFrequency((mxc_spi_regs_t *)spi, hz);

    if (ssPolarity > (MXC_F_SPI_REVA_CTRL2_SS_POL >> MXC_F_SPI_REVA_CTRL2_SS_POL_POS)) {
        return E_BAD_PARAM;
    }

    //set slave select polarity
    spi->ctrl2 |= (ssPolarity << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);

    // Clear the interrupts
    spi->intfl = spi->intfl;

    if (numSlaves == 1) {
        spi->ctrl0 |= MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0;
    }

    if (numSlaves == 2) {
        spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1);
    }

    if (numSlaves == 3) {
        spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1 |
                       MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS2);
    }

    if (numSlaves == 4) {
        spi->ctrl0 |= (MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS0 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS1 |
                       MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS2 | MXC_S_SPI_REVA_CTRL0_SS_ACTIVE_SS3);
    }

    //set quad mode
    if (quadModeUsed) {
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA_Shutdown(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    mxc_spi_reva_req_t *temp_req;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    //disable and clear interrupts
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Disable SPI and FIFOS
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

    //call all of the pending callbacks for this spi
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    if (states[spi_num].req != NULL) {
        //save the request
        temp_req = states[spi_num].req;
        MXC_FreeLock((uint32_t *)(uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].req->completeCB != NULL) {
            states[spi_num].req->completeCB(temp_req, E_SHUTDOWN);
        }
    }

    // Clear registers
    spi->ctrl0 = 0;
    spi->ctrl1 = 0;
    spi->ctrl2 = 0;
    spi->sstime = 0;

    // release any acquired DMA channels
    if (states[spi_num].channelTx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelTx);
        states[spi_num].channelTx = E_NO_DEVICE;
    }
    if (states[spi_num].channelRx >= 0) {
        MXC_DMA_RevA_ReleaseChannel(states[spi_num].channelRx);
        states[spi_num].channelRx = E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA_ReadyForSleep(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);

    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY || (spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) ||
        (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

int MXC_SPI_RevA_SetFrequency(mxc_spi_reva_regs_t *spi, unsigned int hz)
{
    int hi_clk, lo_clk, scale;
    uint32_t freq_div;

    // Check if frequency is too high
    if (hz > PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Set the clock high and low
    freq_div = MXC_SPI_GetPeripheralClock((mxc_spi_regs_t *)spi);
    freq_div = (freq_div / hz);

    hi_clk = freq_div / 2;
    lo_clk = freq_div / 2;
    scale = 0;

    if (freq_div % 2) {
        hi_clk += 1;
    }

    while (hi_clk >= 16 && scale < 8) {
        hi_clk /= 2;
        lo_clk /= 2;
        scale++;
    }

    if (scale == 8) {
        lo_clk = 15;
        hi_clk = 15;
    }

    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_LO,
                 (lo_clk << MXC_F_SPI_REVA_CLKCTRL_LO_POS));
    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_HI,
                 (hi_clk << MXC_F_SPI_REVA_CLKCTRL_HI_POS));
    MXC_SETFIELD(spi->clkctrl, MXC_F_SPI_REVA_CLKCTRL_CLKDIV,
                 (scale << MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS));

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA_GetFrequency(mxc_spi_reva_regs_t *spi)
{
    if (MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) < 0) {
        return E_BAD_PARAM;
    }

    unsigned scale, lo_clk, hi_clk;

    scale = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_CLKDIV) >> MXC_F_SPI_REVA_CLKCTRL_CLKDIV_POS;
    hi_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_HI) >> MXC_F_SPI_REVA_CLKCTRL_HI_POS;
    lo_clk = (spi->clkctrl & MXC_F_SPI_REVA_CLKCTRL_LO) >> MXC_F_SPI_REVA_CLKCTRL_LO_POS;

    return (PeripheralClock / (1 << scale)) / (lo_clk + hi_clk);
}

int MXC_SPI_RevA_SetDataSize(mxc_spi_reva_regs_t *spi, int dataSize)
{
    int spi_num;

    // HW has problem with these two character sizes
    if (dataSize == 1 || dataSize > 16) {
        return E_BAD_PARAM;
    }

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Setup the character size
    if (!(spi->stat & MXC_F_SPI_REVA_STAT_BUSY) && states[spi_num].ssDeassert == 1) {
        //disable spi to change transfer size
        spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
        // set bit size
        states[spi_num].last_size = dataSize;

        if (dataSize < 16) {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         dataSize << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        } else {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         0 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS); //may not be neccessary
        }

        //enable spi to change transfer size
        spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);
    } else {
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA_GetDataSize(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    if (!(spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS)) {
        return 16;
    }

    return (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS) >> MXC_F_SPI_REVA_CTRL2_NUMBITS_POS;
}

int MXC_SPI_RevA_SetSlave(mxc_spi_reva_regs_t *spi, int ssIdx)
{
    int spi_num;

    if (ssIdx >= MXC_SPI_SS_INSTANCES) {
        return E_BAD_PARAM;
    }

    //check if in master mode
    if (!(spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE)) {
        return E_BAD_STATE;
    }

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    // Setup the slave select
    // Activate chosen SS pin
    spi->ctrl0 |= (1 << ssIdx) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS;
    // Deactivate all unchosen pins
    spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_ACTIVE |
                  ((1 << ssIdx) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);
    return E_NO_ERROR;
}

int MXC_SPI_RevA_GetSlave(mxc_spi_reva_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    return ((spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_SS_ACTIVE) >> MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS) >>
           1;
}

int MXC_SPI_RevA_SetWidth(mxc_spi_reva_regs_t *spi, mxc_spi_reva_width_t spiWidth)
{
    int spi_num;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    spi->ctrl2 &= ~(MXC_F_SPI_REVA_CTRL2_THREE_WIRE | MXC_F_SPI_REVA_CTRL2_DATA_WIDTH);

    switch (spiWidth) {
    case SPI_REVA_WIDTH_3WIRE:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;

    case SPI_REVA_WIDTH_STANDARD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_MONO;
        break;

    case SPI_REVA_WIDTH_DUAL:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL;
        break;

    case SPI_REVA_WIDTH_QUAD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
        break;
    }

    return E_NO_ERROR;
}

mxc_spi_reva_width_t MXC_SPI_RevA_GetWidth(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) {
        return SPI_REVA_WIDTH_3WIRE;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL) {
        return SPI_REVA_WIDTH_DUAL;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD) {
        return SPI_REVA_WIDTH_QUAD;
    }

    return SPI_REVA_WIDTH_STANDARD;
}

int MXC_SPI_RevA_SetMode(mxc_spi_reva_regs_t *spi, mxc_spi_reva_mode_t spiMode)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    switch (spiMode) {
    case SPI_REVA_MODE_0:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_1:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_2:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    case SPI_REVA_MODE_3:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    default:
        break;
    }

    return E_NO_ERROR;
}

mxc_spi_reva_mode_t MXC_SPI_RevA_GetMode(mxc_spi_reva_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPHA) {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return SPI_REVA_MODE_3;
        } else {
            return SPI_REVA_MODE_2;
        }
    } else {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return SPI_REVA_MODE_1;
        }
    }

    return SPI_REVA_MODE_0;
}

int MXC_SPI_RevA_StartTransmission(mxc_spi_reva_regs_t *spi)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    (void)spi_num;

    if (MXC_SPI_GetActive((mxc_spi_regs_t *)spi) == E_BUSY) {
        return E_BUSY;
    }

    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    return E_NO_ERROR;
}

int MXC_SPI_RevA_GetActive(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA_ReadRXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                     unsigned int len)
{
    unsigned rx_avail, bits;
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);

    if (!bytes || !len) {
        return 0;
    }

    rx_avail = MXC_SPI_GetRXFIFOAvailable((mxc_spi_regs_t *)spi);
    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)spi);

    if (len > rx_avail) {
        len = rx_avail;
    }

    if (bits > 8) {
        len &= ~(unsigned)0x1;
    }

    unsigned cnt = 0;

    if (bits <= 8 || len >= 2) {
        // Read from the FIFO
        while (len) {
            if (len > 3) {
                memcpy((uint8_t *)(&bytes[cnt]), (void *)(&spi->fifo32), 4);
                len -= 4;
                cnt += 4;
            } else if (len > 1) {
                memcpy((uint8_t *)(&bytes[cnt]), (void *)(&spi->fifo16[0]), 2);
                len -= 2;
                cnt += 2;

            } else {
                ((uint8_t *)bytes)[cnt++] = spi->fifo8[0];
                len -= 1;
            }

            // Don't read less than 2 bytes if we are using greater than 8 bit characters
            if (len == 1 && bits > 8) {
                break;
            }
        }
    }

    return cnt;
}

unsigned int MXC_SPI_RevA_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;
}

unsigned int MXC_SPI_RevA_WriteTXFIFO(mxc_spi_reva_regs_t *spi, unsigned char *bytes,
                                      unsigned int len)
{
    unsigned tx_avail, bits;
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);

    if (!bytes || !len) {
        return 0;
    }

    tx_avail = MXC_SPI_GetTXFIFOAvailable((mxc_spi_regs_t *)spi);
    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)spi);

    if (len > tx_avail) {
        len = tx_avail;
    }

    if (bits > 8) {
        len &= ~(unsigned)0x1;
    }

    unsigned cnt = 0;

    while (len) {
        if (len > 3) {
            memcpy((void *)(&spi->fifo32), (uint8_t *)(&bytes[cnt]), 4);

            len -= 4;
            cnt += 4;

        } else if (len > 1) {
            memcpy((void *)(&spi->fifo16[0]), (uint8_t *)(&bytes[cnt]), 2);

            len -= 2;
            cnt += 2;

        } else if (bits <= 8) {
            spi->fifo8[0] = ((uint8_t *)bytes)[cnt++];
            len--;
        }
    }

    return cnt;
}

unsigned int MXC_SPI_RevA_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    return MXC_SPI_FIFO_DEPTH -
           ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);
}

void MXC_SPI_RevA_ClearRXFIFO(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    spi->dma |= MXC_F_SPI_REVA_DMA_RX_FLUSH;
}

void MXC_SPI_RevA_ClearTXFIFO(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    spi->dma |= MXC_F_SPI_REVA_DMA_TX_FLUSH;
}

int MXC_SPI_RevA_SetRXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);

    if (numBytes > 32) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                 numBytes << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS);

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA_GetRXThreshold(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_THD_VAL) >> MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS;
}

int MXC_SPI_RevA_SetTXThreshold(mxc_spi_reva_regs_t *spi, unsigned int numBytes)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);

    if (numBytes > 32) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 numBytes << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS);

    return E_NO_ERROR;
}

unsigned int MXC_SPI_RevA_GetTXThreshold(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    return (spi->dma & MXC_F_SPI_REVA_DMA_TX_THD_VAL) >> MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS;
}

unsigned int MXC_SPI_RevA_GetFlags(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    return spi->intfl;
}

void MXC_SPI_RevA_ClearFlags(mxc_spi_reva_regs_t *spi)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    spi->intfl = spi->intfl;
}

void MXC_SPI_RevA_EnableInt(mxc_spi_reva_regs_t *spi, unsigned int intEn)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    spi->inten |= intEn;
}

void MXC_SPI_RevA_DisableInt(mxc_spi_reva_regs_t *spi, unsigned int intDis)
{
    MXC_ASSERT(MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi) >= 0);
    spi->inten &= ~(intDis);
}

int MXC_SPI_RevA_TransSetup(mxc_spi_reva_req_t *req)
{
    int spi_num;
    uint8_t bits;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));
    MXC_ASSERT(spi_num >= 0);
    MXC_ASSERT(req->ssIdx < MXC_SPI_SS_INSTANCES);

    if ((!req) || ((req->txData == NULL) && (req->rxData == NULL))) {
        return E_BAD_PARAM;
    }

    // Setup the number of characters to transact
    if (req->txLen > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_BAD_PARAM;
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);
    req->txCnt = 0;
    req->rxCnt = 0;

    states[spi_num].req = req;
    states[spi_num].started = 0;
    states[spi_num].req_done = 0;
    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    if (states[spi_num].ssDeassert == 1) {
        (req->spi)->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    }

    //if  master
    if ((req->spi)->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE) {
        // Setup the slave select
        MXC_SPI_SetSlave((mxc_spi_regs_t *)req->spi, req->ssIdx);
    }

    if (req->rxData != NULL && req->rxLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR,
                     req->rxLen << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_RX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
    }

    // Must use TXFIFO and NUM in full duplex//start  editing here
    if ((mxc_spi_reva_width_t)MXC_SPI_GetWidth((mxc_spi_regs_t *)req->spi) ==
            SPI_REVA_WIDTH_STANDARD &&
        !(((req->spi)->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) >>
          MXC_F_SPI_REVA_CTRL2_THREE_WIRE_POS)) {
        if (req->txData == NULL) {
            // Must have something to send, so we'll use the rx_data buffer initialized to 0.
            //SPI_SetDefaultTXData(spi, 0);
            memset(req->rxData, states[spi_num].defaultTXData,
                   (bits > 8 ? req->rxLen << 1 : req->rxLen));
            req->txData = req->rxData;
            req->txLen = req->rxLen;
        }
    }

    if (req->txData != NULL && req->txLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR,
                     req->txLen << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
    }

    if ((req->txData != NULL && req->txLen) && (req->rxData != NULL && req->rxLen)) {
        states[spi_num].txrx_req = true;
    } else {
        states[spi_num].txrx_req = false;
    }
    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);
    (req->spi)->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    states[spi_num].ssDeassert = req->ssDeassert;
    // Clear master done flag
    (req->spi)->intfl = MXC_F_SPI_REVA_INTFL_MST_DONE;

    return E_NO_ERROR;
}

uint32_t MXC_SPI_RevA_MasterTransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req)
{
    uint32_t retval;
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Leave slave select asserted at the end of the transaction
    if (!req->ssDeassert) {
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
    }

    int remain, spi_num;
    uint32_t int_en = 0;
    uint32_t tx_length = 0, rx_length = 0;
    uint8_t bits;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    //MXC_F_SPI_REVA_CTRL2_NUMBITS data bits
    // Read/write 2x number of bytes if larger character size
    if (bits > 8) {
        tx_length = req->txLen * 2;
        rx_length = req->rxLen * 2;
    } else {
        tx_length = req->txLen;
        rx_length = req->rxLen;
    }

    if (req->txData != NULL) {
        req->txCnt += MXC_SPI_WriteTXFIFO((mxc_spi_regs_t *)spi, &(req->txData[req->txCnt]),
                                          tx_length - req->txCnt);
    }

    remain = tx_length - req->txCnt;

    // Set the TX interrupts
    // Write the FIFO //starting here
    if (remain) {
        if (remain > MXC_SPI_FIFO_DEPTH) {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, MXC_SPI_FIFO_DEPTH);
        } else {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, remain);
        }

        int_en |= MXC_F_SPI_REVA_INTEN_TX_THD;
    }
    // Break out if we've transmitted all the bytes and not receiving
    if ((req->rxData == NULL) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    // Read the RX FIFO
    if (req->rxData != NULL) {
        req->rxCnt += MXC_SPI_ReadRXFIFO((mxc_spi_regs_t *)spi, &(req->rxData[req->rxCnt]),
                                         rx_length - req->rxCnt);

        remain = rx_length - req->rxCnt;

        if (remain) {
            if (remain > MXC_SPI_FIFO_DEPTH) {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, 2);
            } else {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, remain - 1);
            }

            int_en |= MXC_F_SPI_REVA_INTEN_RX_THD;
        }

        // Break out if we've received all the bytes and we're not transmitting
        if ((req->txData == NULL) && (req->rxCnt == rx_length)) {
            spi->inten = 0;
            int_en = 0;
            MXC_FreeLock((uint32_t *)&states[spi_num].req);

            // Callback if not NULL
            if (states[spi_num].async && req->completeCB != NULL) {
                req->completeCB(req, E_NO_ERROR);
            }
        }
    }
    // Break out once we've transmitted and received all of the data
    if ((req->rxCnt == rx_length) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    retval = int_en;

    if (!states[spi_num].started) {
        MXC_SPI_StartTransmission((mxc_spi_regs_t *)spi);
        states[spi_num].started = 1;
    }

    // Deassert slave select at the end of the transaction
    if (req->ssDeassert) {
        spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
    }

    return retval;
}

uint32_t MXC_SPI_RevA_SlaveTransHandler(mxc_spi_reva_req_t *req)
{
    return MXC_SPI_RevA_TransHandler(req->spi, req);
}

uint32_t MXC_SPI_RevA_TransHandler(mxc_spi_reva_regs_t *spi, mxc_spi_reva_req_t *req)
{
    int remain, spi_num;
    uint32_t int_en = 0;
    uint32_t tx_length = 0, rx_length = 0;
    uint8_t bits;
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    //MXC_F_SPI_REVA_CTRL2_NUMBITS data bits
    // Read/write 2x number of bytes if larger character size
    if (bits > 8) {
        tx_length = req->txLen * 2;
        rx_length = req->rxLen * 2;
    } else {
        tx_length = req->txLen;
        rx_length = req->rxLen;
    }

    if (req->txData != NULL) {
        req->txCnt += MXC_SPI_WriteTXFIFO((mxc_spi_regs_t *)spi, &(req->txData[req->txCnt]),
                                          tx_length - req->txCnt);
    }

    remain = tx_length - req->txCnt;

    // Set the TX interrupts
    // Write the FIFO //starting here
    if (remain) {
        if (remain > MXC_SPI_FIFO_DEPTH) {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, MXC_SPI_FIFO_DEPTH);
        } else {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, remain);
        }

        int_en |= MXC_F_SPI_REVA_INTEN_TX_THD;
    }
    // Break out if we've transmitted all the bytes and not receiving
    if ((req->rxData == NULL) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    // Read the RX FIFO
    if (req->rxData != NULL) {
        req->rxCnt += MXC_SPI_ReadRXFIFO((mxc_spi_regs_t *)spi, &(req->rxData[req->rxCnt]),
                                         rx_length - req->rxCnt);

        remain = rx_length - req->rxCnt;

        if (remain) {
            if (remain > MXC_SPI_FIFO_DEPTH) {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, 2);
            } else {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, remain - 1);
            }

            int_en |= MXC_F_SPI_REVA_INTEN_RX_THD;
        }

        // Break out if we've received all the bytes and we're not transmitting
        if ((req->txData == NULL) && (req->rxCnt == rx_length)) {
            spi->inten = 0;
            int_en = 0;
            MXC_FreeLock((uint32_t *)&states[spi_num].req);

            // Callback if not NULL
            if (states[spi_num].async && req->completeCB != NULL) {
                req->completeCB(req, E_NO_ERROR);
            }
        }
    }
    // Break out once we've transmitted and received all of the data
    if ((req->rxCnt == rx_length) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    return int_en;
}

int MXC_SPI_RevA_MasterTransaction(mxc_spi_reva_req_t *req)
{
    int error;

    int spi_num;
    uint8_t bits;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));
    MXC_ASSERT(spi_num >= 0);
    MXC_ASSERT(req->ssIdx < MXC_SPI_SS_INSTANCES);

    if ((!req) || ((req->txData == NULL) && (req->rxData == NULL))) {
        return E_BAD_PARAM;
    }

    // Setup the number of characters to transact
    if (req->txLen > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_BAD_PARAM;
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);
    req->txCnt = 0;
    req->rxCnt = 0;

    states[spi_num].req = req;
    states[spi_num].started = 0;
    states[spi_num].req_done = 0;
    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    if (states[spi_num].ssDeassert == 1) {
        (req->spi)->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    }

    //if  master
    if ((req->spi)->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE) {
        // Setup the slave select
        MXC_SPI_SetSlave((mxc_spi_regs_t *)req->spi, req->ssIdx);
    }

    if (req->rxData != NULL && req->rxLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR,
                     req->rxLen << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_RX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
    }

    // Must use TXFIFO and NUM in full duplex//start  editing here
    if ((mxc_spi_reva_width_t)MXC_SPI_GetWidth((mxc_spi_regs_t *)req->spi) ==
            SPI_REVA_WIDTH_STANDARD &&
        !(((req->spi)->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) >>
          MXC_F_SPI_REVA_CTRL2_THREE_WIRE_POS)) {
        if (req->txData == NULL) {
            // Must have something to send, so we'll use the rx_data buffer initialized to 0.
            //SPI_SetDefaultTXData(spi, 0);
            memset(req->rxData, states[spi_num].defaultTXData,
                   (bits > 8 ? req->rxLen << 1 : req->rxLen));
            req->txData = req->rxData;
            req->txLen = req->rxLen;
        }
    }

    // Tx
    if (req->txData != NULL && req->txLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR,
                     req->txLen << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
    }

    // Tx/Rx
    if ((req->txData != NULL && req->txLen) && (req->rxData != NULL && req->rxLen)) {
        states[spi_num].txrx_req = true;
    } else {
        states[spi_num].txrx_req = false;
    }
    
    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);
    (req->spi)->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    states[spi_num].ssDeassert = req->ssDeassert;
    // Clear master done flag
    (req->spi)->intfl = MXC_F_SPI_REVA_INTFL_MST_DONE;

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 0;

    //call master transHandler
    while (MXC_SPI_RevA_MasterTransHandler(req->spi, req) != 0) {}

    while (!((req->spi)->intfl & MXC_F_SPI_REVA_INTFL_MST_DONE)) {}

    return E_NO_ERROR;
}

int MXC_SPI_RevA_MasterTransactionAsync(mxc_spi_reva_req_t *req)
{
    int error;

    int spi_num;
    uint8_t bits;

    int int_en;

    int remain;
    uint32_t tx_length = 0, rx_length = 0;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));
    MXC_ASSERT(spi_num >= 0);
    MXC_ASSERT(req->ssIdx < MXC_SPI_SS_INSTANCES);

    if ((!req) || ((req->txData == NULL) && (req->rxData == NULL))) {
        return E_BAD_PARAM;
    }

    // Setup the number of characters to transact
    if (req->txLen > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_BAD_PARAM;
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);
    req->txCnt = 0;
    req->rxCnt = 0;

    states[spi_num].req = req;
    states[spi_num].started = 0;
    states[spi_num].req_done = 0;
    // HW requires disabling/renabling SPI block at end of each transaction (when SS is inactive).
    if (states[spi_num].ssDeassert == 1) {
        (req->spi)->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    }

    if (req->rxData != NULL && req->rxLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR,
                     req->rxLen << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_RX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
    }

    // Must use TXFIFO and NUM in full duplex//start  editing here
    if ((mxc_spi_reva_width_t)MXC_SPI_GetWidth((mxc_spi_regs_t *)req->spi) == SPI_REVA_WIDTH_STANDARD && !(((req->spi)->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) >> MXC_F_SPI_REVA_CTRL2_THREE_WIRE_POS)) {
        if (req->txData == NULL) {
            // Must have something to send, so we'll use the rx_data buffer initialized to 0.
            //SPI_SetDefaultTXData(spi, 0);
            memset(req->rxData, states[spi_num].defaultTXData, (bits > 8 ? req->rxLen << 1 : req->rxLen));
            req->txData = req->rxData;
            req->txLen = req->rxLen;
        }
    }

    if (req->txData != NULL && req->txLen > 0) {
        MXC_SETFIELD((req->spi)->ctrl1, MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR,
                     req->txLen << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        (req->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN;
    } else {
        (req->spi)->ctrl1 &= ~(MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR);
        (req->spi)->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
    }

    if ((req->txData != NULL && req->txLen) && (req->rxData != NULL && req->rxLen)) {
        states[spi_num].txrx_req = true;
    } else {
        states[spi_num].txrx_req = false;
    }
    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);
    (req->spi)->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    states[spi_num].ssDeassert = req->ssDeassert;
    // Clear master done flag
    (req->spi)->intfl = MXC_F_SPI_REVA_INTFL_MST_DONE;

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 1;

    // Leave slave select asserted at the end of the transaction
    if (!req->ssDeassert) {
        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
    }

    // TransHandler
    if (bits > 8) {
        tx_length = req->txLen * 2;
        rx_length = req->rxLen * 2;
    } else {
        tx_length = req->txLen;
        rx_length = req->rxLen;
    }

    if (req->txData != NULL) {
        req->txCnt += MXC_SPI_WriteTXFIFO((mxc_spi_regs_t *)spi, &(req->txData[req->txCnt]),
                                          tx_length - req->txCnt);
    }

    remain = tx_length - req->txCnt;

    // Set the TX interrupts
    // Write the FIFO //starting here
    if (remain) {
        if (remain > MXC_SPI_FIFO_DEPTH) {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, MXC_SPI_FIFO_DEPTH);
        } else {
            MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)spi, remain);
        }

        int_en |= MXC_F_SPI_REVA_INTEN_TX_THD;
    }

    // Break out if we've transmitted all the bytes and not receiving
    if ((req->rxData == NULL) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    // Read the RX FIFO
    if (req->rxData != NULL) {
        req->rxCnt += MXC_SPI_ReadRXFIFO((mxc_spi_regs_t *)spi, &(req->rxData[req->rxCnt]),
                                         rx_length - req->rxCnt);

        remain = rx_length - req->rxCnt;

        if (remain) {
            if (remain > MXC_SPI_FIFO_DEPTH) {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, 2);
            } else {
                MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)spi, remain - 1);
            }

            int_en |= MXC_F_SPI_REVA_INTEN_RX_THD;
        }

        // Break out if we've received all the bytes and we're not transmitting
        if ((req->txData == NULL) && (req->rxCnt == rx_length)) {
            spi->inten = 0;
            int_en = 0;
            MXC_FreeLock((uint32_t *)&states[spi_num].req);

            // Callback if not NULL
            if (states[spi_num].async && req->completeCB != NULL) {
                req->completeCB(req, E_NO_ERROR);
            }
        }
    }

    // Break out once we've transmitted and received all of the data
    if ((req->rxCnt == rx_length) && (req->txCnt == tx_length)) {
        spi->inten = 0;
        int_en = 0;
        MXC_FreeLock((uint32_t *)&states[spi_num].req);

        // Callback if not NULL
        if (states[spi_num].async && req->completeCB != NULL) {
            req->completeCB(req, E_NO_ERROR);
        }
    }

    if (!states[spi_num].started) {
        if (MXC_SPI_GetActive((mxc_spi_regs_t *)spi) == E_BUSY) {
            return E_BUSY;
        }

        spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

        states[spi_num].started = 1;
    }

    // Deassert slave select at the end of the transaction
    if (req->ssDeassert) {
        spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
    }

    MXC_SPI_EnableInt((mxc_spi_regs_t *)req->spi, int_en);

    return E_NO_ERROR;
}

int MXC_SPI_RevA_MasterTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                      mxc_dma_regs_t *dma)
{
    int spi_num;
    uint8_t error, bits;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advConfig = { 0, 0, 0, 0, 0, 0 };

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));
    MXC_ASSERT(spi_num >= 0);

    if (req->txData == NULL && req->rxData == NULL) {
        return E_BAD_PARAM;
    }

    if ((error = MXC_SPI_RevA_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    // for non-MT mode do this setup every time, for MT mode only first time
    if ((states[spi_num].mtMode == 0) ||
        ((states[spi_num].mtMode == 1) && (states[spi_num].mtFirstTrans == 1))) {
#if TARGET_NUM == 32665
        MXC_DMA_Init(dma);
        states[spi_num].channelTx = MXC_DMA_AcquireChannel(dma);
        states[spi_num].channelRx = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();
        states[spi_num].channelTx = MXC_DMA_AcquireChannel();
        states[spi_num].channelRx = MXC_DMA_AcquireChannel();
#endif

        if ((states[spi_num].channelTx < 0) || (states[spi_num].channelRx < 0)) {
            states[spi_num].channelTx = E_NO_DEVICE;
            states[spi_num].channelRx = E_NO_DEVICE;
            return E_NO_DEVICE;
        }

        states[spi_num].mtFirstTrans = 0;

        MXC_DMA_SetCallback(states[spi_num].channelTx, MXC_SPI_RevA_DMACallback);
        MXC_DMA_SetCallback(states[spi_num].channelRx, MXC_SPI_RevA_DMACallback);
        MXC_DMA_EnableInt(states[spi_num].channelTx);
        MXC_DMA_EnableInt(states[spi_num].channelRx);

        // Configure SS for per-transaction or always on
        if (req->ssDeassert) {
            req->spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            req->spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    MXC_SPI_RevA_TransHandler(req->spi, req);

    if (bits <= 8) {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 1); //set threshold to 1 byte
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0); //set threshold to 0 bytes
    } else {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 2);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    }

    //tx
    if (req->txData != NULL) {
        config.reqsel = reqselTx;
        config.ch = states[spi_num].channelTx;
        advConfig.ch = states[spi_num].channelTx;
        advConfig.burst_size = 2;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        config.srcinc_en = 1;
        config.dstinc_en = 0;

        srcdst.ch = states[spi_num].channelTx;
        srcdst.source = &(req->txData[req->txCnt]);

        if (bits > 8) {
            srcdst.len = (req->txLen * 2) - req->txCnt;
        } else {
            srcdst.len = (req->txLen) - req->txCnt;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelTx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelTx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 1 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    if (req->rxData != NULL) {
        config.reqsel = reqselRx;
        config.ch = states[spi_num].channelRx;
        config.srcinc_en = 0;
        config.dstinc_en = 1;
        advConfig.ch = states[spi_num].channelRx;
        advConfig.burst_size = 1;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        srcdst.ch = states[spi_num].channelRx;
        srcdst.dest = req->rxData;

        if (bits <= 8) {
            srcdst.len = req->rxLen;
        } else {
            srcdst.len = req->rxLen * 2;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelRx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelRx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 0 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);

    if (!states[spi_num].started) {
        MXC_SPI_StartTransmission((mxc_spi_regs_t *)req->spi);
        states[spi_num].started = 1;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA_SlaveTransaction(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 0;

    while (MXC_SPI_RevA_SlaveTransHandler(req) != 0) {}

    return E_NO_ERROR;
}

int MXC_SPI_RevA_SlaveTransactionAsync(mxc_spi_reva_req_t *req)
{
    int error;

    if ((error = MXC_SPI_RevA_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    states[MXC_SPI_GET_IDX((mxc_spi_regs_t *)req->spi)].async = 1;

    MXC_SPI_EnableInt((mxc_spi_regs_t *)req->spi, MXC_SPI_RevA_SlaveTransHandler(req));

    return E_NO_ERROR;
}

int MXC_SPI_RevA_SlaveTransactionDMA(mxc_spi_reva_req_t *req, int reqselTx, int reqselRx,
                                     mxc_dma_regs_t *dma)
{
    int spi_num;
    uint8_t error, bits;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    mxc_dma_adv_config_t advConfig = { 0, 0, 0, 0, 0, 0 };

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(req->spi));
    MXC_ASSERT(spi_num >= 0);

    if (req->txData == NULL && req->rxData == NULL) {
        return E_BAD_PARAM;
    }

    if ((error = MXC_SPI_RevA_TransSetup(req)) != E_NO_ERROR) {
        return error;
    }

    // for non-MT mode do this setup every time, for MT mode only first time
    if ((states[spi_num].mtMode == 0) ||
        ((states[spi_num].mtMode == 1) && (states[spi_num].mtFirstTrans == 1))) {
#if TARGET_NUM == 32665
        MXC_DMA_Init(dma);
        states[spi_num].channelTx = MXC_DMA_AcquireChannel(dma);
        states[spi_num].channelRx = MXC_DMA_AcquireChannel(dma);
#else
        MXC_DMA_Init();
        states[spi_num].channelTx = MXC_DMA_AcquireChannel();
        states[spi_num].channelRx = MXC_DMA_AcquireChannel();
#endif

        if ((states[spi_num].channelTx < 0) || (states[spi_num].channelRx < 0)) {
            states[spi_num].channelTx = E_NO_DEVICE;
            states[spi_num].channelRx = E_NO_DEVICE;
            return E_NO_DEVICE;
        }

        states[spi_num].mtFirstTrans = 0;

        MXC_DMA_SetCallback(states[spi_num].channelTx, MXC_SPI_RevA_DMACallback);
        MXC_DMA_SetCallback(states[spi_num].channelRx, MXC_SPI_RevA_DMACallback);
        MXC_DMA_EnableInt(states[spi_num].channelTx);
        MXC_DMA_EnableInt(states[spi_num].channelRx);
    }

    bits = MXC_SPI_GetDataSize((mxc_spi_regs_t *)req->spi);

    MXC_SPI_RevA_TransHandler(req->spi, req);

    if (bits <= 8) {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 1);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    } else {
        MXC_SPI_SetTXThreshold((mxc_spi_regs_t *)req->spi, 2);
        MXC_SPI_SetRXThreshold((mxc_spi_regs_t *)req->spi, 0);
    }

    //tx
    if (req->txData != NULL) {
        config.reqsel = reqselTx;
        config.ch = states[spi_num].channelTx;
        advConfig.ch = states[spi_num].channelTx;
        advConfig.burst_size = 2;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        config.srcinc_en = 1;
        config.dstinc_en = 0;

        srcdst.ch = states[spi_num].channelTx;
        srcdst.source = &(req->txData[req->txCnt]);

        if (bits > 8) {
            srcdst.len = (req->txLen * 2) - req->txCnt;
        } else {
            srcdst.len = (req->txLen) - req->txCnt;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelTx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelTx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 1 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    if (req->rxData != NULL) {
        config.reqsel = reqselRx;
        config.ch = states[spi_num].channelRx;
        config.srcinc_en = 0;
        config.dstinc_en = 1;
        advConfig.ch = states[spi_num].channelRx;
        advConfig.burst_size = 1;

        if (bits <= 8) {
            config.srcwd = MXC_DMA_WIDTH_BYTE;
            config.dstwd = MXC_DMA_WIDTH_BYTE;
        } else {
            config.srcwd = MXC_DMA_WIDTH_HALFWORD;
            config.dstwd = MXC_DMA_WIDTH_HALFWORD;
        }

        srcdst.ch = states[spi_num].channelRx;
        srcdst.dest = req->rxData;

        if (bits <= 8) {
            srcdst.len = req->rxLen;
        } else {
            srcdst.len = req->rxLen * 2;
        }

        MXC_DMA_ConfigChannel(config, srcdst);
        MXC_DMA_Start(states[spi_num].channelRx);
        MXC_DMA_SetChannelInterruptEn(states[spi_num].channelRx, false, true);
        //MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

        if (bits > 8) {
            MXC_DMA_AdvConfigChannel(advConfig);
            //MXC_SETFIELD (MXC_DMA->ch[channel].ctrl, MXC_F_DMA_CTRL_BURST_SIZE, 0 << MXC_F_DMA_CTRL_BURST_SIZE_POS);
        }
    }

    (req->spi)->dma |= (MXC_F_SPI_REVA_DMA_DMA_TX_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);

    return E_NO_ERROR;
}

void MXC_SPI_RevA_DMACallback(int ch, int error)
{
    mxc_spi_reva_req_t *temp_req;

    for (int i = 0; i < MXC_SPI_INSTANCES; i++) {
        if (states[i].req != NULL) {
            if (states[i].channelTx == ch) {
                states[i].req_done++;
            } else if (states[i].channelRx == ch) {
                states[i].req_done++;
                //save the request
                temp_req = states[i].req;

                if (MXC_SPI_GetDataSize((mxc_spi_regs_t *)temp_req->spi) > 8) {
                    MXC_SPI_RevA_SwapByte(temp_req->rxData, temp_req->rxLen);
                }
            }

            if (!states[i].txrx_req || (states[i].txrx_req && states[i].req_done == 2)) {
                //save the request
                temp_req = states[i].req;
                MXC_FreeLock((uint32_t *)&states[i].req);
                // Callback if not NULL
                if (temp_req->completeCB != NULL) {
                    temp_req->completeCB(temp_req, E_NO_ERROR);
                }
                if (states[i].mtMode == 0) {
                    // release any acquired DMA channels
                    if (states[i].channelTx >= 0) {
                        MXC_DMA_RevA_ReleaseChannel(states[i].channelTx);
                        states[i].channelTx = E_NO_DEVICE;
                    }
                    if (states[i].channelRx >= 0) {
                        MXC_DMA_RevA_ReleaseChannel(states[i].channelRx);
                        states[i].channelRx = E_NO_DEVICE;
                    }
                }
                break;
            }
        }
    }
}

int MXC_SPI_RevA_SetDefaultTXData(mxc_spi_reva_regs_t *spi, unsigned int defaultTXData)
{
    int spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);
    states[spi_num].defaultTXData = defaultTXData;
    return E_NO_ERROR;
}

void MXC_SPI_RevA_AbortAsync(mxc_spi_reva_regs_t *spi)
{
    MXC_SPI_AbortTransmission((mxc_spi_regs_t *)spi);
}

void MXC_SPI_RevA_AsyncHandler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    unsigned rx_avail;
    uint32_t flags;
    uint32_t int_en;

    // Clear the interrupt flags
    spi->inten = 0;
    flags = spi->intfl;
    spi->intfl = flags;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    // Figure out if this SPI has an active request
    if ((states[spi_num].req != NULL) && (flags)) {
        if ((spi->ctrl0 & MXC_F_SPI_REVA_CTRL0_MST_MODE) >> MXC_F_SPI_REVA_CTRL0_MST_MODE_POS) {
            do {
                // Leave slave select asserted at the end of the transaction
                if (!req->ssDeassert) {
                    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
                } else {
                    spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
                }

                int_en = MXC_SPI_RevA_TransHandler(spi, req);

                if (!states[spi_num].started) {
                    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

                    states[spi_num].started = 1;
                }

                spi->inten = int_en;

                rx_avail = MXC_SPI_RevA_GetRXFIFOAvailable(spi);
            } while (rx_avail > MXC_SPI_RevA_GetRXThreshold(spi));

        } else {
            do {
                spi->inten = MXC_SPI_RevA_SlaveTransHandler(states[spi_num].req);
                rx_avail = MXC_SPI_RevA_GetRXFIFOAvailable(spi);
            } while (rx_avail > MXC_SPI_RevA_GetRXThreshold(spi));
        }
    }
}

//call in DMA IRQHANDLER with rxData for transmissions with bits > 8
void MXC_SPI_RevA_SwapByte(uint8_t *arr, size_t length)
{
    MXC_ASSERT(arr != NULL);

    for (size_t i = 0; i < (length * 2); i += 2) {
        uint8_t tmp = arr[i];
        arr[i] = arr[i + 1];
        arr[i + 1] = tmp;
    }
}
