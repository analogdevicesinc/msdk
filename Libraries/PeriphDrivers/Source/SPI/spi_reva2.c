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
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "spi.h"
#include "spi_reva2.h"
#include "dma_reva.h"

/* **** Definitions **** */

// clang-format off
typedef struct {
    // Info from initialization.
    bool                initialized;
    bool                dma_initialized;
    mxc_spi_init_t      init;

    // Transaction Data.
    uint8_t            *tx_buffer;
    uint32_t            tx_len;         // Terms of bytes
    uint32_t            tx_cnt;         // Terms of bytes
    uint8_t            *rx_buffer;
    uint32_t            rx_len;         // Terms of bytes
    uint32_t            rx_cnt;
    uint16_t            tx_dummy_value;

    mxc_spi_callback_t  callback;
    void                *callback_data;

    // Chip Select Info.
    bool                deassert; // Target Select (TS) Deasserted at the end of a transmission.
    mxc_spi_target_t    current_target;

    // DMA Settings.
    mxc_dma_reva_regs_t *dma;
    int                 tx_dma_ch;
    int                 rx_dma_ch;

    // Status Fields.
    bool                controller_done; // Master
    bool                target_done;     // Slave
    bool                tx_done;
    bool                rx_done;
} mxc_spi_handle_data_t;
// clang-format on

static volatile mxc_spi_handle_data_t STATES[MXC_SPI_INSTANCES];

/* **** Private Functions **** */

// The unique title for Private functions will not be capitalized.

/** Private Function: writeTXFIFO16
 * Writes 2 bytes to the TX FIFO for 9-16 bit frame lengths.
 * This function helps package the frame when the STATES[n] fields
 * are all in terms of bytes.
 * 
 * @param   spi         Pointer to SPI instance.
 * @param   buffer      Pointer to buffer of messages to transmit.
 * @param   len_bytes   Number of messages (in terms of bytes) in buffer to transmit.
 * 
 * @return  cnt         The number of frames written to the TX FIFO.
 */
static uint32_t MXC_SPI_RevA2_writeTXFIFO16(mxc_spi_reva_regs_t *spi, uint8_t *buffer,
                                            uint32_t len_bytes)
{
    uint32_t tx_avail;
    int spi_num;
    uint32_t cnt = 0;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    if (buffer == NULL || len_bytes == 0) {
        return 0;
    }

    tx_avail = MXC_SPI_FIFO_DEPTH -
               ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);

    // Do not write more than the available FIFO size
    if (len_bytes > tx_avail) {
        len_bytes = tx_avail;
    }

    // Ensure even lengths for halfword frame lengths.
    // Note: Len is in terms of bytes, so sending 9-16bit transactions means sending
    //          2 bytes per frame.
    len_bytes &= ~0x01;

    while (len_bytes) {
        if (len_bytes > 3) {
            memcpy((void *)(&spi->fifo32), (uint8_t *)(&buffer[cnt]), 4);

            len_bytes -= 4;
            cnt += 4;

        } else if (len_bytes > 1) {
            memcpy((void *)(&spi->fifo16[0]), (uint8_t *)(&buffer[cnt]), 2);

            len_bytes -= 2;
            cnt += 2;
        }
    }

    return cnt;
}

/** Private Function: readRXFIFO16
 * Reads 2 bytes from the RX FIFO for 9-16 bit frame lengths.
 * This function helps package the frame when the STATES[n] fields
 * are all in terms of bytes.
 * 
 * @param   spi         Pointer to SPI instance.
 * @param   buffer      Pointer to buffer to store read messages.
 * @param   len_bytes   Number of messages (in terms of bytes) to store in receive buffer.
 * 
 * @return  cnt         The number of frames read from the RX FIFO
 */
static uint32_t MXC_SPI_RevA2_readRXFIFO16(mxc_spi_reva_regs_t *spi, uint8_t *buffer,
                                           uint32_t len_bytes)
{
    uint32_t rx_avail;
    int spi_num;
    uint32_t cnt = 0;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    if (buffer == NULL || len_bytes == 0) {
        return 0;
    }

    rx_avail = (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;

    // Do not read more than available frames in RX FIFO.
    if (len_bytes > rx_avail) {
        len_bytes = rx_avail;
    }

    // Ensure even lengths for halfword frame lengths.
    // Note: Len is in terms of bytes, so reading 9-16bit wide messages means reading
    //          2 bytes per frame.
    len_bytes &= ~0x01;

    if (len_bytes >= 2) {
        // Read from the FIFO
        while (len_bytes) {
            if (len_bytes > 3) {
                memcpy((uint8_t *)(&buffer[cnt]), (void *)(&spi->fifo32), 4);
                len_bytes -= 4;
                cnt += 4;

            } else if (len_bytes > 1) {
                memcpy((uint8_t *)(&buffer[cnt]), (void *)(&spi->fifo16[0]), 2);
                len_bytes -= 2;
                cnt += 2;
            }

            // Ensures read of less than 2 bytes aren't read.
            // Code should never get to this point.
            if (len_bytes == 1) {
                break;
            }
        }
    }

    return cnt;
}

/** Private Function: process
 * This function handles the reads and writes to the SPI RX/TX FIFO.
 * 
 * @param   spi     Pointer to SPI instance.
 */
static void MXC_SPI_RevA2_process(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    int remain;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Write any pending bytes out.
    //  Dependent on 1) Valid TX Buffer, 2) TX Length not 0, and 3) TX FIFO Not Empty.
    if (STATES[spi_num].tx_buffer && STATES[spi_num].tx_len > 0) {
        // Write to the FIFO for byte size transactions (message sizes for 8 bits or less)
        if (STATES[spi_num].init.frame_size <= 8) {
            while (((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS) <
                   (MXC_SPI_FIFO_DEPTH)) {
                spi->fifo8[0] = STATES[spi_num].tx_buffer[STATES[spi_num].tx_cnt];
                STATES[spi_num].tx_cnt += 1;
            }

            // Write to the FIFO for halfword size transactions (message sizes for 9 bits or greater)
        } else {
            STATES[spi_num].tx_cnt += MXC_SPI_RevA2_writeTXFIFO16(
                spi, &(STATES[spi_num].tx_buffer[STATES[spi_num].tx_cnt]),
                STATES[spi_num].tx_len - STATES[spi_num].tx_cnt);

            remain = STATES[spi_num].tx_len - STATES[spi_num].tx_cnt;

            if (remain) {
                if (remain >= MXC_SPI_FIFO_DEPTH) {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                                 ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
                } else {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                                 (remain << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
                }
            }
        }
    }

    if (STATES[spi_num].tx_cnt == STATES[spi_num].tx_len) {
        STATES[spi_num].tx_done = true;
    }

    // Unload any SPI data that has come in
    //  Dependent on 1) Valid RX Buffer, 2) RX Length not 0, and 3) RX FIFO Not Empty.
    if (STATES[spi_num].rx_buffer && STATES[spi_num].rx_len > 0) {
        // Read the FIFO for byte size transactions (message sizes for 8 bits or less)
        if (STATES[spi_num].init.frame_size <= 8) {
            while ((spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
                STATES[spi_num].rx_buffer[STATES[spi_num].rx_cnt] = spi->fifo8[0];
                STATES[spi_num].rx_cnt += 1;
            }

            // Read the FIFO for halfword size transactions (message sizes for 9 bits or greater)
        } else {
            STATES[spi_num].rx_cnt += MXC_SPI_RevA2_readRXFIFO16(
                spi, &(STATES[spi_num].rx_buffer[STATES[spi_num].rx_cnt]),
                STATES[spi_num].rx_len - STATES[spi_num].rx_cnt);

            remain = STATES[spi_num].rx_len - STATES[spi_num].rx_cnt;

            if (remain) {
                if (remain >= MXC_SPI_FIFO_DEPTH) {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                                 (2 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
                } else {
                    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                                 ((remain - 1) << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
                }
            }
        }
    }

    if (STATES[spi_num].rx_cnt == STATES[spi_num].rx_len) {
        STATES[spi_num].rx_done = true;
    }
}

/** Private Function: resetStateStruct
 * This functions resets the STATE of an SPI instance.
 * 
 * @param   spi_num     Index number of SPI instance.
 * 
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
static int MXC_SPI_RevA2_resetStateStruct(int spi_num)
{
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Init Data
    STATES[spi_num].initialized = false;
    STATES[spi_num].dma_initialized = false;
    STATES[spi_num].init = (const mxc_spi_init_t){ 0 };

    // Transaction Members
    STATES[spi_num].tx_buffer = NULL;
    STATES[spi_num].tx_len = 0;
    STATES[spi_num].tx_cnt = 0;
    STATES[spi_num].rx_buffer = NULL;
    STATES[spi_num].rx_len = 0;
    STATES[spi_num].rx_cnt = 0;
    STATES[spi_num].deassert =
        true; // Default state is TS will be deasserted at the end of a transmission.

    // DMA
    STATES[spi_num].dma = NULL;
    STATES[spi_num].tx_dma_ch = -1;
    STATES[spi_num].rx_dma_ch = -1;

    // Status Members
    STATES[spi_num].controller_done = false;
    STATES[spi_num].tx_done = false;
    STATES[spi_num].rx_done = false;
    STATES[spi_num].target_done = false;

    return E_NO_ERROR;
}

/* **** Public Functions **** */

int MXC_SPI_RevA2_Init(mxc_spi_init_t *init)
{
    int error, spi_num, i;
    int tx_ch, rx_ch;
    mxc_spi_target_t *target;
    mxc_gpio_regs_t *target_port;

    if (init == NULL) {
        return E_NULL_PTR;
    }

    // Ensure valid SPI instance.
    spi_num = MXC_SPI_GET_IDX(init->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // For code readability.
    target = &(init->target);
    if (target == NULL) {
        return E_NULL_PTR;
    }

    // Reset STATE of current SPI instance.
    error = MXC_SPI_RevA2_resetStateStruct(spi_num);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Save init data for transactions and handlers.
    STATES[spi_num].init = *init;
    STATES[spi_num].dma = NULL;
    STATES[spi_num].current_target = *target;

    // Set up Target Select Control Scheme.
    //  Hardware (Automatic) Controlled.
    if (init->ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
        // Set up preconfigured TSn Pins.
        //   Use target.init_mask for most the convenience.
        if (init->target.init_mask) {
            // Get total number of TSn instances for this SPI instance
            for (i = 0; i < MXC_SPI_GET_TOTAL_TS(init->spi); i++) {
                // Note: The [] represents the bit location of init_mask
                //       init_mask[0] <= Target Select 0 (TS0)
                //       init_mask[1] <= Target Select 1 (TS1)
                //       init_mask[n] <= Target Select n (TSn)
                if (init->target.init_mask & (1 << i)) {
                    error = MXC_SPI_ConfigTargetSelect(init->spi, i, init->vssel);
                    if (error != E_NO_ERROR) {
                        return error;
                    }
                }
            }

            MXC_SETFIELD((init->spi)->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE,
                         ((uint32_t)(init->target.init_mask)
                          << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS));

            // If target.init_mask was not used, then read the target settings and initalize the selected index.
            // Mainly used to test new HW TSn pins that aren't defined in the parts' mxc_pins.h and pins_{part}.c.
        } else {
            if (target->index >= MXC_SPI_GET_TOTAL_TS(init->spi)) {
                return E_BAD_PARAM;
            }

            error = MXC_SPI_ConfigTargetSelect(init->spi, target->index, init->vssel);
            if (error != E_NO_ERROR) {
                return error;
            }

            (init->spi)->ctrl0 |= ((1 << target->index) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS);

            // Set TS Polarity (Default - active low (0))
            if (target->active_polarity) {
                (init->spi)->ctrl2 |= ((1 << target->index) << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);
            } else {
                (init->spi)->ctrl2 &= ~((1 << target->index) << MXC_F_SPI_REVA_CTRL2_SS_POL_POS);
            }
        }

        //  Software Driver Controlled.
    } else if (init->ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
        // Readbility for register access
        target_port = target->pins.port;

        // If SPI driver is handling target, make sure the pin function is set as an output (AF: IO).
        if ((target->pins.port != NULL) && (target->pins.func == MXC_GPIO_FUNC_OUT)) {
            error = MXC_GPIO_Config(&(target->pins));
            if (error != E_NO_ERROR) {
                return error;
            }

            // Ensure VDDIO/VDDIOH Selection
            error = MXC_GPIO_SetVSSEL(target->pins.port, init->vssel, target->pins.mask);
            if (error != E_NO_ERROR) {
                return error;
            }

            // Set IDLE TS Polarity (Default - active low (0))
            if (target->active_polarity) {
                // Active HIGH (1), Set TS Idle State to LOW (0)
                target_port->out_clr |= target->pins.mask;
            } else {
                // Active LOW (0), Set TS Idle State to HIGH (1)
                target_port->out_set |= target->pins.mask;
            }

        } else {
            return E_BAD_PARAM;
        }

        // Don't do anything if SW Application is handling Target Select (TS) pin
        // while still checking for proper ts_control parameter.
        //  Software Application Controlled.
    } else if (init->ts_control != MXC_SPI_TSCONTROL_SW_APP) {
        // Not a valid Target Select (TS) Control option.
        return E_BAD_PARAM;
    }

    // Enable SPI port.
    (init->spi)->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    (init->spi)->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    // Select Controller (L. Master) or Target (L. Slave) Mode.
    if (init->type == MXC_SPI_TYPE_CONTROLLER) {
        (init->spi)->ctrl0 |= MXC_F_SPI_REVA_CTRL0_MST_MODE;
    } else {
        (init->spi)->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_MST_MODE;
    }

    // Set frame size.
    if (init->frame_size <= 1 || init->frame_size > 16) {
        return E_BAD_PARAM;
    } else {
        (init->spi)->ctrl2 |= (init->frame_size) << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS;
    }

    // Remove any delay between TS (L. SS) and SCLK edges.
    (init->spi)->sstime = (1 << MXC_F_SPI_REVA_SSTIME_PRE_POS) |
                          (1 << MXC_F_SPI_REVA_SSTIME_POST_POS) |
                          (1 << MXC_F_SPI_REVA_SSTIME_INACT_POS);

    // Enable TX/RX FIFOs
    (init->spi)->dma |= MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN;

    // Set TX and RX Threshold to (FIFO_DEPTH - 1) and (0), respectively.
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
    MXC_SETFIELD((init->spi)->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                 (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

    // Set Clock Mode (CPOL and CPHA).
    error = MXC_SPI_SetClkMode((init->spi), (init->mode));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Interface mode: 3-wire, standard (4-wire), dual, quad.
    error = MXC_SPI_SetInterface((init->spi), (init->mode));
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_SPI_SetFrequency((init->spi), (init->freq));
    if (error != E_NO_ERROR) {
        return error;
    }

    // Clear any interrupt flags that may already be set.
    (init->spi)->intfl = (init->spi)->intfl;
    (init->spi)->inten = 0;

    if (init->use_dma == false) {
        // Enable Controller Done Interrupt.
        (init->spi)->inten |= MXC_F_SPI_REVA_INTEN_MST_DONE;
    }

    // Set callback.
    STATES[spi_num].callback = init->callback;
    STATES[spi_num].callback_data = init->callback_data;

    // Setup DMA features if used.
    if (init->use_dma) {
        // Even though the Init Struct has a pointer to the DMA instance,
        //   this will make the code a bit more readable since the DMA
        //   instance is now type casted with the DMA RevA Registers.
        STATES[spi_num].dma = (mxc_dma_reva_regs_t *)(init->dma);

#if (MXC_DMA_INSTANCES == 1)
        error = MXC_DMA_Init();
#else
        error = MXC_DMA_Init(init->dma);
#endif
        if (error != E_NO_ERROR) {
            return error;
        }

        // Acquire DMA Channels for SPI TX/RX
        STATES[spi_num].tx_dma_ch = MXC_DMA_AcquireChannel();
        STATES[spi_num].rx_dma_ch = MXC_DMA_AcquireChannel();

        tx_ch = STATES[spi_num].tx_dma_ch;
        rx_ch = STATES[spi_num].rx_dma_ch;

        // Check if failed to acquire channel.
        if (STATES[spi_num].tx_dma_ch < 0 || STATES[spi_num].rx_dma_ch < 0) {
            return E_NONE_AVAIL;
        }

        // TX Channel
        STATES[spi_num].dma->ch[tx_ch].ctrl |=
            (MXC_F_DMA_REVA_CTRL_CTZ_IE); // | MXC_F_DMA_REVA_CTRL_DIS_IE);
        STATES[spi_num].dma->inten |= (1 << tx_ch);

        // RX Channel
        STATES[spi_num].dma->ch[rx_ch].ctrl |=
            (MXC_F_DMA_REVA_CTRL_CTZ_IE); // | MXC_F_DMA_REVA_CTRL_DIS_IE);
        STATES[spi_num].dma->inten |= (1 << rx_ch);

        STATES[spi_num].dma_initialized = true;
    }

    // If successful, mark STATE of this SPI instance as initialized.
    STATES[spi_num].initialized = true;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_Shutdown(mxc_spi_reva_regs_t *spi)
{
    int spi_num, error;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Disable and clear interrupts.
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Disable SPI and FIFOS
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

    // Clear registers.
    spi->ctrl0 = 0;
    spi->ctrl1 = 0;
    spi->ctrl2 = 0;
    spi->sstime = 0;

    // Release any acquired DMA channels.
    if (STATES[spi_num].tx_dma_ch >= 0) {
        MXC_DMA_ReleaseChannel(STATES[spi_num].tx_dma_ch);
        STATES[spi_num].tx_dma_ch = E_NO_DEVICE;
    }
    if (STATES[spi_num].rx_dma_ch >= 0) {
        MXC_DMA_ReleaseChannel(STATES[spi_num].rx_dma_ch);
        STATES[spi_num].rx_dma_ch = E_NO_DEVICE;
    }

    if (STATES[spi_num].init.use_dma) {
        MXC_DMA_DeInit();
    }

    // Reset the SPI instance's STATE when shutting down.
    error = MXC_SPI_RevA2_resetStateStruct(spi_num);
    if (error != E_NO_ERROR) {
        return error;
    }

    return E_NO_ERROR;
}

uint32_t MXC_SPI_RevA2_GetFlags(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    MXC_ASSERT(spi_num >= 0);

    return spi->intfl;
}

void MXC_SPI_RevA2_ClearFlags(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    MXC_ASSERT(spi_num >= 0);

    spi->intfl = spi->intfl;
}

void MXC_SPI_RevA2_EnableInt(mxc_spi_reva_regs_t *spi, uint32_t en)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    MXC_ASSERT(spi_num >= 0);

    spi->inten |= en;
}

void MXC_SPI_RevA2_DisableInt(mxc_spi_reva_regs_t *spi, uint32_t dis)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);

    MXC_ASSERT(spi_num >= 0);

    spi->inten &= ~(dis);
}

int MXC_SPI_RevA2_SetFrequency(mxc_spi_reva_regs_t *spi, uint32_t freq)
{
    int hi_clk, lo_clk, scale;
    uint32_t freq_div;

    // Check if frequency is too high
    if (freq > PeripheralClock) {
        return E_BAD_PARAM;
    }

    // Set the clock high and low
    freq_div = MXC_SPI_GetPeripheralClock((mxc_spi_regs_t *)spi);
    freq_div = (freq_div / freq);

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

int MXC_SPI_RevA2_GetFrequency(mxc_spi_reva_regs_t *spi)
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

int MXC_SPI_RevA2_SetFrameSize(mxc_spi_reva_regs_t *spi, int frame_size)
{
    int spi_num;
    int saved_enable_state;

    // HW has problem with these two character sizes
    if (frame_size <= 1 || frame_size > 16) {
        return E_BAD_PARAM;
    }

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Set up the character size.
    if (!(spi->stat & MXC_F_SPI_REVA_STAT_BUSY)) {
        saved_enable_state = spi->ctrl0 | MXC_F_SPI_REVA_CTRL0_EN;

        // If enabled, disable SPI before changing character size.
        if (saved_enable_state) {
            spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
        }

        // Update data size from save Init function.
        STATES[spi_num].init.frame_size = frame_size;

        if (frame_size < 16) {
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         frame_size << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        } else {
            // Set to 16 bits per character as default.
            MXC_SETFIELD(spi->ctrl2, MXC_F_SPI_REVA_CTRL2_NUMBITS,
                         0 << MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
        }

        // Return back to original SPI enable state.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_EN, saved_enable_state);
    } else {
        return E_BAD_STATE;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_GetFrameSize(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // NUMBITS = 0 means 16-bits per character
    if (!(spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS)) {
        return 16;
    } else {
        return ((spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_NUMBITS) >> MXC_F_SPI_REVA_CTRL2_NUMBITS_POS);
    }
}

int MXC_SPI_RevA2_SetInterface(mxc_spi_reva_regs_t *spi, mxc_spi_interface_t mode)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Clear before setting
    spi->ctrl2 &= ~(MXC_F_SPI_REVA_CTRL2_THREE_WIRE | MXC_F_SPI_REVA_CTRL2_DATA_WIDTH);

    switch (mode) {
    case MXC_SPI_INTERFACE_3WIRE:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;

    case MXC_SPI_INTERFACE_STANDARD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_MONO;
        break;

    case MXC_SPI_INTERFACE_DUAL:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL;
        break;

    case MXC_SPI_INTERFACE_QUAD:
        spi->ctrl2 |= MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD;
        break;

    // Default set to to 3-Wire
    default:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_THREE_WIRE;
        break;
    }

    // Save state of new mode
    STATES[spi_num].init.mode = mode;

    return E_NO_ERROR;
}

mxc_spi_interface_t MXC_SPI_RevA2_GetInterface(mxc_spi_reva_regs_t *spi)
{
    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_THREE_WIRE) {
        return MXC_SPI_INTERFACE_3WIRE;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_DUAL) {
        return MXC_SPI_INTERFACE_DUAL;
    }

    if (spi->ctrl2 & MXC_S_SPI_REVA_CTRL2_DATA_WIDTH_QUAD) {
        return MXC_SPI_INTERFACE_QUAD;
    }

    return MXC_SPI_INTERFACE_STANDARD;
}

int MXC_SPI_RevA2_SetClkMode(mxc_spi_reva_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    switch (clk_mode) {
    // CPOL: 0    CPHA: 0
    case MXC_SPI_CLKMODE_0:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 0    CPHA: 1
    case MXC_SPI_CLKMODE_1:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 1    CPHA: 0
    case MXC_SPI_CLKMODE_2:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // CPOL: 1    CPHA: 1
    case MXC_SPI_CLKMODE_3:
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 |= MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;

    // Mode 0 by default.
    default:
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPHA;
        spi->ctrl2 &= ~MXC_F_SPI_REVA_CTRL2_CLKPOL;
        break;
    }

    // Save state of new clock mode.
    STATES[spi_num].init.clk_mode = clk_mode;

    return E_NO_ERROR;
}

mxc_spi_clkmode_t MXC_SPI_RevA2_GetClkMode(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPHA) {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return MXC_SPI_CLKMODE_3;
        } else {
            return MXC_SPI_CLKMODE_2;
        }
    } else {
        if (spi->ctrl2 & MXC_F_SPI_REVA_CTRL2_CLKPOL) {
            return MXC_SPI_CLKMODE_1;
        }
    }

    return MXC_SPI_CLKMODE_0;
}

int MXC_SPI_RevA2_SetCallback(mxc_spi_reva_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

    STATES[spi_num].callback = callback;
    STATES[spi_num].callback_data = data;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_SetInitStruct(mxc_spi_reva_regs_t *spi, mxc_spi_init_t *init)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Make sure SPI instance is initialized
    if (STATES[spi_num].initialized != true) {
        return E_BAD_STATE;
    }

    STATES[spi_num].init = *init;
    STATES[spi_num].dma = (mxc_dma_reva_regs_t *)(init->dma);

    return E_NO_ERROR;
}

mxc_spi_init_t MXC_SPI_RevA2_GetInitStruct(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Make sure SPI instance is initialized
    MXC_ASSERT(STATES[spi_num].initialized == true);

    return (STATES[spi_num].init);
}

int MXC_SPI_RevA2_GetActive(mxc_spi_reva_regs_t *spi)
{
    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY) {
        return E_BUSY;
    }

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_ReadyForSleep(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (spi->stat & MXC_F_SPI_REVA_STAT_BUSY || (spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) ||
        (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL)) {
        return E_BUSY;
    } else {
        return E_NO_ERROR;
    }
}

int MXC_SPI_RevA2_SetDummyTX(mxc_spi_reva_regs_t *spi, uint16_t tx_value)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    STATES[spi_num].tx_dummy_value = tx_value;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_StartTransmission(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (MXC_SPI_GetActive((mxc_spi_regs_t *)spi) == E_BUSY) {
        return E_BUSY;
    }

    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_AbortTransmission(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Disable interrupts, clear the flags.
    spi->inten = 0;
    spi->intfl = spi->intfl;

    // Cancel on-going transaction before enabling.
    spi->ctrl0 &= ~(MXC_F_SPI_REVA_CTRL0_EN);
    spi->ctrl0 |= (MXC_F_SPI_REVA_CTRL0_EN);

    // Callback if not NULL
    if (STATES[spi_num].callback != NULL) {
        STATES[spi_num].callback(STATES[spi_num].callback_data, E_ABORT);
    }

    return E_NO_ERROR;
}

uint8_t MXC_SPI_RevA2_GetTXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    return MXC_SPI_FIFO_DEPTH -
           ((spi->dma & MXC_F_SPI_REVA_DMA_TX_LVL) >> MXC_F_SPI_REVA_DMA_TX_LVL_POS);
}

uint8_t MXC_SPI_RevA2_GetRXFIFOAvailable(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_LVL) >> MXC_F_SPI_REVA_DMA_RX_LVL_POS;
}

int MXC_SPI_RevA2_ClearTXFIFO(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t save_state;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    save_state = (spi->dma & (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN));

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH);

    // Revert to previous state.
    MXC_SETFIELD(spi->dma, (MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN),
                 save_state);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_ClearRXFIFO(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t save_state;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    save_state = (spi->dma & (MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN));

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Revert to previous state.
    MXC_SETFIELD(spi->dma, (MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN),
                 save_state);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_SetTXThreshold(mxc_spi_reva_regs_t *spi, uint8_t thd_val)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Valid values for the threshold are 0x1 to 0x1F
    if (thd_val > (MXC_SPI_FIFO_DEPTH - 1) || thd_val == 0) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 thd_val << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS);

    return E_NO_ERROR;
}

int MXC_SPI_RevA2_SetRXThreshold(mxc_spi_reva_regs_t *spi, uint8_t thd_val)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (thd_val >= (MXC_SPI_FIFO_DEPTH - 1)) {
        return E_BAD_PARAM;
    }

    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                 thd_val << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS);

    return E_NO_ERROR;
}

uint8_t MXC_SPI_RevA2_GetTXThreshold(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    return (spi->dma & MXC_F_SPI_REVA_DMA_TX_THD_VAL) >> MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS;
}

uint8_t MXC_SPI_RevA2_GetRXThreshold(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    return (spi->dma & MXC_F_SPI_REVA_DMA_RX_THD_VAL) >> MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS;
}

/* ** DMA-Specific Functions ** */

// Available for switching between DMA and non-DMA transactions
int MXC_SPI_RevA2_DMA_Init(mxc_spi_init_t *init)
{
    int error, spi_num;
    int tx_ch, rx_ch;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)(init->spi));
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (init == NULL) {
        return E_NULL_PTR;
    }

    if (init->dma == NULL || init->use_dma == false) {
        return E_BAD_PARAM;
    }

    // Exit function is DMA already initialized.
    if (MXC_SPI_RevA2_DMA_GetInitialized((mxc_spi_reva_regs_t *)(init->spi))) {
        return E_NO_ERROR;
    }

    // Even though the Init Struct has a pointer to the DMA instance,
    //   this will make the code a bit more readable since the DMA
    //   instance is now type casted with the DMA RevA Registers.
    STATES[spi_num].dma = (mxc_dma_reva_regs_t *)(init->dma);

#if (MXC_DMA_INSTANCES == 1)
    error = MXC_DMA_Init();
#else
    error = MXC_DMA_Init(init->dma);
#endif
    if (error != E_NO_ERROR) {
        return error;
    }

    // Acquire DMA Channels for SPI TX/RX
    STATES[spi_num].tx_dma_ch = MXC_DMA_AcquireChannel();
    STATES[spi_num].rx_dma_ch = MXC_DMA_AcquireChannel();

    tx_ch = STATES[spi_num].tx_dma_ch;
    rx_ch = STATES[spi_num].rx_dma_ch;

    // Check if failed to acquire channel.
    if (STATES[spi_num].tx_dma_ch < 0 || STATES[spi_num].rx_dma_ch < 0) {
        return E_NONE_AVAIL;
    }

    // TX Channel
    STATES[spi_num].dma->ch[tx_ch].ctrl |= (MXC_F_DMA_REVA_CTRL_CTZ_IE);
    STATES[spi_num].dma->inten |= (1 << tx_ch);

    // RX Channel
    STATES[spi_num].dma->ch[rx_ch].ctrl |= (MXC_F_DMA_REVA_CTRL_CTZ_IE);
    STATES[spi_num].dma->inten |= (1 << rx_ch);

    STATES[spi_num].dma_initialized = true;

    return E_NO_ERROR;
}

// Available to chech whether DMA is already initialized for SPI instance.
//      Useful for switching from non-DMA to DMA transactions.
bool MXC_SPI_RevA2_DMA_GetInitialized(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].dma_initialized);
}

int MXC_SPI_RevA2_DMA_GetTXChannel(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].tx_dma_ch);
}

int MXC_SPI_RevA2_DMA_GetRXChannel(mxc_spi_reva_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    return (STATES[spi_num].rx_dma_ch);
}

int MXC_SPI_RevA2_DMA_SetRequestSelect(mxc_spi_reva_regs_t *spi, uint32_t tx_reqsel,
                                       uint32_t rx_reqsel)
{
    int spi_num;
    uint32_t tx_ch;
    uint32_t rx_ch;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Ensure DMA was configured before setting DMA Request Selects.
    if (STATES[spi_num].dma == NULL) {
        return E_BAD_STATE;
    }

    tx_ch = STATES[spi_num].tx_dma_ch;
    rx_ch = STATES[spi_num].rx_dma_ch;

    // This function will overwrite the current DMA TX/RX Request Selects.
    if (tx_reqsel != -1) {
        STATES[spi_num].dma->ch[tx_ch].ctrl |= tx_reqsel;
    }

    if (rx_reqsel != -1) {
        STATES[spi_num].dma->ch[rx_ch].ctrl |= rx_reqsel;
    }

    return E_NO_ERROR;
}

// Swaps the upper and lower byte of 2-byte wide frame transactions.
// HW Bug: For 2-byte wide frame transactions, RX DMA swaps the
//      LSB and MSB.
// Example: TX: 0x1234 => RX: 0x3412
// Note: Use __REV assembly instruction for quicker Swap implementation.
void MXC_SPI_RevA2_DMA_SwapByte(uint8_t *buffer, uint32_t len_bytes)
{
    int i;

    MXC_ASSERT(buffer != NULL);

    for (i = 0; i < len_bytes; i += 2) {
        uint8_t temp = buffer[i];
        buffer[i] = buffer[i + 1];
        buffer[i + 1] = temp;
    }
}

/* ** Transaction Functions ** */

int MXC_SPI_RevA2_ControllerTransaction(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                        uint32_t tx_fr_len, uint8_t *rx_buffer, uint32_t rx_fr_len,
                                        uint8_t deassert, mxc_spi_target_t *target)
{
    int spi_num, tx_dummy_fr_len;

    // Ensure valid SPI Instance.
    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Ensure valid chip select option.
    if (target == NULL) {
        return E_NULL_PTR;
    }

    // Make sure SPI Instance was initialized.
    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

    // Make sure DMA is not initialized.
    if (STATES[spi_num].init.use_dma == true) {
        return E_BAD_STATE;
    }

    // Initialize SPIn state to handle data.
    STATES[spi_num].controller_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_cnt = 0;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].rx_cnt = 0;
    STATES[spi_num].rx_done = false;

    // Max number of frames to transmit/receive.
    if (tx_fr_len > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_OVERFLOW;
    }

    if (rx_fr_len > (MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS)) {
        return E_OVERFLOW;
    }

    // STATES[n] TX/RX Length Fields are in terms of number of bytes to send/receive.
    if (STATES[spi_num].init.frame_size <= 8) {
        STATES[spi_num].tx_len = tx_fr_len;
        STATES[spi_num].rx_len = rx_fr_len;
    } else {
        STATES[spi_num].tx_len = tx_fr_len * 2;
        STATES[spi_num].rx_len = rx_fr_len * 2;
    }

    STATES[spi_num].deassert = deassert;
    STATES[spi_num].current_target = *target;

    // Set the number of messages to transmit/receive for the SPI transaction.
    if (STATES[spi_num].init.mode == MXC_SPI_INTERFACE_STANDARD) {
        if (rx_fr_len > tx_fr_len) {
            // In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            // The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            // because the hardware always assume full duplex. Therefore extra
            // dummy bytes must be transmitted to support half duplex.
            tx_dummy_fr_len = rx_fr_len - tx_fr_len;

            // Check whether new frame length exceeds the possible number of frames to transmit.
            if ((tx_fr_len + tx_dummy_fr_len) >
                (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
                return E_OVERFLOW;
            }

            spi->ctrl1 = ((tx_fr_len + tx_dummy_fr_len) << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        } else {
            spi->ctrl1 = (tx_fr_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        }
    } else { // mode != MXC_SPI_INTERFACE_STANDARD
        spi->ctrl1 = (tx_fr_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_fr_len << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN |
                  MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    if (tx_fr_len > 0) {
        // Enable TX FIFO & TX Threshold crossed interrupt.
        spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN);
        spi->inten |= MXC_F_SPI_REVA_INTEN_TX_THD;

        // Set TX Threshold to minimum value after re-enabling TX FIFO.
        MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                     (1 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
    }

    if (rx_fr_len > 0) {
        // Enable RX FIFO & RX Threshold crossed interrupt.
        spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN);
        spi->inten |= MXC_F_SPI_REVA_INTEN_RX_THD;

        // Set RX Threshold to minimum value after re-enabling RX FIFO.
        MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL,
                     (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));
    }

    // This private function, MXC_SPI_RevA2_process, call fills the TX FIFO as much as possible
    //   before launching the transaction. Subsequent FIFO management will
    //   be handled from the MXC_SPI_Handler which should be called in SPI_IRQHandler.
    MXC_SPI_RevA2_process(spi);

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle target-select (L. SS) deassertion if HW is selected as Target Select (TS) Control Scheme. This must be done
    //   AFTER launching the transaction to avoid a glitch on the TS line if:
    //     - The TS line is asserted
    //     - We want to deassert the line as part of this transaction
    //
    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the Target Select (TS) pin based
    //   on the value of CTRL->SS_CTRL, which causes the glitch.
    if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
        // In HW Auto Scheme, only use the target index member.
        // Limitation: This implemention only support transactions with one target at a time.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE,
                     ((1 << target->index) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS));

        if (deassert) {
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }

        // Toggle Chip Select Pin if handled by the driver.
    } else if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
        // Make sure the selected Target Select (L. SS) pin is enabled as an output.
        if (target->pins.func != MXC_GPIO_FUNC_OUT) {
            return E_BAD_STATE;
        }

        // Active HIGH (1).
        if (target->active_polarity) {
            target->pins.port->out_set |= target->pins.mask;
            // Active LOW (0).
        } else {
            target->pins.port->out_clr |= target->pins.mask;
        }
    }

    return E_SUCCESS;
}

int MXC_SPI_RevA2_ControllerTransactionB(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                         uint32_t tx_fr_len, uint8_t *rx_buffer, uint32_t rx_fr_len,
                                         uint8_t deassert, mxc_spi_target_t *target)
{
    int error;
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevA2_ControllerTransaction(spi, tx_buffer, tx_fr_len, rx_buffer, rx_fr_len,
                                                deassert, target);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while (STATES[spi_num].controller_done == false) {}

    return E_SUCCESS;
}

int MXC_SPI_RevA2_ControllerTransactionDMA(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                           uint32_t tx_fr_len, uint8_t *rx_buffer,
                                           uint32_t rx_fr_len, uint8_t deassert,
                                           mxc_spi_target_t *target)
{
    int spi_num, tx_dummy_fr_len;
    // For readability purposes.
    int rx_ch, tx_ch;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Make sure DMA is initialized.
    if (STATES[spi_num].init.use_dma == false || STATES[spi_num].dma_initialized == false) {
        return E_BAD_STATE;
    }

    // Make sure SPI Instance was initialized.
    if (STATES[spi_num].initialized == false) {
        return E_BAD_STATE;
    }

    // Initialize SPIn state to handle DMA transactions.
    STATES[spi_num].controller_done = false;

    STATES[spi_num].tx_buffer = tx_buffer;
    STATES[spi_num].tx_done = false;

    STATES[spi_num].rx_buffer = rx_buffer;
    STATES[spi_num].rx_done = false;

    // Max number of frames to transmit/receive.
    if (tx_fr_len > (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
        return E_OVERFLOW;
    }

    if (rx_fr_len > (MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS)) {
        return E_OVERFLOW;
    }

    // STATES[n] TX/RX Length Fields are in terms of number of bytes to send/receive.
    if (STATES[spi_num].init.frame_size <= 8) {
        STATES[spi_num].tx_len = tx_fr_len;
        STATES[spi_num].rx_len = rx_fr_len;
    } else {
        STATES[spi_num].tx_len = tx_fr_len * 2;
        STATES[spi_num].rx_len = rx_fr_len * 2;
    }

    STATES[spi_num].deassert = deassert;
    STATES[spi_num].current_target = *target;

    // Set the number of bytes to transmit/receive for the SPI transaction.
    if (STATES[spi_num].init.mode == MXC_SPI_INTERFACE_STANDARD) {
        if (rx_fr_len > tx_fr_len) {
            // In standard 4-wire mode, the RX_NUM_CHAR field of ctrl1 is ignored.
            //  The number of bytes to transmit AND receive is set by TX_NUM_CHAR,
            //  because the hardware always assume full duplex. Therefore extra
            //  dummy bytes must be transmitted to support half duplex.
            tx_dummy_fr_len = rx_fr_len - tx_fr_len;

            // Check whether new frame length exceeds the possible number of frames to transmit.
            if ((tx_fr_len + tx_dummy_fr_len) >
                (MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR >> MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS)) {
                return E_OVERFLOW;
            }

            spi->ctrl1 = ((tx_fr_len + tx_dummy_fr_len) << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        } else {
            spi->ctrl1 = (tx_fr_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS);
        }
    } else { // mode != MXC_SPI_INTE_STANDARD
        spi->ctrl1 = (tx_fr_len << MXC_F_SPI_REVA_CTRL1_TX_NUM_CHAR_POS) |
                     (rx_fr_len << MXC_F_SPI_REVA_CTRL1_RX_NUM_CHAR_POS);
    }

    // Disable FIFOs before clearing as recommended by UG.
    spi->dma &= ~(MXC_F_SPI_REVA_DMA_TX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_TX_EN |
                  MXC_F_SPI_REVA_DMA_RX_FIFO_EN | MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FLUSH | MXC_F_SPI_REVA_DMA_RX_FLUSH);

    // Enable TX FIFO before configuring.
    spi->dma |= (MXC_F_SPI_REVA_DMA_TX_FIFO_EN);

    // Set TX and RX Thresholds before loading FIFO.
    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                 ((MXC_SPI_FIFO_DEPTH - 1) << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));
    MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_RX_THD_VAL, (0 << MXC_F_SPI_REVA_DMA_RX_THD_VAL_POS));

    // Set up DMA TX Transactions.
    // 1) For TX transmissions.
    if (tx_fr_len > 1) {
        // For readability purposes.
        tx_ch = STATES[spi_num].tx_dma_ch;

        // Configure DMA TX depending on frame width.
        // 2-8 bit wide frames.
        if (STATES[spi_num].init.frame_size <= 8) {
            // Hardware requires writing the first byte into the FIFO manually.
            spi->fifo8[0] = tx_buffer[0];

            // Threshold set to 1 frame (1 byte) after pre-loading first byte for DMA.
            MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                         (1 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));

            STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(tx_buffer + 1); // 1 Byte offset
            STATES[spi_num].dma->ch[tx_ch].cnt = (tx_fr_len - 1);

            // Set to one byte burst size.
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                         (0 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

            // Set source and destination width to one byte.
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                         MXC_S_DMA_REVA_CTRL_SRCWD_BYTE);
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                         MXC_S_DMA_REVA_CTRL_DSTWD_BYTE);

            // 9-16 bit wide frames.
        } else {
            // Hardware requires writing the first bytes into the FIFO manually.
            STATES[spi_num].tx_cnt += MXC_SPI_RevA2_writeTXFIFO16(
                spi, (uint8_t *)(STATES[spi_num].tx_buffer), STATES[spi_num].tx_len);

            // Threshold set to 1 frame (2 bytes) after pre-loading FIFO for DMA.
            MXC_SETFIELD(spi->dma, MXC_F_SPI_REVA_DMA_TX_THD_VAL,
                         (2 << MXC_F_SPI_REVA_DMA_TX_THD_VAL_POS));

            STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(tx_buffer + STATES[spi_num].tx_cnt);
            STATES[spi_num].dma->ch[tx_ch].cnt = (STATES[spi_num].tx_len - STATES[spi_num].tx_cnt);

            // Set to two byte burst size.
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                         (1 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

            // Set source and destination width to two bytes.
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                         MXC_S_DMA_REVA_CTRL_SRCWD_HALFWORD);
            MXC_SETFIELD(STATES[spi_num].dma->ch[tx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                         MXC_S_DMA_REVA_CTRL_DSTWD_HALFWORD);
        }

        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA

        // 2) For single character transmissions.
        //    NOTE: Single-length transmissions does not trigger CTZ.
    } else if (tx_fr_len == 1) {
        // Write first frame into FIFO.
        if (STATES[spi_num].init.frame_size <= 8) {
            spi->fifo8[0] = tx_buffer[0];
        } else {
            MXC_SPI_RevA2_writeTXFIFO16(spi, (uint8_t *)(STATES[spi_num].tx_buffer), 2);
        }

        // If there is no RX DMA and only one frame is transmitted, then
        //  the transaction is done. Single-length transmissions
        //  does not trigger a CTZ interrupt.
        if (rx_fr_len > 0 && rx_buffer != NULL) {
            STATES[spi_num].controller_done = true;
        }

        STATES[spi_num].tx_done = true;

        // 3) Set up DMA TX for RX only transactions.
        //    Note: Even if you are not transmitting anything in standard 4-wire mode,
        //      the hardware always assume full duplex. Therefore dummy bytes
        //      must be transmitted to support half duplex. The number of bytes to transmit
        //      AND receive is set by TX_NUM_CHAR, and the RX_NUM_CHAR field of ctrl1 is ignored.
    } else if (tx_fr_len == 0 && STATES[spi_num].init.mode == MXC_SPI_INTERFACE_STANDARD) {
        // For readability purposes.
        tx_ch = STATES[spi_num].tx_dma_ch;

        // Configure TX DMA channel to retransmit the dummy byte.
        STATES[spi_num].dma->ch[tx_ch].src = (uint32_t)(&(STATES[spi_num].tx_dummy_value));
        STATES[spi_num].dma->ch[tx_ch].cnt = STATES[spi_num].rx_len; // Only receiving
        STATES[spi_num].dma->ch[tx_ch].ctrl &= ~MXC_F_DMA_REVA_CTRL_SRCINC;
        STATES[spi_num].dma->ch[tx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA
    }

    // Enable SPI TX DMA after configuring.
    spi->dma |= (MXC_F_SPI_REVA_DMA_DMA_TX_EN);

    // Set up DMA RX Transactions.
    if (rx_fr_len > 0 && rx_buffer != NULL) {
        // For readability purposes.
        rx_ch = STATES[spi_num].rx_dma_ch;

        // Enable RX DMA channel before configuring.
        spi->dma |= (MXC_F_SPI_REVA_DMA_RX_FIFO_EN);

        STATES[spi_num].dma->ch[rx_ch].dst = (uint32_t)rx_buffer;
        STATES[spi_num].dma->ch[rx_ch].cnt = STATES[spi_num].rx_len;

        // Set to one byte burst size.
        MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_BURST_SIZE,
                     (0 << MXC_F_DMA_REVA_CTRL_BURST_SIZE_POS));

        // Match frame size (in terms of bytes) in DMA ctrl settings.
        if (STATES[spi_num].init.frame_size <= 8) {
            // Set source and destination width to one byte
            MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                         MXC_S_DMA_REVA_CTRL_SRCWD_BYTE);
            MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                         MXC_S_DMA_REVA_CTRL_DSTWD_BYTE);
        } else {
            // Set source destination width to 2 bytes
            MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_SRCWD,
                         MXC_S_DMA_REVA_CTRL_SRCWD_HALFWORD);
            MXC_SETFIELD(STATES[spi_num].dma->ch[rx_ch].ctrl, MXC_F_DMA_REVA_CTRL_DSTWD,
                         MXC_S_DMA_REVA_CTRL_DSTWD_HALFWORD);
        }

        STATES[spi_num].dma->ch[rx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_DSTINC;
        STATES[spi_num].dma->ch[rx_ch].ctrl |= MXC_F_DMA_REVA_CTRL_EN; // Start the DMA

        // Enable SPI RX DMA after configuring.
        spi->dma |= (MXC_F_SPI_REVA_DMA_DMA_RX_EN);
    }

    // Start the SPI transaction.
    spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_START;

    // Handle target-select (L. SS) deassertion.  This must be done AFTER launching the transaction
    // to avoid a glitch on the TS line if:
    // - The TS line is asserted
    // - We want to deassert the line as part of this transaction
    //
    // As soon as the SPI hardware receives CTRL0->START it seems to reinitialize the TS pin based
    // on the value of CTRL->SS_CTRL, which causes the glitch.
    if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
        // In HW Auto Scheme, only use the target index member.
        MXC_SETFIELD(spi->ctrl0, MXC_F_SPI_REVA_CTRL0_SS_ACTIVE,
                     ((1 << target->index) << MXC_F_SPI_REVA_CTRL0_SS_ACTIVE_POS));

        if (deassert) {
            spi->ctrl0 &= ~MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        } else {
            spi->ctrl0 |= MXC_F_SPI_REVA_CTRL0_SS_CTRL;
        }

        // Toggle Chip Select Pin if handled by the driver.
    } else if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
        // Make sure the selected Target Select (L. SS) pin is enabled as an output.
        if (target->pins.func != MXC_GPIO_FUNC_OUT) {
            return E_BAD_STATE;
        }

        // Active HIGH (1).
        if (target->active_polarity) {
            target->pins.port->out_set |= target->pins.mask;
            // Active LOW (0).
        } else {
            target->pins.port->out_clr |= target->pins.mask;
        }
    }

    return E_SUCCESS;
}

int MXC_SPI_RevA2_ControllerTransactionDMAB(mxc_spi_reva_regs_t *spi, uint8_t *tx_buffer,
                                            uint32_t tx_fr_len, uint8_t *rx_buffer,
                                            uint32_t rx_fr_len, uint8_t deassert,
                                            mxc_spi_target_t *target)
{
    int error;
    int spi_num;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // This function fills in the STATES value for the flags that checks for blocking status.
    error = MXC_SPI_RevA2_ControllerTransactionDMA(spi, tx_buffer, tx_fr_len, rx_buffer, rx_fr_len,
                                                   deassert, target);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Blocking
    while (((STATES[spi_num].controller_done == false && STATES[spi_num].tx_done == false) &&
            !(STATES[spi_num].tx_buffer != NULL && STATES[spi_num].tx_len > 0)) &&
           (STATES[spi_num].rx_done == false &&
            !(STATES[spi_num].rx_buffer != NULL && STATES[spi_num].rx_len > 0))) {}

    return E_SUCCESS;
}

/* ** Handler Functions ** */

void MXC_SPI_RevA2_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t status = spi->intfl;

    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *target_port;
    uint32_t target_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    // Master done (TX complete)
    if (status & MXC_F_SPI_REVA_INTFL_MST_DONE) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_MST_DONE; // Clear flag

        // Toggle Target Select (TS) Pin if Driver is handling it.
        if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                target_port = STATES[spi_num].current_target.pins.port;
                target_mask = STATES[spi_num].current_target.pins.mask;

                target_port->out ^= target_mask;
            } // Don't deassert the Target Select (TS) pin if false for multiple repeated transactions.
        }

        // Callback if valid.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].callback) {
            STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
        }

        // Controller is done after callback (if valid) is handled.
        STATES[spi_num].controller_done = true;
    }

    // Handle RX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_RX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_RX_THD;
        if (STATES[spi_num].init.use_dma == false) {
            // RX threshold has been crossed, there's data to unload from the FIFO
            MXC_SPI_RevA2_process(spi);
        }
    }

    // Handle TX Threshold
    if (status & MXC_F_SPI_REVA_INTFL_TX_THD) {
        spi->intfl |= MXC_F_SPI_REVA_INTFL_TX_THD;
        if (STATES[spi_num].init.use_dma == false) {
            // TX threshold has been crossed, we need to refill the FIFO
            MXC_SPI_RevA2_process(spi);
        }
    }
}

void MXC_SPI_RevA2_DMA_TX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t tx_ch;
    uint32_t status;

    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *target_port;
    uint32_t target_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    tx_ch = STATES[spi_num].tx_dma_ch;
    status = STATES[spi_num].dma->ch[tx_ch].status;

    // Count-to-Zero (DMA TX complete)
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) {
        STATES[spi_num].tx_done = true;
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_CTZ_IF;

        // For completeness-sake.
        STATES[spi_num].tx_cnt = STATES[spi_num].tx_len;

        // Toggle Target Select (TS) Pin if Driver is handling it.
        if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                target_port = STATES[spi_num].current_target.pins.port;
                target_mask = STATES[spi_num].current_target.pins.mask;

                target_port->out ^= target_mask;
            } // Don't deassert the Target Select (TS) pin if false for multiple repeated transactions.
        }

        // Callback if valid and if you're only transmitting.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].rx_buffer == NULL) {
            if (STATES[spi_num].callback) {
                STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
            }
        }

        // TX Transaction is done if there's no RX transaction.
        if (STATES[spi_num].rx_len == 0 || STATES[spi_num].tx_buffer == NULL) {
            STATES[spi_num].controller_done = true;
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[tx_ch].status |= MXC_F_DMA_REVA_STATUS_BUS_ERR;
    }
}

void MXC_SPI_RevA2_DMA_RX_Handler(mxc_spi_reva_regs_t *spi)
{
    int spi_num;
    uint32_t rx_ch;
    uint32_t status;
    // Used later for readability purposes on handling Chip Select.
    mxc_gpio_regs_t *target_port;
    uint32_t target_mask;

    spi_num = MXC_SPI_GET_IDX((mxc_spi_regs_t *)spi);
    MXC_ASSERT(spi_num >= 0);

    rx_ch = STATES[spi_num].rx_dma_ch;
    status = STATES[spi_num].dma->ch[rx_ch].status;

    // Count-to-Zero (DMA RX complete).
    if (status & MXC_F_DMA_REVA_STATUS_CTZ_IF) {
        // HW Bug: For 2-byte wide frame transactions, RX DMA swaps the
        //      LSB and MSB.
        // Example: TX: 0x1234 => RX: 0x3412
        if (STATES[spi_num].init.frame_size > 8) {
            MXC_SPI_RevA2_DMA_SwapByte(STATES[spi_num].rx_buffer, STATES[spi_num].rx_len);
        }

        STATES[spi_num].rx_done = 1;
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_CTZ_IF;

        // Toggle Target Select (TS) Pin if Driver is handling it.
        if (STATES[spi_num].init.ts_control == MXC_SPI_TSCONTROL_SW_DRV) {
            if (STATES[spi_num].deassert == true) {
                // Readability for handling Chip Select.
                target_port = STATES[spi_num].current_target.pins.port;
                target_mask = STATES[spi_num].current_target.pins.mask;

                target_port->out ^= target_mask;
            } // Don't deassert the Target Select (TS) pin if false for multiple repeated transactions.
        }

        // For completeness-sake.
        STATES[spi_num].rx_cnt = STATES[spi_num].rx_len;

        // Callback if valid.
        // Note: If Target Select (TS) Control Scheme is set in SW_App mode, then the caller needs to ensure the
        //   Target Select (TS) pin is asserted or deasserted in their application.
        if (STATES[spi_num].callback) {
            STATES[spi_num].callback(STATES[spi_num].callback_data, E_NO_ERROR);
        }

        // RX transaction determines the controller is done if TX transaction is also present.
        if (STATES[spi_num].tx_len > 0 && STATES[spi_num].tx_buffer != NULL) {
            STATES[spi_num].controller_done = true;
        }
    }

    // Bus Error
    if (status & MXC_F_DMA_REVA_STATUS_BUS_ERR) {
        STATES[spi_num].dma->ch[rx_ch].status |= MXC_F_DMA_STATUS_BUS_ERR;
    }
}
