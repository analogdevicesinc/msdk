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
#include <stdbool.h>

#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_errors.h"
#include "mxc_lock.h"
#include "mxc_sys.h"
#include "mxc_delay.h"
#include "spi_reva2.h"
#include "spi.h"
#include "dma.h"
#include "gpio.h"

/* **** Definitions **** */

/* ************************************************************************** */

// Max 3 Possible Target Select Options per SPI instance
#define MXC_SPI_TS0_MASK_POS (0)
#define MXC_SPI_TS1_MASK_POS (1)
#define MXC_SPI_TS2_MASK_POS (2)

// Private variable used to check if DMA is used or not for Init function.
static int use_dma = 0;

// Private helper function to set up Init struct from legacy implementation.
// Returns Success/Error Codes.
static int MXC_SPI_legacy_setupInit(mxc_spi_init_t *init, mxc_spi_regs_t *spi, int masterMode,
                                    int quadModeUsed, int numSlaves, unsigned ssPolarity,
                                    unsigned int hz, mxc_spi_pins_t pins, bool useDMA)
{
    // Set up init struct.
    init->spi = spi;

    if (masterMode) {
        init->type = MXC_SPI_TYPE_CONTROLLER; // L. Master
    } else {
        init->type = MXC_SPI_TYPE_TARGET; // L. Slave
    }

    init->freq = hz;

    if (quadModeUsed) {
        init->mode = MXC_SPI_INTERFACE_QUAD;
    } else {
        init->mode = MXC_SPI_INTERFACE_STANDARD;
    }

    // New SPI drivers will not use "mxc_spi_pins_t pins" anymore.
    init->spi_pins = NULL;

    // Set up SPI pins like the legacy implementation.
    init->ts_control = MXC_SPI_TSCONTROL_HW_AUTO;
    init->target.pins = (const mxc_gpio_cfg_t){ 0 };
    init->target.active_polarity = ssPolarity;

    // Set VSSEL level
    if (pins.vddioh) {
        init->vssel = MXC_GPIO_VSSEL_VDDIOH;
    } else {
        init->vssel = MXC_GPIO_VSSEL_VDDIO;
    }

    // Set up Target Select pins.
    if (pins.ss0) {
        init->target.init_mask |= (1 << MXC_SPI_TS0_MASK_POS); // Bit position 0
    }

    if (pins.ss1) {
        init->target.init_mask |= (1 << MXC_SPI_TS1_MASK_POS); // Bit position 1
    }

    if (pins.ss2) {
        init->target.init_mask |= (1 << MXC_SPI_TS2_MASK_POS); // Bit position 2
    }

    if (pins.sdio2 || pins.sdio3) {
        // Ensure QUAD mode is enabled if SDIO2/SDIO3 are true.
        if (quadModeUsed == 0) {
            return E_BAD_PARAM;
        }
    }

    // In the previous implementation, the MXC_SPI_Init function does not
    //  set the message size until later by calling MXC_SPI_SetData.
    // By default for the new implementation, the frame_size will be set to 8 bits.
    init->frame_size = 8;

    // Set default clock mode (0).
    init->clk_mode = MXC_SPI_CLKMODE_0;

    // By default, DMA will be initalized for API function
    if (useDMA) {
        init->use_dma = true;
        init->dma = MXC_DMA;
    } else {
        init->use_dma = false;
        init->dma = NULL;
    }

    return E_NO_ERROR;
}

int MXC_SPI_Init(mxc_spi_regs_t *spi, int masterMode, int quadModeUsed, int numSlaves,
                 unsigned ssPolarity, unsigned int hz, mxc_spi_pins_t pins)
{
    int error, spi_num;

    // TODO(5-15-2023): Remove this section when the Init function is updated to
    //      int MXC_SPI_Init(mxc_spi_init_t *init)
    // This function is for backwards compatibility, before fully updating to new
    //   implementation.
    mxc_spi_init_t spi_init = (const mxc_spi_init_t){ 0 };
    mxc_spi_init_t *init = &spi_init;

    error = MXC_SPI_legacy_setupInit(init, spi, masterMode, quadModeUsed, numSlaves, ssPolarity, hz,
                                     pins, use_dma);
    if (error != E_NO_ERROR) {
        return error;
    }

    spi_num = MXC_SPI_GET_IDX(init->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if ((spi_num == 0) && (init->freq > MXC_SPI_GetPeripheralClock(init->spi))) {
        return E_BAD_PARAM;
    }

    if ((spi_num == 1) && (init->freq > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

    // Note: Target Select (L. SS) Pins will be configured in MXC_SPI_RevA2_Init.
    // Configure GPIO for spi
    if (init->spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->mode == MXC_SPI_INTERFACE_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_3wire);

            } else if (init->mode == MXC_SPI_INTERFACE_STANDARD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_standard);

            } else if (init->mode == MXC_SPI_INTERFACE_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_dual);

            } else if (init->mode == MXC_SPI_INTERFACE_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }

// Handles RISCV SPI Numbering.
#ifdef MXC_SPI0
    } else if (init->spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->mode == MXC_SPI_INTERFACE_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_3wire);

            } else if (init->mode == MXC_SPI_INTERFACE_STANDARD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_standard);

            } else if (init->mode == MXC_SPI_INTERFACE_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_dual);

            } else if (init->mode == MXC_SPI_INTERFACE_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }
#endif

    } else {
        return E_NO_DEVICE;
    }

    return MXC_SPI_RevA2_Init(init);
}

int MXC_SPI_Init_v2(mxc_spi_init_t *init)
{
    int error, spi_num;

    spi_num = MXC_SPI_GET_IDX(init->spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if ((spi_num == 0) && (init->freq > MXC_SPI_GetPeripheralClock(init->spi))) {
        return E_BAD_PARAM;
    }

    if ((spi_num == 1) && (init->freq > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

    // Note: Target Select (L. SS) Pins will be configured in MXC_SPI_RevA2_Init.
    // Configure GPIO for spi
    if (init->spi == MXC_SPI1) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->mode == MXC_SPI_INTERFACE_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_3wire);

            } else if (init->mode == MXC_SPI_INTERFACE_STANDARD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_standard);

            } else if (init->mode == MXC_SPI_INTERFACE_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_dual);

            } else if (init->mode == MXC_SPI_INTERFACE_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi1_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }

// Handles RISCV SPI Numbering.
#ifdef MXC_SPI0
    } else if (init->spi == MXC_SPI0) {
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        // Configure SPI to default pins if not provided.
        if (init->spi_pins == NULL) {
            if (init->mode == MXC_SPI_INTERFACE_3WIRE) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_3wire);

            } else if (init->mode == MXC_SPI_INTERFACE_STANDARD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_standard);

            } else if (init->mode == MXC_SPI_INTERFACE_DUAL) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_dual);

            } else if (init->mode == MXC_SPI_INTERFACE_QUAD) {
                error = MXC_GPIO_Config(&gpio_cfg_spi0_quad);

            } else {
                return E_BAD_PARAM;
            }

        } else {
            error = MXC_GPIO_Config(init->spi_pins);
        }

        // Ensure SPI GPIO pins were properly configured.
        if (error != E_NO_ERROR) {
            return error;
        }
#endif

    } else {
        return E_NO_DEVICE;
    }

    return MXC_SPI_RevA2_Init(init);
}

int MXC_SPI_InitStruct(mxc_spi_init_t *init)
{
    if (init == NULL) {
        return E_BAD_PARAM;
    }

    init->spi = MXC_SPI1; // SPI1 is available on both the ARM and RISCV core.
    init->spi_pins = NULL; // Use default, predefined pins
    init->type = MXC_SPI_TYPE_CONTROLLER; // Controller mode
    init->freq = 100000; // 100KHz
    init->clk_mode = MXC_SPI_CLKMODE_0; // 0 - CPOL :: 0 - CPHA
    init->mode = MXC_SPI_INTERFACE_STANDARD; // Standard 4-wire mode
    init->ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // Automatic Hardware Driven TS Control
    init->target.active_polarity = 0; // Active polarity is LOW (0). IDLE is HIGH (1).
    init->vssel = MXC_GPIO_VSSEL_VDDIO; // VDDIO - 1.8V
    init->target.init_mask = 0x01; // Default TS0
    init->use_dma = false; // DMA not used
    init->dma = NULL;

    return E_SUCCESS;
}

int MXC_SPI_InitStruct_DMA(mxc_spi_init_t *init)
{
    if (init == NULL) {
        return E_BAD_PARAM;
    }

    init->spi = MXC_SPI1; // SPI1 is available on both the ARM and RISCV core.
    init->spi_pins = NULL; // Use default, predefined pins
    init->type = MXC_SPI_TYPE_CONTROLLER; // Controller mode
    init->freq = 100000; // 100KHz
    init->clk_mode = MXC_SPI_CLKMODE_0; // 0 - CPOL :: 0 - CPHA
    init->mode = MXC_SPI_INTERFACE_STANDARD; // Standard 4-wire mode
    init->ts_control = MXC_SPI_TSCONTROL_HW_AUTO; // Automatic Hardware Driven TS Control
    init->target.active_polarity = 0; // Active polarity is LOW (0), IDLE is HIGH (1)
    init->vssel = MXC_GPIO_VSSEL_VDDIO; // VDDIO - 1.8V
    init->target.init_mask = 0x01; // Default TS0
    init->use_dma = true; // Use DMA
    init->dma = MXC_DMA;

    return E_SUCCESS;
}

int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    int spi_num;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    MXC_SPI_RevA2_Shutdown((mxc_spi_reva_regs_t *)spi);

    if (spi == MXC_SPI1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
#ifdef MXC_SPI0
    } else if (spi == MXC_SPI0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
#endif
    } else {
        return E_NO_DEVICE;
    }

    return E_NO_ERROR;
}

unsigned int MXC_SPI_GetFlags(mxc_spi_regs_t *spi)
{
    return (unsigned int)MXC_SPI_RevA2_GetFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearFlags(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_ClearFlags((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_EnableInt(mxc_spi_regs_t *spi, unsigned int intEn)
{
    MXC_SPI_RevA2_EnableInt((mxc_spi_reva_regs_t *)spi, (uint32_t)intEn);
}

void MXC_SPI_DisableInt(mxc_spi_regs_t *spi, unsigned int intDis)
{
    MXC_SPI_RevA2_DisableInt((mxc_spi_reva_regs_t *)spi, (uint32_t)intDis);
}

int MXC_SPI_GetPeripheralClock(mxc_spi_regs_t *spi)
{
    int retval;

    if (spi == MXC_SPI1) {
        retval = PeripheralClock;
#ifdef MXC_SPI0 // SPI0 is not accessible from the RISC core.
    } else if (spi == MXC_SPI0) {
        int sys_clk = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
                      MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS;
        switch (sys_clk) {
        case MXC_SYS_CLOCK_IPO:
            retval = IPO_FREQ;
            break;
        case MXC_SYS_CLOCK_IBRO:
            retval = IBRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ISO:
            retval = ISO_FREQ;
            break;
        case MXC_SYS_CLOCK_INRO:
            retval = INRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERTCO:
            retval = ERTCO_FREQ;
            break;
        case MXC_SYS_CLOCK_EXTCLK:
            retval = EXTCLK_FREQ;
            break;
        default:
            return E_BAD_STATE;
        }
#endif // MXC_SPI0
    } else {
        return E_BAD_PARAM;
    }

    retval /= 2;

    return retval;
}

int MXC_SPI_ConfigTargetSelect(mxc_spi_regs_t *spi, uint32_t index, mxc_gpio_vssel_t vssel)
{
    int error, spi_num;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    if (spi == MXC_SPI1) {
        if (index > MXC_SPI1_TS_INSTANCES) {
            return E_BAD_PARAM;
        }

        switch (index) {
        // Target Select 0 - TS0 (L. SS0 pin)
        case 0:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_ts0);
            if (error != E_NO_ERROR) {
                return error;
            }

            error = MXC_GPIO_SetVSSEL(gpio_cfg_spi1_ts0.port, vssel, gpio_cfg_spi1_ts0.mask);
            if (error != E_NO_ERROR) {
                return error;
            }

            break;

        default:
            return E_BAD_PARAM;
        }

#ifdef MXC_SPI0
    } else if (spi == MXC_SPI0) {
        if (index > MXC_SPI0_TS_INSTANCES) {
            return E_BAD_PARAM;
        }

        switch (index) {
        // Target Select 0 - TS0 (L. SS0 pin)
        case 0:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_ts0);
            if (error != E_NO_ERROR) {
                return error;
            }

            error = MXC_GPIO_SetVSSEL(gpio_cfg_spi0_ts0.port, vssel, gpio_cfg_spi0_ts0.mask);
            if (error != E_NO_ERROR) {
                return error;
            }

            break;

        // Target Select 1 - TS1 (L. SS1 pin)
        case 1:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_ts1);
            if (error != E_NO_ERROR) {
                return error;
            }

            error = MXC_GPIO_SetVSSEL(gpio_cfg_spi0_ts1.port, vssel, gpio_cfg_spi0_ts1.mask);
            if (error != E_NO_ERROR) {
                return error;
            }

            break;

        // Target Select 2 (TS2 - L. SS2 pin)
        case 2:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_ts2);
            if (error != E_NO_ERROR) {
                return error;
            }

            error = MXC_GPIO_SetVSSEL(gpio_cfg_spi0_ts2.port, vssel, gpio_cfg_spi0_ts2.mask);
            if (error != E_NO_ERROR) {
                return error;
            }

            break;

        default:
            return E_BAD_PARAM;
        }
#endif
    }

    return E_NO_ERROR;
}

int MXC_SPI_SetFrequency(mxc_spi_regs_t *spi, unsigned int hz)
{
    return MXC_SPI_RevA2_SetFrequency((mxc_spi_reva_regs_t *)spi, hz);
}

unsigned int MXC_SPI_GetFrequency(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetFrequency((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetFrameSize(mxc_spi_regs_t *spi, int frame_size)
{
    return MXC_SPI_RevA2_SetFrameSize((mxc_spi_reva_regs_t *)spi, frame_size);
}

int MXC_SPI_GetFrameSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetFrameSize((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetInterface(mxc_spi_regs_t *spi, mxc_spi_interface_t mode)
{
    return MXC_SPI_RevA2_SetInterface((mxc_spi_reva_regs_t *)spi, mode);
}

mxc_spi_interface_t MXC_SPI_GetInterface(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetInterface((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetClkMode(mxc_spi_regs_t *spi, mxc_spi_clkmode_t clk_mode)
{
    return MXC_SPI_RevA2_SetClkMode((mxc_spi_reva_regs_t *)spi, clk_mode);
}

mxc_spi_clkmode_t MXC_SPI_GetClkMode(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetClkMode((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetCallback(mxc_spi_regs_t *spi, mxc_spi_callback_t callback, void *data)
{
    return MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)spi, callback, data);
}

int MXC_SPI_GetActive(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetActive((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_ReadyForSleep(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_ReadyForSleep((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetDummyTX(mxc_spi_regs_t *spi, uint16_t tx_value)
{
    return MXC_SPI_RevA2_SetDummyTX((mxc_spi_reva_regs_t *)spi, tx_value);
}

int MXC_SPI_StartTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_StartTransmission((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_AbortTransmission(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_AbortTransmission((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_GetTXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetTXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_GetRXFIFOAvailable(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetRXFIFOAvailable((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearTXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_ClearTXFIFO((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_ClearRXFIFO(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_ClearRXFIFO((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetTXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return (int)MXC_SPI_RevA2_SetTXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

int MXC_SPI_SetRXThreshold(mxc_spi_regs_t *spi, unsigned int numBytes)
{
    return (int)MXC_SPI_RevA2_SetRXThreshold((mxc_spi_reva_regs_t *)spi, numBytes);
}

unsigned int MXC_SPI_GetTXThreshold(mxc_spi_regs_t *spi)
{
    return (unsigned int)MXC_SPI_RevA2_GetTXThreshold((mxc_spi_reva_regs_t *)spi);
}

unsigned int MXC_SPI_GetRXThreshold(mxc_spi_regs_t *spi)
{
    return (unsigned int)MXC_SPI_RevA2_GetRXThreshold((mxc_spi_reva_regs_t *)spi);
}

/* ** DMA-Specific Functions ** */

int MXC_SPI_DMA_Init(mxc_spi_init_t *init)
{
    return MXC_SPI_RevA2_DMA_Init(init);
}

bool MXC_SPI_DMA_GetInitialized(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_DMA_GetInitialized((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_DMA_GetTXChannel(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_DMA_GetTXChannel((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_DMA_GetRXChannel(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_DMA_GetRXChannel((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_DMA_SetRequestSelect(mxc_spi_regs_t *spi, uint8_t *tx_buffer, uint8_t *rx_buffer)
{
    int spi_num;
    int tx_reqsel = -1;
    int rx_reqsel = -1;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_INVALID;
    }

    if (tx_buffer != NULL) {
        switch (spi_num) {
        case 0:
            tx_reqsel = MXC_DMA_REQUEST_SPI1TX;
            break;

        case 1:
            tx_reqsel = MXC_DMA_REQUEST_SPI0TX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    if (rx_buffer != NULL) {
        switch (spi_num) {
        case 0:
            rx_reqsel = MXC_DMA_REQUEST_SPI1RX;
            break;

        case 1:
            rx_reqsel = MXC_DMA_REQUEST_SPI0RX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    return MXC_SPI_RevA2_DMA_SetRequestSelect((mxc_spi_reva_regs_t *)spi, tx_reqsel, rx_reqsel);
}

/* ** Transaction Functions ** */

int MXC_SPI_MasterTransaction(mxc_spi_req_t *req)
{
    mxc_spi_target_t target;

    // For backwards-compatibility with previous SPI implementation, use
    //  use the req->ts_idx (L. req->ssIdx) as default.
    // Note: the previous implementation did not have an option to select
    //  targets for transactions.
    target.index = req->ts_idx; // also req->ssIdx

    // Mainly used if MXC_SPI_TSCONTROL_SW_DRV scheme was selected.
    target.pins = req->target_sel.pins;
    target.active_polarity = req->target_sel.active_polarity;

    return MXC_SPI_RevA2_ControllerTransactionB((mxc_spi_reva_regs_t *)(req->spi), req->tx_buffer,
                                                req->tx_len, req->rx_buffer, req->rx_len,
                                                req->deassert, &target);
}

int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    int error;
    mxc_spi_target_t target;

    // For backwards-compatibility with previous SPI implementation, use
    //  use the req->ts_idx (L. req->ssIdx) as default.
    // Note: the previous implementation did not have an option to select
    //  targets for transactions.
    target.index = req->ts_idx; // also req->ssIdx

    // Mainly used if MXC_SPI_TSCONTROL_SW_DRV scheme was selected.
    target.pins = req->target_sel.pins;
    target.active_polarity = req->target_sel.active_polarity;

    error = MXC_SPI_SetCallback(req->spi, req->callback, req->callback_data);
    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_SPI_RevA2_ControllerTransaction((mxc_spi_reva_regs_t *)(req->spi), req->tx_buffer,
                                               req->tx_len, req->rx_buffer, req->rx_len,
                                               req->deassert, &target);
}

int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req)
{
    int error;
    mxc_spi_target_t target;
    mxc_spi_init_t init;

    // For backwards-compatibility with previous SPI implementation, use
    //  use the req->ts_idx (L. req->ssIdx) as default.
    // Note: the previous implementation did not have an option to select
    //  targets for transactions.
    target.index = req->ts_idx; // also req->ssIdx

    // Mainly used if MXC_SPI_TSCONTROL_SW_DRV scheme was selected.
    target.pins = req->target_sel.pins;
    target.active_polarity = req->target_sel.active_polarity;

    init.use_dma = true;
    init.dma = MXC_DMA;

    // More overhead, but this function will initalize DMA if not initialized.
    if (MXC_SPI_DMA_GetInitialized(req->spi) == false) {
        error = MXC_SPI_DMA_Init(&init);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    error = MXC_SPI_SetCallback(req->spi, req->callback, req);
    if (error != E_NO_ERROR) {
        return error;
    }

    error = MXC_SPI_DMA_SetRequestSelect(req->spi, req->tx_buffer, req->rx_buffer);
    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_SPI_RevA2_ControllerTransactionDMA((mxc_spi_reva_regs_t *)(req->spi), req->tx_buffer,
                                                  req->tx_len, req->rx_buffer, req->rx_len,
                                                  req->deassert, &target);
}

int MXC_SPI_ControllerTransaction(mxc_spi_regs_t *spi, uint8_t *tx_buffer, uint32_t tx_fr_len,
                                  uint8_t *rx_buffer, uint32_t rx_fr_len, uint8_t deassert,
                                  mxc_spi_target_t *target)
{
    return MXC_SPI_RevA2_ControllerTransaction((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_fr_len,
                                               rx_buffer, rx_fr_len, deassert, target);
}

int MXC_SPI_ControllerTransactionB(mxc_spi_regs_t *spi, uint8_t *tx_buffer, uint32_t tx_fr_len,
                                   uint8_t *rx_buffer, uint32_t rx_fr_len, uint8_t deassert,
                                   mxc_spi_target_t *target)
{
    return MXC_SPI_RevA2_ControllerTransactionB((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_fr_len,
                                                rx_buffer, rx_fr_len, deassert, target);
}

int MXC_SPI_ControllerTransactionDMA(mxc_spi_regs_t *spi, uint8_t *tx_buffer, uint32_t tx_fr_len,
                                     uint8_t *rx_buffer, uint32_t rx_fr_len, uint8_t deassert,
                                     mxc_spi_target_t *target)
{
    int error;

    error = MXC_SPI_DMA_SetRequestSelect(spi, tx_buffer, rx_buffer);
    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_SPI_RevA2_ControllerTransactionDMA((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_fr_len,
                                                  rx_buffer, rx_fr_len, deassert, target);
}

int MXC_SPI_ControllerTransactionDMAB(mxc_spi_regs_t *spi, uint8_t *tx_buffer, uint32_t tx_fr_len,
                                      uint8_t *rx_buffer, uint32_t rx_fr_len, uint8_t deassert,
                                      mxc_spi_target_t *target)
{
    return MXC_SPI_RevA2_ControllerTransactionDMAB((mxc_spi_reva_regs_t *)spi, tx_buffer, tx_fr_len,
                                                   rx_buffer, rx_fr_len, deassert, target);
}

int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t *req)
{
    return E_NOT_SUPPORTED;
}

/* ** Handler Functions ** */

void MXC_SPI_AsyncHandler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_Handler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_Handler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_DMA_TX_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_DMA_TX_Handler((mxc_spi_reva_regs_t *)spi);
}

void MXC_SPI_DMA_RX_Handler(mxc_spi_regs_t *spi)
{
    MXC_SPI_RevA2_DMA_RX_Handler((mxc_spi_reva_regs_t *)spi);
}

/* ** Unsupported-Legacy Functions from Previous Implementation ** */

int MXC_SPI_SetDataSize(mxc_spi_regs_t *spi, int dataSize)
{
    return MXC_SPI_RevA2_SetFrameSize((mxc_spi_reva_regs_t *)spi, dataSize);
}

int MXC_SPI_GetDataSize(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetFrameSize((mxc_spi_reva_regs_t *)spi);
}

int MXC_SPI_SetWidth(mxc_spi_regs_t *spi, mxc_spi_width_t spiWidth)
{
    switch (spiWidth) {
    case SPI_WIDTH_3WIRE:
        return MXC_SPI_SetInterface(spi, MXC_SPI_INTERFACE_3WIRE);

    case SPI_WIDTH_STANDARD:
        return MXC_SPI_SetInterface(spi, MXC_SPI_INTERFACE_STANDARD);

    case SPI_WIDTH_DUAL:
        return MXC_SPI_SetInterface(spi, MXC_SPI_INTERFACE_DUAL);
        break;

    case SPI_WIDTH_QUAD:
        return MXC_SPI_SetInterface(spi, MXC_SPI_INTERFACE_QUAD);

    default:
        return E_BAD_PARAM;
    }
}

mxc_spi_width_t MXC_SPI_GetWidth(mxc_spi_regs_t *spi)
{
    mxc_spi_interface_t mode;

    mode = MXC_SPI_GetInterface(spi);

    switch (mode) {
    case MXC_SPI_INTERFACE_STANDARD:
        return SPI_WIDTH_STANDARD;

    case MXC_SPI_INTERFACE_QUAD:
        return SPI_WIDTH_QUAD;

    case MXC_SPI_INTERFACE_DUAL:
        return SPI_WIDTH_DUAL;

    case MXC_SPI_INTERFACE_3WIRE:
        return SPI_WIDTH_3WIRE;

    default:
        return SPI_WIDTH_STANDARD;
    }
}

void MXC_SPI_AbortAsync(mxc_spi_regs_t *spi)
{
    MXC_SPI_AbortTransmission(spi);
}

int MXC_SPI_SetSlave(mxc_spi_regs_t *spi, int ssIdx)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_GetSlave(mxc_spi_regs_t *spi)
{
    return E_NOT_SUPPORTED;
}

int MXC_SPI_SetMode(mxc_spi_regs_t *spi, mxc_spi_mode_t spiMode)
{
    switch (spiMode) {
    case SPI_MODE_0:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_0);

    case SPI_MODE_1:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_0);

    case SPI_MODE_2:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_0);

    case SPI_MODE_3:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_0);

    default:
        return E_BAD_PARAM;
    }
}

mxc_spi_mode_t MXC_SPI_GetMode(mxc_spi_regs_t *spi)
{
    mxc_spi_clkmode_t clk_mode;

    clk_mode = MXC_SPI_GetClkMode(spi);

    switch (clk_mode) {
    case MXC_SPI_CLKMODE_0:
        return SPI_MODE_0;

    case MXC_SPI_CLKMODE_1:
        return SPI_MODE_1;

    case MXC_SPI_CLKMODE_2:
        return SPI_MODE_2;

    case MXC_SPI_CLKMODE_3:
        return SPI_MODE_3;

    default:
        return SPI_MODE_0;
    }
}

int MXC_SPI_SetDefaultTXData(mxc_spi_regs_t *spi, unsigned int defaultTXData)
{
    return MXC_SPI_RevA2_SetDummyTX((mxc_spi_reva_regs_t *)spi, defaultTXData);
}
