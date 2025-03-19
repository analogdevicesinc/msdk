/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

int MXC_SPI_Init(mxc_spi_regs_t *spi, mxc_spi_type_t controller_target, mxc_spi_interface_t if_mode,
                 int numTargets, uint8_t ts_active_pol_mask, uint32_t freq, mxc_spi_pins_t pins)
{
    int error;
    int8_t spi_num;
    mxc_spi_tscontrol_t ts_control;
    mxc_gpio_cfg_t temp_ts_cfg; // TS pins.
    mxc_gpio_vssel_t vssel;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    // Check if frequency is too high
    if ((spi_num == 0) && (freq > MXC_SPI_GetPeripheralClock(spi))) {
        return E_BAD_PARAM;
    }

    if ((spi_num == 1) && (freq > SystemCoreClock)) {
        return E_BAD_PARAM;
    }

#ifndef MSDK_NO_GPIO_CLK_INIT
    if (pins.vddioh) {
        vssel = MXC_GPIO_VSSEL_VDDIOH;
    } else {
        vssel = MXC_GPIO_VSSEL_VDDIO;
    }

    // SPI Target mode only supports HW_AUTO.
    if (pins.ss0 || pins.ss1 || pins.ss2 || (controller_target == MXC_SPI_TYPE_TARGET)) {
        ts_control = MXC_SPI_TSCONTROL_HW_AUTO;
    } else {
        ts_control = MXC_SPI_TSCONTROL_SW_APP;
    }

    error = MXC_SPI_RevA2_SetTSControl((mxc_spi_reva_regs_t *)spi, ts_control);
    if (error != E_NO_ERROR) {
        return error;
    }

    // Configure SPI peripheral and pins.
    if (spi == MXC_SPI0) { // <--------------------------------------- SPI0
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI0);

        switch (if_mode) {
        case MXC_SPI_INTERFACE_STANDARD:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_standard);
            break;

        case MXC_SPI_INTERFACE_QUAD:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_quad);
            break;

        case MXC_SPI_INTERFACE_3WIRE:
            error = MXC_GPIO_Config(&gpio_cfg_spi0_3wire);
            break;

        case MXC_SPI_INTERFACE_DUAL:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_dual);
            break;

        default:
            return E_BAD_PARAM;
        }

        if (error != E_NO_ERROR) {
            return error;
        }

        // Set up HW TS pins (if HW_AUTO TS control scheme was selected).
        //  Voltage and drive strength settings will match the SPI pins.
        if (ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
            // Target Select 0 - TS0 (L. SS0 pin)
            if (pins.ss0 == true) {
                temp_ts_cfg = gpio_cfg_spi0_ts0;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 1 - TS1 (L. SS1 pin)
            if (pins.ss1 == true) {
                temp_ts_cfg = gpio_cfg_spi0_ts1;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }
        }

    } else if (spi == MXC_SPI1) { // <--------------------------------------- SPI1
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI1);

        switch (if_mode) {
        case MXC_SPI_INTERFACE_STANDARD:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_standard);
            break;

        case MXC_SPI_INTERFACE_QUAD:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_quad_0);
            error |= MXC_GPIO_Config(&gpio_cfg_spi1_quad_1);
            if (error != E_NO_ERROR) {
                error = E_FAIL;
            }

            break;

        case MXC_SPI_INTERFACE_3WIRE:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_3wire);
            break;

        case MXC_SPI_INTERFACE_DUAL:
            error = MXC_GPIO_Config(&gpio_cfg_spi1_dual);
            break;

        default:
            return E_BAD_PARAM;
        }

        if (error != E_NO_ERROR) {
            return error;
        }

        // Set up HW TS pins (if HW_AUTO TS control scheme was selected).
        //  Voltage and drive strength settings will match the SPI pins.
        if (ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
            // Target Select 0 - TS0 (L. SS0 pin)
            if (pins.ss0 == true) {
                temp_ts_cfg = gpio_cfg_spi1_ts0;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 1 - TS1 (L. SS1 pin)
            if (pins.ss1 == true) {
                temp_ts_cfg = gpio_cfg_spi1_ts1;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 2 - TS1 (L. SS2 pin)
            if (pins.ss2 == true) {
                temp_ts_cfg = gpio_cfg_spi1_ts2;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }
        }

    } else if (spi == MXC_SPI2) { // <--------------------------------------- SPI2
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SPI2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI2);

        switch (if_mode) {
        case MXC_SPI_INTERFACE_STANDARD:
            error = MXC_GPIO_Config(&gpio_cfg_spi2_standard);
            break;

        case MXC_SPI_INTERFACE_QUAD:
            error = MXC_GPIO_Config(&gpio_cfg_spi2_quad_0);
            error |= MXC_GPIO_Config(&gpio_cfg_spi2_quad_1);
            if (error != E_NO_ERROR) {
                error = E_FAIL;
            }

            break;

        case MXC_SPI_INTERFACE_3WIRE:
            error = MXC_GPIO_Config(&gpio_cfg_spi2_3wire);
            break;

        case MXC_SPI_INTERFACE_DUAL:
            error = MXC_GPIO_Config(&gpio_cfg_spi2_dual);
            break;

        default:
            return E_BAD_PARAM;
        }

        if (error != E_NO_ERROR) {
            return error;
        }

        // Set up HW TS pins (if HW_AUTO TS control scheme was selected).
        //  Voltage and drive strength settings will match the SPI pins.
        if (ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
            // Target Select 0 - TS0 (L. SS0 pin)
            if (pins.ss0 == true) {
                temp_ts_cfg = gpio_cfg_spi2_ts0;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 1 - TS1 (L. SS1 pin)
            if (pins.ss1 == true) {
                temp_ts_cfg = gpio_cfg_spi2_ts1;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 2 - TS1 (L. SS2 pin)
            if (pins.ss2 == true) {
                temp_ts_cfg = gpio_cfg_spi2_ts2;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }
        }

#ifndef __riscv
    } else if (spi == MXC_SPI3) { // <--------------------------------------- SPI3
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI3);

        switch (if_mode) {
        case MXC_SPI_INTERFACE_STANDARD:
            error = MXC_GPIO_Config(&gpio_cfg_spi3_standard);
            break;

        case MXC_SPI_INTERFACE_QUAD:
            error = MXC_GPIO_Config(&gpio_cfg_spi3_quad);
            break;

        case MXC_SPI_INTERFACE_3WIRE:
            error = MXC_GPIO_Config(&gpio_cfg_spi3_3wire);
            break;

        case MXC_SPI_INTERFACE_DUAL:
            error = MXC_GPIO_Config(&gpio_cfg_spi3_dual);
            break;

        default:
            return E_BAD_PARAM;
        }

        if (error != E_NO_ERROR) {
            return error;
        }

        // Set up HW TS pins (if HW_AUTO TS control scheme was selected).
        //  Voltage and drive strength settings will match the SPI pins.
        if (ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
            // Target Select 0 - TS0 (L. SS0 pin)
            if (pins.ss0 == true) {
                temp_ts_cfg = gpio_cfg_spi3_ts0;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 1 - TS1 (L. SS1 pin)
            if (pins.ss1 == true) {
                temp_ts_cfg = gpio_cfg_spi3_ts1;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 2 - TS1 (L. SS2 pin)
            if (pins.ss2 == true) {
                temp_ts_cfg = gpio_cfg_spi3_ts2;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }
        }

    } else if (spi == MXC_SPI4) { // <--------------------------------------- SPI4
        MXC_SYS_Reset_Periph(MXC_SYS_RESET1_SPI4);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_SPI4);

        switch (if_mode) {
        case MXC_SPI_INTERFACE_STANDARD:
            error = MXC_GPIO_Config(&gpio_cfg_spi4_standard);
            break;

        case MXC_SPI_INTERFACE_QUAD:
            error = MXC_GPIO_Config(&gpio_cfg_spi4_quad);
            break;

        case MXC_SPI_INTERFACE_3WIRE:
            error = MXC_GPIO_Config(&gpio_cfg_spi4_3wire);
            break;

        case MXC_SPI_INTERFACE_DUAL:
            error = MXC_GPIO_Config(&gpio_cfg_spi4_dual);
            break;

        default:
            return E_BAD_PARAM;
        }

        if (error != E_NO_ERROR) {
            return error;
        }

        // Set up HW TS pins (if HW_AUTO TS control scheme was selected).
        //  Voltage and drive strength settings will match the SPI pins.
        if (ts_control == MXC_SPI_TSCONTROL_HW_AUTO) {
            // Target Select 0 - TS0 (L. SS0 pin)
            if (pins.ss0 == true) {
                temp_ts_cfg = gpio_cfg_spi4_ts0;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 1 - TS1 (L. SS1 pin)
            if (pins.ss1 == true) {
                temp_ts_cfg = gpio_cfg_spi4_ts1;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }

            // Target Select 2 - TS1 (L. SS2 pin)
            if (pins.ss2 == true) {
                temp_ts_cfg = gpio_cfg_spi4_ts2;
                temp_ts_cfg.vssel = vssel;
                temp_ts_cfg.drvstr = pins.drvstr;

                error = MXC_GPIO_Config(&temp_ts_cfg);
                if (error != E_NO_ERROR) {
                    return error;
                }
            }
        }
#endif // __riscv
    } else {
        return E_NO_DEVICE;
    }
#else
    (void)pins;
#endif // MSDK_NO_GPIO_CLK_INIT

    return MXC_SPI_RevA2_Init((mxc_spi_reva_regs_t *)spi, controller_target, if_mode, freq,
                              ts_active_pol_mask);
}

int MXC_SPI_Config(mxc_spi_cfg_t *cfg)
{
    return MXC_SPI_RevA2_Config(cfg);
}

int MXC_SPI_ConfigStruct(mxc_spi_cfg_t *cfg, bool use_dma_tx, bool use_dma_rx)
{
    if (cfg == NULL) {
        return E_BAD_PARAM;
    }

    cfg->spi = MXC_SPI0; // SPI1 is available on both the ARM and RISCV core.
    cfg->clk_mode = MXC_SPI_CLKMODE_0; // 0 - CPOL :: 0 - CPHA

    if (use_dma_tx || use_dma_rx) {
        cfg->use_dma_tx = use_dma_tx;
        cfg->use_dma_rx = use_dma_rx;
        cfg->dma = MXC_DMA;
    } else {
        cfg->use_dma_tx = false;
        cfg->use_dma_rx = false;
        cfg->dma = NULL;
    }

    return E_SUCCESS;
}

int MXC_SPI_Shutdown(mxc_spi_regs_t *spi)
{
    int8_t spi_num;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_BAD_PARAM;
    }

    MXC_SPI_RevA2_Shutdown((mxc_spi_reva_regs_t *)spi);

    if (spi == MXC_SPI0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI0);
    } else if (spi == MXC_SPI1) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI1);
    } else if (spi == MXC_SPI2) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI2);
#ifndef __riscv
    } else if (spi == MXC_SPI3) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI3);
    } else if (spi == MXC_SPI4) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_SPI4);
#endif //__riscv
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
    int clk_freq = 0;

    if (MXC_SPI_GET_IDX(spi) >= 0 && MXC_SPI_GET_IDX(spi) < 3) {
        clk_freq = PeripheralClock;
    } else if (MXC_SPI_GET_IDX(spi) >= 3) {
        uint32_t clk_src = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL) >>
                           MXC_F_GCR_CLKCTRL_SYSCLK_SEL_POS;
        switch (clk_src) {
        case MXC_SYS_CLOCK_IPO:
            clk_freq = IPO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERFO:
            clk_freq = ERFO_FREQ;
            break;
        case MXC_SYS_CLOCK_IBRO:
            clk_freq = IBRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ISO:
            clk_freq = ISO_FREQ;
            break;
        case MXC_SYS_CLOCK_INRO:
            clk_freq = INRO_FREQ;
            break;
        case MXC_SYS_CLOCK_ERTCO:
            clk_freq = ERTCO_FREQ;
            break;
        case MXC_SYS_CLOCK_EXTCLK:
            clk_freq = EXTCLK_FREQ;
            break;
        default:
            return E_BAD_PARAM;
        }
    } else {
        return E_BAD_PARAM;
    }

    return (clk_freq / 2);
}

int MXC_SPI_SetTSControl(mxc_spi_regs_t *spi, mxc_spi_tscontrol_t ts_control)
{
    return MXC_SPI_RevA2_SetTSControl((mxc_spi_reva_regs_t *)spi, ts_control);
}

mxc_spi_tscontrol_t MXC_SPI_GetTSControl(mxc_spi_regs_t *spi)
{
    return MXC_SPI_RevA2_GetTSControl((mxc_spi_reva_regs_t *)spi);
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

int MXC_SPI_SetCallback(mxc_spi_regs_t *spi, mxc_spi_callback_t completeCB, void *data)
{
    return MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)spi, completeCB, data);
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

int MXC_SPI_DMA_Init(mxc_spi_regs_t *spi, mxc_dma_regs_t *dma, bool use_dma_tx, bool use_dma_rx)
{
    return MXC_SPI_RevA2_DMA_Init((mxc_spi_reva_regs_t *)spi, (mxc_dma_reva_regs_t *)dma,
                                  use_dma_tx, use_dma_rx);
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

int MXC_SPI_DMA_SetRequestSelect(mxc_spi_regs_t *spi, bool use_dma_tx, bool use_dma_rx)
{
    int8_t spi_num;
    int tx_reqsel = -1;
    int rx_reqsel = -1;

    spi_num = MXC_SPI_GET_IDX(spi);
    if (spi_num < 0 || spi_num >= MXC_SPI_INSTANCES) {
        return E_INVALID;
    }

    if (use_dma_tx) {
        switch (spi_num) {
        case 0:
            tx_reqsel = MXC_DMA_REQUEST_SPI0TX;
            break;

        case 1:
            tx_reqsel = MXC_DMA_REQUEST_SPI1TX;
            break;

        case 2:
            tx_reqsel = MXC_DMA_REQUEST_SPI2TX;
            break;

        case 3:
            tx_reqsel = MXC_DMA_REQUEST_SPI3TX;
            break;

        case 4:
            tx_reqsel = MXC_DMA_REQUEST_SPI4TX;
            break;

        default:
            return E_BAD_PARAM;
        }
    }

    if (use_dma_rx) {
        switch (spi_num) {
        case 0:
            rx_reqsel = MXC_DMA_REQUEST_SPI0RX;
            break;

        case 1:
            rx_reqsel = MXC_DMA_REQUEST_SPI1RX;
            break;

        case 2:
            rx_reqsel = MXC_DMA_REQUEST_SPI2RX;
            break;

        case 3:
            rx_reqsel = MXC_DMA_REQUEST_SPI3RX;
            break;

        case 4:
            rx_reqsel = MXC_DMA_REQUEST_SPI4RX;
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
    return MXC_SPI_RevA2_ControllerTransaction((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                               req->txLen, req->rxData, req->rxLen, req->ssDeassert,
                                               req->ssIdx);
}

int MXC_SPI_MasterTransactionAsync(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_ControllerTransactionAsync((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                    req->txLen, req->rxData, req->rxLen,
                                                    req->ssDeassert, req->ssIdx);
}

int MXC_SPI_MasterTransactionDMA(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_ControllerTransactionDMA((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                  req->txLen, req->rxData, req->rxLen,
                                                  req->ssDeassert, req->ssIdx,
                                                  (mxc_dma_reva_regs_t *)MXC_DMA);
}

int MXC_SPI_ControllerTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA2_ControllerTransaction((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                               req->txLen, req->rxData, req->rxLen, req->ssDeassert,
                                               req->ssIdx);
}

int MXC_SPI_ControllerTransactionAsync(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_ControllerTransactionAsync((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                    req->txLen, req->rxData, req->rxLen,
                                                    req->ssDeassert, req->ssIdx);
}

int MXC_SPI_ControllerTransactionDMA(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_ControllerTransactionDMA((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                  req->txLen, req->rxData, req->rxLen,
                                                  req->ssDeassert, req->ssIdx,
                                                  (mxc_dma_reva_regs_t *)MXC_DMA);
}

int MXC_SPI_SlaveTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA2_TargetTransaction((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                           req->txLen, req->rxData, req->rxLen);
}

int MXC_SPI_SlaveTransactionAsync(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_TargetTransactionAsync((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                req->txLen, req->rxData, req->rxLen);
}

int MXC_SPI_SlaveTransactionDMA(mxc_spi_req_t *req)
{
    int error;

    // Users can set their own callback and pass in their own data if they choose to.
    if (req->completeCB != NULL) {
        error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
        if (error != E_NO_ERROR) {
            return error;
        }
    }

    return MXC_SPI_RevA2_TargetTransactionDMA((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                              req->txLen, req->rxData, req->rxLen,
                                              (mxc_dma_reva_regs_t *)MXC_DMA);
}

int MXC_SPI_TargetTransaction(mxc_spi_req_t *req)
{
    return MXC_SPI_RevA2_TargetTransaction((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                           req->txLen, req->rxData, req->rxLen);
}

int MXC_SPI_TargetTransactionAsync(mxc_spi_req_t *req)
{
    int error;

    error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_SPI_RevA2_TargetTransactionAsync((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                                req->txLen, req->rxData, req->rxLen);
}

int MXC_SPI_TargetTransactionDMA(mxc_spi_req_t *req)
{
    int error;

    error = MXC_SPI_RevA2_SetCallback((mxc_spi_reva_regs_t *)(req->spi), req->completeCB, req);
    if (error != E_NO_ERROR) {
        return error;
    }

    return MXC_SPI_RevA2_TargetTransactionDMA((mxc_spi_reva_regs_t *)(req->spi), req->txData,
                                              req->txLen, req->rxData, req->rxLen,
                                              (mxc_dma_reva_regs_t *)MXC_DMA);
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
    mxc_spi_interface_t if_mode;

    if_mode = MXC_SPI_GetInterface(spi);

    switch (if_mode) {
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
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_1);

    case SPI_MODE_2:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_2);

    case SPI_MODE_3:
        return MXC_SPI_SetClkMode(spi, MXC_SPI_CLKMODE_3);

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

void MXC_SPI_HWSSControl(mxc_spi_regs_t *spi, int state)
{
    MXC_ASSERT(0);
}
