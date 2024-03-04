/**
 * @file    rpu.h
 * @brief   RPU function prototypes and data types.
 */

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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_RPU_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_RPU_H_

/* **** Includes **** */
#include "rpu_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup rpu Resource Protection Unit
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */

// Bus Masters whose access to peripherals is controlled by the RPU
typedef enum {
    MXC_RPU_DMA0_ALLOW = 0x01,
    MXC_RPU_DMA1_ALLOW = 0x02,
    MXC_RPU_USB_ALLOW = 0x04,
    MXC_RPU_SYS0_ALLOW = 0x08,
    MXC_RPU_SYS1_ALLOW = 0x10,
    MXC_RPU_SDMAD_ALLOW = 0x20,
    MXC_RPU_SDMAI_ALLOW = 0x40,
    MXC_RPU_CRYPTO_ALLOW = 0x80,
    MXC_RPU_SDIO_ALLOW = 0x100
} mxc_rpu_allow_t;

// Peripherals gated by the RPU
typedef enum {
    MXC_RPU_GCR = MXC_R_RPU_GCR,
    MXC_RPU_FLC0 = MXC_R_RPU_FLC0,
    MXC_RPU_SDHCCTRL = MXC_R_RPU_SDHCCTRL,
    MXC_RPU_SIR = MXC_R_RPU_SIR,
    MXC_RPU_FCR = MXC_R_RPU_FCR,
    MXC_RPU_CRYPTO = MXC_R_RPU_TPU,
    MXC_RPU_WDT0 = MXC_R_RPU_WDT0,
    MXC_RPU_WDT1 = MXC_R_RPU_WDT1,
    MXC_RPU_WDT2 = MXC_R_RPU_WDT2,
    MXC_RPU_SMON = MXC_R_RPU_SMON,
    MXC_RPU_SIMO = MXC_R_RPU_SIMO,
    MXC_RPU_DVS = MXC_R_RPU_DVS,
    MXC_RPU_RTC = MXC_R_RPU_RTC,
    MXC_RPU_WUT = MXC_R_RPU_WUT,
    MXC_RPU_PWRSEQ = MXC_R_RPU_PWRSEQ,
    MXC_RPU_MCR = MXC_R_RPU_MCR,
    MXC_RPU_GPIO0 = MXC_R_RPU_GPIO0,
    MXC_RPU_GPIO1 = MXC_R_RPU_GPIO1,
    MXC_RPU_TMR0 = MXC_R_RPU_TMR0,
    MXC_RPU_TMR1 = MXC_R_RPU_TMR1,
    MXC_RPU_TMR2 = MXC_R_RPU_TMR2,
    MXC_RPU_TMR3 = MXC_R_RPU_TMR3,
    MXC_RPU_TMR4 = MXC_R_RPU_TMR4,
    MXC_RPU_TMR5 = MXC_R_RPU_TMR5,
    MXC_RPU_HTIMER0 = MXC_R_RPU_HTIMER0,
    MXC_RPU_HTIMER1 = MXC_R_RPU_HTIMER1,
    MXC_RPU_I2C0_BUS0 = MXC_R_RPU_I2C0_BUS0,
    MXC_RPU_I2C1_BUS0 = MXC_R_RPU_I2C1_BUS0,
    MXC_RPU_I2C2_BUS0 = MXC_R_RPU_I2C2_BUS0,
    MXC_RPU_SPIXFM = MXC_R_RPU_SPIXFM,
    MXC_RPU_SPIXFC = MXC_R_RPU_SPIXFC,
    MXC_RPU_DMA0 = MXC_R_RPU_DMA0,
    MXC_RPU_FLC1 = MXC_R_RPU_FLC1,
    MXC_RPU_ICC0 = MXC_R_RPU_ICC0,
    MXC_RPU_ICC1 = MXC_R_RPU_ICC1,
    MXC_RPU_SFCC = MXC_R_RPU_SFCC,
    MXC_RPU_SRCC = MXC_R_RPU_SRCC,
    MXC_RPU_ADC = MXC_R_RPU_ADC,
    MXC_RPU_DMA1 = MXC_R_RPU_DMA1,
    MXC_RPU_SDMA = MXC_R_RPU_SDMA,
    MXC_RPU_SPIXR = MXC_R_RPU_SPIXR,
    MXC_RPU_PTG_BUS0 = MXC_R_RPU_PTG_BUS0,
    MXC_RPU_OWM = MXC_R_RPU_OWM,
    MXC_RPU_SEMA = MXC_R_RPU_SEMA,
    MXC_RPU_UART0 = MXC_R_RPU_UART0,
    MXC_RPU_UART1 = MXC_R_RPU_UART1,
    MXC_RPU_UART2 = MXC_R_RPU_UART2,
    MXC_RPU_SPI1 = MXC_R_RPU_SPI1,
    MXC_RPU_SPI2 = MXC_R_RPU_SPI2,
    MXC_RPU_AUDIO = MXC_R_RPU_AUDIO,
    MXC_RPU_TRNG = MXC_R_RPU_TRNG,
    MXC_RPU_BTLE = MXC_R_RPU_BTLE,
    MXC_RPU_USBHS = MXC_R_RPU_USBHS,
    MXC_RPU_SDIO = MXC_R_RPU_SDIO,
    MXC_RPU_SPIXFM_FIFO = MXC_R_RPU_SPIXFM_FIFO,
    MXC_RPU_SPI0 = MXC_R_RPU_SPI0
} mxc_rpu_device_t;

/* **** Function Prototypes **** */
/**
 * @brief      Enable access to peripherals restricted by the RPU
 *             This function must be called from handler (privileged) mode
 * @param      periph the peripheral to allow access too
 * @param      allow_mask which bus masters to allow to access periph
 * @return     E_NO_ERROR If function is successful.
 */
int MXC_RPU_Allow(mxc_rpu_device_t periph, uint32_t allow_mask);

/**
 * @brief      Disable access to peripherals restricted by the RPU
 *             This function must be called from handler (privileged) mode
 * @param      periph the peripheral to revoke access too
 * @param      disallow_mask which bus masters to disallow access to periph
 * @return     E_NO_ERROR if function is successful.
 */
int MXC_RPU_Disallow(mxc_rpu_device_t periph, uint32_t disallow_mask);

/**
 * @brief      Check to see if this process is running in handler mode
 * @return     E_NO_RRROR If allowed.
 * @return     E_BAD_STATE If not allowed. 
 */
int MXC_RPU_IsAllowed(void);

/**@} end of group rpu */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_RPU_H_
