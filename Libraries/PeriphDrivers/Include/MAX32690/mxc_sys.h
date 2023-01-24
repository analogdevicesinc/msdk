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

/**
 * @file    mxc_sys.h
 * @brief   System level header file.
 */

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_MXC_SYS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_MXC_SYS_H_

#include "mxc_device.h"
#include "lpgcr_regs.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief System reset0 and reset1 enumeration. Used in MXC_SYS_PeriphReset0 function */
typedef enum {
    MXC_SYS_RESET0_DMA = MXC_F_GCR_RST0_DMA_POS, /**< Reset DMA */
    MXC_SYS_RESET0_WDT0 = MXC_F_GCR_RST0_WDT0_POS, /**< Reset WDT */
    MXC_SYS_RESET0_GPIO0 = MXC_F_GCR_RST0_GPIO0_POS, /**< Reset GPIO0 */
    MXC_SYS_RESET0_GPIO1 = MXC_F_GCR_RST0_GPIO1_POS, /**< Reset GPIO1 */
    MXC_SYS_RESET0_GPIO2 = MXC_F_GCR_RST0_GPIO2_POS, /**< Reset GPIO2 */
    MXC_SYS_RESET0_TMR0 = MXC_F_GCR_RST0_TMR0_POS, /**< Reset TMR0 */
    MXC_SYS_RESET0_TMR1 = MXC_F_GCR_RST0_TMR1_POS, /**< Reset TMR1 */
    MXC_SYS_RESET0_TMR2 = MXC_F_GCR_RST0_TMR2_POS, /**< Reset TMR2 */
    MXC_SYS_RESET0_TMR3 = MXC_F_GCR_RST0_TMR3_POS, /**< Reset TMR3 */
    MXC_SYS_RESET0_UART0 = MXC_F_GCR_RST0_UART0_POS, /**< Reset UART0 */
    MXC_SYS_RESET0_UART1 = MXC_F_GCR_RST0_UART1_POS, /**< Reset UART1 */
    MXC_SYS_RESET0_SPI0 = MXC_F_GCR_RST0_SPI0_POS, /**< Reset SPI0 */
    MXC_SYS_RESET0_SPI1 = MXC_F_GCR_RST0_SPI1_POS, /**< Reset SPI1 */
    MXC_SYS_RESET0_SPI2 = MXC_F_GCR_RST0_SPI2_POS, /**< Reset SPI2 */
    MXC_SYS_RESET0_I2C0 = MXC_F_GCR_RST0_I2C0_POS, /**< Reset I2C0 */
    MXC_SYS_RESET0_RTC = MXC_F_GCR_RST0_RTC_POS, /**< Reset RTC */
    MXC_SYS_RESET0_CRYPTO = MXC_F_GCR_RST0_CRYPTO_POS, /**< Reset CRYPTO */
    MXC_SYS_RESET0_CAN0 = MXC_F_GCR_RST0_CAN0_POS, /**< Reset CAN0 */
    MXC_SYS_RESET0_CAN1 = MXC_F_GCR_RST0_CAN1_POS, /**< Reset CAN1 */
    MXC_SYS_RESET0_HPB = MXC_F_GCR_RST0_HPB_POS, /**< Reset HPB */
    MXC_SYS_RESET0_SMPHR = MXC_F_GCR_RST0_SMPHR_POS, /**< Reset SMPHR */
    MXC_SYS_RESET0_USB = MXC_F_GCR_RST0_USB, /**< Reset USB */
    MXC_SYS_RESET0_TRNG = MXC_F_GCR_RST0_TRNG_POS, /**< Reset TRNG */
    MXC_SYS_RESET0_ADC = MXC_F_GCR_RST0_ADC_POS, /**< Reset ADC */
    MXC_SYS_RESET0_UART2 = MXC_F_GCR_RST0_UART2_POS, /**< Reset UART2 */
    MXC_SYS_RESET0_SOFT = MXC_F_GCR_RST0_SOFT_POS, /**< Soft reset */
    MXC_SYS_RESET0_PERIPH = MXC_F_GCR_RST0_PERIPH_POS, /**< Peripheral reset */
    MXC_SYS_RESET0_SYS = MXC_F_GCR_RST0_SYS_POS, /**< System reset */
    /* RESET1 Below this line we add 32 to separate RESET0 and RESET1 */
    MXC_SYS_RESET1_I2C1 = (MXC_F_GCR_RST1_I2C1_POS + 32), /**< Reset I2C1 */
    MXC_SYS_RESET1_PT = (MXC_F_GCR_RST1_PT_POS + 32), /**< Reset PT */
    MXC_SYS_RESET1_SPIXIP = (MXC_F_GCR_RST1_SPIXIP + 32), /**< Reset SPIXIP */
    MXC_SYS_RESET1_SPIXIPM = (MXC_F_GCR_RST1_SPIXIPM + 32), /**< Reset SPIXIPM */
    MXC_SYS_RESET1_OWM = (MXC_F_GCR_RST1_OWM_POS + 32), /**< Reset OWM */
    MXC_SYS_RESET1_SPI3 = (MXC_F_GCR_RST1_SPI3_POS + 32), /**< Reset SPI3 */
    MXC_SYS_RESET1_SPI4 = (MXC_F_GCR_RST1_SPI4_POS + 32), /**< Reset SPI4 */
    MXC_SYS_RESET1_SMPHR = (MXC_F_GCR_RST1_SMPHR_POS + 32), /**< Reset SMPHR */
    MXC_SYS_RESET1_BTLE = (MXC_F_GCR_RST1_BTLE_POS + 32), /**< Reset BTLE */
    MXC_SYS_RESET1_I2C2 = (MXC_F_GCR_RST1_I2C2_POS + 32), /**< Reset I2C2 */
    MXC_SYS_RESET1_I2S = (MXC_F_GCR_RST1_I2S_POS + 32), /**< Reset I2S*/
    MXC_SYS_RESET1_PUF = (MXC_F_GCR_RST1_PUF_POS + 32), /**< Reset PUF */
    MXC_SYS_RESET1_CPU1 = (MXC_F_GCR_RST1_CPU1_POS + 32), /**< Reset CPU1 */
    /* LPGCR RESET Below this line we add 64 to separate LPGCR and GCR */
    MXC_SYS_RESET_GPIO3 = (MXC_F_LPGCR_RST_GPIO3_POS + 64), /**< Reset GPIO3 */
    MXC_SYS_RESET_WDT1 = (MXC_F_LPGCR_RST_WDT1_POS + 64), /**< Reset WDT1 */
    MXC_SYS_RESET_TMR4 = (MXC_F_LPGCR_RST_TMR4_POS + 64), /**< Reset TMR4 */
    MXC_SYS_RESET_TMR5 = (MXC_F_LPGCR_RST_TMR5_POS + 64), /**< Reset TMR5 */
    MXC_SYS_RESET_UART3 = (MXC_F_LPGCR_RST_UART3_POS + 64), /**< Reset UART3 */
    MXC_SYS_RESET_LPCOMP = (MXC_F_LPGCR_RST_LPCOMP_POS + 64), /**< Reset LPCOMP */
} mxc_sys_reset_t;

/** @brief System clock disable enumeration. Used in MXC_SYS_ClockDisable and MXC_SYS_ClockEnable functions */
typedef enum {
    MXC_SYS_PERIPH_CLOCK_GPIO0 =
        MXC_F_GCR_PCLKDIS0_GPIO0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_GPIO0 clock */
    MXC_SYS_PERIPH_CLOCK_GPIO1 =
        MXC_F_GCR_PCLKDIS0_GPIO1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_GPIO1 clock */
    MXC_SYS_PERIPH_CLOCK_GPIO2 =
        MXC_F_GCR_PCLKDIS0_GPIO2_POS, /**< Disable MXC_F_GCR_PCLKDIS0_GPIO2 clock */
    MXC_SYS_PERIPH_CLOCK_USB =
        MXC_F_GCR_PCLKDIS0_USB_POS, /**< Disable MXC_F_GCR_PCLKDIS0_USB clock */
    MXC_SYS_PERIPH_CLOCK_DMA =
        MXC_F_GCR_PCLKDIS0_DMA_POS, /**< Disable MXC_F_GCR_PCLKDIS0_DMA clock */
    MXC_SYS_PERIPH_CLOCK_SPI0 =
        MXC_F_GCR_PCLKDIS0_SPI0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPI0 clock */
    MXC_SYS_PERIPH_CLOCK_SPI1 =
        MXC_F_GCR_PCLKDIS0_SPI1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPI1 clock */
    MXC_SYS_PERIPH_CLOCK_SPI2 =
        MXC_F_GCR_PCLKDIS0_SPI2_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPI2 clock */
    MXC_SYS_PERIPH_CLOCK_UART0 =
        MXC_F_GCR_PCLKDIS0_UART0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_UART0 clock */
    MXC_SYS_PERIPH_CLOCK_UART1 =
        MXC_F_GCR_PCLKDIS0_UART1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_UART1 clock */
    MXC_SYS_PERIPH_CLOCK_I2C0 =
        MXC_F_GCR_PCLKDIS0_I2C0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_I2C0 clock */
    MXC_SYS_PERIPH_CLOCK_CRYPTO =
        MXC_F_GCR_PCLKDIS0_CRYPTO_POS, /**< Disable MXC_F_GCR_PCLKDIS0_CRYPTO clock */
    MXC_SYS_PERIPH_CLOCK_TMR0 =
        MXC_F_GCR_PCLKDIS0_TMR0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR0 clock */
    MXC_SYS_PERIPH_CLOCK_TMR1 =
        MXC_F_GCR_PCLKDIS0_TMR1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR1 clock */
    MXC_SYS_PERIPH_CLOCK_TMR2 =
        MXC_F_GCR_PCLKDIS0_TMR2_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR2 clock */
    MXC_SYS_PERIPH_CLOCK_TMR3 =
        MXC_F_GCR_PCLKDIS0_TMR3_POS, /**< Disable MXC_F_GCR_PCLKDIS0_T3 clock */
    MXC_SYS_PERIPH_CLOCK_ADC =
        MXC_F_GCR_PCLKDIS0_ADC_POS, /**< Disable MXC_F_GCR_PCLKDIS0_ADC clock */
    MXC_SYS_PERIPH_CLOCK_I2C1 =
        MXC_F_GCR_PCLKDIS0_I2C1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_I2C1 clock */
    MXC_SYS_PERIPH_CLOCK_PT = MXC_F_GCR_PCLKDIS0_PT_POS, /**< Disable MXC_F_GCR_PCLKDIS0_PT clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIP =
        MXC_F_GCR_PCLKDIS0_SPIXIP_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPIXIP clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIPC =
        MXC_F_GCR_PCLKDIS0_SPIXIPC_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPIXIPC clock */
    /* PCLKDIS1 Below this line we add 32 to separate PCLKDIS0 and PCLKDIS1 */
    MXC_SYS_PERIPH_CLOCK_BTLE =
        (MXC_F_GCR_PCLKDIS1_BTLE_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_BTLE clock */
    MXC_SYS_PERIPH_CLOCK_UART2 =
        (MXC_F_GCR_PCLKDIS1_UART2_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_UART2 clock */
    MXC_SYS_PERIPH_CLOCK_TRNG =
        (MXC_F_GCR_PCLKDIS1_TRNG_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_TRNG clock */
    MXC_SYS_PERIPH_CLOCK_PUF =
        (MXC_F_GCR_PCLKDIS1_PUF_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_PUF clock */
    MXC_SYS_PERIPH_CLOCK_SMPHR =
        (MXC_F_GCR_PCLKDIS1_SMPHR_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_SMPHR clock */
    MXC_SYS_PERIPH_CLOCK_HPB =
        (MXC_F_GCR_PCLKDIS1_HPB_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_HPB clock */
    MXC_SYS_PERIPH_CLOCK_SYSCACHE =
        (MXC_F_GCR_PCLKDIS1_SYSCACHE_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_SYSCACHE clock */
    MXC_SYS_PERIPH_CLOCK_CAN0 =
        (MXC_F_GCR_PCLKDIS1_CAN0_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_CAN0 clock */
    MXC_SYS_PERIPH_CLOCK_OWIRE =
        (MXC_F_GCR_PCLKDIS1_OWM_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_OWM clock */
    MXC_SYS_PERIPH_CLOCK_SPI3 =
        (MXC_F_GCR_PCLKDIS1_SPI3_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_SPI3 clock */
    MXC_SYS_PERIPH_CLOCK_SPI4 =
        (MXC_F_GCR_PCLKDIS1_SPI4_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_SPI4 clock */
    MXC_SYS_PERIPH_CLOCK_CAN1 =
        (MXC_F_GCR_PCLKDIS1_CAN1_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_CAN1 clock */
    MXC_SYS_PERIPH_CLOCK_I2S =
        (MXC_F_GCR_PCLKDIS1_I2S_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_I2S clock */
    MXC_SYS_PERIPH_CLOCK_I2C2 =
        (MXC_F_GCR_PCLKDIS1_I2C2_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_I2C2 clock */
    MXC_SYS_PERIPH_CLOCK_WDT0 =
        (MXC_F_GCR_PCLKDIS1_WDT0_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_WDT0 clock */
    MXC_SYS_PERIPH_CLOCK_CPU1 =
        (MXC_F_GCR_PCLKDIS1_CPU1_POS + 32), /**< Disable MXC_F_GCR_PCLKDIS1_CPU1 clock */
    /* LPGCR PCLKDIS Below this line we add 64 to seperate GCR and LPGCR registers */
    MXC_SYS_PERIPH_CLOCK_GPIO3 =
        (MXC_F_LPGCR_PCLKDIS_GPIO3_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_GPIO3 clock */
    MXC_SYS_PERIPH_CLOCK_WDT1 =
        (MXC_F_LPGCR_PCLKDIS_WDT1_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_WDT2 clock */
    MXC_SYS_PERIPH_CLOCK_TMR4 =
        (MXC_F_LPGCR_PCLKDIS_TMR4_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_TMR4 clock */
    MXC_SYS_PERIPH_CLOCK_TMR5 =
        (MXC_F_LPGCR_PCLKDIS_TMR5_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_TMR5 clock */
    MXC_SYS_PERIPH_CLOCK_UART3 =
        (MXC_F_LPGCR_PCLKDIS_UART3_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_UART3 clock */
    MXC_SYS_PERIPH_CLOCK_LPCOMP =
        (MXC_F_LPGCR_PCLKDIS_LPCOMP_POS + 64), /**< Disable MXC_F_LPGCR_PCLKDIS_LPCOMP clock */
} mxc_sys_periph_clock_t;

/** @brief Enumeration to select System Clock source */
typedef enum {
    MXC_SYS_CLOCK_IPO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO,
    MXC_SYS_CLOCK_ERFO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO,
    MXC_SYS_CLOCK_IBRO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO,
    MXC_SYS_CLOCK_ISO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ISO,
    MXC_SYS_CLOCK_INRO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO,
    MXC_SYS_CLOCK_ERTCO = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO,
    MXC_SYS_CLOCK_EXTCLK = MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK
} mxc_sys_system_clock_t;

#define MXC_SYS_USN_CHECKSUM_LEN 16

/***** Function Prototypes *****/

/**
 * @brief Reads the device USN.
 * @param usn       Pointer to store the USN.
 * @param checksum  Optional pointer to store the AES checksum.
 * @returns       E_NO_ERROR if everything is successful.
 */
int MXC_SYS_GetUSN(uint8_t *usn, uint8_t *checksum);

/**
 * @brief Determines if the selected peripheral clock is enabled.
 * @param clock   Enumeration for desired clock.
 * @returns       0 is the clock is disabled, non 0 if the clock is enabled.
 */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock);

/**
 * @brief Disables the selected peripheral clock.
 * @param clock   Enumeration for desired clock.
 */
void MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock);

/**
 * @brief Enables the selected peripheral clock.
 * @param clock   Enumeration for desired clock.
 */
void MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock);

/**
 * @brief Enables the 32kHz oscillator
 * @param mxc_sys_cfg   Not used, may be NULL.
 */
void MXC_SYS_RTCClockEnable(void);

/**
 * @brief Disables the 32kHz oscillator
 * @returns         E_NO_ERROR if everything is successful
 */
int MXC_SYS_RTCClockDisable();

/**
 * @brief Enable System Clock Source without switching to it
 * @param      clock The clock to enable
 * @return     E_NO_ERROR if everything is successful
 */
int MXC_SYS_ClockSourceEnable(mxc_sys_system_clock_t clock);

/**
 * @brief Disable System Clock Source
 * @param      clock The clock to disable
 * @return     E_NO_ERROR if everything is successful
 */
int MXC_SYS_ClockSourceDisable(mxc_sys_system_clock_t clock);

/**
 * @brief Select the system clock.
 * @param clock     Enumeration for desired clock.
 * @param tmr       Optional tmr pointer for timeout. NULL if undesired.
 * @returns         E_NO_ERROR if everything is successful.
 */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock);

/**
 * @brief Wait for a clock to enable with timeout
 * @param      ready The clock to wait for
 * @return     E_NO_ERROR if ready, E_TIME_OUT if timeout
 */
int MXC_SYS_Clock_Timeout(uint32_t ready);
/**
 * @brief Reset the peripherals and/or CPU in the rstr0 or rstr1 register.
 * @param           Enumeration for what to reset. Can reset multiple items at once.
 */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset);

/**
 * @brief Setup and run RISCV core 
 */
void MXC_SYS_RISCVRun(void);

/**
 * @brief Shutdown the RISCV core 
 */
void MXC_SYS_RISCVShutdown(void);

/**
 * @brief Returns the clock rate (in Hz) of the Risc-V core.
 */
uint32_t MXC_SYS_RiscVClockRate(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_MXC_SYS_H_
