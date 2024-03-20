/**
 * @file    mxc_sys.h
 * @brief   System level header file.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_SYS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_SYS_H_

#include "mxc_device.h"
#include "gcr_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup mxc_sys System Configuration (MXC_SYS)
 * @ingroup syscfg
 * @details API for system configuration including clock source selection and entering critical sections of code.
 * @{
 */

/** @brief System reset0 and reset1 enumeration. Used in MXC_SYS_PeriphReset0 function */
typedef enum {
    MXC_SYS_RESET0_DMA = MXC_F_GCR_RST0_DMA_POS, /**< Reset DMA */
    MXC_SYS_RESET0_WDT = MXC_F_GCR_RST0_WDT0_POS, /**< Reset WDT */
    MXC_SYS_RESET0_GPIO0 = MXC_F_GCR_RST0_GPIO0_POS, /**< Reset GPIO0 */
    MXC_SYS_RESET0_GPIO1 = MXC_F_GCR_RST0_GPIO1_POS, /**< Reset GPIO1 */
    MXC_SYS_RESET0_GPIO2 = MXC_F_GCR_RST0_GPIO2_POS, /**< Reset GPIO2 */
    MXC_SYS_RESET0_TMR0 = MXC_F_GCR_RST0_TMR0_POS, /**< Reset TMR0 */
    MXC_SYS_RESET0_TMR1 = MXC_F_GCR_RST0_TMR1_POS, /**< Reset TMR1 */
    MXC_SYS_RESET0_TMR2 = MXC_F_GCR_RST0_TMR2_POS, /**< Reset TMR2 */
    MXC_SYS_RESET0_TMR3 = MXC_F_GCR_RST0_TMR3_POS, /**< Reset TMR3 */
    MXC_SYS_RESET0_TMR4 = MXC_F_GCR_RST0_TMR4_POS, /**< Reset TMR4 */
    MXC_SYS_RESET0_TMR5 = MXC_F_GCR_RST0_TMR5_POS, /**< Reset TMR5 */
    MXC_SYS_RESET0_TMR6 = MXC_F_GCR_RST0_TMR6_POS, /**< Reset TMR6 */
    MXC_SYS_RESET0_TMR7 = MXC_F_GCR_RST0_TMR7_POS, /**< Reset TMR7 */
    MXC_SYS_RESET0_UART0 = MXC_F_GCR_RST0_UART0_POS, /**< Reset UART0 */
    MXC_SYS_RESET0_UART1 = MXC_F_GCR_RST0_UART1_POS, /**< Reset UART1 */
    MXC_SYS_RESET0_SPI0 = MXC_F_GCR_RST0_SPI0_POS, /**< Reset SPI0 */
    MXC_SYS_RESET0_SPI1 = MXC_F_GCR_RST0_SPI1_POS, /**< Reset SPI1 */
    MXC_SYS_RESET0_SPI2 = MXC_F_GCR_RST0_SPI2_POS, /**< Reset SPI2 */
    MXC_SYS_RESET0_I2C0 = MXC_F_GCR_RST0_I2C0_POS, /**< Reset I2C0 */
    MXC_SYS_RESET0_RTC = MXC_F_GCR_RST0_RTC_POS, /**< Reset RTC */
    MXC_SYS_RESET0_CTB = MXC_F_GCR_RST0_CRYPTO_POS, /**< Reset CRYPTO */
    MXC_SYS_RESET0_CLCD = MXC_F_GCR_RST0_CLCD_POS, /**< Reset CLCD */
    MXC_SYS_RESET0_USB = MXC_F_GCR_RST0_USB_POS, /**< Reset USB */
    MXC_SYS_RESET0_ADC = MXC_F_GCR_RST0_ADC_POS, /**< Reset ADC */
    MXC_SYS_RESET0_UART2 = MXC_F_GCR_RST0_UART2_POS, /**< Reset UART2 */
    MXC_SYS_RESET0_SRST = MXC_F_GCR_RST0_SOFT_POS, /**< Soft reset */
    MXC_SYS_RESET0_PRST = MXC_F_GCR_RST0_PERIPH_POS, /**< Peripheral reset */
    MXC_SYS_RESET0_SYS = MXC_F_GCR_RST0_SYS_POS, /**< System reset */
    /* RESET1 Below this line we add 32 to separate RESET0 and RESET1 */
    MXC_SYS_RESET1_I2C1 = (MXC_F_GCR_RST1_I2C1_POS + 32), /**< Reset I2C1 */
    MXC_SYS_RESET1_PT = (MXC_F_GCR_RST1_PT_POS + 32), /**< Reset PT */
    MXC_SYS_RESET1_SPIXIP = (MXC_F_GCR_RST1_SPIXIP_POS + 32), /**< Reset SPIXIP */
    MXC_SYS_RESET1_XSPIM = (MXC_F_GCR_RST1_XSPIM_POS + 32), /**< Reset XSPIM */
    MXC_SYS_RESET1_SDHC = (MXC_F_GCR_RST1_SDHC_POS + 32), /**< Reset SDHC */
    MXC_SYS_RESET1_GPIO3 = (MXC_F_GCR_RST1_GPIO3_POS + 32), /**< Reset GPIO3 */
    MXC_SYS_RESET1_OWIRE = (MXC_F_GCR_RST1_OWIRE_POS + 32), /**< Reset OWIRE */
    MXC_SYS_RESET1_WDT1 = (MXC_F_GCR_RST1_WDT1_POS + 32), /**< Reset WDT1 */
    MXC_SYS_RESET1_SPI3 = (MXC_F_GCR_RST1_SPI3_POS + 32), /**< Reset SPI3 */
    MXC_SYS_RESET1_SPIXMEM = (MXC_F_GCR_RST1_SPIXMEM_POS + 32), /**< Reset SPIXMEM */
    MXC_SYS_RESET1_I2C2 = (MXC_F_GCR_RST1_I2C2_POS + 32), /**< Reset I2C2*/
    MXC_SYS_RESET1_UART3 = (MXC_F_GCR_RST1_UART3_POS + 32), /**< Reset UART3*/
    MXC_SYS_RESET1_UART4 = (MXC_F_GCR_RST1_UART4_POS + 32), /**< Reset USRT4*/
    MXC_SYS_RESET1_UART5 = (MXC_F_GCR_RST1_UART5_POS + 32), /**< Reset UART5*/
    MXC_SYS_RESET1_KBD = (MXC_F_GCR_RST1_KBD_POS + 32), /**< Reset KBD*/
    MXC_SYS_RESET1_ADC9 = (MXC_F_GCR_RST1_ADC9_POS + 32), /**< Reset ADC9*/
    MXC_SYS_RESET1_SC0 = (MXC_F_GCR_RST1_SC0_POS + 32), /**< Reset SC0*/
    MXC_SYS_RESET1_SC1 = (MXC_F_GCR_RST1_SC1_POS + 32), /**< Reset SC1*/
    MXC_SYS_RESET1_NFC = (MXC_F_GCR_RST1_NFC_POS + 32), /**< Reset NFC*/
    MXC_SYS_RESET1_EMAC = (MXC_F_GCR_RST1_EMAC_POS + 32), /**< Reset EMAC*/
    MXC_SYS_RESET1_PCIF = (MXC_F_GCR_RST1_PCIF_POS + 32), /**< Reset PCIF*/
    MXC_SYS_RESET1_HTMR0 = (MXC_F_GCR_RST1_HTMR0_POS + 32), /**< Reset HTMR0*/
    MXC_SYS_RESET1_HTMR1 = (MXC_F_GCR_RST1_HTMR1_POS + 32), /**< Reset HTMR1*/
    MXC_SYS_RESET1_AC = (MXC_F_GCR_RST1_AC_POS + 32), /**< Reset AC*/
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
    MXC_SYS_PERIPH_CLOCK_CLCD =
        MXC_F_GCR_PCLKDIS0_CLCD_POS, /**< Disable MXC_F_GCR_PCLKDIS0_CLC clock */
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
    MXC_SYS_PERIPH_CLOCK_CTB =
        MXC_F_GCR_PCLKDIS0_CRYPTO_POS, /**< Disable MXC_F_GCR_PCLKDIS0_CRYPTO clock */
    MXC_SYS_PERIPH_CLOCK_TMR0 =
        MXC_F_GCR_PCLKDIS0_TMR0_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR0 clock */
    MXC_SYS_PERIPH_CLOCK_TMR1 =
        MXC_F_GCR_PCLKDIS0_TMR1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR1 clock */
    MXC_SYS_PERIPH_CLOCK_TMR2 =
        MXC_F_GCR_PCLKDIS0_TMR2_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR2 clock */
    MXC_SYS_PERIPH_CLOCK_TMR3 =
        MXC_F_GCR_PCLKDIS0_TMR3_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR3 clock */
    MXC_SYS_PERIPH_CLOCK_TMR4 =
        MXC_F_GCR_PCLKDIS0_TMR4_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR4 clock */
    MXC_SYS_PERIPH_CLOCK_TMR5 =
        MXC_F_GCR_PCLKDIS0_TMR5_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR5 clock */
    MXC_SYS_PERIPH_CLOCK_TMR6 =
        MXC_F_GCR_PCLKDIS0_TMR6_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR6 clock */
    MXC_SYS_PERIPH_CLOCK_TMR7 =
        MXC_F_GCR_PCLKDIS0_TMR7_POS, /**< Disable MXC_F_GCR_PCLKDIS0_TMR7 clock */
    MXC_SYS_PERIPH_CLOCK_KBD =
        MXC_F_GCR_PCLKDIS0_KBD_POS, /**< Disable MXC_F_GCR_PCLKDIS0_ADC clock */
    MXC_SYS_PERIPH_CLOCK_ADC =
        MXC_F_GCR_PCLKDIS0_ADC_POS, /**< Disable MXC_F_GCR_PCLKDIS0_ADC clock */
    MXC_SYS_PERIPH_CLOCK_HTMR0 =
        MXC_F_GCR_PCLKDIS0_HTMR0_POS, /**< Disable MXC_F_GCR_PCLKDIS1_HTMR0 clock */
    MXC_SYS_PERIPH_CLOCK_HTMR1 =
        MXC_F_GCR_PCLKDIS0_HTMR1_POS, /**< Disable MXC_F_GCR_PCLKDIS1_HTMR1 clock */
    MXC_SYS_PERIPH_CLOCK_I2C1 =
        MXC_F_GCR_PCLKDIS0_I2C1_POS, /**< Disable MXC_F_GCR_PCLKDIS0_I2C1 clock */
    MXC_SYS_PERIPH_CLOCK_PT = MXC_F_GCR_PCLKDIS0_PT_POS, /**< Disable MXC_F_GCR_PCLKDIS0_PT clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIP =
        MXC_F_GCR_PCLKDIS0_SPIXIP_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPIXIP clock */
    MXC_SYS_PERIPH_CLOCK_SPIXFC =
        MXC_F_GCR_PCLKDIS0_SPIM_POS, /**< Disable MXC_F_GCR_PCLKDIS0_SPIM clock */
    /* PCLKDIS1 Below this line we add 32 to separate PCLKDIS0 and PCLKDIS1 */
    MXC_SYS_PERIPH_CLOCK_UART2 =
        (MXC_F_GCR_PCLKDIS1_UART2_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_UART2 clock */
    MXC_SYS_PERIPH_CLOCK_TRNG =
        (MXC_F_GCR_PCLKDIS1_TRNG_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_TRNG clock */
    MXC_SYS_PERIPH_CLOCK_WDT0 =
        (MXC_F_GCR_PCLKDIS1_WDT0_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_WDT0 clock */
    MXC_SYS_PERIPH_CLOCK_WDT1 =
        (MXC_F_GCR_PCLKDIS1_WDT1_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_WDT1 clock */
    MXC_SYS_PERIPH_CLOCK_GPIO3 =
        (MXC_F_GCR_PCLKDIS1_GPIO3_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_GPIO3 clock */
    MXC_SYS_PERIPH_CLOCK_SCACHE =
        (MXC_F_GCR_PCLKDIS1_SCACHE_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SCACHE clock */
    MXC_SYS_PERIPH_CLOCK_HA0 =
        (MXC_F_GCR_PCLKDIS1_HA0_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_HA0 clock */
    MXC_SYS_PERIPH_CLOCK_SDHC =
        (MXC_F_GCR_PCLKDIS1_SDHC_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SDHC clock */
    MXC_SYS_PERIPH_CLOCK_ICACHE =
        (MXC_F_GCR_PCLKDIS1_ICC0_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_ICACHE clock */
    MXC_SYS_PERIPH_CLOCK_ICACHEXIP =
        (MXC_F_GCR_PCLKDIS1_ICCXIP_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_ICACHEXIP clock */
    MXC_SYS_PERIPH_CLOCK_OWIRE =
        (MXC_F_GCR_PCLKDIS1_OWIRE_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_OWIRE clock */
    MXC_SYS_PERIPH_CLOCK_SPI3 =
        (MXC_F_GCR_PCLKDIS1_SPI3_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SPI3 clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIPD =
        (MXC_F_GCR_PCLKDIS1_SPIXIP_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SPIXIP clock */
    MXC_SYS_PERIPH_CLOCK_I2C2 =
        (MXC_F_GCR_PCLKDIS1_I2C2_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_I2C2 clock */
    MXC_SYS_PERIPH_CLOCK_UART3 =
        (MXC_F_GCR_PCLKDIS1_UART3_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_UART3 clock */
    MXC_SYS_PERIPH_CLOCK_UART4 =
        (MXC_F_GCR_PCLKDIS1_UART4_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_UART4 clock */
    MXC_SYS_PERIPH_CLOCK_UART5 =
        (MXC_F_GCR_PCLKDIS1_UART5_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_UART5 clock */
    MXC_SYS_PERIPH_CLOCK_ADC9 =
        (MXC_F_GCR_PCLKDIS1_ADC9_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_ADC9 clock */
    MXC_SYS_PERIPH_CLOCK_SC0 =
        (MXC_F_GCR_PCLKDIS1_SC0_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SC0 clock */
    MXC_SYS_PERIPH_CLOCK_SC1 =
        (MXC_F_GCR_PCLKDIS1_SC1_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_SC1 clock */
    MXC_SYS_PERIPH_CLOCK_NFC =
        (MXC_F_GCR_PCLKDIS1_NFC_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_NFC clock */
    MXC_SYS_PERIPH_CLOCK_EMAC =
        (MXC_F_GCR_PCLKDIS1_EMAC_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_EMAC clock */
    MXC_SYS_PERIPH_CLOCK_HA1 =
        (MXC_F_GCR_PCLKDIS1_HA1_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_HA1 clock */
    MXC_SYS_PERIPH_CLOCK_PCIF =
        (MXC_F_GCR_PCLKDIS1_PCIF_POS + 32), /**<Disable MXC_F_GCR_PCLKDIS1_PCIF clock */
} mxc_sys_periph_clock_t;

/** @brief Enumeration to select System Clock source */
typedef enum {
    MXC_SYS_CLOCK_IPO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO, /**< Select the Internal Primary Oscillator (IPO) */
    MXC_SYS_CLOCK_IBRO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO, /**< Select the Internal Baud Rate Oscillator (IBRO) */
    MXC_SYS_CLOCK_ISO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ISO, /**< Select the Internal Secondary Oscillator (ISO) */
    MXC_SYS_CLOCK_ERFO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO, /**< Select the External RF Crystal Oscillator */
    MXC_SYS_CLOCK_INRO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO, /**< Select the Internal Nanoring Oscillator (INRO) */
    MXC_SYS_CLOCK_ERTCO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO, /**< Select the External RTC Crystal Oscillator */
} mxc_sys_system_clock_t;

typedef enum {
    MXC_SYS_CLOCK_DIV_1 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV1,
    MXC_SYS_CLOCK_DIV_2 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV2,
    MXC_SYS_CLOCK_DIV_4 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV4,
    MXC_SYS_CLOCK_DIV_8 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV8,
    MXC_SYS_CLOCK_DIV_16 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV16,
    MXC_SYS_CLOCK_DIV_32 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV32,
    MXC_SYS_CLOCK_DIV_64 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV64,
    MXC_SYS_CLOCK_DIV_128 = MXC_S_GCR_CLKCTRL_SYSCLK_DIV_DIV128
} mxc_sys_system_clock_div_t;

#define MXC_SYS_SCACHE_CLK 1 // Enable SCACHE CLK
#define MXC_SYS_CTB_CLK 1 // Enable CTB CLK

/***** Function Prototypes *****/

typedef struct {
    int ie_status;
    int in_critical;
} mxc_crit_state_t;

static mxc_crit_state_t _state = { .ie_status = (int)0xFFFFFFFF, .in_critical = 0 };

static inline void _mxc_crit_get_state(void)
{
#ifndef __riscv
    /*
        On ARM M the 0th bit of the Priority Mask register indicates
        whether interrupts are enabled or not.

        0 = enabled
        1 = disabled
    */
    uint32_t primask = __get_PRIMASK();
    _state.ie_status = (primask == 0);
#else
    /*
        On RISC-V bit position 3 (Machine Interrupt Enable) of the
        mstatus register indicates whether interrupts are enabled.

        0 = disabled
        1 = enabled
    */
    uint32_t mstatus = get_mstatus();
    _state.ie_status = ((mstatus & (1 << 3)) != 0);
#endif
}

/**
 * @brief Enter a critical section of code that cannot be interrupted.  Call @ref MXC_SYS_Crit_Exit to exit the critical section.
 * @details Ex:
 * @code
 * MXC_SYS_Crit_Enter();
 * printf("Hello critical section!\n");
 * MXC_SYS_Crit_Exit();
 * @endcode
 * The @ref MXC_CRITICAL macro is also provided as a convencience macro for wrapping a code section in this way.
 * @returns None
 */
static inline void MXC_SYS_Crit_Enter(void)
{
    _mxc_crit_get_state();
    if (_state.ie_status)
        __disable_irq();
    _state.in_critical = 1;
}

/**
 * @brief Exit a critical section of code from @ref MXC_SYS_Crit_Enter
 * @returns None
 */
static inline void MXC_SYS_Crit_Exit(void)
{
    if (_state.ie_status) {
        __enable_irq();
    }
    _state.in_critical = 0;
    _mxc_crit_get_state();
    /*
        ^ Reset the state again to prevent edge case
        where interrupts get disabled, then Crit_Exit() gets
        called, which would inadvertently re-enable interrupts
        from old state.
    */
}

/**
 * @brief Polls whether code is currently executing from a critical section.
 * @returns 1 if code is currently in a critical section (interrupts are disabled).
 *          0 if code is not in a critical section.
 */
static inline int MXC_SYS_In_Crit_Section(void)
{
    return _state.in_critical;
}

// clang-format off
/**
 * @brief Macro for wrapping a section of code to make it critical (interrupts disabled).  Note: this macro
 * does not support nesting.
 * @details
 * Ex:
 * \code
 * MXC_CRITICAL(
 *      printf("Hello critical section!\n");
 * )
 * \endcode
 * This macro places a call to @ref MXC_SYS_Crit_Enter before the code, and a call to @ref MXC_SYS_Crit_Exit after.
 * @param code The code section to wrap.
 */
#define MXC_CRITICAL(code) {\
    MXC_SYS_Crit_Enter();\
    code;\
    MXC_SYS_Crit_Exit();\
}
// clang-format on

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
int MXC_SYS_RTCClockDisable(void);

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
 * @returns         E_NO_ERROR if everything is successful.
 */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock);

/**
 * @brief Set the system clock divider.
 * @param div       Enumeration for desired clock divider.
 */
void MXC_SYS_SetClockDiv(mxc_sys_system_clock_div_t div);

/**
 * @brief Get the system clock divider.
 * @returns         System clock divider.
 */
mxc_sys_system_clock_div_t MXC_SYS_GetClockDiv(void);

/**
 * @brief Wait for a clock to enable with timeout
 * @param      ready The clock to wait for
 * @return     E_NO_ERROR if ready, E_TIME_OUT if timeout
 */
int MXC_SYS_Clock_Timeout(uint32_t ready);

/**
 * @brief Reset the peripherals and/or CPU in the RST0 or RST1 register.
 * @param           Enumeration for what to reset. Can reset multiple items at once.
 */
void MXC_SYS_Reset_Periph(mxc_sys_reset_t reset);

/**
 * @brief      Get the revision of the chip
 * @returns    the chip revision
 */
uint8_t MXC_SYS_GetRev(void);

/**
 * @brief      Get the USN of the chip
 * @param      serialNumber buffer to store the USN
 * @param      len          length of the USN buffer
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_GetUSN(uint8_t *serialNumber, int len);

/**
 * @brief This function PERMANENTLY locks the Debug Access Port.
 *
 * @warning After executing this function you will never be able
 *          to reprogram the target micro.
 */
int MXC_SYS_LockDAP_Permanent(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32570_MXC_SYS_H_
