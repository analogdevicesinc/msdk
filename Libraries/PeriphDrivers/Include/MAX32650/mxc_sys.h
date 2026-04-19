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

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_SYS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_SYS_H_

/* **** Includes **** */
#include "mxc_device.h"
#include "uart_regs.h"
#include "i2c_regs.h"
#include "pt_regs.h"
#include "ptg_regs.h"
#include "gcr_regs.h"
#include "tmr_regs.h"
#include "gpio.h"
#include "spimss_regs.h"
#include "sdhc_regs.h"
#include "spixfc_regs.h"
#include "spi_regs.h"
#include "wdt_regs.h"
#include "dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup mxc_sys System Configuration (MXC_SYS)
 * @ingroup syscfg
 * @details API for system configuration including clock source selection and entering critical sections of code.
 * @{
 */

#if defined(__CC_ARM) /* Suppressing the warning: "enum value is out of range of int" for Keil */
#pragma push
#pragma diag_suppress 66
#endif /* __CC_ARM */
/** @brief System reset0 enumeration. Used in SYS_PeriphReset0 function */
typedef enum {
    MXC_SYS_RESET_DMA = MXC_F_GCR_RST0_DMA_POS, /**< Reset DMA */
    MXC_SYS_RESET_WDT = MXC_F_GCR_RST0_WDT0_POS, /**< Reset WDT */
    MXC_SYS_RESET_GPIO0 = MXC_F_GCR_RST0_GPIO0_POS, /**< Reset GPIO0 */
    MXC_SYS_RESET_GPIO1 = MXC_F_GCR_RST0_GPIO1_POS, /**< Reset GPIO1 */
    MXC_SYS_RESET_GPIO2 = MXC_F_GCR_RST0_GPIO2_POS, /**< Reset GPIO2 */
    MXC_SYS_RESET_TIMER0 = MXC_F_GCR_RST0_TIMER0_POS, /**< Reset TIMER0 */
    MXC_SYS_RESET_TIMER1 = MXC_F_GCR_RST0_TIMER1_POS, /**< Reset TIMER1 */
    MXC_SYS_RESET_TIMER2 = MXC_F_GCR_RST0_TIMER2_POS, /**< Reset TIMER2 */
    MXC_SYS_RESET_TIMER3 = MXC_F_GCR_RST0_TIMER3_POS, /**< Reset TIMER3 */
    MXC_SYS_RESET_TIMER4 = MXC_F_GCR_RST0_TIMER4_POS, /**< Reset TIMER4 */
    MXC_SYS_RESET_TIMER5 = MXC_F_GCR_RST0_TIMER5_POS, /**< Reset TIMER5 */
    MXC_SYS_RESET_UART0 = MXC_F_GCR_RST0_UART0_POS, /**< Reset UART0 */
    MXC_SYS_RESET_UART1 = MXC_F_GCR_RST0_UART1_POS, /**< Reset UART1 */
    MXC_SYS_RESET_SPI0 = MXC_F_GCR_RST0_SPI0_POS, /**< Reset SPI0 */
    MXC_SYS_RESET_SPI1 = MXC_F_GCR_RST0_SPI1_POS, /**< Reset SPI1 */
    MXC_SYS_RESET_SPI2 = MXC_F_GCR_RST0_SPI2_POS, /**< Reset SPI2 */
    MXC_SYS_RESET_I2C0 = MXC_F_GCR_RST0_I2C0_POS, /**< Reset I2C0 */
    MXC_SYS_RESET_RTC = MXC_F_GCR_RST0_RTC_POS, /**< Reset RTC */
    MXC_SYS_RESET_TPU = MXC_F_GCR_RST0_TPU_POS, /**< Reset TPU */
    MXC_SYS_RESET_HBC = MXC_F_GCR_RST0_HBC_POS, /**< Reset HBC */
    MXC_SYS_RESET_USB = MXC_F_GCR_RST0_USB_POS, /**< Reset USB */
    MXC_SYS_RESET_TFT = MXC_F_GCR_RST0_TFT_POS, /**< Reset TRNG */
    MXC_SYS_RESET_ADC = MXC_F_GCR_RST0_ADC_POS, /**< Reset ADC */
    MXC_SYS_RESET_UART2 = MXC_F_GCR_RST0_UART2_POS, /**< Reset UART2 */
    MXC_SYS_RESET_SRST = MXC_F_GCR_RST0_SOFT_POS, /**< Soft reset */
    MXC_SYS_RESET_PRST = MXC_F_GCR_RST0_PERIPH_POS, /**< Peripheral reset */
    MXC_SYS_RESET_SYSTEM = MXC_F_GCR_RST0_SYS_POS, /**< System reset */
    /* RESET1 Below this line we add 32 to separate RESET0 and RESET1 */
    MXC_SYS_RESET_I2C1 = (MXC_F_GCR_RST1_I2C1_POS + 32), /**< Reset I2C1 */
    MXC_SYS_RESET_PT = (MXC_F_GCR_RST1_PT_POS + 32), /**< Reset PT */
    MXC_SYS_RESET_SPIXIP = (MXC_F_GCR_RST1_SPIXIP_POS + 32), /**< Reset SPIXIP */
    MXC_SYS_RESET_XSPIM = (MXC_F_GCR_RST1_XSPIM_POS + 32), /**< Reset XSPIM */
    MXC_SYS_RESET_GPIO3 = (MXC_F_GCR_RST1_GPIO3_POS + 32), /**< Reset GPIO3 */
    MXC_SYS_RESET_SDHC = (MXC_F_GCR_RST1_SDHC_POS + 32), /**< Reset SDHC */
    MXC_SYS_RESET_OWIRE = (MXC_F_GCR_RST1_OWIRE_POS + 32), /**< Reset OWIRE */
    MXC_SYS_RESET_WDT1 = (MXC_F_GCR_RST1_WDT1_POS + 32), /**< Reset WDT1 */
    MXC_SYS_RESET_SPI3 = (MXC_F_GCR_RST1_SPI3_POS + 32), /**< Reset SPI3 */
    MXC_SYS_RESET_I2S = (MXC_F_GCR_RST1_I2S_POS + 32), /**< Reset I2S */
    MXC_SYS_RESET_XIPR = (MXC_F_GCR_RST1_XIPR_POS + 32), /**< Reset SPIXMEM */
    MXC_SYS_RESET_SEMA = (MXC_F_GCR_RST1_SEMA_POS + 32), /**< Reset SMPHR */
} mxc_sys_reset_t;

/** @brief System clock disable enumeration. Used in SYS_ClockDisable and SYS_ClockEnable functions */
typedef enum {
    MXC_SYS_PERIPH_CLOCK_GPIO0 =
        MXC_F_GCR_PCLK_DIS0_GPIO0_POS, /**< Disable MXC_F_GCR_PERCKCN0_GPIO0D clock */
    MXC_SYS_PERIPH_CLOCK_GPIO1 =
        MXC_F_GCR_PCLK_DIS0_GPIO1_POS, /**< Disable MXC_F_GCR_PERCKCN0_GPIO1D clock */
    MXC_SYS_PERIPH_CLOCK_GPIO2 =
        MXC_F_GCR_PCLK_DIS0_GPIO2_POS, /**< Disable MXC_F_GCR_PERCKCN0_GPIO2D clock */
    MXC_SYS_PERIPH_CLOCK_USB =
        MXC_F_GCR_PCLK_DIS0_USB_POS, /**< Disable MXC_F_GCR_PERCKCN0_USBD clock */
    MXC_SYS_PERIPH_CLOCK_TFT =
        MXC_F_GCR_PCLK_DIS0_TFT_POS, /**< Disable MXC_F_GCR_PERCKCN0_CLCD clock */
    MXC_SYS_PERIPH_CLOCK_DMA =
        MXC_F_GCR_PCLK_DIS0_DMA_POS, /**< Disable MXC_F_GCR_PERCKCN0_DMAD clock */
    MXC_SYS_PERIPH_CLOCK_SPI0 =
        MXC_F_GCR_PCLK_DIS0_SPI0_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPI0D clock */
    MXC_SYS_PERIPH_CLOCK_SPI1 =
        MXC_F_GCR_PCLK_DIS0_SPI1_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPI1D clock */
    MXC_SYS_PERIPH_CLOCK_SPI2 =
        MXC_F_GCR_PCLK_DIS0_SPI2_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPI2D clock */
    MXC_SYS_PERIPH_CLOCK_UART0 =
        MXC_F_GCR_PCLK_DIS0_UART0_POS, /**< Disable MXC_F_GCR_PERCKCN0_UART0D clock */
    MXC_SYS_PERIPH_CLOCK_UART1 =
        MXC_F_GCR_PCLK_DIS0_UART1_POS, /**< Disable MXC_F_GCR_PERCKCN0_UART1D clock */
    MXC_SYS_PERIPH_CLOCK_I2C0 =
        MXC_F_GCR_PCLK_DIS0_I2C0_POS, /**< Disable MXC_F_GCR_PERCKCN0_I2C0D clock */
    MXC_SYS_PERIPH_CLOCK_TPU =
        MXC_F_GCR_PCLK_DIS0_TPU_POS, /**< Disable MXC_F_GCR_PERCKCN0_TPUD clock */
    MXC_SYS_PERIPH_CLOCK_TIMER0 =
        MXC_F_GCR_PCLK_DIS0_TIMER0_POS, /**< Disable MXC_F_GCR_PERCKCN0_T0D clock */
    MXC_SYS_PERIPH_CLOCK_TIMER1 =
        MXC_F_GCR_PCLK_DIS0_TIMER1_POS, /**< Disable MXC_F_GCR_PERCKCN0_T1D clock */
    MXC_SYS_PERIPH_CLOCK_TIMER2 =
        MXC_F_GCR_PCLK_DIS0_TIMER2_POS, /**< Disable MXC_F_GCR_PERCKCN0_T2D clock */
    MXC_SYS_PERIPH_CLOCK_TIMER3 =
        MXC_F_GCR_PCLK_DIS0_TIMER3_POS, /**< Disable MXC_F_GCR_PERCKCN0_T3D clock */
    MXC_SYS_PERIPH_CLOCK_TIMER4 =
        MXC_F_GCR_PCLK_DIS0_TIMER4_POS, /**< Disable MXC_F_GCR_PERCKCN0_T4D clock */
    MXC_SYS_PERIPH_CLOCK_TIMER5 =
        MXC_F_GCR_PCLK_DIS0_TIMER5_POS, /**< Disable MXC_F_GCR_PERCKCN0_T5D clock */
    MXC_SYS_PERIPH_CLOCK_ADC =
        MXC_F_GCR_PCLK_DIS0_ADC_POS, /**< Disable MXC_F_GCR_PERCKCN0_ADCD clock */
    MXC_SYS_PERIPH_CLOCK_I2C1 =
        MXC_F_GCR_PCLK_DIS0_I2C1_POS, /**< Disable MXC_F_GCR_PERCKCN0_I2C1D clock */
    MXC_SYS_PERIPH_CLOCK_PT =
        MXC_F_GCR_PCLK_DIS0_PT_POS, /**< Disable MXC_F_GCR_PERCKCN0_PTD clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIPF =
        MXC_F_GCR_PCLK_DIS0_SPIXIPF_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPIXIPD clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIPM =
        MXC_F_GCR_PCLK_DIS0_SPIXIPM_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPIMD clock */
    /* PERCKCN1 Below this line we add 32 to separate PERCKCN0 and PERCKCN1 */
    MXC_SYS_PERIPH_CLOCK_UART2 =
        (MXC_F_GCR_PCLK_DIS1_UART2_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_UART2D clock */
    MXC_SYS_PERIPH_CLOCK_TRNG =
        (MXC_F_GCR_PCLK_DIS1_TRNG_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_TRNGD clock */
    MXC_SYS_PERIPH_CLOCK_FLC =
        (MXC_F_GCR_PCLK_DIS1_SFLC_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_FLCD clock */
    MXC_SYS_PERIPH_CLOCK_HBC =
        (MXC_F_GCR_PCLK_DIS1_HBC_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_HBCD clock */
    MXC_SYS_PERIPH_CLOCK_GPIO3 =
        (MXC_F_GCR_PCLK_DIS1_GPIO3_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_GPIO3D clock */
    MXC_SYS_PERIPH_CLOCK_SCACHE =
        (MXC_F_GCR_PCLK_DIS1_SCACHE_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SCACHED clock */
    MXC_SYS_PERIPH_CLOCK_SDMA =
        (MXC_F_GCR_PCLK_DIS1_SDMA_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SDMAD clock */
    MXC_SYS_PERIPH_CLOCK_SEMA =
        (MXC_F_GCR_PCLK_DIS1_SEMA_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SMPHRD clock */
    MXC_SYS_PERIPH_CLOCK_SDHC =
        (MXC_F_GCR_PCLK_DIS1_SDHC_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SDHCD clock */
    MXC_SYS_PERIPH_CLOCK_ICACHE =
        (MXC_F_GCR_PCLK_DIS1_ICACHE_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_ICACHED clock */
    MXC_SYS_PERIPH_CLOCK_ICACHEXIP = (MXC_F_GCR_PCLK_DIS1_ICACHEXIPF_POS +
                                      32), /**<Disable MXC_F_GCR_PERCKCN1_ICACHEXIPD clock */
    MXC_SYS_PERIPH_CLOCK_OWIRE =
        (MXC_F_GCR_PCLK_DIS1_OW_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_OWIRED clock */
    MXC_SYS_PERIPH_CLOCK_SPI3 =
        (MXC_F_GCR_PCLK_DIS1_SPI3_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SPI3D clock */
    MXC_SYS_PERIPH_CLOCK_I2S =
        (MXC_F_GCR_PCLK_DIS1_I2S_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_I2SD clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIPR =
        (MXC_F_GCR_PCLK_DIS1_SPIXIPR_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SPIXIPDD clock */
} mxc_sys_periph_clock_t;

typedef enum {
    MXC_SYS_CLOCK_CRYPTO = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_CRYPTO,
    MXC_SYS_CLOCK_NANORING = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_NANORING,
    MXC_SYS_CLOCK_HIRC96 = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC96,
    MXC_SYS_CLOCK_HIRC8 = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HIRC8,
    MXC_SYS_CLOCK_HFXIN = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_HFXIN,
    MXC_SYS_CLOCK_X32K = MXC_V_GCR_CLK_CTRL_SYSOSC_SEL_X32K,
} mxc_sys_system_clock_t;

typedef enum {
    MXC_SYS_SYSTEM_DIV_1 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV1,
    MXC_SYS_SYSTEM_DIV_2 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV2,
    MXC_SYS_SYSTEM_DIV_4 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV4,
    MXC_SYS_SYSTEM_DIV_8 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV8,
    MXC_SYS_SYSTEM_DIV_16 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV16,
    MXC_SYS_SYSTEM_DIV_32 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV32,
    MXC_SYS_SYSTEM_DIV_64 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV64,
    MXC_SYS_SYSTEM_DIV_128 = MXC_S_GCR_CLK_CTRL_SYSCLK_PRESCALE_DIV128,
} mxc_sys_system_div_t;

typedef struct {
    uint8_t scache_flag;
    uint8_t crypto_flag;
} mxc_sys_spid_cfg_t;

/** @brief Enable control */
typedef enum { Disable, Enable } mxc_sys_control_t;

typedef void *mxc_sys_cfg_t;

/** @brief System Cache System Configuration Object */
typedef mxc_sys_cfg_t mxc_sys_cfg_scache_t;

/** @brief USB High Speed System Configuration Object */
typedef mxc_sys_cfg_t mxc_sys_cfg_usbhs_t;

#if defined(__CC_ARM) /* Restore the warning: "enum is out of int range" for Keil */
#pragma pop
#endif /* __CC_ARM */

#define MXC_SYS_SCACHE_CLK 1 // Enable SCACHE CLK
#define MXC_SYS_TPU_CLK 1 // Enable TPU CLK
#define MXC_SYS_USN_LEN 13 // Size of the USN

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
 * @brief      Determines if the selected peripheral clock is enabled.
 * @param      clock   Enumeration for desired clock.
 * @returns    0 is the clock is disabled, non 0 if the clock is enabled.
 */
int MXC_SYS_IsClockEnabled(mxc_sys_periph_clock_t clock);

/**
 * @brief      Disables the selected peripheral clock.
 * @param      clock   Enumeration for desired clock.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_ClockDisable(mxc_sys_periph_clock_t clock);

/**
 * @brief      Enables the selected peripheral clock.
 * @param      clock   Enumeration for desired clock.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_ClockEnable(mxc_sys_periph_clock_t clock);

/**
 * @brief      Enables the external 32k oscillator.
 * @param      rtc  rtc system configuration settings. NULL if undesired.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_RTCClockEnable(void);

/**
 * @brief      Disables the external 32k oscillator.
 * 
 * @returns    #E_NO_ERROR if everything is successful.
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
 * @brief      Select the system clock.
 * @param      clock     Enumeration for desired clock.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_Clock_Select(mxc_sys_system_clock_t clock);

/**
 * @brief Select the system clock divider.
 * @param clock     Enumeration for desired system clock divider.
 */
void MXC_SYS_Clock_Div(mxc_sys_system_div_t div);

/**
 * @brief Wait for a clock to enable with timeout
 * @param      ready The clock to wait for
 * @return     E_NO_ERROR if ready, E_TIME_OUT if timeout
 */
int MXC_SYS_Clock_Timeout(uint32_t ready);

/**
 * @brief      Reset the peripherals and/or CPU in the rstr0 register.
 * @param      reset  Enumeration for what to reset. Can reset multiple items at once.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_Reset_Periph(mxc_sys_reset_t reset);

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
 * @brief      System level initialization for SCHACE module.
 * @param      sys_cfg  System configuration object
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_SCACHE_Init(const mxc_sys_cfg_scache_t *sys_cfg);

/**
 * @brief      System level Shutdown for SCACHE module.
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_SCACHE_Shutdown(void);

/**
 * @brief       System-level initialization for the USBHS module
 * @param       sys_cfg  System configuration object
 * @returns     E_NO_ERROR upon success, or appropriate failure code
 */
int MXC_SYS_USBHS_Init(const mxc_sys_cfg_usbhs_t *sys_cfg);

/**
 * @brief       System-level shutdown for the USBHS module
 * @returns     #E_NO_ERROR upon success, or appropriate failure code
 */
int MXC_SYS_USBHS_Shutdown(void);

/**
 * @brief      System Tick Configuration Helper
 *
 *             The function enables selection of the external clock source for
 *             the System Tick Timer. It initializes the System Timer and its
 *             interrupt, and starts the System Tick Timer. Counter is in free
 *             running mode to generate periodic interrupts.
 *
 * @param      ticks    Number of ticks between two interrupts.
 * @param      clk_src  Selects between default SystemClock or External Clock.
 *                      - 0 Use external clock source
 *                      - 1 SystemClock
 * @param tmr  Optional tmr pointer for timeout. NULL if undesired.
 * @return     #E_NO_ERROR  Function succeeded.
 * @return     #E_INVALID   Invalid reload value requested.
 *
 */
int MXC_SYS_SysTick_Config(uint32_t ticks, int clk_src, mxc_tmr_regs_t *tmr);

/**
 * @brief      Disable System Tick timer
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_SysTick_Disable(void);

/**
 * @brief      Delay a requested number of SysTick Timer Ticks.
 * @param      ticks  Number of System Ticks to delay.
 * @note       This delay function is based on the clock used for the SysTick
 *             timer if the SysTick timer is enabled. If the SysTick timer is
 *             not enabled, the current SysTick registers are saved and the
 *             timer will use the SystemClock as the source for the delay. The
 *             delay is measured in clock ticks and is not based on the SysTick
 *             interval.
 * @return     #E_NO_ERROR if everything is successful
 */
int MXC_SYS_SysTick_Delay(uint32_t ticks);

/**
 * @brief      Get the frequency of the SysTick Timer
 * @return     frequency in Hz
 */
uint32_t MXC_SYS_SysTick_GetFreq(void);

/**
 * @brief      Delay a requested number of microseconds.
 * @param      us    Number of microseconds to delay.
 * @note       Calls SYS_SysTick_Delay().
 * @returns    #E_NO_ERROR if everything is successful.
 */
int MXC_SYS_SysTick_DelayUs(uint32_t us);

/**
 * @brief This function PERMANENTLY locks the Debug Access Port.
 *
 * @warning After executing this function you will never be able
 *          to reprogram the target micro.
 */
int MXC_SYS_LockDAP_Permanent(void);

/**@} end of group mxc_sys */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_MXC_SYS_H_
