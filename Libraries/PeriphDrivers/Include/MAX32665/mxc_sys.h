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

/**
 * @file    mxc_sys.h
 * @brief   System level header file.
 */

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_SYS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_SYS_H_

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

/** @brief System reset0 and reset1 enumeration. Used in SYS_PeriphReset0 function */
typedef enum {
    MXC_SYS_RESET_DMA0 = MXC_F_GCR_RSTR0_DMA_POS, /**< Reset DMA */
    MXC_SYS_RESET_WDT0 = MXC_F_GCR_RSTR0_WDT0_POS, /**< Reset WDT */
    MXC_SYS_RESET_GPIO0 = MXC_F_GCR_RSTR0_GPIO0_POS, /**< Reset GPIO0 */
    MXC_SYS_RESET_GPIO1 = MXC_F_GCR_RSTR0_GPIO1_POS, /**< Reset GPIO1 */
    MXC_SYS_RESET_TIMER0 = MXC_F_GCR_RSTR0_TIMER0_POS, /**< Reset TIMER0 */
    MXC_SYS_RESET_TIMER1 = MXC_F_GCR_RSTR0_TIMER1_POS, /**< Reset TIMER1 */
    MXC_SYS_RESET_TIMER2 = MXC_F_GCR_RSTR0_TIMER2_POS, /**< Reset TIMER2 */
    MXC_SYS_RESET_TIMER3 = MXC_F_GCR_RSTR0_TIMER3_POS, /**< Reset TIMER3 */
    MXC_SYS_RESET_TIMER4 = MXC_F_GCR_RSTR0_TIMER4_POS, /**< Reset TIMER4 */
    MXC_SYS_RESET_TIMER5 = MXC_F_GCR_RSTR0_TIMER5_POS, /**< Reset TIMER5 */
    MXC_SYS_RESET_UART0 = MXC_F_GCR_RSTR0_UART0_POS, /**< Reset UART0 */
    MXC_SYS_RESET_UART1 = MXC_F_GCR_RSTR0_UART1_POS, /**< Reset UART1 */
    MXC_SYS_RESET_SPI1 = MXC_F_GCR_RSTR0_SPI1_POS, /**< Reset SPI0 */
    MXC_SYS_RESET_SPI2 = MXC_F_GCR_RSTR0_SPI2_POS, /**< Reset SPI1 */
    MXC_SYS_RESET_I2C0 = MXC_F_GCR_RSTR0_I2C0_POS, /**< Reset I2C0 */
    MXC_SYS_RESET_RTC = MXC_F_GCR_RSTR0_RTC_POS, /**< Reset RTC */
    MXC_SYS_RESET_CRYPTO = MXC_F_GCR_RSTR0_CRYPTO_POS, /**< Reset CRYPTO */
    MXC_SYS_RESET_SMPHR = MXC_F_GCR_RSTR0_SMPHR_POS, /**< Reset SMPHR */
    MXC_SYS_RESET_USB = MXC_F_GCR_RSTR0_USB_POS, /**< Reset USB */
    //  MXC_SYS_RESET_TRNG      = MXC_F_GCR_RSTR0_TRNG_POS,        /**< Reset TRNG */
    MXC_SYS_RESET_ADC = MXC_F_GCR_RSTR0_ADC_POS, /**< Reset ADC */
    MXC_SYS_RESET_DMA1 = MXC_F_GCR_RSTR0_DMA1_POS, /**< Reset DMA1 */
    MXC_SYS_RESET_UART2 = MXC_F_GCR_RSTR0_UART2_POS, /**< Reset UART2 */
    MXC_SYS_RESET_SRST = MXC_F_GCR_RSTR0_SRST_POS, /**< Soft reset */
    MXC_SYS_RESET_PRST = MXC_F_GCR_RSTR0_PRST_POS, /**< Peripheral reset */
    MXC_SYS_RESET_SYSTEM = MXC_F_GCR_RSTR0_SYSTEM_POS, /**< System reset */
    /* RESET1 Below this line we add 32 to separate RESET0 and RESET1 */
    MXC_SYS_RESET_I2C1 = (MXC_F_GCR_RSTR1_I2C1_POS + 32), /**< Reset I2C1 */
    MXC_SYS_RESET_PT = (MXC_F_GCR_RSTR1_PT_POS + 32), /**< Reset PT */
    MXC_SYS_RESET_SPIXIP = (MXC_F_GCR_RSTR1_SPIXIP_POS + 32), /**< Reset SPIXIP */
    MXC_SYS_RESET_XSPIM = (MXC_F_GCR_RSTR1_XSPIM_POS + 32), /**< Reset XSPIM */
    MXC_SYS_RESET_SDHC = (MXC_F_GCR_RSTR1_SDHC_POS + 32), /**< Reset SDHC */
    MXC_SYS_RESET_OWIRE = (MXC_F_GCR_RSTR1_OWIRE_POS + 32), /**< Reset OWIRE */
    MXC_SYS_RESET_WDT1 = (MXC_F_GCR_RSTR1_WDT1_POS + 32), /**< Reset WDT1 */
    MXC_SYS_RESET_SPI0 = (MXC_F_GCR_RSTR1_SPI0_POS + 32), /**< Reset SPI2 */
    MXC_SYS_RESET_SPIXMEM = (MXC_F_GCR_RSTR1_SPIXMEM_POS + 32), /**< Reset SPIXMEM */
    MXC_SYS_RESET_SEMA = (MXC_F_GCR_RSTR1_SMPHR_POS + 32), /**< Reset SEMA */
    MXC_SYS_RESET_WDT2 = (MXC_F_GCR_RSTR1_WDT2_POS + 32), /**< Reset WDT1 */
    MXC_SYS_RESET_BTLE = (MXC_F_GCR_RSTR1_BTLE_POS + 32), /**< Reset BTLE */
    MXC_SYS_RESET_AUDIO = (MXC_F_GCR_RSTR1_AUDIO_POS + 32), /**< Reset BTLE */
    MXC_SYS_RESET_RPU = (MXC_F_GCR_RSTR1_RPU_POS + 32), /**< Reset BTLE */
    MXC_SYS_RESET_I2C2 = (MXC_F_GCR_RSTR1_I2C2_POS + 32), /**< Reset BTLE */
    MXC_SYS_RESET_HTMR0 = (MXC_F_GCR_RSTR1_HTMR0_POS + 32), /**< Reset HTMR0 */
    MXC_SYS_RESET_HTMR1 = (MXC_F_GCR_RSTR1_HTMR1_POS + 32), /**< Reset HTMR1 */
    MXC_SYS_RESET_DVS = (MXC_F_GCR_RSTR1_DVS_POS + 32), /**< Reset DVS */
    MXC_SYS_RESET_SIMO = (MXC_F_GCR_RSTR1_SIMO_POS + 32), /**< Reset SIMO */
} mxc_sys_reset_t;

/** @brief System clock disable enumeration. Used in SYS_ClockDisable and SYS_ClockEnable functions */
typedef enum {
    MXC_SYS_PERIPH_CLOCK_GPIO0 =
        MXC_F_GCR_PERCKCN0_GPIO0D_POS, /**< Disable MXC_F_GCR_PERCKCN0_GPIO0D clock */
    MXC_SYS_PERIPH_CLOCK_GPIO1 =
        MXC_F_GCR_PERCKCN0_GPIO1D_POS, /**< Disable MXC_F_GCR_PERCKCN0_GPIO1D clock */
    MXC_SYS_PERIPH_CLOCK_USB =
        MXC_F_GCR_PERCKCN0_USBD_POS, /**< Disable MXC_F_GCR_PERCKCN0_USBD clock */
    MXC_SYS_PERIPH_CLOCK_DMA =
        MXC_F_GCR_PERCKCN0_DMAD_POS, /**< Disable MXC_F_GCR_PERCKCN0_DMAD clock */
    MXC_SYS_PERIPH_CLOCK_SPI1 =
        MXC_F_GCR_PERCKCN0_SPI1D_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPI1D clock */
    MXC_SYS_PERIPH_CLOCK_SPI2 =
        MXC_F_GCR_PERCKCN0_SPI2D_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPI2D clock */
    MXC_SYS_PERIPH_CLOCK_UART0 =
        MXC_F_GCR_PERCKCN0_UART0D_POS, /**< Disable MXC_F_GCR_PERCKCN0_UART0D clock */
    MXC_SYS_PERIPH_CLOCK_UART1 =
        MXC_F_GCR_PERCKCN0_UART1D_POS, /**< Disable MXC_F_GCR_PERCKCN0_UART1D clock */
    MXC_SYS_PERIPH_CLOCK_I2C0 =
        MXC_F_GCR_PERCKCN0_I2C0D_POS, /**< Disable MXC_F_GCR_PERCKCN0_I2C0D clock */
    MXC_SYS_PERIPH_CLOCK_TPU =
        MXC_F_GCR_PERCKCN0_CRYPTOD_POS, /**< Disable MXC_F_GCR_PERCKCN0_CRYPTOD clock */
    MXC_SYS_PERIPH_CLOCK_T0 =
        MXC_F_GCR_PERCKCN0_TIMER0D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T0D clock */
    MXC_SYS_PERIPH_CLOCK_T1 =
        MXC_F_GCR_PERCKCN0_TIMER1D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T1D clock */
    MXC_SYS_PERIPH_CLOCK_T2 =
        MXC_F_GCR_PERCKCN0_TIMER2D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T2D clock */
    MXC_SYS_PERIPH_CLOCK_T3 =
        MXC_F_GCR_PERCKCN0_TIMER3D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T3D clock */
    MXC_SYS_PERIPH_CLOCK_T4 =
        MXC_F_GCR_PERCKCN0_TIMER4D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T4D clock */
    MXC_SYS_PERIPH_CLOCK_T5 =
        MXC_F_GCR_PERCKCN0_TIMER5D_POS, /**< Disable MXC_F_GCR_PERCKCN0_T5D clock */
    MXC_SYS_PERIPH_CLOCK_ADC =
        MXC_F_GCR_PERCKCN0_ADCD_POS, /**< Disable MXC_F_GCR_PERCKCN0_ADCD clock */
    MXC_SYS_PERIPH_CLOCK_I2C1 =
        MXC_F_GCR_PERCKCN0_I2C1D_POS, /**< Disable MXC_F_GCR_PERCKCN0_I2C1D clock */
    MXC_SYS_PERIPH_CLOCK_PT =
        MXC_F_GCR_PERCKCN0_PTD_POS, /**< Disable MXC_F_GCR_PERCKCN0_PTD clock */
    MXC_SYS_PERIPH_CLOCK_SPIXIP =
        MXC_F_GCR_PERCKCN0_SPIXIPD_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPIXIPD clock */
    MXC_SYS_PERIPH_CLOCK_SPIXFC =
        MXC_F_GCR_PERCKCN0_SPIMD_POS, /**< Disable MXC_F_GCR_PERCKCN0_SPIMD clock */
    /* PERCKCN1 Below this line we add 32 to separate PERCKCN0 and PERCKCN1 */
    MXC_SYS_PERIPH_CLOCK_BTLE = (MXC_F_GCR_PERCKCN1_BTLED_POS + 32),
    MXC_SYS_PERIPH_CLOCK_UART2 =
        (MXC_F_GCR_PERCKCN1_UART2D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_UART2D clock */
    MXC_SYS_PERIPH_CLOCK_TRNG =
        (MXC_F_GCR_PERCKCN1_TRNGD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_TRNGD clock */
    MXC_SYS_PERIPH_CLOCK_SCACHE =
        (MXC_F_GCR_PERCKCN1_SCACHED_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SCACHED clock */
    MXC_SYS_PERIPH_CLOCK_SDMA =
        (MXC_F_GCR_PERCKCN1_SDMAD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SDMAD clock */
    MXC_SYS_PERIPH_CLOCK_SMPHR =
        (MXC_F_GCR_PERCKCN1_SMPHRD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SMPHRD clock */
    MXC_SYS_PERIPH_CLOCK_SDHC =
        (MXC_F_GCR_PERCKCN1_SDHCD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SDHCD clock */
    MXC_SYS_PERIPH_CLOCK_ICACHEXIP =
        (MXC_F_GCR_PERCKCN1_ICACHEXIPD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_ICACHEXIPD clock */
    MXC_SYS_PERIPH_CLOCK_OWIRE =
        (MXC_F_GCR_PERCKCN1_OWIRED_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_OWIRED clock */
    MXC_SYS_PERIPH_CLOCK_SPI0 =
        (MXC_F_GCR_PERCKCN1_SPI0D_POS + 32), /**<Disable QSPI Clock (API Calls QSPI SPI0) */
    MXC_SYS_PERIPH_CLOCK_SPIXIPD =
        (MXC_F_GCR_PERCKCN1_SPIXIPDD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_SPIXIPDD clock */
    MXC_SYS_PERIPH_CLOCK_DMA1 =
        (MXC_F_GCR_PERCKCN1_DMA1D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_DMA1D clock */
    MXC_SYS_PERIPH_CLOCK_AUDIO =
        (MXC_F_GCR_PERCKCN1_AUDIOD_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_AUDIOD clock */
    MXC_SYS_PERIPH_CLOCK_I2C2 =
        (MXC_F_GCR_PERCKCN1_I2C2D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_I2C2D clock */
    MXC_SYS_PERIPH_CLOCK_HTMR0 =
        (MXC_F_GCR_PERCKCN1_HTMR0D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_HTMR0D clock */
    MXC_SYS_PERIPH_CLOCK_HTMR1 =
        (MXC_F_GCR_PERCKCN1_HTMR1D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_HTMR1D clock */
    MXC_SYS_PERIPH_CLOCK_WDT0 =
        (MXC_F_GCR_PERCKCN1_WDT0D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_WTD0D clock */
    MXC_SYS_PERIPH_CLOCK_WDT1 =
        (MXC_F_GCR_PERCKCN1_WDT1D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_WTD1D clock */
    MXC_SYS_PERIPH_CLOCK_WDT2 =
        (MXC_F_GCR_PERCKCN1_WDT2D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_WTD2D clock */
    MXC_SYS_PERIPH_CLOCK_CPU1 =
        (MXC_F_GCR_PERCKCN1_CPU1D_POS + 32), /**<Disable MXC_F_GCR_PERCKCN1_CPU1D clock */
} mxc_sys_periph_clock_t;

typedef enum {
    MXC_SYS_SYSTEM_DIV_1 = MXC_S_GCR_CLKCN_PSC_DIV1,
    MXC_SYS_SYSTEM_DIV_2 = MXC_S_GCR_CLKCN_PSC_DIV2,
    MXC_SYS_SYSTEM_DIV_4 = MXC_S_GCR_CLKCN_PSC_DIV4,
    MXC_SYS_SYSTEM_DIV_8 = MXC_S_GCR_CLKCN_PSC_DIV8,
    MXC_SYS_SYSTEM_DIV_16 = MXC_S_GCR_CLKCN_PSC_DIV16,
    MXC_SYS_SYSTEM_DIV_32 = MXC_S_GCR_CLKCN_PSC_DIV32,
    MXC_SYS_SYSTEM_DIV_64 = MXC_S_GCR_CLKCN_PSC_DIV64,
    MXC_SYS_SYSTEM_DIV_128 = MXC_S_GCR_CLKCN_PSC_DIV128,
} mxc_sys_system_div_t;

typedef enum {
    MXC_SYS_CLOCK_HIRC96 = MXC_V_GCR_CLKCN_CLKSEL_HIRC96,
    MXC_SYS_CLOCK_HIRC8 = MXC_V_GCR_CLKCN_CLKSEL_HIRC8,
    MXC_SYS_CLOCK_HIRC = MXC_V_GCR_CLKCN_CLKSEL_HIRC,
    MXC_SYS_CLOCK_XTAL32M = MXC_V_GCR_CLKCN_CLKSEL_XTAL32M,
    MXC_SYS_CLOCK_LIRC8K = MXC_V_GCR_CLKCN_CLKSEL_LIRC8,
    MXC_SYS_CLOCK_XTAL32K = MXC_V_GCR_CLKCN_CLKSEL_XTAL32K,
} mxc_sys_system_clock_t;

#define MXC_SYS_SCACHE_CLK 1 // Enable SCACHE CLK
#define MXC_SYS_CTB_CLK 1 // Enable CTB CLK

#define MXC_SYS_USN_CHECKSUM_LEN 16 // Length of the USN + padding for checksum compute
#define MXC_SYS_USN_CSUM_FIELD_LEN 2 // Size of the checksum field in the USN
#define MXC_SYS_USN_LEN 13 // Size of the USN including the checksum

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
 * @brief Reads the device USN and verifies the checksum.
 * @param usn       Pointer to store the USN. Array must be at least MXC_SYS_USN_LEN bytes long.
 * @param checksum  Optional pointer to store the AES checksum. If not NULL, checksum is verified with AES engine.
 * @returns         E_NO_ERROR if everything is successful.
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
 * @param tmr       Optional tmr pointer for timeout. NULL if undesired.
 * @returns         E_NO_ERROR if everything is successful.
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
 * @brief This function PERMANENTLY locks the Debug Access Port.
 *
 * @warning After executing this function you will never be able
 *          to reprogram the target micro.
 */
int MXC_SYS_LockDAP_Permanent(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32665_MXC_SYS_H_
