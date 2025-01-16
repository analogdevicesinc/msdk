/******************************************************************************
 *
 * Copyright (C) 2024-2025 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_SYS_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_SYS_H_

#include "mxc_device.h"
#include "gcr_regs.h"
#include "fcr_regs.h"
#include "mcr_regs.h"

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
    MXC_SYS_RESET0_DMA0 = MXC_F_GCR_RST0_DMA0_POS, /**< Reset Non-Secure DMA0 */
    MXC_SYS_RESET0_WDT = MXC_F_GCR_RST0_WDT_POS, /**< Reset WDT */
    MXC_SYS_RESET0_GPIO0 = MXC_F_GCR_RST0_GPIO0_POS, /**< Reset GPIO0 */
    MXC_SYS_RESET0_TMR0 = MXC_F_GCR_RST0_TMR0_POS, /**< Reset TMR0 */
    MXC_SYS_RESET0_TMR1 = MXC_F_GCR_RST0_TMR1_POS, /**< Reset TMR1 */
    MXC_SYS_RESET0_TMR2 = MXC_F_GCR_RST0_TMR2_POS, /**< Reset TMR2 */
    MXC_SYS_RESET0_TMR3 = MXC_F_GCR_RST0_TMR3_POS, /**< Reset TMR3 */
    MXC_SYS_RESET0_TMR4 = MXC_F_GCR_RST0_TMR4_POS, /**< Reset TMR4 */
    MXC_SYS_RESET0_TMR5 = MXC_F_GCR_RST0_TMR5_POS, /**< Reset TMR5 */
    MXC_SYS_RESET0_UART = MXC_F_GCR_RST0_UART_POS, /**< Reset UART0 */
    MXC_SYS_RESET0_SPI = MXC_F_GCR_RST0_SPI_POS, /**< Reset SPI */
    MXC_SYS_RESET0_I3C = MXC_F_GCR_RST0_I3C, /**< Reset I2C/I3C */
    MXC_SYS_RESET0_RTC = MXC_F_GCR_RST0_RTC_POS, /**< Reset RTC */
    MXC_SYS_RESET0_TRNG = MXC_F_GCR_RST0_TRNG_POS, /**< Reset TRNG */
    MXC_SYS_RESET0_DMA1 = MXC_F_GCR_RST0_DMA1_POS, /**< Reset Secure DMA1 */
    MXC_SYS_RESET0_SOFT = MXC_F_GCR_RST0_SOFT_POS, /**< Soft reset */
    MXC_SYS_RESET0_PERIPH = MXC_F_GCR_RST0_PERIPH_POS, /**< Peripheral reset */
    MXC_SYS_RESET0_SYS = MXC_F_GCR_RST0_SYS_POS, /**< System reset */
    /* RESET1 Below this line we add 32 to separate RESET0 and RESET1 */
    MXC_SYS_RESET1_CRC = (MXC_F_GCR_RST1_CRC_POS + 32), /**< Reset CRC */
    MXC_SYS_RESET1_AES = (MXC_F_GCR_RST1_AES_POS + 32), /**< Reset AES */
    MXC_SYS_RESET1_AUTOCAL = (MXC_F_GCR_RST1_AUTOCAL_POS + 32), /**< Reset AUTOCAL */
} mxc_sys_reset_t;

/** @brief System clock disable enumeration. Used in MXC_SYS_ClockDisable and MXC_SYS_ClockEnable functions */
typedef enum {
    MXC_SYS_PERIPH_CLOCK_GPIO0 = MXC_F_GCR_PCLKDIS0_GPIO0_POS, /**< Disable GPIO0 clock */
    MXC_SYS_PERIPH_CLOCK_DMA0 = MXC_F_GCR_PCLKDIS0_DMA0_POS, /**< Disable Non-Secure DMA0 clock */
    MXC_SYS_PERIPH_CLOCK_SPI = MXC_F_GCR_PCLKDIS0_SPI_POS, /**< Disable SPI clock */
    MXC_SYS_PERIPH_CLOCK_UART = MXC_F_GCR_PCLKDIS0_UART_POS, /**< Disable UART0 clock */
    MXC_SYS_PERIPH_CLOCK_I3C = MXC_F_GCR_PCLKDIS0_I3C_POS, /**< Disable I2C/I3C clock */
    MXC_SYS_PERIPH_CLOCK_TMR0 = MXC_F_GCR_PCLKDIS0_TMR0_POS, /**< Disable TMR0 clock */
    MXC_SYS_PERIPH_CLOCK_TMR1 = MXC_F_GCR_PCLKDIS0_TMR1_POS, /**< Disable TMR1 clock */
    MXC_SYS_PERIPH_CLOCK_TMR2 = MXC_F_GCR_PCLKDIS0_TMR2_POS, /**< Disable TMR2 clock */
    MXC_SYS_PERIPH_CLOCK_TMR3 = MXC_F_GCR_PCLKDIS0_TMR3_POS, /**< Disable TMR3 clock */
    MXC_SYS_PERIPH_CLOCK_TMR4 = MXC_F_GCR_PCLKDIS0_TMR4_POS, /**< Disable TMR4 clock */
    MXC_SYS_PERIPH_CLOCK_TMR5 = MXC_F_GCR_PCLKDIS0_TMR5_POS, /**< Disable TMR4 clock */
    /* PCLKDIS1 Below this line we add 32 to separate PCLKDIS0 and PCLKDIS1 */
    MXC_SYS_PERIPH_CLOCK_TRNG = (MXC_F_GCR_PCLKDIS1_TRNG_POS + 32), /**< Disable TRNG clock */
    MXC_SYS_PERIPH_CLOCK_CRC = (MXC_F_GCR_PCLKDIS1_CRC_POS + 32), /**< Disable CRC clock */
    MXC_SYS_PERIPH_CLOCK_AES = (MXC_F_GCR_PCLKDIS1_AES_POS + 32), /**< Disable AES clock */
    MXC_SYS_PERIPH_CLOCK_DMA1 =
        (MXC_F_GCR_PCLKDIS1_DMA1_POS + 32), /**< Disable Secure DMA1 clock */
    MXC_SYS_PERIPH_CLOCK_WDT = (MXC_F_GCR_PCLKDIS1_WDT_POS), /**< Disable WDT clock */
} mxc_sys_periph_clock_t;

/** @brief Enumeration to select System Clock source */
typedef enum {
    MXC_SYS_CLOCK_IPO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IPO, /**< Select the Internal Primary Oscillator (IPO) */
    MXC_SYS_CLOCK_IBRO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_IBRO, /**< Select the Internal Baud Rate Oscillator (IBRO) */
    MXC_SYS_CLOCK_ERFO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERFO, /**< Select the External RF Crystal Oscillator */
    MXC_SYS_CLOCK_INRO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_INRO, /**< Select the Internal Nanoring Oscillator (INRO) */
    MXC_SYS_CLOCK_ERTCO =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_ERTCO, /**< Select the External RTC Crystal Oscillator */
    MXC_SYS_CLOCK_EXTCLK =
        MXC_V_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK /**< Use the external system clock input */
} mxc_sys_system_clock_t;

/** @brief Enumeration to set the System Clock divider */
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

/** @brief Enumeration to select 32KHz Clock source used by the RTC, the timers, the wakeup timers, 
 * and the system clock */
typedef enum {
    MXC_SYS_32K_CLOCK_ERTCO =
        MXC_V_MCR_CTRL_CLKSEL_ERTCO, /**< Select the External Real-Time Clock Oscillator (ERTCO) */
    MXC_SYS_32K_CLOCK_INRO =
        MXC_V_MCR_CTRL_CLKSEL_INRO_DIV4, /**< Select the Internal Baud Rate Oscillator (INRO) */
    MXC_SYS_32K_CLOCK_RTC_IN =
        MXC_V_MCR_CTRL_CLKSEL_RTC_IN_DIV8, /**< Select the External clock input (RTC_CLK_IN) */
} mxc_sys_32k_clock_t;

/** @brief Compare clock enumeration. Used in MXC_SYS_ClockMeasure function. */
typedef enum {
    MXC_SYS_COMPARE_CLOCK_RTC = MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_RTC,
    MXC_SYS_COMPARE_CLOCK_EXT_GPIO = MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_EXT_GPIO,
    MXC_SYS_COMPARE_CLOCK_INRO = MXC_S_FCR_FRQCNTCTRL_CMP_CLKSEL_INRO
} mxc_sys_compare_clock_t;

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
 * @brief Selects the 32KHz clock source used by the RTC, the timers, the wakeup timers, 
 * and the system clock.
 * 
 * @param       clock Clock source to use. See @ref mxc_sys_32k_clock_t for options.
 * @return      E_NO_ERROR if everything is successful
 *              E_BAD_PARAM if the clock is not valid 
 */
int MXC_SYS_Select32KClockSource(mxc_sys_32k_clock_t clock);

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
 * @param clock     Enumeration for desired clock.  Note:  If using the external clock input be sure to define EXTCLK_FREQ correctly.
 *                  The default EXTCLK_FREQ value is defined in the system_max32655.h file and can be overridden at compile time.
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

/**
 * @brief This function PERMANENTLY locks the Debug Access Port.
 *
 * @warning After executing this function you will never be able
 *          to reprogram the target micro.
 */
int MXC_SYS_LockDAP_Permanent(void);

/**
 * @brief Measure the clock frequency, blocking.
 * 
 * @details Assumes that measurement clock and ERFO are enabled. 
 * Increasing compareClockTicks will provide a more accurate measurement, 
 * but there are limits that could cause overflow. Start and Get function
 * are used for non-blocking implementations.
 * 
 * @param clock Enumeration for which clock to measure.
 * @param compareClockTicks Number of ticks of the comparison clock to use for measurement.
 * @return clock frequency
 */
uint32_t MXC_SYS_ClockMeasure(mxc_sys_compare_clock_t clock, uint32_t compareClockTicks);

/**
 * @brief Start clock frequency measurement, non blocking.
 *
 * @param clock Enumeration for which clock to measure.
 * @param compareClockTicks Number of ticks of the comparison clock to use for measurement.
 */
void MXC_SYS_StartClockMeasure(mxc_sys_compare_clock_t clock, uint32_t compareClockTicks);

/**
 * @brief Get clock frequency measurement after calling start, non blocking.
 *
 * @param clock Enumeration for which clock to measure.
 * @param compareClockTicks Number of ticks of the comparison clock to use for measurement.
 * @return clock frequency, 0 if measurement is unfinished.
 */
uint32_t MXC_SYS_GetClockMeasure(void);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_MXC_SYS_H_
