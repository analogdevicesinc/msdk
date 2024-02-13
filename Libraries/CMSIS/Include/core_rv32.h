/*
 * Copyright (c) 2009-2020 Arm Limited. All rights reserved.
 *
 * Portions Copyright (C) 2022-2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LIBRARIES_CMSIS_INCLUDE_CORE_RV32_H_
#define LIBRARIES_CMSIS_INCLUDE_CORE_RV32_H_

#if defined(__ICCARM__)
#pragma system_include /* treat file as system include file for MISRA check */
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __CORE_CM4_H_GENERIC
#define __CORE_CM4_H_GENERIC

/** \page CMSIS_MISRA_Exceptions  MISRA-C:2004 Compliance Exceptions
  CMSIS violates the following MISRA-C:2004 rules:

   \li Required Rule 8.5, object/function definition in header file.<br>
     Function definitions in header files are used to allow 'inlining'.

   \li Required Rule 18.4, declaration of union type or object of union type: '{...}'.<br>
     Unions are used for effective representation of core registers.

   \li Advisory Rule 19.7, Function-like macro defined.<br>
     Function-like macros are used to allow more efficient code.
 */

/*******************************************************************************
 *                 CMSIS definitions
 ******************************************************************************/
/** \ingroup Cortex_M4
  @{
 */

#if defined(__CC_ARM)
// #define __ASM            __asm                                      /*!< asm keyword for ARM Compiler          */
// #define __INLINE         __inline                                   /*!< inline keyword for ARM Compiler       */
#define __STATIC_INLINE static __inline

#elif defined(__ICCARM__)
// #define __ASM            __asm                                      /*!< asm keyword for IAR Compiler          */
// #define __INLINE         inline                                     /*!< inline keyword for IAR Compiler. Only available in High optimization mode! */
#define __STATIC_INLINE static inline

#elif defined(__TMS470__)
// #define __ASM            __asm                                      /*!< asm keyword for TI CCS Compiler       */
#define __STATIC_INLINE static inline

#elif defined(__GNUC__)
// #define __ASM            __asm                                      /*!< asm keyword for GNU Compiler          */
// #define __INLINE         inline                                     /*!< inline keyword for GNU Compiler       */
#define __STATIC_INLINE static inline

#elif defined(__TASKING__)
// #define __ASM            __asm                                      /*!< asm keyword for TASKING Compiler      */
// #define __INLINE         inline                                     /*!< inline keyword for TASKING Compiler   */
#define __STATIC_INLINE static inline

#endif

/** __FPU_USED indicates whether an FPU is used or not. For this, __FPU_PRESENT has to be checked prior to making use of FPU specific registers and functions.
*/
#if defined(__CC_ARM)
#if defined __TARGET_FPU_VFP
#if (__FPU_PRESENT == 1)
#define __FPU_USED 1
#else
#warning "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
#define __FPU_USED 0
#endif
#else
#define __FPU_USED 0
#endif

#elif defined(__ICCARM__)
#if defined __ARMVFP__
#if (__FPU_PRESENT == 1)
#define __FPU_USED 1
#else
#warning "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
#define __FPU_USED 0
#endif
#else
#define __FPU_USED 0
#endif

#elif defined(__TMS470__)
#if defined __TI_VFP_SUPPORT__
#if (__FPU_PRESENT == 1)
#define __FPU_USED 1
#else
#warning "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
#define __FPU_USED 0
#endif
#else
#define __FPU_USED 0
#endif

#elif defined(__GNUC__)
#if defined(__VFP_FP__) && !defined(__SOFTFP__)
#if (__FPU_PRESENT == 1)
#define __FPU_USED 1
#else
#warning "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
#define __FPU_USED 0
#endif
#else
#define __FPU_USED 0
#endif

#elif defined(__TASKING__)
#if defined __FPU_VFP__
#if (__FPU_PRESENT == 1)
#define __FPU_USED 1
#else
#error "Compiler generates FPU instructions for a device without an FPU (check __FPU_PRESENT)"
#define __FPU_USED 0
#endif
#else
#define __FPU_USED 0
#endif
#endif

#include <stdint.h> /* standard types definitions                      */
//#include <core_cmInstr.h>                /* Core Instruction Access                         */
//#include <core_cmFunc.h>                 /* Core Function Access                            */
//#include <core_cm4_simd.h>               /* Compiler specific SIMD Intrinsics               */

#endif /* __CORE_CM4_H_GENERIC */

#ifndef __CMSIS_GENERIC

#ifndef __CORE_CM4_H_DEPENDENT
#define __CORE_CM4_H_DEPENDENT

/* check device defines and use defaults */
#if defined __CHECK_DEVICE_DEFINES
#ifndef __CM4_REV
#define __CM4_REV 0x0000
#warning "__CM4_REV not defined in device header file; using default!"
#endif

#ifndef __FPU_PRESENT
#define __FPU_PRESENT 0
#warning "__FPU_PRESENT not defined in device header file; using default!"
#endif

#ifndef __MPU_PRESENT
#define __MPU_PRESENT 0
#warning "__MPU_PRESENT not defined in device header file; using default!"
#endif

#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS 4
#warning "__NVIC_PRIO_BITS not defined in device header file; using default!"
#endif

#ifndef __Vendor_SysTickConfig
#define __Vendor_SysTickConfig 0
#warning "__Vendor_SysTickConfig not defined in device header file; using default!"
#endif
#endif

/* IO definitions (access restrictions to peripheral registers) */
/**
    \defgroup CMSIS_glob_defs CMSIS Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.
*/
#ifdef __cplusplus
#define __I volatile /*!< Defines 'read only' permissions                 */
#else
#define __I volatile const /*!< Defines 'read only' permissions                 */
#endif
#define __O volatile /*!< Defines 'write only' permissions                */
#define __IO volatile /*!< Defines 'read / write' permissions              */

/*@} end of group Cortex_M4 */

__attribute__((always_inline)) __STATIC_INLINE uint32_t get_mstatus(void)
{
    uint32_t retval;
    __asm volatile("csrr %0, mstatus" : "=r"(retval));
    return retval;
}

__attribute__((always_inline)) __STATIC_INLINE uint32_t get_mtvec(void)
{
    uint32_t retval;
    __asm volatile("csrr %0, mtvec" : "=r"(retval));
    return retval;
}

__attribute__((always_inline)) __STATIC_INLINE uint32_t get_mepc(void)
{
    uint32_t retval;
    __asm volatile("csrr %0, mepc" : "=r"(retval));
    return retval;
}

__attribute__((always_inline)) __STATIC_INLINE uint32_t get_mcause(void)
{
    uint32_t retval;
    __asm volatile("csrr %0, mcause" : "=r"(retval));
    return retval;
}

__attribute__((always_inline)) __STATIC_INLINE uint32_t get_uepc(void)
{
    uint32_t retval;
    __asm volatile("csrr %0, uepc" : "=r"(retval));
    return retval;
}

/** \brief  No Operation

    No Operation does nothing. This instruction can be used for code alignment purposes.
 */
__attribute__((always_inline)) __STATIC_INLINE void __NOP(void)
{
    __asm volatile("nop");
}

/** \brief  Wait For Interrupt

    Wait For Interrupt is a hint instruction that suspends execution
    until one of a number of events occurs.
 */
__attribute__((always_inline)) __STATIC_INLINE void __WFI(void)
{
    __asm volatile("wfi");
}

/*******************************************************************************
 *                 Register Abstraction
  Core Register contain:
  - Core NVIC Register

  // MLB - These have been purged from RISC V
  - Core Register
  - Core SCB Register
  - Core SysTick Register
  - Core Debug Register
  - Core MPU Register
  - Core FPU Register
 ******************************************************************************/

/** \ingroup    CMSIS_core_register
    \defgroup   CMSIS_NVIC  Nested Vectored Interrupt Controller (NVIC)
    \brief      Type definitions for the NVIC Registers
  @{
 */

/**
 * @brief Structure type to access the GCR Registers.
 */
typedef struct {
    __IO uint32_t irq0_enable; /**< <tt>\b 0x00:<\tt> */
    __IO uint32_t irq0_pending; /**< <tt>\b 0x04:<\tt>  */
    __IO uint32_t irq0_set_pending; /**< <tt>\b 0x08:<\tt>  */
    __IO uint32_t irq0_clear_pending; /**< <tt>\b 0x0c:<\tt>  */
    __IO uint32_t irq1_enable; /**< <tt>\b 0x10:<\tt> */
    __IO uint32_t irq1_pending; /**< <tt>\b 0x14:<\tt>  */
    __IO uint32_t irq1_set_pending; /**< <tt>\b 0x18:<\tt>  */
    __IO uint32_t irq1_clear_pending; /**< <tt>\b 0x1c:<\tt>  */
} mxc_intr_regs_t;

typedef struct {
    __IO uint32_t event0_enable; /**< <tt>\b 0x20:<\tt>*/
    __IO uint32_t event0_pending; /**< <tt>\b 0x24:<\tt> */
    __IO uint32_t event0_set_pending; /**< <tt>\b 0x28:<\tt> */
    __IO uint32_t event0_clear_pending; /**< <tt>\b 0x2C:<\tt> */
    __IO uint32_t event1_enable; /**< <tt>\b 0x30:<\tt>*/
    __IO uint32_t event1_pending; /**< <tt>\b 0x34:<\tt> */
    __IO uint32_t event1_set_pending; /**< <tt>\b 0x38:<\tt> */
    __IO uint32_t event1_clear_pending; /**< <tt>\b 0x3C:<\tt> */
} mxc_event_regs_t;

typedef struct {
    __IO uint32_t sleep_ctrl; /**< <tt>\b 0x40:<\tt>  */
    __IO uint32_t sleep_status; /**< <tt>\b 0x44:<\tt>  */
} mxc_sleep_regs_t;

/* Software Triggered Interrupt Register Definitions */
#define NVIC_STIR_INTID_Pos 0 /*!< STIR: INTLINESNUM Position */
#define NVIC_STIR_INTID_Msk (0x1FFUL << NVIC_STIR_INTID_Pos) /*!< STIR: INTLINESNUM Mask */

/*@} end of group CMSIS_NVIC */

/* For MCUs with extra RISCV core (RISCV1) */
#ifdef __riscv1
#define MXC_BASE_INTR ((uint32_t)0xE5170000UL)
#define MXC_BASE_EVENT ((uint32_t)0xE5170020UL)
#define MXC_BASE_SLEEP ((uint32_t)0xE5170040UL)
#else
#define MXC_BASE_INTR ((uint32_t)0xE5070000UL)
#define MXC_BASE_EVENT ((uint32_t)0xE5070020UL)
#define MXC_BASE_SLEEP ((uint32_t)0xE5070040UL)
#endif

#define MXC_INTR ((mxc_intr_regs_t *)MXC_BASE_INTR)
#define MXC_EVENT ((mxc_event_regs_t *)MXC_BASE_EVENT)
#define MXC_SLEEP ((mxc_sleep_regs_t *)MXC_BASE_SLEEP)

/*******************************************************************************
 *                Hardware Abstraction Layer
  Core Function Interface contains:
  - Core NVIC Functions

  // MLB - These have been purged in RISC V
  - Core SysTick Functions
  - Core Debug Functions
  - Core Register Access Functions
 ******************************************************************************/
/** \defgroup CMSIS_Core_FunctionInterface Functions and Instructions Reference
*/

/* ##########################   NVIC functions  #################################### */
/** \ingroup  CMSIS_Core_FunctionInterface
    \defgroup CMSIS_Core_NVICFunctions NVIC Functions
    \brief      Functions that manage interrupts and exceptions via the NVIC.
    @{
 */
// IRQn < 32
#define CLEAR_PENDING0(IRQn) MXC_INTR->irq0_clear_pending |= 1 << IRQn
// IRQ >= 32
#define CLEAR_PENDING1(IRQn) MXC_INTR->irq1_clear_pending |= 1 << (IRQn - 32)

/** \brief  Enable External Interrupt

    The function enables a device-specific interrupt in the NVIC interrupt controller.

    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_EnableIRQ(IRQn_Type IRQn)
{
    // MLB - Moved global enable into __enable_irq
    // handled event specific enable here
    if (IRQn < 32) {
        MXC_INTR->irq0_enable |= (1 << IRQn);
        MXC_EVENT->event0_enable |= (1 << IRQn);
    } else {
        MXC_INTR->irq1_enable |= (1 << (IRQn - 32));
        MXC_EVENT->event1_enable |= (1 << (IRQn - 32));
    }
}

/** \brief  Disable External Interrupt

    The function disables a device-specific interrupt in the NVIC interrupt controller.

    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_DisableIRQ(IRQn_Type IRQn)
{
    // MLB - Moved Global Disable into __disable_irq
    // handled event specific interrupt here
    if (IRQn < 32) {
        MXC_INTR->irq0_enable &= ~(1 << IRQn);
        MXC_EVENT->event0_enable &= ~(1 << IRQn);
    } else {
        MXC_INTR->irq1_enable &= ~(1 << (IRQn - 32));
        MXC_EVENT->event1_enable &= ~(1 << (IRQn - 32));
    }
}

__STATIC_INLINE void NVIC_EnableEVENT(IRQn_Type EVENT)
{
    if (EVENT < 32)
        MXC_EVENT->event0_enable |= (1 << EVENT);
    else
        MXC_EVENT->event1_enable |= (1 << (EVENT - 32));
}

__STATIC_INLINE void NVIC_DisableEVENT(IRQn_Type EVENT)
{
    if (EVENT < 32)
        MXC_EVENT->event0_enable &= ~(1 << EVENT);
    else
        MXC_EVENT->event1_enable &= ~(1 << (EVENT - 32));
}

__STATIC_INLINE void NVIC_ClearPendingEVENT(IRQn_Type EVENT)
{
    if (EVENT < 32)
        MXC_EVENT->event0_clear_pending |= (1 << EVENT);
    else
        MXC_EVENT->event1_clear_pending |= (1 << (EVENT - 32));
}

// Implemented in system_riscv_*.c
void __enable_irq(void);

__STATIC_INLINE void __disable_irq(void)
{
    // Atomic disable
    __asm volatile("csrw mstatus, 0x0");

    // Set the MPIE bit
    int mstatus = 0x80;
    __asm volatile("csrw mstatus, %0" : /* no output */ : "r"(mstatus));
}

/** \brief  Get Pending Interrupt

    The function reads the pending register in the NVIC and returns the pending bit
    for the specified interrupt.

    \param [in]      IRQn  Interrupt number.

    \return             0  Interrupt status is not pending.
    \return             1  Interrupt status is pending.
 */
__STATIC_INLINE uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
    if (IRQn < 32)
        return (MXC_INTR->irq0_pending & (1 << IRQn));
    else
        return (MXC_INTR->irq1_pending & (1 << (IRQn - 32)));
}

/** \brief  Set Pending Interrupt

    The function sets the pending bit of an external interrupt.

    \param [in]      IRQn  Interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
    //NVIC->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* set interrupt pending */
}

/** \brief  Clear Pending Interrupt

    The function clears the pending bit of an external interrupt.

    \param [in]      IRQn  External interrupt number. Value cannot be negative.
 */
__STATIC_INLINE void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
    //NVIC->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F)); /* Clear pending interrupt */
    if (IRQn < 32) {
        MXC_INTR->irq0_clear_pending |= (1 << IRQn);
        MXC_EVENT->event0_clear_pending |= (1 << IRQn);
    } else {
        MXC_INTR->irq1_clear_pending |= (1 << (IRQn - 32));
        MXC_EVENT->event1_clear_pending |= (1 << (IRQn - 32));
    }
}

/** \brief  Get Active Interrupt

    The function reads the active register in NVIC and returns the active bit.

    \param [in]      IRQn  Interrupt number.

    \return             0  Interrupt status is not active.
    \return             1  Interrupt status is active.
 */
__STATIC_INLINE uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
    return (
        0); //((uint32_t)((NVIC->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0)); /* Return 1 if active else 0 */
}

/** \brief  System Reset

    The function initiates a system reset request to reset the MCU.
 */
__STATIC_INLINE void NVIC_SystemReset(void)
{
    //__DSB();                                                     /* Ensure all outstanding memory accesses included
    //                                                                buffered write are completed before reset */
    // SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos)      |
    //               (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
    //                SCB_AIRCR_SYSRESETREQ_Msk);                   /* Keep priority group unchanged */
    //__DSB();                                                     /* Ensure completion of memory access */
    while (1) {} /* wait until reset */
}

/**
  This function returns the content of the Performance Counter Mode Register (PCMR). 
 */
__STATIC_INLINE uint32_t CSR_GetPCMR(void)
{
    uint32_t reg;
    __asm volatile("csrr %0, 0x7A1" : "=r"(reg));
    return reg;
}

/**
  This function sets the content of the Performance Counter Mode Register (PCMR). 
 */
__STATIC_INLINE void CSR_SetPCMR(uint32_t reg)
{
    __asm volatile("csrw 0x7A1, %0" : /* no output */ : "r"(reg));
}

/**
  This function returns the content of the Performance Counter Event Register (PCER). 
 */
__STATIC_INLINE uint32_t CSR_GetPCER(void)
{
    uint32_t reg;
    __asm volatile("csrr %0, 0x7A0" : "=r"(reg));
    return reg;
}

/**
  This function sets the content of the Performance Counter Event Register (PCER). 
 */
__STATIC_INLINE void CSR_SetPCER(uint32_t reg)
{
    __asm volatile("csrw 0x7A0, %0" : /* no output */ : "r"(reg));
}

/**
  This function returns the content of the Performance Counter Counter Register (PCCR). 
 */
__STATIC_INLINE uint32_t CSR_GetPCCR(void)
{
    uint32_t reg;
    __asm volatile("csrr %0, 0x780" : "=r"(reg));
    return reg;
}

/**
  This function sets the content of the Performance Counter Counter Register (PCCR). 
 */
__STATIC_INLINE void CSR_SetPCCR(uint32_t reg)
{
    __asm volatile("csrw 0x780, %0" : /* no output */ : "r"(reg));
}

/*@} end of CMSIS_Core_NVICFunctions */

/*@} end of CMSIS_core_DebugFunctions */

#endif /* __CORE_CM4_H_DEPENDENT */

#endif /* __CMSIS_GENERIC */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_CMSIS_INCLUDE_CORE_RV32_H_
