/**
 * @file    max32657.h
 * @brief   Device-specific perhiperal header file
 */

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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MAX32657_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MAX32657_H_

#ifndef TARGET_NUM
#define TARGET_NUM 32657
#endif

#define MXC_NUMCORES 1

#include <stdint.h>
#include "system_max32657.h"

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (1)
#endif

/* COMPILER SPECIFIC DEFINES (IAR, ARMCC and GNUC) */
#if defined(__GNUC__)
#ifndef __weak
#define __weak __attribute__((weak))
#endif

#elif defined(__CC_ARM)

#define inline __inline
#pragma anon_unions

#endif

/* ================================================================================ */
/* ==================       Interrupt Number Table Section       ================== */
/* ================================================================================ */

// clang-format off
typedef enum {
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SecureFault_IRQn = -9,
    SVCall_IRQn = -5,
    DebugMonitor_IRQn = -4,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,

    /* Device-specific interrupt sources (external to ARM core)                    */
    /*                         table entry number                                  */
    /*                         ||||                                                */
    /*                         ||||  table offset address                          */
    /*                         vvvv  vvvvvv                                        */

    ICE_IRQn = 0,           /* 0x10  0x0040  16: ICE Unlock */
    WDT_IRQn,               /* 0x11  0x0044  17: Watchdog Timer */
    RTC_IRQn,               /* 0x12  0x0048  18: RTC */
    TRNG_IRQn,              /* 0x13  0x004C  19: True Random Number Generator */
    TMR0_IRQn,              /* 0x14  0x0050  20: Timer 0 */
    TMR1_IRQn,              /* 0x15  0x0054  21: Timer 1 */
    TMR2_IRQn,              /* 0x16  0x0058  22: Timer 2 */
    TMR3_IRQn,              /* 0x17  0x005C  23: Timer 3 */
    TMR4_IRQn,              /* 0x18  0x0060  24: Timer 4 */
    TMR5_IRQn,              /* 0x19  0x0064  25: Timer 5 */
    I3C_IRQn,               /* 0x1A  0x0068  26: I3C */
    UART_IRQn,              /* 0x1B  0x006C  27: UART */
    SPI_IRQn,               /* 0x1C  0x0070  28: SPI */
    FLC_IRQn,               /* 0x1D  0x0074  29: FLC */
    GPIO0_IRQn,             /* 0x1E  0x0078  30: GPIO0 */
    RSV15_IRQn,             /* 0x1F  0x007C  31: Reserved */
    DMA0_CH0_IRQn,          /* 0x20  0x0080  32: DMA0 Channel 0 */
    DMA0_CH1_IRQn,          /* 0x21  0x0084  33: DMA0 Channel 1 */
    DMA0_CH2_IRQn,          /* 0x22  0x0088  34: DMA0 Channel 2 */
    DMA0_CH3_IRQn,          /* 0x23  0x008C  35: DMA0 Channel 3 */
    DMA1_CH0_IRQn,          /* 0x24  0x0090  36: DMA1 Channel 0 (Secure) */
    DMA1_CH1_IRQn,          /* 0x25  0x0094  37: DMA1 Channel 1 (Secure) */
    DMA1_CH2_IRQn,          /* 0x26  0x0098  38: DMA1 CHannel 2 (Secure) */
    DMA1_CH3_IRQn,          /* 0x27  0x009C  39: DMA1 Channel 3 (Secure) */
    WUT0_IRQn,              /* 0x28  0x00A0  40: Wakeup Timer 0 */
    WUT1_IRQn,              /* 0x29  0x00A4  41: Wakeup TImer 1 */
    GPIOWAKE_IRQn,          /* 0x2A  0x00A8  42: GPIO Wakeup */
    CRC_IRQn,               /* 0x2B  0x00AC  43: CRC */
    AES_IRQn,               /* 0x2C  0x00B0  44: AES */
    ERFO_IRQn,              /* 0x2D  0x00B4  45: ERFO Ready */
    BOOST_IRQn,             /* 0x2E  0x00B8  46: Boost Controller */
    ECC_IRQn,               /* 0x2F  0x00BC  47: ECC */
    BTLE_TX_DONE_IRQn,      /* 0x30  0x00C0  48: BTLE TX Done */
    BTLE_RX_RCVD_IRQn,      /* 0x31  0x00C4  49: BTLE RX Received */
    BTLE_RX_ENG_DET_IRQn,   /* 0x32  0x00C8  50: BTLE RX Energy Detected */
    BTLE_SFD_DET_IRQn,      /* 0x33  0x00CC  51: BTLE SFD Detected */
    BTLE_SFD_TO_IRQn,       /* 0x34  0x00D0  52: BTLE SFD Timeout */
    BTLE_GP_EVENT_IRQn,     /* 0x35  0x00D4  53: BTLE BTLE Timestamp */
    BTLE_CFO_IRQn,          /* 0x36  0x00D8  54: BTLE CFO Done */
    BTLE_SIG_DET_IRQn,      /* 0x37  0x00DC  55: BTLE Signal Detected */
    BTLE_AGC_EVENT_IRQn,    /* 0x38  0x00E0  56: BTLE AGC Event */
    BTLE_RFFE_SPIM_IRQn,    /* 0x39  0x00E4  57: BTLE RFFE SPIM Done */
    BTLE_TX_AES_IRQn,       /* 0x3A  0x00E8  58: BTLE TX AES Done */
    BTLE_RX_AES_IRQn,       /* 0x3B  0x00EC  59: BTLE RX AES Done */
    BTLE_INV_APB_ADDR_IRQn, /* 0x3C  0x00F0  60: BTLE Invalid APB Address */
    BTLE_IQ_DATA_VALID_IRQn, /* 0x3D  0x00F4  61:BTLE IQ Data Valid */
    BTLE_RX_CRC_IRQn,       /* 0x3E  0x00F8  62: BTLE RX CRC */
    RSV47_IRQn,             /* 0x3F  0x00FC  63: Reserved */
    MPC_IRQn,               /* 0x40  0x0100  64: MPC Combined (Secure) */
    PPC_IRQn,               /* 0x41  0x0104  65: PPC Combined (Secure) */
    FRQCNT_IRQn,            /* 0x42  0x0108  66: Frequency Counter */
    RSV51_IRQn,             /* 0x43  0x010C  67: Reserved */
    RSV52_IRQn,             /* 0x44  0x0110  68: Reserved */
    RSV53_IRQn,             /* 0x45  0x0114  69: Reserved */
    MXC_IRQ_EXT_COUNT,
} IRQn_Type;
// clang-format on

#define MXC_IRQ_COUNT (MXC_IRQ_EXT_COUNT + 16)

/* ================================================================================ */
/* ==================             Processor Section              ================== */
/* ================================================================================ */

#define __CM33_REV 0x0000U /**< Cortex-M33 Core revision */
#define __DSP_PRESENT 1U /**< Presence of DSP  */
#define __FPU_PRESENT 1U /**< Presence of FPU  */
#define __MPU_PRESENT 1U /**< Presence of MPU  */
#define __SAUREGION_PRESENT 1U /**< Presence of FPU  */
#define __TZ_PRESENT 1U /**< Presence of TrustZone */
#define __VTOR_PRESENT 1U /**< Presence of VTOR register in SCB  */
#define __NVIC_PRIO_BITS 3U /**< NVIC interrupt priority bits */
#define __Vendor_SysTickConfig 0U /**< Is 1 if different SysTick counter is used */

#include <core_cm33.h>
#if (__CM_CMSIS_VERSION == 0x60000)
/* If CMSIS version 6.0 use cmsis_gcc_m.h */
#include <cmsis_gcc_m.h>
#else
#include <cmsis_gcc.h>
#endif
#include <arm_cmse.h>

#if defined(__GNUC__)
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
// Type used for secure code to call non-secure code.
#define __ns_call __attribute((cmse_nonsecure_call))
typedef void __ns_call (*mxc_ns_call_t)(void);
// Type used for non-secure code to call secure code.
#define __ns_entry __attribute((cmse_nonsecure_entry))
#endif
#endif

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

/* Physical Memory Definitions */
/* Bit 28 (security alias bit) of address is cleared. */
#define MXC_PHY_FLASH_MEM_BASE 0x01000000UL
#define MXC_PHY_FLASH_MEM_SIZE 0x00100000UL
#define MXC_PHY_FLASH_PAGE_SIZE 0x00002000UL

#define MXC_PHY_SRAM_MEM_BASE 0x20000000UL
#define MXC_PHY_SRAM_MEM_SIZE 0x00040000UL

#define MXC_PHY_SRAM0_MEM_BASE 0x20000000UL
#define MXC_PHY_SRAM0_MEM_SIZE 0x00008000UL // 32KB
#define MXC_PHY_SRAM1_MEM_BASE 0x20008000UL
#define MXC_PHY_SRAM1_MEM_SIZE 0x00008000UL // 32KB
#define MXC_PHY_SRAM2_MEM_BASE 0x20010000UL
#define MXC_PHY_SRAM2_MEM_SIZE 0x00010000UL // 64KB
#define MXC_PHY_SRAM3_MEM_BASE 0x20020000UL
#define MXC_PHY_SRAM3_MEM_SIZE 0x00010000UL // 64KB
#define MXC_PHY_SRAM4_MEM_BASE 0x20030000UL
#define MXC_PHY_SRAM4_MEM_SIZE 0x00010000UL // 64KB

/**
 * Memory settings are defined accordingly by build system: max32657_memory.mk
 * 
 * Definitions that start with the '__' are defined by the build system.
 *  For example, '__MXC_FLASH_MEM_BASE'
 */

#if CONFIG_TRUSTED_EXECUTION_SECURE
/* Non-secure Regions that secure code knows about. */
#define MXC_FLASH_NS_MEM_BASE __MXC_FLASH_NS_MEM_BASE
#define MXC_FLASH_NS_PAGE_SIZE MXC_PHY_FLASH_PAGE_SIZE
#define MXC_FLASH_NS_MEM_SIZE __MXC_FLASH_NS_MEM_SIZE
#define MXC_SRAM_NS_MEM_BASE __MXC_SRAM_NS_MEM_BASE
#define MXC_SRAM_NS_MEM_SIZE __MXC_SRAM_NS_MEM_SIZE

/* Secure Regions */
/*  ROM is always in secure region. */
#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00010000UL
/* Flash info is always in secure region */
#define MXC_INFO_S_MEM_BASE 0x12000000UL
#define MXC_INFO_S_MEM_SIZE 0x00004000UL

#define MXC_INFO_MEM_BASE MXC_INFO_S_MEM_BASE
#define MXC_INFO_MEM_SIZE MXC_INFO_S_MEM_SIZE
#endif

#define MXC_FLASH_MEM_BASE __MXC_FLASH_MEM_BASE
#define MXC_FLASH_PAGE_SIZE MXC_PHY_FLASH_PAGE_SIZE
#define MXC_FLASH_MEM_SIZE __MXC_FLASH_MEM_SIZE
#define MXC_SRAM_MEM_BASE __MXC_SRAM_MEM_BASE
#define MXC_SRAM_MEM_SIZE __MXC_SRAM_MEM_SIZE

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX78000 peripheral modules.
*/

/******************************************************************************/
/*                                                             Global control */

/* Non-secure Mapping */
#define MXC_BASE_GCR_NS ((uint32_t)0x40000000UL)
#define MXC_GCR_NS ((mxc_gcr_regs_t *)MXC_BASE_GCR_NS)

/* Secure Mapping */
#define MXC_BASE_GCR_S ((uint32_t)0x50000000UL)
#define MXC_GCR_S ((mxc_gcr_regs_t *)MXC_BASE_GCR_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_GCR MXC_GCR_S
#else
#define MXC_GCR MXC_GCR_NS
#endif

/******************************************************************************/
/*                                            Non-battery backed SI Registers */

/* Non-secure Mapping */
#define MXC_BASE_SIR_NS ((uint32_t)0x40000400UL)
#define MXC_SIR_NS ((mxc_sir_regs_t *)MXC_BASE_SIR_NS)

/* Secure Mapping */
#define MXC_BASE_SIR_S ((uint32_t)0x50000400UL)
#define MXC_SIR_S ((mxc_sir_regs_t *)MXC_BASE_SIR_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_SIR MXC_BASE_SIR_S
#define MXC_SIR MXC_SIR_S
#else
#define MXC_BASE_SIR MXC_BASE_SIR_NS
#define MXC_SIR MXC_SIR_NS
#endif

/******************************************************************************/
/*                                        Non-Battery Backed Function Control */

/* Non-secure Mapping */
#define MXC_BASE_FCR_NS ((uint32_t)0x40000800UL)
#define MXC_FCR_NS ((mxc_fcr_regs_t *)MXC_BASE_FCR_NS)

/* Secure Mapping */
#define MXC_BASE_FCR_S ((uint32_t)0x50000800UL)
#define MXC_FCR_S ((mxc_fcr_regs_t *)MXC_BASE_FCR_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_FCR MXC_BASE_FCR_S
#define MXC_FCR MXC_FCR_S
#else
#define MXC_BASE_FCR MXC_BASE_FCR_NS
#define MXC_FCR MXC_FCR_NS
#endif

/******************************************************************************/
/*                                                    Windowed Watchdog Timer */
#define MXC_CFG_WDT_INSTANCES (1)

/* Non-secure Mapping */
#define MXC_BASE_WDT_NS ((uint32_t)0x40003000UL)
#define MXC_WDT_NS ((mxc_wdt_regs_t *)MXC_BASE_WDT_NS)

/* Secure Mapping */
#define MXC_BASE_WDT_S ((uint32_t)0x50003000UL)
#define MXC_WDT_S ((mxc_wdt_regs_t *)MXC_BASE_WDT_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_WDT MXC_BASE_WDT_S
#define MXC_WDT MXC_WDT_S
#else
#define MXC_BASE_WDT MXC_BASE_WDT_NS
#define MXC_WDT MXC_WDT_NS
#endif

/******************************************************************************/
/*                                                            RSTZ Controller */

/* Non-secure Mapping */
#define MXC_BASE_RSTZ_NS ((uint32_t)0x40004800UL)
#define MXC_RSTZ_NS ((mxc_rstz_regs_t *)MXC_BASE_RSTZ_NS)

/* Secure Mapping */
#define MXC_BASE_RSTZ_S ((uint32_t)0x50004800UL)
#define MXC_RSTZ_S ((mxc_rstz_regs_t *)MXC_BASE_RSTZ_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_RSTZ MXC_BASE_RSTZ_S
#define MXC_RSTZ MXC_RSTZ_S //TODO(ME30): Add SVM controller registers
#else
#define MXC_BASE_RSTZ MXC_BASE_RSTZ_NS
#define MXC_RSTZ MXC_RSTZ_NS
#endif

/******************************************************************************/
/*                                                           Boost Controller */

/* Non-secure Mapping */
#define MXC_BASE_BOOST_NS ((uint32_t)0x40004C00UL)
#define MXC_BOOST_NS ((mxc_boost_regs_t *)MXC_BASE_BOOST_NS)

/* Secure Mapping */
#define MXC_BASE_BOOST_S ((uint32_t)0x50004C00UL)
#define MXC_BOOST_S ((mxc_boost_regs_t *)MXC_BASE_BOOST_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_BOOST MXC_BASE_BOOST_S
#define MXC_BOOST MXC_BOOST_S
#else
#define MXC_BASE_BOOST MXC_BASE_BOOST_NS
#define MXC_BOOST MXC_BOOST_NS
#endif

/******************************************************************************/
/*                                         Trim System Initalization Register */

/* Non-secure Mapping */
#define MXC_BASE_TRIMSIR_NS ((uint32_t)0x40005400UL)
#define MXC_TRIMSIR_NS ((mxc_trimsir_regs_t *)MXC_BASE_TRIMSIR_NS)

/* Secure Mapping */
#define MXC_BASE_TRIMSIR_S ((uint32_t)0x50005400UL)
#define MXC_TRIMSIR_S ((mxc_trimsir_regs_t *)MXC_BASE_TRIMSIR_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_TRIMSIR MXC_BASE_TRIMSIR_S
#define MXC_TRIMSIR MXC_TRIMSIR_S
#else
#define MXC_BASE_TRIMSIR MXC_BASE_TRIMSIR_NS
#define MXC_TRIMSIR MXC_TRIMSIR_NS
#endif

/******************************************************************************/
/*                                                            Real Time Clock */

/* Non-secure Mapping */
#define MXC_BASE_RTC_NS ((uint32_t)0x40006000UL)
#define MXC_RTC_NS ((mxc_rtc_regs_t *)MXC_BASE_RTC_NS)

/* Secure Mapping */
#define MXC_BASE_RTC_S ((uint32_t)0x50006000UL)
#define MXC_RTC_S ((mxc_rtc_regs_t *)MXC_BASE_RTC_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_RTC MXC_BASE_RTC_S
#define MXC_RTC MXC_RTC_S
#else
#define MXC_BASE_RTC MXC_BASE_RTC_NS
#define MXC_RTC MXC_RTC_NS
#endif

/******************************************************************************/
/*                                                        Wake-Up Timer (WUT) */
#define MXC_CFG_WUT_INSTANCES (2)

/* Non-secure Mapping */
#define MXC_BASE_WUT0_NS ((uint32_t)0x40006400UL)
#define MXC_WUT0_NS ((mxc_wut_regs_t *)MXC_BASE_WUT0_NS)
#define MXC_BASE_WUT1_NS ((uint32_t)0x40006600UL)
#define MXC_WUT1_NS ((mxc_wut_regs_t *)MXC_BASE_WUT1_NS)

/* Secure Mapping */
#define MXC_BASE_WUT0_S ((uint32_t)0x50006400UL)
#define MXC_WUT0_S ((mxc_wut_regs_t *)MXC_BASE_WUT0_S)
#define MXC_BASE_WUT1_S ((uint32_t)0x50006600UL)
#define MXC_WUT1_S ((mxc_wut_regs_t *)MXC_BASE_WUT1_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_WUT0 MXC_BASE_WUT0_S
#define MXC_WUT0 MXC_WUT0_S
#define MXC_BASE_WUT1 MXC_BASE_WUT1_S
#define MXC_WUT1 MXC_WUT1_S
#else
#define MXC_BASE_WUT0 MXC_BASE_WUT0_NS
#define MXC_WUT0 MXC_WUT0_NS
#define MXC_BASE_WUT1 MXC_BASE_WUT1_NS
#define MXC_WUT1 MXC_WUT1_NS
#endif

/******************************************************************************/
/*                                                            Power Sequencer */

/* Non-secure Mapping */
#define MXC_BASE_PWRSEQ_NS ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ_NS ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ_NS)

/* Secure Mapping */
#define MXC_BASE_PWRSEQ_S ((uint32_t)0x50006800UL)
#define MXC_PWRSEQ_S ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_PWRSEQ MXC_BASE_PWRSEQ_S
#define MXC_PWRSEQ MXC_PWRSEQ_S
#else
#define MXC_BASE_PWRSEQ MXC_BASE_PWRSEQ_NS
#define MXC_PWRSEQ MXC_PWRSEQ_NS
#endif

/******************************************************************************/
/*                                                              Misc Control  */

/* Non-secure Mapping */
#define MXC_BASE_MCR_NS ((uint32_t)0x40006C00UL)
#define MXC_MCR_NS ((mxc_mcr_regs_t *)MXC_BASE_MCR_NS)

/* Secure Mapping */
#define MXC_BASE_MCR_S ((uint32_t)0x50006C00UL)
#define MXC_MCR_S ((mxc_mcr_regs_t *)MXC_BASE_MCR_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_MCR MXC_BASE_MCR_S
#define MXC_MCR MXC_MCR_S
#else
#define MXC_BASE_MCR MXC_BASE_MCR_NS
#define MXC_MCR MXC_MCR_NS
#endif

/******************************************************************************/
/*                                                                        AES */

/* Non-secure Mapping */
#define MXC_BASE_AES_NS ((uint32_t)0x40007400UL)
#define MXC_AES_NS ((mxc_aes_regs_t *)MXC_BASE_AES_NS)

/* Secure Mapping */
#define MXC_BASE_AES_S ((uint32_t)0x50007400UL)
#define MXC_AES_S ((mxc_aes_regs_t *)MXC_BASE_AES_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_AES MXC_BASE_AES_S
#define MXC_AES MXC_AES_S
#else
#define MXC_BASE_AES MXC_BASE_AES_NS
#define MXC_AES MXC_AES_NS
#endif

/******************************************************************************/
/*                                                                   AES Keys */

/* Non-secure Mapping */
#define MXC_BASE_AESKEYS_NS ((uint32_t)0x40007800UL)
#define MXC_AESKEYS_NS ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS_NS)

/* Secure Mapping */
#define MXC_BASE_AESKEYS_S ((uint32_t)0x50007800UL)
#define MXC_AESKEYS_S ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_AESKEYS MXC_BASE_AESKEYS_S
#define MXC_AESKEYS MXC_AESKEYS_S
#else
#define MXC_BASE_AESKEYS MXC_BASE_AESKEYS_NS
#define MXC_AESKEYS MXC_AESKEYS_NS
#endif

/******************************************************************************/
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (1)
#define MXC_CFG_GPIO_PINS_PORT (32)

/* Non-secure Mapping */
#define MXC_BASE_GPIO0_NS ((uint32_t)0x40008000UL)
#define MXC_GPIO0_NS ((mxc_gpio_regs_t *)MXC_BASE_GPIO0_NS)

#define MXC_GPIO_NS_GET_IDX(p) ((p) == MXC_GPIO0_NS ? 0 : -1)
#define MXC_GPIO_NS_GET_GPIO(i) ((i) == 0 ? MXC_GPIO0_NS : 0)

/* Secure Mapping */
#define MXC_BASE_GPIO0_S ((uint32_t)0x50008000UL)
#define MXC_GPIO0_S ((mxc_gpio_regs_t *)MXC_BASE_GPIO0_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_GPIO0 MXC_BASE_GPIO0_S
#define MXC_GPIO0 MXC_GPIO0_S
#else
#define MXC_BASE_GPIO0 MXC_BASE_GPIO0_NS
#define MXC_GPIO0 MXC_GPIO0_NS
#endif

/*
Note(JC): There is only 1 GPIO instance, but for driver compatibility these must be
implemented.

For GET_IRQ we follow precedent and return the base 0 IRQn, which is the ICE unlock.
We may want to handle GET_IRQ better...
*/
#define MXC_GPIO_GET_IDX(p) ((p) == MXC_GPIO0 ? 0 : 0)
#define MXC_GPIO_GET_GPIO(i) ((i) == 0 ? MXC_GPIO0 : 0)
#define MXC_GPIO_GET_IRQ(i) ((i) == 0 ? GPIO0_IRQn : (IRQn_Type)0)

/******************************************************************************/
/*                                                                        CRC */

/* Non-secure Mapping */
#define MXC_BASE_CRC_NS ((uint32_t)0x4000F000UL)
#define MXC_CRC_NS ((mxc_crc_regs_t *)MXC_BASE_CRC_NS)

/* Secure Mapping */
#define MXC_BASE_CRC_S ((uint32_t)0x5000F000UL)
#define MXC_CRC_S ((mxc_crc_regs_t *)MXC_BASE_CRC_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_CRC MXC_BASE_CRC_S
#define MXC_CRC MXC_CRC_S
#else
#define MXC_BASE_CRC MXC_BASE_CRC_NS
#define MXC_CRC MXC_CRC_NS
#endif

/******************************************************************************/
/*                                                                      Timer */
#define SEC(s) (((uint32_t)s) * 1000000UL)
#define MSEC(ms) (ms * 1000UL)
#define USEC(us) (us)

#define MXC_CFG_TMR_INSTANCES (6)

/* Non-secure Mapping */
#define MXC_BASE_TMR0_NS ((uint32_t)0x40010000UL)
#define MXC_TMR0_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR0_NS)
#define MXC_BASE_TMR1_NS ((uint32_t)0x40011000UL)
#define MXC_TMR1_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR1_NS)
#define MXC_BASE_TMR2_NS ((uint32_t)0x40012000UL)
#define MXC_TMR2_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR2_NS)
#define MXC_BASE_TMR3_NS ((uint32_t)0x40013000UL)
#define MXC_TMR3_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR3_NS)
#define MXC_BASE_TMR4_NS ((uint32_t)0x40014000UL)
#define MXC_TMR4_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR4_NS)
#define MXC_BASE_TMR5_NS ((uint32_t)0x40015000UL)
#define MXC_TMR5_NS ((mxc_tmr_regs_t *)MXC_BASE_TMR5_NS)

/* Secure Mapping */
#define MXC_BASE_TMR0_S ((uint32_t)0x50010000UL)
#define MXC_TMR0_S ((mxc_tmr_regs_t *)MXC_BASE_TMR0_S)
#define MXC_BASE_TMR1_S ((uint32_t)0x50011000UL)
#define MXC_TMR1_S ((mxc_tmr_regs_t *)MXC_BASE_TMR1_S)
#define MXC_BASE_TMR2_S ((uint32_t)0x50012000UL)
#define MXC_TMR2_S ((mxc_tmr_regs_t *)MXC_BASE_TMR2_S)
#define MXC_BASE_TMR3_S ((uint32_t)0x50013000UL)
#define MXC_TMR3_S ((mxc_tmr_regs_t *)MXC_BASE_TMR3_S)
#define MXC_BASE_TMR4_S ((uint32_t)0x50014000UL)
#define MXC_TMR4_S ((mxc_tmr_regs_t *)MXC_BASE_TMR4_S)
#define MXC_BASE_TMR5_S ((uint32_t)0x50015000UL)
#define MXC_TMR5_S ((mxc_tmr_regs_t *)MXC_BASE_TMR5_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_TMR0 MXC_TMR0_S
#define MXC_TMR1 MXC_TMR1_S
#define MXC_TMR2 MXC_TMR2_S
#define MXC_TMR3 MXC_TMR3_S
#define MXC_TMR4 MXC_TMR4_S
#define MXC_TMR5 MXC_TMR5_S
#else
#define MXC_TMR0 MXC_TMR0_NS
#define MXC_TMR1 MXC_TMR1_NS
#define MXC_TMR2 MXC_TMR2_NS
#define MXC_TMR3 MXC_TMR3_NS
#define MXC_TMR4 MXC_TMR4_NS
#define MXC_TMR5 MXC_TMR5_NS
#endif

#define MXC_TMR_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_TMR0 : \
     (i) == 1 ? MXC_BASE_TMR1 : \
     (i) == 2 ? MXC_BASE_TMR2 : \
     (i) == 3 ? MXC_BASE_TMR3 : \
     (i) == 4 ? MXC_BASE_TMR4 : \
     (i) == 5 ? MXC_BASE_TMR5 : \
                0)

#define MXC_TMR_GET_TMR(i) \
    ((i) == 0 ? MXC_TMR0 : \
     (i) == 1 ? MXC_TMR1 : \
     (i) == 2 ? MXC_TMR2 : \
     (i) == 3 ? MXC_TMR3 : \
     (i) == 4 ? MXC_TMR4 : \
     (i) == 5 ? MXC_TMR5 : \
                0)

#define MXC_TMR_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? TMR0_IRQn : \
                (i) == 1 ? TMR1_IRQn : \
                (i) == 2 ? TMR2_IRQn : \
                (i) == 3 ? TMR3_IRQn : \
                (i) == 4 ? TMR4_IRQn : \
                (i) == 5 ? TMR5_IRQn : \
                           0)

#define MXC_TMR_GET_IDX(p) \
    ((p) == MXC_TMR0 ? 0 : \
     (p) == MXC_TMR1 ? 1 : \
     (p) == MXC_TMR2 ? 2 : \
     (p) == MXC_TMR3 ? 3 : \
     (p) == MXC_TMR4 ? 4 : \
     (p) == MXC_TMR5 ? 5 : \
                       -1)

/******************************************************************************/
/*                                                                        I3C */
#define MXC_CFG_I3C_INSTANCES (1)
#define MXC_I3C_FIFO_DEPTH (8)

/* Non-secure Mapping */
#define MXC_BASE_I3C_NS ((uint32_t)0x40018000UL)
#define MXC_I3C_NS ((mxc_i3c_regs_t *)MXC_BASE_I3C_NS)

/* Secure Mapping */
#define MXC_BASE_I3C_S ((uint32_t)0x50018000UL)
#define MXC_I3C_S ((mxc_i3c_regs_t *)MXC_BASE_I3C_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_I3C MXC_BASE_I3C_S
#define MXC_I3C MXC_I3C_S
#else
#define MXC_BASE_I3C MXC_BASE_I3C_NS
#define MXC_I3C MXC_I3C_NS
#endif

#define MXC_I3C_GET_BASE(i) ((i) == 0 ? MXC_BASE_I3C : 0)
#define MXC_I3C_GET_I3C(i) ((i) == 0 ? MXC_I3C : 0)
#define MXC_I3C_GET_IRQ(i) (IRQn_Type)((i) == 0 ? I3C_IRQn : 0)
#define MXC_I3C_GET_IDX(p) ((p) == MXC_I3C_NS ? 0 : (p) == MXC_I3C_S ? 0 : -1)

/******************************************************************************/
/*                                                                        DMA */
#define MXC_DMA_CHANNELS (4)
#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_DMA_INSTANCES (2)
#else
#define MXC_DMA_INSTANCES (1)
#endif

/* Non-secure Mapping */
/* DMA0 Security Attribution hardwired to Non-Secure and not configurable via SPC. */
#define MXC_BASE_DMA0_NS ((uint32_t)0x40028000UL)
#define MXC_DMA0_NS ((mxc_dma_regs_t *)MXC_BASE_DMA0_NS)

/* Secure Mapping */
/* DMA1 Security Attribution hardwired to Secure and not configurable via SPC. */
#define MXC_BASE_DMA1_S ((uint32_t)0x50035000UL)
#define MXC_DMA1_S ((mxc_dma_regs_t *)MXC_BASE_DMA1_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_DMA1 MXC_BASE_DMA1_S
#define MXC_DMA1 MXC_DMA1_S
/**
 * MXC_DMA0 is not defined because DMA0 has no Secure mapping.
 * Following ARM naming convention: if Secure world wants to access Non-Secure DMA (DMAO),
 *  then use MXC_DMA0_NS. Similar to how the Secure world accesses the Non-Secure MSP
 *  and VTOR registers using 'MSP_NS' and 'VTOR_NS', respectively.
 */
#ifdef MXC_DMA0
#warning "Non-Secure DMA (DMA0) has no secure mapping. Please use MXC_DMA0_NS from Secure world."
#endif

#define MXC_DMA_CH_GET_IRQ(p, i)                                    \
    ((IRQn_Type)(((p) == MXC_DMA0_NS && (i) == 0) ? DMA0_CH0_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 1) ? DMA0_CH1_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 2) ? DMA0_CH2_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 3) ? DMA0_CH3_IRQn : \
                 ((p) == MXC_DMA1_S && (i) == 0)  ? DMA1_CH0_IRQn : \
                 ((p) == MXC_DMA1_S && (i) == 1)  ? DMA1_CH1_IRQn : \
                 ((p) == MXC_DMA1_S && (i) == 2)  ? DMA1_CH2_IRQn : \
                 ((p) == MXC_DMA1_S && (i) == 3)  ? DMA1_CH3_IRQn : \
                                                    0))

#else
#define MXC_BASE_DMA0 MXC_BASE_DMA0_NS
#define MXC_DMA0 MXC_DMA0_NS
/* MXC_DMA1 is not defined because Non-Secure Code can only access DMA0. */
#ifdef MXC_DMA1
#warning "Secure DMA (DMA1) is not accessible from Non-Secure world."
#endif

/* DMA1 IRQs not usable in Non-Secure state. */
#define MXC_DMA_CH_GET_IRQ(p, i)                                    \
    ((IRQn_Type)(((p) == MXC_DMA0_NS && (i) == 0) ? DMA0_CH0_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 1) ? DMA0_CH1_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 2) ? DMA0_CH2_IRQn : \
                 ((p) == MXC_DMA0_NS && (i) == 3) ? DMA0_CH3_IRQn : \
                                                    0))
#endif // CONFIG_TRUSTED_EXECUTION_SECURE

#define MXC_DMA_GET_BASE(i) ((i) == MXC_BASE_DMA0_NS ? 0 : (p) == MXC_BASE_DMA1_S ? 1 : -1)

#define MXC_DMA_GET_IDX(p) ((p) == MXC_DMA0_NS ? 0 : (p) == MXC_DMA1_S ? 1 : -1)

/******************************************************************************/
/*                                                           Flash Controller */
#define MXC_FLC_INSTANCES (1)

/* Secure Mapping Only */
#define MXC_BASE_FLC ((uint32_t)0x50029000UL)
#define MXC_FLC ((mxc_flc_regs_t *)MXC_BASE_FLC)

/* Added for consistency and explicitness */
#define MXC_BASE_FLC_S MXC_BASE_FLC
#define MXC_FLC_S MXC_FLC

/**
 *  There is only one flash instance, but some bottom-level RevX implementations
 *  depend on MXC_FLC_GET_FLC
 */
#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC : 0)

/******************************************************************************/
/*                                                  Internal Cache Controller */
#define MXC_ICC_INSTANCES (1)

/* Secure Mapping Only */
#define MXC_BASE_ICC ((uint32_t)0x5002A000UL)
#define MXC_ICC ((mxc_icc_regs_t *)MXC_BASE_ICC_S)

/* Added for consistency and explicitness */
#define MXC_BASE_ICC_S MXC_BASE_ICC
#define MXC_ICC_S MXC_ICC

/******************************************************************************/
/*                                               UART / Serial Port Interface */
#define MXC_UART_INSTANCES (1)
#define MXC_UART_FIFO_DEPTH (8) // TODO(ME30): Check this is correct.

/* Non-secure Mapping */
#define MXC_BASE_UART_NS ((uint32_t)0x40042000UL)
#define MXC_UART_NS ((mxc_uart_regs_t *)MXC_BASE_UART_NS)

/* Secure Mapping */
#define MXC_BASE_UART_S ((uint32_t)0x50042000UL)
#define MXC_UART_S ((mxc_uart_regs_t *)MXC_BASE_UART_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_UART MXC_BASE_UART_S
#define MXC_UART MXC_UART_S
#else
#define MXC_BASE_UART MXC_BASE_UART_NS
#define MXC_UART MXC_UART_NS
#endif

#define MXC_UART_GET_BASE(i) ((i) == 0 ? MXC_BASE_UART : 0)
#define MXC_UART_GET_UART(i) ((i) == 0 ? MXC_UART : 0)
#define MXC_UART_GET_IRQ(i) (IRQn_Type)((i) == 0 ? UART_IRQn : 0)
#define MXC_UART_GET_IDX(p) ((p) == MXC_UART_NS ? 0 : (p) == MXC_UART_S ? 0 : -1)

/******************************************************************************/
/*                                                                        SPI */
#define MXC_SPI_INSTANCES (1)
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

/* Non-secure Mapping */
#define MXC_BASE_SPI_NS ((uint32_t)0x40046000UL)
#define MXC_SPI_NS ((mxc_spi_regs_t *)MXC_BASE_SPI_NS)

/* Secure Mapping */
#define MXC_BASE_SPI_S ((uint32_t)0x50046000UL)
#define MXC_SPI_S ((mxc_spi_regs_t *)MXC_BASE_SPI_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_SPI MXC_BASE_SPI_S
#define MXC_SPI MXC_SPI_S
#else
#define MXC_BASE_SPI MXC_BASE_SPI_S
#define MXC_SPI MXC_SPI_NS
#endif

#define MXC_SPI_GET_BASE(i) ((i) == 0 ? MXC_BASE_SPI : 0)
#define MXC_SPI_GET_SPI(i) ((i) == 0 ? MXC_SPI : 0)
#define MXC_SPI_GET_IRQ(i) (IRQn_Type)((i) == 0 ? SPI_IRQn : 0)
#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI_NS ? 0 : (p) == MXC_SPI_S ? 0 : -1)

/******************************************************************************/
/*                                                                       TRNG */

/* Non-secure Mapping */
#define MXC_BASE_TRNG_NS ((uint32_t)0x4004D000UL)
#define MXC_TRNG_NS ((mxc_trng_regs_t *)MXC_BASE_TRNG_NS)

/* Secure Mapping */
#define MXC_BASE_TRNG_S ((uint32_t)0x5004D000UL)
#define MXC_TRNG_S ((mxc_trng_regs_t *)MXC_BASE_TRNG_S)

#if CONFIG_TRUSTED_EXECUTION_SECURE
#define MXC_BASE_TRNG MXC_BASE_TRNG_S
#define MXC_TRNG MXC_TRNG_S
#else
#define MXC_BASE_TRNG MXC_BASE_TRNG_NS
#define MXC_TRNG MXC_TRNG_NS
#endif

/******************************************************************************/
/*                   Non-Secure and Secure Privilege Controller (NSPC/SPC TZ) */

#if CONFIG_TRUSTED_EXECUTION_SECURE

/* Secure Mapping Only */
#define MXC_BASE_SPC ((uint32_t)0x50090000UL)
#define MXC_SPC ((mxc_spc_regs_t *)MXC_BASE_SPC)
#define MXC_SPC_S MXC_SPC

#endif

/* Non-Secure Mapping Only */
#define MXC_BASE_NSPC ((uint32_t)0x40090000UL)
#define MXC_NSPC ((mxc_nspc_regs_t *)MXC_BASE_NSPC)
#define MXC_NSPC_NS MXC_NSPC

/******************************************************************************/
/*                                                                        MPC */

/* Secure Mapping Only */
#define MXC_BASE_MPC_SRAM0 ((uint32_t)0x50091000UL)
#define MXC_MPC_SRAM0 ((mxc_mpc_regs_t *)MXC_BASE_MPC_SRAM0)
#define MXC_BASE_MPC_SRAM1 ((uint32_t)0x50092000UL)
#define MXC_MPC_SRAM1 ((mxc_mpc_regs_t *)MXC_BASE_MPC_SRAM1)
#define MXC_BASE_MPC_SRAM2 ((uint32_t)0x50093000UL)
#define MXC_MPC_SRAM2 ((mxc_mpc_regs_t *)MXC_BASE_MPC_SRAM2)
#define MXC_BASE_MPC_SRAM3 ((uint32_t)0x50094000UL)
#define MXC_MPC_SRAM3 ((mxc_mpc_regs_t *)MXC_BASE_MPC_SRAM3)
#define MXC_BASE_MPC_SRAM4 ((uint32_t)0x50095000UL)
#define MXC_MPC_SRAM4 ((mxc_mpc_regs_t *)MXC_BASE_MPC_SRAM4)
#define MXC_BASE_MPC_FLASH ((uint32_t)0x50096000UL)
#define MXC_MPC_FLASH ((mxc_mpc_regs_t *)MXC_BASE_MPC_FLASH)

/* Added for consistency and explicitness */
#define MXC_BASE_MPC_SRAM0_S MXC_BASE_MPC_SRAM0
#define MXC_MPC_SRAM0_S MXC_MPC_SRAM0
#define MXC_BASE_MPC_SRAM1_S MXC_BASE_MPC_SRAM1
#define MXC_MPC_SRAM1_S MXC_MPC_SRAM1
#define MXC_BASE_MPC_SRAM2_S MXC_BASE_MPC_SRAM2
#define MXC_MPC_SRAM2_S MXC_MPC_SRAM2
#define MXC_BASE_MPC_SRAM3_S MXC_BASE_MPC_SRAM3
#define MXC_MPC_SRAM3_S MXC_MPC_SRAM3
#define MXC_BASE_MPC_SRAM4_S MXC_BASE_MPC_SRAM4
#define MXC_MPC_SRAM4_S MXC_MPC_SRAM4
#define MXC_BASE_MPC_FLASH_S MXC_BASE_MPC_FLASH
#define MXC_MPC_FLASH_S MXC_MPC_FLASH

/* Grab the index associated with each memory region. */
#define MXC_MPC_GET_PHY_MEM_BASE(p)                  \
    ((p) == MXC_MPC_FLASH ? MXC_PHY_FLASH_MEM_BASE : \
     (p) == MXC_MPC_SRAM0 ? MXC_PHY_SRAM0_MEM_BASE : \
     (p) == MXC_MPC_SRAM1 ? MXC_PHY_SRAM1_MEM_BASE : \
     (p) == MXC_MPC_SRAM2 ? MXC_PHY_SRAM2_MEM_BASE : \
     (p) == MXC_MPC_SRAM3 ? MXC_PHY_SRAM3_MEM_BASE : \
     (p) == MXC_MPC_SRAM4 ? MXC_PHY_SRAM4_MEM_BASE : \
                            0)

#define MXC_MPC_GET_PHY_MEM_SIZE(p)                  \
    ((p) == MXC_MPC_FLASH ? MXC_PHY_FLASH_MEM_SIZE : \
     (p) == MXC_MPC_SRAM0 ? MXC_PHY_SRAM0_MEM_SIZE : \
     (p) == MXC_MPC_SRAM1 ? MXC_PHY_SRAM1_MEM_SIZE : \
     (p) == MXC_MPC_SRAM2 ? MXC_PHY_SRAM2_MEM_SIZE : \
     (p) == MXC_MPC_SRAM3 ? MXC_PHY_SRAM3_MEM_SIZE : \
     (p) == MXC_MPC_SRAM4 ? MXC_PHY_SRAM4_MEM_SIZE : \
                            0)

#define MXC_MPC_GET_IDX(p)      \
    ((p) == MXC_MPC_FLASH ? 0 : \
     (p) == MXC_MPC_SRAM0 ? 0 : \
     (p) == MXC_MPC_SRAM1 ? 1 : \
     (p) == MXC_MPC_SRAM2 ? 2 : \
     (p) == MXC_MPC_SRAM3 ? 3 : \
     (p) == MXC_MPC_SRAM4 ? 4 : \
                            -1)

#define MXC_MPC_FLASH_GET_BASE(i) ((i) == 0 ? MXC_MPC_FLASH : 0)

#define MXC_MPC_SRAM_GET_BASE(i) \
    ((i) == 0 ? MXC_MPC_SRAM0 :  \
     (i) == 1 ? MXC_MPC_SRAM1 :  \
     (i) == 2 ? MXC_MPC_SRAM2 :  \
     (i) == 3 ? MXC_MPC_SRAM3 :  \
     (i) == 4 ? MXC_MPC_SRAM4 :  \
                0)

/******************************************************************************/
/*                                                               Bit Shifting */
#define MXC_F_BIT_0 (1 << 0)
#define MXC_F_BIT_1 (1 << 1)
#define MXC_F_BIT_2 (1 << 2)
#define MXC_F_BIT_3 (1 << 3)
#define MXC_F_BIT_4 (1 << 4)
#define MXC_F_BIT_5 (1 << 5)
#define MXC_F_BIT_6 (1 << 6)
#define MXC_F_BIT_7 (1 << 7)
#define MXC_F_BIT_8 (1 << 8)
#define MXC_F_BIT_9 (1 << 9)
#define MXC_F_BIT_10 (1 << 10)
#define MXC_F_BIT_11 (1 << 11)
#define MXC_F_BIT_12 (1 << 12)
#define MXC_F_BIT_13 (1 << 13)
#define MXC_F_BIT_14 (1 << 14)
#define MXC_F_BIT_15 (1 << 15)
#define MXC_F_BIT_16 (1 << 16)
#define MXC_F_BIT_17 (1 << 17)
#define MXC_F_BIT_18 (1 << 18)
#define MXC_F_BIT_19 (1 << 19)
#define MXC_F_BIT_20 (1 << 20)
#define MXC_F_BIT_21 (1 << 21)
#define MXC_F_BIT_22 (1 << 22)
#define MXC_F_BIT_23 (1 << 23)
#define MXC_F_BIT_24 (1 << 24)
#define MXC_F_BIT_25 (1 << 25)
#define MXC_F_BIT_26 (1 << 26)
#define MXC_F_BIT_27 (1 << 27)
#define MXC_F_BIT_28 (1 << 28)
#define MXC_F_BIT_29 (1 << 29)
#define MXC_F_BIT_30 (1 << 30)
#define MXC_F_BIT_31 (1 << 31)

/******************************************************************************/
/*                                                               Bit Banding  */
#define BITBAND(reg, bit)                                                               \
    ((0xf0000000 & (uint32_t)(reg)) + 0x2000000 + (((uint32_t)(reg)&0x0fffffff) << 5) + \
     ((bit) << 2))

#define MXC_CLRBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit) = 0)
#define MXC_SETBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit) = 1)
#define MXC_GETBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit))

#define MXC_SETFIELD(reg, mask, setting) ((reg) = ((reg) & ~(mask)) | ((setting) & (mask)))

/******************************************************************************/
/*                                                         CPACR Definitions  */
/* Note: Added by Maxim Integrated, as these are missing from CMSIS/Core/Include/core_cm33.h */
#define SCB_CPACR_CP10_Pos 20 /*!< SCB CPACR: Coprocessor 10 Position */
#define SCB_CPACR_CP10_Msk (0x3UL << SCB_CPACR_CP10_Pos) /*!< SCB CPACR: Coprocessor 10 Mask */
#define SCB_CPACR_CP11_Pos 22 /*!< SCB CPACR: Coprocessor 11 Position */
#define SCB_CPACR_CP11_Msk (0x3UL << SCB_CPACR_CP11_Pos) /*!< SCB CPACR: Coprocessor 11 Mask */

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32657_INCLUDE_MAX32657_H_
