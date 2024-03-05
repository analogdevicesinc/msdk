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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MAX32665_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MAX32665_H_

#ifndef TARGET_NUM
#define TARGET_NUM 32665
#endif

#define MXC_NUMCORES 2

#include <stdint.h>

#ifndef FALSE
#define FALSE (0)
#endif

#ifndef TRUE
#define TRUE (1)
#endif

#if !defined(__GNUC__)
#define CMSIS_VECTAB_VIRTUAL
#define CMSIS_VECTAB_VIRTUAL_HEADER_FILE "nvic_table.h"
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

typedef enum {
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SVCall_IRQn = -5,
    DebugMonitor_IRQn = -4,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,

    /* Device-specific interrupt sources (external to ARM core)                 */
    /*                      table entry number                                  */
    /*                      ||||                                                */
    /*                      ||||  table offset address                          */
    /*                      vvvv  vvvvvv                                        */

    PF_IRQn = 0, /* 0x10  0x0040  16: Power Fail */
    WDT0_IRQn, /* 0x11  0x0044  17: Watchdog 0 */
    USB_IRQn, /* 0x12  0x0048  18: USB */
    RTC_IRQn, /* 0x13  0x004C  19: RTC */
    TRNG_IRQn, /* 0x14  0x0050  20: True Random Number Generator */
    TMR0_IRQn, /* 0x15  0x0054  21: Timer 0 */
    TMR1_IRQn, /* 0x16  0x0058  22: Timer 1 */
    TMR2_IRQn, /* 0x17  0x005C  23: Timer 2 */
    TMR3_IRQn, /* 0x18  0x0060  24: Timer 3*/
    TMR4_IRQn, /* 0x19  0x0064  25: Timer 4*/
    TMR5_IRQn, /* 0x1A  0x0068  26: Timer 5 */
    RSV11_IRQn, /* 0x1B  0x006C  27: Reserved */
    RSV12_IRQn, /* 0x1C  0x0070  28: Reserved */
    I2C0_IRQn, /* 0x1D  0x0074  29: I2C0 */
    UART0_IRQn, /* 0x1E  0x0078  30: UART 0 */
    UART1_IRQn, /* 0x1F  0x007C  31: UART 1 */
    SPI1_IRQn, /* 0x20  0x0080  32: SPI1 */
    SPI2_IRQn, /* 0x21  0x0084  33: SPI2 */
    RSV18_IRQn, /* 0x22  0x0088  34: Reserved */
    RSV19_IRQn, /* 0x23  0x008C  35: Reserved */
    ADC_IRQn, /* 0x24  0x0090  36: ADC */
    RSV21_IRQn, /* 0x25  0x0094  37: Reserved */
    RSV22_IRQn, /* 0x26  0x0098  38: Reserved */
    FLC0_IRQn, /* 0x27  0x009C  39: Flash Controller 0 */
    GPIO0_IRQn, /* 0x28  0x00A0  40: GPIO0 */
    GPIO1_IRQn, /* 0x29  0x00A4  41: GPIO1 */
    RSV26_IRQn, /* 0x2A  0x00A8  42: Reserved */
    TPU_IRQn, /* 0x2B  0x00AC  43: Crypto */
    DMA0_IRQn, /* 0x2C  0x00B0  44: DMA0 */
    DMA1_IRQn, /* 0x2D  0x00B4  45: DMA1 */
    DMA2_IRQn, /* 0x2E  0x00B8  46: DMA2 */
    DMA3_IRQn, /* 0x2F  0x00BC  47: DMA3 */
    RSV32_IRQn, /* 0x30  0x00C0  48: Reserved */
    RSV33_IRQn, /* 0x31  0x00C4  49: Reserved */
    UART2_IRQn, /* 0x32  0x00C8  50: UART 2 */
    RSV35_IRQn, /* 0x33  0x00CC  51: Reserved */
    I2C1_IRQn, /* 0x34  0x00D0  52: I2C1 */
    RSV36_IRQn, /* 0x35  0x00D4  53: Reserved */
    SPIXFC_IRQn, /* 0x36  0x00D8  54: SPI execute in place */
    BTLE_TX_DONE_IRQn, /* 0x37  0x00DC  55: BTLE TX Done */
    BTLE_RX_RCVD_IRQn, /* 0x38  0x00E0  56: BTLE RX Received */
    BTLE_RX_ENG_DET_IRQn, /* 0x39  0x00E4  57: BTLE RX Energy Detected */
    BTLE_SFD_DET_IRQn, /* 0x3A  0x00E8  58: BTLE SFD Detected */
    BTLE_SFD_TO_IRQn, /* 0x3B  0x00EC  59: BTLE SFD Timeout*/
    BTLE_GP_EVENT_IRQn, /* 0x3C  0x00F0  60: BTLE Timestamp*/
    BTLE_CFO_IRQn, /* 0x3D  0x00F4  61: BTLE CFO Done */
    BTLE_SIG_DET_IRQn, /* 0x3E  0x00F8  62: BTLE Signal Detected */
    BTLE_AGC_EVENT_IRQn, /* 0x3F  0x00FC  63: BTLE AGC Event */
    BTLE_RFFE_SPIM_IRQn, /* 0x40  0x0100  64: BTLE RFFE SPIM Done */
    BTLE_TX_AES_IRQn, /* 0x41  0x0104  65: BTLE TX AES Done */
    BTLE_RX_AES_IRQn, /* 0x42  0x0108  66: BTLE RX AES Done */
    BTLE_INV_APB_ADDR_IRQn, /* 0x43  0x010C  67: BTLE Invalid APB Address*/
    BTLE_IQ_DATA_VALID_IRQn, /* 0x44  0x0110  68: BTLE IQ Data Valid */
    WUT_IRQn, /* 0x45  0x0114  69: WUT Wakeup */
    GPIOWAKE_IRQn, /* 0x46  0x0118  70: GPIO Wakeup */
    RSV55_IRQn, /* 0x47  0x011C  71: Reserved */
    SPI0_IRQn, /* 0x48  0x0120  72: SPI0  AHB*/
    WDT1_IRQn, /* 0x49  0x0124  73: Watchdog 1 */
    RSV58_IRQn, /* 0x4A  0x0128  74: Reserved */
    PT_IRQn, /* 0x4B  0x012C  75: Pulse train */
    SDMA_IRQn, /* 0x4C  0x0130  76: Smart DMA 0 */
    RSV61_IRQn, /* 0x4D  0x0134  77: Reserved */
    I2C2_IRQn, /* 0x4E  0x0138  78: I2C 2 */
    RSV63_IRQn, /* 0x4F  0x013C  79: Reserved */
    RSV64_IRQn, /* 0x50  0x0140  80: Reserved */
    RSV65_IRQn, /* 0x51  0x0144  81: Reserved */
    SDHC_IRQn, /* 0x52  0x0148  82: SDIO/SDHC */
    OWM_IRQn, /* 0x53  0x014C  83: One Wire Master */
    DMA4_IRQn, /* 0x54  0x0150  84: DMA4 */
    DMA5_IRQn, /* 0x55  0x0154  85: DMA5 */
    DMA6_IRQn, /* 0x56  0x0158  86: DMA6 */
    DMA7_IRQn, /* 0x57  0x015C  87: DMA7 */
    DMA8_IRQn, /* 0x58  0x0160  88: DMA8 */
    DMA9_IRQn, /* 0x59  0x0164  89: DMA9 */
    DMA10_IRQn, /* 0x5A  0x0168  90: DMA10 */
    DMA11_IRQn, /* 0x5B  0x016C  91: DMA11 */
    DMA12_IRQn, /* 0x5C  0x0170  92: DMA12 */
    DMA13_IRQn, /* 0x5D  0x0174  93: DMA13 */
    DMA14_IRQn, /* 0x5E  0x0178  94: DMA14 */
    DMA15_IRQn, /* 0x5F  0x017C  95: DMA15 */
    USBDMA_IRQn, /* 0x60  0x0180  96: USB DMA */
    WDT2_IRQn, /* 0x61  0x0184  97: Watchdog Timer 2 */
    ECC_IRQn, /* 0x62  0x0188  98: Error Correction */
    DVS_IRQn, /* 0x63  0x018C  99: DVS Controller */
    SIMO_IRQn, /* 0x64 0x0190  100: SIMO Controller */
    SCA_IRQn, /* 0x65  0x0194  101: SCA */
    AUDIO_IRQn, /* 0x66  0x0198  102: Audio subsystem */
    FLC1_IRQn, /* 0x67  0x019C  103: Flash Control 1 */
    UART3_IRQn, /* 0x68  0x01A0  104: UART 3 */
    UART4_IRQn, /* 0x69  0x01A4  105: UART 4 */
    UART5_IRQn, /* 0x6A  0x01A8  106: UART 5 */
    CameraIF_IRQn, /* 0x6B  0x01AC  107: Camera IF */
    I3C_IRQn, /* 0x6C  0x01B0  108: I3C */
    HTMR0_IRQn, /* 0x6D  0x01B4  109: HTimer0 */
    HTMR1_IRQn, /* 0x6E  0x01B8  110: HTimer1 */
    MXC_IRQ_EXT_COUNT
} IRQn_Type;

#define MXC_IRQ_COUNT (MXC_IRQ_EXT_COUNT + 16)

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ----------------------  Configuration of the Cortex-M Processor and Core Peripherals  ---------------------- */
#define __CM4_REV 0x0100 /*!< Cortex-M4 Core Revision                                */
#define __MPU_PRESENT 1 /*!< MPU present or not                                     */
#define __NVIC_PRIO_BITS 3 /*!< Number of Bits used for Priority Levels                */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used           */
#define __FPU_PRESENT 1 /*!< FPU present or not                                     */

#include <core_cm4.h> /*!< Cortex-M4 processor and core peripherals               */

#include "system_max32665.h" /*!< System Header                                          */

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00020000UL
#define MXC_XIP_MEM_BASE 0x08000000UL
#define MXC_XIP_MEM_SIZE 0x08000000UL
#define MXC_FLASH0_MEM_BASE 0x10000000UL
#define MXC_FLASH1_MEM_BASE 0x10080000UL
#define MXC_FLASH_MEM_BASE MXC_FLASH0_MEM_BASE
#define MXC_FLASH_PAGE_SIZE 0x00002000UL
#define MXC_FLASH_MEM_SIZE 0x00080000UL
#define MXC_INFO0_MEM_BASE 0x10800000UL
#define MXC_INFO1_MEM_BASE 0x10804000UL
#define MXC_INFO_MEM_BASE MXC_INFO0_MEM_BASE
#define MXC_INFO_MEM_SIZE 0x00004000UL
#define MXC_SRAM_MEM_BASE 0x20000000UL
#define MXC_SRAM_MEM_SIZE 0x0008C000UL
#define MXC_XIP_DATA_MEM_BASE 0x80000000UL
#define MXC_XIP_DATA_MEM_SIZE 0x20000000UL

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX32665 peripheral modules.
*/

/******************************************************************************/
/*                                                             Global control */
#define MXC_BASE_GCR ((uint32_t)0x40000000UL)
#define MXC_GCR ((mxc_gcr_regs_t *)MXC_BASE_GCR)

/******************************************************************************/
/*                                            Non-battery backed SI Registers */
#define MXC_BASE_SIR ((uint32_t)0x40000400UL)
#define MXC_SIR ((mxc_sir_regs_t *)MXC_BASE_SIR)

/******************************************************************************/
/*                                        Non-battery backed Function Control */
#define MXC_BASE_FCR ((uint32_t)0x40000800UL)
#define MXC_FCR ((mxc_fcr_regs_t *)MXC_BASE_FCR)

/******************************************************************************/
/*                                                      Trust Protection Unit */
#define MXC_BASE_TPU ((uint32_t)0x40001000UL)
#define MXC_TPU ((mxc_tpu_regs_t *)MXC_BASE_TPU)

/******************************************************************************/
/*                                                                        RPU */
#define MXC_BASE_RPU ((uint32_t)0x40002000UL)
#define MXC_RPU ((mxc_rpu_regs_t *)MXC_BASE_RPU)
#define MXC_RPU_NUM_BUS_MASTERS 9

/******************************************************************************/
/*                                                                   Watchdog */
#define MXC_BASE_WDT0 ((uint32_t)0x40003000UL)
#define MXC_WDT0 ((mxc_wdt_regs_t *)MXC_BASE_WDT0)
#define MXC_BASE_WDT1 ((uint32_t)0x40003400UL)
#define MXC_WDT1 ((mxc_wdt_regs_t *)MXC_BASE_WDT1)
#define MXC_BASE_WDT2 ((uint32_t)0x40003800UL)
#define MXC_WDT2 ((mxc_wdt_regs_t *)MXC_BASE_WDT2)

/******************************************************************************/
/*                                                           Security Monitor */
#define MXC_BASE_SMON ((uint32_t)0x40004000UL)
#define MXC_SMON ((mxc_smon_regs_t *)MXC_BASE_SMON)

/******************************************************************************/
/*                                                                       SIMO */
#define MXC_BASE_SIMO ((uint32_t)0x40004400UL)
#define MXC_SIMO ((mxc_simo_regs_t *)MXC_BASE_SIMO)

/******************************************************************************/
/*                                                                        DVS */
#define MXC_BASE_DVS ((uint32_t)0x40004800UL)
#define MXC_DVS ((mxc_dvs_regs_t *)MXC_BASE_DVS)

/******************************************************************************/
/*                                                                   AES Keys */
#define MXC_BASE_AESKEYS ((uint32_t)0x40005000UL)
#define MXC_AESKEYS ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS)

// DEPRECATED(1-10-2023): Scheduled for removal.
#define MXC_BASE_AESKEY MXC_BASE_AESKEYS
#define MXC_AESKEY ((mxc_aes_key_regs_t *)MXC_BASE_AESKEY)

/******************************************************************************/
/*                                         Trim System Initalization Register */
#define MXC_BASE_TRIMSIR ((uint32_t)0x40005400UL)
#define MXC_TRIMSIR ((mxc_trimsir_regs_t *)MXC_BASE_TRIMSIR)

/******************************************************************************/
/*                                                            Real Time Clock */
#define MXC_BASE_RTC ((uint32_t)0x40006000UL)
#define MXC_RTC ((mxc_rtc_regs_t *)MXC_BASE_RTC)

/******************************************************************************/
/*                                                               Wakeup Timer */
#define MXC_BASE_WUT ((uint32_t)0x40006400UL)
#define MXC_WUT ((mxc_wut_regs_t *)MXC_BASE_WUT)

/******************************************************************************/
/*                                                            Power Sequencer */
#define MXC_BASE_PWRSEQ ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ)
/******************************************************************************/
/*                                                            Power Sequencer */
#define MXC_BASE_MCR ((uint32_t)0x40006C00UL)
#define MXC_MCR ((mxc_mcr_regs_t *)MXC_BASE_MCR)

/******************************************************************************/
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (2)
#define MXC_CFG_GPIO_PINS_PORT (32)

#define MXC_BASE_GPIO0 ((uint32_t)0x40008000UL)
#define MXC_GPIO0 ((mxc_gpio_regs_t *)MXC_BASE_GPIO0)
#define MXC_BASE_GPIO1 ((uint32_t)0x40009000UL)
#define MXC_GPIO1 ((mxc_gpio_regs_t *)MXC_BASE_GPIO1)

#define MXC_GPIO_GET_IDX(p) ((p) == MXC_GPIO0 ? 0 : (p) == MXC_GPIO1 ? 1 : -1)

#define MXC_GPIO_GET_GPIO(i) ((i) == 0 ? MXC_GPIO0 : (i) == 1 ? MXC_GPIO1 : 0)

#define MXC_GPIO_GET_IRQ(i) ((i) == 0 ? GPIO0_IRQn : (i) == 1 ? GPIO1_IRQn : (IRQn_Type)0)

/******************************************************************************/
/*                                                                      Timer */
#define MXC_CFG_TMR_INSTANCES (6)

#define MXC_BASE_TMR0 ((uint32_t)0x40010000UL)
#define MXC_TMR0 ((mxc_tmr_regs_t *)MXC_BASE_TMR0)
#define MXC_BASE_TMR1 ((uint32_t)0x40011000UL)
#define MXC_TMR1 ((mxc_tmr_regs_t *)MXC_BASE_TMR1)
#define MXC_BASE_TMR2 ((uint32_t)0x40012000UL)
#define MXC_TMR2 ((mxc_tmr_regs_t *)MXC_BASE_TMR2)
#define MXC_BASE_TMR3 ((uint32_t)0x40013000UL)
#define MXC_TMR3 ((mxc_tmr_regs_t *)MXC_BASE_TMR3)
#define MXC_BASE_TMR4 ((uint32_t)0x40014000UL)
#define MXC_TMR4 ((mxc_tmr_regs_t *)MXC_BASE_TMR4)
#define MXC_BASE_TMR5 ((uint32_t)0x40015000UL)
#define MXC_TMR5 ((mxc_tmr_regs_t *)MXC_BASE_TMR5)

#define MXC_TMR_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? TMR0_IRQn : \
                (i) == 1 ? TMR1_IRQn : \
                (i) == 2 ? TMR2_IRQn : \
                (i) == 3 ? TMR3_IRQn : \
                (i) == 4 ? TMR4_IRQn : \
                (i) == 5 ? TMR5_IRQn : \
                           0)

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

#define MXC_TMR_GET_IDX(p) \
    ((p) == MXC_TMR0 ? 0 : \
     (p) == MXC_TMR1 ? 1 : \
     (p) == MXC_TMR2 ? 2 : \
     (p) == MXC_TMR3 ? 3 : \
     (p) == MXC_TMR4 ? 4 : \
     (p) == MXC_TMR5 ? 5 : \
                       -1)

/******************************************************************************/
/*                                                           High Speed Timer */
#define MXC_BASE_HTMR0 ((uint32_t)0x4001B000UL)
#define MXC_HTMR0 ((mxc_htmr_regs_t *)MXC_BASE_HTMR0)
#define MXC_BASE_HTMR1 ((uint32_t)0x4001C000UL)
#define MXC_HTMR1 ((mxc_htmr_regs_t *)MXC_BASE_HTMR1)

/******************************************************************************/
/*                                                                        I2C */
#define MXC_I2C_INSTANCES (3)

#define MXC_BASE_I2C0_BUS0 ((uint32_t)0x4001D000UL)
#define MXC_I2C0_BUS0 ((mxc_i2c_regs_t *)MXC_BASE_I2C0_BUS0)
#define MXC_BASE_I2C1_BUS0 ((uint32_t)0x4001E000UL)
#define MXC_I2C1_BUS0 ((mxc_i2c_regs_t *)MXC_BASE_I2C1_BUS0)
#define MXC_BASE_I2C2_BUS0 ((uint32_t)0x4001F000UL)
#define MXC_I2C2_BUS0 ((mxc_i2c_regs_t *)MXC_BASE_I2C2_BUS0)

#define MXC_BASE_I2C0_BUS1 ((uint32_t)0x4011D000UL)
#define MXC_I2C0_BUS1 ((mxc_i2c_regs_t *)MXC_BASE_I2C0_BUS1)
#define MXC_BASE_I2C1_BUS1 ((uint32_t)0x4011E000UL)
#define MXC_I2C1_BUS1 ((mxc_i2c_regs_t *)MXC_BASE_I2C1_BUS1)
#define MXC_BASE_I2C2_BUS1 ((uint32_t)0x4011F000UL)
#define MXC_I2C2_BUS1 ((mxc_i2c_regs_t *)MXC_BASE_I2C2_BUS1)

#define MXC_I2C_GET_IRQ(i)                  \
    (IRQn_Type)((i) == 0x0    ? I2C0_IRQn : \
                (i) == 0x1    ? I2C1_IRQn : \
                (i) == 0x2    ? I2C2_IRQn : \
                (i) == 0x8000 ? I2C0_IRQn : \
                (i) == 0x8001 ? I2C1_IRQn : \
                (i) == 0x8002 ? I2C2_IRQn : \
                                0)

#define MXC_I2C_GET_BASE(i)               \
    ((i) == 0x0    ? MXC_BASE_I2C0_BUS0 : \
     (i) == 0x1    ? MXC_BASE_I2C1_BUS0 : \
     (i) == 0x2    ? MXC_BASE_I2C2_BUS0 : \
     (i) == 0x8000 ? MXC_BASE_I2C0_BUS1 : \
     (i) == 0x8001 ? MXC_BASE_I2C1_BUS1 : \
     (i) == 0x8002 ? MXC_BASE_I2C2_BUS1 : \
                     0)

#define MXC_I2C_GET_IDX(p)           \
    ((p) == MXC_I2C0_BUS0 ? 0x0 :    \
     (p) == MXC_I2C1_BUS0 ? 0x1 :    \
     (p) == MXC_I2C2_BUS0 ? 0x2 :    \
     (p) == MXC_I2C0_BUS1 ? 0x8000 : \
     (p) == MXC_I2C1_BUS1 ? 0x8001 : \
     (p) == MXC_I2C2_BUS1 ? 0x8002 : \
                            -1)

#define MXC_I2C_GET_I2C(p)           \
    ((p) == 0x0    ? MXC_I2C0_BUS0 : \
     (p) == 0x1    ? MXC_I2C1_BUS0 : \
     (p) == 0x2    ? MXC_I2C2_BUS0 : \
     (p) == 0x8000 ? MXC_I2C0_BUS1 : \
     (p) == 0x8001 ? MXC_I2C1_BUS1 : \
     (p) == 0x8002 ? MXC_I2C2_BUS1 : \
                     0)
#define MXC_I2C_FIFO_DEPTH (8)

/******************************************************************************/
/*                                                      SPI Execute in Place  */
#define MXC_BASE_SPIXFM ((uint32_t)0x40026000UL)
#define MXC_SPIXFM ((mxc_spixfm_regs_t *)MXC_BASE_SPIXFM)

#define MXC_BASE_SPIXFC_FIFO ((uint32_t)0x400BC000UL)
#define MXC_SPIXFC_FIFO ((mxc_spixfc_fifo_regs_t *)MXC_BASE_SPIXFC_FIFO)
/******************************************************************************/
/*                                                SPI Execute in Place Master */

#define MXC_CFG_SPIXFC_FIFO_DEPTH (16)

#define MXC_BASE_SPIXFC ((uint32_t)0x40027000UL)
#define MXC_SPIXFC ((mxc_spixfc_regs_t *)MXC_BASE_SPIXFC)

/******************************************************************************/
/*                                                                        DMA */
#define MXC_DMA_CHANNELS (16)
#define MXC_DMA_INSTANCES (2)
#define MXC_DMA_CH_OFFSET (8)

#define MXC_BASE_DMA0 ((uint32_t)0x40028000UL)
#define MXC_DMA0 ((mxc_dma_regs_t *)MXC_BASE_DMA0)
#define MXC_BASE_DMA1 ((uint32_t)0x40035000UL)
#define MXC_DMA1 ((mxc_dma_regs_t *)MXC_BASE_DMA1)

#define MXC_DMA_GET_BASE(i) ((i) == 0 ? MXC_BASE_DMA0 : (i) == 1 ? MXC_BASE_DMA1 : 0)

#define MXC_DMA_GET_DMA(i) ((i) == 0 ? MXC_DMA0 : (i) == 1 ? MXC_DMA1 : 0)

#define MXC_DMA_GET_IDX(p) ((p) == MXC_DMA0 ? 0 : (p) == MXC_DMA1 ? 1 : -1)

#define MXC_DMA0_CH_GET_IRQ(i)            \
    ((IRQn_Type)(((i) == 0) ? DMA0_IRQn : \
                 ((i) == 1) ? DMA1_IRQn : \
                 ((i) == 2) ? DMA2_IRQn : \
                 ((i) == 3) ? DMA3_IRQn : \
                 ((i) == 4) ? DMA4_IRQn : \
                 ((i) == 5) ? DMA5_IRQn : \
                 ((i) == 6) ? DMA6_IRQn : \
                 ((i) == 7) ? DMA7_IRQn : \
                              0))

#define MXC_DMA1_CH_GET_IRQ(i)             \
    ((IRQn_Type)(((i) == 0) ? DMA8_IRQn :  \
                 ((i) == 1) ? DMA9_IRQn :  \
                 ((i) == 2) ? DMA10_IRQn : \
                 ((i) == 3) ? DMA11_IRQn : \
                 ((i) == 4) ? DMA12_IRQn : \
                 ((i) == 5) ? DMA13_IRQn : \
                 ((i) == 6) ? DMA14_IRQn : \
                 ((i) == 7) ? DMA15_IRQn : \
                              0))

#define MXC_DMA_CH_GET_IRQ(i)                                                       \
    (((i) > (MXC_DMA_CH_OFFSET - 1)) ? MXC_DMA1_CH_GET_IRQ(i % MXC_DMA_CH_OFFSET) : \
                                       MXC_DMA0_CH_GET_IRQ(i))

/* Create alias for MXC_DMA0 for backwards compatibility with code that was
   written for parts that only had one DMA instance. */
#define MXC_DMA MXC_DMA0

/******************************************************************************/
/*                                                                        FLC */
#define MXC_FLC_INSTANCES (2)

#define MXC_BASE_FLC0 ((uint32_t)0x40029000UL)
#define MXC_FLC0 ((mxc_flc_regs_t *)MXC_BASE_FLC0)
#define MXC_BASE_FLC1 ((uint32_t)0x40029400UL)
#define MXC_FLC1 ((mxc_flc_regs_t *)MXC_BASE_FLC1)

#define MXC_FLC_GET_IRQ(i) (IRQn_Type)((i) == 0 ? FLC0_IRQn : (i) == 1 ? FLC1_IRQn : 0)

#define MXC_FLC_GET_BASE(i) ((i) == 0 ? MXC_BASE_FLC0 : (i) == 1 ? MXC_BASE_FLC1 : 0)

#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC0 : (i) == 1 ? MXC_FLC1 : 0)

#define MXC_FLC_GET_IDX(p) ((p) == MXC_FLC0 ? 0 : (p) == MXC_FLC1 ? 1 : -1)

/******************************************************************************/
/*                                                          Instruction Cache */
#define MXC_ICC_INSTANCES (2)

#define MXC_BASE_ICC0 ((uint32_t)0x4002A000UL)
#define MXC_ICC0 ((mxc_icc_regs_t *)MXC_BASE_ICC0)
#define MXC_BASE_ICC1 ((uint32_t)0x4002A800UL)
#define MXC_ICC1 ((mxc_icc_regs_t *)MXC_BASE_ICC1)

#define MXC_ICC MXC_ICC0

#define MXC_ICC_GET_BASE(i) ((i) == 0 ? MXC_BASE_ICC0 : (i) == 1 ? MXC_BASE_ICC1 : 0)

#define MXC_ICC_GET_ICC(i) ((i) == 0 ? MXC_ICC0 : (i) == 1 ? MXC_ICC1 : 0)

#define MXC_ICC_GET_IDX(p) ((p) == MXC_ICC0 ? 0 : (p) == MXC_ICC1 ? 1 : -1)

/******************************************************************************/
/*                                                      Instruction Cache XIP */
#define MXC_BASE_SFCC ((uint32_t)0x4002F000UL)
#define MXC_SFCC ((mxc_icc_regs_t *)MXC_BASE_SFCC)

/******************************************************************************/
/*                                                                 Data Cache */
#define MXC_BASE_SRCC ((uint32_t)0x40033000UL)
#define MXC_SRCC ((mxc_srcc_regs_t *)MXC_BASE_SRCC)

/******************************************************************************/
/*                                                                        ADC */
#define MXC_BASE_ADC ((uint32_t)0x40034000UL)
#define MXC_ADC ((mxc_adc_regs_t *)MXC_BASE_ADC)
#define MXC_ADC_MAX_CLOCK 8000000 // Maximum ADC clock in Hz

/******************************************************************************/
/*                                                                  Smart DMA */
#define MXC_BASE_SDMA ((uint32_t)0x40036000UL)
#define MXC_SDMA ((mxc_sdma_regs_t *)MXC_BASE_SDMA)

/******************************************************************************/
/*                                                               SPI XIP Data */
#define MXC_BASE_SPIXR ((uint32_t)0x4003A000UL)
#define MXC_SPIXR ((mxc_spixr_regs_t *)MXC_BASE_SPIXR)

/*******************************************************************************/
/*                                                      Pulse Train Generation */
#define MXC_CFG_PT_INSTANCES (16)

#define MXC_BASE_PTG_BUS0 ((uint32_t)0x4003C000UL)
#define MXC_PTG_BUS0 ((mxc_ptg_regs_t *)MXC_BASE_PTG_BUS0)
#define MXC_BASE_PT0_BUS0 ((uint32_t)0x4003C020UL)
#define MXC_PT0_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT0_BUS0)
#define MXC_BASE_PT1_BUS0 ((uint32_t)0x4003C040UL)
#define MXC_PT1_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT1_BUS0)
#define MXC_BASE_PT2_BUS0 ((uint32_t)0x4003C060UL)
#define MXC_PT2_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT2_BUS0)
#define MXC_BASE_PT3_BUS0 ((uint32_t)0x4003C080UL)
#define MXC_PT3_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT3_BUS0)
#define MXC_BASE_PT4_BUS0 ((uint32_t)0x4003C0A0UL)
#define MXC_PT4_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT4_BUS0)
#define MXC_BASE_PT5_BUS0 ((uint32_t)0x4003C0C0UL)
#define MXC_PT5_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT5_BUS0)
#define MXC_BASE_PT6_BUS0 ((uint32_t)0x4003C0E0UL)
#define MXC_PT6_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT6_BUS0)
#define MXC_BASE_PT7_BUS0 ((uint32_t)0x4003C100UL)
#define MXC_PT7_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT7_BUS0)
#define MXC_BASE_PT8_BUS0 ((uint32_t)0x4003C120UL)
#define MXC_PT8_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT8_BUS0)
#define MXC_BASE_PT9_BUS0 ((uint32_t)0x4003C140UL)
#define MXC_PT9_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT9_BUS0)
#define MXC_BASE_PT10_BUS0 ((uint32_t)0x4003C160UL)
#define MXC_PT10_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT10_BUS0)
#define MXC_BASE_PT11_BUS0 ((uint32_t)0x4003C180UL)
#define MXC_PT11_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT11_BUS0)
#define MXC_BASE_PT12_BUS0 ((uint32_t)0x4003C1A0UL)
#define MXC_PT12_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT12_BUS0)
#define MXC_BASE_PT13_BUS0 ((uint32_t)0x4003C1C0UL)
#define MXC_PT13_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT13_BUS0)
#define MXC_BASE_PT14_BUS0 ((uint32_t)0x4003C1E0UL)
#define MXC_PT14_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT14_BUS0)
#define MXC_BASE_PT15_BUS0 ((uint32_t)0x4003C200UL)
#define MXC_PT15_BUS0 ((mxc_pt_regs_t *)MXC_BASE_PT15_BUS0)

#define MXC_BASE_PTG_BUS1 ((uint32_t)0x4013C000UL)
#define MXC_PTG_BUS1 ((mxc_ptg_regs_t *)MXC_BASE_PTG_BUS1)
#define MXC_BASE_PT0_BUS1 ((uint32_t)0x4013C020UL)
#define MXC_PT0_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT0_BUS1)
#define MXC_BASE_PT1_BUS1 ((uint32_t)0x4013C040UL)
#define MXC_PT1_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT1_BUS1)
#define MXC_BASE_PT2_BUS1 ((uint32_t)0x4013C060UL)
#define MXC_PT2_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT2_BUS1)
#define MXC_BASE_PT3_BUS1 ((uint32_t)0x4013C080UL)
#define MXC_PT3_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT3_BUS1)
#define MXC_BASE_PT4_BUS1 ((uint32_t)0x4013C0A0UL)
#define MXC_PT4_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT4_BUS1)
#define MXC_BASE_PT5_BUS1 ((uint32_t)0x4013C0C0UL)
#define MXC_PT5_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT5_BUS1)
#define MXC_BASE_PT6_BUS1 ((uint32_t)0x4013C0E0UL)
#define MXC_PT6_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT6_BUS1)
#define MXC_BASE_PT7_BUS1 ((uint32_t)0x4013C100UL)
#define MXC_PT7_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT7_BUS1)
#define MXC_BASE_PT8_BUS1 ((uint32_t)0x4013C120UL)
#define MXC_PT8_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT8_BUS1)
#define MXC_BASE_PT9_BUS1 ((uint32_t)0x4013C140UL)
#define MXC_PT9_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT9_BUS1)
#define MXC_BASE_PT10_BUS1 ((uint32_t)0x4013C160UL)
#define MXC_PT10_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT10_BUS1)
#define MXC_BASE_PT11_BUS1 ((uint32_t)0x4013C180UL)
#define MXC_PT11_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT11_BUS1)
#define MXC_BASE_PT12_BUS1 ((uint32_t)0x4013C1A0UL)
#define MXC_PT12_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT12_BUS1)
#define MXC_BASE_PT13_BUS1 ((uint32_t)0x4013C1C0UL)
#define MXC_PT13_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT13_BUS1)
#define MXC_BASE_PT14_BUS1 ((uint32_t)0x4013C1E0UL)
#define MXC_PT14_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT14_BUS1)
#define MXC_BASE_PT15_BUS1 ((uint32_t)0x4013C200UL)
#define MXC_PT15_BUS1 ((mxc_pt_regs_t *)MXC_BASE_PT15_BUS1)

#define MXC_PT_GET_BASE(i)                \
    ((i) == 0x0    ? MXC_BASE_PT0_BUS0 :  \
     (i) == 0x1    ? MXC_BASE_PT1_BUS0 :  \
     (i) == 0x2    ? MXC_BASE_PT2_BUS0 :  \
     (i) == 0x3    ? MXC_BASE_PT3_BUS0 :  \
     (i) == 0x4    ? MXC_BASE_PT4_BUS0 :  \
     (i) == 0x5    ? MXC_BASE_PT5_BUS0 :  \
     (i) == 0x6    ? MXC_BASE_PT6_BUS0 :  \
     (i) == 0x7    ? MXC_BASE_PT7_BUS0 :  \
     (i) == 0x8    ? MXC_BASE_PT8_BUS0 :  \
     (i) == 0x9    ? MXC_BASE_PT9_BUS0 :  \
     (i) == 0xA    ? MXC_BASE_PT10_BUS0 : \
     (i) == 0xB    ? MXC_BASE_PT11_BUS0 : \
     (i) == 0xC    ? MXC_BASE_PT12_BUS0 : \
     (i) == 0xD    ? MXC_BASE_PT13_BUS0 : \
     (i) == 0xE    ? MXC_BASE_PT14_BUS0 : \
     (i) == 0xF    ? MXC_BASE_PT15_BUS0 : \
     (i) == 0x8000 ? MXC_BASE_PT0_BUS1 :  \
     (i) == 0x8001 ? MXC_BASE_PT1_BUS1 :  \
     (i) == 0x8002 ? MXC_BASE_PT2_BUS1 :  \
     (i) == 0x8003 ? MXC_BASE_PT3_BUS1 :  \
     (i) == 0x8004 ? MXC_BASE_PT4_BUS1 :  \
     (i) == 0x8005 ? MXC_BASE_PT5_BUS1 :  \
     (i) == 0x8006 ? MXC_BASE_PT6_BUS1 :  \
     (i) == 0x8007 ? MXC_BASE_PT7_BUS1 :  \
     (i) == 0x8008 ? MXC_BASE_PT8_BUS1 :  \
     (i) == 0x8009 ? MXC_BASE_PT9_BUS1 :  \
     (i) == 0x800A ? MXC_BASE_PT10_BUS1 : \
     (i) == 0x800B ? MXC_BASE_PT11_BUS1 : \
     (i) == 0x800C ? MXC_BASE_PT12_BUS1 : \
     (i) == 0x800D ? MXC_BASE_PT13_BUS1 : \
     (i) == 0x800E ? MXC_BASE_PT14_BUS1 : \
     (i) == 0x800F ? MXC_BASE_PT15_BUS1 : \
                     0)

#define MXC_PT_GET_PT(i)             \
    ((i) == 0x0    ? MXC_PT0_BUS0 :  \
     (i) == 0x1    ? MXC_PT1_BUS0 :  \
     (i) == 0x2    ? MXC_PT2_BUS0 :  \
     (i) == 0x3    ? MXC_PT3_BUS0 :  \
     (i) == 0x4    ? MXC_PT4_BUS0 :  \
     (i) == 0x5    ? MXC_PT5_BUS0 :  \
     (i) == 0x6    ? MXC_PT6_BUS0 :  \
     (i) == 0x7    ? MXC_PT7_BUS0 :  \
     (i) == 0x8    ? MXC_PT8_BUS0 :  \
     (i) == 0x9    ? MXC_PT9_BUS0 :  \
     (i) == 0xA    ? MXC_PT10_BUS0 : \
     (i) == 0xB    ? MXC_PT11_BUS0 : \
     (i) == 0xC    ? MXC_PT12_BUS0 : \
     (i) == 0xD    ? MXC_PT13_BUS0 : \
     (i) == 0xE    ? MXC_PT14_BUS0 : \
     (i) == 0xF    ? MXC_PT15_BUS0 : \
     (i) == 0x8000 ? MXC_PT0_BUS1 :  \
     (i) == 0x8001 ? MXC_PT1_BUS1 :  \
     (i) == 0x8002 ? MXC_PT2_BUS1 :  \
     (i) == 0x8003 ? MXC_PT3_BUS1 :  \
     (i) == 0x8004 ? MXC_PT4_BUS1 :  \
     (i) == 0x8005 ? MXC_PT5_BUS1 :  \
     (i) == 0x8006 ? MXC_PT6_BUS1 :  \
     (i) == 0x8007 ? MXC_PT7_BUS1 :  \
     (i) == 0x8008 ? MXC_PT8_BUS1 :  \
     (i) == 0x8009 ? MXC_PT9_BUS1 :  \
     (i) == 0x800A ? MXC_PT10_BUS1 : \
     (i) == 0x800B ? MXC_PT11_BUS1 : \
     (i) == 0x800C ? MXC_PT12_BUS1 : \
     (i) == 0x800D ? MXC_PT13_BUS1 : \
     (i) == 0x800E ? MXC_PT14_BUS1 : \
     (i) == 0x800F ? MXC_PT15_BUS1 : \
                     0)

#define MXC_PT_GET_IDX(p)            \
    ((p) == MXC_PT0_BUS0  ? 0x0 :    \
     (p) == MXC_PT1_BUS0  ? 0x1 :    \
     (p) == MXC_PT2_BUS0  ? 0x2 :    \
     (p) == MXC_PT3_BUS0  ? 0x3 :    \
     (p) == MXC_PT4_BUS0  ? 0x4 :    \
     (p) == MXC_PT5_BUS0  ? 0x5 :    \
     (p) == MXC_PT6_BUS0  ? 0x6 :    \
     (p) == MXC_PT7_BUS0  ? 0x7 :    \
     (p) == MXC_PT8_BUS0  ? 0x8 :    \
     (p) == MXC_PT9_BUS0  ? 0x9 :    \
     (p) == MXC_PT10_BUS0 ? 0xA :    \
     (p) == MXC_PT11_BUS0 ? 0xB :    \
     (p) == MXC_PT12_BUS0 ? 0xC :    \
     (p) == MXC_PT13_BUS0 ? 0xD :    \
     (p) == MXC_PT14_BUS0 ? 0xE :    \
     (p) == MXC_PT15_BUS0 ? 0xF :    \
     (p) == MXC_PT0_BUS1  ? 0x8000 : \
     (p) == MXC_PT1_BUS1  ? 0x8001 : \
     (p) == MXC_PT2_BUS1  ? 0x8002 : \
     (p) == MXC_PT3_BUS1  ? 0x8003 : \
     (p) == MXC_PT4_BUS1  ? 0x8004 : \
     (p) == MXC_PT5_BUS1  ? 0x8005 : \
     (p) == MXC_PT6_BUS1  ? 0x8006 : \
     (p) == MXC_PT7_BUS1  ? 0x8007 : \
     (p) == MXC_PT8_BUS1  ? 0x8008 : \
     (p) == MXC_PT9_BUS1  ? 0x8009 : \
     (p) == MXC_PT10_BUS1 ? 0x800A : \
     (p) == MXC_PT11_BUS1 ? 0x800B : \
     (p) == MXC_PT12_BUS1 ? 0x800C : \
     (p) == MXC_PT13_BUS1 ? 0x800D : \
     (p) == MXC_PT14_BUS1 ? 0x800E : \
     (p) == MXC_PT15_BUS1 ? 0x800F : \
                            -1)

#define MXC_PT_GET_BUS(i) (((i)&0x00100000UL) >> 20)

#define MXC_PTG_GET_PTG(i) \
    (MXC_PT_GET_BUS((i)) == 0 ? MXC_PTG_BUS0 : MXC_PT_GET_BUS((i)) == 1 ? MXC_PTG_BUS1 : 0)

/******************************************************************************/
/*                                                            One Wire Master */
#define MXC_BASE_OWM ((uint32_t)0x4003D000UL)
#define MXC_OWM ((mxc_owm_regs_t *)MXC_BASE_OWM)

/******************************************************************************/
/*                                                                  Semaphore */
#define MXC_CFG_SEMA_INSTANCES (8)

#define MXC_BASE_SEMA ((uint32_t)0x4003E000UL)
#define MXC_SEMA ((mxc_sema_regs_t *)MXC_BASE_SEMA)

/******************************************************************************/
/*                                               UART / Serial Port Interface */
#define MXC_UART_INSTANCES (3)
#define MXC_UART_FIFO_DEPTH (32)

#define MXC_BASE_UART0 ((uint32_t)0x40042000UL)
#define MXC_UART0 ((mxc_uart_regs_t *)MXC_BASE_UART0)
#define MXC_BASE_UART1 ((uint32_t)0x40043000UL)
#define MXC_UART1 ((mxc_uart_regs_t *)MXC_BASE_UART1)
#define MXC_BASE_UART2 ((uint32_t)0x40044000UL)
#define MXC_UART2 ((mxc_uart_regs_t *)MXC_BASE_UART2)

#define MXC_UART_GET_IRQ(i)                        \
    (IRQn_Type)((i) == 0            ? UART0_IRQn : \
                (IRQn_Type)(i) == 1 ? UART1_IRQn : \
                (IRQn_Type)(i) == 2 ? UART2_IRQn : \
                                      0)

#define MXC_UART_GET_BASE(i) \
    ((i) == 0 ? MXC_BASE_UART0 : (i) == 1 ? MXC_BASE_UART1 : (i) == 2 ? MXC_BASE_UART2 : 0)

#define MXC_UART_GET_UART(i) \
    ((i) == 0 ? MXC_UART0 : (i) == 1 ? MXC_UART1 : (i) == 2 ? MXC_UART2 : 0)

#define MXC_UART_GET_IDX(p) \
    ((p) == MXC_UART0 ? 0 : (p) == MXC_UART1 ? 1 : (p) == MXC_UART2 ? 2 : -1)

/******************************************************************************/
/*                                                                     SPI */
#define MXC_SPI_INSTANCES (3)
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

#define MXC_BASE_SPI0 ((uint32_t)0x400BE000UL)
#define MXC_SPI0 ((mxc_spi_regs_t *)MXC_BASE_SPI0)
#define MXC_BASE_SPI1 ((uint32_t)0x40046000UL)
#define MXC_SPI1 ((mxc_spi_regs_t *)MXC_BASE_SPI1)
#define MXC_BASE_SPI2 ((uint32_t)0x40047000UL)
#define MXC_SPI2 ((mxc_spi_regs_t *)MXC_BASE_SPI2)

#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI0 ? 0 : (p) == MXC_SPI1 ? 1 : (p) == MXC_SPI2 ? 2 : -1)

#define MXC_SPI_GET_BASE(i) \
    ((i) == 0 ? MXC_BASE_SPI0 : (i) == 1 ? MXC_BASE_SPI1 : (i) == 2 ? MXC_BASE_SPI2 : 0)

#define MXC_SPI_GET_SPI(i) ((i) == 0 ? MXC_SPI0 : (i) == 1 ? MXC_SPI1 : (i) == 2 ? MXC_SPI2 : 0)

#define MXC_SPI_GET_IRQ(i) \
    (IRQn_Type)((i) == 0 ? SPI0_IRQn : (i) == 1 ? SPI1_IRQn : (i) == 2 ? SPI2_IRQn : 0)

/******************************************************************************/
/*                                                                       TRNG */
#define MXC_BASE_TRNG ((uint32_t)0x4004D000UL)
#define MXC_TRNG ((mxc_trng_regs_t *)MXC_BASE_TRNG)

/******************************************************************************/
/*                                                            Audio Subsystem */
#define MXC_BASE_AUDIO ((uint32_t)0x4004C000UL)
#define MXC_AUDIO ((mxc_audio_regs_t *)MXC_BASE_AUDIO)

/******************************************************************************/
/*                                                       Bluetooth Low Energy */
#define MXC_BASE_BTLE (0x40050000UL)
#define MXC_BTLE ((mxc_btle_regs_t *)MXC_BASE_BTLE)
#define MXC_BASE_BTLE_DBB_CTRL (MXC_BASE_BTLE + 0x1000)
#define MXC_BASE_BTLE_DBB_TX (MXC_BASE_BTLE + 0x2000)
#define MXC_BASE_BTLE_DBB_RX (MXC_BASE_BTLE + 0x3000)
#define MXC_BASE_BTLE_DBB_EXT_RFFE (MXC_BASE_BTLE + 0x8000)

// Base address definitions needed for DBB register definitions in BTLE stack
#define DBB_CTRL_BASE MXC_BASE_BTLE_DBB_CTRL
#define DBB_TX_BASE MXC_BASE_BTLE_DBB_TX
#define DBB_RX_BASE MXC_BASE_BTLE_DBB_RX
#define DBB_EXT_RFFE_BASE MXC_BASE_BTLE_DBB_EXT_RFFE

/******************************************************************************/
/*                                                                        USB */
#define MXC_BASE_USBHS ((uint32_t)0x400B1000UL)
#define MXC_USBHS ((mxc_usbhs_regs_t *)MXC_BASE_USBHS)
#define MXC_USBHS_NUM_EP 12 /* HW must have at least EP 0 CONTROL + 11 IN/OUT */
#define MXC_USBHS_NUM_DMA 8 /* HW must have at least this many DMA channels */
#define MXC_USBHS_MAX_PACKET 512

/******************************************************************************/
/*                                                                       SDHC */
#define MXC_BASE_SDHC ((uint32_t)0x400B6000UL)
#define MXC_SDHC ((mxc_sdhc_regs_t *)MXC_BASE_SDHC)

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

#define MXC_SETFIELD(reg, mask, setting) (reg = ((reg) & ~(mask)) | ((setting) & (mask)))

/******************************************************************************/
/*                                                                  SCB CPACR */

/* Note: Added by Maxim Integrated, as these are missing from CMSIS/Core/Include/core_cm4.h */
#define SCB_CPACR_CP10_Pos 20 /*!< SCB CPACR: Coprocessor 10 Position */
#define SCB_CPACR_CP10_Msk (0x3UL << SCB_CPACR_CP10_Pos) /*!< SCB CPACR: Coprocessor 10 Mask */
#define SCB_CPACR_CP11_Pos 22 /*!< SCB CPACR: Coprocessor 11 Position */
#define SCB_CPACR_CP11_Msk (0x3UL << SCB_CPACR_CP11_Pos) /*!< SCB CPACR: Coprocessor 11 Mask */

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32665_INCLUDE_MAX32665_H_
