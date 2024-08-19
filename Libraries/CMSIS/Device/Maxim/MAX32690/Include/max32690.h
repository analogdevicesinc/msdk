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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_MAX32690_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_MAX32690_H_

#ifndef TARGET_NUM
#define TARGET_NUM 32690
#endif

#define MXC_NUMCORES 1

#include <stdint.h>

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

typedef enum {
#ifndef __riscv // not RISC-V
    NonMaskableInt_IRQn = -14,
    HardFault_IRQn = -13,
    MemoryManagement_IRQn = -12,
    BusFault_IRQn = -11,
    UsageFault_IRQn = -10,
    SVCall_IRQn = -5,
    DebugMonitor_IRQn = -4,
    PendSV_IRQn = -2,
    SysTick_IRQn = -1,

    /* Device-specific interrupt sources (external to ARM core)                    */
    /*                         table entry number                                  */
    /*                         ||||                                                */
    /*                         ||||  table offset address                          */
    /*                         vvvv  vvvvvv                                        */

    PF_IRQn = 0, /* 0x10  0x0040  16: Power Fail */
    WDT0_IRQn, /* 0x11  0x0044  17: Watchdog 0 */
    USB_IRQn, /* 0x12  0x0048  18: USB */
    RTC_IRQn, /* 0x13  0x004C  19: RTC */
    TRNG_IRQn, /* 0x14  0x0050  20: True Random Number Generator */
    TMR0_IRQn, /* 0x15  0x0054  21: Timer 0 */
    TMR1_IRQn, /* 0x16  0x0058  22: Timer 1 */
    TMR2_IRQn, /* 0x17  0x005C  23: Timer 2 */
    TMR3_IRQn, /* 0x18  0x0060  24: Timer 3 */
    TMR4_IRQn, /* 0x19  0x0064  25: Timer 4 (LP) */
    TMR5_IRQn, /* 0x1A  0x0068  26: Timer 5 (LP) */
    RSV11_IRQn, /* 0x1B  0x006C  27: Reserved */
    RSV12_IRQn, /* 0x1C  0x0070  28: Reserved */
    I2C0_IRQn, /* 0x1D  0x0074  29: I2C0 */
    UART0_IRQn, /* 0x1E  0x0078  30: UART 0 */
    UART1_IRQn, /* 0x1F  0x007C  31: UART 1 */
    SPI0_IRQn, /* 0x20  0x0080  32: SPI0 */
    SPI1_IRQn, /* 0x21  0x0084  33: SPI1 */
    SPI2_IRQn, /* 0x22  0x0088  34: SPI2 */
    RSV19_IRQn, /* 0x23  0x008C  35: Reserved */
    ADC_IRQn, /* 0x24  0x0090  36: ADC */
    RSV21_IRQn, /* 0x25  0x0094  37: Reserved */
    RSV22_IRQn, /* 0x26  0x0098  38: Reserved */
    FLC0_IRQn, /* 0x27  0x009C  39: Flash Controller 0 */
    GPIO0_IRQn, /* 0x28  0x00A0  40: GPIO0 */
    GPIO1_IRQn, /* 0x29  0x00A4  41: GPIO1 */
    GPIO2_IRQn, /* 0x2A  0x00A8  42: GPIO2 */
    CRYPTO_IRQn, /* 0x2B  0x00AC  43: Crypto */
    DMA0_IRQn, /* 0x2C  0x00B0  44: DMA0 */
    DMA1_IRQn, /* 0x2D  0x00B4  45: DMA1 */
    DMA2_IRQn, /* 0x2E  0x00B8  46: DMA2 */
    DMA3_IRQn, /* 0x2F  0x00BC  47: DMA3 */
    RSV32_IRQn, /* 0x30  0x00C0  48: Reserved */
    RSV33_IRQn, /* 0x31  0x00C4  49: Reserved */
    UART2_IRQn, /* 0x32  0x00C8  50: UART 2 */
    RSV35_IRQn, /* 0x33  0x00CC  51: Reserved */
    I2C1_IRQn, /* 0x34  0x00D0  52: I2C1 */
    RSV37_IRQn, /* 0x35  0x00D4  53: Reserved */
    SPIXC_IRQn, /* 0x36  0x00D8  54: SPI execute in place */
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
    WUT0_IRQn, /* 0x45  0x0114  69: Wakeup Timer 0 */
    GPIOWAKE_IRQn, /* 0x46  0x0118  70: GPIO and AIN Wakeup */
    RSV55_IRQn, /* 0x47  0x011C  71: Reserved */
    SPI3_IRQn, /* 0x48  0x0120  72: SPI3 */
    WDT1_IRQn, /* 0x49  0x0124  73: LP Watchdog */
    GPIO3_IRQn, /* 0x4A  0x0128  74: GPIO3 */
    PT_IRQn, /* 0x4B  0x012C  75: Pulse Train */
    RSV60_IRQn, /* 0x4C  0x0130  76: Reserved */
    HPB_IRQn, /* 0x4D  0x0134  77: Hyperbus */
    I2C2_IRQn, /* 0x4E  0x0138  78: I2C2 */
    RISCV_IRQn, /* 0x4F  0x013C  79: RISC-V */
    RSV64_IRQn, /* 0x50  0x0140  80: Reserved */
    RSV65_IRQn, /* 0x51  0x0144  81: Reserved */
    RSV66_IRQn, /* 0x52  0x0148  82: Reserved */
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
    RSV81_IRQn, /* 0x61  0x0184  97: Reserved */
    ECC_IRQn, /* 0x62  0x0188  98: ECC */
    RSV83_IRQn, /* 0x63  0x018C  99: Reserved */
    RSV84_IRQn, /* 0x64  0x0190 100: Resevred */
    SCA_IRQn, /* 0x65  0x0194 101: SCA Crypto Accelerator */
    RSV86_IRQn, /* 0x66  0x0198 102: Reserved */
    FLC1_IRQn, /* 0x67  0x019C 103: Flash Controller 1 */
    UART3_IRQn, /* 0x68  0x01A0 104: UART 3 (LP) */
    RSV89_IRQn, /* 0x69  0x01A4 105: Reserved */
    RSV90_IRQn, /* 0x6A  0x01A8 106: Reserved */
    RSV91_IRQn, /* 0x6B  0x01AC 107: Reserved */
    RSV92_IRQn, /* 0x6C  0x01B0 108: Reserved */
    RSV93_IRQn, /* 0x6D  0x01B4 109: Reserved */
    RSV94_IRQn, /* 0x6E  0x01B8 110: Reserved */
    RSV95_IRQn, /* 0x6F  0x01BC 111: Reserved */
    RSV96_IRQn, /* 0x70  0x01C0 112: Reserved */
    RSV97_IRQn, /* 0x71  0x01C4 113: Reserved */
    RSV98_IRQn, /* 0x72  0x01C8 114: Reserved */
    I2S_IRQn, /* 0x73  0x01CC 115: I2S */
    RSV100_IRQn, /* 0x74  0x01D0 116: Reserved */
    RSV101_IRQn, /* 0x75  0x01D4 117: Reserved */
    RSV102_IRQn, /* 0x76  0x01D8 118: Reserved */
    LPCMP_IRQn, /* 0x77  0x01Dc 119: LP Comparator */
    RSV104_IRQn, /* 0x78  0x01E0  120: Reserved */
    SPI4_IRQn, /* 0x79  0x01E4  121: SPI4 */
    RSV106_IRQn, /* 0x7A  0x01E8  122: Reserved */
    CAN0_IRQn, /* 0x7B  0x01EC  123: CAN0 */
    CAN1_IRQn, /* 0x7C  0x01F0  124: CAN1 */
    WUT1_IRQn, /* 0x7D  0x01F4  125: Wake up timer 1 */
    RSV110_IRQn, /* 0x7E  0x01F8  126: Reserved */
    RSV111_IRQn, /* 0x7F  0x01FC  127: Reserved */
#else // __riscv
    HardFault_IRQn = 3, /* 0x03,3 HardFault */
    PF_IRQn = 4, /* 0x04,4 PFW | SYSFAULT | CM4 */
    WDT0_IRQn, /* 0x05,5 Watchdog 0 */
    GPIOWAKE_IRQn = 6, /* 0x06,6 GPIO Wakeup */
    AINComp_IRQn = 6, /* 0x06,6 Analog In Comparator */
    RTC_IRQn, /* 0x07,7 RTC */
    TMR0_IRQn, /* 0x08,8 Timer 0 */
    TMR1_IRQn, /* 0x09,9 Timer 1 */
    TMR2_IRQn, /* 0x0A,10 Timer 2 */
    TMR3_IRQn, /* 0x0B,11 Timer 3 */
    TMR4_IRQn, /* 0x0C,12 Timer 4 (LP) */
    TMR5_IRQn, /* 0x0D,13 Timer 5 (LP) */
    I2C0_IRQn, /* 0x0E,14 I2C0 */
    UART0_IRQn, /* 0x0F,15 UART 0 */
    CM4_IRQn, /* 0x10,16 CM4 */
    I2C1_IRQn, /* 0x11,17 I2C1 */
    UART1_IRQn, /* 0x12,18 UART 1 */
    UART2_IRQn, /* 0x13,19 UART 2 */
    I2C2_IRQn, /* 0x14,20 I2C2 */
    UART3_IRQn, /* 0x15,21 LPUART */
    SPI0_IRQn, /* 0x16,22 SPI0 */
    WUT0_IRQn, /* 0x17,23 WUT0 */
    FLC1_IRQn, /* 0x18,24 Flash Controller 1 */
    GPIO0_IRQn, /* 0x19,25 GPIO0 */
    GPIO1_IRQn, /* 0x1A,26 GPIO1 */
    GPIO3_IRQn, /* 0x1B,27 GPIO3 (LP) */
    DMA0_IRQn, /* 0x1C,28 DMA0 */
    DMA1_IRQn, /* 0x1D,29 DMA1 */
    DMA2_IRQn, /* 0x1E,30 DMA2 */
    DMA3_IRQn, /* 0x1F,31 DMA3 */
    BTLE_TX_DONE_IRQn, /* 0x20,32 Reserved */
    BTLE_RX_RCVD_IRQn, /* 0x21,33 Reserved */
    BTLE_RX_ENG_DET_IRQn, /* 0x22,34 Reserved */
    BTLE_SFD_DET_IRQn, /* 0x23,35 Reserved */
    BTLE_SFD_TO_IRQn, /* 0x24,36 Reserved */
    BTLE_GP_EVENT_IRQn, /* 0x25,37 Reserved */
    BTLE_CFO_IRQn, /* 0x26,38 Reserved */
    BTLE_SIG_DET_IRQn, /* 0x27,39 Reserved */
    BTLE_AGC_EVENT_IRQn, /* 0x28,40 Reserved */
    BTLE_RFFE_SPIM_IRQn, /* 0x29,41 Reserved */
    BTLE_TX_AES_IRQn, /* 0x2A,42 Reserved */
    BTLE_RX_AES_IRQn, /* 0x2B,43 Reserved */
    BTLE_INV_APB_ADDR_IRQn, /* 0x2C,44 Reserved */
    BTLE_IQ_DATA_VALID_IRQn, /* 0x2D,45 Reserved */
    DMA4_15_IRQn, /* 0x2E,46 DMA4 - 15  */
    TRNG_IRQn, /* 0x2F,47 True Random Number Generator */
    WDT1_IRQn, /* 0x30,48 Watchdog 1 (LP) */
    RSV49_IRQn, /* 0x31,49 Reserved */
    RSV50_IRQn, /* 0x32,50 Reserved */
    WUT1_IRQn, /* 0x33,51 WUT1  */
    PT_IRQn, /* 0x34,52 Pulse train */
    ADC_IRQn, /* 0x35,53 ADC */
    OWM_IRQn, /* 0x36,54 One Wire Master */
    I2S_IRQn, /* 0x37,55 I2S */
    RSV56_IRQn, /* 0x38,56 Reserved */
    RSV57_IRQn, /* 0x39,57 Reserved  */
    CAN0_IRQn, /* 0x3A,58 CAN0  */
    RSV59_IRQn, /* 0x3B,59 Reserved */
    GPIO2_IRQn, /* 0x3C,60 GPIO2 */
    SPI1_IRQn, /* 0x3D,61 SPI1  */
    SPI2_IRQn, /* 0x3E,62 SPI2  */
    CAN1_IRQn, /* 0x3F,63 CAN1 */
#endif // __riscv
    MXC_IRQ_EXT_COUNT,
} IRQn_Type;

#define MXC_IRQ_COUNT (MXC_IRQ_EXT_COUNT + 16)

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

#ifndef __riscv
/* ----------------------  Configuration of the Cortex-M Processor and Core Peripherals  ---------------------- */
#define __CM4_REV 0x0100 /*!< Cortex-M4 Core Revision                                */
#define __MPU_PRESENT 1 /*!< MPU present or not                                     */
#define __NVIC_PRIO_BITS 3 /*!< Number of Bits used for Priority Levels                */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used           */
#define __FPU_PRESENT 1 /*!< FPU present or not                                     */

#include <core_cm4.h> /*!< Cortex-M4 processor and core peripherals               */

#else // __riscv

#include <core_rv32.h>

#endif // __riscv

#include "system_max32690.h" /*!< System Header                                          */

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00020000UL
#define MXC_XIP_MEM_BASE 0x08000000UL
#define MXC_XIP_MEM_SIZE 0x08000000UL
#define MXC_FLASH0_MEM_BASE 0x10000000UL
#define MXC_FLASH1_MEM_BASE 0x10300000UL
#define MXC_FLASH_MEM_BASE MXC_FLASH0_MEM_BASE
#define MXC_FLASH_PAGE_SIZE 0x00004000UL
#define MXC_FLASH0_PAGE_SIZE 0x00004000UL
#define MXC_FLASH1_PAGE_SIZE 0x00002000UL
#define MXC_FLASH0_MEM_SIZE 0x00300000UL
#define MXC_FLASH1_MEM_SIZE 0x00040000UL
#define MXC_FLASH_MEM_SIZE (MXC_FLASH0_MEM_SIZE + MXC_FLASH1_MEM_SIZE)
#define MXC_INFO0_MEM_BASE 0x10800000UL
#define MXC_INFO1_MEM_BASE 0x10802000UL
#define MXC_INFO_MEM_BASE MXC_INFO0_MEM_BASE
#define MXC_INFO_MEM_SIZE 0x00002000UL
#define MXC_INFO0_MEM_SIZE 0x00002000UL
#define MXC_INFO1_MEM_SIZE 0x00002000UL
#define MXC_SRAM_MEM_BASE 0x20000000UL
#define MXC_SRAM_MEM_SIZE 0x00120000UL
#define MXC_HPB_MEM_BASE 0x60000000UL
#define MXC_HPB_MEM_SIZE 0x20000000UL
#define MXC_XIP_DATA_MEM_BASE 0x80000000UL
#define MXC_XIP_DATA_MEM_SIZE 0x20000000UL

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX32690 peripheral modules.
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
/*                                        Non-Battery Backed Function Control */
#define MXC_BASE_FCR ((uint32_t)0x40000800UL)
#define MXC_FCR ((mxc_fcr_regs_t *)MXC_BASE_FCR)

/******************************************************************************/
/*                                                                        CTB */
#define MXC_BASE_CTB ((uint32_t)0x40001000UL)
#define MXC_CTB ((mxc_ctb_regs_t *)MXC_BASE_CTB)

/******************************************************************************/
/*                                                    Windowed Watchdog Timer */
#define MXC_CFG_WDT_INSTANCES (2)

#define MXC_BASE_WDT0 ((uint32_t)0x40003000UL)
#define MXC_WDT0 ((mxc_wdt_regs_t *)MXC_BASE_WDT0)
#define MXC_WDT MXC_WDT0
#define MXC_BASE_WDT1 ((uint32_t)0x40080800UL)
#define MXC_WDT1 ((mxc_wdt_regs_t *)MXC_BASE_WDT1)

#define MXC_WDT_GET_IDX(p) ((p) == MXC_WDT0 ? 0 : (p) == MXC_WDT1 ? 1 : -1)

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
/*                                                                       GCFR */
#define MXC_BASE_GCFR ((uint32_t)0x40005800UL)
#define MXC_GCFR ((mxc_gcfr_regs_t *)MXC_BASE_GCFR)

/******************************************************************************/
/*                                                            Real Time Clock */
#define MXC_BASE_RTC ((uint32_t)0x40006000UL)
#define MXC_RTC ((mxc_rtc_regs_t *)MXC_BASE_RTC)

/******************************************************************************/
/*                                                        Wake-Up Timer (WUT) */
#define MXC_CFG_WUT_INSTANCES (2)

#define MXC_BASE_WUT0 ((uint32_t)0x40006400UL)
#define MXC_WUT0 ((mxc_wut_regs_t *)MXC_BASE_WUT0)
#define MXC_WUT MXC_WUT0
#define MXC_BASE_WUT1 ((uint32_t)0x40006600UL)
#define MXC_WUT1 ((mxc_wut_regs_t *)MXC_BASE_WUT1)

/******************************************************************************/
/*                                                            Power Sequencer */
#define MXC_BASE_PWRSEQ ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ)

/******************************************************************************/
/*                                                              Misc Control  */
#define MXC_BASE_MCR ((uint32_t)0x40006C00UL)
#define MXC_MCR ((mxc_mcr_regs_t *)MXC_BASE_MCR)

/******************************************************************************/
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (5)
#define MXC_CFG_GPIO_PINS_PORT (32)

#define MXC_BASE_GPIO0 ((uint32_t)0x40008000UL)
#define MXC_GPIO0 ((mxc_gpio_regs_t *)MXC_BASE_GPIO0)
#define MXC_BASE_GPIO1 ((uint32_t)0x40009000UL)
#define MXC_GPIO1 ((mxc_gpio_regs_t *)MXC_BASE_GPIO1)
#define MXC_BASE_GPIO2 ((uint32_t)0x4000A000UL)
#define MXC_GPIO2 ((mxc_gpio_regs_t *)MXC_BASE_GPIO2)
#define MXC_BASE_GPIO3 ((uint32_t)0x40080400UL)
#define MXC_GPIO3 ((mxc_gpio_regs_t *)MXC_BASE_GPIO3)
//GPIO4 dummy address it does not live here and will be handled in code different than other gpios but this allow our macros to work.
#define MXC_BASE_GPIO4 ((uint32_t)0x4000C000UL)
#define MXC_GPIO4 ((mxc_gpio_regs_t *)MXC_BASE_GPIO4)

#define MXC_GPIO_GET_IDX(p) \
    ((p) == MXC_GPIO0 ? 0 : \
     (p) == MXC_GPIO1 ? 1 : \
     (p) == MXC_GPIO2 ? 2 : \
     (p) == MXC_GPIO3 ? 3 : \
     (p) == MXC_GPIO4 ? 4 : \
                        -1)

#define MXC_GPIO_GET_GPIO(i) \
    ((i) == 0 ? MXC_GPIO0 :  \
     (i) == 1 ? MXC_GPIO1 :  \
     (i) == 2 ? MXC_GPIO2 :  \
     (i) == 3 ? MXC_GPIO3 :  \
     (i) == 4 ? MXC_GPIO4 :  \
                0)

#define MXC_GPIO_GET_IRQ(i)     \
    ((i) == 0 ? GPIO0_IRQn :    \
     (i) == 1 ? GPIO1_IRQn :    \
     (i) == 2 ? GPIO2_IRQn :    \
     (i) == 3 ? GPIO3_IRQn :    \
     (i) == 4 ? GPIOWAKE_IRQn : \
                0)

/******************************************************************************/
/*                                                                      Timer */
#define SEC(s) (((uint32_t)s) * 1000000UL)
#define MSEC(ms) (ms * 1000UL)
#define USEC(us) (us)

#define MXC_CFG_TMR_INSTANCES (6)

#define MXC_BASE_TMR0 ((uint32_t)0x40010000UL)
#define MXC_TMR0 ((mxc_tmr_regs_t *)MXC_BASE_TMR0)
#define MXC_BASE_TMR1 ((uint32_t)0x40011000UL)
#define MXC_TMR1 ((mxc_tmr_regs_t *)MXC_BASE_TMR1)
#define MXC_BASE_TMR2 ((uint32_t)0x40012000UL)
#define MXC_TMR2 ((mxc_tmr_regs_t *)MXC_BASE_TMR2)
#define MXC_BASE_TMR3 ((uint32_t)0x40013000UL)
#define MXC_TMR3 ((mxc_tmr_regs_t *)MXC_BASE_TMR3)
#define MXC_BASE_TMR4 ((uint32_t)0x40080C00UL)
#define MXC_TMR4 ((mxc_tmr_regs_t *)MXC_BASE_TMR4)
#define MXC_BASE_TMR5 ((uint32_t)0x40081000UL)
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
/*                                                                        I2C */
#define MXC_I2C_INSTANCES (3)

#define MXC_BASE_I2C0 ((uint32_t)0x4001D000UL)
#define MXC_I2C0 ((mxc_i2c_regs_t *)MXC_BASE_I2C0)
#define MXC_BASE_I2C1 ((uint32_t)0x4001E000UL)
#define MXC_I2C1 ((mxc_i2c_regs_t *)MXC_BASE_I2C1)
#define MXC_BASE_I2C2 ((uint32_t)0x4001F000UL)
#define MXC_I2C2 ((mxc_i2c_regs_t *)MXC_BASE_I2C2)

#define MXC_I2C_GET_IRQ(i) \
    (IRQn_Type)((i) == 0 ? I2C0_IRQn : (i) == 1 ? I2C1_IRQn : (i) == 2 ? I2C2_IRQn : 0)

#define MXC_I2C_GET_BASE(i) \
    ((i) == 0 ? MXC_BASE_I2C0 : (i) == 1 ? MXC_BASE_I2C1 : (i) == 2 ? MXC_BASE_I2C2 : 0)

#define MXC_I2C_GET_I2C(i) ((i) == 0 ? MXC_I2C0 : (i) == 1 ? MXC_I2C1 : (i) == 2 ? MXC_I2C2 : 0)

#define MXC_I2C_GET_IDX(p) ((p) == MXC_I2C0 ? 0 : (p) == MXC_I2C1 ? 1 : (p) == MXC_I2C2 ? 2 : -1)

#define MXC_I2C_FIFO_DEPTH (8)

/* ************************************************************************** */
/*                                                SPI Execute in Place Master */
#define MXC_BASE_SPIXFM ((uint32_t)0x40026000UL)
#define MXC_SPIXFM ((mxc_spixfm_regs_t *)MXC_BASE_SPIXFM)

/* ************************************************************************** */
/*                                     SPI Execute in Place Master Controller */
#define MXC_CFG_SPIXFC_FIFO_DEPTH (16)

#define MXC_BASE_SPIXFC ((uint32_t)0x40027000UL)
#define MXC_SPIXFC ((mxc_spixfc_regs_t *)MXC_BASE_SPIXFC)
#define MXC_BASE_SPIXFC_FIFO ((uint32_t)0x400BC000UL)
#define MXC_SPIXFC_FIFO ((mxc_spixfc_fifo_regs_t *)MXC_BASE_SPIXFC_FIFO)

/******************************************************************************/
/*                                                                        DMA */
#define MXC_DMA_CHANNELS (16)
#define MXC_DMA_INSTANCES (1)

#define MXC_BASE_DMA ((uint32_t)0x40028000UL)
#define MXC_DMA ((mxc_dma_regs_t *)MXC_BASE_DMA)

#define MXC_DMA_GET_IDX(p) ((p) == MXC_DMA ? 0 : -1)

#define MXC_DMA_CH_GET_IRQ(i)               \
    ((IRQn_Type)(((i) == 0)  ? DMA0_IRQn :  \
                 ((i) == 1)  ? DMA1_IRQn :  \
                 ((i) == 2)  ? DMA2_IRQn :  \
                 ((i) == 3)  ? DMA3_IRQn :  \
                 ((i) == 4)  ? DMA4_IRQn :  \
                 ((i) == 5)  ? DMA5_IRQn :  \
                 ((i) == 6)  ? DMA6_IRQn :  \
                 ((i) == 7)  ? DMA7_IRQn :  \
                 ((i) == 8)  ? DMA8_IRQn :  \
                 ((i) == 9)  ? DMA9_IRQn :  \
                 ((i) == 10) ? DMA10_IRQn : \
                 ((i) == 11) ? DMA11_IRQn : \
                 ((i) == 12) ? DMA12_IRQn : \
                 ((i) == 13) ? DMA13_IRQn : \
                 ((i) == 14) ? DMA14_IRQn : \
                 ((i) == 15) ? DMA15_IRQn : \
                               0))

/******************************************************************************/
/*                                                                        FLC */
#define MXC_FLC_INSTANCES (2)

#define MXC_BASE_FLC0 ((uint32_t)0x40029000UL)
#define MXC_FLC0 ((mxc_flc_regs_t *)MXC_BASE_FLC0)
#define MXC_FLC MXC_FLC0
#define MXC_BASE_FLC1 ((uint32_t)0x40029400UL)
#define MXC_FLC1 ((mxc_flc_regs_t *)MXC_BASE_FLC1)

#define MXC_FLC_GET_IRQ(i) (IRQn_Type)((i) == 0 ? FLC0_IRQn : (i) == 1 ? FLC1_IRQn : 0)

#define MXC_FLC_GET_BASE(i) ((i) == 0 ? MXC_BASE_FLC0 : (i) == 1 ? MXC_BASE_FLC1 : 0)

#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC0 : (i) == 1 ? MXC_FLC1 : 0)

#define MXC_FLC_GET_IDX(p) ((p) == MXC_FLC0 ? 0 : (p) == MXC_FLC1 ? 1 : -1)

/******************************************************************************/
/*                                                  Internal Cache Controller */
#define MXC_ICC_INSTANCES (2)

#define MXC_BASE_ICC0 ((uint32_t)0x4002A000UL)
#define MXC_ICC0 ((mxc_icc_regs_t *)MXC_BASE_ICC0)

#define MXC_BASE_ICC1 ((uint32_t)0x4002A800UL)
#define MXC_ICC1 ((mxc_icc_regs_t *)MXC_BASE_ICC1)

#define MXC_ICC MXC_ICC0
// ICC1 is the RISC-V cache

/******************************************************************************/
/*                                              Internal Cache XIP Controller */
#define MXC_BASE_SFCC ((uint32_t)0x4002F000UL)
#define MXC_SFCC ((mxc_icc_regs_t *)MXC_BASE_SFCC)

/******************************************************************************/
/*                                           External Memory Cache Controller */
#define MXC_BASE_EMCC ((uint32_t)0x40033000UL)
#define MXC_EMCC ((mxc_emcc_regs_t *)MXC_BASE_EMCC)

/******************************************************************************/
/*                                                                        ADC */
#define MXC_BASE_ADC ((uint32_t)0x40034000UL)
#define MXC_ADC ((mxc_adc_regs_t *)MXC_BASE_ADC)
#define MXC_ADC_MAX_CLOCK 8000000 // Maximum ADC clock in Hz

/******************************************************************************/
/*                                                                   HyperBus */
#define MXC_BASE_HPB ((uint32_t)0x40039000UL)
#define MXC_HPB ((mxc_hpb_regs_t *)MXC_BASE_HPB)

/* ************************************************************************** */
/*                                SPI Execute in Place Data Master Controller */
#define MXC_BASE_SPIXR ((uint32_t)0x4003A000UL)
#define MXC_SPIXR ((mxc_spixr_regs_t *)MXC_BASE_SPIXR)

/*******************************************************************************/
/*                                                      Pulse Train Generation */
#define MXC_CFG_PT_INSTANCES (16)

#define MXC_BASE_PTG ((uint32_t)0x4003C000UL)
#define MXC_PTG ((mxc_ptg_regs_t *)MXC_BASE_PTG)
#define MXC_BASE_PT0 ((uint32_t)0x4003C020UL)
#define MXC_PT0 ((mxc_pt_regs_t *)MXC_BASE_PT0)
#define MXC_BASE_PT1 ((uint32_t)0x4003C040UL)
#define MXC_PT1 ((mxc_pt_regs_t *)MXC_BASE_PT1)
#define MXC_BASE_PT2 ((uint32_t)0x4003C060UL)
#define MXC_PT2 ((mxc_pt_regs_t *)MXC_BASE_PT2)
#define MXC_BASE_PT3 ((uint32_t)0x4003C080UL)
#define MXC_PT3 ((mxc_pt_regs_t *)MXC_BASE_PT3)
#define MXC_BASE_PT4 ((uint32_t)0x4003C0A0UL)
#define MXC_PT4 ((mxc_pt_regs_t *)MXC_BASE_PT4)
#define MXC_BASE_PT5 ((uint32_t)0x4003C0C0UL)
#define MXC_PT5 ((mxc_pt_regs_t *)MXC_BASE_PT5)
#define MXC_BASE_PT6 ((uint32_t)0x4003C0E0UL)
#define MXC_PT6 ((mxc_pt_regs_t *)MXC_BASE_PT6)
#define MXC_BASE_PT7 ((uint32_t)0x4003C100UL)
#define MXC_PT7 ((mxc_pt_regs_t *)MXC_BASE_PT7)
#define MXC_BASE_PT8 ((uint32_t)0x4003C120UL)
#define MXC_PT8 ((mxc_pt_regs_t *)MXC_BASE_PT8)
#define MXC_BASE_PT9 ((uint32_t)0x4003C140UL)
#define MXC_PT9 ((mxc_pt_regs_t *)MXC_BASE_PT9)
#define MXC_BASE_PT10 ((uint32_t)0x4003C160UL)
#define MXC_PT10 ((mxc_pt_regs_t *)MXC_BASE_PT10)
#define MXC_BASE_PT11 ((uint32_t)0x4003C180UL)
#define MXC_PT11 ((mxc_pt_regs_t *)MXC_BASE_PT11)
#define MXC_BASE_PT12 ((uint32_t)0x4003C1A0UL)
#define MXC_PT12 ((mxc_pt_regs_t *)MXC_BASE_PT12)
#define MXC_BASE_PT13 ((uint32_t)0x4003C1C0UL)
#define MXC_PT13 ((mxc_pt_regs_t *)MXC_BASE_PT13)
#define MXC_BASE_PT14 ((uint32_t)0x4003C1E0UL)
#define MXC_PT14 ((mxc_pt_regs_t *)MXC_BASE_PT14)
#define MXC_BASE_PT15 ((uint32_t)0x4003C200UL)
#define MXC_PT15 ((mxc_pt_regs_t *)MXC_BASE_PT15)

#define MXC_PT_GET_BASE(i)       \
    ((i) == 0  ? MXC_BASE_PT0 :  \
     (i) == 1  ? MXC_BASE_PT1 :  \
     (i) == 2  ? MXC_BASE_PT2 :  \
     (i) == 3  ? MXC_BASE_PT3 :  \
     (i) == 4  ? MXC_BASE_PT4 :  \
     (i) == 5  ? MXC_BASE_PT5 :  \
     (i) == 6  ? MXC_BASE_PT6 :  \
     (i) == 7  ? MXC_BASE_PT7 :  \
     (i) == 8  ? MXC_BASE_PT8 :  \
     (i) == 9  ? MXC_BASE_PT9 :  \
     (i) == 10 ? MXC_BASE_PT10 : \
     (i) == 11 ? MXC_BASE_PT11 : \
     (i) == 12 ? MXC_BASE_PT12 : \
     (i) == 13 ? MXC_BASE_PT13 : \
     (i) == 14 ? MXC_BASE_PT14 : \
     (i) == 15 ? MXC_BASE_PT15 : \
                 0)

#define MXC_PT_GET_PT(i)    \
    ((i) == 0  ? MXC_PT0 :  \
     (i) == 1  ? MXC_PT1 :  \
     (i) == 2  ? MXC_PT2 :  \
     (i) == 3  ? MXC_PT3 :  \
     (i) == 4  ? MXC_PT4 :  \
     (i) == 5  ? MXC_PT5 :  \
     (i) == 6  ? MXC_PT6 :  \
     (i) == 7  ? MXC_PT7 :  \
     (i) == 8  ? MXC_PT8 :  \
     (i) == 9  ? MXC_PT9 :  \
     (i) == 10 ? MXC_PT10 : \
     (i) == 11 ? MXC_PT11 : \
     (i) == 12 ? MXC_PT12 : \
     (i) == 13 ? MXC_PT13 : \
     (i) == 14 ? MXC_PT14 : \
     (i) == 15 ? MXC_PT15 : \
                 0)

#define MXC_PT_GET_IDX(p)   \
    ((p) == MXC_PT0  ? 0 :  \
     (p) == MXC_PT1  ? 1 :  \
     (p) == MXC_PT2  ? 2 :  \
     (p) == MXC_PT3  ? 3 :  \
     (p) == MXC_PT4  ? 4 :  \
     (p) == MXC_PT5  ? 5 :  \
     (p) == MXC_PT6  ? 6 :  \
     (p) == MXC_PT7  ? 7 :  \
     (p) == MXC_PT8  ? 8 :  \
     (p) == MXC_PT9  ? 9 :  \
     (p) == MXC_PT10 ? 10 : \
     (p) == MXC_PT11 ? 11 : \
     (p) == MXC_PT12 ? 12 : \
     (p) == MXC_PT13 ? 13 : \
     (p) == MXC_PT14 ? 14 : \
     (p) == MXC_PT15 ? 15 : \
                       -1)

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
#define MXC_UART_INSTANCES (4)
#define MXC_UART_FIFO_DEPTH (8)

#define MXC_BASE_UART0 ((uint32_t)0x40042000UL)
#define MXC_UART0 ((mxc_uart_regs_t *)MXC_BASE_UART0)
#define MXC_BASE_UART1 ((uint32_t)0x40043000UL)
#define MXC_UART1 ((mxc_uart_regs_t *)MXC_BASE_UART1)
#define MXC_BASE_UART2 ((uint32_t)0x40044000UL)
#define MXC_UART2 ((mxc_uart_regs_t *)MXC_BASE_UART2)
#define MXC_BASE_UART3 ((uint32_t)0x40081400UL)
#define MXC_UART3 ((mxc_uart_regs_t *)MXC_BASE_UART3)

#define MXC_UART_GET_IRQ(i)                        \
    (IRQn_Type)((i) == 0            ? UART0_IRQn : \
                (IRQn_Type)(i) == 1 ? UART1_IRQn : \
                (IRQn_Type)(i) == 2 ? UART2_IRQn : \
                (IRQn_Type)(i) == 3 ? UART3_IRQn : \
                                      0)

#define MXC_UART_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_UART0 : \
     (i) == 1 ? MXC_BASE_UART1 : \
     (i) == 2 ? MXC_BASE_UART2 : \
     (i) == 3 ? MXC_BASE_UART3 : \
                0)

#define MXC_UART_GET_UART(i) \
    ((i) == 0 ? MXC_UART0 : (i) == 1 ? MXC_UART1 : (i) == 2 ? MXC_UART2 : (i) == 3 ? MXC_UART3 : 0)

#define MXC_UART_GET_IDX(p) \
    ((p) == MXC_UART0 ? 0 : (p) == MXC_UART1 ? 1 : (p) == MXC_UART2 ? 2 : (p) == MXC_UART3 ? 3 : -1)

/******************************************************************************/
/*                                                                        SPI */
#ifndef __riscv
#define MXC_SPI_INSTANCES (5)
#else
#define MXC_SPI_INSTANCES (3)
#endif // __riscv
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

#define MXC_BASE_SPI0 ((uint32_t)0x40046000UL)
#define MXC_SPI0 ((mxc_spi_regs_t *)MXC_BASE_SPI0)
#define MXC_BASE_SPI1 ((uint32_t)0x40047000UL)
#define MXC_SPI1 ((mxc_spi_regs_t *)MXC_BASE_SPI1)
#define MXC_BASE_SPI2 ((uint32_t)0x40048000UL)
#define MXC_SPI2 ((mxc_spi_regs_t *)MXC_BASE_SPI2)

#ifndef __riscv
#define MXC_BASE_SPI3 ((uint32_t)0x400BE000UL)
#define MXC_SPI3 ((mxc_spi_regs_t *)MXC_BASE_SPI3)
#define MXC_BASE_SPI4 ((uint32_t)0x400BE400UL)
#define MXC_SPI4 ((mxc_spi_regs_t *)MXC_BASE_SPI4)

#define MXC_SPI_GET_IDX(p) \
    ((p) == MXC_SPI0 ? 0 : \
     (p) == MXC_SPI1 ? 1 : \
     (p) == MXC_SPI2 ? 2 : \
     (p) == MXC_SPI3 ? 3 : \
     (p) == MXC_SPI4 ? 4 : \
                       -1)

#define MXC_SPI_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_SPI0 : \
     (i) == 1 ? MXC_BASE_SPI1 : \
     (i) == 2 ? MXC_BASE_SPI2 : \
     (i) == 3 ? MXC_BASE_SPI3 : \
     (i) == 4 ? MXC_BASE_SPI4 : \
                0)

#define MXC_SPI_GET_SPI(i) \
    ((i) == 0 ? MXC_SPI0 : \
     (i) == 1 ? MXC_SPI1 : \
     (i) == 2 ? MXC_SPI2 : \
     (i) == 3 ? MXC_SPI3 : \
     (i) == 4 ? MXC_SPI4 : \
                0)

#define MXC_SPI_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? SPI0_IRQn : \
                (i) == 1 ? SPI1_IRQn : \
                (i) == 2 ? SPI2_IRQn : \
                (i) == 3 ? SPI3_IRQn : \
                (i) == 4 ? SPI4_IRQn : \
                           0)
#else // __riscv

#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI0 ? 0 : (p) == MXC_SPI1 ? 1 : (p) == MXC_SPI2 ? 2 : -1)

#define MXC_SPI_GET_BASE(i) \
    ((i) == 0 ? MXC_BASE_SPI0 : (i) == 1 ? MXC_BASE_SPI1 : (i) == 2 ? MXC_BASE_SPI2 : 0)

#define MXC_SPI_GET_SPI(i)              ((i) == 0 ? MXC_SPI0                   \
                                            (i) == 1 ? MXC_SPI1                \
                                            (i) == 2 ? MXC_SPI2 : 0)

#define MXC_SPI_GET_IRQ(i) \
    (IRQn_Type)((i) == 0 ? SPI0_IRQn : (i) == 1 ? SPI1_IRQn : (i) == 2 ? SPI2_IRQn : 0)

#endif // __riscv

/******************************************************************************/
/*                                                                       TRNG */
#define MXC_BASE_TRNG ((uint32_t)0x4004D000UL)
#define MXC_TRNG ((mxc_trng_regs_t *)MXC_BASE_TRNG)

/******************************************************************************/
/*                                                                        I2S */
#define MXC_BASE_I2S ((uint32_t)0x40060000UL)
#define MXC_I2S ((mxc_i2s_regs_t *)MXC_BASE_I2S)

/******************************************************************************/
/*                                                                        USB */
#define MXC_BASE_USBHS ((uint32_t)0x400B1000UL)
#define MXC_USBHS ((mxc_usbhs_regs_t *)MXC_BASE_USBHS)
#define MXC_USBHS_NUM_EP 12 /* HW must have at least EP 0 CONTROL + 11 IN/OUT */
#define MXC_USBHS_NUM_DMA 8 /* HW must have at least this many DMA channels */
#define MXC_USBHS_MAX_PACKET 512

/** @brief USB clock source options */
typedef enum {
    MXC_USB_CLOCK_SYS_DIV_10 = 0, ///< SYS_CLK divded by 10
    MXC_USB_CLOCK_EXTCLK = 1, ///< External clock input
    MXC_USB_CLOCK_ERFO = 2 ///< External RF Oscillator input
} _mxc_usb_clock_t;

/**
 * @brief   USB clock source options macro
 * @note    (Developers): "mxc_usb_clock_t" should be defined as a macro in the top-level 
            file here so that the pre-processor can check for its existence when the USB
            library is built.  The macro should pass through to the actual enum
*/
#define mxc_usb_clock_t _mxc_usb_clock_t

/******************************************************************************/
/*                                                  Low Power General control */
#define MXC_BASE_LPGCR ((uint32_t)0x40080000UL)
#define MXC_LPGCR ((mxc_lpgcr_regs_t *)MXC_BASE_LPGCR)

/******************************************************************************/
/*                                                       Low-Power Comparator */
#define MXC_BASE_LPCMP ((uint32_t)0x40088000UL)
#define MXC_LPCMP ((mxc_lpcmp_regs_t *)MXC_BASE_LPCMP)

/******************************************************************************/
/*                                                                        CAN */
#define MXC_CAN_INSTANCES (2)

#define MXC_BASE_CAN0 ((uint32_t)0x40064000UL)
#define MXC_CAN0 ((mxc_can_regs_t *)MXC_BASE_CAN0)
#define MXC_BASE_CAN1 ((uint32_t)0x40065000UL)
#define MXC_CAN1 ((mxc_can_regs_t *)MXC_BASE_CAN1)

#define MXC_CAN_GET_IDX(p) ((p) == MXC_CAN0 ? 0 : (p) == MXC_CAN1 ? 1 : -1)

#define MXC_CAN_GET_BASE(i) ((i) == 0 ? MXC_BASE_CAN0 : (i) == 1 ? MXC_BASE_CAN1 : 0)

#define MXC_CAN_GET_CAN(i) ((i) == 0 ? MXC_CAN0 : (i) == 1 ? MXC_CAN1 : 0)

#define MXC_CAN_GET_IRQ(i) (IRQn_Type)((i) == 0 ? CAN0_IRQn : (i) == 1 ? CAN1_IRQn : 0)

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

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32690_INCLUDE_MAX32690_H_
