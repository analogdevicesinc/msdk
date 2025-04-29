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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MAX78002_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MAX78002_H_

// clang-format off
#ifndef TARGET_NUM
#define TARGET_NUM 78002
#endif

#define MXC_NUMCORES 2

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
    SPI1_IRQn, /* 0x20  0x0080  32: SPI1 */
    RSV17_IRQn, /* 0x21  0x0084  33: Reserved */
    RSV18_IRQn, /* 0x22  0x0088  34: Reserved */
    RSV19_IRQn, /* 0x23  0x008C  35: Reserved */
    ADC_IRQn, /* 0x24  0x0090  36: ADC */
    RSV21_IRQn, /* 0x25  0x0094  37: Reserved */
    RSV22_IRQn, /* 0x26  0x0098  38: Reserved */
    FLC0_IRQn, /* 0x27  0x009C  39: Flash Controller */
    GPIO0_IRQn, /* 0x28  0x00A0  40: GPIO0 */
    GPIO1_IRQn, /* 0x29  0x00A4  41: GPIO1 */
    GPIO2_IRQn, /* 0x2A  0x00A8  42: GPIO2 (LP) */
    RSV27_IRQn, /* 0x2B  0x00AC  43: Reserved */
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
    RSV38_IRQn, /* 0x36  0x00D8  54: Reserved */
    RSV39_IRQn, /* 0x37  0x00DC  55: Reserved */
    RSV40_IRQn, /* 0x38  0x00E0  56: Reserved */
    RSV41_IRQn, /* 0x39  0x00E4  57: Reserved */
    RSV42_IRQn, /* 0x3A  0x00E8  58: Reserved */
    RSV43_IRQn, /* 0x3B  0x00EC  59: Reserved */
    RSV44_IRQn, /* 0x3C  0x00F0  60: Reserved */
    RSV45_IRQn, /* 0x3D  0x00F4  61: Reserved */
    RSV46_IRQn, /* 0x3E  0x00F8  62: Reserved */
    RSV47_IRQn, /* 0x3F  0x00FC  63: Reserved */
    RSV48_IRQn, /* 0x40  0x0100  64: Reserved */
    RSV49_IRQn, /* 0x41  0x0104  65: Reserved */
    RSV50_IRQn, /* 0x42  0x0108  66: Reserved */
    RSV51_IRQn, /* 0x43  0x010C  67: Reserved */
    RSV52_IRQn, /* 0x44  0x0110  68: Reserved */
    WUT_IRQn, /* 0x45  0x0114  69: Wakeup Timer */
    GPIOWAKE_IRQn, /* 0x46  0x0118  70: GPIO and AIN Wakeup */
    RSV55_IRQn, /* 0x47  0x011C  71: Reserved */
    SPI0_IRQn, /* 0x48  0x0120  72: SPI0 */
    WDT1_IRQn, /* 0x49  0x0124  73: LP Watchdog */
    RSV58_IRQn, /* 0x4A  0x0128  74: Reserved */
    PT_IRQn, /* 0x4B  0x012C  75: Pulse Train */
    RSV60_IRQn, /* 0x4C  0x0130  76: Reserved */
    RSV61_IRQn, /* 0x4D  0x0134  77: Reserved */
    I2C2_IRQn, /* 0x4E  0x0138  78: I2C2 */
    RISCV_IRQn, /* 0x4F  0x013C  79: RISC-V */
    RSV64_IRQn, /* 0x50  0x0140  80: Reserved */
    RSV65_IRQn, /* 0x51  0x0144  81: Reserved */
    SDHC_IRQn, /* 0x52  0x0148  82: SDHC SDIO */
    OWM_IRQn, /* 0x53  0x014C  83: One Wire Master */
    RSV68_IRQn, /* 0x54  0x0150  84: Reserved */
    RSV69_IRQn, /* 0x55  0x0154  85: Reserved */
    RSV70_IRQn, /* 0x56  0x0158  86: Reserved */
    RSV71_IRQn, /* 0x57  0x015C  87: Reserved */
    RSV72_IRQn, /* 0x58  0x0160  88: Reserved */
    RSV73_IRQn, /* 0x59  0x0164  89: Reserved */
    RSV74_IRQn, /* 0x5A  0x0168  90: Reserved */
    RSV75_IRQn, /* 0x5B  0x016C  91: Reserved */
    RSV76_IRQn, /* 0x5C  0x0170  92: Reserved */
    RSV77_IRQn, /* 0x5D  0x0174  93: Reserved */
    RSV78_IRQn, /* 0x5E  0x0178  94: Reserved */
    RSV79_IRQn, /* 0x5F  0x017C  95: Reserved */
    RSV80_IRQn, /* 0x60  0x0180  96: Reserved */
    RSV81_IRQn, /* 0x61  0x0184  97: Reserved */
    ECC_IRQn, /* 0x62  0x0188  98: ECC */
    DVS_IRQn, /* 0x63  0x018C  99: DVS */
    SIMO_IRQn, /* 0x64  0x0190 100: SIMO */
    RSV85_IRQn, /* 0x65  0x0194 101: Reserved */
    RSV86_IRQn, /* 0x66  0x0198 102: Reserved */
    RSV87_IRQn, /* 0x67  0x019C 103: Reserved */
    UART3_IRQn, /* 0x68  0x01A0 104: UART 3 (LP) */
    RSV89_IRQn, /* 0x69  0x01A4 105: Reserved */
    RSV90_IRQn, /* 0x6A  0x01A8 106: Reserved */
    PCIF_IRQn, /* 0x6B  0x01AC 107: PCIF (Camera) */
    RSV92_IRQn, /* 0x6C  0x01B0 108: Reserved */
    RSV93_IRQn, /* 0x6D  0x01B4 109: Reserved */
    RSV94_IRQn, /* 0x6E  0x01B8 110: Reserved */
    RSV95_IRQn, /* 0x6F  0x01BC 111: Reserved */
    RSV96_IRQn, /* 0x70  0x01C0 112: Reserved */
    AES_IRQn, /* 0x71  0x01C4 113: AES */
    CRC_IRQn, /* 0x72  0x01C8 114: CRC */
    I2S_IRQn, /* 0x73  0x01CC 115: I2S */
    CNN_FIFO_IRQn, /* 0x74  0x01D0 116: CNN FIFO */
    CNN_IRQn, /* 0x75  0x01D4 117: CNN */
    RSV102_IRQn, /* 0x76  0x01D8 118: Reserved */
    LPCMP_IRQn, /* 0x77  0x01DC 119: LP Comparator */
    CSI2_IRQn, /* 0x78  0x01E0 120: CSI2 APB */
#else // __riscv
    PF_IRQn = 4, /* 0x04,4 PFW | SYSFAULT | CM4 */
    WDT0_IRQn, /* 0x05,5 Watchdog 0 */
    GPIOWAKE_IRQn = 6, /* 0x06,6 GPIO Wakeup */
    AINComp_IRQn = 6, /* 0x06,6 AINComp */
    RTC_IRQn, /* 0x07,7 RTC */
    TMR0_IRQn, /* 0x08,8 Timer 0 */
    TMR1_IRQn, /* 0x09,9 Timer 1 */
    TMR2_IRQn, /* 0x0A,10 Timer 2 */
    TMR3_IRQn, /* 0x0B,11 Timer 3 */
    TMR4_IRQn, /* 0x0C,12 Timer 4 (LP) */
    TMR5_IRQn, /* 0x0D,13 Timer 5 (LP) */
    I2C0_IRQn, /* 0x0E,14 I2C0 */
    UART0_IRQn, /* 0x0F,15 UART 0 */
    RSV16_IRQn, /* 0x10,16 Reserved */
    I2C1_IRQn, /* 0x11,17 I2C1 */
    UART1_IRQn, /* 0x12,18 UART 1 */
    UART2_IRQn, /* 0x13,19 UART 2 */
    I2C2_IRQn, /* 0x14,20 I2C2 */
    UART3_IRQn, /* 0x15,21 LPUART */
    SPI1_IRQn, /* 0x16,22 SPI1 */
    WUT_IRQn, /* 0x17,23 WUT */
    FLC0_IRQn, /* 0x18,24 Flash Controller */
    GPIO0_IRQn, /* 0x19,25 GPIO0 */
    GPIO1_IRQn, /* 0x1A,26 GPIO1 */
    GPIO2_IRQn, /* 0x1B,27 GPIO2 (LP) */
    DMA0_IRQn, /* 0x1C,28 DMA0 */
    DMA1_IRQn, /* 0x1D,29 DMA1 */
    DMA2_IRQn, /* 0x1E,30 DMA2 */
    DMA3_IRQn, /* 0x1F,31 DMA3 */
    RSV32_IRQn, /* 0x20,32 Reserved */
    RSV33_IRQn, /* 0x21,33 Reserved */
    RSV34_IRQn, /* 0x22,34 Reserved */
    RSV35_IRQn, /* 0x23,35 Reserved */
    RSV36_IRQn, /* 0x24,36 Reserved */
    RSV37_IRQn, /* 0x25,37 Reserved */
    RSV38_IRQn, /* 0x26,38 Reserved */
    RSV39_IRQn, /* 0x27,39 Reserved */
    RSV40_IRQn, /* 0x28,40 Reserved */
    RSV41_IRQn, /* 0x29,41 Reserved */
    RSV42_IRQn, /* 0x2A,42 Reserved */
    RSV43_IRQn, /* 0x2B,43 Reserved */
    RSV44_IRQn, /* 0x2C,44 Reserved */
    RSV45_IRQn, /* 0x2D,45 Reserved */
    AES_IRQn, /* 0x2E,46 AES  */
    TRNG_IRQn, /* 0x2F,47 True Random Number Generator */
    WDT1_IRQn, /* 0x30,48 Watchdog 1 (LP) */
    DVS_IRQn, /* 0x31,49 DVS Controller */
    SIMO_IRQn, /* 0x32,50 SIMO Controller */
    CRC_IRQn, /* 0x33,51 CRC  */
    PT_IRQn, /* 0x34,52 Pulse train */
    ADC_IRQn, /* 0x35,53 ADC */
    OWM_IRQn, /* 0x36,54 One Wire Master */
    I2S_IRQn, /* 0x37,55 I2S */
    CNN_FIFO_IRQn, /* 0x38,56 CNN FIFO */
    CNN_IRQn, /* 0x39,57 CNN  */
    RSV58_IRQn, /* 0x3A,58 Reserved  */
    PCIF_IRQn, /* 0x3B,59 Parallel Camera IF */
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

#include "system_max78002.h" /*!< System Header                                          */

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00010000UL
#define MXC_FLASH0_MEM_BASE 0x10000000UL
#define MXC_FLASH_MEM_BASE MXC_FLASH0_MEM_BASE
#define MXC_FLASH_PAGE_SIZE 0x00004000UL
#define MXC_FLASH_MEM_SIZE 0x00280000UL
#define MXC_INFO0_MEM_BASE 0x10800000UL
#define MXC_INFO_MEM_BASE MXC_INFO0_MEM_BASE
#define MXC_INFO_MEM_SIZE 0x00008000UL
#define MXC_SRAM_MEM_BASE 0x20000000UL
#define MXC_SRAM_MEM_SIZE 0x00060000UL

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX78002 peripheral modules.
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
/*                                                    Windowed Watchdog Timer */
#define MXC_CFG_WDT_INSTANCES (2)

#define MXC_BASE_WDT0 ((uint32_t)0x40003000UL)
#define MXC_WDT0 ((mxc_wdt_regs_t *)MXC_BASE_WDT0)
#define MXC_BASE_WDT1 ((uint32_t)0x40080800UL)
#define MXC_WDT1 ((mxc_wdt_regs_t *)MXC_BASE_WDT1)

/******************************************************************************/
/*                                                               SIMO Control */
#define MXC_BASE_SIMO ((uint32_t)0x40004400UL)
#define MXC_SIMO ((mxc_simo_regs_t *)MXC_BASE_SIMO)

/******************************************************************************/
/*                                      Dynamic Voltage Scaling (DVS) Control */
#define MXC_BASE_DVS ((uint32_t)0x40004800UL)
#define MXC_DVS ((mxc_dvs_regs_t *)MXC_BASE_DVS)

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
#define MXC_BASE_WUT ((uint32_t)0x40006400UL)
#define MXC_WUT ((mxc_wut_regs_t *)MXC_BASE_WUT)

/******************************************************************************/
/*                                                            Power Sequencer */
#define MXC_BASE_PWRSEQ ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ)

/******************************************************************************/
/*                                                              Misc Control  */
#define MXC_BASE_MCR ((uint32_t)0x40006C00UL)
#define MXC_MCR ((mxc_mcr_regs_t *)MXC_BASE_MCR)

/******************************************************************************/
/*                                                                        AES */
#define MXC_BASE_AES ((uint32_t)0x40007400UL)
#define MXC_AES ((mxc_aes_regs_t *)MXC_BASE_AES)

/******************************************************************************/
/*                                                                   AES Keys */
#define MXC_BASE_AESKEYS ((uint32_t)0x40007800UL)
#define MXC_AESKEYS ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS)

// DEPRECATED(1-10-2023): Scheduled for removal.
#define MXC_BASE_AESKEY MXC_BASE_AESKEYS
#define MXC_AESKEY ((mxc_aes_key_regs_t *)MXC_BASE_AESKEY)

/******************************************************************************/
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (4)
#define MXC_CFG_GPIO_PINS_PORT (32)

#define MXC_BASE_GPIO0 ((uint32_t)0x40008000UL)
#define MXC_GPIO0 ((mxc_gpio_regs_t *)MXC_BASE_GPIO0)
#define MXC_BASE_GPIO1 ((uint32_t)0x40009000UL)
#define MXC_GPIO1 ((mxc_gpio_regs_t *)MXC_BASE_GPIO1)
#define MXC_BASE_GPIO2 ((uint32_t)0x40080400UL)
#define MXC_GPIO2 ((mxc_gpio_regs_t *)MXC_BASE_GPIO2)
//GPIO3 dummy address it does not live here and will be handled in code different than other gpios but this allow our macros to work.
#define MXC_BASE_GPIO3 ((uint32_t)0x4000B000UL)
#define MXC_GPIO3 ((mxc_gpio_regs_t *)MXC_BASE_GPIO3)

#define MXC_GPIO_GET_IDX(p) \
    ((p) == MXC_GPIO0 ? 0 : (p) == MXC_GPIO1 ? 1 : (p) == MXC_GPIO2 ? 2 : (p) == MXC_GPIO3 ? 3 : -1)

#define MXC_GPIO_GET_GPIO(i) \
    ((i) == 0 ? MXC_GPIO0 : (i) == 1 ? MXC_GPIO1 : (i) == 2 ? MXC_GPIO2 : (i) == 3 ? MXC_GPIO3 : 0)

#define MXC_GPIO_GET_IRQ(i)     \
    ((i) == 0 ? GPIO0_IRQn :    \
     (i) == 1 ? GPIO1_IRQn :    \
     (i) == 2 ? GPIO2_IRQn :    \
     (i) == 3 ? GPIOWAKE_IRQn : \
                0)

/******************************************************************************/
/*                                                  Parallel Camera Interface */
#define MXC_BASE_PCIF ((uint32_t)0x4000E000UL)
#define MXC_PCIF ((mxc_cameraif_regs_t *)MXC_BASE_PCIF)

/******************************************************************************/
/*                                                                        CRC */
#define MXC_BASE_CRC ((uint32_t)0x4000F000UL)
#define MXC_CRC ((mxc_crc_regs_t *)MXC_BASE_CRC)

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

#define MXC_I2C_GET_TMR(i) ((i) == 0 ? MXC_I2C0 : (i) == 1 ? MXC_I2C1 : (i) == 2 ? MXC_I2C2 : 0)

#define MXC_I2C_GET_IDX(p) ((p) == MXC_I2C0 ? 0 : (p) == MXC_I2C1 ? 1 : (p) == MXC_I2C2 ? 2 : -1)
#define MXC_I2C_FIFO_DEPTH (8)

/******************************************************************************/
/*                                                                        DMA */
#define MXC_DMA_CHANNELS (4)
#define MXC_DMA_INSTANCES (1)

#define MXC_BASE_DMA ((uint32_t)0x40028000UL)
#define MXC_DMA ((mxc_dma_regs_t *)MXC_BASE_DMA)

#define MXC_DMA_GET_IDX(p) ((p) == MXC_DMA ? 0 : -1)

#define MXC_DMA_CH_GET_IRQ(i) ((IRQn_Type)(((i) == 0) ? DMA0_IRQn : ((i) == 1) ? DMA1_IRQn : \
                                ((i) == 2) ? DMA2_IRQn : ((i) == 3) ? DMA3_IRQn : 0))

/******************************************************************************/
/*                                                                        FLC */
#define MXC_FLC_INSTANCES (1)

#define MXC_BASE_FLC0 ((uint32_t)0x40029000UL)
#define MXC_FLC0 ((mxc_flc_regs_t *)MXC_BASE_FLC0)
#define MXC_FLC MXC_FLC0

#define MXC_FLC_GET_IRQ(i) (IRQn_Type)((i) == 0 ? FLC0_IRQn : 0)

#define MXC_FLC_GET_BASE(i) ((i) == 0 ? MXC_BASE_FLC0 : 0)

#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC0 : 0)

#define MXC_FLC_GET_IDX(p) ((p) == MXC_FLC0 ? 0 : -1)

/******************************************************************************/
/*                                                          Instruction Cache */
#define MXC_ICC_INSTANCES (2)

#define MXC_BASE_ICC0 ((uint32_t)0x4002A000UL)
#define MXC_ICC0 ((mxc_icc_regs_t *)MXC_BASE_ICC0)

#define MXC_BASE_ICC1 ((uint32_t)0x4002A800UL)
#define MXC_ICC1 ((mxc_icc_regs_t *)MXC_BASE_ICC1)

#define MXC_ICC MXC_ICC0
// ICC1 is the RISC-V cache

/******************************************************************************/
/*                                                                        ADC */
#define MXC_BASE_ADC ((uint32_t)0x40034000UL)
#define MXC_ADC ((mxc_adc_regs_t *)MXC_BASE_ADC)
#define MXC_ADC_MAX_CLOCK 8000000 // Maximum ADC clock in Hz

/*******************************************************************************/
/*                                                      Pulse Train Generation */
#define MXC_CFG_PT_INSTANCES (4)

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

#define MXC_PT_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_PT0 : \
     (i) == 1 ? MXC_BASE_PT1 : \
     (i) == 2 ? MXC_BASE_PT2 : \
     (i) == 3 ? MXC_BASE_PT3 : \
                0)

#define MXC_PT_GET_PT(i) \
    ((i) == 0 ? MXC_PT0 : (i) == 1 ? MXC_PT1 : (i) == 2 ? MXC_PT2 : (i) == 3 ? MXC_PT3 : 0)

#define MXC_PT_GET_IDX(p) \
    ((p) == MXC_PT0 ? 0 : (p) == MXC_PT1 ? 1 : (p) == MXC_PT2 ? 2 : (p) == MXC_PT3 ? 3 : -1)

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

#define MXC_UART_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? UART0_IRQn : \
                (i) == 1 ? UART1_IRQn : \
                (i) == 2 ? UART2_IRQn : \
                (i) == 3 ? UART3_IRQn : \
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
#define MXC_SPI_INSTANCES (2)
#else
#define MXC_SPI_INSTANCES (1)
#endif // __riscv
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

#define MXC_BASE_SPI1 ((uint32_t)0x40046000UL)
#define MXC_SPI1 ((mxc_spi_regs_t *)MXC_BASE_SPI1)
#define MXC_SPI1_TS_INSTANCES (1)
#ifndef __riscv
#define MXC_BASE_SPI0 ((uint32_t)0x400BE000UL)
#define MXC_SPI0 ((mxc_spi_regs_t *)MXC_BASE_SPI0)
#define MXC_SPI0_TS_INSTANCES (3)

// Note: These must be in order SPI1, SPI0 to support RISC-V
#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI1 ? 0 : (p) == MXC_SPI0 ? 1 : -1)

#define MXC_SPI_GET_BASE(i) ((i) == 0 ? MXC_BASE_SPI1 : (i) == 1 ? MXC_BASE_SPI0 : 0)

#define MXC_SPI_GET_SPI(i) ((i) == 0 ? MXC_SPI1 : (i) == 1 ? MXC_SPI0 : 0)

#define MXC_SPI_GET_IRQ(i) (IRQn_Type)((i) == 0 ? SPI1_IRQn : (i) == 1 ? SPI0_IRQn : 0)

#define MXC_SPI_GET_TOTAL_TS(p) \
    ((p) == MXC_SPI1 ? MXC_SPI1_TS_INSTANCES : (p) == MXC_SPI0 ? MXC_SPI0_TS_INSTANCES : 0)
#else // __riscv

#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI1 ? 0 : -1)

#define MXC_SPI_GET_BASE(i) ((i) == 0 ? MXC_BASE_SPI1 : 0)

#define MXC_SPI_GET_SPI(i) ((i) == 0 ? MXC_SPI1 : 0)

#define MXC_SPI_GET_IRQ(i) (IRQn_Type)((i) == 0 ? SPI1_IRQn : 0)

#define MXC_SPI_GET_TOTAL_TS(p) ((p) == MXC_SPI1 ? MXC_SPI1_TS_INSTANCES : 0)

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
/*                                                                       CSI2 */
#define MXC_BASE_CSI2 ((uint32_t)0x40062000UL)
#define MXC_CSI2 ((mxc_csi2_regs_t *)MXC_BASE_CSI2)

/******************************************************************************/
/*                                                  Low Power General control */
#define MXC_BASE_LPGCR ((uint32_t)0x40080000UL)
#define MXC_LPGCR ((mxc_lpgcr_regs_t *)MXC_BASE_LPGCR)

/******************************************************************************/
/*                                                       Low-Power Comparator */
#define MXC_BASE_LPCMP ((uint32_t)0x40088000UL)
#define MXC_LPCMP ((mxc_lpcmp_regs_t *)MXC_BASE_LPCMP)

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

#define mxc_usb_clock_t _mxc_usb_clock_t

/******************************************************************************/
/*                                                                       SDHC */
#define MXC_BASE_SDHC ((uint32_t)0x400B6000UL)
#define MXC_SDHC ((mxc_sdhc_regs_t *)MXC_BASE_SDHC)

/******************************************************************************/
/*                                                                  CSI2 FIFO */
#define MXC_BASE_CSI2_FIFO ((uint32_t)0x400C0800UL)
#define MXC_CSI2_FIFO ((uint32_t *)MXC_BASE_CSI2_FIFO)
#define MXC_CSI2_FIFO_DEPTH (128)

/******************************************************************************/
/*                                                                   CNN FIFO */
#define MXC_BASE_CNN_FIFO ((uint32_t)0x400C0400UL)
#define MXC_CNN_FIFO ((mxc_cnn_fifo_regs_t *)MXC_BASE_CNN_FIFO)

/******************************************************************************/
/*                                                                        CNN */
#define MXC_BASE_CNN ((uint32_t)0x50000000UL)
#define MXC_CNN ((mxc_cnn_regs_t *)MXC_BASE_CNN)

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

// clang-format on

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX78002_INCLUDE_MAX78002_H_
