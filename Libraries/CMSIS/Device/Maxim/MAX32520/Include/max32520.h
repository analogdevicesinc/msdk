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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MAX32520_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MAX32520_H_

#ifndef TARGET_NUM
#define TARGET_NUM 32520
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
    RSV2_IRQn, /* 0x12  0x0048  18: Reserved */
    RSV3_IRQn, /* 0x13  0x004C  19: Reserved */
    TRNG_IRQn, /* 0x14  0x0050  20: True Random Number Generator */
    TMR0_IRQn, /* 0x15  0x0054  21: Timer 0 */
    TMR1_IRQn, /* 0x16  0x0058  22: Timer 1 */
    TMR2_IRQn, /* 0x17  0x005C  23: Timer 2 */
    TMR3_IRQn, /* 0x18  0x0060  24: Timer 3*/
    RSV9_IRQn, /* 0x19  0x0064  25: Reserved */
    RSV10_IRQn, /* 0x1A  0x0068  26: Reserved */
    RSV11_IRQn, /* 0x1B  0x006C  27: Reserved */
    RSV12_IRQn, /* 0x1C  0x0070  28: Reserved */
    I2C0_IRQn, /* 0x1D  0x0074  29: I2C0 */
    UART0_IRQn, /* 0x1E  0x0078  30: UART 0 */
    RSV15_IRQn, /* 0x1F  0x007C  31: Reserved */
    SPI0_IRQn, /* 0x20  0x0080  32: SPI0 */
    SPI1_IRQn, /* 0x21  0x0084  33: SPI1 */
    RSV18_IRQn, /* 0x22  0x0088  34: Reserved */
    RSV19_IRQn, /* 0x23  0x008C  35: Reserved */
    RSV20_IRQn, /* 0x24  0x0090  36: Reserved */
    RSV21_IRQn, /* 0x25  0x0094  37: Reserved */
    RSV22_IRQn, /* 0x26  0x0098  38: Reserved */
    FLC_IRQn, /* 0x27  0x009C  39: Flash Controller */
    GPIO0_IRQn, /* 0x28  0x00A0  40: GPIO0 */
    GPIO1_IRQn, /* 0x29  0x00A4  41: GPIO2 */
    RSV26_IRQn, /* 0x2A  0x00A8  42: Reserved */
    CRYPTO_IRQn, /* 0x2B  0x00AC  43: Crypto */
    DMA0_IRQn, /* 0x2C  0x00B0  44: DMA0 */
    DMA1_IRQn, /* 0x2D  0x00B4  45: DMA1 */
    DMA2_IRQn, /* 0x2E  0x00B8  46: DMA2 */
    DMA3_IRQn, /* 0x2F  0x00BC  47: DMA3 */
    RSV32_IRQn, /* 0x30  0x00C0  48: Reserved */
    RSV33_IRQn, /* 0x31  0x00C4  49: Reserved */
    RSV34_IRQn, /* 0x32  0x00C8  50: Reserved */
    RSV35_IRQn, /* 0x33  0x00CC  51: Reserved */
    RSV36_IRQn, /* 0x34  0x00D0  52: Reserved */
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
    RSV53_IRQn, /* 0x45  0x0114  69: Reserved */
    GPIOWAKE_IRQn, /* 0x46  0x0118  70: GPIO Wakeup */
    RSV55_IRQn, /* 0x47  0x011C  71: Reserved */
    RSV56_IRQn, /* 0x48  0x0120  72: Reserved */
    WDT1_IRQn, /* 0x49  0x0124  73: Watchdog 1 */
    RSV58_IRQn, /* 0x4A  0x0128  74: Reserved */
    RSV59_IRQn, /* 0x4B  0x012C  75: Reserved */
    RSV60_IRQn, /* 0x4C  0x0130  76: Reserved */
    RSV61_IRQn, /* 0x4D  0x0134  77: Reserved */
    RSV62_IRQn, /* 0x4E  0x0138  78: Reserved */
    RSV63_IRQn, /* 0x4F  0x013C  79: Reserved */
    RSV64_IRQn, /* 0x50  0x0140  80: Reserved */
    RSV65_IRQn, /* 0x51  0x0144  81: Reserved */
    RSV66_IRQn, /* 0x52  0x0148  82: Reserved */
    RSV67_IRQn, /* 0x53  0x014C  83: Reserved */
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
    ECC_IRQn, /* 0x62  0x0188  98: Error Correction */
    RSV83_IRQn, /* 0x63  0x018C  99: Reserved */
    RSV84_IRQn, /* 0x64  0x0190  100: Reserved */
    SCA_IRQn, /* 0x65  0x0194  101: SCA Crypto */
    RSV86_IRQn, /* 0x66  0x0198  102: Reserved */
    RSV87_IRQn, /* 0x67  0x019C  103: Reserved */
    RSV88_IRQn, /* 0x68  0x01A0  104: Reserved */
    RSV89_IRQn, /* 0x69  0x01A4  105: Reserved */
    RSV90_IRQn, /* 0x6A  0x01A8  106: Reserved */
    RSV91_IRQn, /* 0x6B  0x01AC  107: Reserved */
    RSV92_IRQn, /* 0x6C  0x01B0  108: Reserved */
    RSV93_IRQn, /* 0x6D  0x01B4  109: Reserved */
    RSV94_IRQn, /* 0x6E  0x01B8  110: Reserved */
    RSV95_IRQn, /* 0x6F  0x01BC  111: Reserved */
    SFE_IRQn, /* 0x70  0x01C0  112: SFE */
    MXC_IRQ_EXT_COUNT,
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
#include "system_max32520.h" /*!< System Header                                          */

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00020000UL
#define MXC_FLASH0_MEM_BASE 0x10000000UL
#define MXC_FLASH1_MEM_BASE 0x10100000UL
#define MXC_FLASH_MEM_BASE MXC_FLASH0_MEM_BASE
#define MXC_FLASH_PAGE_SIZE 0x00004000UL
#define MXC_FLASH_MEM_SIZE 0x00200000UL
#define MXC_INFO0_MEM_BASE 0x10800000UL
#define MXC_INFO1_MEM_BASE 0x10806000UL
#define MXC_INFO_MEM_BASE MXC_INFO0_MEM_BASE
#define MXC_INFO_MEM_SIZE 0x00008000UL
#define MXC_SRAM_MEM_BASE 0x20000000UL
#define MXC_SRAM_MEM_SIZE 0x0002A800UL

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX32520 peripheral modules.
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
/*                                                           Function Control */
#define MXC_BASE_FCR ((uint32_t)0x40000800UL)
#define MXC_FCR ((mxc_nbbfc_regs_t *)MXC_BASE_FCR)

/******************************************************************************/
/*                                                      Trust Protection Unit */
#define MXC_BASE_CTB ((uint32_t)0x40001000UL)
#define MXC_CTB ((mxc_ctb_regs_t *)MXC_BASE_CTB)

/******************************************************************************/
/*                                                                   Watchdog */
#define MXC_BASE_WDT0 ((uint32_t)0x40003000UL)
#define MXC_WDT0 ((mxc_wdt_regs_t *)MXC_BASE_WDT0)
#define MXC_BASE_WDT1 ((uint32_t)0x40003400UL)
#define MXC_WDT1 ((mxc_wdt_regs_t *)MXC_BASE_WDT1)

/******************************************************************************/
/*                                                           Security Monitor */
#define MXC_BASE_SMON ((uint32_t)0x40004000UL)
#define MXC_SMON ((mxc_smon_regs_t *)MXC_BASE_SMON)

/******************************************************************************/
/*                                                                   AES Keys */
#define MXC_BASE_AESKEYS ((uint32_t)0x40005000UL)
#define MXC_AESKEYS ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS)

/******************************************************************************/
/*                                                            Power Sequencer */
#define MXC_BASE_PWRSEQ ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ)

/******************************************************************************/
/*                                                          MISC Control Regs */
#define MXC_BASE_MCR ((uint32_t)0x40006C00UL)
#define MXC_MCR ((mxc_mcr_regs_t *)MXC_BASE_MCR)

/******************************************************************************/
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (2)
#define MXC_CFG_GPIO_PINS_PORT (16)

#define MXC_BASE_GPIO0 ((uint32_t)0x40008000UL)
#define MXC_GPIO0 ((mxc_gpio_regs_t *)MXC_BASE_GPIO0)
#define MXC_BASE_GPIO1 ((uint32_t)0x40009000UL)
#define MXC_GPIO1 ((mxc_gpio_regs_t *)MXC_BASE_GPIO1)

#define MXC_GPIO_GET_IDX(p) ((p) == MXC_GPIO0 ? 0 : (p) == MXC_GPIO1 ? 1 : -1)

#define MXC_GPIO_GET_GPIO(i) ((i) == 0 ? MXC_GPIO0 : (i) == 1 ? MXC_GPIO1 : 0)

#define MXC_GPIO_GET_IRQ(i) ((i) == 0 ? GPIO0_IRQn : (i) == 1 ? GPIO1_IRQn : 0)

/******************************************************************************/
#define SEC(s) (((uint32_t)s) * 1000000UL)
#define MSEC(ms) (ms * 1000UL)
#define USEC(us) (us)
/*                                                                      Timer */
#define MXC_CFG_TMR_INSTANCES (4)

#define MXC_BASE_TMR0 ((uint32_t)0x40010000UL)
#define MXC_TMR0 ((mxc_tmr_regs_t *)MXC_BASE_TMR0)
#define MXC_BASE_TMR1 ((uint32_t)0x40011000UL)
#define MXC_TMR1 ((mxc_tmr_regs_t *)MXC_BASE_TMR1)
#define MXC_BASE_TMR2 ((uint32_t)0x40012000UL)
#define MXC_TMR2 ((mxc_tmr_regs_t *)MXC_BASE_TMR2)
#define MXC_BASE_TMR3 ((uint32_t)0x40013000UL)
#define MXC_TMR3 ((mxc_tmr_regs_t *)MXC_BASE_TMR3)

#define MXC_TMR_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? TMR0_IRQn : \
                (i) == 1 ? TMR1_IRQn : \
                (i) == 2 ? TMR2_IRQn : \
                (i) == 3 ? TMR3_IRQn : \
                           0)

#define MXC_TMR_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_TMR0 : \
     (i) == 1 ? MXC_BASE_TMR1 : \
     (i) == 2 ? MXC_BASE_TMR2 : \
     (i) == 3 ? MXC_BASE_TMR3 : \
                0)

#define MXC_TMR_GET_TMR(i) \
    ((i) == 0 ? MXC_TMR0 : (i) == 1 ? MXC_TMR1 : (i) == 2 ? MXC_TMR2 : (i) == 3 ? MXC_TMR3 : 0)

#define MXC_TMR_GET_IDX(p) \
    ((p) == MXC_TMR0 ? 0 : (p) == MXC_TMR1 ? 1 : (p) == MXC_TMR2 ? 2 : (p) == MXC_TMR3 ? 3 : -1)

/******************************************************************************/
/*                                                                        I2C */
#define MXC_I2C_INSTANCES (1)

#define MXC_BASE_I2C0 ((uint32_t)0x4001D000UL)
#define MXC_I2C0 ((mxc_i2c_regs_t *)MXC_BASE_I2C0)

#define MXC_I2C_GET_IRQ(i) (IRQn_Type)((i) == 0 ? I2C0_IRQn : 0)

#define MXC_I2C_GET_BASE(i) ((i) == 0 ? MXC_BASE_I2C0 : 0)

#define MXC_I2C_GET_TMR(i) ((i) == 0 ? MXC_I2C0 : 0)

#define MXC_I2C_GET_IDX(p) ((p) == MXC_I2C0 ? 0 : -1)
#define MXC_I2C_FIFO_DEPTH (8)

#define MXC_I2C_GET_I2C(p) ((p) == 0x0 ? MXC_I2C0 : 0)

/******************************************************************************/
/*                                               UART / Serial Port Interface */
#define MXC_UART_INSTANCES (1)
#define MXC_UART_FIFO_DEPTH (8)

#define MXC_BASE_UART0 ((uint32_t)0x40020000UL)
#define MXC_UART0 ((mxc_uart_regs_t *)MXC_BASE_UART0)

#define MXC_UART_GET_IRQ(i) (IRQn_Type)((i) == 0 ? UART0_IRQn : 0)

#define MXC_UART_GET_BASE(i) ((i) == 0 ? MXC_BASE_UART0 : 0)

#define MXC_UART_GET_UART(i) ((i) == 0 ? MXC_UART0 : 0)

#define MXC_UART_GET_IDX(p) ((p) == MXC_UART0 ? 0 : -1)

/******************************************************************************/
/*                                                                        DMA */
#define MXC_DMA_CHANNELS (4)
#define MXC_DMA_INSTANCES (1)

#define MXC_BASE_DMA ((uint32_t)0x40028000UL)
#define MXC_DMA ((mxc_dma_regs_t *)MXC_BASE_DMA)

#define MXC_DMA_GET_IDX(p) ((p) == MXC_DMA ? 0 : -1)

#define MXC_DMA_CH_GET_IRQ(i)             \
    ((IRQn_Type)(((i) == 0) ? DMA0_IRQn : \
                 ((i) == 1) ? DMA1_IRQn : \
                 ((i) == 2) ? DMA2_IRQn : \
                 ((i) == 3) ? DMA3_IRQn : \
                              0))

/******************************************************************************/
/*                                                                        FLC */
#define MXC_FLC_INSTANCES (1)

#define MXC_BASE_FLC ((uint32_t)0x40029000UL)
#define MXC_FLC ((mxc_flc_regs_t *)MXC_BASE_FLC)

#define MXC_FLC0 MXC_FLC
#define FLC0_IRQn FLC_IRQn

#define MXC_FLC_GET_IRQ(i) (IRQn_Type)((i) == 0 ? FLC_IRQn : 0)

#define MXC_FLC_GET_BASE(i) ((i) == 0 ? MXC_BASE_FLC : 0)

#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC : 0)

#define MXC_FLC_GET_IDX(p) ((p) == MXC_FLC ? 0 : -1)

/******************************************************************************/
/*                                                          Instruction Cache */
#define MXC_BASE_ICC ((uint32_t)0x4002A000UL)
#define MXC_ICC ((mxc_icc_regs_t *)MXC_BASE_ICC)

/******************************************************************************/
/*                                                                        SPI */
#define MXC_SPI_INSTANCES (2)
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

#define MXC_BASE_SPI0 ((uint32_t)0x40046000UL)
#define MXC_SPI0 ((mxc_spi_regs_t *)MXC_BASE_SPI0)
#define MXC_BASE_SPI1 ((uint32_t)0x40047000UL)
#define MXC_SPI1 ((mxc_spi_regs_t *)MXC_BASE_SPI1)

#define MXC_SPI_GET_IDX(p) ((p) == MXC_SPI0 ? 0 : (p) == MXC_SPI1 ? 1 : -1)

#define MXC_SPI_GET_BASE(i) ((i) == 0 ? MXC_BASE_SPI0 : (i) == 1 ? MXC_BASE_SPI1 : 0)

#define MXC_SPI_GET_SPI(i) ((i) == 0 ? MXC_SPI0 : (i) == 1 ? MXC_SPI1 : 0)

#define MXC_SPI_GET_IRQ(i) (IRQn_Type)((i) == 0 ? SPI0_IRQn : (i) == 1 ? SPI1_IRQn : 0)

/******************************************************************************/
/*                                                                       TRNG */
#define MXC_BASE_TRNG ((uint32_t)0x4004D000UL)
#define MXC_TRNG ((mxc_trng_regs_t *)MXC_BASE_TRNG)

/******************************************************************************/
/*                                                                        SFE */
#define MXC_BASE_SFE ((uint32_t)0x400A0000UL)
#define MXC_SFE ((mxc_sfe_regs_t *)MXC_BASE_SFE)

/******************************************************************************/
/*                                                                      CSPIS */
#define MXC_BASE_CSPIS ((uint32_t)0x400A0000UL)
#define MXC_CSPIS ((mxc_cspis_regs_t *)MXC_BASE_CSPIS)

#define CSPIS_IRQn SFE_IRQn

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
/*                                                                  SCB CPACR */

/* Note: Added by Maxim Integrated, as these are missing from CMSIS/Core/Include/core_cm4.h */
#define SCB_CPACR_CP10_Pos 20 /*!< SCB CPACR: Coprocessor 10 Position */
#define SCB_CPACR_CP10_Msk (0x3UL << SCB_CPACR_CP10_Pos) /*!< SCB CPACR: Coprocessor 10 Mask */
#define SCB_CPACR_CP11_Pos 22 /*!< SCB CPACR: Coprocessor 11 Position */
#define SCB_CPACR_CP11_Msk (0x3UL << SCB_CPACR_CP11_Pos) /*!< SCB CPACR: Coprocessor 11 Mask */

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32520_INCLUDE_MAX32520_H_
