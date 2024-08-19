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

#ifndef LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_MAX32650_H_
#define LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_MAX32650_H_

#ifndef TARGET_NUM
#define TARGET_NUM 32650
#endif

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
#endif /* !__GNUC__ */

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

    /* Device-specific interrupt sources (external to ARM core)          */
    /*               table entry number                                  */
    /*               ||||                                                */
    /*               ||||  table offset address                          */
    /*               vvvv  vvvvvv                                        */

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
    LCD_IRQn, /* 0x1C  0x0070  28: LCD Controller*/
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
    FLC_IRQn, /* 0x27  0x009C  39: Flash Controller */
    GPIO0_IRQn, /* 0x28  0x00A0  40: GPIO0 */
    GPIO1_IRQn, /* 0x29  0x00A4  41: GPIO2 */
    GPIO2_IRQn, /* 0x2A  0x00A8  42: GPIO3 */
    TPU_IRQn, /* 0x2B  0x00AC  43: TPU */
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
    SPIXFC_IRQn, /* 0x36  0x00D8  54: SPI execute in place */
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
    SPI3_IRQn, /* 0x48  0x0120  72: SPI3 */
    WDT1_IRQn, /* 0x49  0x0124  73: Watchdog 1 */
    GPIO3_IRQn, /* 0x4A  0x0128  74: GPIO3 */
    PT_IRQn, /* 0x4B  0x012C  75: Pulse train */
    SDMA_IRQn, /* 0x4C  0x0130  76: Smart DMA */
    HBMC_IRQn, /* 0x4D  0x0134  77: HyperBus */
    RSV62_IRQn, /* 0x4E  0x0138  78: Reserved */
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
    MXC_IRQ_EXT_COUNT,
} IRQn_Type;

#define MXC_IRQ_COUNT (MXC_IRQ_EXT_COUNT + 16)

/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* ------------------------  Configuration of the Cortex-M Processor and Core Peripherals  ------------------------ */
#define __CM4_REV 0x0100 /*!< Cortex-M4 Core Revision                                */
#define __MPU_PRESENT 1 /*!< MPU present or not                                     */
#define __NVIC_PRIO_BITS 3 /*!< Number of Bits used for Priority Levels                */
#define __Vendor_SysTickConfig 0 /*!< Set to 1 if different SysTick Config is used           */
#define __FPU_PRESENT 1 /*!< FPU present or not                                     */

#include <core_cm4.h> /*!< Cortex-M4 processor and core peripherals               */
#include "system_max32650.h" /*!< System Header                                          */

/* ================================================================================ */
/* ==================       Device Specific Memory Section       ================== */
/* ================================================================================ */

#define MXC_ROM_MEM_BASE 0x00000000UL
#define MXC_ROM_MEM_SIZE 0x00020000UL
#define MXC_XIP_MEM_BASE 0x08000000UL
#define MXC_XIP_MEM_SIZE 0x08000000UL
#define MXC_FLASH_MEM_BASE 0x10000000UL
#define MXC_FLASH_PAGE_SIZE 0x00004000UL
#define MXC_FLASH_MEM_SIZE 0x00300000UL
#define MXC_INFO_MEM_BASE 0x10800000UL
#define MXC_INFO_MEM_SIZE 0x00004000UL
#define MXC_SRAM_MEM_BASE 0x20000000UL
#define MXC_SRAM_MEM_SIZE 0x00100000UL
#define MXC_HBMC_MEM_BASE 0x60000000UL
#define MXC_HBMC_MEM_SIZE 0x20000000UL
#define MXC_XIP_DATA_MEM_BASE 0x80000000UL
#define MXC_XIP_DATA_MEM_SIZE 0x20000000UL

/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*
   Base addresses and configuration settings for all MAX32650 peripheral modules.
*/

/* ************************************************************************** */
/*                                                             Global control */
#define MXC_BASE_GCR ((uint32_t)0x40000000UL)
#define MXC_GCR ((mxc_gcr_regs_t *)MXC_BASE_GCR)

/* ************************************************************************** */
/*                                            Non-battery backed SI Registers */
#define MXC_BASE_SIR ((uint32_t)0x40000400UL)
#define MXC_SIR ((mxc_sir_regs_t *)MXC_BASE_SIR)

/* ************************************************************************** */
/*                                        Non-battery backed Function Control */
#define MXC_BASE_NBBFC ((uint32_t)0x40000800UL)
#define MXC_NBBFC ((mxc_nbbfc_regs_t *)MXC_BASE_NBBFC)

/* ************************************************************************** */
/*                                                                     TPU */
#define MXC_BASE_TPU ((uint32_t)0x40001000UL)
#define MXC_TPU ((mxc_tpu_regs_t *)MXC_BASE_TPU)

/* ************************************************************************** */
/*                                                                   Watchdog */
#define MXC_BASE_WDT0 ((uint32_t)0x40003000UL)
#define MXC_WDT0 ((mxc_wdt_regs_t *)MXC_BASE_WDT0)
#define MXC_BASE_WDT1 ((uint32_t)0x40003400UL)
#define MXC_WDT1 ((mxc_wdt_regs_t *)MXC_BASE_WDT1)

/* ************************************************************************** */
/*                                                           Security Monitor */
#define MXC_BASE_SMON ((uint32_t)0x40004000UL)
#define MXC_SMON ((mxc_smon_regs_t *)MXC_BASE_SMON)

/* ************************************************************************** */
/*                                                                   AES Keys */
#define MXC_BASE_AESKEYS ((uint32_t)0x40005000UL)
#define MXC_AESKEYS ((mxc_aeskeys_regs_t *)MXC_BASE_AESKEYS)

/* ************************************************************************** */
/*                                       Trim System Initialization Registers */
#define MXC_BASE_TRIMSIR ((uint32_t)0x40005400UL)
#define MXC_TRIMSIR ((mxc_trimsir_regs_t *)MXC_BASE_TRIMSIR)

/* ************************************************************************** */
/*                                                                       BBFC */
#define MXC_BASE_BBFC ((uint32_t)0x40005800UL)
#define MXC_BBFC ((mxc_bbfc_regs_t *)MXC_BASE_BBFC)

/* ************************************************************************** */
/*                                                            Real Time Clock */
#define MXC_BASE_RTC ((uint32_t)0x40006000UL)
#define MXC_RTC ((mxc_rtc_regs_t *)MXC_BASE_RTC)

/* ************************************************************************** */
/*                                                            Power Sequencer */
#define MXC_BASE_PWRSEQ ((uint32_t)0x40006800UL)
#define MXC_PWRSEQ ((mxc_pwrseq_regs_t *)MXC_BASE_PWRSEQ)

/* ************************************************************************** */
/*                                                                       GPIO */
#define MXC_CFG_GPIO_INSTANCES (4)
#define MXC_CFG_GPIO_PINS_PORT (32)

#define MXC_BASE_GPIO0 ((uint32_t)0x40008000UL)
#define MXC_GPIO0 ((mxc_gpio_regs_t *)MXC_BASE_GPIO0)
#define MXC_BASE_GPIO1 ((uint32_t)0x40009000UL)
#define MXC_GPIO1 ((mxc_gpio_regs_t *)MXC_BASE_GPIO1)
#define MXC_BASE_GPIO2 ((uint32_t)0x4000A000UL)
#define MXC_GPIO2 ((mxc_gpio_regs_t *)MXC_BASE_GPIO2)
#define MXC_BASE_GPIO3 ((uint32_t)0x4000B000UL)
#define MXC_GPIO3 ((mxc_gpio_regs_t *)MXC_BASE_GPIO3)

#define MXC_GPIO_GET_IDX(p) \
    ((p) == MXC_GPIO0 ? 0 : (p) == MXC_GPIO1 ? 1 : (p) == MXC_GPIO2 ? 2 : (p) == MXC_GPIO3 ? 3 : -1)

#define MXC_GPIO_GET_GPIO(i) \
    ((i) == 0 ? MXC_GPIO0 : (i) == 1 ? MXC_GPIO1 : (i) == 2 ? MXC_GPIO2 : (i) == 3 ? MXC_GPIO3 : 0)

#define MXC_GPIO_GET_IRQ(i)  \
    ((i) == 0 ? GPIO0_IRQn : \
     (i) == 1 ? GPIO1_IRQn : \
     (i) == 2 ? GPIO2_IRQn : \
     (i) == 3 ? GPIO3_IRQn : \
                0)

/* ************************************************************************** */
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

/* ************************************************************************** */
/*                                                                    SPIMSS  */
#define MXC_SPIMSS_INSTANCES (1)
#define MXC_SPIMSS_FIFO_DEPTH (8)

#define MXC_BASE_SPIMSS ((uint32_t)0x40018000UL)
#define MXC_SPIMSS ((mxc_spimss_regs_t *)MXC_BASE_SPIMSS)

#define MXC_SPIMSS_GET_IDX(p) ((p) == MXC_SPIMSS ? 0 : -1)
#define MXC_SPIMSS_GET_SPI(i) ((i) == 0 ? MXC_SPIMSS : 0)

/* ************************************************************************** */
/*                                                                        I2C */
#define MXC_I2C_INSTANCES (2)

#define MXC_BASE_I2C0 ((uint32_t)0x4001D000UL)
#define MXC_I2C0 ((mxc_i2c_regs_t *)MXC_BASE_I2C0)
#define MXC_BASE_I2C1 ((uint32_t)0x4001E000UL)
#define MXC_I2C1 ((mxc_i2c_regs_t *)MXC_BASE_I2C1)

#define MXC_I2C_GET_IRQ(i) (IRQn_Type)((i) == 0 ? I2C0_IRQn : (i) == 1 ? I2C1_IRQn : 0)

#define MXC_I2C_GET_BASE(i) ((i) == 0 ? MXC_BASE_I2C0 : (i) == 1 ? MXC_BASE_I2C1 : 0)

#define MXC_I2C_GET_I2C(i) ((i) == 0 ? MXC_I2C0 : (i) == 1 ? MXC_I2C1 : 0)

#define MXC_I2C_GET_IDX(p) ((p) == MXC_I2C0 ? 0 : (p) == MXC_I2C1 ? 1 : -1)
#define MXC_I2C_FIFO_DEPTH (8)

/* ************************************************************************** */
/*                                                       SPI Execute in Place */
#define MXC_BASE_SPIXF ((uint32_t)0x40026000UL)
#define MXC_SPIXF ((mxc_spixf_regs_t *)MXC_BASE_SPIXF)

/* ************************************************************************** */
/*                                     SPI Execute in Place Master Controller */
#define MXC_CFG_SPIXFC_FIFO_DEPTH (16)

#define MXC_BASE_SPIXFC ((uint32_t)0x40027000UL)
#define MXC_SPIXFC ((mxc_spixfc_regs_t *)MXC_BASE_SPIXFC)
#define MXC_BASE_SPIXFC_FIFO ((uint32_t)0x400BC000UL)
#define MXC_SPIXFC_FIFO ((mxc_spixfc_fifo_regs_t *)MXC_BASE_SPIXFC_FIFO)

/* ************************************************************************** */
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

/* ************************************************************************** */
/*                                                                        FLC */
#define MXC_FLC_INSTANCES (1)
#define MXC_BASE_FLC ((uint32_t)0x40029000UL)
#define MXC_FLC ((mxc_flc_regs_t *)MXC_BASE_FLC)
#define MXC_FLC0 MXC_FLC

#define MXC_FLC_GET_IRQ(i) (IRQn_Type)((i) == 0 ? FLC_IRQn : 0)

#define MXC_FLC_GET_BASE(i) ((i) == 0 ? MXC_BASE_FLC : 0)

#define MXC_FLC_GET_FLC(i) ((i) == 0 ? MXC_FLC : 0)

#define MXC_FLC_GET_IDX(p) ((p) == MXC_FLC ? 0 : -1)

/* ************************************************************************** */
/*                                                          Instruction Cache */
#define MXC_BASE_ICC0 ((uint32_t)0x4002A000UL)
#define MXC_ICC0 ((mxc_icc_regs_t *)MXC_BASE_ICC0)

#define MXC_ICC MXC_ICC0

/* ************************************************************************** */
/*                                                      Instruction Cache XIP */
#define MXC_BASE_ICC1 ((uint32_t)0x4002F000UL)
#define MXC_ICC1 ((mxc_icc_regs_t *)MXC_BASE_ICC1)

#define MXC_ICX MXC_ICC1

/* ************************************************************************** */
/*                                                                        CLCD */
#define MXC_BASE_CLCD ((uint32_t)0x40031000UL)
#define MXC_CLCD ((mxc_clcd_regs_t *)MXC_BASE_CLCD)

/* ************************************************************************** */
/*                                                                 Data Cache */
#define MXC_BASE_EMCC ((uint32_t)0x40033000UL)
#define MXC_EMCC ((mxc_emcc_regs_t *)MXC_BASE_EMCC)

/* ************************************************************************** */
/*                                                                        ADC */
#define MXC_BASE_ADC ((uint32_t)0x40034000UL)
#define MXC_ADC ((mxc_adc_regs_t *)MXC_BASE_ADC)
#define MXC_ADC_MAX_CLOCK 8000000 // Maximum ADC clock in Hz

/* ************************************************************************** */
/*                                                     XXX Actually reserved! */
#define MXC_BASE_RESERVED ((uint32_t)0x40035000UL)

/* ************************************************************************** */
/*                                                                  Smart DMA */
#define MXC_BASE_SDMA ((uint32_t)0x40036000UL)
#define MXC_SDMA ((mxc_sdma_regs_t *)MXC_BASE_SDMA)

/* ************************************************************************** */
/*                                                                   HyperBus */
#define MXC_BASE_HPB ((uint32_t)0x40039000UL)
#define MXC_HPB ((mxc_hpb_regs_t *)MXC_BASE_HPB)

/* ************************************************************************** */
/*                                                               SPI XIP Data */
#define MXC_BASE_SPIXR ((uint32_t)0x4003A000UL)
#define MXC_SPIXR ((mxc_spixr_regs_t *)MXC_BASE_SPIXR)

/* ************************************************************************** */
/*                                                                   MIPI DSI */
#define MXC_BASE_MIPI ((uint32_t)0x4003B000UL)
#define MXC_MIPI ((mxc_mipi_regs_t *)MXC_BASE_MIPI)

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

/* ************************************************************************** */
/*                                                            One Wire Master */
#define MXC_BASE_OWM ((uint32_t)0x4003D000UL)
#define MXC_OWM ((mxc_owm_regs_t *)MXC_BASE_OWM)

/* ************************************************************************** */
/*                                                                  Semaphore */
#define MXC_CFG_SEMA_INSTANCES (8)

#define MXC_BASE_SEMA ((uint32_t)0x4003E000UL)
#define MXC_SEMA ((mxc_sema_regs_t *)MXC_BASE_SEMA)

/* ************************************************************************** */
/*                                               UART / Serial Port Interface */
#define MXC_UART_INSTANCES (3)
#define MXC_UART_FIFO_DEPTH (32)

#define MXC_BASE_UART0 ((uint32_t)0x40042000UL)
#define MXC_UART0 ((mxc_uart_regs_t *)MXC_BASE_UART0)
#define MXC_BASE_UART1 ((uint32_t)0x40043000UL)
#define MXC_UART1 ((mxc_uart_regs_t *)MXC_BASE_UART1)
#define MXC_BASE_UART2 ((uint32_t)0x40044000UL)
#define MXC_UART2 ((mxc_uart_regs_t *)MXC_BASE_UART2)

#define MXC_UART_GET_IRQ(i) \
    (IRQn_Type)((i) == 0 ? UART0_IRQn : (i) == 1 ? UART1_IRQn : (i) == 2 ? UART2_IRQn : 0)

#define MXC_UART_GET_BASE(i) \
    ((i) == 0 ? MXC_BASE_UART0 : (i) == 1 ? MXC_BASE_UART1 : (i) == 2 ? MXC_BASE_UART2 : 0)

#define MXC_UART_GET_UART(i) \
    ((i) == 0 ? MXC_UART0 : (i) == 1 ? MXC_UART1 : (i) == 2 ? MXC_UART2 : 0)

#define MXC_UART_GET_IDX(p) \
    ((p) == MXC_UART0 ? 0 : (p) == MXC_UART1 ? 1 : (p) == MXC_UART2 ? 2 : -1)

/* ************************************************************************** */
/*                                                                        SPI */
#define MXC_SPI_INSTANCES (4)
#define MXC_SPI_SS_INSTANCES (4)
#define MXC_SPI_FIFO_DEPTH (32)

#define MXC_BASE_SPI0 ((uint32_t)0x40046000UL)
#define MXC_SPI0 ((mxc_spi_regs_t *)MXC_BASE_SPI0)
#define MXC_BASE_SPI1 ((uint32_t)0x40047000UL)
#define MXC_SPI1 ((mxc_spi_regs_t *)MXC_BASE_SPI1)
#define MXC_BASE_SPI2 ((uint32_t)0x40048000UL)
#define MXC_SPI2 ((mxc_spi_regs_t *)MXC_BASE_SPI2)
#define MXC_BASE_SPI3 ((uint32_t)0x400BE000UL)
#define MXC_SPI3 ((mxc_spi_regs_t *)MXC_BASE_SPI3)

#define MXC_SPI_GET_IDX(p) \
    ((p) == MXC_SPI0 ? 0 : (p) == MXC_SPI1 ? 1 : (p) == MXC_SPI2 ? 2 : (p) == MXC_SPI3 ? 3 : -1)

#define MXC_SPI_GET_BASE(i)     \
    ((i) == 0 ? MXC_BASE_SPI0 : \
     (i) == 1 ? MXC_BASE_SPI1 : \
     (i) == 2 ? MXC_BASE_SPI2 : \
     (i) == 3 ? MXC_BASE_SPI3 : \
                0)

#define MXC_SPI_GET_SPI(i) \
    ((i) == 0 ? MXC_SPI0 : (i) == 1 ? MXC_SPI1 : (i) == 2 ? MXC_SPI2 : (i) == 3 ? MXC_SPI3 : 0)

#define MXC_SPI_GET_IRQ(i)             \
    (IRQn_Type)((i) == 0 ? SPI0_IRQn : \
                (i) == 1 ? SPI1_IRQn : \
                (i) == 2 ? SPI2_IRQn : \
                (i) == 3 ? SPI3_IRQn : \
                           0)

/* ************************************************************************** */
/*                                                                        USB */
#define MXC_BASE_USBHS ((uint32_t)0x400B1000UL)
#define MXC_USBHS ((mxc_usbhs_regs_t *)MXC_BASE_USBHS)
#define MXC_USBHS_NUM_EP 12 /* HW must have at least EP 0 CONTROL + 11 IN/OUT */
#define MXC_USBHS_NUM_DMA 8 /* HW must have at least this many DMA channels */
#define MXC_USBHS_MAX_PACKET 512

/* ************************************************************************** */
/*                                                                       TRNG */
#define MXC_BASE_TRNG ((uint32_t)0x400B5000UL)
#define MXC_TRNG ((mxc_trng_regs_t *)MXC_BASE_TRNG)

/* ************************************************************************** */
/*                                                                       SDHC */
#define MXC_BASE_SDHC ((uint32_t)0x400B6000UL)
#define MXC_SDHC ((mxc_sdhc_regs_t *)MXC_BASE_SDHC)

/* ************************************************************************** */
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

/* ************************************************************************** */
/*                                                               Bit Banding  */

#define BITBAND(reg, bit)                                                               \
    ((0xf0000000 & (uint32_t)(reg)) + 0x2000000 + (((uint32_t)(reg)&0x0fffffff) << 5) + \
     ((bit) << 2))

#define MXC_CLRBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit) = 0)
#define MXC_SETBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit) = 1)
#define MXC_GETBIT(reg, bit) (*(volatile uint32_t *)BITBAND(reg, bit))

#define MXC_SETFIELD(reg, mask, setting) ((reg) = ((reg) & ~(mask)) | ((setting) & (mask)))

/* ************************************************************************** */
/*                                                                  SCB CPACR */

/* Note: Added by Maxim Integrated, as these are missing from CMSIS/Core/Include/core_cm4.h */
#define SCB_CPACR_CP10_Pos 20 /*!< SCB CPACR: Coprocessor 10 Position */
#define SCB_CPACR_CP10_Msk (0x3UL << SCB_CPACR_CP10_Pos) /*!< SCB CPACR: Coprocessor 10 Mask */
#define SCB_CPACR_CP11_Pos 22 /*!< SCB CPACR: Coprocessor 11 Position */
#define SCB_CPACR_CP11_Msk (0x3UL << SCB_CPACR_CP11_Pos) /*!< SCB CPACR: Coprocessor 11 Mask */

#endif // LIBRARIES_CMSIS_DEVICE_MAXIM_MAX32650_INCLUDE_MAX32650_H_
