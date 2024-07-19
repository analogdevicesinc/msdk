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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "max32655.h"
#include "gcr_regs.h"
#include "icc.h"
#include "mxc_sys.h"

#define MXC_NBBFC_REG4 *((volatile uint32_t *)(0x40000810))

uint32_t SystemCoreClock = HIRC_FREQ;
static volatile int intContext;

extern uint32_t *__isr_vector;

/*
The libc implementation from GCC 11+ depends on _getpid and _kill in some places.
There is no concept of processes/PIDs in the baremetal PeriphDrivers, therefore
we implement stub functions that return an error code to resolve linker warnings.
*/
__weak int _getpid(void)
{
    return E_NOT_SUPPORTED;
}

__weak int _kill(void)
{
    return E_NOT_SUPPORTED;
}

void illegal_ISNHandler(void);

void __attribute__((weak)) PF_IRQHandler(void) {}
void __attribute__((weak)) WDT0_IRQHandler(void) {}
void __attribute__((weak)) GPIOWAKE_IRQHandler(void) {}
void __attribute__((weak)) RTC_IRQHandler(void) {}
void __attribute__((weak)) TMR0_IRQHandler(void) {}
void __attribute__((weak)) TMR1_IRQHandler(void) {}
void __attribute__((weak)) TMR2_IRQHandler(void) {}
void __attribute__((weak)) TMR3_IRQHandler(void) {}
void __attribute__((weak)) TMR4_IRQHandler(void) {}
void __attribute__((weak)) TMR5_IRQHandler(void) {}
void __attribute__((weak)) I2C0_IRQHandler(void) {}
void __attribute__((weak)) UART0_IRQHandler(void) {}
void __attribute__((weak)) CM4_IRQHandler(void) {}
void __attribute__((weak)) I2C1_IRQHandler(void) {}
void __attribute__((weak)) UART1_IRQHandler(void) {}
void __attribute__((weak)) UART2_IRQHandler(void) {}
void __attribute__((weak)) I2C2_IRQHandler(void) {}
void __attribute__((weak)) UART3_IRQHandler(void) {}
void __attribute__((weak)) SPI1_IRQHandler(void) {}
void __attribute__((weak)) WUT0_IRQHandler(void) {}
void __attribute__((weak)) FLC0_IRQHandler(void) {}
void __attribute__((weak)) GPIO0_IRQHandler(void) {}
void __attribute__((weak)) GPIO1_IRQHandler(void) {}
void __attribute__((weak)) GPIO2_IRQHandler(void) {}
void __attribute__((weak)) DMA0_IRQHandler(void) {}
void __attribute__((weak)) DMA1_IRQHandler(void) {}
void __attribute__((weak)) DMA2_IRQHandler(void) {}
void __attribute__((weak)) DMA3_IRQHandler(void) {}
void __attribute__((weak)) BTLE_TX_DONE_IRQHandler(void) {}
void __attribute__((weak)) BTLE_RX_RCVD_IRQHandler(void) {}
void __attribute__((weak)) BTLE_RX_ENG_DET_IRQHandler(void) {}
void __attribute__((weak)) BTLE_SFD_DET_IRQHandler(void) {}
void __attribute__((weak)) BTLE_SFD_TO_IRQHandler(void) {}
void __attribute__((weak)) BTLE_GP_EVENT_IRQHandler(void) {}
void __attribute__((weak)) BTLE_CFO_IRQHandler(void) {}
void __attribute__((weak)) BTLE_SIG_DET_IRQHandler(void) {}
void __attribute__((weak)) BTLE_AGC_EVENT_IRQHandler(void) {}
void __attribute__((weak)) BTLE_RFFE_SPIM_IRQHandler(void) {}
void __attribute__((weak)) BTLE_TX_AES_IRQHandler(void) {}
void __attribute__((weak)) BTLE_RX_AES_IRQHandler(void) {}
void __attribute__((weak)) BTLE_INV_APB_ADDR_IRQHandler(void) {}
void __attribute__((weak)) BTLE_IQ_DATA_VALID_IRQHandler(void) {}
void __attribute__((weak)) AES_IRQHandler(void) {}
void __attribute__((weak)) TRNG_IRQHandler(void) {}
void __attribute__((weak)) WDT1_IRQHandler(void) {}
void __attribute__((weak)) DVS_IRQHandler(void) {}
void __attribute__((weak)) SIMO_IRQHandler(void) {}
void __attribute__((weak)) WUT1_IRQHandler(void) {}
void __attribute__((weak)) PT_IRQHandler(void) {}
void __attribute__((weak)) ADC_IRQHandler(void) {}
void __attribute__((weak)) OWM_IRQHandler(void) {}
void __attribute__((weak)) I2S_IRQHandler(void) {}
void __attribute__((weak)) CNN_FIFO_IRQHandler(void) {}
void __attribute__((weak)) CNN_IRQHandler(void) {}
void __attribute__((weak)) RSV58_IRQHandler(void) {}
void __attribute__((weak)) PCIF_IRQHandler(void) {}

void __attribute__((interrupt("machine"))) PF_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(PF_IRQn);
    NVIC_ClearPendingIRQ(PF_IRQn);
    PF_IRQHandler();
    NVIC_EnableIRQ(PF_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) WDT0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(WDT0_IRQn);
    NVIC_ClearPendingIRQ(WDT0_IRQn);
    WDT0_IRQHandler();
    NVIC_EnableIRQ(WDT0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) GPIOWAKE_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(GPIOWAKE_IRQn);
    NVIC_ClearPendingIRQ(GPIOWAKE_IRQn);
    GPIOWAKE_IRQHandler();
    NVIC_EnableIRQ(GPIOWAKE_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) RTC_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    RTC_IRQHandler();
    NVIC_EnableIRQ(RTC_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR0_IRQn);
    NVIC_ClearPendingIRQ(TMR0_IRQn);
    TMR0_IRQHandler();
    NVIC_EnableIRQ(TMR0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR1_IRQn);
    NVIC_ClearPendingIRQ(TMR1_IRQn);
    TMR1_IRQHandler();
    NVIC_EnableIRQ(TMR1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR2_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR2_IRQn);
    NVIC_ClearPendingIRQ(TMR2_IRQn);
    TMR2_IRQHandler();
    NVIC_EnableIRQ(TMR2_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR3_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR3_IRQn);
    NVIC_ClearPendingIRQ(TMR3_IRQn);
    TMR3_IRQHandler();
    NVIC_EnableIRQ(TMR3_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR4_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR4_IRQn);
    NVIC_ClearPendingIRQ(TMR4_IRQn);
    TMR4_IRQHandler();
    NVIC_EnableIRQ(TMR4_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TMR5_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TMR5_IRQn);
    NVIC_ClearPendingIRQ(TMR5_IRQn);
    TMR5_IRQHandler();
    NVIC_EnableIRQ(TMR5_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) I2C0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(I2C0_IRQn);
    NVIC_ClearPendingIRQ(I2C0_IRQn);
    I2C0_IRQHandler();
    NVIC_EnableIRQ(I2C0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) UART0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(UART0_IRQn);
    NVIC_ClearPendingIRQ(UART0_IRQn);
    UART0_IRQHandler();
    NVIC_EnableIRQ(UART0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) CM4_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(CM4_IRQn);
    NVIC_ClearPendingIRQ(CM4_IRQn);
    CM4_IRQHandler();
    NVIC_EnableIRQ(CM4_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) I2C1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(I2C1_IRQn);
    NVIC_ClearPendingIRQ(I2C1_IRQn);
    I2C1_IRQHandler();
    NVIC_EnableIRQ(I2C1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) UART1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(UART1_IRQn);
    NVIC_ClearPendingIRQ(UART1_IRQn);
    UART1_IRQHandler();
    NVIC_EnableIRQ(UART1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) UART2_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(UART2_IRQn);
    NVIC_ClearPendingIRQ(UART2_IRQn);
    UART2_IRQHandler();
    NVIC_EnableIRQ(UART2_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) I2C2_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(I2C2_IRQn);
    NVIC_ClearPendingIRQ(I2C2_IRQn);
    I2C2_IRQHandler();
    NVIC_EnableIRQ(I2C2_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) UART3_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(UART3_IRQn);
    NVIC_ClearPendingIRQ(UART3_IRQn);
    UART3_IRQHandler();
    NVIC_EnableIRQ(UART3_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) SPI1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(SPI1_IRQn);
    NVIC_ClearPendingIRQ(SPI1_IRQn);
    SPI1_IRQHandler();
    NVIC_EnableIRQ(SPI1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) WUT0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(WUT0_IRQn);
    NVIC_ClearPendingIRQ(WUT0_IRQn);
    WUT0_IRQHandler();
    NVIC_EnableIRQ(WUT0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) FLC0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(FLC0_IRQn);
    NVIC_ClearPendingIRQ(FLC0_IRQn);
    FLC0_IRQHandler();
    NVIC_EnableIRQ(FLC0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) GPIO0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(GPIO0_IRQn);
    NVIC_ClearPendingIRQ(GPIO0_IRQn);
    GPIO0_IRQHandler();
    NVIC_EnableIRQ(GPIO0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) GPIO1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(GPIO1_IRQn);
    NVIC_ClearPendingIRQ(GPIO1_IRQn);
    GPIO1_IRQHandler();
    NVIC_EnableIRQ(GPIO1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) GPIO2_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(GPIO2_IRQn);
    NVIC_ClearPendingIRQ(GPIO2_IRQn);
    GPIO2_IRQHandler();
    NVIC_EnableIRQ(GPIO2_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) DMA0_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_ClearPendingIRQ(DMA0_IRQn);
    DMA0_IRQHandler();
    NVIC_EnableIRQ(DMA0_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) DMA1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(DMA1_IRQn);
    NVIC_ClearPendingIRQ(DMA1_IRQn);
    DMA1_IRQHandler();
    NVIC_EnableIRQ(DMA1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) DMA2_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(DMA2_IRQn);
    NVIC_ClearPendingIRQ(DMA2_IRQn);
    DMA2_IRQHandler();
    NVIC_EnableIRQ(DMA2_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) DMA3_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(DMA3_IRQn);
    NVIC_ClearPendingIRQ(DMA3_IRQn);
    DMA3_IRQHandler();
    NVIC_EnableIRQ(DMA3_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_TX_DONE_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_TX_DONE_IRQn);
    NVIC_ClearPendingIRQ(BTLE_TX_DONE_IRQn);
    BTLE_TX_DONE_IRQHandler();
    NVIC_EnableIRQ(BTLE_TX_DONE_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_RX_RCVD_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_RX_RCVD_IRQn);
    NVIC_ClearPendingIRQ(BTLE_RX_RCVD_IRQn);
    BTLE_RX_RCVD_IRQHandler();
    NVIC_EnableIRQ(BTLE_RX_RCVD_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_RX_ENG_DET_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_RX_ENG_DET_IRQn);
    NVIC_ClearPendingIRQ(BTLE_RX_ENG_DET_IRQn);
    BTLE_RX_ENG_DET_IRQHandler();
    NVIC_EnableIRQ(BTLE_RX_ENG_DET_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_SFD_DET_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_SFD_DET_IRQn);
    NVIC_ClearPendingIRQ(BTLE_SFD_DET_IRQn);
    BTLE_SFD_DET_IRQHandler();
    NVIC_EnableIRQ(BTLE_SFD_DET_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_SFD_TO_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_SFD_TO_IRQn);
    NVIC_ClearPendingIRQ(BTLE_SFD_TO_IRQn);
    BTLE_SFD_TO_IRQHandler();
    NVIC_EnableIRQ(BTLE_SFD_TO_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_GP_EVENT_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_GP_EVENT_IRQn);
    NVIC_ClearPendingIRQ(BTLE_GP_EVENT_IRQn);
    BTLE_GP_EVENT_IRQHandler();
    NVIC_EnableIRQ(BTLE_GP_EVENT_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_CFO_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_CFO_IRQn);
    NVIC_ClearPendingIRQ(BTLE_CFO_IRQn);
    BTLE_CFO_IRQHandler();
    NVIC_EnableIRQ(BTLE_CFO_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_SIG_DET_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_SIG_DET_IRQn);
    NVIC_ClearPendingIRQ(BTLE_SIG_DET_IRQn);
    BTLE_SIG_DET_IRQHandler();
    NVIC_EnableIRQ(BTLE_SIG_DET_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_AGC_EVENT_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_AGC_EVENT_IRQn);
    NVIC_ClearPendingIRQ(BTLE_AGC_EVENT_IRQn);
    BTLE_AGC_EVENT_IRQHandler();
    NVIC_EnableIRQ(BTLE_AGC_EVENT_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_RFFE_SPIM_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_RFFE_SPIM_IRQn);
    NVIC_ClearPendingIRQ(BTLE_RFFE_SPIM_IRQn);
    BTLE_RFFE_SPIM_IRQHandler();
    NVIC_EnableIRQ(BTLE_RFFE_SPIM_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_TX_AES_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_TX_AES_IRQn);
    NVIC_ClearPendingIRQ(BTLE_TX_AES_IRQn);
    BTLE_TX_AES_IRQHandler();
    NVIC_EnableIRQ(BTLE_TX_AES_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_RX_AES_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_RX_AES_IRQn);
    NVIC_ClearPendingIRQ(BTLE_RX_AES_IRQn);
    BTLE_RX_AES_IRQHandler();
    NVIC_EnableIRQ(BTLE_RX_AES_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_INV_APB_ADDR_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_INV_APB_ADDR_IRQn);
    NVIC_ClearPendingIRQ(BTLE_INV_APB_ADDR_IRQn);
    BTLE_INV_APB_ADDR_IRQHandler();
    NVIC_EnableIRQ(BTLE_INV_APB_ADDR_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) BTLE_IQ_DATA_VALID_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(BTLE_IQ_DATA_VALID_IRQn);
    NVIC_ClearPendingIRQ(BTLE_IQ_DATA_VALID_IRQn);
    BTLE_IQ_DATA_VALID_IRQHandler();
    NVIC_EnableIRQ(BTLE_IQ_DATA_VALID_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) AES_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(AES_IRQn);
    NVIC_ClearPendingIRQ(AES_IRQn);
    AES_IRQHandler();
    NVIC_EnableIRQ(AES_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) TRNG_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(TRNG_IRQn);
    NVIC_ClearPendingIRQ(TRNG_IRQn);
    TRNG_IRQHandler();
    NVIC_EnableIRQ(TRNG_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) WDT1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(WDT1_IRQn);
    NVIC_ClearPendingIRQ(WDT1_IRQn);
    WDT1_IRQHandler();
    NVIC_EnableIRQ(WDT1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) DVS_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(DVS_IRQn);
    NVIC_ClearPendingIRQ(DVS_IRQn);
    DVS_IRQHandler();
    NVIC_EnableIRQ(DVS_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) SIMO_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(SIMO_IRQn);
    NVIC_ClearPendingIRQ(SIMO_IRQn);
    SIMO_IRQHandler();
    NVIC_EnableIRQ(SIMO_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) WUT1_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(WUT1_IRQn);
    NVIC_ClearPendingIRQ(WUT1_IRQn);
    WUT1_IRQHandler();
    NVIC_EnableIRQ(WUT1_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) PT_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(PT_IRQn);
    NVIC_ClearPendingIRQ(PT_IRQn);
    PT_IRQHandler();
    NVIC_EnableIRQ(PT_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) ADC_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_ClearPendingIRQ(ADC_IRQn);
    ADC_IRQHandler();
    NVIC_EnableIRQ(ADC_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) OWM_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(OWM_IRQn);
    NVIC_ClearPendingIRQ(OWM_IRQn);
    OWM_IRQHandler();
    NVIC_EnableIRQ(OWM_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) I2S_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(I2S_IRQn);
    NVIC_ClearPendingIRQ(I2S_IRQn);
    I2S_IRQHandler();
    NVIC_EnableIRQ(I2S_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) CNN_FIFO_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(CNN_FIFO_IRQn);
    NVIC_ClearPendingIRQ(CNN_FIFO_IRQn);
    CNN_FIFO_IRQHandler();
    NVIC_EnableIRQ(CNN_FIFO_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) CNN_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(CNN_IRQn);
    NVIC_ClearPendingIRQ(CNN_IRQn);
    CNN_IRQHandler();
    NVIC_EnableIRQ(CNN_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) RSV58_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(RSV58_IRQn);
    NVIC_ClearPendingIRQ(RSV58_IRQn);
    RSV58_IRQHandler();
    NVIC_EnableIRQ(RSV58_IRQn);
    intContext = 0;
}
void __attribute__((interrupt("machine"))) PCIF_IRQHandlerWrap(void)
{
    intContext = 1;

    NVIC_DisableIRQ(PCIF_IRQn);
    NVIC_ClearPendingIRQ(PCIF_IRQn);
    PCIF_IRQHandler();
    NVIC_EnableIRQ(PCIF_IRQn);
    intContext = 0;
}

void __attribute__((interrupt("machine"))) illegal_insn_handler(void)
{
    volatile uint32_t mstatus = get_mstatus();
    volatile uint32_t mtvec = get_mtvec();
    volatile uint32_t mcause = get_mcause();
    volatile uint32_t mepc = get_mepc();
    volatile uint32_t uepc = get_uepc();

    /* Use debugger to read the status register values */
    (void)mstatus;
    (void)mtvec;
    (void)mcause;
    (void)mepc;
    (void)uepc;
    while (1) {}
}

void __enable_irq(void)
{
    // Interrupts will be automatically re-enabled when leaving interrupt context
    if (!intContext) {
        // Set the MIE bit if we're outside the interrupt context
        __asm volatile("csrw mstatus, 0x8");
    }
}

void SystemCoreClockUpdate(void)
{
    uint32_t base_freq, div, clk_src;

    // Get the clock source and frequency
    clk_src = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_SEL);
    switch (clk_src) {
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_EXTCLK:
        base_freq = EXTCLK_FREQ;
        break;
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERFO:
        base_freq = ERFO_FREQ;
        break;
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_INRO:
        base_freq = INRO_FREQ;
        break;
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IPO:
        base_freq = IPO_FREQ;
        break;
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_IBRO:
        base_freq = IBRO_FREQ;
        break;
    case MXC_S_GCR_CLKCTRL_SYSCLK_SEL_ERTCO:
        base_freq = ERTCO_FREQ;
        break;
    default:
        // Codes 001 and 111 are reserved.
        // This code should never execute, however, initialize to safe value.
        base_freq = 60000000;
        break;
    }

    // Get the clock divider
    div = (MXC_GCR->clkctrl & MXC_F_GCR_CLKCTRL_SYSCLK_DIV) >> MXC_F_GCR_CLKCTRL_SYSCLK_DIV_POS;
    SystemCoreClock = base_freq >> div;
}
/* This function is called before C runtime initialization and can be
 * implemented by the application for early initializations. If a value other
 * than '0' is returned, the C runtime initialization will be skipped.
 *
 * You may over-ride this function in your program by defining a custom
 *  PreInit(), but care should be taken to reproduce the initilization steps
 *  or a non-functional system may result.
 */
__weak int PreInit(void)
{
    /* Do nothing */
    return 0;
}

/* This function can be implemented by the application to initialize the board */
__weak int Board_Init(void)
{
    /* Do nothing */
    return 0;
}

__weak void PalSysInit(void) {}

/* This function is called just before control is transferred to main().
 *
 * You may over-ride this function in your program by defining a custom
 *  SystemInit(), but care should be taken to reproduce the initialization
 *  steps or a non-functional system may result.
 */
__weak void SystemInit(void)
{
    SystemCoreClockUpdate();

    /* Set the interrupt vector base address */
    MXC_NBBFC_REG4 = (uint32_t)&__isr_vector;

    intContext = 0;

    MXC_ICC_Enable(MXC_ICC1);

    __enable_irq();

    Board_Init();

    PalSysInit();
}
