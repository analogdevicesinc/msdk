/*******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc. (now owned by Analog
 * Devices, Inc.), Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary and confidential to Analog Devices, Inc. and its licensors.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2017-08-10 11:01:15 -0500 (Thu, 10 Aug 2017) $
 * $Revision: 29282 $
 *
 ******************************************************************************/

#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "mxc_assert.h"
#include "board.h"
#include "uart.h"
#include "gpio.h"
#include "mxc_pins.h"
#include "led.h"
#include "pb.h"
#include "spixf.h"
#include "i2c.h"
#include "Ext_Flash.h"

/***** Global Variables *****/
mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
extern uint32_t SystemCoreClock;

const mxc_gpio_cfg_t pb_pin[] = { { MXC_GPIO0, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_IN,
                                    MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO },
                                  { MXC_GPIO1, MXC_GPIO_PIN_29, MXC_GPIO_FUNC_IN,
                                    MXC_GPIO_PAD_PULL_UP, MXC_GPIO_VSSEL_VDDIO } };
const unsigned int num_pbs = (sizeof(pb_pin) / sizeof(mxc_gpio_cfg_t));

const mxc_gpio_cfg_t led_pin[] = { { MXC_GPIO1, MXC_GPIO_PIN_30, MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO },
                                   { MXC_GPIO1, MXC_GPIO_PIN_31, MXC_GPIO_FUNC_OUT,
                                     MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO } };
const unsigned int num_leds = (sizeof(led_pin) / sizeof(mxc_gpio_cfg_t));
// clang-format on


/******************************************************************************/
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int ext_flash_board_init(void)
{
    int err;
    err = MXC_SPIXF_Init(SPIXFC_CMD_VAL, EXT_FLASH_SPIXFC_BAUD);
    if (err == E_NO_ERROR) {
        MXC_SPIXF_Enable();
    }
    return err;
}

/******************************************************************************/
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int ext_flash_board_read(uint8_t *read, unsigned len, unsigned deassert,
                                Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, NULL, read, (mxc_spixf_width_t)width, len, 0, 0, NULL };
    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int ext_flash_board_write(const uint8_t *write, unsigned len, unsigned deassert,
                                 Ext_Flash_DataLine_t width)
{
    mxc_spixf_req_t req = { deassert, 0, write, NULL, (mxc_spixf_width_t)width, len, 0, 0, NULL };
    if (MXC_SPIXF_Transaction(&req) != len) {
        return E_COMM_ERR;
    }
    return E_NO_ERROR;
}

/******************************************************************************/
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
static int ext_flash_clock(unsigned len, unsigned deassert)
{
    return MXC_SPIXF_Clocks(len, deassert);
}


/******************************************************************************/
void mxc_assert(const char *expr, const char *file, int line)
{
    printf("MXC_ASSERT %s #%d: (%s)\n", file, line, expr);

    while (1) {}
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button interrupts in
 *       case the user does not define this interrupt handler in their application.
 **/
__weak void GPIO0_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO0));
}

/******************************************************************************/
/** 
 * NOTE: This weak definition is included to support Push Button/Touchscreen interrupts
 *       in case the user does not define this interrupt handler in their application.
 **/
__weak void GPIO1_IRQHandler(void)
{
    MXC_GPIO_Handler(MXC_GPIO_GET_IDX(MXC_GPIO1));
}


// void TS_SPI_Init(void)
// {
//     mxc_spi_pins_t ts_pins = {
//         // CLK, MISO, MOSI enabled, SS IDx = 1
//         .clock = true, .ss0 = false, .ss1 = true,    .ss2 = false,
//         .miso = true,  .mosi = true, .sdio2 = false, .sdio3 = false,
//     };

//     MXC_SPI_Init(TS_SPI, true, false, 1, 0, TS_SPI_FREQ, ts_pins);
//     MXC_GPIO_SetVSSEL(MXC_GPIO0, MXC_GPIO_VSSEL_VDDIOH,
//                       MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_26);
//     MXC_SPI_SetFrameSize(TS_SPI, 8);
//     MXC_SPI_SetInterface(TS_SPI, MXC_SPI_INTERFACE_STANDARD);
// }

// void TS_SPI_Transmit(uint8_t datain, uint16_t *dataout)
// {
//     int i;
//     uint8_t rx[2] = { 0, 0 };
//     mxc_spi_req_t request;

//     request.spi = TS_SPI;
//     request.ssDeassert = 0;
//     request.txData = (uint8_t *)(&datain);
//     request.rxData = NULL;
//     request.txLen = 1;
//     request.rxLen = 0;
//     request.ssIdx = 1;

//     MXC_SPI_SetFrequency(TS_SPI, TS_SPI_FREQ);
//     MXC_SPI_SetFrameSize(TS_SPI, 8);

//     MXC_SPI_ControllerTransaction(&request);

//     // Wait to clear TS busy signal
//     for (i = 0; i < 100; i++) {
//         __asm volatile("nop\n");
//     }

//     request.ssDeassert = 1;
//     request.txData = NULL;
//     request.rxData = (uint8_t *)(rx);
//     request.txLen = 0;
//     request.rxLen = 2;

//     MXC_SPI_ControllerTransaction(&request);

//     if (dataout != NULL) {
//         *dataout = (rx[1] | (rx[0] << 8)) >> 4;
//     }
// }

/******************************************************************************/
int Board_Init(void)
{
    int err;
    // callback functions for external flash driver
    Ext_Flash_Config_t exf_cfg = { .init = ext_flash_board_init,
                                   .read = ext_flash_board_read,
                                   .write = ext_flash_board_write,
                                   .clock = ext_flash_clock };

    // configure callback functions for external flash driver
    if ((err = Ext_Flash_Configure(&exf_cfg)) != E_NO_ERROR) {
        return err;
    }

    if ((err = Console_Init()) < E_NO_ERROR) {
        return err;
    }

    if ((err = PB_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    if ((err = LED_Init()) != E_NO_ERROR) {
        MXC_ASSERT_FAIL();
        return err;
    }

    // /* TFT reset and backlight signal */
    // mxc_tft_spi_config tft_spi_config = {
    //     .regs = MXC_SPI1,
    //     .gpio = { MXC_GPIO0, MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_20,
    //               MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 },
    //     .freq = 12000000,
    //     .ss_idx = 0,
    // };

    // mxc_gpio_cfg_t tft_reset_pin = { MXC_GPIO3,         MXC_GPIO_PIN_0,        MXC_GPIO_FUNC_OUT,
    //                                  MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    // mxc_gpio_cfg_t tft_bl_pin = { MXC_GPIO0,         MXC_GPIO_PIN_27,       MXC_GPIO_FUNC_OUT,
    //                               MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };

    // /* Initialize TFT display */
    // MXC_TFT_PreInit(&tft_spi_config, &tft_reset_pin, &tft_bl_pin);

    // /* Enable Touchscreen */
    // mxc_ts_spi_config ts_spi_config = {
    //     .regs = MXC_SPI1,
    //     .gpio = { MXC_GPIO0, MXC_GPIO_PIN_21 | MXC_GPIO_PIN_22 | MXC_GPIO_PIN_23 | MXC_GPIO_PIN_26,
    //               MXC_GPIO_FUNC_ALT1, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 },
    //     .freq = 200000,
    //     .ss_idx = 1,
    // };

    // /* Touch screen controller interrupt signal */
    // mxc_gpio_cfg_t int_pin = { MXC_GPIO0,         MXC_GPIO_PIN_13,       MXC_GPIO_FUNC_IN,
    //                            MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    // /* Touch screen controller busy signal */
    // mxc_gpio_cfg_t busy_pin = { MXC_GPIO0,         MXC_GPIO_PIN_12,       MXC_GPIO_FUNC_IN,
    //                             MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH, MXC_GPIO_DRVSTR_0 };
    // /* Initialize Touch Screen controller */
    // MXC_TS_PreInit(&ts_spi_config, &int_pin, &busy_pin);

    return E_NO_ERROR;
}

/******************************************************************************/
int Console_Init(void)
{
    int err;

    if ((err = MXC_UART_Init(ConsoleUart, CONSOLE_BAUD, MXC_UART_APB_CLK)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}

int Console_Shutdown(void)
{
    int err;

    if ((err = MXC_UART_Shutdown(ConsoleUart)) != E_NO_ERROR) {
        return err;
    }

    return E_NO_ERROR;
}
/******************************************************************************/
#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
/**
 * @brief      Configurations for SPIXFM (XiP)
 */
void SPIXFM_Config()
{
    MXC_SPIXF_SetSPIFrequency(EXT_FLASH_SPIXFM_BAUD);
    MXC_SPIXF_SetMode(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetSSPolActiveLow();
    MXC_SPIXF_SetSSActiveTime(MXC_SPIXF_SYS_CLOCKS_2);
    MXC_SPIXF_SetSSInactiveTime(MXC_SPIXF_SYS_CLOCKS_9);

    MXC_SPIXF_SetBusIdle(SPIXFM_BUS_IDLE_VAL);

    MXC_SPIXF_SetCmdValue(EXT_FLASH_CMD_QREAD);
    MXC_SPIXF_SetModeData(MXC_SPIXF_MODE_0);
    MXC_SPIXF_SetCmdWidth(MXC_SPIXF_SINGLE_SDIO);
    MXC_SPIXF_SetAddrWidth(MXC_SPIXF_QUAD_SDIO);
    MXC_SPIXF_SetDataWidth(MXC_SPIXF_WIDTH_4);
    MXC_SPIXF_SetModeClk(EXT_FLASH_QREAD_DUMMY);

    MXC_SPIXF_Set3ByteAddr();
    MXC_SPIXF_SCKFeedbackEnable();
    MXC_SPIXF_SetSCKNonInverted();
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_Ext_Write(uint32_t address, uint32_t length, uint8_t *buffer)
{
    int err = E_NO_ERROR;
    // reset SPIXF peripheral (both SPIXFC and SPIXFM)
    MXC_GCR->rst1 |= MXC_F_GCR_RST1_SPIXIPM | MXC_F_GCR_RST1_SPIXIP;
    // initialize spixfc
    MXC_SPIXF_Init(SPIXFC_CMD_VAL, EXT_FLASH_SPIXFC_BAUD);
    // enable spixfc
    MXC_SPIXF_Enable();

    // write to external flash
    err = Ext_Flash_Program_Page(address, buffer, length, Ext_Flash_DataLine_Single);

    // disable spixfc
    MXC_SPIXF_Disable();
    // re-config SPIXF for XiP
    SPIXFM_Config();

    return err;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_Ext_Read(int address, uint8_t* buffer, int len)
{
    int err = E_NO_ERROR;
    // reset SPIXF peripheral (both SPIXFC and SPIXFM)
    MXC_GCR->rst1 |= MXC_F_GCR_RST1_SPIXIPM | MXC_F_GCR_RST1_SPIXIP;
    // initialize spixfc
    MXC_SPIXF_Init(SPIXFC_CMD_VAL, EXT_FLASH_SPIXFC_BAUD);
    // enable spixfc
    MXC_SPIXF_Enable();

    // read from external flash
    err = Ext_Flash_Read(address, buffer, len, Ext_Flash_DataLine_Single);

    // disable spixfc
    MXC_SPIXF_Disable();
    // re-config SPIXF for XiP
    SPIXFM_Config();

    return err;
}

#if defined(SPIXF_RAM) && IAR_PRAGMAS
#pragma section = ".spix_config"
#elif defined(SPIXF_RAM)
__attribute__((section(".spix_config")))
#endif
int MXC_Ext_SectorErase(int address)
{
    int err  = E_NO_ERROR;
    // reset SPIXF peripheral (both SPIXFC and SPIXFM)
    MXC_GCR->rst1 |= MXC_F_GCR_RST1_SPIXIPM | MXC_F_GCR_RST1_SPIXIP;
    // initialize spixfc
    MXC_SPIXF_Init(SPIXFC_CMD_VAL, EXT_FLASH_SPIXFC_BAUD);
    // enable spixfc
    MXC_SPIXF_Enable();

    // erase 4KB from flash which corresponds to a sector
    err = Ext_Flash_Erase(address, Ext_Flash_Erase_4K);

    // disable spixfc
    MXC_SPIXF_Disable();
    // re-config SPIXF for XiP
    SPIXFM_Config();

    return err;
}

/******************************************************************************/
__weak void NMI_Handler(void)
{
    __NOP();
}
