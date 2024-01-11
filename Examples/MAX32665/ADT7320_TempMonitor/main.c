/**
 * @file    main.c
 * @brief   ADT7320 Temp Sensor Demo
 * @details Reading Temperature via ADT7320
 */

/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "mxc_delay.h"
#include "mxc_pins.h"
#include "nvic_table.h"
#include "uart.h"
#include "spi.h"
#include "led.h"
#include "adt7320_driver.h"

/***** Preprocessors *****/

/***** Definitions *****/
#define SPI_SPEED 10000 // Bit Rate
#if defined(BOARD_FTHR)
#define SPI MXC_SPI1
#else
#define SPI MXC_SPI0
#endif

/***** Globals *****/

/***** Functions *****/

int main(void)
{
    int retVal = E_NO_ERROR;
    float temp_c = 0.0f;
    float overTempLimit = 0.0f;
    uint8_t hysValue = 0.0f;
    uint8_t statusReg = 0x00;
    uint8_t overTemp = 0x00;

    printf("\n***************** ADT7320 Temperature Sensor Example *****************\n");
    printf("This example reads the temperature data from ADT7320 temperature sensor\n");
    printf("via SPI and write it to the terminal. \n");
    printf("You will need to connect the ADT7320 sensor to the ");
#if defined(BOARD_FTHR)
    printf("SPI1 pins.\n\n");
#else
    printf("SPI0 pins.\n\n");
#endif

    printf("\n");

#if defined(BOARD_FTHR)
    MXC_GPIO_Config(&gpio_cfg_spi1_ss0);
#else
    MXC_GPIO_Config(&gpio_cfg_spi0_ss0a);
#endif

    retVal = MXC_SPI_Init(SPI, 1, 0, 1, 0, SPI_SPEED, MAP_A);
    MXC_SPI_SetWidth(SPI, SPI_WIDTH_STANDARD);
    MXC_SPI_SetMode(SPI, SPI_MODE_3);
    if (retVal != E_NO_ERROR) {
        printf("\nSPI INITIALIZATION ERROR\n");
        return retVal;
    }

    MXC_Delay(MXC_DELAY_MSEC(500));

    adt7320_init(SPI);
    adt7320_SetADCResolution(ADC_RES_16);
    adt7320_SetIntCtMode(COMPARATOR_MODE);
    adt7320_SetOverTemperatureLimit(29.3);
    adt7320_ReadOverTemperatureLimit(&overTempLimit);
    printf("\nOver Temperature Limit Set to %.3f", (double)overTempLimit);
    adt7320_SetHysteresisValue(1);
    adt7320_ReadTHysteresisSetPoint(&hysValue);
    printf("\nHysteresis Set to %d", hysValue);

    printf("\n");
    while (1) {
        adt7320_ReadTemperature(&temp_c);
        adt7320_ReadStatusRegister(&statusReg);

        overTemp = (statusReg >> 5) & 0x01;

        printf("\033[A");
        printf("\nTemperature Value = %.3f, Over Temp. Flag: %d", (double)temp_c, overTemp);

        MXC_Delay(MXC_DELAY_MSEC(500));
    }
}
