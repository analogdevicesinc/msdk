/**
 * @file        main.c
 * @brief       I2C generic sensor Example
 * @details     This example uses the I2C Master to read/write from/to the I2C sensor.
 */

/*******************************************************************************
 * Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
#include "i2c_sensor.h"
#include "max31889_driver.h"

/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 ///< I2C instance
#define I2C_FREQ 100000 ///< I2C clock frequency

// *****************************************************************************
int main(void)
{
    int error = E_NO_ERROR;
    float temperature;

    printf("\n****************** I2C SENSOR DEMO *******************\n");

    error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %i\n", error);
        return error;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    mxc_i2c_sensor_driver_t MAX31889 = MAX31889_Open();

    MAX31889.init(I2C_MASTER, MAX31889_I2C_SLAVE_ADDR0); // init the sensor

    while (1) {
        error = MAX31889.read(&temperature);
        if (error != E_NO_ERROR) {
            printf("\nSensor read error: %i", error);
        } else {
            printf("\n-->Temperature: %02f %cC", temperature, 176);
        }
        // Wait for 1s
        MXC_Delay(MXC_DELAY_SEC(1));
    }

    return E_NO_ERROR;
}
