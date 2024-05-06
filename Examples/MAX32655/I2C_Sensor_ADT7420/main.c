/**
 * @file        main.c
 * @brief       I2C generic sensor Example
 * @details     This example uses the I2C Master to read/write from/to the I2C sensor.
 */

/******************************************************************************
 *
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

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "i2c.h"
#include "mxc_device.h"
#include "adt7420_driver.h"
/***** Definitions *****/
#define I2C_MASTER MXC_I2C1 ///< I2C instance
#define I2C_FREQ 100000 ///< I2C clock frequency

// *****************************************************************************
int main(void)
{
    int error = E_NO_ERROR;
    float temperature;

    printf("\n****************** I2C ADT7420 TEMPERATURE SENSOR DEMO *******************\n");
    printf(
        "\n***Make sure that the SCL and SDA pins of the EVAL-ADT7420MBZ board are connected to P0.16 and P0.17 on the MAX32655EVKIT***\n");
    error = MXC_I2C_Init(I2C_MASTER, 1, 0);
    if (error != E_NO_ERROR) {
        printf("I2C master configure failed with error %i\n", error);
        return error;
    }

    MXC_I2C_SetFrequency(I2C_MASTER, I2C_FREQ);

    adt7420_driver_t ADT7420 = ADT7420_Open();
    ADT7420.init(I2C_MASTER, ADT7420_I2C_SLAVE_ADDR1); //  the ADT7420

    while (1) {
        error = ADT7420.read(&temperature);
        if (error != E_NO_ERROR) {
            printf("\nSensor read error: %i", error);
        } else {
            printf("\n-->Temperature: %02f %cC", (double)temperature, 176);
        }
        // Wait for 1s
        MXC_Delay(MXC_DELAY_SEC(1));
    }

    return E_NO_ERROR;
}
