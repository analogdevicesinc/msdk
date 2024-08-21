/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef LIBRARIES_MISCDRIVERS_PMIC_MAX20303_H_
#define LIBRARIES_MISCDRIVERS_PMIC_MAX20303_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mxc_device.h"
#include "i2c_regs.h"

/**
 * @brief      Initialize I2C peripheral for use with MAX20303.
 * @retval     #E_NO_ERROR   Intialization successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_init(mxc_i2c_regs_t *i2c_inst);

/**
 * @brief      Set state of specified LED.
 * @param      led   LED index.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_led(int led, int on);

/**
 * @brief      Set state of PMIC red LED.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_led_red(int on);

/**
 * @brief      Set state of PMIC green LED.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_led_green(int on);

/**
 * @brief      Set state of PMIC blue LED.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_led_blue(int on);

/**
 * @brief      Set power state of microphone.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_mic_power(int on);

/**
 * @brief      Set power state of camera.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_camera_power(int on);

/**
 * @brief      Set power state of SD card.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_sd_power(int on);

/**
 * @brief      Set state of GPIO.
 * @param      gpio  GPIO index.
 * @param      on    1 for ON, 0 for OFF.
 * @retval     #E_NO_ERROR   If successful.
 * @retval     "Error Code"  @ref MXC_Error_Codes "Error Code" if unsuccessful.
 */
int max20303_gpio(int gpio, int on);

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_MISCDRIVERS_PMIC_MAX20303_H_
