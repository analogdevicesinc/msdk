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

#ifndef EXAMPLES_MAX32690_TEMP_MONITOR_TEMP_MONITOR_H_
#define EXAMPLES_MAX32690_TEMP_MONITOR_TEMP_MONITOR_H_

/*
 * @brief Initializes the components of the temperature monitor (Flash, RTC, I2C, and MAX31889).
 */
int temp_monitor_init(void);

/*
 * @brief Reads the latest temperature readings from flash and prints them to the terminal.
 */
void temp_monitor_print_temps(void);

/*
 * @brief Checks the current air temperature. This function will activate or deactivate the warning
 * 		  light if necessary, and will also print a temperature warning message if necessary.
 */
void temp_monitor_check_temp(void);

/*
 * @brief Toggles warning light. 
 */
void temp_monitor_flash_warning_light(void);

#endif // EXAMPLES_MAX32690_TEMP_MONITOR_TEMP_MONITOR_H_
