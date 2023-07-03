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

#ifndef EXAMPLES_MAX32672_TEMP_MONITOR_TEMP_MONITOR_H_
#define EXAMPLES_MAX32672_TEMP_MONITOR_TEMP_MONITOR_H_

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

#endif // EXAMPLES_MAX32672_TEMP_MONITOR_TEMP_MONITOR_H_
