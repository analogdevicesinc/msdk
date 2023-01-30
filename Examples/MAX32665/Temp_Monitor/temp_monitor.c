/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
*******************************************************************************
*/
#include <stdbool.h>
#include <stdio.h>
#include "flc.h"
#include "i2c.h"
#include "icc.h"
#include "led.h"
#include "lp.h"
#include "max31889_driver.h"
#include "mxc_device.h"
#include "rtc.h"
#include "temp_monitor.h"

/***** Definitions *****/
/***** Modifiable defines *****/
#define HI_TEMP_THRESHOLD 30
#define LO_TEMP_THRESHOLD 15
#define TEMP_CHECK_PERIOD 5
#define TR_STORAGE_PAGE 1
//NOTE: Ensure "TR_STORAGE_PAGE" is less than the total number of flash pages in the device.
//      If you select a higher value, consider setting LINKERFILE to $(TARGET_LC)_ram.ld in
//      project.mk otherwise application code may get overwritten.

/****** Do not Modify these Defines *****/
#define TR_STORAGE_BASE_ADDR \
    (MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE - (TR_STORAGE_PAGE * MXC_FLASH_PAGE_SIZE))

#define TEMP_SENS_I2C MXC_I2C1_BUS0
#define TEMP_SENS_FREQ 100000

#define MSEC_TO_RSSA(msec) ((msec * 4096) / 1000)
#define WARN_LIGHT_PERIOD (0xFFFFFFFF - MSEC_TO_RSSA(250))

#define TEMPERATURE_MASK 0x0000FFFF
#define TIMESTAMP_MASK 0xFFFF0000
#define TIMESTAMP_OFFSET 16

/***** Global Variables *****/
static max31889_driver_t g_temp_sensor;
static bool g_temp_warning = false;
static uint32_t g_temp_readings[4];
static uint32_t g_num_readings = 0;

/******************************************************************/
/************************ Private Functions ***********************/
/******************************************************************/

/***** Flash Functions *****/
static int init_flash(void)
{
    int err;

    // Initialize flash controller
    err = MXC_FLC_Init();

    // Clear flash page used to store temperature readings
    err += MXC_FLC_PageErase(TR_STORAGE_BASE_ADDR);

    return err;
}

static void store_temp_readings(void)
{
    // Calculate flash address to store temperature readings at
    int flash_addr = TR_STORAGE_BASE_ADDR + (g_num_readings - 4) * sizeof(g_temp_readings[0]);

    // Write last 4 temp readings to flash
    MXC_ICC_Disable();
    if (MXC_FLC_Write128(flash_addr, g_temp_readings) != E_NO_ERROR) {
        //Failed to write to flash, decrement num_readings to reflect fact
        //that the last 4 readings taken are not stored in flash
        g_num_readings -= 4;
    }
    MXC_ICC_Enable();
}

/***** Temperature Sensor Functions *****/
static int init_temp_sensor(void)
{
    int err;

    // Initialize I2C connected to temperature sensor
    if ((err = MXC_I2C_Init(TEMP_SENS_I2C, 1, 0)) != E_NO_ERROR) {
        return err;
    }
    MXC_I2C_SetFrequency(TEMP_SENS_I2C, TEMP_SENS_FREQ);

    // Get MAX31889 function pointers
    g_temp_sensor = MAX31889_Open();

    // Initialize MAX31889 temperature sensor
    return g_temp_sensor.init(TEMP_SENS_I2C, MAX31889_I2C_SLAVE_ADDR0);
}

static void record_current_temp(uint32_t current_time)
{
    uint32_t temp_reading;
    float temp_reading_float;

    // Get current temperature
    if (g_temp_sensor.read((void *)&temp_reading_float) != E_NO_ERROR) {
        return;
    }

    // Record current temperature with timestamp
    temp_reading = (uint32_t)temp_reading_float;
    g_temp_readings[(g_num_readings % 4)] = (current_time << TIMESTAMP_OFFSET) | temp_reading;

    // Increment readings counter (reset to 0 if flash page has been filled)
    g_num_readings = (g_num_readings + 1) % (MXC_FLASH_PAGE_SIZE / sizeof(g_temp_readings[0]));

    // Store temperatures to flash if 4 readings have been taken
    // Note: we store them every 4 readings so we are able to do a 128-bit write
    if (g_num_readings % 4 == 0) {
        store_temp_readings();
    }

    // Check if temperature has exceeded a threshold
    if (temp_reading > HI_TEMP_THRESHOLD) {
        LED_Off(1);
        g_temp_warning = true;
        printf("Temperature exceeded upper threshold!\n");
    } else if (temp_reading < LO_TEMP_THRESHOLD) {
        LED_Off(1);
        g_temp_warning = true;
        printf("Temperature exceeded lower threshold!\n");
    } else {
        LED_Toggle(1); //Heartbeat
        g_temp_warning = false;
    }

    return;
}

/***** RTC Functions *****/
static int start_rtc(void)
{
    int err;

    // Initialize RTC
    if ((err = MXC_RTC_Init(0, 0)) != E_NO_ERROR) {
        return err;
    }

    // Setup TOD alarm
    MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG); //TOD alarm interrupt most be disabled to set period
    if ((err = MXC_RTC_SetTimeofdayAlarm(TEMP_CHECK_PERIOD)) != E_NO_ERROR) {
        return err;
    }
    MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG);

    // Enable RTC Wakeup
    MXC_LP_EnableRTCAlarmWakeup();
    NVIC_EnableIRQ(RTC_IRQn);

    // Start RTC
    return MXC_RTC_Start();
}

static void enable_warning_light(void)
{
    // Set subsecond alarm period and enable interrupt
    MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT); //SSEC alarm must be disabled to set period
    MXC_RTC_SetSubsecondAlarm(WARN_LIGHT_PERIOD);
    MXC_RTC_EnableInt(MXC_RTC_INT_EN_SHORT);
    return;
}

static void disable_warning_light(void)
{
    // Disable SSEC alarm
    MXC_RTC_DisableInt(MXC_RTC_INT_EN_SHORT);

    LED_Off(0); //Make sure warning light is off
    return;
}

/******************************************************************/
/************************ Public Functions ************************/
/******************************************************************/

/***** Initialization Function *****/
int temp_monitor_init(void)
{
    int err;

    if ((err = init_flash()) != E_NO_ERROR) { //Init flash space used to store temp readings
        return err;
    } else if ((err = init_temp_sensor()) != E_NO_ERROR) { //Init temperature sensor (and I2C)
        return err;
    } else if ((err = start_rtc()) != E_NO_ERROR) { //Init and start RTC, and set up TOD alarm
        return err;
    }

    return E_NO_ERROR;
}

/***** Push Button ISR *****/
void temp_monitor_print_temps(void)
{
    int addr_offset;
    int num_temp;
    uint32_t temps[12]; //Read a maximum of 12 temperatures
    uint16_t time, temp;

    // Calculate the number of temperature readings currently stored in flash
    num_temp = g_num_readings - (g_num_readings % 4);

    // Determine how many temperatures to read (limit 12) and where in flash they're located
    if (num_temp == 0) {
        printf("\nNo temperatures recorded, try reading again later.\n");
        return;
    } else if (num_temp < 12) {
        addr_offset = 0;
    } else {
        addr_offset = (num_temp - 12) * sizeof(g_temp_readings[0]);
        num_temp = 12;
    }

    // Read all temperature readings stored in flash
    MXC_FLC_Read(TR_STORAGE_BASE_ADDR + addr_offset, temps, sizeof(temps));

    // Print out each temperature reading with it's timestamp
    printf("\nLast %d temperature Readings:\n", num_temp);
    for (int i = 0; i < num_temp; i++) {
        time = (temps[i] & TIMESTAMP_MASK) >> TIMESTAMP_OFFSET;
        temp = temps[i] & TEMPERATURE_MASK;
        printf("%ds - Temp: %dC\n", time, temp);
    }
}

/***** RTC Alarm ISRs *****/
void temp_monitor_check_temp(void)
{
    uint32_t current_sec, dummy;

    // Get current time
    while (MXC_RTC_GetTime(&current_sec, &dummy) == E_BUSY) {}

    // Set next TOD alarm TEMP_CHECK_PERIOD seconds into the future
    MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG);
    while (MXC_RTC_SetTimeofdayAlarm(current_sec + TEMP_CHECK_PERIOD) == E_BUSY) {}
    MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG);

    // Take temperature readinf
    record_current_temp(current_sec);

    // Enable or disable warning light if necessary
    if (g_temp_warning && !(MXC_RTC->ctrl & MXC_RTC_INT_EN_SHORT)) {
        //Temp warning active and warning light inactive --> enable
        enable_warning_light();
    } else if (!g_temp_warning && (MXC_RTC->ctrl & MXC_RTC_INT_EN_SHORT)) {
        //Temp warning inactive and warning light active --> disable
        disable_warning_light();
    }
}

void temp_monitor_flash_warning_light(void)
{
    // Toggle warning light
    LED_Toggle(0);
}
