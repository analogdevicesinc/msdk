#include "temp_monitor.h"

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

/***** Definitions *****/
#define HI_TEMP_THRESHOLD     	30
#define LO_TEMP_THRESHOLD    	15
#define TEMP_CHECK_PERIOD 		5
#define TR_STORAGE_PAGE 		1
#define TR_STORAGE_BASE_ADDR 	(MXC_FLASH_MEM_BASE + MXC_FLASH_MEM_SIZE - \
								(TR_STORAGE_PAGE * MXC_FLASH_PAGE_SIZE))
#define TEMP_SENS_I2C 			MXC_I2C0

/***** Global Variables *****/
static max31889_driver_t temp_sensor;
static bool temp_warning = false;
static uint32_t temp_readings[4];
static uint32_t num_readings = 0;

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
	int flash_addr = TR_STORAGE_BASE_ADDR + (num_readings - 4) * 4;

	// Write last 4 temp readings to flash
	MXC_ICC_Disable();
	if(MXC_FLC_Write128(flash_addr, temp_readings) != E_NO_ERROR) {
		num_readings -= 4;
	}
	MXC_ICC_Enable();
}

/***** Temperature Sensor Functions *****/
static int init_temp_sensor(void)
{
	int err;

	// Initialize I2C connected to temperature sensor
	if((err = MXC_I2C_Init(TEMP_SENS_I2C, 1, 0)) != E_NO_ERROR) {
		return err;
	}
	MXC_I2C_SetFrequency(TEMP_SENS_I2C, 100000);

	// Get MAX31889 function pointers
	temp_sensor = MAX31889_Open(); 

	// Initialize MAX31889 temperature sensor
	if((err = temp_sensor.init(TEMP_SENS_I2C, MAX31889_I2C_SLAVE_ADDR0)) != E_NO_ERROR) {
		return err;
	}

	return E_NO_ERROR;
}

static void record_current_temp(uint32_t current_time)
{
	uint32_t temp_reading;
	float temp_reading_float;

	// Get current temperature
	if(temp_sensor.read((void*) &temp_reading_float) != E_NO_ERROR) {
		return;
	}

	// Record current temperature with timestamp
	temp_reading = (uint32_t) temp_reading_float;
	temp_readings[(num_readings % 4)] = (current_time << 16) | temp_reading;
	num_readings = (num_readings + 1) % (MXC_FLASH_PAGE_SIZE / sizeof(temp_readings[0]));

	// Store temperatures to flash if 4 readings have been taken
	// Note: we store them every 4 readings so we are able to do a 128-bit write 
	if(num_readings % 4 == 0) {
		store_temp_readings();
	}

	// Check if temperature has exceeded a threshold
	if(temp_reading > HI_TEMP_THRESHOLD) {
		LED_Off(1);
		temp_warning = true;
		printf("Temperature exceeded upper threshold!\n");
	} else if(temp_reading < LO_TEMP_THRESHOLD) {
		LED_Off(1);
		temp_warning = true;
		printf("Temperature exceeded lower threshold!\n");
	} else {
		LED_Toggle(1); //Heartbeat
		temp_warning = false;
	}

	return;
}

/***** RTC Functions *****/
static int start_rtc(void)
{
	int err;

	// Initialize RTC
	if((err = MXC_RTC_Init(0, 0)) != E_NO_ERROR) {
		return err;
	}

	// Setup TOD alarm
	MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG); //TOD alarm most be disabled to set period
	if((err = MXC_RTC_SetTimeofdayAlarm(TEMP_CHECK_PERIOD)) != E_NO_ERROR) {
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
	MXC_RTC_SetSubsecondAlarm(0xFFFFFFFF - 2048);
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

	if((err = init_flash()) != E_NO_ERROR) { // Initialize flash space used to store temp readings
        return err;
   } else if((err = init_temp_sensor()) != E_NO_ERROR) { // Initialize temperature sensor (and I2C)
       return err;
    } else if((err = start_rtc()) != E_NO_ERROR) { // Initialize and start RTC, and set up TOD alarm
    	return err;
    }

    return E_NO_ERROR;
}

/***** Push Button ISR *****/
void temp_monitor_print_temps(void)
{
	// Calculate the number of temperature readings currently stored in flash
	int addr_offset;
	int num_temp = num_readings - (num_readings % 4);

	// Determine how many temperatures to read and where in flash they're located
	if(num_temp == 0) {
		printf("\nNo temperatures recorded, try reading again later.\n");
		return;
	} else if(num_temp < 12) {
		addr_offset = 0;
	} else {
		addr_offset = (num_temp - 12) * sizeof(temp_readings[0]);
		num_temp = 12;
	}

	uint32_t temps[num_temp];
	uint16_t time, temp;

	// Read all temperature readings stored in flash
	MXC_FLC_Read(TR_STORAGE_BASE_ADDR + addr_offset, (void*) temps, sizeof(temps));

	// Print out each temperature reading with it's timestamp
	printf("\nLast %d temperature Readings:\n", num_temp);
	for(int i = 0; i < num_temp; i++) {
		time = (temps[i] & 0xFFFF0000) >> 16;
		temp = temps[i] & 0xFFFF;
		printf("%ds - Temp: %dC\n", time, temp);
	}
}

/***** RTC Alarm ISRs *****/
void temp_monitor_check_temp(void)
{
	uint32_t current_sec, dummy;

	// Get current time
	while(MXC_RTC_GetTime(&current_sec, &dummy) == E_BUSY);

	// Set next TOD alarm TEMP_CHECK_PERIOD seconds into the future
	MXC_RTC_DisableInt(MXC_RTC_INT_EN_LONG);
	while(MXC_RTC_SetTimeofdayAlarm(current_sec + TEMP_CHECK_PERIOD) == E_BUSY);
	MXC_RTC_EnableInt(MXC_RTC_INT_EN_LONG);

	// Take temperature readinf
	record_current_temp(current_sec);

	// Enable or disable warning light if necessary
	if(temp_warning && !(MXC_RTC->ctrl & MXC_RTC_INT_EN_SHORT)) { //Temp warning active and warning light inactive --> enable
		enable_warning_light();
	} else if(!temp_warning && (MXC_RTC->ctrl & MXC_RTC_INT_EN_SHORT)) { //Temp warning inactive and warning light active --> disable
		disable_warning_light();
	}
}

void temp_monitor_flash_warning_light(void)
{
	// Toggle warning light
	LED_Toggle(0);
}
