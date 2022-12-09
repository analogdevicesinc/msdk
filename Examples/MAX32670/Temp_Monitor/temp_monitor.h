#ifndef TEMP_MONITOR_H_
#define TEMP_MONITOR_H_

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

#endif // TEMP_MONITOR_H_
