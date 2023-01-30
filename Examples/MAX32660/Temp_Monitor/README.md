## Description

This simple application demonstrates the use of the MAX32660 as a temperature monitor. **NOTE: A MAX31889EVSYS is required to run the example.**

The device uses the RTC to trigger periodic measurements of the air temperature. Each time the RTC time-of-day alarm expires, the latest temperature reading is taken from an external MAX31889 temperature sensor which is connected to the device via I2C. The measurment is then time-stamped with the current time in the RTC and placed in a buffer. Once four successful measurements have been taken, they are stored in flash.

If a temperature reading exceeds the upper or lower limits, a warning message will be printed in the terminal and the LED will begin to flash at a frequnecy of 4Hz. Otherwise if the temperature is within the defined limits, the LED will stay illuminated until a temperature warning occurs.

Additionally, pressing push button SW2 will print the last 12 temperature readings in the terminal.

The temperature limits, flash storage page, and RTC time-of-day alarm period are defined in [temp_monitor.c](./temp_monitor.c) with the HI/LO\_TEMP\_THRESHOLD, TR\_STORAGE\_PAGE, and TEMP\_CHECK\_PERIOD defines respectively.

## Required Connections
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Make the following connections between the MAX32660 and MAX31889 EV Kits: P0.8(JH3.6)-->SCL(J2.11), P0.7(JH3.7)-->SDA(J2.12), VDDIO(JH3.9)-->VDD(J1.2), GND(JH4.9)-->GND(J1.4)

## Expected Output

The Console UART of the device will output these messages:

```
********************** Temperature Monitor Demo **********************
This simple example demonstrates the use of the MAX32660 as a temperature
monitor.

The device periodically measures the air temperature using an external
MAX31889 temperature sensor.

If a temperature reading exceeds the upper or lower limits, a warning message
will be printed in the terminal and the LED will begin to flash ata rate of
4Hz. Otherwise if the temperature is within the defined limits, the LED will
stay illuminated until a temperature warning occurs.

Press SW2 to print the last 12 temperature readings taken.
```

Additonally you should see either LED 0 or LED 1 being toggled periodically.