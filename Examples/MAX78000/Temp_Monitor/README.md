## Description

This simple application demonstrates the use of the MAX78000 as a temperature monitor. **NOTE: A MAX31889EVSYS is required to run the example.**

The device uses the RTC to trigger periodic measurements of the air temperature. Each time the RTC time-of-day alarm expires, the latest temperature reading is taken from an external MAX31889 temperature sensor which is connected to the device via I2C. The measurment is then time-stamped with the current time in the RTC and placed in a buffer. Once four successful measurements have been taken, they are stored in flash.

If a temperature reading exceeds the upper or lower limits, a warning message will be printed in the terminal and the red warning LED will begin to blink at a frequnecy of 4Hz. Otherwise if the temperature is within the defined limits, the green LED will continue to toggle each time a new measurement is taken.

Additionally, pressing the push button will print the last 12 temperature readings in the terminal.

The temperature limits, flash storage page, and RTC time-of-day alarm period are defined in [temp_monitor.c](./temp_monitor.c) with the HI/LO\_TEMP\_THRESHOLD, TR\_STORAGE\_PAGE, and TEMP\_CHECK\_PERIOD defines respectively.

## Required Connections

If using the Standard EvKit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-	Select "EvKit_V1" for _BOARD_ in "project.mk"
-   Make the following connections between the MAX78000 and MAX31889 EV Kits: P0.30-->SCL(J2.11), P0.31-->SDA(J2.12), VDDIO(JP9.3)-->VDD(J1.2), GND-->GND(J1.4)

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Select "FTHR_RevA" for _BOARD_ in "project.mk"
-   Make the following connections between the MAX78000 and MAX31889 EV Kits: P0.16(J4.11)-->SCL(J2.11), P0.17(J4.12)-->SDA(J2.12), 3V3(J8.2)-->VDD(J1.2), GND(J8.4)-->GND(J1.4)

## Expected Output

The Console UART of the device will output these messages:

```
********************** Temperature Monitor Demo **********************
This simple example demonstrates the use of the MAX78000 as a temperature
monitor.

The device periodically measures the air temperature using an external
MAX31889 temperature sensor.

If a temperature reading exceeds the upper or lower limits, a warning message
will be printed in the terminal and the red warning LED will begin to blink.
Otherwise if the temperature is within the defined limits, the green LED
will continue to toggle each time a new measurement is taken.

Press SW2 to print the last 12 temperature readings taken.
```

Additonally you should see either LED 0 or LED 1 being toggled periodically.