## Description

This simple application demonstrates the use of the MAX32672 as a temperature monitor. **NOTE: A MAX31889EVSYS is required to run the example.**

The device uses the RTC to trigger periodic measurements of the air temperature. Each time the RTC time-of-day alarm expires, the latest temperature reading is taken from an external MAX31889 temperature sensor which is connected to the device via I2C. The measurment is then time-stamped with the current time in the RTC and placed in a buffer. Once four successful measurements have been taken, they are stored in flash.

If a temperature reading exceeds the upper or lower limits, a warning message will be printed in the terminal and the red warning LED will begin to blink at a frequnecy of 4Hz. Otherwise if the temperature is within the defined limits, the green LED will continue to toggle each time a new measurement is taken.

Additionally, pressing push button SW3 will print the last 12 temperature readings in the terminal.

The temperature limits, flash storage page, and RTC time-of-day alarm period are defined in [temp_monitor.c](./temp_monitor.c) with the HI/LO\_TEMP\_THRESHOLD, TR\_STORAGE\_PAGE, and TEMP\_CHECK\_PERIOD defines respectively.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Ensure the I2C0 pullup enable jumpers (JP4 and JP5) are installed.
-   Make the following connections between the MAX32672 and MAX31889 EV Kits: P0.6-->SCL(J2.11), P0.7-->SDA(J2.12), VAUX(JH3.7)-->VDD(J1.2), GND(JH3.8)-->GND(J1.4)

## Expected Output

The Console UART of the device will output these messages:

```
********************** Temperature Monitor Demo **********************
This simple example demonstrates the use of the MAX32672 as a temperature
monitor.

The device periodically measures the air temperature using an external
MAX31889 temperature sensor.

If a temperature reading exceeds the upper or lower limits, a warning message
will be printed in the terminal and the red warning LED will begin to blink.
Otherwise if the temperature is within the defined limits, the green LED
will continue to toggle each time a new measurement is taken.

Press SW3 to print the last 12 temperature readings taken.
```

Additonally you should see either LED 0 or LED 1 being toggled periodically.