## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
(LED1) is toggled each time the sub-second alarm triggers.  The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm triggers, the rate of the sub-second alarm is switched to 500 ms.

The time-of-day alarm is then rearmed for another 10 sec.  Pressing PB1 will output the current value of the RTC to the console UART.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************************** RTC Example ****************************

In this example the RTC is set up to demonstrate the functionality of the
   RTC second and sub-second alarms. When the sub-second alarm expires,
   the LED is toggled. When the second alarm expires, the rate at which
   the LED is toggled is switched between 250 and 750ms. Additionally,
   when SW2 is pressed the current time will be printed to the console.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:00.00


Current Time (dd:hh:mm:ss): 00:00:00:03.03

```