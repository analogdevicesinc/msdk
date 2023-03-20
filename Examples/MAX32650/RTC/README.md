## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The sub-second alarm triggers at intervals of 250ms and 1000ms. LED0 is toggled each time the sub-second alarm triggers.  

The time-of-day alarm is set to 5 seconds.  When the time-of-day alarm triggers, the sub-second alarm interval is switched, if it is currently triggering at 250ms it will change to every 1000ms and vice versa.

Pressing SW2 will output the current value of the RTC to the console UART.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************************** RTC Example ****************************

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
LED0 (P2.25) is toggled each time the sub-second alarm triggers.

The time-of-day alarm is set to 5 seconds.  When the time-of-day alarm
triggers, the rate of the sub-second alarm is switched to 1000 ms.

The time-of-day alarm is then rearmed for another 5 sec.  Pressing SW2 will
output the current value of the RTC to the console UART.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:00.50


Current Time (dd:hh:mm:ss): 00:00:00:04.65
```