## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC has two alarms available for use, the sub-second alarm and the time-of-day alarm. In this example the time-of-day alarm is used change the frequency of the subsecond alarm each time it is triggered (every 5 seconds). It will alternate between setting the subsecond alarm period to 250ms and 750ms. Each time the sub-second alarm is triggered LED0 is toggled. Thus, every 5 seconds you will see the rate at which LED0 is blinking change between 250ms and 750ms.

Additionally, pressing PB0 (SW2) will output the current time of the RTC to the console UART.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************************** RTC Example ****************************

In this example the RTC is set up to demonstrate the functionality of the
   RTC second and sub-second alarms. When the sub-second alarm expires,
   the LED is toggled. When the second alarm expires, the rate at which
   the LED is toggled is switched between 250ms and 750ms. Additionally,
   when SW2 is pressed the current time will be printed to the console.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:00.00


Current Time (dd:hh:mm:ss): 00:00:00:03.67


Current Time (dd:hh:mm:ss): 00:00:00:09.78
```