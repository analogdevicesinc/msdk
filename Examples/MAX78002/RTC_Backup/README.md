## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC time-of-day alarm is used to wake the device from backup mode every TIME\_OF\_DAY\_SECONDS seconds (defined at the top of main.c). When the device wakes up, it will print the current time of the RTC to the terminal window and return to backup mode while waiting for the next time-of-day alarm to fire.


## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

```
***************** RTC Wake from Backup Example *****************

The time-of-day alarm is set to wake the device every 7 seconds.
When the alarm goes off it will print the current time to the console.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:07


Current Time (dd:hh:mm:ss): 00:00:00:14


Current Time (dd:hh:mm:ss): 00:00:00:21

...
```


