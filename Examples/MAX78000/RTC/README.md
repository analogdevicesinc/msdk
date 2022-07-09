## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
LED1 is toggled each time the sub-second alarm triggers.  

The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm triggers, the rate of the sub-second alarm is switched to 500 ms, and, if running the example on the standard EV kit, LED2 will be toggled. The time-of-day alarm is then rearmed for another 10 sec.

Pressing SW2 will output the current value of the RTC to the console UART.

## Setup

##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
*************************** RTC Example ****************************

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
(LED1) is toggled each time the sub-second alarm triggers.

The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm
triggers, the rate of the sub-second alarm is switched to 500 ms.

(LED2) is toggled each time the time-of-day alarm triggers.

The time-of-day alarm is then rearmed for another 10 sec.  Pressing SW2
will output the current value of the RTC to the console UART.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:00.00


Current Time (dd:hh:mm:ss): 00:00:00:02.38
```


