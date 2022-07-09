## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
(LED 1) is toggled each time the sub-second alarm triggers.  The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm triggers, the rate of the sub-second alarm is switched to 500 ms.

(LED 2) is toggled each time the time-of-day alarm triggers. The time-of-day alarm is then rearmed for another 10 sec.  Pressing PB1 will output the current value of the RTC to the console UART.

On the Standard EV Kit:
-	PB1: P0.18/SW3
-	PB2: P0.19/SW4
-	LED 1: P0.24/LED0
-	LED 2: P0.25/LED1

On the Featherboard:
-	PB1: P0.2/SW2
-	PB2: P0.3/SW3
-	LED 1: P0.18/Red LED
-	LED 2: P0.19/Green LED

## Setup

##### Board Selection
Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_Apps\_P1", depending on the board version you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

```
*************************** RTC Example ****************************

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
(LED 1) is toggled each time the sub-second alarm triggers.

The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm
triggers, the rate of the sub-second alarm is switched to 500 ms.

(LED 2) is toggled each time the time-of-day alarm triggers.

The time-of-day alarm is then rearmed for another 10 sec.  Pressing PB1
will output the current value of the RTC to the console UART.

RTC started

Current Time (dd:hh:mm:ss): 00:00:00:00.00


Current Time (dd:hh:mm:ss): 00:00:00:02.38
```


