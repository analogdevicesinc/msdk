## Description

This example demonstrates the use of the Real Time Clock (RTC) and its alarm functionality.

The RTC is enabled and the sub-second alarm set to trigger every 250 ms.
(LED 1) is toggled each time the sub-second alarm triggers.  The time-of-day alarm is set to 10 seconds.  When the time-of-day alarm triggers, the rate of the sub-second alarm is switched to 500 ms.

(LED 2) is toggled each time the time-of-day alarm triggers. The time-of-day alarm is then rearmed for another 10 sec.  Pressing PB1 will output the current value of the RTC to the console UART.

-	PB1: P4.0/SW2
-	LED 1: P0.14/LED1
-	LED 2: P2.12/LED2

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

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


