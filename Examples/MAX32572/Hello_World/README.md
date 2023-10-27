## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a LED1 every 500 ms.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the J4 (USB to UART0) connector.
-   Install P1.8 (UART0 RX EN) and P1.9 (UART0 TX EN) on header JP8.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP6 (LED0 EN).
-   Close jumper JP7 (LED1 EN).

## Expected Output

The Console UART of the device will output these messages:

```
Hello World!
count : 0
count : 1
count : 2
count : 3
```

You will also observe LED0 blinking at a rate of 1Hz.
