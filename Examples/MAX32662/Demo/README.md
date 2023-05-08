## Description

A basic getting started program using the TFT Display.

This version of Hello_World prints an incrementing count to the console UART and TFT Display, and toggles a GPIO (P0.14 - LED1) once every 500 ms.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***********Hello World!***********

Count = 0
Count = 1
Count = 2
Count = 3
```

