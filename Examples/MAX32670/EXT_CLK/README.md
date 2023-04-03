## Description

A basic example demonstrating how to switch the system clock to use the external clock input.  The example initializes off of the default system clock, then switches to the external clock input before running the standard Hello World example.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The system header file for the target microcontroller expects `EXTCLK_FREQ` frequency to match the provided clock input.  This is defined for the build in [project.mk](project.mk).

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Provide an external clock signal on P0.12 with the frequency defined by `EXTCLK_FREQ` in [project.mk](project.mk).

## Expected Output

The Console UART of the device will output these messages:

```
TODO
```

You will also observe LED1 blinking at a rate of 1Hz.
