## Description

A basic example demonstrating how to switch the system clock to use the external clock input.  The example initializes off of the default system clock, then switches to the external clock input before running the standard Hello World example to a count of 10.  Once the Hello World example completes, the example switches back to the IPO and shuts down.

Operation requires a clean external clock signal (square wave, 50% duty cycle) from a waveform generator.  Check your microcontroller's datasheet/EVKIT schematic to ensure the correct voltage levels.  When testing over the P0.12 GPIO header it's recommended to keep the clock frequency around the 2Mhz range to avoid signal degradation.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* The system header file for the target microcontroller expects `EXTCLK_FREQ` frequency to match the provided clock input.  This is defined for the build in [project.mk](project.mk).  The default value is 2Mhz.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-   Provide an external clock signal (square wave 50% duty cycle) on P0.12 with the frequency defined by `EXTCLK_FREQ` in [project.mk](project.mk).  At higher clock frequencies be careful to preserve the signal integrity when routing the signal to P0.12.  Otherwise, operation could become unreliable.

## Expected Output

The Console UART of the device will output these messages:

```
External Clock (EXT_CLK) example
Switching to 2000000 Hz external clock input in...
3...
2...
1...
Successfully switched to external clock (2000000 Hz)
Hello World!
count = 0
count = 1
count = 2
count = 3
count = 4
count = 5
count = 6
count = 7
count = 8
count = 9
count = 10
Success!  Example complete, switching back to IPO...
Back on IPO.  Done!
```

You will also observe LED1 blinking at a rate of 1Hz, even after the system clock has been switched to the external clock input.
