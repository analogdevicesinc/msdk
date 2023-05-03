## Description

A basic getting started program.

This version of Hello_World prints an incrementing count to the console UART and toggles a GPIO (LED0) once every 500 ms.

Please check the board.c file in ${MSDKPath}\Libraries\Boards\MAX32665\${BoardName}\Source path to learn switch and LED pins for specific board.


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
***********Hello World!***********

LED0 toggles every 500 ms

Count = 0
Count = 1
Count = 2
Count = 3
```

