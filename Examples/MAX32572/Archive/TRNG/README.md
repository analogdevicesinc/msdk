## Description

The true random number generator (TRNG) hardware is exercised in this example.  Random values are generated both using the blocking and non-blocking (asynchronous) functions.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** TRNG Example **********

Test TRNG Sync
0x15 0x93 0xcb 0x69
0xaa 0xb6 0x73 0xf9
0x87 0x04 0x81 0xc4
0xd8 0x7f 0x15 0x01

Test TRNG Async
0x4b 0x5c 0xec 0x30
0x52 0x9e 0x67 0x7e
0x6b 0xd7 0xb5 0xd8
0xad 0x95 0x66 0xbe
```
