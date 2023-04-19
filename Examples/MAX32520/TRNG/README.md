## Description

The true random number generator (TRNG) hardware is exercised in this example.  Random values are generated both using the blocking and non-blocking (asynchronous) functions.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** TRNG Example **********

Test TRNG Sync
0xdd 0x70 0xda 0xe3
0x09 0xfc 0xf4 0x1c
0x13 0xb3 0xb3 0x1a
0x5c 0x34 0x68 0x27

Test TRNG Async
0x92 0x5b 0x17 0x63
0x19 0x66 0x89 0xce
0xe2 0x44 0xf3 0x98
0x87 0xd5 0x60 0x06
```