## Description

The true random number generator (TRNG) hardware is exercised in this example.  Random values are generated using both the blocking and non-blocking (asynchronous) functions.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** TRNG Example **********

Test TRNG Sync
0x76 0x88 0x51 0x45
0xf8 0x30 0xe3 0x78
0x6d 0x93 0x28 0xa1
0xee 0x52 0x22 0x84

Test TRNG Async
0xc9 0xb5 0x6a 0xc3
0x47 0x2e 0x0f 0xdb
0xd0 0x86 0xc1 0x7a
0x25 0x3b 0xa6 0x8d

********** Test Complete **********
```