## Description

This example demonstrates read/write to OTP area inside device's internal memory. Write test irreversibly modifies the memory and it is disabled by default.

## Required Connections

### MAX32520 Evaluation Kit
-   Connect a USB cable between the PC and the CN1 (USB TO UART) connector.
-   Connect a debugger to J2 or J3 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32520FTHR Evaluation Kit
-   Connect a USB cable between the PC and the J1 connector.
-   Connect a debugger to J2.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** OTP Memory Read/Write Example *****
***** This example demonstrates how you can read/write OTP memory *****


***** MAXIM AREA *****

0x10800000:    b6510000 048408c4    a81f0000 728c0481
0x10800010:    00560000 00000000    5a5aa5a5 5a5aa5a5
0x10800020:    5a5aa5a5 5a5aa5a5    5a5aa5a5 5a5aa5a5
0x10800030:    ffffffff ffffffff    ffffffff ffffffff
0x10800040:    00003800 d9455305    00003800 d9455305
0x10800050:    ...

***** USER AREA *****

0x10806000:    ffffffff ffffffff    ffffffff ffffffff
0x10806010:    ffffffff ffffffff    ffffffff ffffffff
0x10806020:    ffffffff ffffffff    ffffffff ffffffff
0x10806030:    ffffffff ffffffff    ffffffff ffffffff
0x10806040:    ffffffff ffffffff    ffffffff ffffffff
0x10806050:    ...

Example End
```