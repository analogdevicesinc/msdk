## Description

This example demonstrates read/write to OTP area inside device's internal memory. Write test irreversibly modifies the memory and it is disabled by default.

## Required Connections

### MAX32650 Evaluation Kit
-   Connect a USB cable between the PC and the CN2 (USB TO UART) connector.
-   Connect a debugger to J3 or J4 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP12 and JP13.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32650FTHR Evaluation Kit
-   Connect a USB cable between the PC and the J1 connector.
-   Connect a debugger to J5.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** OTP Memory Read/Write Example *****
***** This example demonstrates how you can read/write OTP memory *****


***** MANUFACTURER AREA *****

0x10800000:    c0520000 00121841    d7f48000 6618057f
0x10800010:    000d8000 00000000    5a5aa5a5 5a5aa5a5
0x10800020:    5a5aa5a5 5a5aa5a5    5a5aa5a5 5a5aa5a5
0x10800030:    ffffffff ffffffff    ffffffff ffffffff
0x10800040:    00003800 d9455305    00003800 d9455305
0x10800050:    ...

***** USER AREA *****

0x10801000:    11223344 ffffffff    ffffffff ffffffff
0x10801010:    ffffffff ffffffff    ffffffff ffffffff
0x10801020:    ffffffff ffffffff    ffffffff ffffffff
0x10801030:    ffffffff ffffffff    ffffffff ffffffff
0x10801040:    ffffffff ffffffff    ffffffff ffffffff
0x10801050:    ...

Example End
```