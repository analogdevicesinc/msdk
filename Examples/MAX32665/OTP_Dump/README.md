## Description

This example demonstrates read/write to OTP area inside device's internal memory. Write test irreversibly modifies the memory and it is disabled by default.

## Required Connections

### MAX32665 Evaluation Kit
-   Connect a USB cable between the PC and the CN2 (USB TO UART) connector.
-   Connect a debugger to J6 or J7 (JTAG/SWD).
-   Select RX SEL and TX SEL on headers JP9 and JP10.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32665FTHR
-   Connect a USB cable between the PC and the J2 connector.
-   Connect a debugger to JH2.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

### MAX32666FTHR2
-   Connect a USB cable between the PC and the J10 connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
***** OTP Memory Read/Write Example *****
***** This example demonstrates how you can read/write OTP memory *****


***** MAXIM AREA *****

0x10800000:    80028000 80f7e6d5    00800000 fb66d581
0x10800010:    00570000 80000000    5a5aa5a5 5a5aa5a5
0x10800020:    ffffffff ffffffff    ffffffff ffffffff
0x10800030:    ffffffff ffffffff    ffffffff ffffffff
0x10800040:    ffffffff ffffffff    ffffffff ffffffff
0x10800050:    ...

***** USER AREA *****

0x10804000:    ffffffff ffffffff    ffffffff ffffffff
0x10804010:    ffffffff ffffffff    ffffffff ffffffff
0x10804020:    ffffffff ffffffff    ffffffff ffffffff
0x10804030:    ffffffff ffffffff    ffffffff ffffffff
0x10804040:    ffffffff ffffffff    ffffffff ffffffff
0x10804050:    ...

Example End
```