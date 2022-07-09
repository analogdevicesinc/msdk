## Description

TBD<!--TBD-->

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32665 SRAM ECC Example *****

This example will corrupt a word of data
and ensure that the ECC interrupts on an error
when the corrupted address is read

Preliminary scan to ensure no pre-existing ECC errors
537329664
PASS: No errors

Data before Corruption: 0xdeadbeef
Address of data: 0x20000c50

Data after single-bit error: 0xdeadbeef
ECC Error:              0x00000001
ECC Not Double Error:   0x00000001
ECC Error Address:      0x20000c50
PASS: An ECC Error was found

Data after double-bit error: 0xdeadbeec
ECC Error:              0x00000001
ECC Not Double Error:   0x00000000
ECC Error Address:      0x20000c50
PASS: An ECC Error was found

# Passed: 3, # Failed: 0, Test Ok
Example Complete

```
