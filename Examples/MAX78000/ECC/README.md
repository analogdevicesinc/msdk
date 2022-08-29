## Description

The ECC engine is used to watch for errors in data RAM.  The memory is scanned to ensure no errors exist.  

A word is then modified to have a single bit error.  That word is read and the ECC flags are examined to make sure a correctable error is detected.  The expected behavior here is that the read of the corrupted memory will result in the non-corrupted value, but the ECC error and ECC not double bit error flags will be set.

That same word is modified again to contain a double bit error.  The word is read and the ECC flags are examined to make sure a non-correctable error is reported.

## Setup
##### Building Firmware:
Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:
If using the standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX78000 SRAM ECC Example *****

This example will corrupt a word of data
and ensure that the ECC interrupts on an error
when the corrupted address is read

Preliminary scan to ensure no pre-existing ECC errors
536975769
PASS: No errors

Data before Corruption: 0xdeadbeef
Address of data: 0x20000e24

Data after single-bit error: 0xdeadbeef
ECC Error:              0x00000001
ECC Not Double Error:   0x00000001
ECC Error Address:      0x20000e24
PASS: An ECC Error was found

Data after double-bit error: 0xdeadbeec
ECC Error:              0x00000001
ECC Not Double Error:   0x00000000
ECC Error Address:      0x20000e24
PASS: An ECC Error was found

# Passed: 3, # Failed: 0, Test Ok
Example Complete
```

