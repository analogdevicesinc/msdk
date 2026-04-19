## Description

This example demonstrates various features of the Flash Controller (page erase and write), and how to use the CRC to compute a CRC value. In the terminal window, you can just type the command that you'd like to execute. The available commands are "write" (writes a text string to flash), "read" (reads text from flash), "erase" (erases the flash page being operated on), and "crc" (computes the CRC value of the entire flash page). For more details on how to input these commands, enter "help" in the terminal window.

*** NOTE ***: Attempting to overwrite flash will return an error. To overwrite an address, you must first delete it.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

### Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP3 and JP4 (UART 0).
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
*************** Flash Control CLI Example ***************

This example demonstrates various features of the Flash Controller
(page erase and write), and how to use the CRC to compute the
CRC value of an array. Enter commands in the terminal window.

CLI Initialized! Enter 'help' to see a list of available commands.

$ help
help

write:
  Usage: write <word offset> <text>
  Description: Writes text string to flash starting at the 32-bit word in the
    flash page specified by "word offset" (e.g. word offset=3 -> address
    offset=0xC, word offset=4 -> address offset=0x10)

erase:
  Usage: erase
  Description: Erases page in flash being operated on

read:
  Usage: read <word offset> <number of letters>
  Description: Reads text from flash starting at the 32-bit word in the flash
    page specified by "word offset" (e.g. word offset=3 -> address offset=0xC,
    word offset=4 -> address offset=0x10)

crc:
  Usage: crc
  Description: Calculates CRC of entire flash page

$ erase
erase
Success

$ crc
crc
CRC: 0x4BD6CBCA

$ write 23 DEADBEEF
write 23 DEADBEEF
Write addr 0x1005E05C: D
Write addr 0x1005E060: E
Write addr 0x1005E064: A
Write addr 0x1005E068: D
Write addr 0x1005E06C: B
Write addr 0x1005E070: E
Write addr 0x1005E074: E
Write addr 0x1005E078: F
Success

$ crc
crc
CRC: 0xA61BF020

$ read 23 8
read 23 8
Read addr 0x1005E05C: D
Read addr 0x1005E060: E
Read addr 0x1005E064: A
Read addr 0x1005E068: D
Read addr 0x1005E06C: B
Read addr 0x1005E070: E
Read addr 0x1005E074: E
Read addr 0x1005E078: F
Success:
DEADBEEF

$
```
