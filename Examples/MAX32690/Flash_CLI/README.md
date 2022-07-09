## Description

This example demonstates the CLI commands feature of FreeRTOS, various features of the Flash Controller (page erase and write), and how to use the CTB to compute a CRC value. In the terminal window, type the command that you wish to execute. The available commands are "write" (writes a text string to flash), "read" (reads text from flash), "erase" (erases the flash page being operated on) and "crc" (computes the CRC value of the entire flash page.) For more details on how to input these commands, enter "help" in the terminal window.

*** NOTE ***: Attempting to overwrite flash will return error. If you want to overwrite an address you must first erase.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).

## Expected Output

The Console UART of the device will output these messages:

```
*************** Flash Control CLI Example ***************

This example demonstrates the CLI commands feature of FreeRTOS, various features
of the Flash Controller (page erase and write), and how to use the CTB to
compute the CRC value of an array. Enter commands in the terminal window.

Starting FreeRTOS scheduler.

Enter 'help' to view a list of available commands.
cmd> help
help

help:
 Lists all the registered commands


erase:
 Erases page in flash being operated on

write <word offset> <text>:
 Writes text string to flash starting at the 32-bit word in the flash page
 specified by "word offset" (e.g. word offset=3 -> address offset=0xC,
 word offset=4 -> address offset=0x10)

read <word offset> <number of letters>:
 Reads text from flash starting at the 32-bit word in the flash page
 specified by "word offset" (e.g. word offset=3 -> address offset=0xC,
 word offset=4 -> address offset=0x10)

crc:
 Calculates CRC of entire flash page

cmd> erase
erase
Success

cmd> crc
crc
CRC: 0x96F4C82C

cmd> write 69 DEADBEEF
write 69 DEADBEEF
Write addr 0x102FC114: D
Write addr 0x102FC118: E
Write addr 0x102FC11C: A
Write addr 0x102FC120: D
Write addr 0x102FC124: B
Write addr 0x102FC128: E
Write addr 0x102FC12C: E
Write addr 0x102FC130: F
Success

cmd> read 69 8
read 69 8
Read addr 0x102FC114: D
Read addr 0x102FC118: E
Read addr 0x102FC11C: A
Read addr 0x102FC120: D
Read addr 0x102FC124: B
Read addr 0x102FC128: E
Read addr 0x102FC12C: E
Read addr 0x102FC130: F
Success:
DEADBEEF

cmd> crc
crc
CRC: 0x31A53ACA

cmd>
```