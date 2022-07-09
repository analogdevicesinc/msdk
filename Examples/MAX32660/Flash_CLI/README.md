## Description

This example demonstates the CLI commands feature of FreeRTOS and various features of the Flash Controller (page erase and write). In the terminal window, type the command that you wish to execute. The available commands are "write" (writes a text string to flash), "read" (reads text from flash) and "erase" (erases the flash page being operated on.) For more details on how to input these commands, enter "help" in the terminal window.

*** NOTE ***: Attempting to overwrite flash will return error. If you want to overwrite an address you must first erase.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Flash Control CLI Example ***************

This example demonstrates the CLI commands feature of FreeRTOS and various
features of the Flash Controller (page erase and write). Enter commands in
the terminal window.Starting FreeRTOS scheduler.

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

cmd> erase
erase
Success

cmd> write 63 DEADBEEF
write 63 DEADBEEF
Write addr 0x0003E0FC: D
Write addr 0x0003E100: E
Write addr 0x0003E104: A
Write addr 0x0003E108: D
Write addr 0x0003E10C: B
Write addr 0x0003E110: E
Write addr 0x0003E114: E
Write addr 0x0003E118: F
Success

cmd> read 63 8
read 63 8
Read addr 0x0003E0FC: D
Read addr 0x0003E100: E
Read addr 0x0003E104: A
Read addr 0x0003E108: D
Read addr 0x0003E10C: B
Read addr 0x0003E110: E
Read addr 0x0003E114: E
Read addr 0x0003E118: F
Success:
DEADBEEF

cmd>
```