## Description

This example demonstates the CLI commands feature of FreeRTOS and various features of the Flash Controller (page erase and write). In the terminal window, type the command that you wish to execute. The available commands are "write" (writes a text string to flash), "read" (reads text from flash) and "erase" (erases the flash page being operated on.) For more details on how to input these commands, enter "help" in the terminal window.

*** NOTE ***: Attempting to overwrite flash will return error. If you want to overwrite an address you must first erase.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Flash Control CLI Example ***************

This example demonstrates the CLI commands feature of FreeRTOS and various
features of the Flash Controller (page erase and write). Enter commands in
the terminal window.Starting FreeRTOS scheduler.

Enter 'help' to view a list of available commands.
cmd> erase
erase
Success

cmd> write 13 DEADBEEF
write 13 DEADBEEF
Write addr 0x1003E034: D
Write addr 0x1003E038: E
Write addr 0x1003E03C: A
Write addr 0x1003E040: D
Write addr 0x1003E044: B
Write addr 0x1003E048: E
Write addr 0x1003E04C: E
Write addr 0x1003E050: F
Success

cmd> read 13 8
read 13 8
Read addr 0x1003E034: D
Read addr 0x1003E038: E
Read addr 0x1003E03C: A
Read addr 0x1003E040: D
Read addr 0x1003E044: B
Read addr 0x1003E048: E
Read addr 0x1003E04C: E
Read addr 0x1003E050: F
Success:
DEADBEEF

cmd>
```