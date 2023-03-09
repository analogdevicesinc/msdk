## Description

This example demonstates the CLI commands feature of FreeRTOS, various features of the Flash Controller (page erase and write), and how to use the CRC to compute a CRC value. In the terminal window, type the command that you wish to execute. The available commands are "write" (writes a text string to flash), "read" (reads text from flash), "erase" (erases the flash page being operated on) and "crc" (computes the CRC value of the entire flash page.) For more details on how to input these commands, enter "help" in the terminal window.

*** NOTE ***: Attempting to overwrite flash will return error. If you want to overwrite an address you must first erase.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Setup

##### Board Selection

Before building firmware you must select the correct value for _BOARD_  in "[project.mk](project.mk)", either `EvKit_V1` or `FTHR_Apps_P1`, depending on the EV kit you are using to run the example.

##### Required Connections
If using the Standard EV Kit (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP2 (LED0 EN).
-   Close jumper JP3 (LED1 EN).

If using the Featherboard (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Flash Control CLI Example ***************

This example demonstrates the CLI commands feature of FreeRTOS, various features
of the Flash Controller (page erase and write), and how to use the CRC to
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
CRC: 0x4BD6CBCA

cmd> write 23 DEADBEEF
write 23 DEADBEEF
Write addr 0x1007E05C: D
Write addr 0x1007E060: E
Write addr 0x1007E064: A
Write addr 0x1007E068: D
Write addr 0x1007E06C: B
Write addr 0x1007E070: E
Write addr 0x1007E074: E
Write addr 0x1007E078: F
Success

cmd> crc
crc
CRC: 0xA61BF020

cmd> read 23 8
read 23 8
Read addr 0x1007E05C: D
Read addr 0x1007E060: E
Read addr 0x1007E064: A
Read addr 0x1007E068: D
Read addr 0x1007E06C: B
Read addr 0x1007E070: E
Read addr 0x1007E074: E
Read addr 0x1007E078: F
Success:
DEADBEEF

cmd>
```