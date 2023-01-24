# Flash Example

## Description

This example demonstrates the usage of the Flash Controller (FLC) for general purpose storage.  The following use-cases are demonstrated:

1. Reading bytes from a specific location in Flash
2. Writing and verifying a test pattern into Flash
3. Modifying Flash contents

Flash is **non-volatile** memory, meaning that it can retain state through power cycles.  However, application code is stored in Flash and the FLC has some limitations in how it can perform writes, so there are a few minor challenges to deal with when using it for general purpose storage.  This example demonstrates a simplified use-case that covers the most common scenarios.

The _first_ time the example is run the application will use the FLC to write and verify a test pattern into the last page of flash.  It will also write a 32-bit "magic" sequence into the page.

Once complete, the example will prompt the user to reset or power cycle the board.  This is to demonstrate that the written data is non-volatile and can survive a power cycle.

The _second_ time the example is run the application will see the "magic" 32-bit sequence in flash.  When this happens, the application will verify that the test pattern has survived the power cycle first.  Then, it will _modify_ the "magic" sequence _without_ modifying the rest of the test pattern.

## Building and Running

**See the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)** for detailed instructions on building and running example projects from supported development environments.

## Hardware Connections

MAX32660-EVSYS:

- Connect a USB cable between the PC and the J1 (micro-USB) connector.
- Open a terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

## Expected Output

After flashing and launching the example, an LED on the board will blink once every second.  This is the application waiting for PushButton 1 (PB1) to be pressed, and gives a window for a serial terminal to be connected.  After connecting the serial terminal, the application will output the following contents:

```
***** Flash Control Example *****
Press Push Button 1 (PB1/SW1) to continue...

---(Critical)---
Sucessfully erased page 64 of flash (addr 0x1007e000)
Writing magic value 0xfeedbeef to address 0x1007e000...
Done!
Writing test pattern...
Done!
----------------
 -> Interrupt! (Flash operation done)


Now reset or power cycle the board...

```

At this point, the "magic" and test pattern values have been written to flash.  Press SW5 to reset the board, after which the application will restart.  Push PB1 to continue the application again, which will print out the following contents:

```
***** Flash Control Example *****
Press Push Button 1 (PB1/SW1) to continue...

** Magic value 0xfeedbeef found at address 0x1007e000! **

(Flash modifications have survived a reset and/or power cycle.)

Verifying test pattern...
Sucessfully verified test pattern!

---(Critical)---
Erasing magic...
Buffering page...
Erasing page...
Re-writing from buffer...
New magic value: 0xabcd1234
----------------
 -> Interrupt! (Flash operation done)

Verifying test pattern...
Sucessfully verified test pattern!

Flash example successfully completed.

```
