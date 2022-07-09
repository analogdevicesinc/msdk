## Description

Demonstrates flash wear leveling on MAX78002 MCU.
 
The internal storage flash memory block count is specified by *FLASH_STORAGE_PAGE_CNT* macro.
 
 ```
 #define FLASH_STORAGE_PAGE_CNT 8
 ```
 
 that corresponds to 64kb (8 of 8kb blocks) 

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX78002 Wear Leveling *****
Filesystem is mounted
boot_count: 12

Example Succeeded

```

