## Description

Demonstrates Flash Wear Leveling usage on MAX32690 MCU.

MCU internal flash is partitioned as follows:
 -  Application code area: 128kb (Flash memory pages 0 - 7)
 -  Flash storage area: 128kb (Flash memory pages 8 - 15)
 
The internal storage flash memory block count is specified by *FLASH_STORAGE_PAGE_CNT* macro.
 
 ```
 #define FLASH_STORAGE_PAGE_CNT 8
 ```
 
 that corresponds to 128kb (8 of 16kb blocks) 

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7 (RX\_EN) and JP8 (TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32690 Wear Leveling *****
Filesystem is mounted
boot_count: 1

Example Succeeded
```

