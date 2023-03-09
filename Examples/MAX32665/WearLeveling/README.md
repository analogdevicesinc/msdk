## Description

Demonstrates Flash Wear Leveling usage on MAX32665 MCU.

MCU internal flash is partitioned as follows:
 -  Application code area: 64kb (Flash memory pages 0 - 7)
 -  Flash storage area: 64kb (Flash memory pages 8 - 15)
 
 The application code area should be defined in the linker script file *"max32665.ld"*:
 
 ```
 MEMORY {
    FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 64K /* 64kB "FLASH" */
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 160K /* 160kB SRAM */
}
 ```
 
The internal storage flash memory block count is specified by *FLASH_STORAGE_PAGE_CNT* macro.
 
 ```

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

 #define FLASH_STORAGE_PAGE_CNT 8
 ```
 
 that corresponds to 64kb (8 of 8kb blocks) 

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** MAX32665 Wear Leveling *****
Filesystem is mounted
boot_count: 12

Example Succeeded

```

