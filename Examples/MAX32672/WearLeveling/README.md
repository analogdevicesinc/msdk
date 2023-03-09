## Description

A basic application to demonstrate the setup and usage of the LittleFS library.

There are two tests available in this application, the Flash test and the LittleFS test. The Flash test exercises the functions defined in *flash.h* and *flash.c* which perform basic read, write and erase Flash operations (these functions are passed as pointers for use by the LittleFS library). The test will erase a section of Flash, write sample data to it, and verify the write was successful. Enable the flash test by setting the "FLASH_TEST" macro to 1 in *main.c*.

The LittleFS test will attempt to mount a filesystem in the flash memory, open a file in that filesystem, then read/modify/write a value in that file which keeps track of the number of times the filesystem has booted. The updated boot count is printed to the terminal on each successful iteration of this test. Enable this test by setting the "FLASH_TEST" macro to 0 in *main.c*.
 
## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

MAX32672 internal flash is partitioned as follows:
 -  Application code area: 128kb (Flash memory pages 0 - 15)
 -  LittleFS area: 64kb (Flash memory pages 16 - 23)
 
To ensure that these partitions are enforced, a special linker file, either *wearlevel.ld* or *wearlevel-sla.ld*, is used that stores application code in flash addresses 0x10000000-0x1001FFFF.
 
The LittleFS memory area can be modified by changing the page count specified by the "LFS\_PGE\_CNT" macro. Valid values are 1-112.
 
 ```
 #define LFS_PGE_CNT 8
 ```

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

If running the LittleFS test, the Console UART of the device will output these messages:

```
***** MAX32672 Wear Leveling *****
Filesystem is mounted
boot_count: 3

Example Succeeded
```
If running the FLASH_TEST portion of the example, the Console UART will output these messages:

```
***** MAX32672 Wear Leveling *****
Flash erase is verified.
Writing 16384 32-bit words to flash
Size of testdata : 65536
Verifying 16384 32-bit words in flash
Size of testdata : 65536

Example Succeeded
```
