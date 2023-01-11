## Description

This example utilizes the MAX78000 to emulate a 32KiB EEPROM chip.

This "EEEPROM" can only perform read and write operations.

To write to the EEPROM, transmit a master write command followed by two write address bytes, and finally the data bytes. The write address is a 16-bit address, with the first address byte as the MSB and the second as the LSB. You may transmit as many data bytes as you wish, however only the last 64 data bytes will be stored. If you write past the end of the EEPROM address space, the write will continue at the beginning of the EEPROM address space.

To read from the EEPROM, simply issue a master read command and the device will begin transmitting its stored data from where the previous read operation left off. It is also possible to specify where the read operation will start by issuing a master write command followed by the two byte read address (same format as the write address), then a repeated start and master read command. Once you have received all the bytes you need, issue a NACK+STOP. If you read past the end of the EEPROM address space, the write will continue at the beginning of the EEPROM address space.

The default slave address of the EEPROM is 0x24. This can be modified by changing the value of the EEPROM_ADDR define in include/eeprom.h.

To help with syncronization, a "Ready Signal" is output from a GPIO pin. When the signal is high, the EEPROM is not currently processing a transaction and is ready for the next transaction to begin. When the signal is low, the EEPROM has either not been initialized or is currently still processing the previous transaction, and thus is not ready to process the next transaction.

**** NOTE ****: Due to the limitations of the flash controller, this example was implemented with a pseudo-cache. The cache copies the current page being operated on into volatile memory, where all reads and writes are performed. The cache is only written back to flash when there is a cache miss. So, if you wish to ensure the data you have written gets stored in flash, you will need to perform a read or write operation on another flash page. For reference, the flash memory used as EEPROM memory in this example is made up of four 8KiB flash pages.

## Setup

If using the Standard EvKit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).
-	Select "EvKit_V1" for _BOARD_ in "project.mk"
-   Connect pins SCL - P0.30 (J4.11) and SDA - P0.31 (J4.12) to the I2C Bus
-   Connect Ready Signal (P2.4) to the pin used for ready signal on your micro.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Select "FTHR_RevA" for _BOARD_ in "project.mk"
-   Connect pins SCL - P0.16 (J4.8) and SDA - P0.17 (J4.6) to the I2C Bus
-   Connect Ready Signal - P0.19 (J4.9) to the pin used for ready signal on your micro.

## Expected Output

The Console UART of the device will output these messages:

```
********************  EEPROM Emulator Demo *******************
```
