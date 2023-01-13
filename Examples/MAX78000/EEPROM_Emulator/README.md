## Description

This example utilizes the MAX78000 to emulate a 32KiB EEPROM chip with an I2C interface.

This "EEEPROM" can only perform read and write operations.

To write to the EEPROM, transmit a master write command followed by two write address bytes, and finally the data bytes. The write address is a 16-bit address, with the first address byte as the MSB and the second as the LSB. The EEPROM emulator is able to receive up to 64 bytes per write operation. If you write past the end of the EEPROM address space, the write will continue at the beginning of the EEPROM address space.

Unlike write operations, the EEPROM determines the location of the read based on the value of an internal read address pointer. The read pointer is initialized to address 0x0000 and increments by 1 after transmitting the value at the current address. For example, if the value of the read pointer was initially 0x0000 and you read 8 bytes, the device will transmit bytes 0x0000-0x0007 and the final value of the read pointer will be 0x0008 which is where the next read operation will start from.

To initiate a read operation simply transmit a master read command. The device will then begin transmitting bytes starting from the current read pointer. If, however, you do not wish to read from the current address of the read pointer, you may set the read pointer by initiating the read operation with a master write command followed by the new two byte address of the read pointer (same format as the write address). Then, issue a repeated start and master read command. The device will then begin transmitting bytes starting from the new read pointer. Read operations will continue until a NACK+STOP is issued by the master. If you read past the end of the EEPROM address space, the read will continue at the beginning of the EEPROM address space.

The default slave address of the EEPROM is 0x24. This can be modified by changing the value of the EEPROM_ADDR define in include/eeprom.h.

To help with syncronization, a "Ready Signal" is output from a GPIO pin. When the signal is high, the EEPROM is not currently processing a transaction and is ready for the next transaction to begin. When the signal is low, the EEPROM has either not been initialized or is currently still processing the previous transaction, and thus is not ready to process the next transaction.

**** NOTE ****: Due to the limitations of the flash controller, this example was implemented with a pseudo-cache. The cache copies the current page being operated on into volatile memory, where all reads and writes are performed. The cache is only written back to flash when there is a cache miss. So, to ensure the data you have written gets stored in flash, you will need to perform a read or write operation on another flash page. For reference, the flash memory used as EEPROM memory in this example is made up of four 8KiB flash pages.

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
