## Description

This example utilizes the MAX32675 to emulate a 32KiB EEPROM chip with an I2C interface.

This "EEEPROM" can only perform read and write operations.

To write data to the EEPROM emulator execute the following steps:
```
1. Issue an I2C start followed by the EEPROM Emulator slave address with the R/W bit set to 0.
2. Send the write address (the address to start storing data bytes at). The write address is 2 bytes long with the first address byte as the MSB and the second address byte as the LSB.
3. Send up to 64 data bytes.
```

![image info](./EEPROM_OP_Diagrams/EEPROM_Write.png)

Unlike write operations, the EEPROM determines the location of the read based on the value of an internal read address pointer. The read pointer is initialized to address 0x0000 and increments by 1 after transmitting the value at the current address. For example, if the value of the read pointer was initially 0x0000 and you read 8 bytes, the device will transmit the contents of locations 0x0000-0x0007 and the final value of the read pointer will be 0x0008 which is where the next read operation will start from.

To read from the currrent address of the read pointer:
```
1. Issue an I2C start followed by the EEPROM Emulator slave address with the R/W bit set to 1.
2. Read as many bytes as you need.
3. Once you have received the last byte, issue a NACK+STOP.
```

![image info](./EEPROM_OP_Diagrams/EEPROM_Read.png)

To set the read pointer to a new address and read from there, execute the following steps:
```
1. Issue an I2C start followed by the EEPROM Emulator slave address with the R/W bit set to 0.
2. Send the new read pointer address. The read pointer address is the same format as a write address, 2 bytes long with the first address byte as the MSB and the second address byte as the LSB.
3. Issue a repeated start followed by the EEPROM Emulator slave address with the R/W bit set to 1.
4. Read as many bytes as you need.
5. Once you have received the last byte, issue a NACK+STOP.
```

![image info](./EEPROM_OP_Diagrams/EEPROM_Read_Set_Pointer.png)

The default slave address of the EEPROM is 0x24. This can be modified by changing the value of the EEPROM_ADDR define in include/eeprom.h.

To help with syncronization, a "Ready Signal" is set up as an output from a GPIO pin. When the signal is high, the EEPROM is not currently processing a transaction and is ready for the next transaction to begin. When the signal is low, the EEPROM has either not been initialized or is currently still processing the previous transaction, and thus is not ready to process the next transaction.

**** NOTE ****: Due to the limitations of the internal flash, EEPROM data is buffered in SRAM. Two events can trigger the buffer to be written back to flash: 1) when an operation (read or write) is performed on a flash page that is not currently buffered (for reference the EEPROM is made of four 8KiB flash pages and the buffer holds a single page at a time), or 2) a write operation is received with a write address of 0xBEEF.

![image info](./EEPROM_OP_Diagrams/EEPROM_Force_WB.png)

## Setup

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pins SCL - P0.6 and SDA - P0.7 to the I2C Bus
-   Install jumpers JP3 and JP4 to enable I2C pullup resistors.
-   Connect Ready Signal (P0.19) to the pin used for ready signal on your micro.

## Expected Output

The Console UART of the device will output these messages:

```
********************  EEPROM Emulator Demo *******************
```
