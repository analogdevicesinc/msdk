## Description

This example shows how to manage multiple I2C transaction requests to the same I2C instance. To demonstrate, an I2C manager is used to ensure conflicting I2C requests (which could be submitted concurrently) don't interfere with one another.

More specifically, in this example the EEPROM0 task attempts to read data from EEPROM0 every 100ms at a bus frequency of 100kHz and the EEPROM1 task attempts to read data from EEPROM1 every 50ms at a bus frequency of 400kHz. The I2C manager protects the I2C instance from having to service both requests simultaneously by locking the instance while it's executing a transaction. If a transaction request is started while another transaction is executing, the requested transaction will wait for the I2C instance to be unlocked before submitting the request.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

You may change the configuration of each EEPROM's I2C transaction parameters (slave address, bus frequency, EEPROM read address, transaction interval, I2C timeout) by modifying their definitions at the top of main.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.18, Analog pin 10 on header JH3 to the SCL line of the I2C Bus.
-   Connect P0.19, Analog pin 11 on header JH3 to the SDA line of the I2C Bus.
-   Connect two EEPROM IC's to the I2C Bus.

## Expected Output

The Console UART of the device will output these messages:

```
***************** I2C Transaction Manager Demo *****************
Data is read from EEPROM0 every 100ms with an I2C bus frequency
of 100000Hz. And data is read from EEPROM1 every 50ms with an
I2C bus frequency of 400000Hz.

LED0 is toggled each time the read from EEPROM0 is executed and
LED1 is toggled each time the read from EEPROM1 is executed.

Starting scheduler.
``` 