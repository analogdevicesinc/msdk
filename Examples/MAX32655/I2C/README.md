## Description

This is a basic getting started example for the I2C. 

In this example, one I2C block is configured in master mode and another is configured in slave mode.

The example will proceed as follows. The master and slave will each fill of their transmit buffers with identical data. The master will then transmit all data in it's transmit buffer to the slave. Next, the master will issue a repeated start and read request, and clock out all data bytes from the slave's transmit buffer. Once the entire transaction is complete the data received from the slave is compared against the master's transmit FIFO to ensure the master received the data successfully.

NOTE: This example is not supported by the MAX32655FTHR kit due to a lack of exposed I2C ports.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* NOTE: This project comes pre-configured for the MAX32655EVKIT.  It is not supported by the MAX32655FTHR kit.

## Required Connections
If using the MAX32655EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.10 to P0.16 (SCL) and P0.11 to P0.17 (SDA).
-   You must also connect the pull-up jumpers (JP21 and JP22) to the proper I/O voltage.

## Expected Output

The Console UART of the device will output these messages:

```
******** I2C SLAVE ASYNC TRANSACTION TEST *********

This example uses one I2C peripheral as a master to
read and write to another I2C which acts as a slave.

You will need to connect P0.10 to P0.16 (SCL) and
P0.11 to P0.17 (SDA).

-->I2C Master Initialization Complete
-->I2C Slave Initialization Complete

-->Writing data to slave, and reading the data back

-->Result:

-->TxData: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 60 61 62 63

-->RxData: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 60 61 62 63


-->I2C Transaction Successful
```
