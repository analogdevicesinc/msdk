## Description

This example uses the I2C0 as an I2C master to read/write from/to I2C2, an I2C slave. 

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Connect AIN10 (P0.18) to P0.6 (SDA) and AIN11 (P0.19) to P0.7 (SCL).
-   Connect the I2C Pull-Up Resisters by connecting jumpers JP4 and JP5. 

## Expected Output
```
******** I2C SLAVE ASYNC TRANSACTION TEST *********

This example uses one I2C peripheral as a master to
read and write to another I2C which acts as a slave.

You will need to connect AIN10 (P0.18) to P0.6 (SCL) and
AIN11 (P0.19) to P0.7 (SDA).

Connect the I2C Pull-Up Registers by connecting jumpers
at JP4 and JP5.

-->I2C Master Initialization Complete
-->I2C Slave Initialization Complete

-->Writing data to slave, and reading the data back

-->Result:

-->TxData: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 60 61 62 63

-->RxData: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f 10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f 20 21 22 23 24 25 26 27 28 29 2a 2b 2c 2d 2e 2f 30 31 32 33 34 35 36 37 38 39 3a 3b 3c 3d 3e 3f 40 41 42 43 44 45 46 47 48 49 4a 4b 4c 4d 4e 4f 50 51 52 53 54 55 56 57 58 59 5a 5b 5c 5d 5e 5f 60 61 62 63


-->I2C Transaction Successful
```