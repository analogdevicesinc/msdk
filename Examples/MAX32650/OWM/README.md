## Description

This example demonstrate how 1-Wire master can be configured and read slave ROM ID.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the board connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect 1-Wire pin to the 1-Wire slave
-   Connect VCC and GND to the slave

## Expected Output

```
***** 1-Wire ROM (DS2401) Example *****
This example reads ROM ID of 1-Wire slave device
Connect 1-Wire pin, VCC and GND to the target


ROM ID: 3B D0 15 00 00 00 00 C8
Example Succeeded
```