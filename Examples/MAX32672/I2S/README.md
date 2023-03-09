## Description

This example demonstrates the use of I2S peripheral. In this example 128 bytes of data are output using an I2S DMA transaction. The I2S signals can be viewed on pins AIN0-AIN2. 


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select TX0 on Header JP11 (UART 0).
-   Remove header JP10 to view I2S Data Signal.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect a logic analyzer or oscilloscope to pins AIN0-AIN2 (P0.8 - P0.10) to view I2S data output, chip select and clock signals.

## Expected Output

```
I2S Transmission Example
Remove header JP10 to see I2S data on AIN0.

I2S Transaction Complete. Ignore any random characters previously
displayed. The I2S and UART are sharing the same pins.
```
