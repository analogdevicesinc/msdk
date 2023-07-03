## Description

TBD<!--TBD-->


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Disconnect RX_SEL (JP3) and TX_SEL jumpers if no data is moving in and out. I2S and UART share the same pins.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
I2S Transmission Example
You may need to disconnect RX_SEL (JP3) and TX_SEL
(JP4) in case no data is moving in and out of SDO/SDI.

I2S Transaction Complete. Ignore any random characters previously
displayed. The I2S and UART are sharing the same pins.
```
