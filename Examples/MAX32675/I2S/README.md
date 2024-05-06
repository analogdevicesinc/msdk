## Description

TBD<!--TBD-->


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

The MAX32675 Revision B does not support I2S.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
I2S Transmission Example
I2S Signals may be viewed on pins P0.8-P0.11.
You may need to disconnect RX_SEL (JP5) and TX_SEL
(JP6) in case no data is moving in and out of SDO/SDI.

```
