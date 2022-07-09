## Description

This example demonstrates an I2S transmission. The I2S signals can be viewed on pins P1.2 (SCK), P1.3 (WS), P1.4 (SDI), and P1.5(SDO). DMA callback functionality can be enabled by defining DMA_CALLBACK.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select UART1 on Headers JP22 and JP23.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect logic analyzer or oscilloscope to pins P1.2-P1.5 to view I2S signals.

## Expected Output

```
I2S Transmission Example
I2S Signals may be viewed on pins P1.2-P1.5.

I2S Transaction Complete. Ignore any random characters previously
displayed. The I2S and UART are sharing the same pins.
```
