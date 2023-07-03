## Description

This example demonstrates use of the I2S peripheral on the MAX32690 by transmitting 64 16-bit words using DMA. The use of a DMA callback function can optionally be enabled using the DMA callback define.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```
I2S Transmission Example
I2S Signals may be viewed on pins P2.26-P2.29.
```
