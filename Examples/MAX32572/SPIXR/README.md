## Description

This example communicates with an MX25 SPI RAM on the EvKit using Quad SPI mode And the SPIXR peripheral

It writes random data to the MX25 SPI RAM and reads it back.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
****************** SPIXR Example ******************

This example communicates with an MX25 SPI RAM on the
EvKit using Quad SPI mode And the SPIXR peripheral

Setting up the SPIXR

TX BUFFER:       0  2d  cf  46  29  4  b4  78  d8  68  a7  ff  3f  2b  f1  fc

RX BUFFER:       0  2d  cf  46  29  4  b4  78  d8  68  a7  ff  3f  2b  f1  fc

DATA IS VERIFIED.

Example Succeeded
```
