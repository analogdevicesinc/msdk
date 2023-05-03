## Description

This example communicates with SPI RAM on the EvKit using Quad SPI mode and the SPIXR peripheral

It writes random data to the SPI RAM and reads it back.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** SPIXR Example communicating with RAM in SPI Quad Mode *****
Setting up the SPIXR to communicate with RAM in Quad Mode
SPIXR was initialized properly.
Initializing & Writing pseudo-random data to be written to RAM
Reading data from RAM and store it inside the read_buffer
Data is verified.

Example Succeeded
```
