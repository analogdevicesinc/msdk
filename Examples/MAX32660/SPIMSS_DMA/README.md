## Description

This example configures the SPIMSS to send data between the MOSI (P0.11) and
MISO (P0.10) pins.  Connect these two pins together.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.10 (MISO) to P0.11 (MOSI).

## Expected Output

```
************** SPIMSS-DMA Master Demo ****************
This example configures the SPIMSS to send data between the MISO (P0.10) and
MOSI (P0.11) pins over dma channels.  Connect these two pins
together. This demo shows 1024 byte data transfer for 100 times using dma.;
During this demo you may see junk data printed to the serial port because the
console UART shares the same pins as the SPIMSS. One DMA channel is used as tx from
memory to SPIMSS TX FIFO and another dma channel is used for reading data from SPIMSS
RX FIFO to memory.

Test successful!
```
