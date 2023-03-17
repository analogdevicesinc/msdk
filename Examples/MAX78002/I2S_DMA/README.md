## Description

This application demonstrates receiving data from the microphone on the MAX78002 EV Kits using the I2S and DMA modules.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections:

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect the I2S mic (SPH0645) at header JH4.

## Expected Output

The Console UART of the device will output these messages:

```
***** I2S Receiver Example *****
Connect the mic at header JH4.
Receiving microphone data!

```

