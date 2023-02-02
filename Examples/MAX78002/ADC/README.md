## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 0.  Vary the voltage on the AIN0 input (0 to 1.8V) to observe different readings from the ADC.

The example can be configured to either use a polling, interrupt driven, or DMA ADC Conversion by commenting or uncommenting the defined modes.

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.

Beware which Port 2 (channel) you use. Some of the Port 2 pins are connected to the switches, LED, and TFT display, so the readings may be off when using the EV Kit.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the [MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/).

- For a "quick-start" or for first-time users see ["Getting Started"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#getting-started)
- See ["Development Guide"](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#development-guide) for a detailed reference.

### Project-Specific Build Notes

(None)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Connect the 5V power cable at (5V IN).
-   Close jumper (RX - P0.0) and (TX - P0.1) at Headers JP23 (UART 0 EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Apply an input voltage to pin P2_0 (Channel 0).

## Expected Output

The Console UART of the device will output these messages:

```
********** ADC Example **********

The voltage applied to analog pin P2.0 is continuously
measured and the result is printed to the terminal.

The example can be configured to take the measurements
by polling, using interrupts, or using DMA.

0: 0x0001


0: 0x003f


0: 0x01ad


0: 0x028d


0: 0x0289


0: 0x006a
```
