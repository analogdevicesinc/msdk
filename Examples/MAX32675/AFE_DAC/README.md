## Description

This example configures the MAX32675 AFE 12-Bit DAC to output 1V.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP1 (LED0 EN).
-   Connect multimeter in DC voltage mode between GND (TP2) and DAC12 (TP9) to measure output Voltage

## Expected Output

```
MAX32675 AFE DAC Example

This example configures the AFE's DAC to output a static 1.0V
Reseting DAC
DAC Configured for 1.0V output. Current AFE DAC registers:
AFE DAC CTRL:           0x30100040
AFE DAC RATE:           0x00FF2710
AFE DAC INT:            0x00000008
AFE DAC REG:            0x00000000
AFE DAC TRIM:           0x00001C00
AFE DAC VREF CTRL:      0x0000001C
AFE DAC FIFO:           0x00000000
AFE DAC VREF TRIM:      0x00000000
Done. Halting...
```
