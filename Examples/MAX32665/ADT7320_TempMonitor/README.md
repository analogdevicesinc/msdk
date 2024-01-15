## Description

This example reads the temperature data from ADT7320 via SPI and writes it to the serial terminal.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect ADT7320 to the related pins on MAX32666 FTHR2 board.    
    
| ADT7320 | FTHR2 / EvKit   | FTHR  |
| ------- | --------------- | ----- |
| SCLK -> | P1_8_QSPI0_SCK  | P0_16 |
| GND ->  | GND             | GND   |
| DOUT -> | P1_8_QSPI0_MISO | P0_18 |
| VDD ->  | 3V3             | 3V3   |
| DIN ->  | P1_8_QSPI0_MOSI | P0_17 |
| CS ->   | P1_8_QSPI0_SS   | P0_16 |

## Expected Output

The Console UART of the device will output these messages:

```
***************** ADT7320 Temperature Sensor Example *****************
This example reads the temperature data from ADT7320 temperature sensor
via SPI and write it to the terminal.
You will need to connect the ADT7320 sensor to the SPI0 pins.

Over Temperature Limit Set to 29.297
Hysteresis Set to 1
Temperature Value = 28.438, Over Temp. Flag: 0

```

