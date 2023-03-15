## Description

[MAX32672EVKIT](https://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/max32672evkit.html) has [CFAF128128B1](https://www.crystalfontz.com/product/cfaf128128b10145t-128x128-graphic-tft-spi) display 
which driven by [ST7735S driver](https://www.crystalfontz.com/controllers/Sitronix/ST7735S/).

This example uses [LVGL Graphics Library](https://lvgl.io/) to display images and text on the display.
The example bouncing text and blinking virtual and phsical LEDs.

To create LVGL compatible fonts and images use [Font Converter](https://lvgl.io/tools/fontconverter) and [Bitmap Converter](https://lvgl.io/tools/imageconverter).
To create LVGL compatible screens with drag & drop use [SquareLine GUI Builder](https://squareline.io/)

Note: This example only works on MAX32672EVKIT not on MAX32672FTHR board.



## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP10 and JP11 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
MAX32672 EvKit Display Demo
This example uses LVGL graphics library to manage display
For more demos please check: https://github.com/lvgl/lvgl
```
