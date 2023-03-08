## Description

A basic OLED Display program that use [LVGL Graphics Library](https://lvgl.io/).

This example
-   Works in FTHR Board that has CFAL12832C OLED Display.
-   Uses LVGL graphic library to generate graphics.

To create LVGL compatible fonts and images use [Font Converter](https://lvgl.io/tools/fontconverter) and [Bitmap Converter](https://lvgl.io/tools/imageconverter)
To create LVGL compatible screens with drag & drop use [SquareLine GUI Builder](https://squareline.io/)

Please check project.mk to learn how compile lvgl in MSDK


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
OLED demo
This example uses LVGL graphics library to manage display
For more demos please check: https://github.com/lvgl/lvgl
Note: This example only works on MAX32672 FTHR board
```
