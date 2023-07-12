## Description

This example demonstrates the use of the LCD Display on the MAX32650 EV Kit. This is accomplished by showing and clearing the ADI logo on the display continuously in a loop.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

To generate a bitmap C array for a particular image, external tools like GIMP (GNU Image Manipulation Program) or **[LVGL Online Image Converter](https://lvgl.io/tools/imageconverter)** can be used.

For example, to generate an 8 Bits Per Pixel palletized image array using GIMP:
- scale image to appropriate dimensions (Image -> Scale Image...)
- convert image mode to "Indexed" with a maximum of 255 colors (Image -> Mode -> Indexed...)
- export image as "C source code header" (File -> Export As...)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********** CLCD Example **********
```

The LCD Display will show the following patter in a loop: the ADI logo for 3 seconds, logo fades to blank, white screen, white screen holds for 3 seconds.
