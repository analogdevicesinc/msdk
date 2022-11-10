## Description

This example demonstrates how to use the TFT drivers to control the CFAF128128B1-0145T TFT display on the MAX32690 EV Kit.

The main functions which are shown in this example are displaying a bitmap image, displaying geometric shapes, and displaying text.

## Setup
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Ensure the display is plugged in to connector J4.

## Expected Display Sequence

The display will cycle through the the following sequence while the example is running:
    1. The Analog Devices logo.
    2. Various geometric shapes.
    3. Printing "Analog Devices, Inc." several times, each time in a different location and with a different font.

## Resources
Several resources have been provided in the "resources" directory to assist you in using the display for your own applications.
 - The "resources/tft_demo" directory includes the files which contain the bit mappings for the ADI logo and the fonts used in the demo.
 - In the "resources/tft_demo/bmp" folder you will find the bmp2c.py application which allows you convert .jpg and .bmp files into c arrays that 
can be passed to the MXC_TFT_ShowImage function. Instructions for how to used this application can be found in the README.md file in that directory.
 - The "resources/fonts" folder includes bitmap images which can be converted into new fonts using the bmp2c.py application.
 - The "resources/bmp_rle" and "resources/bmp_slides" directories contain other sample bitmap images.