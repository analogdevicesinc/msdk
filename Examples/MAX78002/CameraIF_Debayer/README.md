# CameraIF_Debayer

## Overview

This example demonstrates the HM0360 camera drivers for the [HM0360-AWA](https://www.digikey.com/en/products/detail/himax/HM0360-AWA/14109822) color sensor, which is a Bayer-patterned sensor.

It requires debayering/demosaicking and color correction post-processing algorithms to reconstruct a color image.  This example demonstrates how to configure the camera drivers, apply the post-processing functions, and display the image to the TFT display or send it over UART.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Hardware

For instructions on setting up the MAX78002EVKIT see the [MAX78002EVKIT Quick-Start Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/tree/master/MAX78002_Evaluation_Kit)

**Note: The CSI2 camera must be unplugged from J8 for this example to work.  Connecting both the DVP and CSI2 cameras at the same time will cause the DVP camera initialization to fail.**
