## Description

A starting Demo using the Display, RTC, LED, Push Buttons, and the [LVGL Graphics Library](https://lvgl.io/).

This example
-   Is intended for the EV Kit with the [LS013B7DH03 Monochrome LCD Display](https://www.sharpsde.com/products/displays/model/ls013b7dh03/).
-   Uses LVGL graphic library to generate ADI Logo and prints the count.

To create LVGL compatible fonts and images use [Font Converter](https://lvgl.io/tools/fontconverter) and [Bitmap Converter](https://lvgl.io/tools/imageconverter)
To create LVGL compatible screens with drag & drop use [SquareLine GUI Builder](https://squareline.io/)

Please check project.mk to learn how compile lvgl in MSDK


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This project is only supported on the MAX32665EVKIT.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
**** MAX32665 EV Kit Demo ****

(ddd:hh:mm:ss): 000:00:00:00


(ddd:hh:mm:ss): 000:00:00:01


```
