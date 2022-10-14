# CameraIF_Debayer

This example demonstrates the HM0360 camera drivers for the [HM0360-AWA](https://www.digikey.com/en/products/detail/himax/HM0360-AWA/14109822) color sensor, which is a Bayer-patterned sensor.

It also demonstrates the debayering functions available for the HM0360 (bilinear, Malvar-He-Cutler, and a utility passthrough function).

To select the debayering function, change the value of the `g_bayer_function` configuration variable in [main.c](main.c) and recompile the project.
