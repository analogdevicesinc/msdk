# CameraIF_Debayer

This example demonstrates the HM0360 camera drivers for the [HM0360-AWA](https://www.digikey.com/en/products/detail/himax/HM0360-AWA/14109822) color sensor, which is a Bayer-patterned sensor.

It requires debayering/demosaicking and color correction post-processing algorithms to reconstruct a color image.  This example demonstrates how to configure the camera drivers, apply the post-processing functions, and display the image to the TFT display or send it over UART.

The example builds for the TFT display by default.

**Note: The CSI2 camera must be unplugged from J8 for this example to work.  Connecting both the DVP and CSI2 cameras at the same time will cause the DVP camera initialization to fail.**

For instructions on setting up the MAX78002EVKIT see the [MAX78002EVKIT Quick-Start Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/tree/master/MAX78002_Evaluation_Kit)

For instructions on building and running the project, see the [.vscode/README.md](.vscode/README.md) file.
