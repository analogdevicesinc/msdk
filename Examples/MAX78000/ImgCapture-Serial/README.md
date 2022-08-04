## Description

This example demonstrates how to capture an image using the PCIF peripheral and `Libraries/MiscDrivers/Camera` drivers, then send the captured image to a host device over UART.

It can perform standard blocking captures up to the memory limits of the device, and streaming DMA captures up to 352x352, including QVGA (320x240) at max camera clock speeds.

This example is designed for use with the Python utilities that can be found in the `pc_utility` folder.  See their [README](pc_utility/README.md) for more usage details.