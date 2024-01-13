## Description

A basic "ping-pong" application for getting started with micro-ROS.  It can be used to follow along with the "Getting Started" micro-ros tutorials [here](https://micro.ros.org/docs/tutorials/core/overview/).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

Instructions on exercising this demo with ROS can be found in the [MSDK's micro-ROS library documentation]().

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* Configuration options for the custom micro-ROS serial transports be changed via the [mxc_microros_config.h](mxc_microros_config.h) file.

## Required Connections

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Close jumper JP1 (LED1 EN).
-   Close jumper JP2 (LED2 EN).

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.


