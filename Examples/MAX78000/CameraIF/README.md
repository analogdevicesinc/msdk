# CameraIF

This example demonstrates the Parallel Camera Interface (PCIF) drivers and camera-specific drivers for the following cameras:

* OV7962 (Default)
* HM0360 (MONO), for color use the CameraIF_Debayer example
* OV5642
* HM01B0

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* To change the camera drivers, set the `CAMERA` build [configuration variable](.vscode/README.md#build-configuration) in [project.mk](project.mk).
