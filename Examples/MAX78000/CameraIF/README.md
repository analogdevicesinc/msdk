# CameraIF

This example demonstrates the Parallel Camera Interface (PCIF) drivers and camera-specific drivers for the following cameras:

* OV7962 (Default)
* HM0360 (MONO), for color use the CameraIF_Debayer example
* OV5642
* HM01B0

To change the camera the project builds for, set the `CAMERA` build [configuration variable](.vscode/README.md#build-configuration) in [project.mk](project.mk)

The project comes pre-configured for the [MAX78000EVKIT](https://github.com/MaximIntegratedAI/MaximAI_Documentation/tree/master/MAX78000_Evaluation_Kit) by default.  It also supports the MAX78000FTHR board (OV7692 only), and can be reconfigured using the `BOARD` [configuration variable](.vscode/README.md#build-configuration).  Depending on your development environment, set `BOARD=FTHR_RevA` in the following place:

* Command-line development: project.mk
* Visual Studio Code: [.vscode/settings.json](https://github.com/Analog-Devices-MSDK/VSCode-Maxim/tree/develop#project-configuration)
* Eclipse: Project Properties -> C/C++ Build -> Environment
