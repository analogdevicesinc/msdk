## Description

This example demonstrates how to capture an image using the PCIF peripheral and `Libraries/MiscDrivers/Camera` drivers, then send the captured image to a host device over UART.

It can perform standard blocking captures up to the memory limits of the device, and streaming DMA captures up to 352x352, including QVGA (320x240) at max camera clock speeds.

This example is designed for use with the Python utilities that can be found in the `pc_utility` folder.  See their [README](pc_utility/README.md) for more usage details.

## Build Notes

### Setting BOARD Correctly

Ensure you've set the `BOARD` value to match your evaluation platform.

For the MAX78000EVKIT, use `EvKit_V1`.
For the MAX78000FTHR, use `FTHR_RevA`.

* If you're developing on the command-line, set this value in [project.mk](project.mk).

* If you're developing with Visual Studio Code, set `"board"` in [.vscode/settings.json](.vscode/settings.json).  See the VSCode-Maxim [readme](.vscode/readme.md) for more details.

* If you're developing with Eclipse, set the `"BOARD"` environment variable.  Right click project -> Properties -> C/C++ Build -> Environment -> `"BOARD"`.  Apply, clean, and rebuild.