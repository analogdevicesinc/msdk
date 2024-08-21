# UNet Hi-Resolution Demo



Description
-----------

This demo shows a UNet network with 352x352 resolution input, trained to segment images into four categories and color them as follows:

- Building: Red
- Tree: Green
- Sky: Blue
- Unknown: Black

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* Before building the firmware, please make sure to enable your intended mode of operation by enabling/disabling the following define in [main.c](main.c):

    ```c
    #define USE_CAMERA   // if enabled, it uses the camera specified in the make file, otherwise it uses serial loader
    ```

### Running Demo

If camera mode is selected (#define USE_CAMERA), a captured camera image and calculated mask are both displayed on TFT.

If the project is built without `#define USE_CAMERA`, offline sample data image is transfered over serial port from PC and calculated mask is sent back to PC.

Check the [README](Utility/README.md)  in the `Utility` folder for instructions on how to send serial images.

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation