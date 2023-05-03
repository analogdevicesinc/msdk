# U-Net Demo



Description
-----------

This demo shows a U-Net network, trained to segment images into four categories and color them as follows:

- Building: Red
- Tree: Green
- Sky: Blue
- Unknown: Black

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* Before building the firmware, please make sure to enable your intended mode of operation by enabling/disabling the following defines in [main.c](main.c):

    ```c
    // SELECT THE FOLLOWING BUILD OPTIONS
    // To use the camera, comment out both
    //#define USE_SAMPLEDATA        // shows the sample data
    //#define USE_SPIDATA			// shows images received from serial SPI - ONLY ON EVKIT
    ```

    To use sample_data,  only enable `#define USE_SAMPLEDATA` and rebuild the project. 

    To use serial SPI images, enable `#define USE_SPIDATA`.  The demo will expect images via the serial loader script in the `Utility` folder. **This mode is only supported on EVKIT**.

    To use the camera to capture images, disable both.

### Running Demo

If the project is built with `#define USE_SAMPLEDATA`, the offline sample data image, mask and the image overlaid with the mask will be shown on TFT. 

In serial SPI mode(only EVKIT), the display shows "Waiting for SPI data...".  Please check the [README](Utility/README.md)  in the `Utility` folder for instructions on how to send serial images.

If camera mode is selected (disable both `#define USE_SAMPLEDATA` and `#define USE_SPIDATA`), the image is captured from the camera and displayed.



### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation