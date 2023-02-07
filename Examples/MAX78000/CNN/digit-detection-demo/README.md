# Digit Detection Demo



Description
-----------

This demo shows a tiny SSD network, trained to localize and recognize digits in images.

The image input size is 74x74 pixels RGB which is 74x74x3 in HWC format.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* By default, this project is configured for [camera mode](#camera-mode).  It can be configured for [offline mode](#offline-mode) in [example_config.h](example_config.h) by defining `USE_SAMPLE_DATA`.

    ```C
    #define USE_SAMPLEDATA
    // ^ Uncomment this to use static sample data.
    // ^ Comment this out to use live camera data.
    ```

* This project supports output to a TFT display.  This feature can be toggled via the `TFT_ENABLE` option defined in [example_config.h](example_config.h)

    * For the MAX78000EVKIT, the TFT display is **enabled** by default.  To _disable_ it, undefine `TFT_ENABLE`.

        ```C
        #ifdef BOARD_EVKIT_V1
        //#define TFT_ENABLE
        #endif
        ```

    * For the MAX78000FTHR, the TFT display is **disabled** by default.  The TFT display is not supplied with the MAX78000 Feather board. The compatible 2.4'' TFT FeatherWing display can be ordered [here](https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing).  To _enable_ the display code, uncomment `#define ENABLE_TFT` in [example_config.h](example_config.h)

        ```C
        #ifdef BOARD_FTHR_REVA
        #define TFT_ENABLE
        #endif
        ```

### Offline Mode

If the project is built with `#define USE_SAMPLEDATA`, the offline sample data image, mask and the image overlaid with the mask will be shown on TFT.

This mode uses a header file "sampledata.h" containing RGB image data and it should be included in the project to use it as an input to the CNN network. 

To create your own header file follow these steps:

1. Navigate to Resources directory. $ cd Resources

2. Download image in this directory.

3. Open 'rgb.py' file and change the name of the image file on line 8:

   im = (Image.open('image_filename.format')). Save the changes.

4. Now generate a header file using this command: python3 rgb.py

5. Use this header file in your main.c

### Camera Mode

To operate in this mode, comment out "#define USE\_SAMPLEDATA", defined in main.c.

This mode uses OVM7692 camera module to capture an image in RGB888 format. Since the model is trained using 74x74 pixel image, the PCIF peripheral captures 74x74 pixel image and displays it on LCD.

The data received from camera interface is an unsigned data and should be converted to signed data before feeding to the CNN network.

<img src="Resources/evkit.jpg" style="zoom: 10%;" />

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation