# MAX78000 Rock Paper Scissors Game

This is a classic rock-paper-scissors game demo that user can play against the computer via the camera module. The model trained in this demo is used to classify images of "rock", "paper" and "scissors" hand gestures. The input size is 64x64 pixels RGB which is 3x64x64 in CHW format.

The example supports live capture from camera module and displays the result on the TFT LCD. The code also uses a sample data header (sampledata.h) file to test a pre-defined input sample.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* By default, this project is configured for [camera mode](#camera-mode).  It can be configured for [offline mode](#offline-mode) by defining `USE_SAMPLE_DATA` in [main.c](main.c).

    ```C
    // Comment out USE_SAMPLEDATA to use Camera module
    #define USE_SAMPLEDATA
    ```

* This project supports output to a TFT display.  When building for the MAX78000EVKIT, the display is **enabled** by default.

    * To _disable_ the TFT display code, comment out `PROJ_CFLAGS += -DTFT_ENABLE` in [project.mk](project.mk)

        ```Makefile
        ifeq "$(BOARD)" "EvKit_V1"
        # PROJ_CFLAGS+=-DTFT_ENABLE
        IPATH += TFT/evkit/
        VPATH += TFT/evkit/
        endif
        ```

* When building for the MAX78000FTHR, the TFT display is **disabled** by default.  The compatible 2.4'' TFT FeatherWing is an optional display that does not come with the MAX7800FTHR.  It can be ordered [here](https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing)

    * To _enable_ the TFT display code, uncomment `PROJ_CFLAGS += -DTFT_ENABLE` in [project.mk](project.mk)

        ```Makefile
        ifeq "$(BOARD)" "FTHR_RevA"
        # Only Enable if 2.4" TFT is connected to Feather
        PROJ_CFLAGS+=-DTFT_ENABLE
        IPATH += TFT/fthr
        VPATH += TFT/fthr
        endif
        ```

### MAX78000 EVKIT operations

*   If using camera and TFT LCD, place OVM7692 camera module on 'J4 Camera' header. Place TFT display on the display header.
*   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
*   Place jumper P0\_0 and P0\_1 on UART\_0\_EN header JH1.
*   Open a serial port application on the PC and connect to Ev-Kit's console UART at 115200, 8-N-1 configuration.

This demo is operated in two modes: Real-time data using Camera module or using sample image header file in offline mode.

In either mode, pushbutton trigger PB1(SW2) is used to capture and load an image into CNN engine. User is prompted to press PB1 to load an image

### MAX78000 Feather operations

The TFT display is optional and not supplied with the MAX78000 Feather board.
User should use PC terminal program to observe **cats-dogs_demo** result as described in "Terminal output" section.

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

See [build notes](#project-specific-build-notes) for instructions on enabling the display.

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset.
Press PB1 (SW1) button to start demo.

### Camera Mode�

If `USE_SAMPLEDATA` is _not_ defined the project will operate in "Camera mode".  See [build notes](#project-specific-build-notes) for instructions on enabling this mode.

This mode uses OVM7692 camera module to capture an image in RGB888 format. Since the model is trained using 64x64 pixel image, the PCIF peripheral captures 64x64 pixel image and displays it on LCD.

The data received from camera interface is an unsigned data and should be converted to signed data before feeding to the CNN network.

### Offline Mode

If `USE_SAMPLEDATA` is defined this project will operate in "offline mode".

This mode uses a header file [sampledata.h](sampledata.h) containing RGB image data and it should be included in the project to use it as an input to the cnn network.

To create your own header file follow these steps:

1.  Navigate to Utility directory. $ cd Utility
2.  Download rock, paper or scissors hand gesture image in this directory.
3.  Open 'rgb.py' file and change the name of the image file on line 8. im = (Image.open('image_filename.format')). Save the changes.
4.  Now generate a header file using this command: python3 rgb.py
5.  Use this header file in your main.c

Terminal output
---------------

The Console UART of the device will output these messages:

```
RPS Feather Demo
Waiting...
Init LCD.
Init Camera.
********** Press PB1 to capture an image **********

Capture a camera frame 1
Copy camera frame to CNN input buffers.
Show camera frame on LCD.
Time for CNN: 2667 us

Classification results:
[ 153671] -> Class 0    Paper: 100.0%
[-216581] -> Class 1     Rock: 0.0%
[-283383] -> Class 2 Scissors: 0.0%

User choose: Paper 
Computer choose: Scissors

COMPUTER WINS!!!

********** Press PB1 to capture an image **********
```

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation
