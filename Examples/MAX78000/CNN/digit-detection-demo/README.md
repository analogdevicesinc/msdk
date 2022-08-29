# Digit Detection Demo



Description
-----------

This demo shows a tiny SSD network, trained to localize and recognize digits in images.

The image input size is 74x74 pixels RGB which is 74x74x3 in HWC format.


Before building the firmware, please make sure to enable intended mode of operation by comment/uncomment the following in `example_config.h`:

```c
//#define USE_SAMPLEDATA        // shows the sample data
```

### Building Firmware:

Navigate directory where digit detection demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/digit-detection-demo
$ make -r
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project:

```bash
$ make -r distclean
```

By default, the code is compiled for MAX78000 EVKIT.  To compile code for MAX78000 Feather board enable **BOARD=FTHR_RevA** in project.mk:

```bash
$ make -r BOARD=FTHR_RevA
```

**Note: If you are using Eclipse, please also make sure to change the value of Board environment variable to "FTHR_RevA by:**

*Right click project name > Properties > C/C++ Build > Environment > Board"*

<img src="./Resources/eclipse_board.png" style="zoom:33%;" />

### Load firmware image to MAX78000 EVKIT

- Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1).

- Connect PICO adapter to JH5 SWD header.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/max78000.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/max78000.elf verify reset exit"
```

### Load firmware image to MAX78000 Feather

Connect USB cable to CN1 USB connector.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/max78000.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/max78000.elf verify reset exit"
```

### MAX78000 Feather operations

The TFT display is optional and not supplied with the MAX78000 Feather board.
The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

To compile code with enabled TFT feature use the following setting in `example_config.h`:

```c
#ifdef BOARD_FTHR_REVA
// Disable TFT by default on FTHR
#define TFT_ENABLE
#endif
```

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset.

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