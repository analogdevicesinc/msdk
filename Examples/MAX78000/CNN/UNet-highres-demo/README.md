# UNet Hi-Resolution Demo



Description
-----------

This demo shows a UNet network with 352x352 input, trained to segment images into four categories and color them as follows:

- Building: Red
- Tree: Green
- Sky: Blue
- Unknown: Black

Before building the firmware, please make sure to enable your intended mode of operation by enabling/disabling the following defines in `main.c`:

```c
#define USE_CAMERA   // if enabled, it uses the camera specified in the make file, otherwise it uses serial loader
```

### Building Firmware:

Navigate directory where UNet-highres-demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/UNet-highres-demo
$ make -r
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make -r distclean
```

To compile code for MAX78000 EVKIT enable **BOARD=EvKit_V1** in Makefile:

```bash
# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
#BOARD=FTHR_RevA
endif
```

To compile code for MAX78000 Feather board enable **BOARD=FTHR_RevA** in Makefile (requires TFT display for Feather https://github.com/MaximIntegratedAI/MaximAI_Documentation/tree/master/MAX78000_Feather):

```bash
# Specify the board used
ifeq "$(BOARD)" ""
#BOARD=EvKit_V1
BOARD=FTHR_RevA
endif
```

**Note: If you are using Eclipse, please also make sure to change the value of Board environment variable to "FTHR_RevA by:**

*right click project name > Properties > C/C++ Build > Environment > Board"*

<img src="Resources/eclipse_board.png" style="zoom:33%;" />



### Load firmware image to MAX78000 EVKIT or Feather Board

#### EVKIT:

- Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1).

- Connect PICO adapter to JH5 SWD header. 

#### Feather Board:

- Connect USB cable to CN1 USB connector.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/MAX78000.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### Running Demo

If camera mode is selected (#define USE_CAMERA), a captured camera image and calculated mask are both displayed on TFT. 

If the project is built without `#define USE_CAMERA`, offline sample data image is transfered over serial port from PC and calculated mask is sent back to PC. 

Check the [README](Utility/README.md)  in the `Utility` folder for instructions on how to send serial images.

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation