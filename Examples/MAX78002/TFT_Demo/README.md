# MAX78002 TFT and Touchscreen Demo Example

### Building firmware:

Navigate directory where **TFT_Demo** software is located and build the project:

```bash
$ cd /Examples/MAX78002/TFT_Demo
$ make -r
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make -r distclean
```


### Load firmware image to MAX78002 EVKIT

Connect USB cable to CN2 (USB/PWR) and turn ON power switch (SW4).

Connect PICO adapter to JH8 SWD header.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78002.cfg -c "program build/max78002.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78002.cfg -c "program build/max78002.elf verify reset exit"
```

### MAX78002 EVKIT operations

* Connect the DC 5V supply to MAX78002 evkit J1. Connect PICO adapter to JH8 SWD header. Turn the power switch SW1 to on position.

* To see debug messages, you can optionally connect a USB cable to CN2. Open a serial port application on the PC and connect to Ev-Kit's console UART at 115200, 8-N-1 configuration.

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation
