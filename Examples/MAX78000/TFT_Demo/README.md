## Description

TFT Demo example demonstrates how TFT and touchscreen can be used.

The bitmap and font files keep in resources directory.
The structure of resources directory shall be 
- **bmp_rle**	 : Includes RLE or standard bitmap encoded image files
- **fonts**		 : Include fonts files

The tool **../Tools/BitmapConverter/maxim_bitmap_converter.exe** is used to convert bitmap and font files to maxim specific file format (.mpi maxim picture information)
And **bitmap.h** file under resources/ directory will be created automatically.

### bitmap.h file include bitmap definitions id. Like below:
#ifndef _BITMAP_H_
#define _BITMAP_H_

// bitmaps id
#define    check_empty_bg_white_bmp                           0
#define    check_empty_bg_darkgrey_bmp                     1
#define    check_empty_bg_lightgrey_bmp                     2
...

### #endif //_BITMAP_H_

Restriction: 

bitmap files has to be encoded 8 bits (256 color bitmap).

### Building firmware:

Navigate directory where TFT_demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/TFT_demo
$ make
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make distclean
```

To compile code for MAX78000 EVKIT enable **BOARD=EvKit_V1** in project.mk:

```Makefile
BOARD=EvKit_V1
```

To compile code for MAX78000 Feather board enable **BOARD=FTHR_RevA** in project.mk:

```Makefile
BOARD=FTHR_RevA
```

### Load firmware image to MAX78000 EVKIT

Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1).

Connect PICO adapter to JH5 SWD header. 

Load firmware image using Openocd.

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### Load firmware image to MAX78000 Feather

Connect USB cable to CN1 USB connector.

Load firmware image using Openocd.

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### MAX78000 Feather operations

The TFT display is optional and not supplied with the MAX78000 Feather board.

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered from here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

To compile code with enabled TFT feature use following setting in project.mk:

```bash
ifeq "$(BOARD)" "FTHR_RevA"
PROJ_CFLAGS += -DENABLE_TFT
endif
```

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset.

