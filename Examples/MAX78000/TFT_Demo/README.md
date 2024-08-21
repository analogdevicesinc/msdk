## Description

TFT Demo example demonstrates how TFT and touchscreen can be used.

The bitmap and font files keep in resources directory.
The structure of resources directory shall be 
- **bmp_rle**	 : Includes RLE or standard bitmap encoded image files
- **fonts**		 : Include fonts files

The tool **../Tools/BitmapConverter/maxim_bitmap_converter.exe** is used to convert bitmap and font files to maxim specific file format (.mpi maxim picture information)
And **bitmap.h** file under resources/ directory will be created automatically.

bitmap.h file include bitmap definitions id. Like below:

```C
#ifndef _BITMAP_H_
#define _BITMAP_H_

// bitmaps id
#define    check_empty_bg_white_bmp                           0
#define    check_empty_bg_darkgrey_bmp                     1
#define    check_empty_bg_lightgrey_bmp                     2
...

### #endif //_BITMAP_H_
```

Restriction: 

bitmap files has to be encoded 8 bits (256 color bitmap).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

### MAX78000 Feather operations

The TFT display is optional and not supplied with the MAX78000 Feather board.

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered from here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

To compile code with enabled TFT feature use following setting in project.mk:

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset.

