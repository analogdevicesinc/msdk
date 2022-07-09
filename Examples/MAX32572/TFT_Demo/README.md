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

- bitmap files has to be encoded 8 bits (256 color bitmap).