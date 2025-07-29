## Description

Python script can be used to send image file to MAX78000 EVKIT or Feather board
and receive calculated segmentation mask

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be
found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Example Usage

Load and run Spectrum Sensing demo on MAX78000 EVKIT or Feather board.
Next, run the host application to send the image:

```
$ python SerialLoader.py -c COM6 -b 115200 -i LTE_frame_12.png -o CNNout.txt
```

Parameters:
-c/--com - COM Port. Default COM52
-b/--baud - Baud rate, Default 115200
-i/--input - Input file. Default LTE_frame_12.png
-o/--output - Output file. Default CNNout.txt

Default baud rate is 115200bps. If you would like to change it, don't forget
to update the board side too (#define CON_BAUD 115200 in main.c).
