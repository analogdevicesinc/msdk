## Description

Python script can be used to send image file to MAX78000 EVKIT or Feather board and receive calculated

segmentation mask



## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Example Usage

Load and run AI-segmentation demo on MAX78000 EVKIT or Feather board. Next, run the host application to send the image:

```
$ python SerialLoader.py image1.png COM52 115200
```

Parameters: 

**image1.png** - input image file

**COM52** - serial port

**115200** - serial port baud rate

Default baud rate is 115200bps. If you would like to change it, don't forget 
to update the board side too (#define CON_BAUD 115200 in main.c).
