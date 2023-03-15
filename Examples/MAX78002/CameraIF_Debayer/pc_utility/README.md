## Description

Python scripts can be used to grab images and convert it to .png on PC side
Existing codes was tested and work as expected, tested with python 3.7
You may need to install png and serial and opencv python library on your machine.

To grab camera output run:  python grab_image.py <your_comport> <baudrate>
default baudrate is 921600, if you would like to change it do not forget 
to update the board side too.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Requirements

These requirements are needed to run the scripts
pySerial
opencv-python
Pillow

A requirements.txt can be found in this directory.  Use `pip install -r requirements.txt` to install all dependencies.

## Example Usage

sudo python3 grab_image.py COM42 921600
