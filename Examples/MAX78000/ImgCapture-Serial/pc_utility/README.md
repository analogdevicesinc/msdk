## Description

Python scripts designed for use with the "CameraIF" firmware that can be used to grab images and convert it to .png on the host PC.

## Requirements

Python 3.9+

Then, install Python packages using the `requirements.txt` file in this folder.

Ex:

```shell
pip install -r requirements.txt
```

To grab camera output run:  python grab_image.py <your_comport> <baudrate>
default baudrate is 921600, if you would like to change it do not forget 
to update the board side too.

## Usage

1. Ensure the latest CameraIF is flashed to the host MCU with `TFT_ENABLE` **un**-defined in `example_config.h`.

2. Ensure the firmware is running, and that the LED on the host MCU is flashing.  This is the firmware searching for the host.

3. Connect the Python console to the host MCU firmware using `console.py`.  Ex:

    ```shell
    python console.py <COM port>
    ```

    Alternatively, use `python console.py -h` to see a list of console start-up options.

4. Verify a successful connection, which should look similar to the following:

    ```shell
    Started CameraIF console.  Type 'help' for help, 'quit' to quit.
    Available commands:
            'quit' : Quits this console
            'help' : Prints this help string
            'reset' : Issue a soft reset to the host MCU.
            'capture' : This command will perform a blocking capture of a single image.
            'imgres' <width> <height> : Set the image resolution of the camera to <width> x <height>
            'stream' : Performs a line-by-line streaming DMA capture of a single image.
            'set-reg' <register> <value> : Write a value to a camera register.
                    Auto-converts all integer types (hex, binary, etc.)
                    Ex: set-reg 0x11 0b1
            'get-reg' <register> : Prints the value in a camera register.
                    Auto-converts all integer types (hex, binary, etc.)
                    Ex: get-reg 0x11
    MCU: *SYNC*
    MCU: Established communications with host!
    MCU: Registered 6 total commands:
    MCU: Command 0: 'reset'
    MCU: Command 1: 'capture'        
    MCU: Command 2: 'imgres'
    MCU: Command 3: 'stream'
    MCU: Command 4: 'set-reg'        
    MCU: Command 5: 'get-reg'        
    MCU: Initializing DMA
    MCU: Initializing camera
    MCU: Camera I2C slave address: 3c
    MCU: Camera ID detected: 7fa2
    MCU: Ready!
    ```

5. Enter any of the registered commands to excercise the example.  Any `capture`-d or `stream`-ed images will be saved to an `"Image.png"` file next to the `console.py` script.

