## Description

This example captures an image using the camera (e.g. OV5640) via MIPI CSI-2 Protocol, saves it into external SRAM and streams to your PC through the COM port. Press PB1 to capture an image once the device is ready.

Use the [utils/console.py](utils/console.py) script to grab the camera data and create a PNG image of the captured image. More information can be found in [utils/README.md](utils/README.md).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Expected Output

```shell
$ python utils/console.py COM6

Started ImgCapture console and opened COM6
MCU: *SYNC*
MCU: Established communications with host!
MCU: Registered 5 total commands:
MCU: -----
MCU: 'help' : Print this help string
MCU: 'reset' : Issue a soft reset to the host MCU.
MCU: 'capture' : Perform a standard blocking capture of a single image
MCU: 'set-reg' <register> <value> : Write a value to a camera register.
MCU: 'get-reg' <register> : Prints the value in a camera register.
MCU: -----
MCU:
MCU:
MCU: **** MIPI CSI-2 Example ****
MCU: This example streams the image data through the COM port
MCU: and a script running on the host pc converts the data into
MCU: a .png image.
MCU:
MCU: Go into the pc_utility folder and run the script:
MCU: python console.py [COM#]
MCU:
MCU: Press PB1 (SW4) or send the 'capture' command to trigger a frame capture.
MCU:
MCU: Initializing camera...
MCU: Camera ID = 5640
MCU: Initializing SRAM...
MCU: RAM ID:
MCU: MFID: 0x0d
MCU: KGD: 0x5d
MCU: Density: 0x02
MCU: EID: 0x588e670d
MCU: Capturing image...
MCU: Done! (took 73321 us)
MCU: Sending image over serial port...
MCU: *IMG* RGB565 153600 320 240
Collecting 153600 bytes...
$
```

## Required Connections

- Connect CSI2 camera module to J8 (CSI CAMERA)
- Set JP38 to OFF position (DVP CAM PWR)
- Connect a USB cable between the PC and the CN2 (USB/UART) connector.
- Install jumper JP41 (CSI2 CAM I2C EN)
- Install jumper JP40 (HW PWUP/SW PWUP)
- Remove JP39 (SW CAM PWUP)
- Remove JP16 and JP17 (I2C1 SDA/I2C1 SL)
    - This is required to ensure the I2C1 bus is pulled up properly for the CSI2 camera.  Duplicate pull-ups result in unreliable camera communication.
- Install jumpers (RX - P0.0) and (TX - P0.1) at Header JP20 (UART 0 EN).
- Open an terminal application on the PC and connect to the EV kit's console UART t 115200, 8-N-1.
- Install jumper JP44 (LED0 EN).
- Install jumper JP45 (LED1 EN).
- Install jumper JP25 (PB1 PU).
- Connect the 5V power cable at (5V IN).
