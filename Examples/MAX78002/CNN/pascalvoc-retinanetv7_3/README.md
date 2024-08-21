## Description

This demo showcases MAX78002 multi-class object detection with image data captured from the OV5640 CSI2 camera.  

After initialization, the demo program will continuously capture a 320x256 image from the OV5640 and pass it into the CNN accelerator.  Once the CNN inference completes, the NewHaven TFT display will show bounding boxes with class labels overlaid on the captured image.  Additional timing information can be viewed over the serial port.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project only supports the NewHaven NHD-2.4-240320CF TFT carrier board as its display.

* This project has only supports the OmniVision OV5640 camera, and has only been tested with the Pcam 5C module included with the MAX78002EVKIT.

## Required Connections

* Install JP41 (CSI2 CAM I2C EN)
* Install JP40 (HW PWUP/SW PWUP)
* Remove JP16 and JP17 (I2C1 SDA/I2C1 SCL)
    * This is required to ensure the I2C1 bus is pulled up properly for the CSI2 camera.  Duplicate pull-ups result in unreliable camera communication.

## Expected Output

Connecting to the serial port presented by CN2 (USB/UART) at a baud rate of 115200 (8-N-1) and running the example will show the following output:

```
*** Object Detection Demo (pascalvoc-retinanetv7_3) ***
Waiting...
Camera ID = 5640
Initializing SRAM...
RAM ID:
        MFID: 0x0d
        KGD: 0x5d
        Density: 0x02
        EID: 0x5877b90d
Capturing image...
Done! (took 93754 us)
Loading CNN...
Done! (took 108718 us)
Inference complete!  Approximate data loading and inference time: 232746 us
Starting NMS... 
Done!  (Took 174346 us)

Capturing image...
Done! (took 92402 us)
Loading CNN...
Done! (took 108720 us)
Inference complete!  Approximate data loading and inference time: 231393 us
Starting NMS... 
Done!  (Took 169444 us)

Capturing image...
Done! (took 72116 us)
Loading CNN...
Done! (took 108716 us)
Inference complete!  Approximate data loading and inference time: 211104 us
Starting NMS... 
Done!  (Took 168330 us)

...

```
