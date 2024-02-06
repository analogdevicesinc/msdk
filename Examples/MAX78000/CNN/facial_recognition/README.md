# MAX78000 Facial Recognition Demo

## Overview

This demo includes **FaceDetection** and **FaceID** CNN models and runs them sequentially on MAX78000FTHR board.  

The **FaceDetection** CNN model detects a face and draws a bounding box around it.  Then, the bounding box is used to crop the image, which is passed to the **FaceID** CNN model.

The **FaceID** model classifies the face against a database of known users.  For this purpose, the **FaceID** CNN model generates a 64-length embedding for the cropped input image, whose distance to whole embeddings stored for each subject is calculated. The image is identified as either one of these subjects or 'Unknown' depending on the embedding distances.

It should be noted that the **FaceID** model is loaded from an SD card due to memory constraints.  There isn't enough flash in the MAX78000 to store both (See [Loading Weights to SD Card](#loading-weights-to-sd-card)).

## Facial Recognition Demo Software

### Loading Weights to SD Card

This project loads weights for the **FaceID** CNN model from an SD card.  It expects a `weights_2.bin` in the root directory of a FAT32-formatted micro-SDHC card.  The required file can be found at [SDHC_weights/weights_2.bin](./SDHC_weights/weights_2.bin).  Load this file into the root directory of a FAT32 formatted SD card before running this project.

Alternatively, the [SDHC_weights](SDHC_weights) sub-project can be used to load the weights onto the SD card with the MAX78000FTHR.  See [SDHC_weights/README.md](SDHC_weights/README.md) for more details.

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project is supported on the MAX78000FTHR board only.

* This project supports displaying its results to a TFT display.  The TFT display is **disabled** by default since it's not supplied with the MAX78000 Feather board. The compatible 2.4'' TFT FeatherWing display can be ordered [here](https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing).  To _enable_ the display code, set `TFT_ENABLE = 1` in [project.mk](project.mk)

## Required Connections

* Load the CNN model's weights into the SD card (see [Loading Weights to SD Card](#loading-weights-to-sd-card)) and insert the SD card into the FTHR boards SD slot.
* Insert a micro-USB cable into CN1 of the FTHR board.

### MAX78000 Feather operations

The TFT display is optional and not supplied with the MAX78000 Feather board.

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered from here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

To compile code with enabled TFT feature, set `TFT_ENABLE = 1` in [project.mk](project.mk)

```Makefile
# Project config options (see README):
# -----------------------
TFT_ENABLE = 1
# -----------------------
```

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset.

<img src="Resources/feather_tft.jpg" style="zoom: 25%;" />

The FTHR board must be used in a specific orientation relative to the subject's face.  For example, a user taking a picture of one's own face should use the following orientation:

<img src="Resources/feather_orientation.jpg" style="zoom: 25%;" />

Status LEDs:
- RED: No face detected
- GREEN: Face detected
- YELLOW: Face detected, recapturing image

### Using Debug Terminal

The debug terminal shows more information on status and detected faces.

The USB cable connected to CN1 (USB/PWR) provides power and serial communication to  MAX78000 Feather board.

To configure PC terminal program select correct COM port and settings as follows:

![](Resources/terminal_setup.jpg)

The serial terminal messages will output the following information, which logs the results and timing for each fo the models.

```serial
MAX78000 Feather Facial Recognition Demo
I[main      : 106]      Initializing...

I[main      : 180]      Initializing SD Card...
I[sd        : 554]      Card inserted

I[sd        :  86]      SD card mounted.

I[sd        : 566]      SD Card Opened

I[main      : 183]      Launching face detection loop...
I[main      : 186]      -----
I[facedetection: 148]   Image Capture Time : 47ms
I[facedetection: 149]   FaceDetect Process Time : 74ms
I[facedetection: 150]   Total FaceDetect Time : 122ms
I[main      : 193]      Face detected!
I[faceid    : 369]      FaceID result: Unknown subject
I[faceid    :  97]      FaceID Processing Time : 326ms
I[main      : 203]      ----- (Total loop time: 470ms)
...
```



## Face Database Generation

This section describes how to add new pictures to the data base.

### Prerequisites:

Requires ai8x-training environment. Please refer to [ai8x-training](https://github.com/MaximIntegratedAI/ai8x-training?tab=readme-ov-file#installation) for environment installation instructions.

### Taking Face Pictures

In order to achieve best identification, following steps are recommended:

1. Phone or webcam can be used to take pictures.
2. Make sure the subject face is well illuminated.

3. Pictures should be a passport style, with subject's face covering about 60-70% of the area:

![](Resources/image1.png)

4. Use a light color background; make sure it is not underexposed, and there is no shadow in the background or face.

5. Take one picture with each of the following poses, total of 6-8 pictures:

   a) facing directly to camera (1 picture)

   b) tilting face slightly (5-10 degree) to right, left, up and down, but still looking at the camera (4 pictures)

   c) zoomed out face directly to camera covering about 20-30% of area (1 picture):

![](Resources/image2.png)

6. If needed, you may add additional 1-2 pictures with some facial expressions (eg. smile) directly facing the camera, similar to 5-a (1-2 pictures)

7. Save pictures in .jpg format with arbitrary names.

### Populate Face Images

Navigate to 'db' folder and create a folder for each subject in your database and copy each subject's photos into these folders. The name of the folder will be used as the detected subject's name. These photos must contain subject's face directed to the camera but do not need to be consisting only the face. The structure of the directory should be as the following.

```bash
├── db
    ├── Name1
    │    ├── Image1.jpg
    │    ├── Image2.jpg
    │    ├── Image3.jpg
    │    ├── Image4.jpg
    │    ├── Image5.jpg
    │    └── Image6.jpg
    └── Name2
         ├── Image1.jpg
         ├── Image2.jpg
         ├── Image3.jpg
         ├── Image4.jpg
         └── Image5.jpg
```

Having at least 5 images per subject is recommended. There must be no other person in the images besides the subject. For the both the images in the database and in the operation time, the face should be well and evenly illuminated.

### Generate DB

Run the python script by:

```bash
$ sh gen_db.sh
```

The script updates embeddings.h file under 'include' folder using the images under 'db' folder.  Next, you can rebuild the project and load the firmware as described before:

```bash
cd$ make clean
$ make
```



## CNN Model Design
### Problem Definition
To identify people from 3 channel (RGB) frontal facial images, i.e. portraits. Small amount of rotations should be considered to have a robust application.

### Approach
Detailed description of the approach can be found in [Facial Recognition System](https://github.com/MaximIntegratedAI/ai8x-training/blob/develop/docs/FacialRecognitionSystem.md) document.

## References

[1] https://github.com/MaximIntegratedAI/MaximAI_Documentation
