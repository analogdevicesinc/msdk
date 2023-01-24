# MAX78000 Keyword Spotting Demo v.3.2



## Overview

The Keyword Spotting Demo software demonstrates recognition of a number of keywords using MAX78000 EVKIT.  

A new option `-DSEND_MIC_OUT_SDCARD` has been added to [`project.mk`](project.mk) to enable saving the detected sound snippets to SD card interface of the MAX78000 Feather board. This feature is not available for MAX7800 EVKIT.

The KWS20 demo software utilizes 2nd version of the Google speech commands dataset which consists of 35 keywords and more than 100K utterances:

https://storage.cloud.google.com/download.tensorflow.org/data/speech_commands_v0.02.tar.gz



The following 20 keyword subset from the complete dataset is used for this demo:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'yes', 'no', 'on', 'off', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'zero**']

The rest of the keywords and unrecognized words fall into the "**Unknown**" category. 

To improve the unknown detection, the model in version 3.2 is trained with an additional speech dataset from LibriSpeech (http://us.openslr.org/resources/12/dev-clean.tar.gz), segmented to 1-sec audio data and labeled as unknown.

## Keyword Spotting Demo Software

### Building firmware

Navigate the directory where the KWS20 demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/kws20_demo
$ make
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make distclean
```

To compile code for MAX78000 EVKIT enable **BOARD=EvKit_V1** in [`project.mk`](project.mk):

```bash
# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
#BOARD=FTHR_RevA
endif
```

To compile code for the MAX78000 Feather board enable **BOARD=FTHR_RevA** in [`project.mk`](project.mk):

```bash
# Specify the board used
ifeq "$(BOARD)" ""
#BOARD=EvKit_V1
BOARD=FTHR_RevA
endif
```

**Note: If you are using Eclipse, please also make sure to change the value of the `BOARD` environment variable to `FTHR_RevA` by right-clicking on *"[project name] > Properties > C/C++ Build > Environment > BOARD"***

<img src="Resources/eclipse_board.png" style="zoom:33%;" />



### Load firmware image to MAX78000 EVKIT

Connect the USB cable to CN1 (USB/PWR) and turn ON the power switch (SW1).

Connect the PICO adapter to the JH5 SWD header.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/MAX78000.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

**Make sure to remove the PICO adapter once the firmware is loaded.**

### MAX78000 EVKIT jumper setting

Make sure to install the jumper at JP20-CLK (INT position) as shown below:

<img src="Resources/I2S_jumper.png" style="zoom:25%;" />

Note: On board, external oscillator Y3 is used to generate an I2S clock. The I2S sample rate is 16kHz to match speech samples of the dataset.

### MAX78000 EVKIT operations

After power-cycle,  if the TFT display is blank, or not shown properly as below, please press RESET (SW5).

The TFT display shows that it is ready. Press PB1 to start:

<img src="Resources/20200604_142849.jpg" style="zoom: 25%;" />



Once RED LED2 turns on, the initialization is complete and it is ready to accept keywords. If the PICO adapter is still connected to SWD, disconnect it and power cycle.

The following words can be detected:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'yes', 'no', 'on', 'off', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'zero**']

 The MAX78000 KWS20 demo firmware recognizes keywords and reports result and confidence level.

The microphone (U15) is located between JH4 and JH5 headers on EVKIT, (MK1) between J5 and J7 audio connectors on the MAX78000 Feather board.



<img src="Resources/20200604_142536_1.jpg" style="zoom:25%;" />



### Load firmware image to MAX78000 Feather

Connect the USB cable to the CN1 USB connector.

If you are using Windows, load the firmware image with OpenOCD in a MinGW shell:

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/MAX78000.elf reset exit"
```

If using Linux, perform this step:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### MAX78000 Feather operations

The KWS20 demo starts automatically after power-up or pressing the reset button (SW4).
The TFT display is optional and not supplied with the MAX78000 Feather board.
Users should use the PC terminal program to observe the KWS20 demo result as described in the "Using Debug Terminal" section.

The MAX78000 Feather compatible 2.4'' TFT FeatherWing display can be ordered here:

https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing

This TFT display comes fully assembled with dual sockets for MAX78000 Feather to plug into.

To compile code with the enabled TFT feature use the following setting in [`project.mk`](project.mk):

```bash
ifeq "$(BOARD)" "FTHR_RevA"
PROJ_CFLAGS += -DENABLE_TFT
endif
```

***Note: If the SD card option is enabled, the TFT support will automatically be disabled regardless of the above setting.***

While using the TFT display keep its power switch in the "ON" position. The TFT "Reset" button also can be used as Feather reset.
Press PB1 (SW1) button to start the demo.

<img src="Resources/feather_tft.jpg" style="zoom:25%;" />

The PB1 (SW1) button is located as shown in the picture below:

![](Resources/pb1_button.jpg)



### Using Debug Terminal

Debug terminal shows more status information and detected words. 

The USB cable connected to CN1 (USB/PWR) provides power and serial communication.

To configure the PC terminal program select the correct COM port and settings as follow:

![](Resources/Terminal2.png)

After turning on power or pressing the reset button the following message will appear in the terminal window:

![](Resources/Terminal1.png)



Terminal display after detecting words:

![](Resources/Terminal3.png)



The software components of the KWS20 demo are shown in the diagram below:

![](Resources/Diagram.png)



## CNN Model

The KWS20 v.3 Convolutional Neural Network (CNN) model consists of **1D** CNN with 8 layers and one fully connected layer to recognize keywords from 20 words dictionary used for training.

```python
class AI85KWS20Netv3(nn.Module):
    """
    Compound KWS20 v3 Audio net, all with Conv1Ds
    """

    # num_classes = n keywords + 1 unknown
    def __init__(
            self,
            num_classes=21,
            num_channels=128,
            dimensions=(128, 1),  # pylint: disable=unused-argument
            bias=False,
            **kwargs

    ):
        super().__init__()
        self.drop = nn.Dropout(p=0.2)
        # Time: 128 Feature :128
        self.voice_conv1 = ai8x.FusedConv1dReLU(num_channels, 100, 1, 
                                                stride=1, padding=0,
                                                bias=bias, **kwargs)
        # T: 128 F: 100
        self.voice_conv2 = ai8x.FusedConv1dReLU(100, 96, 3, 
                                                stride=1, padding=0,
                                                bias=bias, **kwargs)
        # T: 126 F : 96
        self.voice_conv3 = ai8x.FusedMaxPoolConv1dReLU(96, 64, 3, 
                                                       stride=1, padding=1,
                                                       bias=bias, **kwargs)
        # T: 62 F : 64
        self.voice_conv4 = ai8x.FusedConv1dReLU(64, 48, 3, 
                                                stride=1, padding=0,
                                                bias=bias, **kwargs)
        # T : 60 F : 48
        self.kws_conv1 = ai8x.FusedMaxPoolConv1dReLU(48, 64, 3, 
                                                     stride=1, padding=1,
                                                     bias=bias, **kwargs)
        # T: 30 F : 64
        self.kws_conv2 = ai8x.FusedConv1dReLU(64, 96, 3, 
                                              stride=1, padding=0,
                                              bias=bias, **kwargs)
        # T: 28 F : 96
        self.kws_conv3 = ai8x.FusedAvgPoolConv1dReLU(96, 100, 3, 
                                                     stride=1, padding=1,
                                                     bias=bias, **kwargs)
        # T : 14 F: 100
        self.kws_conv4 = ai8x.FusedMaxPoolConv1dReLU(100, 64, 6, 
                                                     stride=1, padding=1,
                                                     bias=bias, **kwargs)
        # T : 2 F: 128
        self.fc = ai8x.Linear(256, num_classes, bias=bias, wide=True, **kwargs)

    def forward(self, x):  # pylint: disable=arguments-differ
        """Forward prop"""
        # Run CNN
        x = self.voice_conv1(x)
        x = self.voice_conv2(x)
        x = self.drop(x)
        x = self.voice_conv3(x)
        x = self.voice_conv4(x)
        x = self.drop(x)
        x = self.kws_conv1(x)
        x = self.kws_conv2(x)
        x = self.drop(x)
        x = self.kws_conv3(x)
        x = self.kws_conv4(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)

        return x
```

The CNN input is 128x128=16384 8-bit signed speech samples.

## Network Training

To invoke network training execute the script:

```bash
(ai8x-training) $ ./scripts/train_kws20_v3.sh
```

If this is the first time, and the dataset does not exist locally, the scrip will automatically download the Google speech commands dataset (1-second keyword .wav files, sampled at 16KHz, 16-bit) into /data/KWS/raw, and process it to make appropriate training, test and validation dataset integrated into /data/KWS/process/dataset2.pt. To improve the detection of unknown keywords, it also downloads an additional speech dataset from librispeech and segments them into 1-second audio files to be labeled as unknown. The processing step expands the training dataset by using augmentation techniques like adding white noise, random time shift, and stretch to improve training results. In addition, each 16000 sample word example is padded with zeros to make it 128x128=16384 speech samples. The augmentation process triples the size of the dataset and could take 30min to complete.

Details of network training methodology are described in [AI8X Model Training and Quantization](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)

After training unquantized network can be evaluated by executing the script:

```bash
(ai8x-training) $ ./scripts/evaluate_kws20_v3.sh
```



## Network Quantization

The CNN weights generated during training need to be quantized:

```bash
(ai8x-synthesis) $ ./scripts/quantize_kws20_v3.sh
```

Details of quantization are described in [AI8X Model Training and Quantization](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)

## Network Synthesis

The network synthesis script generates a pass/fail C example code which includes necessary functions to initialize the MAX78000 CNN accelerator, load quantized CNN weights and input samples, and unload classification results. A sample input with the expected result is part of this automatically generated code to verify.  The following script generates all example projects including **kws20_v3**:

```bash
(ai8x-synthesis) $ ./gen-demos-max78000.sh
```

The **kws20_v3** bare-bone C code is partially used in KWS20 Demo. In particular, CNN initialization, weights (kernels), and helper functions to load/unload weights and samples are ported from **kws20_v3** to KWS20 Demo.



## KWS20 Demo Code

KWS20 demo works in two modes:  Using a microphone (real-time), or offline processing:

```c
#define ENABLE_MIC_PROCESSING
```

### Microphone Mode

In this mode, EVKIT I2S Mic is initialized to operate at 16KHz 32-bit samples.  In the main loop, the I2S buffer is checked and samples are stored in  **pChunkBuffonboard** buffer.  

### CODEC Mode

In this mode, the left channel (tip of the J5 3.5mm audio jack) of the line-in of MAX9867 audio CODEC (is used as the audio input source.

 To enable using CODEC as the audio input source, make sure the `PROJ_CFLAGS` line is uncommented. This mode can only be enabled for the  Feather board.

```make
# If enabled, it captures audio from line input of MAX9867 audio codec instead of the on-board mic.
# Note that SEND_MIC_OUT_SDCARD should be disabled in this mode
PROJ_CFLAGS+=-DENABLE_CODEC_MIC
```

### Offline Mode

if **ENABLE_MIC_PROCESSING** is not defined, a header file containing the 16-bit samples (e.g. **kws_five.h**) should be included in the project to be used as the input . To create a header file from a wav file, use included utilities to record a wav file and convert it to the header file. 

```bash
# record 1sec of 16-bit 16KHz sampled wav file 
$ python VoiceRecorder.py -d 1 -o voicefile.wav
# convert to header
$ python RealtimeAudio.py -i voicefile.wav -o voicefile.h
```

### Saving Sound Snippets to SD Card

The following option has been added to [`project.mk`](project.mk). To enable saving the detected sound snippets to the SD card make sure the `PROJ_CFLAGS" line is uncommented.

```make
# If enabled, it saves out the Mic samples used for inference to SDCARD
PROJ_CFLAGS+=-DSEND_MIC_OUT_SDCARD
```

When this mode is enabled, a new sequential directory is created on the SD card on every power-up or reset.
![directory](Resources/SDcard_files.PNG)

After a few moments, the green LED lights up and upon detecting a new word, the LED blinks and a file is created with 8-bit sample recorded audio. The file name includes an index and the detected word. If the detection has low confidence, the file name will have a "_L" suffix. (for example `0003_RIGHT_L`)

![snippets](Resources/soundSnippet.PNG)

The LED

- stays *green* when it is listening
- blinks *green* if a keyword is detected
- blinks *yellow* if detection confidence is low or unknown keyword
- stays *red* if there is an error in the SD card interface

A utility (`bin2wav.py`) is provided in the `/Utility` folder to convert these files into wave (.wav) format to listen to.
To convert individual files:

```bash
$ python bin2wav.py -i <sound snippet file>
```

To convert all the files in the current directory and all subdirectories:

```bash
$ python bin2wav.py -a
```

To convert all the files in a directory and all its subdirectories:

```bash
$ python bin2wav.py -a -d <folder name>
```

When option `-a` is used, each file is converted to a .wav file once and subsequent execution of the command skips all the files that have previously been converted to wave files.

***Note 1: When `SEND_MIC_OUT_SDCARD` is selected, the Wake-Up Timer (WUT) is disabled.***

***Note 2: When `SEND_MIC_OUT_SDCARD` is selected, the `ENABLE_TFT` is disabled regardless of make options.***

### Sending Sound Snippets to serial

To send the snippets to the serial port in binary format, uncomment the following line in [`project.mk`](project.mk). 

```make
# If enabled, it sends out the Mic samples used for inference to the serial port
PROJ_CFLAGS+=-DSEND_MIC_OUT_SERIAL
```

A utility (`capture_serial_bin.py`) is provided in the `/Utility` folder to capture the serial snippets and save them as  .wav files:

```bash
$ python capture_serial_bin.py -c COM3 -o out.wav
```

The snippets will be stored with incremental tags.

### KWS20 Demo Firmware Structure

The following figure shows the processing in KWS20 Demo firmware:

![](Resources/KWS_Demo_flowchart.png)

Collected samples from mic/file are 18/16 bit signed and are converted to 8-bit signed to feed into CNN. If Microphone mode, a high pass filter is used to filter out the DC level in captured samples. Scaled samples are stored in **micBuff** circular buffer in chunks of 128 samples (bytes). 

The following parameters in the firmware can be tuned:

```c
#define SAMPLE_SCALE_FACTOR    		4		// multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH				350  	// voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW				100  	// voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD 	20 		// [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE				30*CHUNK// how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD   		49 		// min probability (0-100) to accept an inference
```

When the average absolute values of samples during the last 128 samples go above a threshold, the beginning of an utterance is marked. 

The end of a word is signaled when the **SILENCE_COUNTER_THRESHOLD** back-to-back chunks of samples with an average absolute threshold lower than **THRESHOLD_LOW** are observed. 

The CNN requires 1sec worth of samples (128*128) to start processing. This window starts at **PREAMBLE_SIZE** samples before the beginning of the word and ends after 16384 samples. If the end of a word is determined earlier, the pAI85Buffer sample buffer is padded with zeros.

The CNN-related API functions are in **cnn.c**. They are used to load weights and data, start CNN, wait for CNN to complete processing, and unload the result. 

If a new network is developed and synthesized, the new weight file and related API functions are needed to be ported from the automatically generated kws20 example project. Furthermore, if the input layer or organization of 128x128 sample sets in the trained network is changed, **AddTranspose()** function should be changed to reflect the new sample data arrangement in CNN memory.

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation
