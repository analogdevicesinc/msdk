# MAX78000 ASL Recognition Demo v.1

# Overview

The ASL Demo software demonstrates recognition of American Sign Language hand symbols using MAX78000 EVKIT.

The ASL demo software utilizes an ASL dataset which consists of 26 hand symbols, one for each letter in the alphabet, and a total of 81000 images. Below is the pre-augmented dataset that was downloaded and used for training and testing:

https://www.kaggle.com/grassknoted/asl-alphabet

The following 27 keyword subset from the complete dataset is used for this demo:

 **[‘a’, ‘b’, ‘c’, …. ‘y’, ‘z’, ‘empty’]**

In the demo, “empty” represents blank images missing hand symbols.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

* This project supports output to a TFT display.
    * For the MAX78000EVKIT, the TFT display is **enabled** by default.  It can be _disabled_ by commenting out `#define ENABLE_TFT` in [example_config.h](example_config.h).

        ```C
        #ifdef BOARD_EVKIT_V1
        // #define ENABLE_TFT
        #include "bitmap.h"
        #include "tft_ssd2119.h"
        #endif
        ```

    * For the MAX78000FTHR, the TFT display is **disabled** by default.  The TFT display is not supplied with the MAX78000 Feather board. The compatible 2.4'' TFT FeatherWing display can be ordered [here](https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing).  To _enable_ the display code, uncomment `#define ENABLE_TFT` in [example_config.h](example_config.h)

        ```C
        #ifdef BOARD_FTHR_REVA
        // #define ENABLE_TFT
        #include "tft_ili9341.h"
        #endif
        ```

## Hardware

### MAX78000EVKIT operations                                 [ ](af://n152/)

After power-cycle, if the TFT display is blank, or not shown properly as below, please press RESET (SW5).

The TFT display shows that it is ready. Press PB1 to start:

![TFT_display](Resources/TFT_display.jpg)

Once RED LED2 turns on, the initialization is complete and ready to accept keywords. If PICO adapter is still connected to SWD, disconnect it and power cycle.

The following 27 symbols can be detected:

 **[‘a’, ‘b’, ‘c’, …. ‘y’, ‘z’, ‘empty’]**

 The MAX78000 ASL demo firmware recognizes keywords and reports result and confidence level.

### MAX78000FTHR operations                               [ ](af://n152/)

The ASL demo starts automatically after power-up or pressing reset button (SW4).

While using TFT display keep its power switch in "ON" position. The TFT "Reset" button also can be used as Feather reset. Press PB1 (SW1) button to start demo.

## CNN Model

The ASL Convolutional Neural Network (CNN) model consists of **2D** CNN with 6 layers and one fully connected layer to recognize 27 different hand symbols used for training, with 64x64 images as input.

![CNN_code_part1](Resources/CNN_code_part1.png)



![CNN_code_part2](Resources/CNN_code_part2.png)

![CNN_code_part3](Resources/CNN_code_part3.png)



# Network Training                                 [ ](af://n199/)

To invoke network training execute the script:

```bash
(ai8x-training) $ ./scripts/train_asl.sh
```

Details of network training methodology are described in [AI8X Model Trainin](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)[g](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)[ and ](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)[Q](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)[uantization ](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)After training unquantized network can be evaluated by executing script:

```bash
(ai8x-training) $ ./scripts/evaluate_asl.sh
```

## Network Quantization

The CNN weights generated during training need to be quantized:

```bash
(ai8x-synthesis) $ ./scripts/quantize_asl.sh
```

Details of quantization are described in [AI8X Model Training and Quantization](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)

## Network Synthesis

The network synthesis script generates a pass/fail C example code which includes necessary functions to initialize MAX78000 CNN accelerator, to load quantized CNN weights and input samples and to unload classification results. 

```bash
(ai8x-synthesis) $ ./gen_asl.sh
```

## References                                       [ ](af://n235/)

[https://](https://github.com/MaximIntegratedAI/MaximAI_Documentation)[g](https://github.com/MaximIntegratedAI/MaximAI_Documentation)[ithub.com/MaximInte](https://github.com/MaximIntegratedAI/MaximAI_Documentation)[g](https://github.com/MaximIntegratedAI/MaximAI_Documentation)[ratedAI/MaximAI_Documentation](https://github.com/MaximIntegratedAI/MaximAI_Documentation)