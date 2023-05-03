## Description

This example demonstrates the capabilities of the I2S peripheral. This example demonstrates how to record audio from the on-board digital microphone and play the recorded audio back out through the headphone jack.

Once the green LED is illiuminated the system is ready to begin recording. Recording will start when button SW3 is pressed and will continue until the button is released or the external memory is full.

After the recording has finished the audio will begin playing after a 3 second delay. 


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Plug in headphones to the headphone jack (connector J3).

## Expected Output

```
******************* I2S Record/Playback Example *******************

This example demonstrates how to record audio from the on-board
digital microphone and play the recorded audio back out through
the headphone jack.

Once the green LED is illiuminated the system is ready to begin
recording. Recording will start when button SW3 is pressed and will
continue until the button is released or the external memory is
full.

After the recording has finished the audio will begin playing after
a 3 second delay.

Erasing external flash. This will take a moment...
Erase complete.

Press and hold SW3 to start recording.

Recording started.
Recording ended.

Playback starting.
Playback ended.
```

