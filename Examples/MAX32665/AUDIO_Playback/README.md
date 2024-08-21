## Description

This example utilizes the audio subsystem peripheral to receive and transmit audio data via the I2S protocol.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect microphone to LINE_IN jack in order to use line in input as audio source.

## Expected Output
```
************************ Audio Subsystem I2S Example ************************


In this example, the device reads audio data simultaneously from the LINE_IN input or 
the Digital Microphone, then writes it to HD_PHONE using the MAX9867 Audio Codec. 

This example utilizes the audio subsystem peripheral to receive and transmit audio 
data via the I2S protocol. If you wish to use 'LINE_IN' as the input source, 
you must connect a microphone to the 'LINE_IN' port and listen to the sound using 
headphones connected to 'HD_PHONE'. 

Alternatively, you can switch the input source to the Digital Microphone by 
setting the 'lineIn' global variable to 'false'. 

It's important to note that the Digital microphone input source is only compatible 
with MAX32666 EV KIT REV D. 
```
