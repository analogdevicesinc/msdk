## Description

This example demonstrates how to transmit a set of audio samples over the I2S bus. The transmission is configured to send 16-bit samples in stereo mode at a 16kHz sampling rate. The I2S signals are available to view on pins P0.2-P0.5.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   To view I2S signals, connect a logic analyzer or oscilloscope to pins P0.2 (Data Out), P0.4 (BCLK), and P0.5 (LRCLK).

## Expected Output

```
****************** I2S Transmission Example ******************
This example demonstrates how to transmit a set of audio samples
over the I2S bus. The transmission is configured to send 16-bit
samples in stereo mode at a 16kHz sampling rate. The I2S signals
are available to view on pins P0.2-P0.5.

I2S Initialized.
Transmitting audio samples...
I2S Transmission Complete.
```

