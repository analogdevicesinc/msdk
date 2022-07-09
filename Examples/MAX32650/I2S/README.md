## Description

This example demonstrates of the SPIMSS I2S functionality. To demonstrate, an audio signal is output to the I2S pins (P2.2, P2.4 and P2.5). 


## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect audio signals to pins P2.2 (Bit Clock), P2.4 (Data Out), and P2.5 (LR Clock).

## Expected Output

The Console UART of the device will output these messages:

```
*********** I2S Example **********
I2S Configured
Starting I2S Output
Muting I2S Output
Unmuting I2S Output
Pausing I2S Output
Resuming I2S Output
Stopping I2S Output. Example Complete.
```


