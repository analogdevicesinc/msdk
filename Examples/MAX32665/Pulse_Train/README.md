## Description

Two pulse trains are configured in different modes.  

The first, PT14 (P1.14), is set to generate a repeating bit pattern of 0x10110 (lsb first) at a rate of 2 bits per second.  If you make the connections described below, you can observe the pattern on P1.14.

The second, PT15 (P1.15) , is set to generate a 10Hz square wave.  If you make the connections described below, you can observe the square wave on P1.15.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Pulse Train Demo ****************
LED0 = Outputs continuous pattern of 10110b at 2bps
LED1 = Outputs 10Hz continuous square wave
Push button 0 = Stop/Start all pulse trains
```