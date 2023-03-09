## Description

Two pulse trains are configured in different modes.  

The first, PT14, is set to generate a repeating bit pattern of 0x10110 (lsb first) at a rate of 2 bits per second.  If you make the connections described below, you can observe the pattern on LED0. Note: the pattern will be inverted (1's will appear as LED Off and vice versa).

The second, PT15, is set to generate a 10Hz square wave.  If you make the connections described below, you can observe the square wave on LED1.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
- 	Connect P2.28 (PT14) -> PT2.25 (LED0)
-	Connect P0.23 (PT15) -> PT2.26 (LED1)
-	Ensure LED enable jumpers (EN0, EN1) are connected.

## Expected Output

The Console UART of the device will output these messages:

```
*************** Pulse Train Demo ****************

Please make the following connections: P2.28->P2.25 and P0.23->P2.26
LED0 = Outputs continuous pattern of 10110b at 2bps
LED1 = Outputs 10Hz continuous square wave
SW3  = Stop/Start all pulse trains
```