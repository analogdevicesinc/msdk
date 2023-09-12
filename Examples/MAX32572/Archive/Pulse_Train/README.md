## Description

Two pulse trains are configured in different modes.  

The first, PT0, is set to generate a repeating bit pattern of 0x10110 (lsb first) at a rate of 2 bits per second.  If you make the connections described below, you can observe the pattern on LED1.

The second, PT1, is set to generate a 10Hz square wave.  If you make the connections described below, you can observe the square wave on LED2.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect External LEDs to P0.13 and P0.12

## Expected Output

The Console UART of the device will output these messages:

```
*************** Pulse Train Demo ****************
LED0 = Outputs continuous pattern of 10110b at 2bps
LED1 = Outputs 10Hz continuous square wave
    
Connect external LEDS to see the demo.
LED0 to P0.13 and LED1 to P0.12
```