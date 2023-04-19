## Description

Two pulse trains are configured in different modes.  

The first, PT2, is set to generate a repeating bit pattern of 0x10110 (lsb first) at a rate of 2 bits per second.  If you make the connections described below, you can observe the pattern on LED 1.

The second, PT3, is set to generate a 10Hz square wave.  If you make the connections described below, you can observe the square wave on LED 2.

On the standard EV Kit:
-    PT2: P1.8
-    PT3: P1.9

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pin 1.8 to pin 1 of JP5 (LED1_EN).
-   Connect pin 1.9 to pin 1 of JP6 (LED2_EN).

## Expected Output

The Console UART of the device will output these messages:

```
*************** Pulse Train Demo ****************
PT2 (P1.8) = Outputs continuous pattern of 10110b at 2bps
PT3 (P1.9) = Outputs 10Hz continuous square wave
```

