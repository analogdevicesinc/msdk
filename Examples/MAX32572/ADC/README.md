## Description

Demonstrates the use of the ADC by continuously monitoring ADC input channel 0.  Vary the voltage on the AIN0 input (0 to 0.9V) to observe different readings from the ADC.  

High and low limits are set arbitrarily to demonstrate the detection of overvoltage and undervoltage conditions respectively.  If the ADC reading exceeds 0x300, the example will report that the high limit has been reached.  If the ADC reading falls below 0x25, the example will report the low limit has been reached.

Any reading that exceeds the full-scale value of the ADC will have an '*' appended to the value.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect a USB cable between the PC and the CN1 (USB/UART0) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
ADC Example
0: 0x0001


0: 0x003f


0: 0x0001


0: 0x0001


0: 0x0289


0: 0x0001
```
