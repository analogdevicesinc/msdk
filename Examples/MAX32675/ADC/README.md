## Description

This example demostrates the use of the ADC on the MAX32675.

Each of the ADC modules are set to sample differentially between ADC channels AIN2(+) and AIN3(-), with ADC0 configured with an 8x input gain and ADC1 configured with a 16x input gain. Samples are continuously collected and every 50 samples collected, the average of those 50 samples is printed to the terminal. The reference voltage is set to 3.3V, thus the difference between the inputs cannot exceed 3.3V.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-	Connect the negative input to AIN3 and the postive input to AIN2.

## Expected Output

The Console UART of the device will output these messages:

```
MAX32675 ADC Example

ADC0 and ADC1 are set to sample differentially between AIN2(pos)
and AIN3(neg). Both ADCs are configured to a sampling rate of 120
samples per second. ADC0 has a PGA gain of 8X for its input, ADC1 uses 16X
input gain. ADC sample data is stored in the DATA0 register of each ADC.
This example gets 50 samples from each, and calculates and prints
the average reading.

Sampling will begin after 5 seconds...
ADC0 AVG val: 0063D3D2
ADC1 AVG val: 0069C518

ADC0 AVG val: 0063D50D
ADC1 AVG val: 0069C633
```
