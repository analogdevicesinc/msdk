## Description

This example demonstrates how to take temperature readings from the ADT7420 using the I2C.

After startup, a new reading is printed to the terminal every second.

NOTE: This example is only supported on the MAX32655EVKIT.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT and is not yet supported by the MAX32655FTHR kit.

## Required Connections

If using the MAX32655EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.16 (SCL), P0.17 (SDA), VDD and GND to corresponding pins of EVAL-ADT7420MBZ 

## Expected Output

```
****************** I2C ADT7420 TEMPERATURE SENSOR DEMO *******************

Make sure that the SCL and SDA pins of the EVAL-ADT7420MBZ board are connected to P0.16 and P0.17 on the MAX32655EVKIT

The Device ID is: 0xCB

-->Temperature: 24.125000 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.187500 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.187500 °C
-->Temperature: 24.187500 °C
-->Temperature: 24.125000 °C
-->Temperature: 24.187500 °C
-->Temperature: 24.125000 °C

...
```


