## Description

This application demonstrates I2C communication between the MAX78000FTHR Application Platform and an ADXL343 Digital MEMS Accelerometer.  The application first configures the I2C peripheral instance, probes the I2C bus for the presence of a ADXL343, configures the ADXL343, waits for console input then enters low power mode.  The ADXL343 configured to enable Data Ready interrupts on pin INT2.  The INT2 signal is used as an external interrupt source capable of waking the MAX78000 from sleep mode.  Acceleration data is printed to the console UART on each interrupt.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Setup

### Prepare Hardware:

-   Connect INT2 on the ADXL343 to header J4.9, labeled "3".
-   Connect ADXL343 I2C pins to the header pins J4.11 and J4.12, labeled "SDA" and "SCL" respectively.

### Required Connections:

-   Connect a USB cable between the PC and the MAX78000FTHR CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the MAX78000FTHR console UART at 115200, 8-N-1.

## Expected Output

The console UART of the MAX78000FTHR will output these messages:

```
MAX78000FTHR I2C ADXL343 demo.
Press Enter/Return to continue ...
x:-0.02  y: 0.02  z: 0.99
```
