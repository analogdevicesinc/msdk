## Description

This example demonstrates with the N01S830HA external SRAM IC on the MAX78000FTHR.  It also excercises the N01S830HA by reading and validating various test patterns and demonstrates the speed increases using QSPI vs traditional SPI.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project is only supported on the MAX78000FTHR board.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-	Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

```shell
QSPI SRAM Test:
        Test Address: 0x0
        Test size: 4096 bytes
        Test speed: 25000000 Hz
(Benchmark) Wrote 4096 bytes to internal SRAM in 20us
Test 1: Standard SPI write...
        Done (4096 bytes in 1364us)
Test 2: Standard SPI read...
        Read finished (4096 bytes in 1363us)
        Checking for mismatches...
        Success.
Test 3: QSPI write...
        Done (4096 bytes in 544us)
Test 4: QSPI read...
        Read finished (4096 bytes in 377us)
        Checking for mismatches...
        Success.
```
