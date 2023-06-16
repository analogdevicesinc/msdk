## Description

This example demonstrates QSPI communication in drivers written for the APS6404 external SRAM IC on the MAX78002EVKIT.  It also excercises the APS6404 by reading and validating various test patterns and demonstrates the speed increases using QSPI vs traditional SPI.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds and runs as a standard example)

## Expected Output

```shell
QSPI SRAM Test:
        Test Address: 0x0
        Test size: 640 bytes
        Test count: 480 rows
        Test speed: 24000000 Hz
Reading ID...
RAM ID:
        MFID: 0x0d
        KGD: 0x5d
        Density: 0x02
        EID: 0x576c8d0d
(Benchmark) Wrote 640 bytes to internal SRAM in 5us
Test 1: Standard SPI write...
        Done (640 bytes in 202us)
Test 2: Validate w/ standard SPI...
        Read finished (640 bytes in 204us)
        Checking for mismatches...
        Done
Test 3: Validate w/ QSPI...
        Read finished (640 bytes in 87us)
        Checking for mismatches...
        Done
Test 4: QSPI Write...
        Done (640 bytes in 94us)
Test 5: Validate w/ standard SPI...
        Read finished (640 bytes in 220us)
        Checking for mismatches...
        Done
Test 6: Validate w/ QSPI...
        Read finished (640 bytes in 87us)
        Checking for mismatches...
        Done
Test 7: QSPI Writing across page boundaries...
        Wrote 307200 bytes in 45408us
Test 8: Validating with standard SPI...
        Success
Test 9: Validating with QSPI...
        Success
Success!
```

## Required Connections

- Connect a USB cable between the PC and the CN2 (USB/PWR) connector and connect a serial terminal at 115200 BAUD rate (8-N-1) to view the console output.
