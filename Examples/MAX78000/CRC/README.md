## Description

This example demonstrates the use of the HW CRC calculator.  The example first generates a 32-bit CRC over 4096 bytes.  The resulting CRC is inverted and placed at the end of the 4096+4 bytes.  Another CRC is then calculated over the resulting 4096+4 bytes.  The final result is compared with the expected value.  This sequence is repeated twice - once using the blocking CRC functions and once using the non-blocking (asynchronous) functions.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX78000EVKIT.  See [Board Support Packages](https://analog-devices-msdk.github.io/msdk/USERGUIDE/#board-support-packages) in the UG for instructions on changing the target board.

## Required Connections:

If using the MAX78000EVKIT (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX78000FTHR (FTHR_RevA)
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
CRC Sync and Async Example

TEST CRC SYNC

Computed CRC: 13b5a483
CRC Check Result: debb20e3
**Test Passed**

TEST CRC ASYNC

Computed CRC: 13b5a483
CRC Check Result: debb20e3
**Test Passed**


Example Succeeded
```
