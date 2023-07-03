## Description

The SPI Execute-in-Place Flash (SPIXF) peripheral enables code stored in an external flash chip to be executed transparently. The SPIXF comes with a dedicated cache controller, the SPIXF Cache Controller (SFCC), that enables faster code execution when executing instructions stored in external flash. The purpose of this example is to demonstrate the improvement in code execution time when the SFCC is enabled.

To demonstrate, a sample function is loaded into the MX25 external flash chip. The function is then called twice, once with the SFCC enabled (Test 1) and once with the SFCC disabled (Test 2). For each function call, the RTC measures the execution time of each function call. After both function calls have completed, the execution times are compared. If Test 1 has a faster execution time than Test 2 a success message will be printed to the terminal.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
********************* SPIXF/SFCC Example *********************
This example demonstrates the performance benefits of enabling the
SFCC when executing from the MX25 external flash chip.

MX25 Initialized.
Loading test function into external flash.

Setup complete. Press SW2 to run SFCC test.

Running test function with SFCC enabled.
Test 1 Complete!
Execution Time: 1.519s

Running test function with SFCC disabled.
This will take a few minutes...
Test 2 Complete!
Execution Time: 94.310s

Example Succeeded
```

