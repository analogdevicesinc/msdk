## Description

This example demonstrates the performance benefits of using Instruction Cache Controller in conjunction with SPIXF.

A sample function which performs a multiplication operation repeatedly is loaded into the MX25 external flash chip. The function is then called twice, once with the SFCC enabled and once with the SFCC disabled. For each function call, the RTC is used measure execution time and the results are compared after both tests have completed.

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

