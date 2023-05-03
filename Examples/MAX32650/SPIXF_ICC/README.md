## Description

This example demonstrates the performance benefits of using Instruction Cache Controller in conjunction with SPIXF.

A sample function which performs a multiplication operation repeatedly is loaded into the MX25 external flash chip. The function is then called twice, once with ICC1 enabled and once with ICC1 disabled. For each function call, the timer is used measure execution time and the results are compared after both tests have completed.


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
********************* SPIXF/ICC1 Example *********************
This example demonstrates the performance benefits of using the
instruction cache controller when executing from the MX25 external
flash chip.

MX25 Initialized.
Loading test function into external flash.

Setup complete. Press SW2 to run Instruction Cache test.

Running test function with ICC1 enabled.
Test function complete.
Time elapsed 0:0.373

Running test function with ICC1 disabled.
This will take a few minutes...
Test function complete.
Time elapsed 1:58.346

Example Succeeded
```

