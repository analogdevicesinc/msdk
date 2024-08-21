## Description

This example demonstrates the use of the External Memory Cache Controller.

The same external memory operations are performed with both the cache enabled and the cache disabled. The time it takes to complete each set of operations is printed to the terminal.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** EMCC Example *****

Running test reads with data cache enabled.   Time elapsed: 2.021
Running test reads with data cache disabled.  Time elapsed: 200.245
Example complete.
```


