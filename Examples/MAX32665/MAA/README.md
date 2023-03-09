## Description

TBD<!--TBD-->


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
***** MAA Example *****
Computed:
result[0] = 0x76543210
result[1] = 0xfedcba98
result[2] = 0x9abcdef0
result[3] = 0x12345678

Expected:
expected[0] = 0x76543210
expected[1] = 0xfedcba98
expected[2] = 0x9abcdef0
expected[3] = 0x12345678

Example Succeeded
```
