## Description

This example demonstrates the use of the DES encryption and decryption features of the Trust Protection Unit. 

To demonstrate, encryption and decryption operations are executed with the TPU and compared with the expected results of those operations. This repeated for both DES and Trple DES encryption and decryption.


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
***** DES Example *****
DES ECB Encryption ... Pass.
DES ECB Decryption ... Pass.
Triple DES ECB Encryption ... Pass.
Triple DES ECB Decryption ... Pass.

Example complete.
```
