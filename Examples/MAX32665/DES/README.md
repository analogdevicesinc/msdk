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

***** DES Example *****

Demonstrating DES ECB (electronic code book) encryption...
Data verified.

Demonstrating DES CBC (cipher block chaining) encryption...
Data verified.

Demonstrating DES CFB (cipher feedback) encryption...
Data verified.

Demonstrating DES ECB (electronic code book) decryption...
Data verified.

Demonstrating DES CBC (cipher block chaining) decryption...
Data verified.

Demonstrating DES CFB (cipher feedback) decryption...
Data verified.

Demonstrating 3DES ECB (electronic code book) encryption...
Data verified.

Demonstrating 3DES CBC (cipher block chaining) encryption...
Data verified.

Demonstrating 3DES CFB (cipher feedback) encryption...
Data verified.

Demonstrating 3DES ECB (electronic code book) decryption...
Data verified.

Demonstrating 3DES CBC (cipher block chaining) decryption...
Data verified.

Demonstrating 3DES CBC (cipher feedback) decryption...
Data verified.

Example Succeeded.

```
