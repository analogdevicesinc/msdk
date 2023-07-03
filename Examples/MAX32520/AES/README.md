## Description

This application demonstrates both encryption and decryption using AES.  A block of data is encrypted.  The resulting encrypted data is then decrypted.  The new plain text is then compared with the original plain text to confirm they match.  This is repeated three times using a different key size each time.


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX SEL and TX SEL on headers JP7 and JP8.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** AES Example *****
Test Cipher Sync
Pass.
Test Cipher Async
Pass.
Test Cipher Sync
Pass.
Test Cipher Async
Pass.
Test Cipher Sync
Pass.
Test Cipher Async
Pass.
Test Cipher Sync
Pass.
Test Cipher Async
Pass.
Test Cipher Sync
Pass.
Test Cipher Async
Pass.
Test Cipher Sync
Pass.
Test Cipher Async
Pass.

Example Succeeded
```


