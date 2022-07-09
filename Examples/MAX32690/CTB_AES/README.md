## Description

This application demonstrates both encryption and decryption using the Crypto Toolbox's AES.  A block of data is encrypted.  The resulting encrypted data is then decrypted.  The new plain text is then compared with the original plain text to confirm they match.  This is repeated three times using a different key size each time.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install RX\_EN (JP7) and TX\_EN (JP8) jumpers.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** AES Example *****
AES 128-bit Key Test --> PASS
AES 192-bit Key Test --> PASS
AES 256-bit Key Test --> PASS
```


