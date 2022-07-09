## Description

This application demonstrates both encryption and decryption using AES.  A block of data is encrypted.  The resulting encrypted data is then decrypted.  The new plain text is then compared with the original plain text to confirm they match.  This is repeated three times using a different key size each time.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Select RX0 and TX0 on Headers JP1 and JP3 (UART 0).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** AES Example *****

AES 128 bits Key Test
Data Verified

AES 192 bits Key Test
Data Verified

AES 256 bits Key Test
Data Verified

Example Succeeded
```


