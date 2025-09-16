## Description

This application demonstrates both encryption and decryption using AES.  A block of data is encrypted.  The resulting encrypted data is then decrypted.  The new plain text is then compared with the original plain text to confirm they match.  This is repeated three times using a different key size each time.



## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

* This project comes pre-configured for the MAX32655EVKIT.  See [Board Support Packages](https://analogdevicesinc.github.io/msdk/USERGUIDE/#board-support-packages) in the MSDK User Guide for instructions on changing the target board.

##  Required Connections
If using the MAX32655EVKIT (EvKit\_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins JP4(RX_SEL) and JP5(TX_SEL) to RX0 and TX0  header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX32655FTHR (FTHR\_Apps\_P1):
-   Connect a USB cable between the PC and the J4 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the board's console UART at 115200, 8-N-1.

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


