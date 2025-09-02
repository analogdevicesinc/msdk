## Description

This application demonstrates the creation, use, and clearing of the device's unique PUF keys for AES encryption.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) jumpers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the AD-APARD32690-SL:
-   Connect a USB cable between the PC and the P10 (USB-C) connector.
-   Connect a MAXPICO Debug adapter to P9 (SWD Connector)
-   Open a terminal application on the PC and connect to the MAXPICO's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** PUF Example *****

***** AES256 ECB Encryption with NULL Key *****
Ciphertext:
DD C6 BF 79 0C 15 76 0D 8D 9A EB 6F 9A 75 FD 4E

PUF KEY0 Generation: Success

***** AES ECB Encryption with PUF Key0 *****
Ciphertext:
16 F6 7F D7 9E 35 5B 43 5A 4F B0 64 6C B7 38 7A

PUF KEY1 Generation: Success

***** AES ECB Encryption with PUF Key1 *****
Ciphertext:
B3 2F 64 09 D4 59 47 50 D4 71 12 28 38 3F 0D BC

PUF KEY0 and KEY1 Generation: Success

***** AES ECB Encryption with PUF Key0 *****
Ciphertext:
16 F6 7F D7 9E 35 5B 43 5A 4F B0 64 6C B7 38 7A

***** AES ECB Encryption with PUF Key1 *****
Ciphertext:
B3 2F 64 09 D4 59 47 50 D4 71 12 28 38 3F 0D BC

***** AES ECB Encryption with Cleared PUF Key0 *****
Ciphertext:
DD C6 BF 79 0C 15 76 0D 8D 9A EB 6F 9A 75 FD 4E

***** AES ECB Encryption with Cleared PUF Key1 *****
Ciphertext:
DD C6 BF 79 0C 15 76 0D 8D 9A EB 6F 9A 75 FD 4E

Example Succeded
```
