## Description

This application sends data between two serial ports on the MAX32672. UART2 transmits data and UART1 receives it. The data received is compared to the data trnasmitted at the end to verify the transaction was successful.

By default the reading UART does an asynchronus read, however it can instead do the operation using DMA by defining "DMA" at the top of *main*.  

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect P0.28 to P0.15 (AIN7).

## Expected Output

The Console UART of the device will output these messages:

```
********************* UART Example ***********************
This example sends data from UART2 to UART1 to another.

If the transaction succeeds, the green LED will illuminate.
If the transaction fails, the red LED will illuminate.

Connect UART1 RX (P0.28) to UART2 TX (P0.15, AIN7).

-->UART Baud    : 115200 Hz
-->Test Length  : 1024 bytes

-->Reading UART Initialized
-->Writing UART Initialized

-->Starting transaction
-->Transaction complete

-->Data verified

-->Example Succeeded
```

The green LED will illuminate for successful transaction.
The red LED will illuminate if transaction failed.
