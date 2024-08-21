## Description

This example configures the SPI to send data between the MISO (P0.6) and
MOSI (P0.5) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.  To switch to non-blocking (asynchronous) transactions, undefine the SYNC macro and define the ASYNC macro. 


## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

-   Connect a USB cable between the PC and the USB connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect P0.10 (MISO) to P0.11 (MOSI).

## Expected Output

```
************** SPIMSS Master Loopback Demo ****************
This example configures the SPIMSS to send data between the MISO (P0.10) and
MOSI (P0.11) pins.  Connect these two pins together.  This demo shows SPIMSS
sending between 1 and 16 bits of data at a time.  During this demo you
may see junk data printed to the serial port because the console UART
shares the same pins as the SPIMSS.

Sent 1 bits per transaction
Sent 2 bits per transaction
Sent 3 bits per transaction
Sent 4 bits per transaction
Sent 5 bits per transaction
Sent 6 bits per transaction
Sent 7 bits per transaction
Sent 8 bits per transaction
Sent 9 bits per transaction
Sent 10 bits per transaction
Sent 11 bits per transaction
Sent 12 bits per transaction
Sent 13 bits per transaction
Sent 14 bits per transaction
Sent 15 bits per transaction
Sent 16 bits per transaction

Test successful!
```
