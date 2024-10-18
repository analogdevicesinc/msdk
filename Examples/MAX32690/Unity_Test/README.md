## Description

A basic unit testing example that leverages the Unity Test Project.  In this example, a function that adds two `uint8_t` is tested.  The tests can be run on the target microcontroller, or from the developer's host PC.

Additional documentation on the Unity Test Project and its integration with the MSDK can also be found in the [MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/).

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

Alternatively, the unit tests can be run from the developer's host PC with `make test`.

### Project-Specific Build Notes

- Run `make test` to run the host-side unit tests.  This will pull from the `test` folder inside the project and run them on the developer's host PC.

## Required Connections

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Close jumper JP5 (LED1 EN).
-   Close jumper JP6 (LED2 EN).

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
-----------------------
2 Tests 0 Failures 0 Ignored 
OK
```

