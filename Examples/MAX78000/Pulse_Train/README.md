## Description

Two pulse trains are configured in different modes.  

The first is set to generate a repeating bit pattern (default is 0b10110) at a user configurable bit rate (default 2bps). The pattern and bit rate can be changed with the "CONT\_WV\_PATTERN" and "CONT\_WV\_BPS" defines respectively.

The second is set to generate a square wave at a frequency defined by "SQ\_WV\_HZ".

On the Standard EV Kit, the continuous bit pattern and square wave signals are output to the P0.18 (PT0, pin 12 on header J4) and P0.19 (PT1, pin 4 on header DSP2) header pins respectively. If the connections described below are made, the signals will be visible on LEDs 1 and 2.

On the featherboard, the continuous bit pattern and square wave signals are output to the P0.19 (PT1, pin 9 on header J4) and P0.16 (PT2, pin 11 on header J4) header pins respectively.

##Setup

##### Building Firmware:

Before building firmware you must select the correct value for _BOARD_  in "project.mk", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pin 12 of JH4 (Camera) to pin 2 of JP1 (LED1).
-   Connect pin 4 of the DSP2 connector to pin 2 of JP2 (LED2).

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect logic analyzer to pin 9 (P0.19) on header J4 to view the continuous bit pattern
-   Connect logic analyzer to pin 11 (P0.16) on header J4 to view the square wave

## Expected Output

The Console UART of the device will output these messages for the Standard EV Kit:

```
************************ Pulse Train Demo ************************
PT0 (P0.18, pin 12 on camera header J4) = Continuous pattern of 0x16 at 2bps
PT1 (P0.19, pin 4 on display header DSP2) = 10Hz square wave
```