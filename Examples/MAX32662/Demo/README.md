## Description

This example demonstrates the use of the TFT Display, UART Terminal, RTC, Pushbuttons, and LEDs on the MAX32662EVKIT.

Features:
	1. Displays the Analog Devices Logo when the example starts.
	2. Displays the uptime of the board in the format: (hhh:mm:ss).
	3. LED Toggles at 1Hz by default, or 2Hz if the pushbutton is held down.
	4. Displays the chip info for 3 seconds if the pushbutton is pressed 5 times.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analog-devices-msdk.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

This project is only supported on the MAX32662EVKIT.

## Required Connections
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Install headers JP7(RX\_EN) and JP8(TX\_EN).
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Display Output

The display will show the Analog Devices logo for several seconds before displaying the uptime of the board in the format: (hhh:mm:ss) for (Hours:Minutes:Seconds).

Pressing the button 5 or more times will wipe the screen and display the chip information (USN and Revision) for 3 seconds before the uptime is displayed up again.

## Expected Output

The Console UART of the device will output these messages:

```
**** MAX32662 EV Kit Demo ****

(hhh:mm:ss): 000:00:00


(hhh:mm:ss): 000:00:01


(hhh:mm:ss): 000:00:02


(hhh:mm:ss): 000:00:03

```
