## Description

This example demonstrates entering and exiting from the various operating modes. The operating modes which are used can be enabled or disabled by setting the DO_SLEEP, DO_LPM, DO_UPM, DO_BACKUP, and DO_STANDBY defines to 1 or 0 respectively. The mecahanism to switch to the next operating mode, either the RTC or the push button, is selected with the USE_ALARM and USE_BUTTON defines, only one may be selected at a time.

## Software

### WARNING: Low-Power Mode Flash / Debug Access

Microcontrollers usually shutdown SWD /JTAG debug interfaces in very low-power mode operation.

This can cause an MCU to permanently lose debug access because, upon reset, the device may enter LP modes so fast that the debug probe cannot attach to and reset the target core before the debug interface is shutdown. This is true regardless of the debug probe you choose to use.

To avoid this problem, please observe the 2-second delay at the beginning of the main() function. If you find that your MCU can no longer be programmed or debugged after flashing an LP example, you can attempt to recover it using the following procedure from the MSDK User Guide:
https://analogdevicesinc.github.io/msdk//USERGUIDE/#how-to-unlock-a-microcontroller-that-can-no-longer-be-programmed

Another option for production builds would be to implement Lockout Protection using a free GPIO attached to a switch. Below is some example code for how this may be implemented:

```C
#ifdef LOCKOUT_PROTECT
/**
 * If debugger lockout protection is enabled,
 * define a switch for enabling the protection.
 *
 * Note this is not a board-specific configuration;
 * Please adapt the GPIO pin(s) to your specific hardware.
 **/
static const mxc_gpio_cfg_t lockoutConfig = {
    .func = MXC_GPIO_FUNC_ALT1,
    .port = MXC_GPIO0,
    .mask = MXC_GPIO_PIN_0,
    .pad = MXC_GPIO_PAD_PULL_UP
};
#endif

int main(void) {
#ifdef LOCKOUT_PROTECT
    MXC_GPIO_Config(&lockoutConfig);

    // If lockout switch is pulled low, prevent sleep or trigger a long delay
    if (MXC_GPIO_InGet(lockoutConfig.port, lockoutConfig.mask)) {
        MXC_Delay(MXC_DELAY_SEC(2));
        allowSleep = 0;
    }
#endif
```

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

(None - this project builds as a standard example)

## Required Connections

If using the MAX32690EVKIT:
-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Install JP7(RX_EN) and JP8(TX_EN) headers.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the MAX32690FTHR:
-   Connect a USB cable between the PC and the J5 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

If using the AD-APARD32690-SL:
-   Connect a USB cable between the PC and the P10 (USB-C) connector.
-   Connect a MAXPICO Debug adapter to P9 (SWD Connector)
-   Open a terminal application on the PC and connect to the MAXPICO's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages if USE_ALARM is defined:

```
****Low Power Mode Example****

This code cycles through the MAX32690 power modes, using the RTC alarm
to exit from each mode.  The modes will change every 2 seconds.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.
Entering SLEEP mode.
...
```

The Console UART of the device will output these messages if USE_BUTTON is defined:

```
****Low Power Mode Example****

This code cycles through the MAX32690 power modes. Use push button (SW2)
to exit from each power mode and enter the next.

Running in ACTIVE mode.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering LPM mode.
Waking up from LPM mode.
Entering SLEEP mode.
...
```
