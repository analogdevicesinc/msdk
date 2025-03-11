## Description

This example demonstrates the various low power modes on the MAX32672.

The device is capable of entering the Sleep, Deep Sleep, Backup and Storage low power modes. The user may select which low power modes to enter in this example by setting the DO\_SLEEP, DO\_DEEPSLEEP, DO\_BACKUP, and DO\_STORAGE defines at the top of main.

Either the push button or the RTC may be used as a wakeup source in this example. Select which one is used by setting either the USE\_BUTTON or USE\_ALARM defines at the top of main.

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

This example uses a special linkerfile (either lp-nonsecure.ld or lp-sla.ld depending on whether Secure Boot Tools are enabled) that limits the SRAM usage to SRAM0 and SRAM1. This prevents a hardfault that would otherwise occur when the other SRAMs are shutdown in the example.

## Required Connections

-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   To achieve lowest current measurements, remove jumpers JP2-JP11 and disconnect the LCD.

## Expected Output

The Console UART of the device will output these messages:

```
************ Low Power Mode Example ************

This code cycles through the MAX32672 power modes, using a push button (SW3) to exit from each mode and enter the next.

Running in ACTIVE mode.
All unused RAMs placed in LIGHT SLEEP mode.
All unused RAMs shutdown.
Entering SLEEP mode.
Waking up from SLEEP mode.
Entering DEEPSLEEP mode.
Waking up from DEEPSLEEP mode.
```
