## Description

A basic getting started program using the Cortex-M33 TrustZone feature.

This version of Hello_World prints an incrementing count to the console UART and toggles a LED0 every 500 ms from the Non-Secure world. Incrementing the count is done in the Secure world.

Under the **Hello_World_TZ** example, the **Secure** and **NonSecure** projects demonstrate building the secure and non-secure code into a single image. You do not build directly in the **NonSecure** project.

Refer to the user guide for more information about TrustZone.

## Software

### Project Usage

Universal instructions on building, flashing, and debugging this project can be found in the **[MSDK User Guide](https://analogdevicesinc.github.io/msdk/USERGUIDE/)**.

### Project-Specific Build Notes

The **Secure** project must have the **partition_max32657.h** file included for the SAU setup done in `SystemInit()` at startup.

The Secure Project must set these flags:
- `TRUSTZONE=1`
- `MSECURITY_MODE=SECURE`
- `NONSECURE_CODE_DIR=path/to/nonsecure/project/dir`

The **partition_max32657.h** file SAU regios must match the linker script memory sections.

## Required Connections
-   Connect a USB cable between the PC and the J1 (PWR-OBD-UART0) connector.
-   Connect pins JP19 (OBD VCOM EN) RX and TX header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

LED0 toggles every 500ms.

```
**** Hello_World example with TrustZone ****
Currently in the secure world.
Beginning transition to the non-secure.

Hello from Non-Secure World!
count = 1
count = 2
count = 3
count = 4
count = 5
```
