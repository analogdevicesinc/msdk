## Useful WSL documentation

https://learn.microsoft.com/en-us/windows/wsl/filesystems

## usbipd setup

Follow [WSL Setup](https://github.com/dorssel/usbipd-win/wiki/WSL-support#wsl-setup)

## Symptoms

Mount USB device with `usbipd wsl attach -b <busid>`

USB device shows up successfully with `lsusb`

openocd fails to connect to PICO debugger running git SHA a209 with error `[ 6222.907293] hid-generic 0003:0D28:0204.000C: device has no listeners, quitting`

Listing the Linux kernel configuration with `zcat /proc/config.gz > wsl_kernel.config` shows `CONFIG_HIDRAW` is disabled

## Recompiling the kernel

Clone the [WSL2-Linux-Kernel](https://github.com/microsoft/WSL2-Linux-Kernel)

Install Ubuntu build dependencies

`sudo apt install build-essential flex bison dwarves libssl-dev libelf-dev`

Edit the kernel configuration

`make menuconfig KCONFIG_CONFIG=Microsoft/config-wsl`

Enable `CONFIG_HIDRAW`

Recompile the kernel

`make KCONFIG_CONFIG=Microsoft/config-wsl`

Copy the kernel onto Windows filesystem

`cp arch/x86/boot/bzImage /mnt/c/Users/<username>/wsl_kernel_mod`

Edit Windows-side [.wslconfig](https://learn.microsoft.com/en-us/windows/wsl/wsl-config#configure-global-options-with-wslconfig) to use the new kernel.

```config
[wsl2]
kernel = C:/Users/<username>/wsl_kernel_mod
```

On Windows command prompt, shut down wsl

`wsl --shutdown`

Restart WSL and reattach usb device

`usbipd.exe wsl attach -b 5-10`

Verify device connects successfully

`dmesg | tail`

```bash
[  595.414780] usb 1-1: new full-speed USB device number 3 using vhci_hcd
[  595.494812] vhci_hcd: vhci_device speed not set
[  595.564789] usb 1-1: SetAddress Request (3) to port 0
[  595.707665] usb 1-1: New USB device found, idVendor=0d28, idProduct=0204, bcdDevice=10.00
[  595.707669] usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  595.707670] usb 1-1: Product: DAPLink CMSIS-DAP
[  595.707670] usb 1-1: Manufacturer: ARM
[  595.707671] usb 1-1: SerialNumber: 04441701dde703f600000000000000000000000097969906
[  595.721771] cdc_acm 1-1:1.1: ttyACM0: USB ACM device
[  595.736740] hid-generic 0003:0D28:0204.0002: hidraw0: USB HID v1.00 Device [ARM DAPLink CMSIS-DAP] on usb-vhci_hcd.0-1/input3
```
