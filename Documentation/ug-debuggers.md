# Debuggers

## Debug Limitations

- ???+ warning "**⚠️ Warning**"
    It’s important to note some fundamental limitations of debugging the MSDK's supported parts. These limitations may make the device difficult (or impossible) for the debugger to connect in certain states. In such cases, the device can attempt to be recovered using a [MAX32625PICO](#max32625pico-pico).  See [How to Unlock a Microcontroller That Can No Longer Be Programmed](#how-to-unlock-a-microcontroller-that-can-no-longer-be-programmed)
    * A debugger can not be connected to a microcontroller _while_ the device is in reset.
    * A microcontroller can not be debugged while it's _Sleep_, _Low Power Mode_, _Micro Power Mode_, _Standby_, _Backup_, or _Shutdown_ mode.  These modes shut down the SWD clock.
        * For low-power development, it's recommended to place a 2-second blocking delay at the start of `main`.

## MAX32625PICO (PICO)

A MAX32625PICO (affectionately called the "PICO") debug adapter is provided with almost all the evaluation platforms supported by the MSDK.  Additionally, most small form-factor evaluation kits have a PICO _embedded_ into the PCB.

It's good practice to update the PICO's firmware to the latest version, which can be found on the [MAX32625PICO Firmware Images](https://github.com/MaximIntegrated/max32625pico-firmware-images) Github page.

### Updating the MAX32625PICO (PICO) Debug Adapter Firmware

1. Download the correct image for your evaluation platform from the [MAX32625PICO Firmware Images](https://github.com/MaximIntegrated/max32625pico-firmware-images) Github page.

2. Connect the included micro-USB cable to the PICO _without_ connecting the other side of the cable to your host PC yet.

    ![Pico USB Connection](res/pico_partial_connected.jpg)

3. Press and hold the pushbutton on the top of the PICO.

    ![Pico Pushbutton](res/pico_pushbutton.jpg)

4. _While holding down the pushbutton on the PICO_ connect the other side of the micro-USB cable to your host PC.

    Keep the pushbutton held down until the LED on the PICO blinks and becomes solid.

    ![Pico Connected](res/pico_connected.jpg)

5. A `MAINTENANCE` drive should now appear on your file system.

    ![Maintenance Drive](res/MAINTENANCE.jpg)

6. Drag and drop the downloaded file from step 1 onto the `MAINTENANCE` drive. This will flash the PICO with the updated firmware.

    ![Maintenance Drive](res/MAINTENANCE.jpg)

    ![Drag and Drop](res/drag_and_drop.JPG)

    ![Flashing](res/pico_flashing.JPG)

7. Once the flashing is complete, the PICO will restart and present itself as a `DAPLINK` drive.

    ![DAPLINK Drive](res/DAPLINK.jpg)

8. Open the `DAPLINK` drive.

    ![Opened DAPLINK Drive](res/DAPLINK_opened.jpg)

9. Open the `DETAILS.TXT` file and verify the Git SHA matches the expected value for the updated file.

    ![DETAILS.TXT](res/DETAILS_Git_SHA.jpg)

10. Your PICO debugger is now ready to use with the latest firmware.

### How to Unlock a Microcontroller That Can No Longer Be Programmed

The [debug limitations](#debug-limitations) of the MSDK's supported parts may make some devices difficult to connect to if bad firmware has been flashed. In such cases, the device can attempt to be recovered from the "locked out" firmware by mass erasing the application code from the flash memory bank. Note that this does not always work. Success will depend on a small window being available for the debugger to connect immediately after reset.

Before following the procedure below, ensure that you have updated the PICO debugger firmware to the latest version. See [Updating the MAX32625PICO (PICO) Debug Adapter Firmware](#updating-the-max32625pico-pico-debug-adapter-firmware)

#### Unlock with VS Code

For VS Code users, the `"erase flash"` [build task](#build-tasks) can be used to attempt a mass erase.  If this task fails to recover the part, attempt the procedure below.

#### Unlock with `erase.act`

1. Connect the PICO debugger to the microcontroller to recover.

2. Connect the PICO debugger to the host PC with the included micro-USB cable.

3. Open the DAPLINK` drive on the host PC.

    ![DAPLINK Drive](res/DAPLINK.jpg)

4. Open the `DETAILS.TXT` inside of the `DAPLINK` drive in a text editor.

    ![DAPLINK Opened](res/DAPLINK_opened.jpg)

5. Verify that the “Automation allowed” field is set to 1.

    ![Automation Allowed](res/DETAILS_automation_allowed.jpg)

    ??? note "ℹ️ **Enabling Automation**"
        If this field is _not_ set to 1, follow the procedure below:

        1. Create a new _empty_ file, and save it as `auto_on.cfg`.

        2. Press and hold the pushbutton on top of the PICO.

            ![PICO Pushbutton](res/pico_pushbutton.jpg)

        3. _While holding the pushbutton_, drag and drop the `auto_on.cfg` file onto the `DAPLINK` drive.

            ![Drag and Drop File](res/auto_on.cfg.jpg)

        4. Continue holding the pushbutton until the file is finished transferring over, then release it.

        5. The PICO should power cycle, and the DAPLINK drive should re-appear with “Automation allowed” set to 1.

6. Power on the evaluation platform (if it isn’t already).

7. Create an empty file called `erase.act`.

8. Drag and drop the `erase.act` file onto the `DAPLINK` drive.

    ![Drag and Drop erase.act](res/erase.act.jpg)

9. The PICO debugger will attempt to mass erase the microcontroller's flash bank, which will completely wipe any application firmware that is programmed on the device.

    ???+ note "ℹ️ **Note**"
        If this process fails, a `FAIL.TXT` file will be present in the DAPLINK drive with an additional error message inside of it.  A failure is generally indicative of firmware that has completely shut down the debug port, or has entered low power loop immediately on power-up.  In these cases, it's not possible to recover the device.

10. Power cycle the microcontroller. It step 8 succeeded, it's blank and ready to re-program. The debugger should have no issues connecting to the device in this blank state.
