@page NFC_PCD_RF_Driver_APIOverviewAndUsage NFC PCD RF Driver

@section NFC_PCD_RF_Driver_introPage Introduction                               

This document describes the MAX32570 NFC PCD RF Driver library, also known as the RF Driver, provides an overview for new developers, and details integration specifics. The primary purpose of this library is to act as a Hardware Abstraction Layer (HAL) providing a straightforward interface to the complex underlying hardware without loss of functionality and configurability.

As part of an EMV Contactless stack the MAX32570 NFC PCD RF Driver library sits below Level 1 as shown in \ref NFC_PCD_RF_Driver_figure1_overview "Figure 1". It is designed to be agnostic of the software above it providing low level routines that can quickly be interfaced to any existing EMV L1 software stacks.

@anchor NFC_PCD_RF_Driver_figure1_overview
@image html ./NFC_EMV_SW_Stack.bmp "Figure 1: EMV Contactless Stack Overview" width=40% height=40%

@section NFC_PCD_RF_Driver_apiPage API

Full API details are found in @ref NFC_PCD_RF_DRIVER. The below sections provide an overview of the functionality.

The RF driver provides the following features:
- Field Control: Power Up or Down the 13.56Mhz Field
- Transceive: Send an NFC command and attempt to receive a response
- Configure Analog settings: Modify field power level, Transmit modulation depth, Receive gain and Receive trigger levels
- Provide routines to detect the current field level, and allows for analog paramter modification based on the deteted level
- Generation of Types A,B and F CRC

Although the RF driver is primarily designed to sit underneath a higher-level stack, it may be directly used to handle non EMV commands and cards.  It is designed to provide a flexible interface to the contactless hardware.  For instance if the application requires ISO 14443-3B style modulation and signaling but, also requires a different CRC polynomial, the built in CRC generation and validation can be handled via the calling application, simply by using @ref FT_STANDARD_NO_CRC_NO_EMD as the frame type. 

@subsection settingsPage Analog Configuration

Every EMV/NFC Contactless application is unique.  Due to the wide variety of physical implementations it is necessary to tune for the optimal analog performance.  The @ref mml_nfc_pcd_set_analog_config function provides a convenient method for changing the analog parameters to maximize individual application performance.  

@note Before attempting to tune the analog parameters in the RF driver it is __required__ to select an appropriate antenna for the application, and tune the external antenna matching network.

@see <a href="MAX32570_PCD_AntennaMatching_Design_Guide.pdf"><b>MAX32570 PCD Antenna Matching Design Guide</b></a>
@see <a href="MAX32570_NFC_PCD_AFE_Tuning_Guide.pdf"><b>MAX32570 NFC PCD AFE Tuning Guide</b></a>

@section NFC_PCD_RF_Driver_portPage RF Driver Integration Guidance

While the RF driver is actively transmitting or receiving (calling @ref mml_nfc_pcd_transceive) it needs to be in near total control of the processor.  It must respond quickly to correctly timestamp RF events, such as RX start, and RX/TX done.

If not actively transmitting or receiving, the RF driver will continue to energize the RF field until commanded to disable it by the application. During this time, the RF Driver has no requirements for processing cycles. 

In a typical usage scenario, the field is powered up, then the NFC application sleeps for 5ms before polling for different card types A and B.  Between the polling commands, there is additional sleeps of 5ms. If a card is identified in the field, it will be activated and communicated with various delays between TX and RX events.  The typical duty cycle of active transmit and receive time, is less than 20%.

@subsection NFC_PCD_RF_Driver_resourcePage RF Driver Resource Usage

The RF Driver makes use of the several pieces of hardware on the MAX32570:

### Timers

<table>
<caption id="multi_row">RF Driver Timer Usage</caption>
<tr><th>Timer Instance                      <th>RF Driver Use        <th>Notes          <th>Fixed*
<tr><td rowspan="1">System Tick<td>FreeRTOS Scheduling, @ref mml_nfc_pcd_task_sleep, Yielding etc. <td>Part of Cortex M4 Core <td> No
<tr><td rowspan="1">Timer 0<td>Not Used<td> <td> No
<tr><td rowspan="1">Timer 1<td>Not Used<td> <td> No
<tr><td rowspan="1">Timer 2<td>Not Used<td> <td> No
<tr><td rowspan="1">Timer 3<td>PWM Signal for Piezo Buzzer<td>External Pin Tied to Buzzer<td> No
<tr><td rowspan="1">Timer 4<td>Not Used<td> <td> No
<tr><td rowspan="1">Timer 5<td>Not Used<td> <td> No
<tr><td rowspan="1">Timer 6<td>Software Timer, Non-Yeilding Delays<td>No External Pin<td> Yes
<tr><td rowspan="1">Timer 7<td><b>FWT</b> (Frame Wait Time) and <b>FDT_pcd</b> (Frame Delay Time) Timer, Timed Operations<td>No External Pin <td> Yes
<tr><td colspan="4">*Fixed timers are tied to the RF Driver and may not changed to other timer instances
</table>

The RF Driver takes over control of Timers 6 and 7 completely.  Timer 7 is used to validate the required __FDT__ and __FWT__ times for receptions.  For full details of __FDT__ and __FTW__ refer to the EMV L1 Specification.  At a basic level __FDT__ defines the minimum time before a reception is considered valid and __FWT__ is the maximum time to wait before timing out. __FDT_pcd__ refers to the time between the end of the last reception from a PICC and the beginning of the next transmission by the PCD.

For proper operation of the RF Driver, Timers 6 and 7 must not be used by any other software during NFC operations.

Timer 3 output pin is tied to a piezo buzzer on the EVKit, utilizing its PWM mode.  This is used in the interoperability mode, beeping short and higher picked for success, and longer, and lower pitched for failure.

The System Tick timer is a part of the Cortex M4 Core, and is utilized by FreeRTOS for scheduling purposes.  It may be used in single theaded applications, but is not demonstrated in the DTE example.  The MAX32570 also has an RTC (Real Time Clock) timer and a watchdog timer, please see the user's guide for more details.

### Contactless Peripheral

The Contactless Peripheral must only be used by the RF Driver.  Attempts to overwrite any of the registers in its memory area will result in unexpected and incorrect behavior.

### Contactless and Timer IRQs

The Contactless, and Timer 6 and 7 IRQ vectors and ISRs must only be used by the RF Driver.  Attempts to redirect or disable these IRQs will result in unexpected and incorrect behavior.

- In the single process DTE example:
    - Contactless IRQ priority is set to 0
    - FWT Timer IRQ priority is set to 0
- In the multi process FreeRTOS demo example:
    - Contactless IRQ priority is set to 1
    - FWT Timer IRQ priority is set to 1

These priorites may be overridden using the @ref mml_nfc_pcd_set_nfc_interrupt_priority and @ref mml_nfc_pcd_set_FWT_timer_interrupt_priority functions.  If priorites are changed, please take care to verify proper NFC operation is maintained.

@subsection portingFile Application Portability and RTOS Configuration

Use @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT to configure specific implementation routines for the library.

- For __Single Process Applications__, refer to the DTE Example @ref CSP_DTE_Example_Overview_page
    - @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT implemented by <code>mml_nfc_pcd_port</code> comes preconfigured with a basic configuration, which can be used in a single threaded system 
    - @ref mml_nfc_pcd_task_sleep only needs to delay, and in the default configuration uses routines provided by <b><code> timer_utils.h </code></b> but can easily be overridden based on the applications needs.
    - @ref mml_nfc_pcd_enter_critical and @ref mml_nfc_pcd_leave_critical simply disable/enable global interrupts in the default configuration.
    - @ref mml_nfc_pcd_set_nfc_interrupt_priority and @ref mml_nfc_pcd_set_FWT_timer_interrupt_priority functions are desribed above
    - The semaphore functions are setup to simply delay until the done flag is set by the RD Driver ISR
    - @ref mml_nfc_pcd_field_level_detection_callback is called by @ref mml_nfc_pcd_transceive to allow user code to configure the analog settings for the transaction.  This allows the implementer to decide which
        settings are ideal for this transaction by calling the @ref mml_nfc_pcd_detect_loading function and selecting a pre-tuned setting for the current loading level.  For more information regarding this tuning
        refer to <a href="MAX32570_NFC_PCD_AFE_Tuning_Guide.pdf">MAX32570 NFC PCD AFE Tuning Guide.</a>

- For __Multi-Process Applications__, refer to the FreeRTOS Demo @ref CSP_FREERTOS_Demo_Overview_page
    - @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT implmented by <code>mml_nfc_pcd_port</code> comes preconfigured with a full configuration, using FreeRTOS Semaphores
    - @ref mml_nfc_pcd_task_sleep should be modified to yield, allowing other important system tasks to execute.  For example, when using FreeRTOS it might be changed to:
        - <code>vTaskDelay(delay_ms / portTICK_PERIOD_MS);</code>
    - @ref mml_nfc_pcd_enter_critical and @ref mml_nfc_pcd_leave_critical should be changed to use the required RTOS primitives for critical sections.  For example, with FreeRTOS it might be changed to:
        - <code>taskENTER_CRITICAL();</code> and <code>taskEXIT_CRITICAL();</code>
    - @ref mml_nfc_pcd_set_nfc_interrupt_priority and @ref mml_nfc_pcd_set_FWT_timer_interrupt_priority functions are desribed above
    - The semaphore functions are setup as described in the
        <a href="https://www.freertos.org/Embedded-RTOS-Binary-Semaphores.html"> FreeRTOS Semaphore Documentation</a>.
    - @ref mml_nfc_pcd_field_level_detection_callback is called by @ref mml_nfc_pcd_transceive to allow user code to configure the analog settings for the transaction.  This allows the implementer to decide which
        settings are ideal for this transaction by calling the @ref mml_nfc_pcd_detect_loading function and selecting a pre-tuned setting for the current loading level.  For more information regarding this tuning
        refer to <a href="MAX32570_NFC_PCD_AFE_Tuning_Guide.pdf">MAX32570 NFC PCD AFE Tuning Guide.</a>

@section NFC_PCD_RF_Driver_Version_History_page Version History

Refer to @ref MML_NFC_PCD_DRIVER_VER_HIST

@section NFC_PCD_RF_Driver_license License Agreement

    /*******************************************************************************
    * Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
    * 
    * This software is protected by copyright laws of the United States and
    * of foreign countries. This material may also be protected by patent laws
    * and technology transfer regulations of the United States and of foreign
    * countries. This software is furnished under a license agreement and/or a
    * nondisclosure agreement and may only be used or reproduced in accordance
    * with the terms of those agreements. Dissemination of this information to
    * any party or parties not specified in the license agreement and/or
    * nondisclosure agreement is expressly prohibited.
    *
    * The above copyright notice and this permission notice shall be included
    * in all copies or substantial portions of the Software.
    *
    * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
    * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
    * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    * OTHER DEALINGS IN THE SOFTWARE.
    *
    * Except as contained in this notice, the name of Maxim Integrated
    * Products, Inc. shall not be used except as stated in the Maxim Integrated
    * Products, Inc. Branding Policy.
    *
    * The mere transfer of this software does not imply any licenses
    * of trade secrets, proprietary technology, copyrights, patents,
    * trademarks, maskwork rights, or any other form of intellectual
    * property whatsoever. Maxim Integrated Products, Inc. retains all
    * ownership rights.
    *******************************************************************************
    */
