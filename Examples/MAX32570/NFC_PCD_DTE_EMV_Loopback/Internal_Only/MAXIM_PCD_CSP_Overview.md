@section MAXIM_PCD_CSP_Overview_introPage Introduction

This Contactless Support Package (CSP) contains all the various documentation, and example code required for evaluation and development with the contactless interface on the MAX32570.

@section CSP_Documentation_Overview_page Documentation
Several external documents are included with the CSP:

- <a href="MAX32570 EMV DTE Users Manual.pdf"><b>MAX32570 EMV DTE Users Manual</b></a> <br>
This manual accompanies the @ref CSP_DTE_Example_Overview_page and explain how to use the example: board inspection, connection, powering, menu options and troubleshooting.

- <a href="MAX32570_PCD_AntennaMatching_Design_Guide.pdf"><b>MAX32570 PCD Antenna Matching Design Guide</b></a> <br>
This guide walks through design decisions and issues to get optimum analog performance for the Contactless interface.  It is critical that this is done properly for every contactless design.

- <a href="MAX32570_Matching_Design_Calculations.xlsx"><b>MAX32570 Matching Design Calculations</b></a> <br>
This spreadsheet should be used in conjunction with the <a href="MAX32570_PCD_AntennaMatching_Design_Guide.pdf"><b>MAX32570 PCD Antenna Matching Design Guide</b></a>.  It provides easy to use equations to solve for antenna matching component values.

- <a href="MAX32570_NFC_PCD_AFE_Tuning_Guide.pdf"><b>MAX32570 NFC PCD AFE Tuning Guide</b></a> <br>
The guide covers the final analog tuning required for contactless designs.  It should be used after optimal antenna matching.   

- <a href="MAX32570_EVKIT_EMV_L1_Test_Reports.zip"><b>MAX32570 EVKIT EMV L1 Test Reports</b></a> <br>
These Analog and Digital reports are generated using our in house Micropross EMV Level 1 tester and serve as regression tests for this release.

- <a href="EMVCo_TTA_PCD_L1_ICS_v30a_190206_MAX32570.pdf"><b>MAX32570 Implementation Conformance Statement (ICS)</b></a> <br>
This <a href="https://www.emvco.com/wp-content/uploads/documents/EMVCo_TTA_PCD_L1_ICS_v30a_190206.pdf"><b>form</b></a>
is required by EMVCo for every unique design/product submitted for certification.  It details implementations specific details regarding behaviors of the Level 1
stack and hardware in the MAX32570, and is provided for reference purposes.

@section CSP_RF_Driver_Overview_page PCD RF Driver
The @ref NFC_PCD_RF_Driver_APIOverviewAndUsage provides a low-level, straight forward interface to the complex radio hardware.  Its API is designed to be stack agnostic, so it may quickly be ported to other EMV L1 stacks.  It may also be used stand alone for other applications such as transport applications. 

@section CSP_EMV_PCD_Stack_Overview_page EMV PCD L1 Stack
The @ref NFC_PCD_EMV_L1_Stack_APIOverviewAndUsage sits on top of @ref NFC_PCD_RF_Driver_APIOverviewAndUsage, which acts as a Hardware Abstraction Layer (HAL). It provides an APDU interface as required for Level 2 stacks and contactless applications.

@section CSP_PBM_Library_Overview_page PBM Library
The @ref NFC_PCD_PBMOverviewAndUsage provides interface commands to PBM compatible transportation cards.  It includes poll, authentication, access commands including read/write and value commands including increment, and transfer.
PBM cards use a <b>Non</b>-ISO14443-3 compliant bit encoding with encrypted messages and secret keys to store and retrieve data from the card.

@section CSP_DTE_Example_Overview_page Device Test Environment
The DTE example is typically used for loopback testing and following the specification for DTE from EMVCo.  In addition, it provides various tuning functionality and testing methods for evaluation of the contactless interface.  Please refer to the <a href="MAX32570 EMV DTE Users Manual.pdf"><b>MAX32570 EMV DTE Users Manual</b></a> for full usage details.  This example also covers how to set the analog settings identified in the <a href="MAX32570_NFC_PCD_AFE_Tuning_Guide.pdf"><b>MAX32570 NFC PCD AFE Tuning Guide</b></a> and allows exercising of @ref PBM_COMMANDS.

@section CSP_FREERTOS_Demo_Overview_page FreeRTOS Demo
The FreeRTOS demo incorporates the contactless into a more fully featured application that uses the LCD, Magstripe, Contact Smart Card, and Keypad in addition to the Contactless interface.  The contactless demo will poll for cards in the field and attempt to recognize them.  Compatible cards will be queried for their Application ID and the card issuer name will be displayed.  Please note that this is ONLY a demo and will not read or interface correctly to all card types.  Not all cards respond the PPSE command that is used by this demo to find the AID.  True handling of cards and payment kernels requires a Level 2 stack.

@section CSP_Library_Memory_Usage Contactless Library Memory Usage

    arm-none-eabi-gcc --version:

    arm-none-eabi-gcc.exe (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599]
    Copyright (C) 2019 Free Software Foundation, Inc.
    This is free software; see the source for copying conditions.  There is NO
    warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

    arm-none-eabi-size -t libnfc_pcd_pbm_softfp.a libnfc_pcd_rf_driver_MAX32570_softfp.a *.o:

       text    data     bss     dec     hex filename
       5643       0       0    5643    160b pbm_commands.o (ex libnfc_pcd_pbm_softfp.a)
       1196       0      12    1208     4b8 pbm_crypto.o (ex libnfc_pcd_pbm_softfp.a)
       2055       0       0    2055     807 mml_nfc_pcd_rf_driver.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
       7921       0       0    7921    1ef1 mml_nfc_pcd_device.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
       3054      16       0    3070     bfe mml_nfc_pcd_internal.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
        883       0       0     883     373 mml_nfc_pcd_sw_type_a.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
         36       0       4      40      28 mml_nfc_pcd_sw_type_b.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
       1297       0      83    1380     564 mml_nfc_pcd_sw_type_f.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
       3327       0       0    3327     cff mml_nfc_pcd_bit_packing.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
       1188       0       0    1188     4a4 mml_nfc_pcd_timer_utils.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
         20       0       0      20      14 mml_nfc_pcd_trace.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
         36       0       0      36      24 mml_nfc_pcd_emulation_hooks.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
        132       0       0     132      84 mml_nfc_pcd_utils.o (ex libnfc_pcd_rf_driver_MAX32570_softfp.a)
        785      10       4     799     31f iso14443_3_common.o
        436       0       0     436     1b4 iso14443_3a_cmd.o
       2093       0       1    2094     82e iso14443_3a_flow.o
        524       0       0     524     20c iso14443_3b_cmd.o
        843       0      13     856     358 iso14443_3b_flow.o
       2813       4       1    2818     b02 iso14443_4_transitive.o
      34282      30     118   34430    867e (TOTALS)

    cat libnfc_pcd_pbm_softfp.a libnfc_pcd_rf_driver_MAX32570_softfp.a  > nfc_lib_blob
    cat iso14443_3_common.o iso14443_3a_cmd.o iso14443_3a_flow.o iso14443_3b_cmd.o iso14443_3b_flow.o iso14443_4_transitive.o nfc_lib_blob > nfc_lib_blob_combo
    sha1sum nfc_lib_blob_combo

    c14b161e313baed353e0c06d241388feecea362e *nfc_lib_blob_combo



* Contactless libraries do not use dynamic allocation (<code>malloc</code>)

* Use of the application stack is not tracked here.  Some variables and arrays are initialized on the application stack.  The FreeRTOS demo (not included in this CSP release) utilizes a separate stack for the NFC demo task of 2000 32bit works (8KB), this is adequate for the demo but may be resized based on application needs.  Some of the space used by the demo are buffers for EMV Level 2 data such as AID (Application IDentifiers) etc.

@section MAXIM_PCD_CSP_Overview_license License Agreement

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
