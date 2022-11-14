@page NFC_PCD_EMV_L1_Stack_APIOverviewAndUsage NFC PCD EMV L1 Stack

@section NFC_PCD_EMV_L1_Stack_introPage Introduction

This document describes the Maxim Integrated NFC PCD EMV Contactless L1 Stack Library, provides an overview for new developers, and details integration specifics.  This library sits on top of @ref NFC_PCD_RF_DRIVER, which acts as a Hardware Abstraction Layer (HAL) for this EMV L1 Stack as shown in \ref NFC_PCD_EMV_L1_Stack_figure1_overview "Figure 1".  It provides an APDU interface as detailed in ISO/IEC 7816-3, and EMV Book 1. While this library is primary designed for EMV Contactless, it also has the basic functionality of ISO14443-4, namely it implements and utilizes the half-duplex block transmission protocol detailed in the ISO/IEC 14443-4 specification.

@anchor NFC_PCD_EMV_L1_Stack_figure1_overview
@image html ./NFC_EMV_SW_Stack.bmp "Figure 1: EMV Contactless Stack Overview" width=40% height=40%

@section NFC_PCD_EMV_L1_Stack_apiPage API

@subsection NFC_PCD_EMV_L1_Stack_activation PICC Selection and Activation

A large portion of the @ref NFC_PCD_EMV_LVL1_STACK deals with correct selection and proper activation of PICCs.  @ref EMV_POLLING_AND_LOOPBACK provides example implementations to get application development started.  Many applications will require modification of these polling routines to tune for its specific needs.  Unlike the @ref NFC_PCD_RF_DRIVER these routines are provided as source to allow for application specific configuration.

These polling routines are implemented as required and detailed in [EMV Level 1 Specification](https://www.emvco.com/wp-content/plugins/pmpro-customizations/oy-getfile.php?u=/wp-content/uploads/documents/EMV-Level-1-Contactless-Interface-Specification-V3.0-180423.pdf).

@ref NFC_PCD_EMV_LVL1_STACK provides activation routines for ISO14443 Types A and B only.  Routines provided correspond to the top-level steps detailed in the EMV L1 Specification, Ch 9: Polling, Collision Detection, Activation, and Removal.  The routines are available in:
- @ref NFC_PCD_EMV_LVL1_PART3AF
- @ref NFC_PCD_EMV_LVL1_PART3BF

These higher-level routines use more basic commands found in:
- @ref NFC_PCD_EMV_LVL1_PART3AC
- @ref NFC_PCD_EMV_LVL1_PART3BC

@subsection NFC_PCD_EMV_L1_Stack_Trasport APDU Transport

After a PICC is located and activated via @ref NFC_PCD_EMV_L1_Stack_activation communications can begin.  This communication utilizes Application Protocol Data Units (APDU) as detailed in [EMV Contact Book 1](https://www.emvco.com/emv-technologies/contact/).  Command APDU (CAPDU) and Response APDU (RAPDU) are paired together, and while some are standard, many are specific to the individual application, as such they are beyond the scope of this document.  Some details pertaining to these APDUs can be found in [EMV Contactless Kernel Specifications 1-7](https://www.emvco.com/emv-technologies/contactless/).

@subsection Level_2_Stack_Connection Connections to EMV Contactless Level 2

@ref SendAPDU is the primary interface to a EMV Contactless Level 2 Stack and connections/usage should be very straightforward.

Before use of @ref SendAPDU, a compatible PICC must be identified, and activated. Refer to @ref EMV_POLLING_AND_LOOPBACK.

@subsection API_Details Full API Details
- @ref NFC_PCD_EMV_LVL1_PART4
- @ref EMV_POLLING_AND_LOOPBACK
- @ref NFC_PCD_EMV_LVL1_COMMON
- @ref NFC_PCD_EMV_LVL1_PART3AF
- @ref NFC_PCD_EMV_LVL1_PART3AC
- @ref NFC_PCD_EMV_LVL1_PART3BF
- @ref NFC_PCD_EMV_LVL1_PART3BC
- @ref PBM_COMMANDS

@note For analog configuration refer to: @ref mml_nfc_pcd_set_analog_config

@section NFC_PCD_EMV_L1_Stack_portPage Integration

For simplicity and ease of integration, the EMV L1 Stack uses the same primitives as the underlying RF driver, implemented in @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT. As such be certain to follow the @ref NFC_PCD_RF_Driver_portPage and the @ref NFC_PCD_RF_Driver_resourcePage recommendations.

@subsection NFC_PCD_EMV_L1_Stack_portingFile Application Portability and RTOS Configuration

As mentioned in @ref NFC_PCD_EMV_L1_Stack_portPage, the EMV L1 Stack uses the RF Driver @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT. Therefore, again follow the guidance in @ref portingFile for details on configuration for Single or Multi-Process applications.

In addition to the information provided by the RF Driver documentation, these routines must also be configured:

@section NFC_PCD_EMV_L1_Stack_Version_History_page Version History

Refer to @ref NFC_PCD_EMV_LVL1_STACK_VER_HIST

@section NFC_PCD_EMV_L1_Stack_license License Agreement

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
