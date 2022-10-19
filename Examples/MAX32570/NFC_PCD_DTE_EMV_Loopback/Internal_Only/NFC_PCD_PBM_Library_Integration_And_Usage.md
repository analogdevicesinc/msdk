@page NFC_PCD_PBMOverviewAndUsage NFC PCD PBM Library

@section NFC_PCD_PBM_Library_introPage Introduction

This document describes the MAX32570 NFC PCD PBM Library, provides an overview for new developers, and details integration specifics. The purpose of this library is to provide routines that facilitate access to Parity Bypass Mode (PBM) compatible cards. <br>
The PBM Library provides interface commands to PBM compatible transportation cards.  PBM cards use a <b>Non</b>-ISO14443-3 compliant bit encoding with encrypted messages and secret keys to store and retrieve data from the card.

The @ref PBM_COMMANDS module contains the PBM compatible interface routines. The PICC must be identified and activated using the ISO14443 Type A routines before attempting any PBM commands.

As part of a Contactless stack the MAX32570 NFC PCD PBM Library sits as shown in \ref NFC_PCD_PBM_Library_figure1_overview "Figure 1". It uses ISO14443-3 Type A Driver routines to perform detection and anti-collision and then uses the ISO14443-2 level to perform the encrypted PBM commands.

@anchor NFC_PCD_PBM_Library_figure1_overview
@image html ./NFC_EMV_SW_Stack.bmp "Figure 1: EMV Contactless Stack Overview" width=40% height=40%

@section NFC_PCD_PBM_Library_apiPage API

Full API details are found in @ref PBM_COMMANDS. The below sections provide an overview of the functionality.

The PBM library provides basic commands to access PBM cards.  Please refer to the specific card datasheets for
a more complete overview of the card functionality.  The current PBM library provides complete functionality
but does not provide convenience functions for access, only the low-level commands.  It assumed the
users/integrators for this library will be familiar with standard PBM card functionality.  For instance, that
when using a value block increment, the incremented result is only stored in the internal working register until
it is written back into EEPROM via the Transfer Command.  Or the meaning of the various security access bit
stored in the sector trailers.

The Library consists of 3 main features:
- Block Authentication: Authenticate the PICC for a specific block using either Key A or Key B
- Block Read and Write: Send or receive a 16 byte block of data
- Value Block Commands: Increment, Decrement, Transfer, or Restore. These commands operate on a specific block and use the PICC's internal transfer buffer.

@section NFC_PCD_PBM_Library_portPage PBM Library Integration Guidance

For simplicity and ease of integration, the PBM Library uses the same primitives as the underlying RF driver, implemented in @ref NFC_EMV_CONTACTLESS_RF_DRIVER_PORT. As such be certain to follow the @ref NFC_PCD_RF_Driver_portPage and the @ref NFC_PCD_RF_Driver_resourcePage recommendations.

PBM communication uses a <b>Non</b>-ISO14443-3 compliant bit encoding and uses @ref mml_nfc_pcd_transceive_bits.

@section NFC_PCD_PBM_Library_Version_History_page Version History

Refer to @ref MML_NFC_PCD_PBM_LIBRARY_VER_HIST

@section NFC_PCD_PBM_Library_license License Agreement

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
