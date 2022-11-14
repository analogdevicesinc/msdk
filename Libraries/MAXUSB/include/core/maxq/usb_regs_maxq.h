/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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
 *
 *
 ******************************************************************************/
 
/* USB controller registers accessed via UADDR/UDATA interface */
#define rFNADDR   0x01
#define rUSBCN    0x02
#define rUSBCFG   0x03
#define rUSBIEN   0x04
#define rUSBINT   0x05
#define rEPIEN    0x06
#define rEPINT    0x07
#define rEPSTL    0x08
#define rEPNAK    0x09
#define rEPCTG    0x0a
#define rEP0BC    0x0b
#define rEP1BC    0x0c
#define rEP2BC    0x0d
#define rEP3BC    0x0e
#define rRSVD_0F  0x0f
#define rEP0BUF   0x10
#define rEP1BUF   0x11
#define rEP2BUF   0x12
#define rEP3BUF   0x13
#define rSUDBUF   0x14
#define rEP4BC    0x15
#define rEP4BUF   0x16
#define rEP5BC    0x17
#define rEP5BUF   0x18

/* USBCN Register */
#define bmOSCST     0x80
#define bmVBGATE    0x40
#define bmURST      0x20
#define bmPWRDN     0x10
#define bmCONNECT   0x08
#define bmSIGRWU    0x04

/* USBCFG Register */
#define bmEPPWRDN   0x40
#define bmCONSW1    0x20
#define bmCONSW0    0x10
#define bmSOFO      0x04
#define bmBACTO     0x02
#define bmUSBEN     0x01

/* USBIEN Register */
#define bmBRSTDNIE  0x80
#define bmVBUSIE    0x40
#define bmNOVBUSIE  0x20
#define bmSUSPIE    0x10
#define bmBRSTIE    0x08
#define bmBACTIE    0x04
#define bmRWUDNIE   0x02
#define bmDPACTIE   0x01

/* USBINT Register */
#define bmBRSTDN    0x80
#define bmVBUS      0x40
#define bmNOVBUS    0x20
#define bmSUSP      0x10
#define bmBRST      0x08
#define bmBACT      0x04
#define bmRWUDN     0x02
#define bmDPACT     0x01

/* EPIEN Register */
#define bmIN5BAVIE  0x80
#define bmOUT4DAVIE 0x40
#define bmSUDAVIE   0x20
#define bmIN3BAVIE  0x10 
#define bmIN2BAVIE  0x08
#define bmOUT1DAVIE 0x04
#define bmOUT0DAVIE 0x02
#define bmIN0BAVIE  0x01

/* EPINT Register */
#define bmIN5BAV    0x80
#define bmOUT4DAV   0x40
#define bmSUDAV     0x20 
#define bmIN3BAV    0x10 
#define bmIN2BAV    0x08
#define bmOUT1DAV   0x04
#define bmOUT0DAV   0x02
#define bmIN0BAV    0x01

/* EPSTL Register */
#define bmSTLEP5    0x100
#define bmSTLEP4    0x080
#define bmACKSTAT   0x040
#define bmSTLSTAT   0x020
#define bmSTLEP3    0x010
#define bmSTLEP2    0x008
#define bmSTLEP1    0x004
#define bmSTLOUT0   0x002
#define bmSTLIN0    0x001

/* EPNAK Register */
#define bmEP3NAK    0x80
#define bmEP2NAK    0x40
#define bmEP0NAK    0x20
#define bmEP5NAK    0x10

/* EPCTG Register */
#define bmEP5DIS    0x800
#define bmCTGEP5    0x400
#define bmEP4DIS    0x200
#define bmCTGEP4    0x100
#define bmEP3DIS    0x080
#define bmEP2DIS    0x040
#define bmEP1DIS    0x020
#define bmCTGEP3    0x010
#define bmCTGEP2    0x008
#define bmCTGEP1    0x004


