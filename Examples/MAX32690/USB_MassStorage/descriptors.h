/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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
 ******************************************************************************/

#ifndef EXAMPLES_MAX32690_USB_MASSSTORAGE_DESCRIPTORS_H_
#define EXAMPLES_MAX32690_USB_MASSSTORAGE_DESCRIPTORS_H_

#include <stdint.h>
#include "usb.h"
#include "hid_kbd.h"

MXC_USB_device_descriptor_t __attribute__((aligned(4))) device_descriptor = {
    0x12, /* bLength = 18                     */
    0x01, /* bDescriptorType = Device         */
    0x0200, /* bcdUSB USB spec rev (BCD)        */
    0x00, /* bDeviceClass = Unspecified       */
    0x00, /* bDeviceSubClass                  */
    0x00, /* bDeviceProtocol                  */
    0x40, /* bMaxPacketSize0 is 64 bytes      */
    0x0B6A, /* idVendor (Maxim Integrated)      */
    0x4402, /* idProduct                        */
    0x0100, /* bcdDevice                        */
    0x01, /* iManufacturer Descriptor ID      */
    0x02, /* iProduct Descriptor ID           */
    0x03, /* iSerialNumber = (0) No string    */
    0x01 /* bNumConfigurations               */
};

/* Device qualifier needed for high-speed operation */
MXC_USB_device_qualifier_descriptor_t __attribute__((aligned(4))) device_qualifier_descriptor = {
    0x0A, /* bLength = 10                       */
    0x06, /* bDescriptorType = Device Qualifier */
    0x0200, /* bcdUSB USB spec rev (BCD)          */
    0x00, /* bDeviceClass = Unspecified         */
    0x00, /* bDeviceSubClass                    */
    0x00, /* bDeviceProtocol                    */
    0x40, /* bMaxPacketSize0 is 64 bytes        */
    0x01, /* bNumConfigurations                 */
    0x00 /* Reserved, must be 0                */
};

__attribute__((aligned(4))) struct __attribute__((packed)) {
    MXC_USB_configuration_descriptor_t config_descriptor;
    /* Interface #0 Mass Storage Device */
    MXC_USB_interface_descriptor_t msc_interface_descriptor;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_1;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_2;
} config_descriptor = { {
                            0x09, /*  bLength = 9                     */
                            0x02, /*  bDescriptorType = Config (2)    */
                            0x0020, /*  wTotalLength(L/H)               */
                            0x01, /*  bNumInterfaces                  */
                            0x01, /*  bConfigValue                    */
                            0x00, /*  iConfiguration                  */
                            0xC0, /*  bmAttributes (self-powered, no remote wakeup) */
                            0x32, /*  MaxPower is 100ma (units are 2ma/bit) */
                        },
                        {
                            /*  First Interface Descriptor For MSC Interface */
                            0x09, /*  bLength = 9                     */
                            0x04, /*  bDescriptorType = Interface (4) */
                            0x00, /*  bInterfaceNumber                */
                            0x00, /*  bAlternateSetting               */
                            0x02, /*  bNumEndpoints (one for INm one for OUT)     */
                            0x08, /*  bInterfaceClass = Mass Storage (8) */
                            0x06, /*  bInterfaceSubClass = SCSI Transparent Command Set */
                            0x50, /*  bInterfaceProtocol = Bulk-Only Transport */
                            0x00, /*  iInterface                      */
                        },
                        {
                            /*  OUT Endpoint 1 (Descriptor #1) */
                            0x07, /*  bLength                          */
                            0x05, /*  bDescriptorType (Endpoint)       */
                            0x01, /*  bEndpointAddress (EP1-OUT)       */
                            0x02, /*  bmAttributes (bulk)              */
                            0x0040, /*  wMaxPacketSize                   */
                            0x00, /*  bInterval (N/A)                  */
                        },
                        {
                            /*  IN Endpoint 2 (Descriptor #2) */
                            0x07, /*  bLength                          */
                            0x05, /*  bDescriptorType (Endpoint)       */
                            0x82, /*  bEndpointAddress (EP2-IN)        */
                            0x02, /*  bmAttributes (bulk)              */
                            0x0040, /*  wMaxPacketSize                   */
                            0x00 /*  bInterval (N/A)                  */
                        } };

__attribute__((aligned(4))) struct __attribute__((packed)) {
    MXC_USB_configuration_descriptor_t config_descriptor;
    /* Interface #0 Mass Storage Device */
    MXC_USB_interface_descriptor_t msc_interface_descriptor;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_1;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_2;
} config_descriptor_hs = { {
                               0x09, /*  bLength = 9                     */
                               0x02, /*  bDescriptorType = Config (2)    */
                               0x0020, /*  wTotalLength(L/H)               */
                               0x01, /*  bNumInterfaces                  */
                               0x01, /*  bConfigValue                    */
                               0x00, /*  iConfiguration                  */
                               0xC0, /*  bmAttributes (self-powered, no remote wakeup) */
                               0x32, /*  MaxPower is 100ma (units are 2ma/bit) */
                           },
                           {
                               /*  First Interface Descriptor For MSC Interface */
                               0x09, /*  bLength = 9                     */
                               0x04, /*  bDescriptorType = Interface (4) */
                               0x00, /*  bInterfaceNumber                */
                               0x00, /*  bAlternateSetting               */
                               0x02, /*  bNumEndpoints (one for INm one for OUT)     */
                               0x08, /*  bInterfaceClass = Mass Storage (8) */
                               0x06, /*  bInterfaceSubClass = SCSI Transparent Command Set */
                               0x50, /*  bInterfaceProtocol = Bulk-Only Transport */
                               0x00, /*  iInterface                      */
                           },
                           {
                               /*  OUT Endpoint 1 (Descriptor #1) */
                               0x07, /*  bLength                          */
                               0x05, /*  bDescriptorType (Endpoint)       */
                               0x01, /*  bEndpointAddress (EP1-OUT)       */
                               0x02, /*  bmAttributes (bulk)              */
                               0x0200, /*  wMaxPacketSize                   */
                               0x01, /*  bInterval                        */
                           },
                           {
                               /*  IN Endpoint 2 (Descriptor #2) */
                               0x07, /*  bLength                          */
                               0x05, /*  bDescriptorType (Endpoint)       */
                               0x82, /*  bEndpointAddress (EP2-IN)        */
                               0x02, /*  bmAttributes (bulk)              */
                               0x0200, /*  wMaxPacketSize                   */
                               0x01 /*  bInterval                        */
                           } };

__attribute__((aligned(4))) uint8_t lang_id_desc[] = {
    0x04, /* bLength */
    0x03, /* bDescriptorType */
    0x09, 0x04 /* bString = wLANGID (see usb_20.pdf 9.6.7 String) */
};

__attribute__((aligned(4))) uint8_t mfg_id_desc[] = {
    0x22, /* bLength */
    0x03, /* bDescriptorType */
    'M',  0, 'a', 0, 'x', 0, 'i', 0, 'm', 0, ' ', 0, 'I', 0, 'n', 0,
    't',  0, 'e', 0, 'g', 0, 'r', 0, 'a', 0, 't', 0, 'e', 0, 'd', 0,
};

__attribute__((aligned(4))) uint8_t prod_id_desc[] = {
    0x38, /* bLength */
    0x03, /* bDescriptorType */
    'M',  0, 'A', 0, 'X', 0, 'U', 0, 'S', 0, 'B', 0, ' ', 0, 'M', 0, 'a', 0,
    's',  0, 's', 0, ' ', 0, 'S', 0, 't', 0, 'o', 0, 'r', 0, 'a', 0, 'g', 0,
    'e',  0, ' ', 0, 'E', 0, 'x', 0, 'a', 0, 'm', 0, 'p', 0, 'l', 0, 'e', 0,
};

/* Not currently used (see device descriptor), but could be enabled if desired */
__attribute__((aligned(4)))
uint8_t serial_id_desc[] = { 26, /* bLength */
                             0x03, /* bDescriptorType */
                             '0',  0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0,
                             '0',  0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0 };

#endif // EXAMPLES_MAX32690_USB_MASSSTORAGE_DESCRIPTORS_H_
