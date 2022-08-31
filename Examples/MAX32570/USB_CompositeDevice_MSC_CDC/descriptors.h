/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

#include "hid_kbd.h"
#include "usb.h"
#include <stdint.h>

MXC_USB_device_descriptor_t __attribute__((aligned(4))) composite_device_descriptor = {
    0x12, /* bLength                           */
    0x01, /* bDescriptorType = Device          */
    0x0200,
    /* bcdUSB USB spec rev (BCD)         */ ///
    0xEF, /* bDeviceClass = code specified by interface descriptors        */
    0x02, /* bDeviceSubClass = code specified by interface descriptors     */
    0x01, /* bDeviceProtocol = code specified by interface descriptors     */
    0x40, /* bMaxPacketSize0 is 64 bytes       */
    0x0B6A, /* idVendor (Maxim Integrated)       */
    0x003C,
    /* idProduct                         */ ///
    0x0100, /* bcdDevice                         */
    0x01, /* iManufacturer Descriptor ID       */
    0x02, /* iProduct Descriptor ID            */
    0x00, /* iSerialNumber Descriptor ID       */
    0x01 /* bNumConfigurations                */
};

/* Device qualifier needed for high-speed operation */
MXC_USB_device_qualifier_descriptor_t __attribute__((aligned(4)))
composite_device_qualifier_descriptor
    = {
          0x0A, /* bLength = 10                       */
          0x06, /* bDescriptorType = Device Qualifier */
          0x0200, /* bcdUSB USB spec rev (BCD)          */
          0xEF, /* bDeviceClass = Unspecified         */
          0x02, /* bDeviceSubClass                    */
          0x01, /* bDeviceProtocol                    */
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
    /* Interface Association Descriptor */
    uint8_t interface_association_descriptor[8];
    /* Interface #1 CDCACM Device */
    MXC_USB_interface_descriptor_t comm_interface_descriptor;
    uint8_t header_functional_descriptor[5];
    uint8_t call_management_descriptor[5];
    uint8_t acm_functional_descriptor[4];
    uint8_t union_functional_descriptor[5];
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_3;
    /* Interface #2 CDC Data*/
    MXC_USB_interface_descriptor_t data_interface_descriptor;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_4;
    MXC_USB_endpoint_descriptor_t endpoint_descriptor_5;
}

composite_config_descriptor
    = {
          {
              0x09, /*  bLength                          */
              0x02, /*  bDescriptorType = Config         */
              0x0062, /*  wTotalLength(L/H) = 98 bytes     */
              0x03, /*  bNumInterfaces                   */
              0x01, /*  bConfigurationValue              */
              0x02, /*  iConfiguration                   */ ///
              0xE0, /*  bmAttributes (bus-powered, remote wakeup) */
              0x01, /*  MaxPower is 100ma (units are 2ma/bit) */
          },
          /********** Interface #0 : Mass Storage Device **********/
          {
              /*  Second Interface Descriptor For MSC Interface */
              0x09, /*  bLength = 9                     */
              0x04, /*  bDescriptorType = Interface (4) */
              0x00, /*  bInterfaceNumber                */
              0x00, /*  bAlternateSetting               */
              0x02, /*  bNumEndpoints (one for IN one for OUT)     */
              0x08, /*  bInterfaceClass = Mass Storage (8) */
              0x06, /*  bInterfaceSubClass = SCSI Transparent Command Set */
              0x50, /*  bInterfaceProtocol = Bulk-Only Transport */
              0x05, /*  iInterface                      */
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
          },
          /********** Interface Association Descriptor **********/
          {
              0x08, /* bLength                          */
              0x0B, /* bDescriptorType                  */
              0x01, /* bFirstInterface                  */
              0x02, /* bInterfaceCount                  */
              0x02, /* bFunctionClass                   */
              0x02, /* bFunctionSubClass                */
              0x01, /* bFunctionProtocol                */
              0x00, /* iFunction                        */
          },
          /********** Interface #1 : COMM Interface **********/
          {
              /*  First Interface Descriptor For Comm Class Interface */
              0x09, /*  bLength = 9                     */
              0x04, /*  bDescriptorType = Interface (4) */
              0x01, /*  bInterfaceNumber                */
              0x00, /*  bAlternateSetting               */
              0x01, /*  bNumEndpoints (one for OUT)     */
              0x02, /*  bInterfaceClass = Communications Interface Class (2) */
              0x02, /*  bInterfaceSubClass = Abstract Control Model (2) */
              0x01, /*  bInterfaceProtocol = Common "AT" commands (1), no class specific protocol
                       (0) */
              0x04, /*  iInterface                      */
          },
          {
              /*  Header Functional Descriptor */
              0x05, /*  bFunctionalLength = 5           */
              0x24, /*  bDescriptorType                 */
              0x00, /*  bDescriptorSubtype              */
              0x10, 0x01, /*  bcdCDC                          */
          },
          {
              /*  Call Management Descriptor */
              0x05, /*  bFunctionalLength = 5           */
              0x24, /*  bDescriptorType                 */
              0x01, /*  bDescriptorSubtype              */
              0x03, /*  bmCapabilities = Device handles call management itself (0x01), management
                       over data class (0x02) */
              0x01, /*  bmDataInterface                 */
          },
          {
              /*  Abstract Control Management Functional Descriptor */
              0x04, /*  bFunctionalLength = 4           */
              0x24, /*  bDescriptorType                 */
              0x02, /*  bDescriptorSubtype              */
              0x02, /*  bmCapabilities                  */
          },
          {
              /*  Union Functional Descriptor */
              0x05, /*  bFunctionalLength = 5           */
              0x24, /*  bDescriptorType                 */
              0x06, /*  bDescriptorSubtype              */
              0x00, /*  bmMasterInterface               */
              0x01, /*  bmSlaveInterface0               */
          },
          {
              /*  IN Endpoint 3 (Descriptor #1) */
              0x07, /*  bLength                          */
              0x05, /*  bDescriptorType (Endpoint)       */
              0x84, /*  bEndpointAddress (EP3-IN)        */
              0x03, /*  bmAttributes (interrupt)         */
              0x0040, /*  wMaxPacketSize                   */
              0xff, /*  bInterval (milliseconds)         */
          },
          {
              /*  Second Interface Descriptor For Data Interface */
              0x09, /*  bLength                          */
              0x04, /*  bDescriptorType (Interface)      */
              0x02, /*  bInterfaceNumber                 */
              0x00, /*  bAlternateSetting                */
              0x02, /*  bNumEndpoints                    */
              0x0a, /*  bInterfaceClass = Data Interface (10) */
              0x00, /*  bInterfaceSubClass = none (0)    */
              0x00, /*  bInterfaceProtocol = No class specific protocol (0) */
              0x04, /*  biInterface = No Text String (0) */
          },
          {
              /*  OUT Endpoint 1 (Descriptor #2) */
              0x07, /*  bLength                          */
              0x05, /*  bDescriptorType (Endpoint)       */
              0x05, /*  bEndpointAddress (EP1-OUT)       */
              0x02, /*  bmAttributes (bulk)              */
              0x0040, /*  wMaxPacketSize                   */
              0x00, /*  bInterval (N/A)                  */
          },
          {
              /*  IN Endpoint 2 (Descriptor #3) */
              0x07, /*  bLength                          */
              0x05, /*  bDescriptorType (Endpoint)       */
              0x83, /*  bEndpointAddress (EP2-IN)        */
              0x02, /*  bmAttributes (bulk)              */
              0x0040, /*  wMaxPacketSize                   */
              0x00, /*  bInterval (N/A)                  */
          },
      };

__attribute__((aligned(4))) uint8_t report_descriptor[] = {
    0x05, 0x01, /*  Usage Page (generic desktop)      */
    0x09, 0x06, /*  Usage (keyboard)                  */
    0xa1, 0x01, /*  Collection                        */
    0x05, 0x07, /*    Usage Page 7 (keyboard/keypad)  */
    0x19, 0xe0, /*    Usage Minimum = 224             */
    0x29, 0xe7, /*    Usage Maximum = 231             */
    0x15, 0x00, /*    Logical Minimum = 0             */
    0x25, 0x01, /*    Logical Maximum = 1             */
    0x75, 0x01, /*    Report Size = 1                 */
    0x95, 0x08, /*    Report Count = 8                */
    0x81, 0x02, /*   Input(Data,Variable,Absolute)    */
    0x95, 0x01, /*    Report Count = 1                */
    0x75, 0x08, /*    Report Size = 8                 */
    0x81, 0x01, /*   Input(Constant)                  */
    0x19, 0x00, /*    Usage Minimum = 0               */
    0x29, 0x65, /*    Usage Maximum = 101             */
    0x15, 0x00, /*    Logical Minimum = 0             */
    0x25, 0x65, /*    Logical Maximum = 101           */
    0x75, 0x08, /*    Report Size = 8                 */
    0x95, 0x01, /*    Report Count = 1                */
    0x81, 0x00, /*   Input(Data,Variable,Array)       */
    0xc0 /*  End Collection                    */
};

__attribute__((aligned(4))) uint8_t lang_id_desc[] = {
    0x04, /* bLength */
    0x03, /* bDescriptorType */
    0x09, 0x04 /* bString = wLANGID (see usb_20.pdf 9.6.7 String) */
};

__attribute__((aligned(4))) uint8_t mfg_id_desc[] = {
    0x22, /* bLength */
    0x03, /* bDescriptorType */
    'M',
    0,
    'a',
    0,
    'x',
    0,
    'i',
    0,
    'm',
    0,
    ' ',
    0,
    'I',
    0,
    'n',
    0,
    't',
    0,
    'e',
    0,
    'g',
    0,
    'r',
    0,
    'a',
    0,
    't',
    0,
    'e',
    0,
    'd',
    0,
};

__attribute__((aligned(4))) uint8_t prod_id_desc[] = {
    0x34, /* bLength */
    0x03, /* bDescriptorType */
    'M',
    0,
    'A',
    0,
    'X',
    0,
    '3',
    0,
    '2',
    0,
    '6',
    0,
    '6',
    0,
    '5',
    0,
    ' ',
    0,
    'C',
    0,
    'o',
    0,
    'm',
    0,
    'p',
    0,
    'o',
    0,
    's',
    0,
    'i',
    0,
    't',
    0,
    'e',
    0,
    ' ',
    0,
    'D',
    0,
    'e',
    0,
    'v',
    0,
    'i',
    0,
    'c',
    0,
    'e',
    0,
};

__attribute__((aligned(4))) uint8_t serial_id_desc[] = { 0x14, /* bLength */
    0x03, /* bDescriptorType */
    '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0 };

__attribute__((aligned(4))) uint8_t cdcacm_func_desc[] = {
    0x20, /* bLength */
    0x03, /* bDescriptorType */
    'M',
    0,
    'A',
    0,
    'X',
    0,
    '3',
    0,
    '2',
    0,
    '6',
    0,
    '6',
    0,
    '5',
    0,
    ' ',
    0,
    'C',
    0,
    'D',
    0,
    'C',
    0,
    'A',
    0,
    'C',
    0,
    'M',
    0,
};

__attribute__((aligned(4))) uint8_t msc_func_desc[] = {
    0x3A, /* bLength */
    0x03, /* bDescriptorType */
    'M',
    0,
    'A',
    0,
    'X',
    0,
    '3',
    0,
    '2',
    0,
    '6',
    0,
    '6',
    0,
    '5',
    0,
    ' ',
    0,
    'M',
    0,
    'a',
    0,
    's',
    0,
    's',
    0,
    ' ',
    0,
    'S',
    0,
    't',
    0,
    'o',
    0,
    'r',
    0,
    'a',
    0,
    'g',
    0,
    'e',
    0,
    ' ',
    0,
    'D',
    0,
    'e',
    0,
    'v',
    0,
    'i',
    0,
    'c',
    0,
    'e',
    0,
};

#endif /* _DESCRIPTORS_H_ */
