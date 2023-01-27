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

#ifndef EXAMPLES_MAX32670_SECUREROM_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_H_
#define EXAMPLES_MAX32670_SECUREROM_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_H_

/*******************************      INCLUDES    ****************************/

/*******************************      DEFINES     ****************************/
// RESET pin used to restart target before SCP communication
#define GPIO_IDX_RSTN 0
// Stimulus pin to enable SCP communication
#define GPIO_IDX_STIMULUS_PIN 1

#define STIMULUS_PIN_ACTIVE_STATE 0 // 0 or 1

// Packet Type
#define PT_HELLO_REQ 1
#define PT_HELLO_RSP 2
#define PT_DELETE_MEM 3
#define PT_DELETE_MEM_RSP 4
#define PT_WRITE_MEM 5
#define PT_WRITE_MEM_RSP 6

// MAX wait time for packets
#define PACKET_WAIT_DEL_MEM_RSP 5000 // ms
#define PACKET_WAIT_WRITE_MEM_RSP 5000 // ms

/******************************* Type Definitions ****************************/
typedef int (*comm_read_t)(unsigned char *dst, unsigned int len, unsigned int to);
typedef int (*comm_write_t)(const unsigned char *src, unsigned int len, unsigned int to);

typedef struct {
    comm_read_t read;
    comm_write_t write;
    void (*gpio_set)(unsigned int idx, int state);
    void (*delay_ms)(unsigned int ms);
    int (*printf)(const char *pcFmt, ...);
} bl_conf_struct_t;

typedef struct {
    unsigned char type; // 1:hello_reply, 2:erase/del_mem
    unsigned char is_tx; // 1: From host to target, 0: From target to host
    unsigned short len;
    const unsigned char *data;
} scp_packet_struct;

/******************************* Public Functions ****************************/
int sbl_init(bl_conf_struct_t *plt_funcs);
int sbl_load(scp_packet_struct *scp_packets);

#endif // EXAMPLES_MAX32670_SECUREROM_BOOTLOADER_HOST_BOOTLOADER_BOOTLOADER_H_
