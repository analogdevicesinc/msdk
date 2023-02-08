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

/********************************      INCLUDES    ****************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bootloader.h"

/*******************************      DEFINES     ****************************/
// Packet Delay
#define PACKET_WAIT 200 // ms
//
#define DELAY_BETWEEN_TWO_TX_PACKET 100 // ms

/******************************* Type Definitions ****************************/

/*******************************    Variables    ****************************/
static bl_conf_struct_t g_plt_funcs;

/******************************* Static Functions ****************************/
static void hexdump(const char *title, const unsigned char *buf, unsigned int len)
{
    unsigned int i;

    if (title) {
        g_plt_funcs.printf("%s", title);
    }

    /* Print buffer bytes */
    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
            g_plt_funcs.printf("\r\n");
        }
        g_plt_funcs.printf("%02X ", buf[i]);
    }

    g_plt_funcs.printf("\r\n");
}

static void reset_target(void)
{
    g_plt_funcs.gpio_set(GPIO_IDX_RSTN, 0);
    g_plt_funcs.delay_ms(1);
    g_plt_funcs.gpio_set(GPIO_IDX_RSTN, 1);
}

static int write_packet(scp_packet_struct *packet)
{
    int ret = 0;
    ret = g_plt_funcs.write(packet->data, packet->len, 200);
    return ret;
}

static int read_packet(scp_packet_struct *expected, unsigned char *buf, unsigned int delay_ms)
{
    int ret = 0;

    ret = g_plt_funcs.read(buf, expected->len, delay_ms);
    if (ret == 0) {
        if (memcmp(buf, expected->data, expected->len) != 0) {
            ret = -1;
            g_plt_funcs.printf("\r\nBuffers did not compare\r\n");
            hexdump("Expected", expected->data, expected->len);
            hexdump("Received:", buf, expected->len);
        }
    }

    return ret;
}

static int read_hello_rsp(scp_packet_struct *expected, unsigned char *buf)
{
    int ret = 0;

    ret = g_plt_funcs.read(buf, expected->len, 2 * PACKET_WAIT);
    if (ret == 0) {
        /* Handle special case where hello_reply contains variable data,
		 * just check header of response. */
        int len = 22;
        if (memcmp(buf, expected->data, len) != 0) {
            ret = -1;
            g_plt_funcs.printf("\r\nError: received packet is not the expected one\r\n");
            hexdump("Expected", expected->data, len);
            hexdump("Received:", buf, len);
        }
    }

    return ret;
}

static int decode_hello_rsp(unsigned char *hello_rsp)
{
    int ret = 0;
    unsigned int offset = 12; // past the packet link layer header.
    unsigned char *p_rsp = &hello_rsp[offset];
    unsigned int rom_version;
    unsigned int flag;

    rom_version = (p_rsp[0x0d] << 24) | (p_rsp[0x0c] << 16) | (p_rsp[0x0b] << 8) | p_rsp[0x0a];
    g_plt_funcs.printf("ROM Version: 0x%08X\r\n", rom_version);

    g_plt_funcs.printf("Phase: 0x%02X\r\n", p_rsp[0x0e]);

    // Annex Value
    unsigned char config = p_rsp[0x11];

    flag = config & (1 << 6);
    g_plt_funcs.printf("Rework: %s\r\n", flag ? "Allowed" : "Not allowed");

    flag = config & (1 << 7);
    g_plt_funcs.printf("JTAG: %s\r\n", flag ? "Disabled" : "Enabled");

    switch (p_rsp[0x12]) {
    case 0x59: // MAX32590 / 91 / 92
    case 0x67: // MAX32550
    case 0x68: // MAX32510 / 50 / 58
    case 0x69: // MAX32552 / 60 / 61
    case 0x6A: // MAX32565 / 66
    case 0x80: // MAX32651
    case 0x81: // MAX32666
        // no CRK flag
        break;
    default:
        flag = config & (1 << 4);
        g_plt_funcs.printf("CRK: %s\r\n", flag ? "Not Exist" : "Exist");
        break;
    }

    g_plt_funcs.printf("USN: ");
    for (int i = 0; i < 13; i++) {
        g_plt_funcs.printf("%02X", p_rsp[0x12 + i]);
    }
    g_plt_funcs.printf("\r\n");

    return ret;
}

static int connect(scp_packet_struct *req, scp_packet_struct *reply, scp_packet_struct *ack)
{
    int ret = 0;
    int i;
    unsigned char rxBuf[16];

    if (reply->len > sizeof(rxBuf)) {
        return -1;
    }

    for (i = 0; i < 200; i++) {
        g_plt_funcs.printf("Trying to connect: %d\r\n", i);

        // Send Request Packet
        ret = write_packet(req);
        if (ret) {
            continue;
        }

        // Read Expected Reply Packet
        ret = read_packet(reply, rxBuf, PACKET_WAIT);
        if (ret) {
            continue;
        }

        // Send ACK
        ret = write_packet(ack);

        if (ret == 0) {
            break;
        }
    }

    return ret;
}

/******************************* Public Functions ****************************/
int sbl_init(bl_conf_struct_t *plt_funcs)
{
    int ret = 0;

    g_plt_funcs = *plt_funcs;

    return ret;
}

int sbl_load(scp_packet_struct *scp_packets)
{
    int ret = 0;
    int i;
    int num_of_packets;
    unsigned char rxBuf[256];
    scp_packet_struct *scp_packet;
    scp_packet_struct *con_req;
    scp_packet_struct *con_reply;
    scp_packet_struct *con_ack;
    int last_packet_was_tx = 0;

    // Assume there would be max 1000 packet
    for (i = 0; i < 1000; i++) {
        if (scp_packets[i].len == 0) {
            // len = 0 indicates end of array
            break;
        }
    }
    num_of_packets = i;

    // assert stimulus pin
    g_plt_funcs.gpio_set(GPIO_IDX_STIMULUS_PIN, STIMULUS_PIN_ACTIVE_STATE);
    // reset target
    reset_target();

    con_req = &scp_packets[0];
    con_reply = &scp_packets[1];
    con_ack = &scp_packets[2];

    ret = connect(con_req, con_reply, con_ack);

    if (ret == 0) {
        g_plt_funcs.printf("\r\n---------- Connected  ----------\r\n");

        for (i = 3; i < num_of_packets; i++) {
            scp_packet = &scp_packets[i];

            if (scp_packet->is_tx) {
                g_plt_funcs.printf("Please wait loading %02d/%d...\r\n", i, num_of_packets);

                if (last_packet_was_tx) {
                    // wait a little between two tx packet
                    g_plt_funcs.delay_ms(DELAY_BETWEEN_TWO_TX_PACKET);
                }
                ret = write_packet(scp_packet);
                last_packet_was_tx = 1;
            } else {
                last_packet_was_tx = 0; // no

                switch (scp_packet->type) {
                case PT_HELLO_RSP:
                    ret = read_hello_rsp(scp_packet, rxBuf);
                    if (ret == 0) {
                        g_plt_funcs.printf("--------------------------------\r\n");
                        decode_hello_rsp(rxBuf);
                        g_plt_funcs.printf("--------------------------------\r\n");
                    }
                    break;
                case PT_DELETE_MEM_RSP:
                    // wait more until flash to be deleted
                    ret = read_packet(scp_packet, rxBuf, PACKET_WAIT_DEL_MEM_RSP);
                    break;
                case PT_WRITE_MEM_RSP:
                    // wait more until flash operation
                    ret = read_packet(scp_packet, rxBuf, PACKET_WAIT_WRITE_MEM_RSP);
                    break;
                default:
                    ret = read_packet(scp_packet, rxBuf, PACKET_WAIT);
                    break;
                }
            }

            if (ret) {
                g_plt_funcs.printf("Loading %03d/%d FAILED\r\n", i, num_of_packets);
                break; // failed return
            }
        }
    }

    if (ret == 0) {
        g_plt_funcs.printf("Loading done.\r\n");
    }

    // release stimulus pin
    g_plt_funcs.gpio_set(GPIO_IDX_STIMULUS_PIN, !STIMULUS_PIN_ACTIVE_STATE);

    return ret;
}
