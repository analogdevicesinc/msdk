/*******************************************************************************
 * Copyright (C) 2009-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *******************************************************************************
 *
 * @author: Yann Loisel <yann.loisel@maximintegrated.com>
 * @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
 *
 */

#include <ctype.h>
#include <errno.h>
#include <regex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "aes.h"
#include "ecdsa.h"
#include "maxim_c_utils.h"
#include "read_file.h"
#include "rsa.h"
#include "scp.h"
#include "scp_definitions.h"
#include "scp_utils.h"
#include "session_build.h"
#include <log.h>
#include <process.h>
#include <utils.h>

#ifndef MAX_ARG_LEN
#define MAX_ARG_LEN 4096
#endif

u8 response[UCL_AES_BLOCKSIZE];

u8 crk_rsa_modulus[RSA_4096_MODULUS_LEN];
u8 crk_rsa_pubexp[RSA_PUBLIC_EXPONENT_LEN];
u8 mrk_signature_g[RSA_4096_MODULUS_LEN];
u8 crk_rsa_modulus2[RSA_4096_MODULUS_LEN];
u8 crk_rsa_pubexp2[RSA_PUBLIC_EXPONENT_LEN];
u8 mrk_signature2[RSA_4096_MODULUS_LEN];

u8 crk_ecdsa_x[ECDSA_MODULUS_LEN];
u8 crk_ecdsa_y[ECDSA_MODULUS_LEN];
u8 crk_ecdsa_x2[ECDSA_MODULUS_LEN];
u8 crk_ecdsa_y2[ECDSA_MODULUS_LEN];
u8 mrk_ecdsa_r[ECDSA_MODULUS_LEN];
u8 mrk_ecdsa_s[ECDSA_MODULUS_LEN];
u8 mrk_ecdsa_r2[ECDSA_MODULUS_LEN];
u8 mrk_ecdsa_s2[ECDSA_MODULUS_LEN];

int ch_id;

static const uint8_t hello_scp_req_const[HELLO_SCP_REQ_CONST_LEN]
    = { 'H', 'E', 'L', 'L', 'O', ' ', 'B', 'L', 0x00 };
static const uint8_t hello_scp_rep_const[HELLO_SCP_REP_CONST_LEN]
    = { 'H', 'E', 'L', 'L', 'O', ' ', 'H', 'O', 'S', 'T' };

const char idf_cmd[MAX_IDF][32] = { [1] = "HELLO",
    [2] = "HELLO_REP",
    [7] = "CHLG",
    [3] = "SUCCESS",
    [4] = "FAILURE",
    [5] = "DATA",
    [8] = "HELLO_OFF",
    [9] = "HELLO_OFF_REP" };

/* DATA Link Segment */
/**************************************************************************************************/

int data_link(ctl_code_t ctl_code, const uint8_t* payload_l, uint16_t payload_len, const char* msg)
{
    unsigned int i = 0;
    int result = 0;
    static int sequence = -1;
    u8 crc[16];
    unsigned int ch_idf = 9;
    uint8_t data_frame[MAX_FRAME];
    size_t iframe = 0;

    data_frame[iframe++] = SYNCH1;
    data_frame[iframe++] = SYNCH2;
    data_frame[iframe++] = SYNCH3;
    data_frame[iframe++] = ctl_code;

    /* len */
    data_frame[iframe++] = payload_len >> 8;
    data_frame[iframe++] = payload_len & 255;

    if (ctl_code == DISC_REQ || ctl_code == DATA_TRANSFER) {
        sequence++;
    }

    /* add_channel_id_seq */
    data_frame[iframe++] = (ch_idf << 4) + (((sequence < 0) ? 0 : sequence) & 15);

    /* header_crc */
    aes_checksum(crc, data_frame, iframe, 1);
    data_frame[iframe++] = crc[0];

    if (payload_len != 0) {
        /* Add payload */
        for (i = 0; i < payload_len; i++) {
            data_frame[iframe++] = payload_l[i];
        }

        /* Add Payload CRC */
        ASSERT_OK(aes_checksum(crc, payload_l, payload_len, 4));

        for (i = 4; i != 0; i--) {
            data_frame[iframe++] = crc[i - 1];
        }
    }

    return packet_send(data_frame, iframe, idf_ctl[ctl_code], msg);
}

int connection_request(void)
{
    return data_link(CON_REQ, NULL, 0, "connection_request");
}

int connection_reply(void)
{
    return data_link(CON_REP, NULL, 0, "connection_reply");
}

int disconnection_request(void)
{
    return data_link(DISC_REQ, NULL, 0, "disconnection_request");
}

int disconnection_reply(void)
{
    return data_link(DISC_REP, NULL, 0, "disconnection_reply");
}

int ack(void)
{
    return data_link(ACK, NULL, 0, "ack");
}

int echo_req(void)
{
    return data_link(ECHO_REQ, NULL, 0, "echo_request");
}

int echo_reply(void)
{
    return data_link(ECHO_REP, NULL, 0, "echo_reply");
}

int dump(void)
{
    uint8_t data_frame[MAX_FRAME];
    size_t iframe = 0;

    return packet_send(data_frame, iframe, "", "dump");
}

/* Session Layer */
/**************************************************************************************************/

int session_layer(session_cmd_t cmd, const uint8_t* data, uint16_t data_len, const char* msg)
{
    uint8_t segment[MAX_SEGMENT];
    uint8_t signature[UCL_RSA_KEY_MAXSIZE];
    size_t isegment = 0;
    size_t signature_len = 0;
    unsigned int i = 0;
    static uint8_t tr_id = 0;
    int result;

    if (cmd == DATA) {
        segment[isegment++] = (cmd << 4) ^ config_g.pp;
    } else {
        segment[isegment++] = (cmd << 4);
    }

    segment[isegment++] = tr_id;

    /* The “Transaction ID” field is one byte. The initial value is fixed to zero. The
     *	Transaction ID is incremented after each data transfer ended successfully (modulo 256).
     */
    if (TARGET == whoami() && cmd == DATA) {
        tr_id = (tr_id + 1) % 256;
    }

    /* data payload length */
    segment[isegment++] = data_len >> 8;
    segment[isegment++] = data_len & 0xFF;

    if (data_len != 0) {
        /* Add payload */
        for (i = 0; i < data_len; i++) {
            segment[isegment++] = data[i];
        }

        /* Only Command are Signed */
        if (HOST == whoami() && cmd == DATA) {
            /* Compute payload signature */
            if (SCP_RSA == config_g.session_mode || SCP_FLORA_RSA == config_g.session_mode
                || SCP_PAOLA == config_g.session_mode) {
                ASSERT_OK(rsa_sign(data, data_len, signature, config_g.rsaKey));
                signature_len = config_g.rsaKey.keyPr.modulus_length;
            } else if (SCP_ANGELA_ECDSA == config_g.session_mode) {
                ASSERT_OK(ecdsa_sign(data, data_len, signature, config_g.ecdsaKey));
                signature_len = config_g.ecdsaKey.ecdsa_len * 2;
            } else {
                print_error("Unknown session Mode \n");
                return ERR_BAD_MODE;
            }

            /* Add signature */
            memcpy(&segment[isegment], signature, signature_len);
            isegment += signature_len;
        }
    }

    return data_link(DATA_TRANSFER, segment, isegment, msg);
}

int generic_response(char* msg)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = 0;
    payload[ipayload++] = 0;
    payload[ipayload++] = 0;
    payload[ipayload++] = 0;

    return session_layer(DATA, payload, ipayload, msg);
    {
    }
}

int hello_request(void)
{
    int i;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    for (i = 0; i < HELLO_SCP_REQ_CONST_LEN; i++) {
        payload[ipayload++] = hello_scp_req_const[i];
    }

    /* from the specs, the hello are the same for these two modes */
    if ((SCP_RSA == config_g.session_mode) || (SCP_FLORA_RSA == config_g.session_mode)
        || (SCP_PAOLA == config_g.session_mode)) {
        payload[ipayload - 1] = 0x02;
    }

    if ((SCP_ANGELA_ECDSA == config_g.session_mode)) {
        payload[ipayload - 1] = 0x03;
    }

    /* Admin mode always used for SCP_FLORA */
    payload[ipayload++] = 0x02;

    return session_layer(HELLO_REQ, payload, ipayload, "hello_request");
    {
    }
}

int hello_reply(void)
{
    int i;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    for (i = 0; i < HELLO_SCP_REP_CONST_LEN; i++) {
        payload[ipayload++] = hello_scp_rep_const[i];
    }

    if (SCP_FLORA_RSA == config_g.session_mode || SCP_ANGELA_ECDSA == config_g.session_mode
        || SCP_PAOLA == config_g.session_mode) {
        /* flora major version, one byte */
        payload[ipayload++] = 0x01;
        /* flora minor version, one byte */
        payload[ipayload++] = 0x00;
        /* four RFU bytes */
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x00;
        /* 0x00 byte */
        payload[ipayload++] = 0x00;
        /* configuration byte */
        /* jtag and rwk enabled */
        payload[ipayload++] = (1 << 7) ^ (1 << 6);
    } else {
        /* major version number, on 4 bytes */
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x01;
        /* rfu byte, 0x00 */
        payload[ipayload++] = 0x00;
        /* annex identifier */
        payload[ipayload++] = 0x00;
        payload[ipayload++] = 0x01;
        payload[ipayload++] = 0x01;
    }

    /* usn */
    for (i = 0; i < USN_LEN; i++) {
        payload[ipayload++] = config_g.usn[i];
    }

    /* for flora modes, pad with 0 up to 16 bytes */
    for (i = USN_LEN; i < 16; i++) {
        payload[ipayload++] = 0;
    }

    /* random number */
    for (i = 0; i < 16; i++) {
        payload[ipayload++] = 0x00;
    }

    return session_layer(HELLO_REP, payload, ipayload, "hello_reply");
}

int write_crk(char* signaturefile)
{
    int rsa_explen = RSA_PUBLIC_EXPONENT_LEN;
    int ecdsa_len = ECDSA_MODULUS_LEN;
    size_t rsa_len;
    size_t mrk_signature_len;
    int result;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_CRK >> 8;
    payload[ipayload++] = WRITE_CRK & 255;

    if (SCP_ANGELA_ECDSA == config_g.session_mode) {
        ASSERT_OK(read_file_signed_ecdsa_publickey(
            crk_ecdsa_x, crk_ecdsa_y, mrk_ecdsa_r, mrk_ecdsa_s, ecdsa_len, signaturefile));

        payload[ipayload++] = (ecdsa_len * 4) >> 8;
        payload[ipayload++] = (ecdsa_len * 4) & 255;

        memcpy(&payload[ipayload], crk_ecdsa_x, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], crk_ecdsa_y, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], mrk_ecdsa_r, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], mrk_ecdsa_s, ecdsa_len);
        ipayload += ecdsa_len;

    } else {
        ASSERT_OK(read_file_signed_rsa_publickey(crk_rsa_modulus, &rsa_len, crk_rsa_pubexp,
            rsa_explen, mrk_signature_g, &mrk_signature_len, signaturefile));

        payload[ipayload++] = (rsa_len + mrk_signature_len + rsa_explen) >> 8;
        payload[ipayload++] = (rsa_len + mrk_signature_len + rsa_explen) & 255;

        memcpy(&payload[ipayload], crk_rsa_modulus, rsa_len);
        ipayload += rsa_len;

        memcpy(&payload[ipayload], crk_rsa_pubexp, rsa_explen);
        ipayload += rsa_explen;

        memcpy(&payload[ipayload], mrk_signature_g, mrk_signature_len);
        ipayload += mrk_signature_len;
    }

    return session_layer(DATA, payload, ipayload, "write_crk");
}

int rewrite_crk(const char* oldsignaturefile, const char* newsignaturefile)
{
    int ecdsa_len = ECDSA_MODULUS_LEN;
    size_t rsa_len = RSA_4096_MODULUS_LEN;
    size_t rsa_len2 = RSA_4096_MODULUS_LEN;
    int rsa_pubexplen = RSA_PUBLIC_EXPONENT_LEN;
    int result;
    size_t mrk_signature_len;
    size_t mrk_signature2_len;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = REWRITE_CRK >> 8;
    payload[ipayload++] = REWRITE_CRK & 255;

    if (SCP_PAOLA == config_g.session_mode) {
        ASSERT_OK(read_file_signed_rsa_publickey(crk_rsa_modulus, &rsa_len, crk_rsa_pubexp,
            rsa_pubexplen, mrk_signature_g, &mrk_signature_len, oldsignaturefile));

        ASSERT_OK(read_file_signed_rsa_publickey(crk_rsa_modulus2, &rsa_len2, crk_rsa_pubexp2,
            rsa_pubexplen, mrk_signature2, &mrk_signature2_len, newsignaturefile));

        print_debug("rsa_len=" SSIZET_FMT " mrk len=" SSIZET_FMT "\n", rsa_len, mrk_signature_len);

        payload[ipayload++]
            = (rsa_len + rsa_pubexplen + rsa_len2 + rsa_pubexplen + mrk_signature2_len) >> 8;
        payload[ipayload++]
            = (rsa_len + rsa_pubexplen + rsa_len2 + rsa_pubexplen + mrk_signature2_len) & 255;

        memcpy(&payload[ipayload], crk_rsa_modulus, rsa_len);
        ipayload += rsa_len;

        memcpy(&payload[ipayload], crk_rsa_pubexp, rsa_pubexplen);
        ipayload += rsa_pubexplen;

        memcpy(&payload[ipayload], crk_rsa_pubexp2, rsa_len2);
        ipayload += rsa_len2;

        memcpy(&payload[ipayload], crk_rsa_modulus2, rsa_pubexplen);
        ipayload += rsa_pubexplen;

        memcpy(&payload[ipayload], mrk_signature2, mrk_signature2_len);
        ipayload += mrk_signature2_len;
    }

    if (SCP_ANGELA_ECDSA == config_g.session_mode) {
        ASSERT_OK(read_file_signed_ecdsa_publickey(
            crk_ecdsa_x, crk_ecdsa_y, mrk_ecdsa_r, mrk_ecdsa_s, ecdsa_len, oldsignaturefile));
        ASSERT_OK(read_file_signed_ecdsa_publickey(
            crk_ecdsa_x2, crk_ecdsa_y2, mrk_ecdsa_r2, mrk_ecdsa_s2, ecdsa_len, newsignaturefile));

        payload[ipayload++] = (2 * ecdsa_len + 2 * ecdsa_len + 2 * ecdsa_len) >> 8;
        payload[ipayload++] = (2 * ecdsa_len + 2 * ecdsa_len + 2 * ecdsa_len) & 255;

        memcpy(&payload[ipayload], crk_ecdsa_x, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], crk_ecdsa_y, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], crk_ecdsa_x2, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], crk_ecdsa_y2, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], mrk_ecdsa_r2, ecdsa_len);
        ipayload += ecdsa_len;

        memcpy(&payload[ipayload], mrk_ecdsa_s2, ecdsa_len);
        ipayload += ecdsa_len;
    }

    return session_layer(DATA, payload, ipayload, "rewrite_crk");
}

int del_mem(char* sfilename, size_t address_offset)
{
    int result;
    size_t start_addr;
    size_t end_addr;
    int i;
    int range;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    print_debug("SCP - del_mem\n");

    if (extension("s19", sfilename)) {
        ASSERT_OK(get_start_addr_and_length_s19(sfilename, &start_addr, &end_addr));
    } else if (extension("sbin", sfilename)) {
        start_addr = address_offset;
        read_file_size(&end_addr, sfilename);
        end_addr += start_addr;
    } else {
        print_error("File extension not supported %s (only .s19 and sbin)\n", sfilename);
        return ERR_UNSUPPORTED_EXT;
    }

    payload[ipayload++] = ERASE_MEM >> 8;
    payload[ipayload++] = ERASE_MEM & 255;

    print_debug("address_offset is: " SSIZET_XFMT " bytes\n", address_offset);

    /* set up the start addr on 4 bytes */
    for (i = 3; i >= 0; i--) {
        payload[ipayload++] = (start_addr >> (8 * i)) & 0xFF;
    }

    /* set up the length on 4 bytes */
    range = (end_addr - start_addr);

    if ((range % 16) != 0) {
        range = ((range / 16) + 1) * 16;
    }

    print_debug("erase length is: %d bytes\n", range);

    for (i = 3; i >= 0; i--) {
        payload[ipayload++] = ((range) >> (8 * i)) & 255;
    }

    return session_layer(DATA, payload, ipayload, "del_mem");
}

int write_mem(const uint8_t* data, size_t data_len, size_t data_addr)
{
    unsigned int i;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_MEM >> 8;
    payload[ipayload++] = WRITE_MEM & 255;

    /* set up the start addr on 4 bytes */
    for (i = 4; i != 0; i--) {
        payload[ipayload++] = (data_addr >> (8 * (i - 1))) & 255;
    }

    /* set up the length */
    for (i = 4; i != 0; i--) {
        payload[ipayload] = (data_len >> (8 * (i - 1))) & 255;
        ipayload++;
    }

    for (i = 0; i < data_len; i++) {
        payload[ipayload++] = data[i];
    }

    return session_layer(DATA, payload, ipayload, "write_mem");
}

int verify_data(const uint8_t* data, size_t data_len, size_t data_addr)
{
    unsigned int i;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = VERIFY_MEM >> 8;
    payload[ipayload++] = VERIFY_MEM & 255;

    /* set up the start addr on 4 bytes */
    for (i = 4; i != 0; i--) {
        payload[ipayload++] = (data_addr >> (8 * (i - 1))) & 255;
    }

    /* set up the length on 4 bytes */
    for (i = 4; i != 0; i--) {
        payload[ipayload++] = (data_len >> (8 * (i - 1))) & 255;
    }

    for (i = 0; i < data_len; i++) {
        payload[ipayload++] = data[i];
    }

    return session_layer(DATA, payload, ipayload, "verify_data");
}

int del_data(size_t start_addr, size_t length)
{
    int i;
    unsigned int end_addr, range;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    end_addr = start_addr + length;

    payload[ipayload++] = ERASE_MEM >> 8;
    payload[ipayload++] = ERASE_MEM & 255;

    /* set up the start addr on 4 bytes */
    for (i = 3; i >= 0; i--) {
        payload[ipayload++] = (start_addr >> (8 * i)) & 255;
    }

    /* set up the length on 4 bytes */
    range = (end_addr - start_addr);

    if ((range % 16) != 0) {
        range = ((range / 16) + 1) * 16;
    }

    print_debug("erase length is: %d bytes\n", range);

    for (i = 3; i >= 0; i--) {
        payload[ipayload++] = ((range) >> (8 * i)) & 255;
    }

    return session_layer(DATA, payload, ipayload, "del_data");
}

/* SCP FLORA command */
int write_timeout(scp_target_t timeout_target_char, size_t timeout_value)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_TIMEOUT >> 8;
    payload[ipayload++] = WRITE_TIMEOUT & 255;

    payload[ipayload++] = timeout_c[timeout_target_char].target;

    payload[ipayload++] = timeout_value >> 8;
    payload[ipayload++] = timeout_value & 255;

    print_debug("Timeout %c : " SSIZET_FMT "\n", timeout_c[timeout_target_char].timeout_char,
        timeout_value);

    return session_layer(DATA, payload, ipayload, "write_timeout");
}

int write_params(scp_target_t param_target_char, const uint8_t* param_value)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_PARAMS >> 8;
    payload[ipayload++] = WRITE_PARAMS & 255;

    payload[ipayload++] = timeout_c[param_target_char].target;

    payload[ipayload++] = param_value[0];
    payload[ipayload++] = param_value[1];
    payload[ipayload++] = param_value[2];
    payload[ipayload++] = param_value[3];
    payload[ipayload++] = param_value[4];
    payload[ipayload++] = param_value[5];

    return session_layer(DATA, payload, ipayload, "write_params");
}

int write_stims(scp_target_t stims_target_char, uint8_t stims_value)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_STIM >> 8;
    payload[ipayload++] = WRITE_STIM & 255;

    payload[ipayload++] = timeout_c[stims_target_char].target;

    payload[ipayload++] = stims_value;

    return session_layer(DATA, payload, ipayload, "write_stims");
}

int write_deact(scp_target_t deact_target_char)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_DEACT >> 8;
    payload[ipayload++] = WRITE_DEACT & 255;

    payload[ipayload++] = timeout_c[deact_target_char].target;

    return session_layer(DATA, payload, ipayload, "write_deact");
}

/* SCP FLORA command */
int kill_chip(void)
{
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = KILL_CHIP >> 8;
    payload[ipayload++] = KILL_CHIP & 255;

    return session_layer(DATA, payload, ipayload, "kill_chip");
}

/* SCP FLORA command */
int kill_chip2(void)
{
    int i = 0;
    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = KILL_CHIP2 >> 8;
    payload[ipayload++] = KILL_CHIP2 & 255;

    for (i = 0; i < USN_LEN; i++) {
        payload[ipayload++] = config_g.usn[i];
    }

    return session_layer(DATA, payload, ipayload, "kill_chip2");
}

int execute_code(size_t address)
{
    unsigned int i;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = EXECUTE_CODE >> 8;
    payload[ipayload++] = EXECUTE_CODE & 255;

    print_debug("address: " SSIZET_XFMT " \n", address);

    for (i = 4; i != 0; i--) {
        payload[ipayload++] = ((address) >> (8 * (i - 1))) & 255;
        {
        }
    }

    return session_layer(DATA, payload, ipayload, "execute_code");
}

int write_app_version(size_t version)
{
    unsigned int i;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_APP_VER >> 8;
    payload[ipayload++] = WRITE_APP_VER & 255;

    print_debug("Version : " SSIZET_XFMT " \n", version);

    for (i = 4; i != 0; i--) {
        payload[ipayload++] = ((version) >> (8 * (i - 1))) & 255;
        {
        }
    }

    return session_layer(DATA, payload, ipayload, "write_app_version");
}

/* SCP FLORA command */
int write_otp(size_t offset, const uint8_t* data, size_t data_length)
{
    unsigned int i;

    uint8_t payload[MAX_FRAME];
    size_t ipayload = 0;

    payload[ipayload++] = WRITE_OTP >> 8;
    payload[ipayload++] = WRITE_OTP & 255;
    payload[ipayload++] = (2 + data_length) >> 8;
    payload[ipayload++] = (2 + data_length) & 255;

    payload[ipayload++] = offset >> 8;
    payload[ipayload++] = offset & 255;

    print_debug("offset:" SSIZET_XFMT "\n", offset);

    print_debug("data:");
    for (i = 0; i < data_length; i++) {
        print_d("%c", data[i]);
    }
    print_d("\n");

    for (i = 0; i < data_length; i++) {
        payload[ipayload++] = data[i];
    }

    return session_layer(DATA, payload, ipayload, "write_otp");
}

/* META COMMAND */
/**************************************************************************************************/

int write_file(char* sfilename, size_t address_offset)
{
    int result;
    /* write-file includes automatic erasure of concerned area; */

    ASSERT_OK(del_mem(sfilename, address_offset));

    target();
    ASSERT_OK(ack());
    ASSERT_OK(generic_response("del_mem_response"));
    host();
    ASSERT_OK(ack());

    ASSERT_OK(write_only(sfilename, address_offset));

    return ERR_OK;
}

int write_only(char* sfilename, size_t address_offset)
{
    unsigned int i;
    int result;
    u8 chunk[MAX_CHUNK_SIZE];
    size_t chunk_len;
    size_t new_chunk_size;
    int chunk_addr;

    size_t data_len = sizeof(u8) * 1024 * 1024 * config_g.flash_mb;
    uint8_t* data = malloc(data_len);
    if (NULL == data) {
        print_error(
            "Unable to allocate memory for binary data (%dMB requested)\n", config_g.flash_mb);
        return ERR_MEMORY_ERROR;
    }

    if (extension("s19", sfilename)) {
        ASSERT_OK(read_s19_file(sfilename, 0, data, &data_len, addr_g));
    } else if (extension("sbin", sfilename)) {
        ASSERT_OK(read_binary_file(sfilename, data, &data_len));
    } else {
        free(data);
        print_error("Unsupported file extension: %s (only .sbin and .s19)\n", sfilename);
        return ERR_UNSUPPORTED_EXT;
    }

    /* 10 is the write-data command len: 2 bytes for the command, 4 bytes for the data length, 4
     * bytes for the data address */
    new_chunk_size = config_g.chunk_size - HEADER_LEN - CRC_LEN - COMMAND_LEN - 10;
    /* if the mode is RSA mode, the signature len shall also be taken into account */
    if (SCP_FLORA_RSA == config_g.session_mode || SCP_PAOLA == config_g.session_mode) {
        new_chunk_size -= config_g.rsaKey.keyPr.modulus_length;
    }

    print_debug(SSIZET_FMT " new chunk_size=" SSIZET_FMT "\n", config_g.chunk_size, new_chunk_size);

    i = 0;
    /* parse data */
    /* grouping them by contiguous addresses, up to CHUNK-SIZE */
    /* meaning each time an address is not the next one than the previous one */
    /* or each time a block of CHUNK-SIZE data has been built, */
    /* a write-mem is issued */
    /* until the end. */

    while (i < data_len) {
        chunk_len = 0;
        chunk[chunk_len] = data[i];
        /* the block starting address (needed for write-mem) */
        chunk_addr = addr_g[i] + address_offset;
        chunk_len++;
        i++;
        /* while consecutive addresses and not too big chunk */
        while ((addr_g[i] == addr_g[i - 1] + 1) && (chunk_len < new_chunk_size) && (i < data_len)) {
            chunk[chunk_len] = data[i];
            chunk_len++;
            i++;
        }
        /* the last packet is filled up with FF if needed */
        /* 2.3.6 (#2252): not filled up with FF anymore */
        if (i == data_len) {
            print_debug("last chunk (" SSIZET_FMT " bytes):", chunk_len);
        }

        ASSERT_OK(write_mem(chunk, chunk_len, chunk_addr));

        target();
        ack();
        ASSERT_OK(generic_response("write_mem_response"));
        host();
        ack();
    }

    return ERR_OK;
}

int verify_file(char* sfilename, size_t address_offset, char b_dump)
{
    unsigned int i;
    int result;
    u8 chunk[MAX_CHUNK_SIZE];
    size_t chunk_len;
    size_t new_chunk_size;
    int chunk_addr;

    size_t data_len = sizeof(u8) * 1024 * 1024 * config_g.flash_mb;
    uint8_t* data = malloc(data_len);
    if (NULL == data) {
        print_error(
            "Unable to allocate memory for binary data (%dMB requested)\n", config_g.flash_mb);
        return ERR_MEMORY_ERROR;
    }

    if (extension("s19", sfilename)) {
        ASSERT_OK(read_s19_file(sfilename, address_offset, data, &data_len, addr_g));
    } else if (extension("sbin", sfilename)) {
        ASSERT_OK(read_binary_file(sfilename, data, &data_len));
    } else {
        free(data);
        print_error("Unsupported file extension: %s (only .sbin and .s19)\n", sfilename);
        return ERR_UNSUPPORTED_EXT;
    }

    /* 10 is the write-data command len: 2 bytes for the command, 4 bytes for the data length, 4
     * bytes for the data address */
    new_chunk_size = config_g.chunk_size - HEADER_LEN - CRC_LEN - COMMAND_LEN - 10;
    /* if the mode is RSA mode, the signature len shall also be taken into account */
    if (SCP_FLORA_RSA == config_g.session_mode || SCP_PAOLA == config_g.session_mode) {
        new_chunk_size -= config_g.rsaKey.keyPr.modulus_length;
    }

    print_debug(SSIZET_FMT " new chunk_size=" SSIZET_FMT "\n", config_g.chunk_size, new_chunk_size);

    i = 0;
    /* parse data */
    /* grouping them by contiguous addresses, up to CHUNK-SIZE */
    /* meaning each time an address is not the next one than the previous one */
    /* or each time a block of CHUNK-SIZE data has been built, */
    /* a write-mem is issued */
    /* until the end. */
    while (i < data_len) {
        chunk_len = 0;
        chunk[chunk_len] = data[i];
        /* the block starting address (needed for write-mem) */
        chunk_addr = addr_g[i];
        chunk_len++;
        i++;
        /* while consecutive addresses and not too big chunk */
        while ((addr_g[i] == addr_g[i - 1] + 1) && (chunk_len < new_chunk_size) && (i < data_len)) {
            chunk[chunk_len] = data[i];
            chunk_len++;
            i++;
        }
        /* the last packet is filled up with FF if needed */
        /* 2.3.6 (#2252): not filled up with FF anymore */
        if (i == data_len) {
            print_debug("last chunk (" SSIZET_FMT " bytes):", chunk_len);
        }

        ASSERT_OK(verify_data(chunk, chunk_len, chunk_addr));

        target();
        ack();
        if (b_dump == TRUE) {
            dump();
        }
        ASSERT_OK(generic_response("verify_data_response"));
        host();
        ack();
    }

    return ERR_OK;
}

int parse_scp_script(const char* filename, script_cmd_list_t** script, size_t* list_length)
{
    char line[MAX_STRING];
    unsigned int i;
    unsigned int list_index = 0;
    FILE* pFile;

    print_debug("<session %s>\n", mode_name[config_g.session_mode]);

    *script = malloc(MAX_SCP_SCRIPT_LINE * sizeof(script_cmd_list_t));
    if (script == NULL) {
        print_error("Unable to allocate memory");
        return ERR_MEMORY_ERROR;
    }

    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        print_error("on opening %s : %s\n", filename, strerror(errno));
        return errno;
    }

    while (fgets(line, MAX_STRING, pFile) != NULL) {
        regex_t regex;
        regmatch_t rm[10];
        int result;
        char name[MAX_ARG_LEN];
        char param[3][MAX_ARG_LEN];
        size_t memcpyLen;

        size_t nb_params = 0;

        print_debug("Script line : %s", line);

        result
            = regcomp(&regex, "^\\s*([0-9a-zA-Z-]+)( (\\S+))?( (\\S+))?( (\\S+))?", REG_EXTENDED);
        if (result) {
            return ERR_INTERNAL_ERROR;
        }

        result = regexec(&regex, line, 10, rm, 0);
        if (!result) {
            memcpyLen = rm[1].rm_eo - rm[1].rm_so;
            if (memcpyLen > MAX_ARG_LEN) {
                print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                return ERR_MEMORY_ERROR;
            }
            memcpy(name, &(line[rm[1].rm_so]), memcpyLen);
            name[rm[1].rm_eo - rm[1].rm_so] = '\0';
            if (rm[3].rm_eo != -1) {
                memcpyLen = rm[3].rm_eo - rm[3].rm_so;
                if (memcpyLen > MAX_ARG_LEN) {
                    print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                    return ERR_MEMORY_ERROR;
                }
                memcpy(param[0], &(line[rm[3].rm_so]), memcpyLen);
                param[0][rm[3].rm_eo - rm[3].rm_so] = '\0';
                replace_extra_params(param[0]);
                nb_params++;

                if (rm[5].rm_eo != -1) {
                    memcpyLen = rm[5].rm_eo - rm[5].rm_so;
                    if (memcpyLen > MAX_ARG_LEN) {
                        print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                        return ERR_MEMORY_ERROR;
                    }
                    memcpy(param[1], &(line[rm[5].rm_so]), memcpyLen);
                    param[1][rm[5].rm_eo - rm[5].rm_so] = '\0';
                    replace_extra_params(param[1]);
                    nb_params++;

                    if (rm[7].rm_eo != -1) {
                        memcpyLen = rm[7].rm_eo - rm[7].rm_so;
                        if (memcpyLen > MAX_ARG_LEN) {
                            print_error("Argument length overflow, max length is %d", MAX_ARG_LEN);
                            return ERR_MEMORY_ERROR;
                        }
                        memcpy(param[2], &(line[rm[7].rm_so]), memcpyLen);
                        param[2][rm[7].rm_eo - rm[7].rm_so] = '\0';
                        replace_extra_params(param[2]);
                        nb_params++;
                    }
                }
            }
        } else {
            if ('#' != line[0]) {
                print_warn("Malformed script line : %s \n", line);
            }

            continue;
        }

        regfree(&regex);

        i = 0;

        // clear buffer before use
        memset(&(*script)[list_index], 0, sizeof((*script)[list_index]));

        while (idf_scp_cmd[i].name != 0) {
            if (strcmp(idf_scp_cmd[i].name, name) == 0) {
                print_d("\tCommand: %s\n", idf_scp_cmd[i].name);
                (*script)[list_index].cmd = i;

                unsigned int j;
                for (j = 0; j < 3; j++) {
                    print_d("\tParam %d: %s\n", j, param[j]);

                    if (idf_scp_cmd[i].param[j].type == 0 && nb_params > j + 1) {
                        print_error("Too many parameter for %s (expected 0, actually " SSIZET_FMT
                                    ")\n",
                            idf_scp_cmd[i].name, nb_params);
                        return ERR_TOO_MANY_ARGS;
                    }

                    if (nb_params >= j + 1) {
                        char* new_param
                            = str_replace(param[j], "%MAXIM_SBT_DIR%", config_g.fullpath);

                        ASSERT_OK(parse_store(idf_scp_cmd[i].param[j].type,
                            &((*script)[list_index].param[j]), new_param,
                            idf_scp_cmd[i].param[j].min));

                        print_d("\tNew Params %d: %s\n", j, new_param);
                        free(new_param);

                        print_debug("param 1 : " SSIZET_XFMT "\n",
                            *(size_t*)(*script)[list_index].param[j]);
                        if (idf_scp_cmd[i].param[j].type == OT_FILE) {
                            if (!file_exist((char*)(*script)[list_index].param[j])) {
                                print_error(
                                    "File %s not found.\n", (char*)(*script)[list_index].param[j]);
                                return ERR_FILE_NOT_FOUND;
                            }
                        }

                    } else if (idf_scp_cmd[i].param[j].mandatory) {
                        print_error(
                            "Not enough parameters for %s (need at least %d, actually " SSIZET_FMT
                            ")\n",
                            idf_scp_cmd[i].name, j + 1, nb_params);
                        return ERR_MISSING_ARGS;
                    }
                }

                list_index++;

                break;
            }

            i++;
        }
    }

    *list_length = list_index;

    return ERR_OK;
}

int process_script(script_cmd_list_t* script, size_t list_length)
{
    unsigned int i;
    int result;
    size_t val;

    print_debug("<session %s>\n", mode_name[config_g.session_mode]);
    fprintf(fp_g, "<session %s>\n", mode_name[config_g.session_mode]);

    open_packetlist_file();

    host();
    connection_request();
    target();
    connection_reply();
    host();
    ack();
    host();
    hello_request();
    target();
    ack();
    hello_reply();
    host();
    ack();

    for (i = 0; i < list_length; i++) {
        print_debug("Command : %s\n", idf_scp_cmd[script[i].cmd].name);

        switch (script[i].cmd) {
        case COMMAND_WRITE_FILE:
            ASSERT_OK(write_file((char*)script[i].param[0], *((size_t*)script[i].param[1])));
            break;

        case COMMAND_WRITE_ONLY:
            ASSERT_OK(write_only((char*)script[i].param[0], *(size_t*)script[i].param[1]));
            break;

        case COMMAND_ERASE_DATA:
            ASSERT_OK(del_data(*((size_t*)script[i].param[0]), *((size_t*)script[i].param[1])));
            target();
            ASSERT_OK(ack());
            if (*((char*)script[i].param[2])) {
                dump();
            }
            ASSERT_OK(generic_response("del_mem_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_VERIFY_FILE:
            ASSERT_OK(verify_file((char*)script[i].param[0], *((size_t*)script[i].param[1]),
                *((char*)script[i].param[2])));
            break;

        case COMMAND_WRITE_CRK:
            ASSERT_OK(write_crk((char*)script[i].param[0]));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_crk_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_REWRITE_CRK:
            ASSERT_OK(
                rewrite_crk((const char*)script[i].param[0], (const char*)script[i].param[1]));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("rewrite_crk_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_ECHO:
            ASSERT_OK(echo_req());
            target();
            ASSERT_OK(echo_reply());
            host();
            break;

        case COMMAND_WRITE_OTP:
            // 0th index of array keeps len of array so pass it if not needed
            val = (script[i].param[0][1] << 8) | script[i].param[0][2];
            ASSERT_OK(
                write_otp(val, (uint8_t*)&(script[i].param[1][1]), (size_t)script[i].param[1][0]));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_otp_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_WRITE_TIMEOUT:
            // 0th index of array keeps len of array so pass it if not needed
            val = (script[i].param[1][1] << 8) | script[i].param[1][2];
            ASSERT_OK(write_timeout(*((scp_target_t*)script[i].param[0]), val));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_timeout_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_KILL_CHIP2:
            ASSERT_OK(kill_chip2());
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("kill_chip_response"));
            host();
            ASSERT_OK(ack());
            break;
        case COMMAND_KILL_CHIP:
            ASSERT_OK(kill_chip());
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("kill_chip_response"));
            host();
            ASSERT_OK(ack());
            break;

        case COMMAND_EXECUTE_CODE:
            ASSERT_OK(execute_code(*((size_t*)script[i].param[0])));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("execute_code_response"));
            host();
            ASSERT_OK(ack());
            break;
        case COMMAND_WRITE_APP_VER:
            ASSERT_OK(execute_code(*((size_t*)script[i].param[0])));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_app_version_response"));
            host();
            ASSERT_OK(ack());
            break;
        case COMMAND_WRITE_STIM:
            // 0th index of array keeps len of array so pass it if not needed
            val = *(size_t*)&(script[i].param[1][1]);
            ASSERT_OK(write_stims(*((scp_target_t*)script[i].param[0]), val));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_stim_response"));
            host();
            ASSERT_OK(ack());
            break;
        case COMMAND_WRITE_PARAMS:
            // 0th index of array keeps len of array so pass it if not needed
            ASSERT_OK(write_params(
                *((scp_target_t*)script[i].param[0]), (uint8_t*)&(script[i].param[1][1])));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_params_response"));
            host();
            ASSERT_OK(ack());
            break;
        case COMMAND_WRITE_DEACT:
            ASSERT_OK(write_deact(*((scp_target_t*)script[i].param[0])));
            target();
            ASSERT_OK(ack());
            ASSERT_OK(generic_response("write_deact_response"));
            host();
            ASSERT_OK(ack());
            break;

        default:
            print_error("Unsupported command %s\n", idf_scp_cmd[script[i].cmd].name);
            return ERR_CMD_UNKNOWN;
        }
    }

    disconnection_request();
    target();
    disconnection_reply();

    close_packetlist_file();

    return ERR_OK;
}
