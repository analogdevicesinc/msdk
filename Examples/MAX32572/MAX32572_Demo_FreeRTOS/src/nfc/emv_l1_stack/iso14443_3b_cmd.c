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

#include <emv_l1_stack/iso14443_3b_cmd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

int32_t iso_14443_3b_cmd_req_wup(uint8_t *atq, int32_t *atq_len, uint8_t doretry)
{
    uint8_t tx_buf[100];
    int32_t tx_len;
    uint8_t *rx_buf;
    uint32_t rx_len;
    int32_t ret;
    uint8_t retry = doretry ? 3 : 1;

    tx_buf[0] = 0x05;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x08;
    tx_len = 3;

    rx_buf = atq;

    do {
        ret = nfc_pcd_transceive(PROTOCOL_ISO14443B, FT_STANDARD_CRC_EMD, tx_buf, tx_len, rx_buf,
                                 &rx_len, ISO14443_FWT_ATQB);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            if ((rx_len != ISO3B_ATQB_MINLEN && rx_len != ISO3B_ATQB_MAXLEN) ||
                rx_buf[0] != ISO3B_ATQB_BYTE1) {
                ret = ISO14443_3_ERR_PROTOCOL;
            } else {
                ret = ISO14443_3_ERR_SUCCESS;
                *atq_len = rx_len;
            }
        }

        // EMV 2.6b case TB311, now enforces a minimum retransmission time of 3ms
        if ((retry > 1) && ret == ISO14443_3_ERR_TIMEOUT) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_ATQB);
        }

    } while (--retry && ret == ISO14443_3_ERR_TIMEOUT);

    return ret;
}

int32_t iso_14443_3b_cmd_attrib(uint8_t *pupi, uint8_t para1, uint8_t para2, uint8_t para3,
                                uint8_t para4, uint8_t *inf, uint32_t *inf_len, uint32_t timeout,
                                uint8_t *attrib_resp, int32_t *attrib_resp_len)
{
    uint8_t tx_buf[256];
    int32_t tx_len = 0;
    uint8_t rx_buf[256];
    uint32_t rx_len;
    int32_t ret;
    uint8_t retry = 3;

    tx_buf[tx_len++] = 0x1d;
    memcpy(tx_buf + tx_len, pupi, PUPI_SIZE);
    tx_len += PUPI_SIZE;

    tx_buf[tx_len++] = para1;
    tx_buf[tx_len++] = para2;
    tx_buf[tx_len++] = para3;
    tx_buf[tx_len++] = para4;

    if ((inf != NULL) && (inf_len != NULL)) {
        memcpy(tx_buf + tx_len, inf, *inf_len);
        tx_len += (*inf_len);
    }

    do {
        ret = nfc_pcd_transceive(PROTOCOL_ISO14443B, FT_STANDARD_CRC_EMD, tx_buf, tx_len, rx_buf,
                                 &rx_len, timeout);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            // If we have a non NULL buffer to save it, copy in the raw ATTRIB_RESPONSE
            if (attrib_resp) {
                memcpy(attrib_resp, rx_buf, rx_len);
                *attrib_resp_len = rx_len;
            }

            /*CID value should be 0*/
            if ((rx_buf[0] & 0x0f) != para4)
                ret = ISO14443_3_ERR_PROTOCOL;

            /*disregard MBLI*/

            /*high-inf should be empty*/
            if (rx_len != 1)
                ret = ISO14443_3_ERR_CONTAINS_HIGH_INF;
        }

        // EMV 2.6b case TB305, TB3012, now enforces a minimum retransmission time of 3ms
        if ((retry > 1) && (ret == ISO14443_3_ERR_TIMEOUT || ret == ISO14443_3_ERR_COLLISION)) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + timeout);
        }

    } while (--retry && (ret == ISO14443_3_ERR_TIMEOUT || ret == ISO14443_3_ERR_COLLISION));

    return ret;
}

int32_t iso_14443_3b_cmd_halt(uint8_t *pupi)
{
    uint8_t tx_buf[20];
    int32_t tx_len;
    uint8_t rx_buf[10];
    uint32_t rx_len;
    int32_t ret;

    tx_buf[0] = 0x50;
    memcpy(tx_buf + 1, pupi, 4);
    tx_len = 5;

    ret = nfc_pcd_transceive(PROTOCOL_ISO14443B, FT_STANDARD_CRC_EMD, tx_buf, tx_len, rx_buf,
                             &rx_len, ISO14443_FWT_DEFAULT);

    if (ret == ISO14443_3_ERR_SUCCESS) {
        if (rx_len != 1 || rx_buf[0] != 0x00) {
            ret = ISO14443_3_ERR_PROTOCOL;
        } else {
            ret = ISO14443_3_ERR_SUCCESS;
        }
    }
    return ret;
}
