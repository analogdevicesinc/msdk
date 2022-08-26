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

#include <emv_l1_stack/iso14443_3_common.h>
#include <emv_l1_stack/iso14443_3a_cmd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define SAK_LEN                 1


int32_t iso_14443_3a_cmd_req_wupa(uint8_t req, uint8_t *atq, uint8_t doretry)
{
    uint8_t tx_buf[2];
    int32_t tx_len;
    uint8_t *rx_buf;
    uint32_t rx_len;
    int32_t ret;
    uint8_t retry=doretry?3:1;

    tx_buf[0] = req;
    tx_len = 1;

    rx_buf = atq;

    do {
        ret = nfc_pcd_transceive(PROTOCOL_ISO14443A, FT_SHORT_NO_CRC_NO_EMD, tx_buf, tx_len, rx_buf, &rx_len, ISO14443_FWT_A_ACT);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            /*ATQA should be 2 bytes*/
            if(rx_len != ATQA_LEN)
                ret=ISO14443_3_ERR_PROTOCOL;
        }

        // EMV 2.6b case TA311, now enforces a minimum retransmission time of 3ms
        if ( (retry > 1) && ret==ISO14443_3_ERR_TIMEOUT ) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_A_ACT);
        }

    } while(--retry && ret==ISO14443_3_ERR_TIMEOUT);

    return ret;
}


int32_t iso_14443_3a_cmd_anticoll(uint8_t sel,uint8_t *uid)
{
    uint8_t tx_buf[2];
    int32_t tx_len;
    uint8_t *rx_buf;
    uint32_t rx_len=UID_EACH_LEN;
    int32_t ret;

    int32_t i;
    uint8_t bcc = 0;
    uint8_t retry=3;

    tx_buf[0] = sel;
    tx_buf[1] = 0x20;
    tx_len = 2;

    rx_buf = uid;

    do {
        ret = nfc_pcd_transceive(PROTOCOL_ISO14443A, FT_STANDARD_NO_CRC_NO_EMD, tx_buf, tx_len, rx_buf, &rx_len, ISO14443_FWT_A_ACT);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            /*each uid should be 5 bytes*/
            if(rx_len != UID_EACH_LEN) {
                ret=ISO14443_3_ERR_PROTOCOL;
                break;
            }

            //Check BCC
            for (i = 0,bcc=0; i < 4; i++) {
                bcc ^= uid[i];
            }

            if (bcc != uid[4]) {
                ret = ISO14443_3_ERR_TRANSMISSION;
            }
        }

        // EMV 2.6b case TA310, now enforces a minimum retransmission time of 3ms
        if ( (retry > 1) && ret==ISO14443_3_ERR_TIMEOUT ) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_A_ACT);
        }

    } while(--retry && ret==ISO14443_3_ERR_TIMEOUT);

    return ret;
}


int32_t iso_14443_3a_cmd_select(uint8_t sel, uint8_t *uid, uint8_t *sak)
{
    uint8_t tx_buf[10];
    int32_t tx_len;
    uint8_t *rx_buf;
    uint32_t rx_len=1;                  //SAK should be one byte.
    int32_t ret;
    uint8_t retry=3;

    tx_buf[0] = sel;
    tx_buf[1] = 0x70;
    memcpy(tx_buf + 2, uid, UID_EACH_LEN);
    tx_len = 7;

    rx_buf = sak;

    do {
        ret = nfc_pcd_transceive(PROTOCOL_ISO14443A, FT_STANDARD_CRC_NO_EMD, tx_buf, tx_len, rx_buf, &rx_len, ISO14443_FWT_A_ACT);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            if(rx_len != SAK_LEN)
                ret=ISO14443_3_ERR_PROTOCOL;

        }

        // EMV 2.6b case TA312, now enforces a minimum retransmission time of 3ms
        if ( (retry > 1) && ret==ISO14443_3_ERR_TIMEOUT ) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_A_ACT);
        }

    } while(--retry && ret==ISO14443_3_ERR_TIMEOUT);

    return ret;
}


int32_t iso_14443_3a_cmd_rats(uint8_t fsdi, uint8_t cid, uint8_t *ats,  uint32_t *ats_len)
{
    uint8_t tx_buf[2];
    int32_t tx_len;
    uint8_t *rx_buf;
    int32_t ret;
    uint8_t retry=3;

    tx_buf[0] = 0xe0;

    tx_buf[1] = (fsdi << 4) + (cid & 0x0f);

    tx_len = 2;

    rx_buf = ats;

    do {

        ret = nfc_pcd_transceive(PROTOCOL_ISO14443A, FT_STANDARD_CRC_EMD, tx_buf, tx_len, rx_buf, ats_len, ISO14443_FWT_ACTIVATION);

        if (ret == ISO14443_3_ERR_SUCCESS) {
            ret = ISO14443_3_ERR_SUCCESS;
        }

        // EMV 2.6b case TA307 and TA313, now enforces a minimum retransmission time of 3ms
        if ( (retry > 1) && ret==ISO14443_3_ERR_TIMEOUT ) {
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_ACTIVATION);
        }

        /*debug for case TA306.05*/
    } while(--retry && (ret==ISO14443_3_ERR_TIMEOUT));

    return ret;
}


int32_t iso_14443_3a_cmd_halt(void)
{
    uint8_t tx_buf[2];
    int32_t tx_len;
    uint8_t *rx_buf;
    uint32_t rx_len;
    int32_t ret;

    uint8_t tmp[100];

    tx_buf[0] = 0x50;
    tx_buf[1] = 0x00;
    tx_len = 2;

    rx_buf = tmp;

    ret = nfc_pcd_transceive(PROTOCOL_ISO14443A, FT_STANDARD_CRC_EMD, tx_buf, tx_len, rx_buf, &rx_len, ISO14443_FWT_A_ACT);

    if (ret == ISO14443_3_ERR_TIMEOUT) {
        //HALT no response.
        ret = ISO14443_3_ERR_SUCCESS;
    } else {
        ret = ISO14443_3_ERR_OTHER;
    }

    return ret;
}

