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

#include <emv_l1_stack/iso14443_3_common.h>
#include <emv_l1_stack/iso14443_3a_cmd.h>
#include <emv_l1_stack/iso14443_3a_flow.h>
#include <emv_l1_stack/iso14443_4_transitive.h>
#include <string.h>

#include <mml_nfc_pcd_rf_driver.h>
#include "logging.h"

#define UID_LEVEL_1 1
#define UID_LEVEL_2 2
#define UID_LEVEL_3 3

#define TL_MAX_VALUE (20)
#define TL_MIN_VALUE (1)

#define TA_RESERVE_MAKE (0x08)

#define TA_DEFAULT_VALUE (0x00)

#define ISO14443_3A_ANTICOLLISION_CT_VALUE 0x88
#define TYPE_A_SAK_14443_4_SUPPORT 0x20

#define ATQA_LEN 2

typedef struct {
    uint8_t fsci : 4;
    uint8_t has_ta : 1;
    uint8_t has_tb : 1;
    uint8_t has_tc : 1;
    uint8_t reserve : 1;
} T0_t;

typedef struct {
    uint8_t sfgi : 4;
    uint8_t fwi : 4;
} TB_t;

typedef struct {
    uint8_t TL;
    T0_t T0;
    uint8_t TA;
    TB_t TB;
    uint8_t TC;
    uint8_t *PTk;
} ATS_t;

/**
 * Last received SAK
 * - Can be used to decipher MIFARE card type
 */
static uint8_t sak;

uint8_t get_last_sak()
{
    return sak;
}

/*
   new version the PCD shall not examine the ATQ RFU bits,only check b5~b8 of byte2
 */
static int32_t check_atq(uint8_t *atq, uint32_t *uid_level)
{
    if ((atq[0] & 0xC0) == 0x00) {
        *uid_level = UID_LEVEL_1; // single size UID
    } else if ((atq[0] & 0xC0) == 0x40) {
        *uid_level = UID_LEVEL_2; // double size UID
    } else if ((atq[0] & 0xC0) == 0x80) {
        *uid_level = UID_LEVEL_3; // triple size UID
    } else {
        *uid_level = UID_LEVEL_1; // default to use single size UID
    }

    /*for Bit1~5 RFU*/
    if ((atq[0] & 0x1F) != 0x01 && (atq[0] & 0x1F) != 0x02 && (atq[0] & 0x1F) != 0x04 &&
        (atq[0] & 0x1F) != 0x08 && (atq[0] & 0x1F) != 0x10) {
        *uid_level = UID_LEVEL_1;
    }

    /*b5~b8 should be 0.*/
    if (atq[1] & 0xF0)
        return ISO14443_3_ERR_PROTOCOL;

    return ISO14443_3_ERR_SUCCESS;
}

int32_t iso_14443_3a_polling()
{
    int32_t void_len = 0;

    return iso_14443_3a_polling_response(NULL, &void_len);
}

int32_t iso_14443_3a_polling_response(uint8_t *atqa_resp, int32_t *atqa_resp_len)
{
    int32_t ret;
    uint8_t *atq = GetCommonBuffer();

    ret = iso_14443_3a_cmd_req_wupa(ISO_14443_3A_CMD_WUPA, atq, WAKEUP_NOTRETRY);

    if (ret == ISO14443_3_ERR_ABORTED) {
        return ret;
    }

    if (ret != ISO14443_3_ERR_TIMEOUT) {
        iso_14443_3a_cmd_halt();

        // If atqa_resp exits, then save off the ATQA
        if (atqa_resp) {
            memcpy(atqa_resp, atq, ATQA_LEN);
            *atqa_resp_len = ATQA_LEN;
        }

        return ISO14443_3_ERR_SUCCESS;
    }

    return ISO14443_3_ERR_TIMEOUT;
}

int32_t iso_14443_3a_collision_detect()
{
    int32_t void_len = 0;

    return iso_14443_3a_collision_detect_response(NULL, &void_len, NULL, &void_len, NULL);
}

int32_t iso_14443_3a_collision_detect_response(uint8_t *atqa_resp, int32_t *atqa_resp_len,
                                               uint8_t *uid_resp, int32_t *uid_resp_len,
                                               uint8_t *sak_resp)
{
    uint8_t *atq = GetCommonBuffer();
    uint8_t *uid = GetCommonBuffer();
    uint32_t uid_level;
    int32_t status = ISO14443_3_ERR_OTHER;

    status = iso_14443_3a_cmd_req_wupa(ISO_14443_3A_CMD_WUPA, atq, WAKEUP_DORETRY);

    if (status != ISO14443_3_ERR_SUCCESS)
        return status;

    // If atqa_resp exits, then save off the ATQA
    if (atqa_resp) {
        memcpy(atqa_resp, atq, ATQA_LEN);
        *atqa_resp_len = ATQA_LEN;
    }

    if (check_atq(atq, &uid_level) != ISO14443_3_ERR_SUCCESS) {
        return ISO14443_3_ERR_CMD;
    }
    info("uid_level is = %d, atq = %02x%02x\n", uid_level, atq[0], atq[1]);

    /*UID CL1*/
    status = iso_14443_3a_cmd_anticoll(ISO_14443_3A_CMD_ANTICOLL_SEL_L1, uid);

    if (status == ISO14443_3_ERR_ABORTED) {
        return status;
    }

    if (status != ISO14443_3_ERR_SUCCESS) {
        error("Failed anticoll l1\n");
        return ISO14443_3_ERR_CMD;
    }
    info("uid = [%02x%02x%02x%02x%02x]\n", uid[0], uid[1], uid[2], uid[3], uid[4]);

    /*UID CL1 uid0 should not be CT value when 1-level uid*/
    if ((uid_level == UID_LEVEL_1) && uid[0] == ISO14443_3A_ANTICOLLISION_CT_VALUE)
        return ISO14443_3_ERR_PROTOCOL;

    if (uid_level >= UID_LEVEL_1) {
        // If anticol_resp exists, fill it in
        if (uid_resp) {
            memcpy(uid_resp, uid, UID_EACH_LEN);
            *uid_resp_len = UID_EACH_LEN;
        }

        status = iso_14443_3a_cmd_select(ISO_14443_3A_CMD_SELECT_SEL_L1, uid, &sak);

        if (status == ISO14443_3_ERR_ABORTED) {
            return status;
        }

        if (status != 0) {
            error("Failed select l1\n");
            return ISO14443_3_ERR_CMD;
        }

        // If sak_resp exists, fill it in
        if (sak_resp) {
            *sak_resp = sak;
        }
    }

    if (uid_level >= UID_LEVEL_2) {
        status = iso_14443_3a_cmd_anticoll(ISO_14443_3A_CMD_ANTICOLL_SEL_L2, uid + UID_EACH_LEN);

        if (status == ISO14443_3_ERR_ABORTED) {
            return status;
        }

        if (status != ISO14443_3_ERR_SUCCESS) {
            error("Failed anticoll l2\n");
            return ISO14443_3_ERR_CMD;
        }

        /*UID CL2 uid3 should not be CT value when 2-level uid*/
        if ((uid_level == UID_LEVEL_2) && uid[UID_EACH_LEN] == ISO14443_3A_ANTICOLLISION_CT_VALUE)
            return ISO14443_3_ERR_PROTOCOL;

        // If uid_resp exists, fill it in
        if (uid_resp) {
            memcpy(uid_resp, uid, UID_EACH_LEN * 2);
            *uid_resp_len = UID_EACH_LEN * 2;
        }

        status = iso_14443_3a_cmd_select(ISO_14443_3A_CMD_SELECT_SEL_L2, uid + UID_EACH_LEN, &sak);

        if (status == ISO14443_3_ERR_ABORTED) {
            return status;
        }

        if (status != ISO14443_3_ERR_SUCCESS) {
            error("Failed select l2\n");
            return ISO14443_3_ERR_CMD;
        }

        // If sak_resp exists, fill it in
        if (sak_resp) {
            *sak_resp = sak;
        }
    }

    if (uid_level == UID_LEVEL_3) {
        status =
            iso_14443_3a_cmd_anticoll(ISO_14443_3A_CMD_ANTICOLL_SEL_L3, uid + (2 * UID_EACH_LEN));

        if (status == ISO14443_3_ERR_ABORTED) {
            return status;
        }

        if (status != ISO14443_3_ERR_SUCCESS) {
            error("Failed anticoll l3\n");
            return ISO14443_3_ERR_CMD;
        }

        // If uid_resp exists, fill it in
        if (uid_resp) {
            memcpy(uid_resp, uid, UID_EACH_LEN * 3);
            *uid_resp_len = UID_EACH_LEN * 3;
        }

        status =
            iso_14443_3a_cmd_select(ISO_14443_3A_CMD_SELECT_SEL_L3, uid + (2 * UID_EACH_LEN), &sak);

        if (status == ISO14443_3_ERR_ABORTED) {
            return status;
        }

        if (status != ISO14443_3_ERR_SUCCESS) {
            error("Failed select l3\n");
            return ISO14443_3_ERR_CMD;
        }

        // If sak_resp exists, fill it in
        if (sak_resp) {
            *sak_resp = sak;
        }
    }

    // Check to see if this card supports ISO_14443_4
    if ((sak & TYPE_A_SAK_14443_4_SUPPORT) != TYPE_A_SAK_14443_4_SUPPORT) {
        return ISO14443_3_ERR_NON_ISO14443_4_CARD;
    }

    return ISO14443_3_ERR_SUCCESS;
}

int32_t iso_14443_3a_active()
{
    int32_t void_len = 0;

    return iso_14443_3a_active_response(NULL, &void_len);
}

int32_t iso_14443_3a_active_response(uint8_t *ats_resp, int32_t *ats_resp_len)
{
    int32_t status = ISO14443_3_ERR_OTHER;
    uint8_t fsci = FSCI_DEFAULT_VALUE;
    uint8_t cid = 0, nad = 0;
    uint8_t *ats = GetCommonBuffer();
    uint32_t ats_len;
    uint8_t fwi = FWI_DEFAULT_VALUE, sfgi = SFGI_DEFAULT_VALUE;
    uint32_t sfgi_fc = 0;
    ATS_t *pats = (ATS_t *)ats;

    status = iso_14443_3a_cmd_rats(FSDI_DEFAULT_VALUE, cid, ats, &ats_len);

    if (status == ISO14443_3_ERR_ABORTED) {
        return status;
    }

    if (status != ISO14443_3_ERR_SUCCESS) {
        return ISO14443_3_ERR_CMD;
    }

    // If ats_resp exists, fill it in
    if (ats_resp) {
        memcpy(ats_resp, ats, ats_len);
        *ats_resp_len = ats_len;
    }

    //check TL and ats len
    if (!ats_len || pats->TL != ats_len)
        return ISO14443_3_ERR_PROTOCOL;

    if (pats->TL > TL_MAX_VALUE)
        return ISO14443_3_ERR_CMD;

    //for debug ta104.14/15
    if (pats->TL == 0x13 || pats->TL == 0x14) {
        info("Case TA102.14/15.\n");
    }

    if (pats->TL > TL_MIN_VALUE) {
        fsci = pats->T0.fsci <= FSCI_MAX_VALUE ? pats->T0.fsci : FSCI_MAX_VALUE;

        if (pats->T0.has_ta && pats->TA != TA_DEFAULT_VALUE && !(pats->TA & TA_RESERVE_MAKE)) {
            warning("warn TA %x\n", pats->TA);
            //return ISO14443_3_ERR_CMD;
        }

        if (pats->T0.has_tb) {
            fwi = pats->TB.fwi <= FWI_MAX_VALUE ? pats->TB.fwi : FWI_DEFAULT_VALUE;
            sfgi = pats->TB.sfgi <= SFGI_MAX_VALUE ? pats->TB.sfgi : SFGI_DEFAULT_VALUE;
        }

        //for emv we just ignore the tc value.

    } else if (!pats->TL) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    info("fsci %d\n", fsci);
    set_ats(PROTOCOL_ISO14443A, fsci, fwi, sfgi, nad, cid);
    seqnuminit();

    //SFGI delay = 256x16x2^sfgi + 384x2^sfgi   /fc
    if (sfgi) {
        sfgi_fc = 4480 * (1 << sfgi);

        nfc_set_delay_till_next_send_fc(sfgi_fc);

        info("sfgi=%d, time(fc)=%d \n", sfgi, sfgi_fc);
    }

    return ISO14443_3_ERR_SUCCESS;
}

int32_t iso_14443_3a_remove()
{
    int32_t loop = 0;
    int32_t ret;
    uint8_t *atq = GetCommonBuffer();

    nfc_reset();

    while (loop < 3) {
        ret = iso_14443_3a_cmd_req_wupa(ISO_14443_3A_CMD_WUPA, atq, WAKEUP_NOTRETRY);

        if (ret == ISO14443_3_ERR_ABORTED) {
            return ret;
        }

        if (ret != ISO14443_3_ERR_TIMEOUT) {
            iso_14443_3a_cmd_halt();
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            loop = 0;
        } else {
            loop++;

            // EMV 2.6b case TA311, now enforces a minimum retransmission time of 3ms
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_A_ACT);
        }
    }

    return ISO14443_3_ERR_SUCCESS;
}
