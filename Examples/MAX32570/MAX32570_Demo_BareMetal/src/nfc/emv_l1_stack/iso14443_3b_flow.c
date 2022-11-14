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

#include <iso14443_3b_cmd.h>
#include <iso14443_3b_flow.h>
#include <iso14443_4_transitive.h>
#include <string.h>

#include <mml_nfc_pcd_rf_driver.h>
#include "logging.h"

typedef struct {
    uint8_t bit_rate;

    uint8_t pro_type : 4;
    uint8_t fsci : 4;

    uint8_t fo : 2;
    uint8_t adc : 2;
    uint8_t fwi : 4;

    uint8_t rfu : 4;
    uint8_t sfgi : 4;
} PROINFO_t;

typedef struct {
    uint8_t atqb_f;
    uint8_t pupi[PUPI_SIZE];
    uint8_t appdata[4];
    PROINFO_t proinfo;
} ATQB_t;

static ATQB_t GAtqb;

int32_t iso_14443_3b_polling(void)
{
    int32_t void_len = 0;

    return iso_14443_3b_polling_response(NULL, &void_len);
}

int32_t iso_14443_3b_polling_response(uint8_t *atqb_resp, int32_t *atqb_resp_len)
{
    uint8_t *atq = GetCommonBuffer();
    int32_t atq_len;
    int32_t ret;

    ret = iso_14443_3b_cmd_req_wup(atq, &atq_len, WAKEUP_NOTRETRY);

    if (ret == ISO14443_3_ERR_ABORTED) {
        return ret;
    }

    if (ret == ISO14443_3_ERR_TIMEOUT) {
        //Timeout
        return ISO14443_3_ERR_OTHER;
    }

    // If atqb_resp exists, copy in the ATQB
    if (atqb_resp) {
        memcpy(atqb_resp, atq, atq_len);
        *atqb_resp_len = atq_len;
    }

    return 0;
}

static void set_atqb(ATQB_t *patqb)
{
    memcpy(&GAtqb, patqb, sizeof(ATQB_t));
}

static void get_atqb(ATQB_t *patqb)
{
    memcpy(patqb, &GAtqb, sizeof(ATQB_t));
}

int32_t iso_14443_3b_collision_detect(void)
{
    int32_t void_len = 0;

    return iso_14443_3b_collision_detect_response(NULL, &void_len);
}

int32_t iso_14443_3b_collision_detect_response(uint8_t *atqb_resp, int32_t *atqb_resp_len)
{
    uint8_t *atqb = GetCommonBuffer();
    int32_t atqb_len;

    uint8_t fsci = FSCI_DEFAULT_VALUE;
    uint8_t fwi = FWI_DEFAULT_VALUE;
    uint8_t sfgi = SFGI_DEFAULT_VALUE;
    uint8_t nad = 0, cid = 0; /*we don't support nad&cid in default*/

    ATQB_t *patqb = (ATQB_t *)atqb;
    int32_t ret;

    ret = iso_14443_3b_cmd_req_wup(atqb, &atqb_len, WAKEUP_DORETRY);

    if (ret != ISO14443_3_ERR_SUCCESS)
        return ret;

    // If atqb_resp exists, copy in the ATQB
    if (atqb_resp) {
        memcpy(atqb_resp, atqb, atqb_len);
        *atqb_resp_len = atqb_len;
    }

    fsci = patqb->proinfo.fsci <= FSCI_MAX_VALUE ? patqb->proinfo.fsci : FSCI_MAX_VALUE;
    fwi = patqb->proinfo.fwi <= FWI_MAX_VALUE ? patqb->proinfo.fwi : FWI_DEFAULT_VALUE;

    if (atqb_len == ISO3B_ATQB_MAXLEN)
        sfgi = patqb->proinfo.sfgi <= SFGI_MAX_VALUE ? patqb->proinfo.sfgi : SFGI_DEFAULT_VALUE;

    /**check protocol type  B4 must be 0*/
    if (patqb->proinfo.pro_type & 0x08)
        return ISO14443_3_ERR_PROTOCOL;

    /*FO,CID NAD support*/
    if (patqb->proinfo.fo & 0x01)
        cid = 1;
    if (patqb->proinfo.fo & 0x02)
        nad = 1;

    set_atqb(patqb);
    set_ats(PROTOCOL_ISO14443B, fsci, fwi, sfgi, nad, cid);

    return ISO14443_3_ERR_SUCCESS;
}

int32_t iso_14443_3b_active(void)
{
    int32_t void_len = 0;

    return iso_14443_3b_active_response(NULL, &void_len);
}

int32_t iso_14443_3b_active_response(uint8_t *attrib_resp, int32_t *attrib_resp_len)
{
    int32_t ret;
    uint32_t sfgi_fc = 0;
    ATQB_t atqb;
    ATSConfig_t ats;

    get_atqb(&atqb);
    get_ats(&ats);

    /* 6.3.2.10 PCD must disregard the value of bits b4-b2 of Protocol_Type */
    ret = iso_14443_3b_cmd_attrib(atqb.pupi, 0x00, FSDI_DEFAULT_VALUE,
                                  (atqb.proinfo.pro_type & PROTOCOL_DISREGARD_BITS), 0x00, NULL,
                                  NULL, (4096 * (1 << ats.FWI) + ISO14443_FWT_DELTA), attrib_resp,
                                  attrib_resp_len);

    if (ret == ISO14443_3_ERR_ABORTED) {
        return ret;
    }

    if ((ret != ISO14443_3_ERR_SUCCESS) && (ret != ISO14443_3_ERR_CONTAINS_HIGH_INF)) {
        return ISO14443_3_ERR_CMD;
    }

    seqnuminit();

    //SFGI delay = 256x16x2^sfgi + 384x2^sfgi
    if (ats.SFGI) {
        sfgi_fc = 4480 * (1 << ats.SFGI);

        nfc_set_delay_till_next_send_fc(sfgi_fc);

        info("sfgi=%d, time(fc)=%d \n", ats.SFGI, sfgi_fc);
    }
    return ret;
}

int32_t iso_14443_3b_remove(void)
{
    int32_t loop = 0;
    int32_t ret;
    uint8_t *atq = GetCommonBuffer();
    int32_t atq_len;

    nfc_reset();

    while (loop < 3) {
        ret = iso_14443_3b_cmd_req_wup(atq, &atq_len, WAKEUP_NOTRETRY);

        if (ret == ISO14443_3_ERR_ABORTED) {
            return ret;
        }

        if (ret != ISO14443_3_ERR_TIMEOUT) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

            loop = 0;
        } else {
            loop++;

            // EMV 2.6b case TB311, now enforces a minimum retransmission time of 3ms
            nfc_set_delay_till_next_send_fc(TMIN_RETRANSMISSION_FC + ISO14443_FWT_ATQB);
        }
    }

    return ISO14443_3_ERR_SUCCESS;
}
