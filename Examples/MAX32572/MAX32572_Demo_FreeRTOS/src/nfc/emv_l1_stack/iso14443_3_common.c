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

#include "logging.h"
#include "mml_nfc_pcd_port.h"
#include <emv_l1_stack/iso14443_3_common.h>
#include <string.h>

uint8_t gcommonbuffer[MAX_BUFFER_LEN]; // use for read data from picc.

// The default delay between a reception from PICC and a new transmit from PCD
uint32_t g_delay_till_send = ISO14443_FDT_MIN;

static abort_check_callback_t abort_callback_function = NULL;

static ATSConfig_t ATScfg = {
    .Pro_Type = PROTOCOL_ISO14443A,
    .FSCI = FSCI_DEFAULT_VALUE,
    .FWI = FWI_DEFAULT_VALUE,
    .SFGI = SFGI_DEFAULT_VALUE,
};

void get_ats(ATSConfig_t* cfg)
{
    memcpy((uint8_t*)cfg, (uint8_t*)&ATScfg, sizeof(ATSConfig_t));
}

void set_ats(uint8_t pro_type, uint8_t fsci, uint8_t fwi, uint8_t sfgi, uint8_t nad, uint8_t cid)
{
    ATScfg.Pro_Type = pro_type;
    ATScfg.FSCI = fsci;
    ATScfg.FWI = fwi;
    ATScfg.SFGI = sfgi;

    ATScfg.NAD_support = nad;
    ATScfg.CID_support = cid;
}

void nfc_yield_ms(uint32_t yield_ms)
{
    mml_nfc_pcd_task_sleep(yield_ms);
}

void nfc_set_delay_till_next_send_fc(uint32_t delay)
{
    g_delay_till_send = delay;
}

void nfc_block_for_us(uint32_t block_us)
{
    mml_nfc_pcd_block_for_us(block_us);
}

void poweron_operatingfield(void)
{
    mml_nfc_pcd_field_on();
}

void poweroff_operatingfield(void)
{
    mml_nfc_pcd_field_off();
}

void nfc_pcd_reset_wait(void)
{
    mml_nfc_pcd_smart_yield_us(TRESET_US);
}

int32_t nfc_reset(void)
{
    // Wait at least ISO14443_FDT_MIN to avoid any violations of the EOS
    // HOWEVER, we can't use mml_nfc_pcd_delay_for_FDT_PCD(ISO14443_FDT_MIN);
    // because we don't have the receive done time stamp.  If we only base it off
    // the end of TX we will be too short.
    mml_nfc_pcd_block_for_us(ISO14443_FDT_MIN_US);

    mml_nfc_pcd_field_off();
    nfc_pcd_reset_wait();

    mml_nfc_pcd_field_on();
    nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

    return 0;
}

void hexdump(int32_t dbg_level, uint8_t* buf, int32_t len, int32_t send)
{
    int i;

    do_log(dbg_level, "[%03d]%s\n      ", len, send ? "-->" : "<--");
    for (i = 0; i < len; i++) {
        do_log(dbg_level, "%02X ", buf[i]);
        if (((i + 1) % 16 == 0) && (i < (len - 1)))
            do_log(dbg_level, "\n      ");
    }
    do_log(dbg_level, "\n");
}

static int32_t check_for_requested_abort(int32_t status)
{
    // If the callback exists, call it
    if (abort_callback_function) {
        if (abort_callback_function()) {
            info("Transaction Aborted\n");
            return ISO14443_3_ERR_ABORTED;
        }
    }

    return status;
}

void set_abort_check_callback(abort_check_callback_t abort_callback)
{
    abort_callback_function = abort_callback;
}

static int32_t convert_status_codes(int32_t rf_status)
{
    switch (rf_status) {
    case MML_NFC_PCD_E_SUCCESS:
        return ISO14443_3_ERR_SUCCESS;

    case MML_NFC_PCD_E_TIMEOUT:
    case MML_NFC_PCD_E_TX_TIMEOUT:
    case MML_NFC_PCD_E_TO_EMD:
    case MML_NFC_PCD_E_TO_PART_RX:
    case MML_NFC_PCD_E_HARD_TO:
        return ISO14443_3_ERR_TIMEOUT;

    case MML_NFC_PCD_E_PARITY:
    case MML_NFC_PCD_E_INVALID_CRC:
    case MML_NFC_PCD_E_PROTOCOL:
    case MML_NFC_PCD_E_RX_EGT:
    case MML_NFC_PCD_E_A_INV_BIT_LEN:
    case MML_NFC_PCD_E_B_NOT_ENF_SAMP:
    case MML_NFC_PCD_E_B_TR1_INVALID:
    case MML_NFC_PCD_E_B_SOS_0S_INVALID:
    case MML_NFC_PCD_E_B_SOS_1S_INVALID:
    case MML_NFC_PCD_E_B_START_BIT:
    case MML_NFC_PCD_E_B_STOP_BIT:
    case MML_NFC_PCD_E_B_INCOMPLETE_BYTE:
    case MML_NFC_PCD_E_B_EGT_EXCEEDED:
    case MML_NFC_PCD_E_F_INV_BIT_LEN:
    case MML_NFC_PCD_E_NO_RX_DATA:
    case MML_NFC_PCD_E_B_NO_SOS:
    case MML_NFC_PCD_E_B_BAD_SOS:
    case MML_NFC_PCD_E_B_SUB_DROP_DATA:
    case MML_NFC_PCD_E_B_SUB_DROP_SOS:
    case MML_NFC_PCD_E_B_ILL_PHZ_CHANGE:
    case MML_NFC_PCD_E_B_NON_01_PATTERN:
    case MML_NFC_PCD_E_A_BAD_SOF:
    case MML_NFC_PCD_E_B_BAD_EOF:
    case MML_NFC_PCD_E_NOT_ENF_BITS:
        return ISO14443_3_ERR_TRANSMISSION;

    case MML_NFC_PCD_E_COLLISION:
    case MML_NFC_PCD_E_A_CONT_MOD_COL:
    case MML_NFC_PCD_E_F_SOF_COL:
    case MML_NFC_PCD_E_F_PART_SYNC_COL:
        return ISO14443_3_ERR_COLLISION;

    case MML_NFC_PCD_E_EARLY_RESPONSE:
        return ISO14443_3_ERR_EARLY_RESPONSE;

    default:
        return ISO14443_3_ERR_OTHER;
    }
}

static uint32_t pad_for_crystal_margin(uint32_t counts_to_pad)
{
    uint32_t count_margin = 0;

    // Here we only pad the fc count, we depend on the RF driver
    // enforce this using what ever time mechanism it has, which
    // already accounts for tick length conversion between fc and us.

    // Per ARM Data book, unsigned divide instruction takes 2-12 cycles.
    // This is probably fast enough for our purposes.
    if (counts_to_pad > CRYSTAL_PPM_MARGIN_DIVISOR) {
        count_margin = counts_to_pad / CRYSTAL_PPM_MARGIN_DIVISOR;
    }

    return counts_to_pad + (count_margin * CRYSTAL_PPM_MARGIN_MULTIPLIER);
}

int32_t nfc_pcd_transceive(uint8_t protocol, uint8_t frametype, uint8_t* tx_buf, uint32_t tx_len,
    uint8_t* rx_buf, uint32_t* rx_len, uint32_t timeout)

{
    int32_t status = MML_NFC_PCD_E_SUCCESS;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = protocol;
    trans_params.frametype = frametype;
    trans_params.tx_buf = tx_buf;
    trans_params.tx_len = tx_len;
    trans_params.rx_buf = rx_buf;
    trans_params.rx_len = rx_len;
    trans_params.timeout = timeout;

    if ((trans_params.timeout != ISO14443_FWT_A_ACT)
        && (trans_params.timeout != ISO14443_FWT_ATQB)) {
        debug("RF_TO: %d\n", trans_params.timeout);
        debug("RF_FT: %d\n", frametype);
    }

    // New for CSP3.3.0 RF Driver requires us to specify the early limit time
    switch (protocol) {
    case PROTOCOL_ISO14443A:
        trans_params.early_limit = ISO14443_FDT_A_EARLY_LIMIT;
        if (trans_params.early_limit > ISO14443_FDT_A_EXTRA_MARGIN) {
            trans_params.early_limit -= ISO14443_FDT_A_EXTRA_MARGIN;
        }
        trans_params.timeout += ISO14443_FDT_A_EXTRA_MARGIN;
        break;

    case PROTOCOL_ISO14443B:
        trans_params.early_limit = ISO14443_FDT_B_PICC_MIN;
        if (trans_params.early_limit > ISO14443_FDT_B_EXTRA_MARGIN) {
            trans_params.early_limit -= ISO14443_FDT_B_EXTRA_MARGIN;
        }
        trans_params.timeout += ISO14443_FDT_B_EXTRA_MARGIN;
        break;

    default:
        return ISO14443_3_ERR_OTHER;
    }

    // New in CSP 3.3.0
    // Need to provide extra margin for potential crystal oscillator PPM drift
    // Due to temperature, improper loading, and normal crystal variances, the accuracy
    // of the system clock can be off by 100 ppm or even more.
    //
    // This is normally not an issue, as for most times we are so small that the 100ppm
    // only amounts to a small number of nanoseconds.  However, some of the negotiated delays
    // and wait times can be several million counts.  Therefore, it is safer to pad with
    // extra margin on these times as they grow large.
    trans_params.delay_till_send = pad_for_crystal_margin(g_delay_till_send);
    trans_params.timeout = pad_for_crystal_margin(trans_params.timeout);

    // FDT PCD min delay is 6780fc this works out to 500us
    // The RF driver now handles these inter-packet delays via the delay_till_send parameter

    status = mml_nfc_pcd_transceive(trans_params);

    // Reset the delay_till_send for next transaction, unless overridden before
    // This simplifies inter packet delay handling, as usually, we only need to
    // wait for this minimum time.  Longer delay are special cases.
    g_delay_till_send = ISO14443_FDT_MIN;

    if ((status != MML_NFC_PCD_E_SUCCESS) && (status != MML_NFC_PCD_E_TIMEOUT)) {
        debug("RF_ST: %d, 0x%X\n", status, status);
    }

    status = convert_status_codes(status);

    // Check if application wants to abort this transaction
    return check_for_requested_abort(status);
}
