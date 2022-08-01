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

/* Global includes */
#include <stdio.h>
#include <string.h>

#include "MAX32xxx.h"
#include "task_nfc.h"
#include "message.h"

#include "mml_nfc_pcd_port.h"
#include "mml_nfc_pcd_rf_driver.h"
#include "iso14443_3a_flow.h"
#include "iso14443_3a_cmd.h"
#include "iso14443_3b_flow.h"
#include "iso14443_3b_cmd.h"
#include "iso14443_3_common.h"
#include "iso14443_4_transitive.h"
#include "logging.h"
#include "EMV_polling_and_loopback.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

/******************************   		DEFINES	    **************************/
#define BEEPER_PORT       MXC_GPIO3
#define BEEPER_PIN        MXC_GPIO_PIN_3
#define BEEP_PASS_TIME_MS 150
#define BEEP_FAIL_TIME_MS 150

#define BEEP_PASS_TONE 250 //847
#define BEEP_PASS_VOL  25  //84

/******************************   	TYPE DEFINES	**************************/
typedef struct {
    uint8_t rapdu[261];
    int32_t rapdu_len;
    uint8_t aid_bin[50];
    int32_t aid_bin_len;
    char application_label[50];
    int32_t application_label_len;
} ppse_response_t;

/********************************* 		VARIABLES	 *************************/
extern mml_nfc_pcd_analog_params_t current_analog_parameters;

extern xQueueHandle xQueueMain;
static xSemaphoreHandle xNFCLock;
static volatile int g_nfc_active_polling = 0;

/******************************   STATIC FUNCTIONS  **************************/
static void beep_for_success(void)
{
    uint32_t beep_time     = 0;
    uint32_t beep_loop_cnt = 0;

    uint32_t tone        = BEEP_PASS_TONE;
    uint32_t vol         = BEEP_PASS_VOL;
    uint32_t duration_ms = BEEP_PASS_TIME_MS;

    // No timer or pulse train on the buzzer gpio, got to bit bang it
    // 4Khz is maximum tone volume, as the buzzer resonates here
    // Tone set the period, in us

    // Volumes sets the duty cycle us
    beep_time = (duration_ms * 1000) / tone;

    // Correct tone for the requested volume
    tone = tone - vol;

    MXC_GPIO_OutClr(BEEPER_PORT, BEEPER_PIN);

    while (beep_loop_cnt++ < beep_time) {
        MXC_GPIO_OutSet(BEEPER_PORT, BEEPER_PIN);
        mml_nfc_pcd_block_for_us(vol);
        MXC_GPIO_OutClr(BEEPER_PORT, BEEPER_PIN);
        mml_nfc_pcd_block_for_us(tone);
    }

    MXC_GPIO_OutClr(BEEPER_PORT, BEEPER_PIN);
}

static void setup_beeper(void)
{
    mxc_gpio_cfg_t buzzer_out;

    buzzer_out.port = BEEPER_PORT;
    buzzer_out.mask = BEEPER_PIN;
    buzzer_out.pad  = MXC_GPIO_PAD_NONE;
    buzzer_out.func = MXC_GPIO_FUNC_OUT;
    // Use 3.3V for louder Buzz
    buzzer_out.vssel = MXC_GPIO_VSSEL_VDDIOH;

    MXC_GPIO_Config(&buzzer_out);
    MXC_GPIO_OutClr(BEEPER_PORT, BEEPER_PIN);
}

static int32_t do_ppse(ppse_response_t* resp)
{
    int32_t ret;
    uint8_t capdu[261] = {0x00, 0xA4, 0x04, 0x00, 0x0E, '2', 'P', 'A', 'Y', '.',
                          'S',  'Y',  'S',  '.',  'D',  'D', 'F', '0', '1', 0x00};
    int32_t capdulen   = 20;

    logging("CAPDU ");
    hexdump(DBG_LVL_LOG, capdu, capdulen, 1);

    ret = SendAPDU(capdu, capdulen, resp->rapdu, &resp->rapdu_len);

    if (ret) {
        switch (ret) {
            case ISO14443_3_ERR_PROTOCOL:
            case ISO14443_3_ERR_TIMEOUT:
            case ISO14443_3_ERR_TRANSMISSION:
                return RESETPROCEDURE;
        }
    }

    if (resp->rapdu_len <= 2) {
        // 2 bytes should be wrong case.
        warning("Short APDU: %d\n", resp->rapdu_len);
        return RESETPROCEDURE;
    }

    logging("RAPDU ");
    hexdump(DBG_LVL_LOG, resp->rapdu, resp->rapdu_len, 0);

    return ret;
}

// Decode AIDs and figure out card vendor
static int32_t aid_lookup(ppse_response_t* resp)
{
    // VISA
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x03}, 5) == 0) {
        strncpy(resp->application_label, "VISA", 50);
        resp->application_label_len = 5;
        return ISO14443_3_ERR_SUCCESS;
    }
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "VISA", 50);
        resp->application_label_len = 5;
        return ISO14443_3_ERR_SUCCESS;
    }
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x98, 0x08, 0x48}, 7) == 0) {
        strncpy(resp->application_label, "VISA", 50);
        resp->application_label_len = 5;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Master card
    if (memcmp(resp->aid_bin, (uint8_t[]){0xA0, 0x00, 0x00, 0x00, 0x04}, 5) == 0) {
        strncpy(resp->application_label, "Master Card", 50);
        resp->application_label_len = 11;
        return ISO14443_3_ERR_SUCCESS;
    }
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x05}, 5) == 0) {
        strncpy(resp->application_label, "Master Card", 50);
        resp->application_label_len = 11;
        return ISO14443_3_ERR_SUCCESS;
    }

    // American express
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x25}, 5) == 0) {
        strncpy(resp->application_label, "American Express", 50);
        resp->application_label_len = 16;
        return ISO14443_3_ERR_SUCCESS;
    }

    // CB
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x42}, 5) == 0) {
        strncpy(resp->application_label, "CB", 50);
        resp->application_label_len = 3;
        return ISO14443_3_ERR_SUCCESS;
    }

    // LINK
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x29}, 5) == 0) {
        strncpy(resp->application_label, "LINK", 50);
        resp->application_label_len = 5;
        return ISO14443_3_ERR_SUCCESS;
    }

    // JCB
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x65}, 5) == 0) {
        strncpy(resp->application_label, "JCB", 50);
        resp->application_label_len = 4;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Dankort
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x01, 0x21, 0x10, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "Dankort", 50);
        resp->application_label_len = 8;
        return ISO14443_3_ERR_SUCCESS;
    }

    // CoGeBan
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x01, 0x41, 0x00, 0x01}, 7) == 0) {
        strncpy(resp->application_label, "Banrisul", 50);
        resp->application_label_len = 9;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Discover
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x01, 0x52, 0x30, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "Discover", 50);
        resp->application_label_len = 9;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Banrisul
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x01, 0x54}, 5) == 0) {
        strncpy(resp->application_label, "Banrisul", 50);
        resp->application_label_len = 9;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Saudi Payments Network
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x02, 0x28}, 5) == 0) {
        strncpy(resp->application_label, "Saudi Payments Network", 50);
        resp->application_label_len = 22;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Interac
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x02, 0x77}, 5) == 0) {
        strncpy(resp->application_label, "Interac", 50);
        resp->application_label_len = 8;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Discover Card
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x03, 0x24}, 5) == 0) {
        strncpy(resp->application_label, "Discover Card", 50);
        resp->application_label_len = 14;
        return ISO14443_3_ERR_SUCCESS;
    }

    // UnionPay
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x03, 0x33}, 5) == 0) {
        strncpy(resp->application_label, "UnionPay", 50);
        resp->application_label_len = 9;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Euro Alliance of Payment Schemes
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x03, 0x59}, 5) == 0) {
        strncpy(resp->application_label, "Euro Alliance", 50);
        resp->application_label_len = 13;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Verve
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x03, 0x71}, 5) == 0) {
        strncpy(resp->application_label, "Verve", 50);
        resp->application_label_len = 6;
        return ISO14443_3_ERR_SUCCESS;
    }

    // The Exchange Network ATM Network
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x04, 0x39}, 5) == 0) {
        strncpy(resp->application_label, "Exchange Network ATM", 50);
        resp->application_label_len = 20;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Rupay
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x05, 0x24, 0x10, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "Rupay", 50);
        resp->application_label_len = 6;
        return ISO14443_3_ERR_SUCCESS;
    }

    // ???100
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x04, 0x32, 0x00, 0x01}, 7) == 0) {
        strncpy(resp->application_label, "???100", 50);
        resp->application_label_len = 7;
        return ISO14443_3_ERR_SUCCESS;
    }

    // ZKA
    if (memcmp(resp->aid_bin, (char[]){0xD2, 0x76, 0x00, 0x00, 0x25, 0x45, 0x50, 0x01, 0x00}, 9) ==
        0) {
        strncpy(resp->application_label, "ZKA", 50);
        resp->application_label_len = 4;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Bankaxept
    if (memcmp(resp->aid_bin, (char[]){0xD5, 0x78, 0x00, 0x00, 0x02, 0x10, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "Bankaxept", 50);
        resp->application_label_len = 10;
        return ISO14443_3_ERR_SUCCESS;
    }

    // BRADESCO
    if (memcmp(resp->aid_bin, (char[]){0xF0, 0x00, 0x00, 0x00, 0x03, 0x00, 0x01}, 7) == 0) {
        strncpy(resp->application_label, "BRADESCO", 50);
        resp->application_label_len = 9;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Midland
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x00, 0x24, 0x01}, 6) == 0) {
        strncpy(resp->application_label, "Midland", 50);
        resp->application_label_len = 8;
        return ISO14443_3_ERR_SUCCESS;
    }

    // PBS
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x01, 0x21, 0x10, 0x10}, 7) == 0) {
        strncpy(resp->application_label, "PBS", 50);
        resp->application_label_len = 4;
        return ISO14443_3_ERR_SUCCESS;
    }

    // eTranzact
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x04, 0x54}, 5) == 0) {
        strncpy(resp->application_label, "eTranzact", 50);
        resp->application_label_len = 10;
        return ISO14443_3_ERR_SUCCESS;
    }

    // Google
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x04, 0x76, 0x6C}, 6) == 0) {
        strncpy(resp->application_label, "Google", 50);
        resp->application_label_len = 7;
        return ISO14443_3_ERR_SUCCESS;
    }

    // InterSwitch
    if (memcmp(resp->aid_bin, (char[]){0xA0, 0x00, 0x00, 0x03, 0x71, 0x00, 0x01}, 7) == 0) {
        strncpy(resp->application_label, "InterSwitch", 50);
        resp->application_label_len = 12;
        return ISO14443_3_ERR_SUCCESS;
    }

    return -1;
}

#define FCI_TEMPLATE                0x6F
#define DEDICATED_FILE_TEMPLATE     0x84
#define FCI_PROPRIETARY_TEMPLATE    0xA5
#define FCI_ISSUER_DISCRETIONARY_B0 0xBF
#define FCI_ISSUER_DISCRETIONARY_B1 0x0C
#define APPLICATION_TEMPLATE        0x61
#define APPLICATION_IDENTIFIER      0x4F
#define APPLICATION_LABEL           0x50

// Inspect ppse response, if valid, lookup Application ID
static int32_t parse_ppse_response(ppse_response_t* resp)
{
    int32_t index        = 0;
    int32_t i            = 0;
    int32_t fci_prop_len = 0;

    if (resp->rapdu_len < 2) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Check for SW success 0x9000
    if ((resp->rapdu[resp->rapdu_len - 1] != 0x00) || (resp->rapdu[resp->rapdu_len - 2] != 0x90)) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // First comes File Control Information (FCI) Template
    if (resp->rapdu[index++] != FCI_TEMPLATE) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // This is followed by the length of the FCI
    // Verify we have good length
    if (resp->rapdu[index++] >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Next is Dedicated File Name (DF)
    if (resp->rapdu[index++] != DEDICATED_FILE_TEMPLATE) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // This is followed by the length of the Dedicated File (DF) Name
    // Skip the DF for now
    index += resp->rapdu[index];
    index++;
    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Next comes File Control Information (FCI) Proprietary Template
    if (resp->rapdu[index++] != FCI_PROPRIETARY_TEMPLATE) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    fci_prop_len = resp->rapdu[index++];

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // If we have enough data for this we are good to go to get our data
    if (((index - 1) + fci_prop_len) >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (resp->rapdu[index++] != FCI_ISSUER_DISCRETIONARY_B0) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (resp->rapdu[index++] != FCI_ISSUER_DISCRETIONARY_B1) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Skip FCI_ISSUER_DISCRETIONARY length
    index++;

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Next is APPLICATION_TEMPLATE
    if (resp->rapdu[index++] != APPLICATION_TEMPLATE) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Skip APPLICATION_TEMPLATE len
    index++;

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Next is Application Identifier (AID) – card
    if (resp->rapdu[index++] != APPLICATION_IDENTIFIER) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Get Application Identifier (AID) len
    resp->aid_bin_len = resp->rapdu[index++];

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Also need binary version to match to card AID
    for (i = 0; i < resp->aid_bin_len; i++) {
        resp->aid_bin[i] = resp->rapdu[index++];

        if (index >= resp->rapdu_len) {
            return ISO14443_3_ERR_PROTOCOL;
        }
    }

    // Finally we get to Application Label
    if (resp->rapdu[index++] != APPLICATION_LABEL) {
        // Some cards don't have this
        // In this case we have to do a AID lookup

        info("No App Label, using lookup of AID\n");
        return aid_lookup(resp);
    }

    info("Got a valid App Label!\n");

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // Get Application Label len
    resp->application_label_len = resp->rapdu[index++];

    if (index >= resp->rapdu_len) {
        return ISO14443_3_ERR_PROTOCOL;
    }

    // OK.  Copy the Label into the buffer
    for (i = 0; i < resp->application_label_len; i++) {
        resp->application_label[i] = resp->rapdu[index++];

        if (index >= resp->rapdu_len) {
            return ISO14443_3_ERR_PROTOCOL;
        }
    }

    return ISO14443_3_ERR_SUCCESS;
}

static void decode_MIFARE_type(char* response)
{
    // **************************************************
    // If SAK bit 6 is cleared, this is a non ISO14443-4 Type A
    // card such as MIFARE Classic 1K.
    // Used http://www.nxp.com/documents/application_note/130830.pdf
    // for this decoding
    // **************************************************

    uint8_t sak = get_last_sak();

    if (sak & 0x08) {
        // Bit 4 == 1
        if (sak & 0x10) {
            // Bit 5 == 1
            sprintf(response, "MIFARE 4K\n");
        } else {
            // Bit 5 == 0
            if (sak & 0x01) {
                // Bit 1 == 1
                sprintf(response, "MIFARE Mini\n");
            } else {
                // Bit 1 == 0
                sprintf(response, "MIFARE 1K\n");
            }
        }
    } else {
        // Bit 4 == 0
        if (sak & 0x10) {
            // Bit 5 == 1
            if (sak & 0x01) {
                // Bit 1 == 1
                sprintf(response, "MIFARE Plus 4K SL2\n");
            } else {
                // Bit 1 == 0
                sprintf(response, "MIFARE Plus 2K SL2\n");
            }
        } else {
            // Bit 5 == 0
            if (sak & 0x20) {
                // Bit 6 == 1
                // This version requires RATS etc
                // Is must therfore be compliant, so we should not be here
                // Do nothing
                sprintf(response, "MIFARE ?\n");
            } else {
                // Bit 6 == 0
                sprintf(response, "MIFARE UL\n");
            }
        }
    }
}

/******************************   PUBLIC FUNCTIONS  **************************/
int nfc_init(void)
{
    mml_nfc_pcd_init();
    setup_beeper();

    // ********************************************************************
    // Set the desired default antenna analog configuration
    // ********************************************************************
    if (mml_nfc_pcd_set_analog_config(current_analog_parameters) != MML_NFC_PCD_E_SUCCESS) {
        debug("Failed to get current analog config, possible RF driver version problem\n");
    }

    poweron_operatingfield();
    poweroff_operatingfield();

    return 0;
}

void nfc_enable_polling(void)
{
    g_nfc_active_polling = 1;
    xSemaphoreGive(xNFCLock);
}

void nfc_disable_polling(void)
{
    g_nfc_active_polling = 0;
    xSemaphoreTake(xNFCLock, 0);
}

void vGetNFCTask(void* pvParameter)
{
    (void)pvParameter;

    int status           = 0;
    int i                = 0;
    int k                = 0;
    char mifare_strn[50] = {0x00};
    message_t msgNFC;

    ppse_response_t card_response;

    //g_logging_level = DBG_LVL_NON;

    /* Initialize the NFC reader and wait before enabling */
    nfc_init();

    xNFCLock = xSemaphoreCreateBinary();

    for (;;) {
        while (xSemaphoreTake(xNFCLock, 0xFFFF) != pdTRUE) {
            ;
        }

        msgNFC.pcType = 'L';
        msgNFC.len    = 0;

        // Keep looking for cards as long as we are active
        while (g_nfc_active_polling) {
            vTaskDelay(10);

            poweron_operatingfield();

            status = emvl1_poll_for_card(1);

            if ((status == TYPE_A_READY) || (status == TYPE_B_READY)) {
                if (do_ppse(&card_response) == ISO14443_3_ERR_SUCCESS) {
                    debug("Got a valid ppse response\n");

                    beep_for_success();

                    status = parse_ppse_response(&card_response);

                    if (status == ISO14443_3_ERR_SUCCESS) {
                        debug("parse ppse success\n");

                        // Got a response, put in buffer
                        msgNFC.pcType = 'N';

                        // pcMessage buffer is only 50 bytes.

                        i                     = 0;
                        msgNFC.pcMessage[i++] = 'A';
                        msgNFC.pcMessage[i++] = 'I';
                        msgNFC.pcMessage[i++] = 'D';
                        msgNFC.pcMessage[i++] = ':';
                        msgNFC.pcMessage[i++] = ' ';

                        // print in the AID (convert from HEX to ASCII)
                        for (k = 0; ((k < card_response.aid_bin_len) && (i < 50)); k++, i += 2) {
                            // Note, using pointer math for offset here
                            sprintf((char*)msgNFC.pcMessage + i, "%02X ", card_response.aid_bin[k]);
                        }

                        // Insert Carriage return
                        if (i < 50) {
                            msgNFC.pcMessage[i++] = '\n';
                        }

                        // Now insert Application Label
                        for (k = 0; ((k < card_response.application_label_len) && (i < 50));
                             i++, k++) {
                            msgNFC.pcMessage[i] = card_response.application_label[k];
                        }

                        msgNFC.len = i;

                        debug("Message cnt: %d\n%s\n\n", i, msgNFC.pcMessage);

                        xQueueSendToFront(xQueueMain, &msgNFC, 0);
                        break;
                    } else {
                        debug("Unknown Card\n");

                        // Got a response, put in buffer
                        msgNFC.pcType = 'N';

                        // pcMessage buffer is only 50 bytes.
                        strncpy((char*)msgNFC.pcMessage, "Unknown Card Type", 50);

                        msgNFC.len = 50;

                        xQueueSendToFront(xQueueMain, &msgNFC, 0);
                        break;
                    }
                } else {
                    beep_for_success();

                    debug("Card does not handle PPSE\n");

                    // Got a response, put in buffer
                    msgNFC.pcType = 'N';

                    // pcMessage buffer is only 50 bytes.
                    strncpy((char*)msgNFC.pcMessage, "Card doesn't handle PPSE", 50);

                    msgNFC.len = 50;

                    // Forge success for this card to display message.
                    // It's a success but just doesn't support PPSE
                    status = ISO14443_3_ERR_SUCCESS;

                    xQueueSendToFront(xQueueMain, &msgNFC, 0);
                    break;
                }
            } else if (status == TYPE_A_NON_ISO14443_4_READY) {
                beep_for_success();
                decode_MIFARE_type(mifare_strn);

                msgNFC.pcType = 'N';

                debug("Possible MIFARE Card %s\n", mifare_strn);
                sprintf((char*)msgNFC.pcMessage, "%s", mifare_strn);

                // Set len to null termination of string
                for (i = 0; i < 50; i++) {
                    if (msgNFC.pcMessage[i] == 0) {
                        break;
                    }
                }

                msgNFC.len = i;
                xQueueSendToFront(xQueueMain, &msgNFC, 0);
                break;
            } else if (status == TYPE_B_NON_ISO14443_4_READY) {
                beep_for_success();

                msgNFC.pcType = 'N';

                debug("Type B Non ISO14443 Card found.\n");
                sprintf((char*)msgNFC.pcMessage, "%s", "Non ISO14443 Type B card found");

                // Set len to null termination of string
                for (i = 0; i < 50; i++) {
                    if (msgNFC.pcMessage[i] == 0) {
                        break;
                    }
                }

                msgNFC.len = i;
                xQueueSendToFront(xQueueMain, &msgNFC, 0);
                break;
            } else if ((status != ISO14443_3_ERR_SUCCESS) || (status == NO_CARD_FOUND)) {
                // No supported card detected
                // or card does not support ppse
                // or application id is not recognizable
                // or NO CARD in the field

                // Keep Trying
                continue;
            }

        } // End of while (1) keep looking for cards

        mml_nfc_pcd_field_off();
    }
}
