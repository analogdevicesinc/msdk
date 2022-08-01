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
#include <emv_l1_stack/iso14443_3a_flow.h>
#include <emv_l1_stack/iso14443_3b_cmd.h>
#include <emv_l1_stack/iso14443_3b_flow.h>
#include <emv_l1_stack/iso14443_4_transitive.h>

#include <string.h>
#include "mml_nfc_pcd_port.h"
#include "EMV_polling_and_loopback.h"

#include <mml_nfc_pcd_rf_driver.h>
#include "logging.h"
#include "mxc_device.h"
#include "gpio.h"

#define KEYPRESS_RETURN_DELAY_MS 25

uint8_t rapdu[261];      /**< Shared RAPDU buffer */
int32_t rapdulen;        /**< Length of current RAPDU in the shared buffer */
int32_t rapdu_displayed; /**< Display flag, used to satisfy required EMV DTE logging */

/*
 * uid_resp Buffer to save anticollision response: UID.
 */
uid_storage_t uid_store;

uid_storage_t get_stored_uid(void)
{
    return uid_store;
}

#define BEEPER_PORT       MXC_GPIO3
#define BEEPER_PIN        MXC_GPIO_PIN_3
#define BEEP_PASS_TIME_MS 150
#define BEEP_FAIL_TIME_MS 150

// NOTE: Volume is set as a % of Duty cycle. DTE ships with volume at 10% duty cycle
// 8470 * .1 = 847.
#define BEEP_FAIL_TONE 2500 //8470
#define BEEP_FAIL_VOL  2000 //847
#define BEEP_PASS_TONE 250  //847
#define BEEP_PASS_VOL  25   //84

#define PASS_LED_GPIO_PORT MXC_GPIO3
#define PASS_LED_GPIO_PIN  MXC_GPIO_PIN_4
#define FAIL_LED_GPIO_PORT MXC_GPIO3
#define FAIL_LED_GPIO_PIN  MXC_GPIO_PIN_5

uint32_t indicator_setup = 0;

mxc_gpio_cfg_t pass_led;
mxc_gpio_cfg_t fail_led;
mxc_gpio_cfg_t buzzer_out;

static void do_beep(uint32_t tone, uint32_t vol, uint32_t duration_ms)
{
    uint32_t beep_time     = 0;
    uint32_t beep_loop_cnt = 0;

    if (tone == 0) {
        return;
    }

    if (duration_ms == 0) {
        return;
    }

    if (tone < vol) {
        return;
    }

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

static void setup_indicator(void)
{
    pass_led.port = PASS_LED_GPIO_PORT;
    pass_led.mask = PASS_LED_GPIO_PIN;
    pass_led.pad  = MXC_GPIO_PAD_NONE;
    pass_led.func = MXC_GPIO_FUNC_OUT;

    MXC_GPIO_Config(&pass_led);
    MXC_GPIO_OutSet(PASS_LED_GPIO_PORT, PASS_LED_GPIO_PIN);

    fail_led.port = FAIL_LED_GPIO_PORT;
    fail_led.mask = FAIL_LED_GPIO_PIN;
    fail_led.pad  = MXC_GPIO_PAD_NONE;
    fail_led.func = MXC_GPIO_FUNC_OUT;

    MXC_GPIO_Config(&fail_led);
    MXC_GPIO_OutSet(FAIL_LED_GPIO_PORT, FAIL_LED_GPIO_PIN);

    buzzer_out.port = BEEPER_PORT;
    buzzer_out.mask = BEEPER_PIN;
    buzzer_out.pad  = MXC_GPIO_PAD_NONE;
    buzzer_out.func = MXC_GPIO_FUNC_OUT;
    // Use 3.3V for louder Buzz
    buzzer_out.vssel = MXC_GPIO_VSSEL_VDDIOH;

    MXC_GPIO_Config(&buzzer_out);
    MXC_GPIO_OutClr(BEEPER_PORT, BEEPER_PIN);

    indicator_setup = 1;
}

static void indicate_success(void)
{
    MXC_GPIO_OutClr(PASS_LED_GPIO_PORT, PASS_LED_GPIO_PIN);
    MXC_GPIO_OutSet(FAIL_LED_GPIO_PORT, FAIL_LED_GPIO_PIN);

    do_beep(BEEP_PASS_TONE, BEEP_PASS_VOL, BEEP_PASS_TIME_MS);
}

static void indicate_failure(void)
{
    MXC_GPIO_OutSet(PASS_LED_GPIO_PORT, PASS_LED_GPIO_PIN);
    MXC_GPIO_OutClr(FAIL_LED_GPIO_PORT, FAIL_LED_GPIO_PIN);

    do_beep(BEEP_FAIL_TONE, BEEP_FAIL_VOL, BEEP_FAIL_TIME_MS);
}

static void clear_indications(void)
{
    MXC_GPIO_OutSet(PASS_LED_GPIO_PORT, PASS_LED_GPIO_PIN);
    MXC_GPIO_OutSet(FAIL_LED_GPIO_PORT, FAIL_LED_GPIO_PIN);
}

static int32_t emvl1interopapduloop(void)
{
    int32_t ret;
    // NOTE: need to have a large enough buffer here to handle loopback commands as large as 256 bytes,
    //  plus some header/footer bytes.
    uint8_t capdu[261] = {0x00, 0xA4, 0x04, 0x00, 0x0E, '2', 'P', 'A', 'Y', '.',
                          'S',  'Y',  'S',  '.',  'D',  'D', 'F', '0', '1', 0x00};
    int32_t capdulen   = 20;

    //do apdu.
    do {
        logging("CAPDU ");
        hexdump(DBG_LVL_LOG, capdu, capdulen, 1);

        ret             = SendAPDU(capdu, capdulen, rapdu, &rapdulen);
        rapdu_displayed = 2;

        if (ret == ISO14443_3_ERR_ABORTED) {
            return ret;
        }

        if (ret) {
            switch (ret) {
                case ISO14443_3_ERR_PROTOCOL:
                case ISO14443_3_ERR_TIMEOUT:
                case ISO14443_3_ERR_TRANSMISSION:
                    return RESETPROCEDURE;
            }
        }

        if (rapdulen <= 2) {
            // 2 bytes should be wrong case.
            warning("Short APDU: %ld\n", rapdulen);
            return RESETPROCEDURE;
        }

        // This appears to be a valid RAPDU, allow it to be displayed
        // To comply with logging requirement by EMVCo, we use this
        // mechanic to only display the rapdu once.  Sometimes we do here in the
        // loop-back, other times at the end of the exchange loop below.
        rapdu_displayed = 0;

        if (rapdu[1] == REMOVALPROCEDURE) {
            info("Removal procedure\n");
            return REMOVALPROCEDURE;
        }

        if (rapdu_displayed == 0) {
            rapdu_displayed = 1;
            logging("RAPDU ");
            hexdump(DBG_LVL_LOG, rapdu, rapdulen, 0);
        }

        //pre next capdu,no status
        memcpy(capdu, rapdu, rapdulen - 2);
        capdulen = rapdulen - 2;

    } while (1);

    return ret;
}

/**
 * @brief Implements EMV APDU Loopback for EMV L1 testing and verification
 *
 * Implements EMV APDU Loopback for EMV L1 testing and verification
 *
 * Called by @ref singleemvl1exchange
 *
 * @return #LOOPBACK_RAPDUS
 */
static int32_t emvl1apduloop(void)
{
    int32_t ret;
    // NOTE: need to have a large enough buffer here to handle loopback commands as large as 256 bytes,
    //  plus some header/footer bytes.
    uint8_t capdu[261] = {0x00, 0xA4, 0x04, 0x00, 0x0E, '2', 'P', 'A', 'Y', '.',
                          'S',  'Y',  'S',  '.',  'D',  'D', 'F', '0', '1', 0x00};
    int32_t capdulen   = 20;

    //do apdu.
    do {
        logging("CAPDU ");
        hexdump(DBG_LVL_LOG, capdu, capdulen, 1);

        ret             = SendAPDU(capdu, capdulen, rapdu, &rapdulen);
        rapdu_displayed = 2;

        if (ret == ISO14443_3_ERR_ABORTED) {
            return ret;
        }

        if (ret) {
            switch (ret) {
                case ISO14443_3_ERR_PROTOCOL:
                case ISO14443_3_ERR_TIMEOUT:
                case ISO14443_3_ERR_TRANSMISSION:
                    return RESETPROCEDURE;
            }
        }

        if (rapdulen <= 2) {
            // 2 bytes should be wrong case.
            warning("Short APDU: %ld\n", rapdulen);
            return RESETPROCEDURE;
        }

        // This appears to be a valid RAPDU, allow it to be displayed
        // To comply with logging requirement by EMVCo, we use this
        // mechanic to only display the rapdu once.  Sometimes we do here in the
        // loop-back, other times at the end of the exchange loop below.
        rapdu_displayed = 0;

        if (rapdu[1] == REMOVALPROCEDURE) {
            info("Removal procedure\n");
            return REMOVALPROCEDURE;
        } else if (rapdu[1] == POWEROFFPROCEDURE) {
            info("Power-Off procedure\n");
            return POWEROFFPROCEDURE;
        }

        if (rapdu_displayed == 0) {
            rapdu_displayed = 1;
            logging("RAPDU ");
            hexdump(DBG_LVL_LOG, rapdu, rapdulen, 0);
        }

        //pre next capdu,no status
        memcpy(capdu, rapdu, rapdulen - 2);
        capdulen = rapdulen - 2;

    } while (1);

    return ret;
}

/**
 * @brief Implements EMV card polling for use in application/demo environment
 *
 * Implements EMV card polling for use in application/demo environment
 *
 * @param loop_num how many times through the loop before returning
 *
 * @return @ref POLLING_RESPONSES
 */
#ifdef LOG_ACTIVATION_RESPONSES
int32_t emvl1_poll_for_card(uint32_t loop_num)
{
    int32_t ret;
    uint8_t type_a = 0;
    uint8_t type_b = 0;
    uint32_t count = 0;

    uint8_t atqa_buff[2];
    int32_t atqa_response_len;

    uint8_t resp_buff[256];
    int32_t resp_buff_len = 0;

    uint8_t sak_response;

    for (count = 0; count < loop_num; count++) {
        type_a = 0;
        type_b = 0;

        ret = iso_14443_3a_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_a = 1;
            info("TYPEA\n");
        }

        nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

        ret = iso_14443_3b_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_b = 1;
            info("TYPEB\n");
        }

        if (type_a == 1 && type_b == 1) {
            //Both type a and type b, Collision
            nfc_reset();
            return COLLISION_DETECTED;
        } else if (type_a == 0 && type_b == 0) {
            //No card
            nfc_reset(); // NOTE: This is not done for EMV loopback
            return NO_CARD_FOUND;
        } else if (type_a == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_collision_detect_response(atqa_buff, &atqa_response_len, resp_buff,
                                                         &resp_buff_len, &sak_response);

            if (ret == ISO14443_3_ERR_SUCCESS) {
                logging("ATQA ");
                hexdump(DBG_LVL_LOG, atqa_buff, atqa_response_len, 0);
                logging("UID ");
                hexdump(DBG_LVL_LOG, resp_buff, resp_buff_len, 0);
                memcpy(uid_store.uid, resp_buff, resp_buff_len);
                uid_store.uid_length = resp_buff_len;
                logging("SAK ");
                hexdump(DBG_LVL_LOG, &sak_response, 1, 0);

                ret = iso_14443_3a_active_response(resp_buff, &resp_buff_len);

                if (ret == ISO14443_3_ERR_SUCCESS) {
                    logging("ATS ");
                    hexdump(DBG_LVL_LOG, resp_buff, resp_buff_len, 0);

                    // Got a Type A Card ready for APDUs
                    return TYPE_A_READY;
                } else {
                    //Active  type A failed
                    warning("A active fail: 0x%X\n", ret);
                    nfc_reset();
                    return CARD_FOUND_WITH_ERROR;
                }
            } else {
                // Is this a NON 14443-4 card?
                if (ret == ISO14443_3_ERR_NON_ISO14443_4_CARD) {
                    logging("UID ");
                    hexdump(DBG_LVL_LOG, resp_buff, resp_buff_len, 0);
                    memcpy(uid_store.uid, resp_buff, resp_buff_len);
                    uid_store.uid_length = resp_buff_len;
                    return TYPE_A_NON_ISO14443_4_READY;
                }
                //Type A collision
                warning("A coll fail: 0x%X\n", ret);
                nfc_reset();
                return COLLISION_DETECTED;
            }
        } else if (type_b == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_polling();

            if (ret == ISO14443_3_ERR_SUCCESS) {
                type_a = 1;
                info("TYPEA found after B, Collision\n");
                nfc_reset();
                return COLLISION_DETECTED;
            }

            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3b_collision_detect_response(resp_buff, &resp_buff_len);

            if (ret == ISO14443_3_ERR_SUCCESS) {
                logging("ATQB ");
                hexdump(DBG_LVL_LOG, resp_buff, resp_buff_len, 0);

                ret = iso_14443_3b_active_response(resp_buff, &resp_buff_len);
#ifdef IGNORE_HIGH_INF
                if ((ret == ISO14443_3_ERR_SUCCESS) || (ret == ISO14443_3_ERR_CONTAINS_HIGH_INF)) {
#else
                if (ret == ISO14443_3_ERR_SUCCESS) {
#endif
                    // Got a Type B Card ready for APDUs

                    logging("ATTRIB_RESP ");
                    hexdump(DBG_LVL_LOG, resp_buff, resp_buff_len, 0);
                    return TYPE_B_READY;
                } else {
                    //Active  type B failed
                    warning("B active fail: 0x%x\n", ret);
                    nfc_reset();
                    return CARD_FOUND_WITH_ERROR;
                }
            } else {
                // Is this a NON 14443-4 card?
                if (ret == ISO14443_3_ERR_NON_ISO14443_4_CARD) {
                    return TYPE_B_NON_ISO14443_4_READY;
                }
                //Type B collision
                warning("B coll fail: 0x%X\n", ret);
                nfc_reset();
                return COLLISION_DETECTED;
            }
        }
    }

    return 0;
}

#else
// Don't log activation responses
int32_t emvl1_poll_for_card(uint32_t loop_num)
{
    int32_t ret;
    uint8_t type_a = 0;
    uint8_t type_b = 0;
    uint32_t count = 0;

    for (count = 0; count < loop_num; count++) {
        type_a = 0;
        type_b = 0;

        ret = iso_14443_3a_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_a = 1;
            info("TYPEA\n");
        }

        nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

        ret = iso_14443_3b_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_b = 1;
            info("TYPEB\n");
        }

        if (type_a == 1 && type_b == 1) {
            //Both type a and type b, Collision
            nfc_reset();
            return COLLISION_DETECTED;
        } else if (type_a == 0 && type_b == 0) {
            //No card
            nfc_reset(); // NOTE: This is not done for EMV loopback
            return NO_CARD_FOUND;
        } else if (type_a == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_collision_detect();

            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3a_active();
                if (ret == ISO14443_3_ERR_SUCCESS) {
                    // Got a Type A Card ready for APDUs
                    return TYPE_A_READY;
                } else {
                    //Active  type A failed
                    warning("A active fail: 0x%X\n", ret);
                    nfc_reset();
                    return CARD_FOUND_WITH_ERROR;
                }
            } else {
                // Is this a NON 14443-4 card?
                if (ret == ISO14443_3_ERR_NON_ISO14443_4_CARD) {
                    return TYPE_A_NON_ISO14443_4_READY;
                }
                //Type A collision
                warning("A coll fail: 0x%X\n", ret);
                nfc_reset();
                return COLLISION_DETECTED;
            }
        } else if (type_b == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_polling();

            if (ret == ISO14443_3_ERR_SUCCESS) {
                type_a = 1;
                info("TYPEA found after B, Collision\n");
                nfc_reset();
                return COLLISION_DETECTED;
            }

            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3b_collision_detect();
            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3b_active();
#ifdef IGNORE_HIGH_INF
                if ((ret == ISO14443_3_ERR_SUCCESS) || (ret == ISO14443_3_ERR_CONTAINS_HIGH_INF)) {
#else
                if (ret == ISO14443_3_ERR_SUCCESS) {
#endif
                    // Got a Type B Card ready for APDUs
                    return TYPE_B_READY;
                } else {
                    //Active  type B failed
                    warning("B active fail: 0x%x\n", ret);
                    nfc_reset();
                    return CARD_FOUND_WITH_ERROR;
                }
            } else {
                // Is this a NON 14443-4 card?
                if (ret == ISO14443_3_ERR_NON_ISO14443_4_CARD) {
                    return TYPE_B_NON_ISO14443_4_READY;
                }
                //Type B collision
                warning("B coll fail: 0x%X\n", ret);
                nfc_reset();
                return COLLISION_DETECTED;
            }
        }
    }

    return 0;
}
#endif // LOG_ACTIVATION_RESPONSES

/**
 * @brief Implements EMV card polling for use in Level 1 loop-back testing environment
 *
 * Implements EMV card polling for use in Level 1 loop-back testing environment
 *
 * @param callback @ref callback_check_for_loop_termination_t
 *          Callback to check if this polling loop should be terminated.
 *          For instance the operator desires to switch to a different test mode.
 *
 * @note    If @ref callback_check_for_loop_termination_t is NULL, loop will
 *              not return until a card is found and one exchange is completed.
 *
 * @retval  #POLLING_TERMINATED
 * @retval  #EXCHANGE_COMPLETE
 */
int32_t singleemvl1exchange(callback_check_for_loop_termination_t callback)
{
    int32_t ret;
    uint8_t type_a = 0;
    uint8_t type_b = 0;

    while (1) {
        type_a = 0;
        type_b = 0;

        // If callback for early termination exists, call it
        if (callback) {
            if (callback()) {
                mml_nfc_pcd_task_sleep(KEYPRESS_RETURN_DELAY_MS);
                logging("\nStopping Test\n");
                return 1;
            }
        }

        ret = iso_14443_3a_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_a = 1;
            info("TYPEA\n");
        } else if (ret == ISO14443_3_ERR_ABORTED) {
            logging("Type A polling aborted\n");
            return 1;
        }

        nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

        ret = iso_14443_3b_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_b = 1;
            info("TYPEB\n");
        } else if (ret == ISO14443_3_ERR_ABORTED) {
            logging("Type B polling aborted\n");
            return 1;
        }

        if (type_a == 1 && type_b == 1) {
            //Both type a and type b, Collision
            nfc_reset();
            break;
        } else if (type_a == 0 && type_b == 0) {
            //No card
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            continue;
        } else if (type_a == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_collision_detect();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type A collision detect aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3a_active();

                if (ret == ISO14443_3_ERR_ABORTED) {
                    logging("Type A activate aborted\n");
                    return 1;
                }

                if (ret == ISO14443_3_ERR_SUCCESS) {
                    //APDU
                    ret = emvl1apduloop();

                    if (ret == ISO14443_3_ERR_ABORTED) {
                        logging("Type A APDU loop aborted\n");
                        return 1;
                    }

                    switch (ret) {
                        case REMOVALPROCEDURE:
                            info("A: Removal\n");
                            iso_14443_3a_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                        case RESETPROCEDURE:
                            info("A: Reset\n");
                            nfc_reset();
                            break;
                        case POWEROFFPROCEDURE:
                            info("A: Power Off\n");
                            poweroff_operatingfield();
                            mml_nfc_pcd_task_sleep(TIMEOUT_POWEROFF_MS);
                            poweron_operatingfield();
                            break;
                    }
                    break;
                } else {
                    //Active  type A failed
                    warning("A active fail: 0x%X\n", ret);
                    nfc_reset();
                    break;
                }
            } else {
                //Type A collision
                warning("A coll fail: 0x%X\n", ret);
                nfc_reset();
                break;
            }
        } else if (type_b == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_polling();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type B polling 2 aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                type_a = 1;
                info("TYPEA found after B, Collision\n");
                nfc_reset();
                break;
            }

            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3b_collision_detect();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type B collision detect aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3b_active();

                if (ret == ISO14443_3_ERR_ABORTED) {
                    logging("Type B activate aborted\n");
                    return 1;
                }

#ifdef IGNORE_HIGH_INF
                if ((ret == ISO14443_3_ERR_SUCCESS) || (ret == ISO14443_3_ERR_CONTAINS_HIGH_INF)) {
#else
                if (ret == ISO14443_3_ERR_SUCCESS) {
#endif
                    //APDU
                    ret = emvl1apduloop();

                    if (ret == ISO14443_3_ERR_ABORTED) {
                        logging("Type B APDU loop aborted\n");
                        return 1;
                    }

                    switch (ret) {
                        case REMOVALPROCEDURE:
                            info("B: Removal\n");
                            iso_14443_3b_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                        case RESETPROCEDURE:
                            info("B: Reset\n");
                            nfc_reset();
                            break;
                        case POWEROFFPROCEDURE:
                            info("B: Power Off\n");
                            poweroff_operatingfield();
                            mml_nfc_pcd_task_sleep(TIMEOUT_POWEROFF_MS);
                            poweron_operatingfield();
                            break;
                    }

                    break;
                } else {
                    //Active  type B failed
                    warning("B active fail: 0x%x\n", ret);
                    nfc_reset();
                    break;
                }
            } else {
                //Type B collision
                warning("B coll fail: 0x%X\n", ret);
                nfc_reset();
                break;
            }
        }
        break;
    }

    if (rapdu_displayed == 0) {
        rapdu_displayed = 1;
        logging("RAPDU ");
        hexdump(DBG_LVL_LOG, rapdu, rapdulen, 0);
    }

    return 0;
}

int32_t singleemvl1interopexchange(callback_check_for_loop_termination_t callback)
{
    int32_t ret;
    uint8_t type_a = 0;
    uint8_t type_b = 0;

    if (!indicator_setup) {
        setup_indicator();
    }

    clear_indications();

    while (1) {
        type_a = 0;
        type_b = 0;

        // If callback for early termination exists, call it
        if (callback) {
            if (callback()) {
                mml_nfc_pcd_task_sleep(KEYPRESS_RETURN_DELAY_MS);
                logging("\nStopping Test\n");
                return 1;
            }
        }

        ret = iso_14443_3a_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_a = 1;
            info("TYPEA\n");
        } else if (ret == ISO14443_3_ERR_ABORTED) {
            logging("Type A polling aborted\n");
            return 1;
        }

        nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);

        ret = iso_14443_3b_polling();
        if (ret == ISO14443_3_ERR_SUCCESS) {
            type_b = 1;
            info("TYPEB\n");
        } else if (ret == ISO14443_3_ERR_ABORTED) {
            logging("Type B polling aborted\n");
            return 1;
        }

        if (type_a == 1 && type_b == 1) {
            //Both type a and type b, Collision
            nfc_reset();
            break;
        } else if (type_a == 0 && type_b == 0) {
            //No card
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            continue;
        } else if (type_a == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_collision_detect();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type A collision detect aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3a_active();

                if (ret == ISO14443_3_ERR_ABORTED) {
                    logging("Type A activate aborted\n");
                    return 1;
                }

                if (ret == ISO14443_3_ERR_SUCCESS) {
                    //APDU
                    ret = emvl1interopapduloop();

                    if (ret == ISO14443_3_ERR_ABORTED) {
                        logging("Type A APDU loop aborted\n");
                        return 1;
                    }

                    switch (ret) {
                        case REMOVALPROCEDURE:
                            info("A: Removal\n");
                            logging("Success\n");
                            indicate_success();
                            iso_14443_3a_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                        case RESETPROCEDURE:
                            info("A: Reset\n");
                            logging("Failure\n");
                            indicate_failure();
                            iso_14443_3a_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                    }
                    break;
                } else {
                    //Active  type A failed
                    warning("A active fail: 0x%X\n", ret);
                    nfc_reset();
                    break;
                }
            } else {
                //Type A collision
                warning("A coll fail: 0x%X\n", ret);
                nfc_reset();
                break;
            }
        } else if (type_b == 1) {
            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3a_polling();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type B polling 2 aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                type_a = 1;
                info("TYPEA found after B, Collision\n");
                nfc_reset();
                break;
            }

            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
            ret = iso_14443_3b_collision_detect();

            if (ret == ISO14443_3_ERR_ABORTED) {
                logging("Type B collision detect aborted\n");
                return 1;
            }

            if (ret == ISO14443_3_ERR_SUCCESS) {
                ret = iso_14443_3b_active();

                if (ret == ISO14443_3_ERR_ABORTED) {
                    logging("Type B activate aborted\n");
                    return 1;
                }

#ifdef IGNORE_HIGH_INF
                if ((ret == ISO14443_3_ERR_SUCCESS) || (ret == ISO14443_3_ERR_CONTAINS_HIGH_INF)) {
#else
                if (ret == ISO14443_3_ERR_SUCCESS) {
#endif
                    //APDU
                    ret = emvl1apduloop();

                    if (ret == ISO14443_3_ERR_ABORTED) {
                        logging("Type B APDU loop aborted\n");
                        return 1;
                    }

                    switch (ret) {
                        case REMOVALPROCEDURE:
                            info("B: Removal\n");
                            logging("Success\n");
                            indicate_success();
                            iso_14443_3b_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                        case RESETPROCEDURE:
                            info("B: Reset\n");
                            logging("Failure\n");
                            indicate_failure();
                            iso_14443_3b_remove();
                            nfc_set_delay_till_next_send_fc(TPDELAY_IN_FC);
                            break;
                    }

                    break;
                } else {
                    //Active  type B failed
                    warning("B active fail: 0x%x\n", ret);
                    nfc_reset();
                    break;
                }
            } else {
                //Type B collision
                warning("B coll fail: 0x%X\n", ret);
                nfc_reset();
                break;
            }
        }
        break;
    }

    if (rapdu_displayed == 0) {
        rapdu_displayed = 1;
        logging("RAPDU ");
        hexdump(DBG_LVL_LOG, rapdu, rapdulen, 0);
    }

    return 0;
}
