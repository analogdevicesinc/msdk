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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "EMV_polling_and_loopback.h"
#include "emvl1_app.h"
#include "logging.h"
#include "mml_nfc_pcd_port.h"
#include "mxc_device.h"
#include "pbm_commands.h"

#include <mml_nfc_pcd_rf_driver.h>
#include <uart.h>

#define FIELD_LOAD_DELAY_MS 100
#define KEYPRESS_RETURN_DELAY_MS 25

#define TIMEOUT_TRANSAC_US 509
#define PAUSE_TRANSAC_MS 95

#define REMOVALPROCEDURE 0x70
#define POWEROFFPROCEDURE 0x72

#define RESETPROCEDURE 0x80

#define TYPE_A_READY 0x0A
#define TYPE_B_READY 0x0B

#define MATRIX_HORIZONTAL_NUM_TICKS 27

#define PCD_HW_VER "2.0"
#define PCD_SW_VER "4.0.0"
#define PCD_FW_SUM "4b49c1cbf6e10d37d7559940246a7b9b1993a8dc"

// Per Sony Whitepaper: Card Technical Note for Software Development
//   the Guard Time after transmission of Command Packet Data to when the Reader should
//   be ready to receive preamble is (42 x 64 - 16)/fc ~197us
#define FDT_F_PICC_MIN_TOLERANCE_EARLY ((42 * 64) - 16)

mml_nfc_pcd_analog_params_matrix_t evkit_antenna_112x76_matrix
    = { .fd_thresholds = { 180, 170, 140 },
          .fd_dyn_trigger_a = { 125, 125, 125 },
          .fd_dyn_math_a = { IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I },
          .fd_dyn_trigger_b = { 125, 125, 125 },
          .fd_dyn_math_b = { IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I },
          .fd_dyn_trigger_f = { 100, 100, 100 },
          .fd_dyn_math_f = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
          .fd_dyn_trigger_v = { 125, 125, 125 },
          .fd_dyn_math_v = { IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I },
          .fd_dyn_sttm_a = { 0x00000000, 0x00000000, 0x00000000 },
          .fd_dyn_stfm_a = { 0x7F000000, 0x7F000000, 0x7F000000 },
          .fd_dyn_sttm_bfv = { 0x06060606, 0x06060606, 0x06060606 },
          .fd_dyn_stfm_bfv = { 0x7F060606, 0x7F060606, 0x7F060606 },
          .fd_dyn_gain = { 12, 12, 12 },
          .fd_dyn_atten = { 0x1F, 0x1F, 0x1F } };

mml_nfc_pcd_analog_params_matrix_t custom_antenna_65x65_matrix = { .fd_thresholds
    = { 220, 195, 180, 160, 120, 50 },
    .fd_dyn_trigger_a = { 45, 45, 50, 85, 100, 120 },
    .fd_dyn_math_a
    = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
    .fd_dyn_trigger_b = { 60, 60, 60, 20, 20, 20 },
    .fd_dyn_math_b
    = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
    .fd_dyn_trigger_f = { 40, 35, 25 },
    .fd_dyn_math_f = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
    .fd_dyn_trigger_v = { 40, 35, 25 },
    .fd_dyn_math_v = { IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I, IQ_MATH_Q_MINUS_I },
    .fd_dyn_sttm_a = { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 },
    .fd_dyn_stfm_a = { 0x7F000000, 0x7F000000, 0x7F000000, 0x7F000000, 0x7F000000, 0x7F000000 },
    .fd_dyn_sttm_bfv = { 0x0D0D0D0D, 0x0B0B0B0B, 0x0B0B0B0B, 0x0B0B0B0B, 0x08080808, 0x06060606 },
    .fd_dyn_stfm_bfv = { 0x7F0D0D0D, 0x7F0B0B0B, 0x7F0B0B0B, 0x7F0B0B0B, 0x7F080808, 0x7F5A5550 },
    .fd_dyn_gain = { 12, 12, 12, 12, 12, 12 },
    .fd_dyn_atten = { 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F } };

/*
mml_nfc_pcd_analog_params_matrix_t custom_antenna_65x65_matrix = {
    .fd_thresholds          = {255, 170, 160, 125},
    .fd_dyn_trigger_a       = { 45,  45,  45},
    .fd_dyn_math_a          = { IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I},
    .fd_dyn_trigger_b       = { 125, 125, 125},
    .fd_dyn_trigger_b       = { 35,  35,  35},
    .fd_dyn_math_b          = { IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I},
    .fd_dyn_trigger_f       = { 40,  35,  25},
    .fd_dyn_math_f          = { IQ_MATH_CH_I,       IQ_MATH_CH_I,       IQ_MATH_CH_I},
    .fd_dyn_trigger_v       = { 40,  35,  25},
    .fd_dyn_math_v          = { IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I,  IQ_MATH_Q_MINUS_I},
    .fd_dyn_sttm_a          = { 0x00000000, 0x00000000, 0x00000000},
    .fd_dyn_stfm_a          = { 0x7F000000, 0x7F000000, 0x7F000000},
    .fd_dyn_sttm_bfv        = { 0x06060606, 0x06060606, 0x06060606},
    .fd_dyn_stfm_bfv        = { 0x7F060606, 0x7F060606, 0x7F060606},
    .fd_dyn_gain            = { 12,   12,    12},
    .fd_dyn_atten           = { 0x1F, 0x1F,  0x1F}
};
*/

uint8_t dte_buffer[280];

// Character buffer for data input
char input_buffer[32];

int32_t key_has_been_pressed(void);
uint8_t get_key_press_no_echo(void);

// FIXME: remove when replaced by mxc_sys subroutine
int32_t trim_ro_to_rtc(void)
{
#define NBB_BASE 0x40000800
#define NBBFCR1_OFF 0x04
#define AC_EN_POS 0
#define AC_RUNNING_POS 1

    volatile uint32_t* nbbfcr1_reg = (volatile uint32_t*)(NBB_BASE + NBBFCR1_OFF);

    // Need the 32Khz crystal enabled to do this trim
    int ret_val = MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);

    if (ret_val != E_SUCCESS) {
        return ret_val;
    }

    // Bits: [9:8] MU[1:0] 	- How quickly it allowed to tune, provides some hysteresis
    //	         2 LDTRM	- Allows it to actually use the trim?  unclear
    //           1 ACRUN	- Runs the autocal
    //           0 ACEN		- Enabled the autocal block.  Unclear why need this and run
    *nbbfcr1_reg = 0x307;

    // At this point it should run until disabled.

    // Wait a bit to let autocal run, see if it did anything.
    mml_nfc_pcd_task_sleep(100);

    if ((*nbbfcr1_reg & 0xFF80000) == 0) {
        return E_BAD_STATE;
    }

    return E_SUCCESS;
}

void clear_screen(void)
{
    printf("\n\n\n\n\n\n\n\n\n\n \n\n\n\n\n\n\n\n\n\n \n\n\n\n\n\n\n\n\n\n");
}

uint8_t get_key_press(void)
{
    int32_t status = 0;
    uint8_t key_press = 0;

    // Wait for available char
    do {
        mml_nfc_pcd_task_sleep(FIELD_LOAD_DELAY_MS);
        status = MXC_UART_GetRXFIFOAvailable(MXC_UART0);
    } while (status < 1);

    key_press = MXC_UART_ReadCharacter(MXC_UART0);

    // Echo the key back
    MXC_UART_WriteCharacter(MXC_UART0, key_press);

    return key_press;
}

uint8_t get_key_press_no_echo(void)
{
    int32_t status = 0;
    uint8_t key_press = 0;

    // Wait for available char
    do {
        mml_nfc_pcd_task_sleep(FIELD_LOAD_DELAY_MS);
        status = MXC_UART_GetRXFIFOAvailable(MXC_UART0);
    } while (status < 1);

    key_press = MXC_UART_ReadCharacter(MXC_UART0);

    return key_press;
}

int32_t key_has_been_pressed(void)
{
    int32_t status = 0;

    status = MXC_UART_GetRXFIFOAvailable(MXC_UART0);

    // Eat random keypresses, to avoid bad menu interactions
    MXC_UART_ClearRXFIFO(MXC_UART0);

    // If we have any available then a key has been pressed
    if (status > 0) {
        return 1;
    }
    return 0;
}

int32_t dte_get_input_string(char* input_string)
{
    uint8_t key_press = 0;
    int32_t char_count = 0;

    while (1) {
        key_press = get_key_press();

        if ((key_press == '\n') || (key_press == '\r')) {
            break;
        }

        if ((key_press == ' ') && (char_count == 0)) {
            continue; // ignore leading spaces
        }

        if ((key_press == '\b') && (char_count > 0)) {
            // backspace, back up one character
            // NOTE: we sent the terminal one backspace already
            // but now we want to clear out the character, so put ' ' and then another '\b'
            printf(" \b");
            char_count--;
            continue;
        }

        input_string[char_count++] = key_press;
    }

    // Null terminate the string
    input_string[char_count] = 0x00;

    return char_count;
}

int32_t dte_get_int_val_raw(int32_t exp_char_count, int32_t min_val)
{
    int32_t char_count = 0;
    int32_t input_int = 0;

    while (1) {
        char_count = dte_get_input_string(input_buffer);

        if (char_count > exp_char_count) {
            printf("\nMax characters allowed %ld try again\n", exp_char_count);
            return -1;
        }

        if (char_count > 0) {
            break;
        }
    }

    // Get actual data
    sscanf(input_buffer, "%d", &input_int);

    if (char_count > exp_char_count) {
        printf("\nMax characters allowed %ld try again\n", exp_char_count);
        return -1;
    }

    return input_int;
}

int32_t dte_get_int_val(char name[], int32_t exp_char_count, int32_t min_val, int32_t max_val)
{
    int32_t new_value = 0;

    printf("Enter an new %s value from %ld to %ld\n", name, min_val, max_val);
    printf("=> ");

    new_value = dte_get_int_val_raw(exp_char_count, min_val);

    if ((new_value < min_val) || (new_value > max_val)) {
        printf("Value not in allowed range try again\n");
        return -1;
    }

    printf("New %s value: %ld\n", name, new_value);

    return new_value;
}

int32_t dte_get_math_val(char name[])
{
    int32_t new_value = 0;

    while (1) {
        printf("Select a combiner math from these options:\n");
        printf("1. IQ_MATH_CH_I\n");
        printf("2. IQ_MATH_CH_Q\n");
        printf("3. IQ_MATH_I_MINUS_Q\n");
        printf("4. IQ_MATH_Q_MINUS_I\n");
        printf("5. IQ_MATH_I_PLUS_Q\n");
        printf("=> ");

        new_value = dte_get_int_val_raw(1, 1);

        if ((new_value < 1) || (new_value > 5)) {
            printf("Selection not in allowed range try again\n");
        } else {
            break;
        }
    }

    printf("New %s selection: ", name);

    switch (new_value) {
    case 1:
        printf("IQ_MATH_CH_I\n");
        return IQ_MATH_CH_I;
    case 2:
        printf("IQ_MATH_CH_Q\n");
        return IQ_MATH_CH_Q;
    case 3:
        printf("IQ_MATH_I_MINUS_Q\n");
        return IQ_MATH_I_MINUS_Q;
    case 4:
        printf("IQ_MATH_Q_MINUS_I\n");
        return IQ_MATH_Q_MINUS_I;
    case 5:
        printf("IQ_MATH_I_PLUS_Q\n");
        return IQ_MATH_I_PLUS_Q;

    default:
        info("IQ_MATH_I_MINUS_Q\n");
        return IQ_MATH_I_MINUS_Q;
    }
}

void show_current_field_loading_level(void)
{
    // Displays both the current sensed field level, and the current threshold
    uint8_t sensed_threshold = 0;
    uint32_t field_sensed = 0;
    uint8_t current_field_level = 0;

    poweroff_operatingfield();
    poweron_operatingfield();

    printf("\nField Load Level:              ");

    while (1) {
        if (key_has_been_pressed()) {
            mml_nfc_pcd_task_sleep(FIELD_LOAD_DELAY_MS);
            logging("\n\n");
            poweroff_operatingfield();
            return;
        }

        // Sweep through and identify the requested level
        for (sensed_threshold = 0; sensed_threshold < FD_THRESH_NUM_STEPS;) {
            mml_nfc_pcd_detect_loading(
                mml_nfc_pcd_analog_parameters_matrix.fd_thresholds[sensed_threshold],
                &field_sensed);

            if (field_sensed) {
                break;
            }

            sensed_threshold++;
        }

        // Bounds check
        if (sensed_threshold > (FD_THRESH_NUM_STEPS - 1)) {
            sensed_threshold = FD_THRESH_NUM_STEPS - 1;
        }

        mml_nfc_pcd_find_current_field_loading(&current_field_level);

        printf("\b\b\b\b\b\b\b\b\b\b\b\b%3d index: %d", current_field_level, sensed_threshold);

        mml_nfc_pcd_task_sleep(FIELD_LOAD_DELAY_MS);
    }
}

void dte_antenna_selection(void)
{
    while (1) {
        clear_screen();

        printf("Antenna Selection Menu\n");
        printf("L. Load Analog Settings for 112x76 Antenna\n");
        printf("C. Load Analog Settings for Custom 65x65 Antenna\n");
        printf("E. Exit back to previous menu\n");
        printf("=> ");

        switch (get_key_press()) {
            // Load Analog Settings for 112x76 Antenna
        case 'L':
        case 'l':
            mml_nfc_pcd_analog_parameters_matrix = evkit_antenna_112x76_matrix;
            poweroff_operatingfield();
            poweron_operatingfield();
            printf("\nEVKIT Antenna 112x76 Settings Loaded\n");
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
            // Load Analog Settings for 65x65 Antenna
        case 'C':
        case 'c':
            mml_nfc_pcd_analog_parameters_matrix = custom_antenna_65x65_matrix;
            poweroff_operatingfield();
            poweron_operatingfield();
            printf("\nCustom antenna 65x65 Settings Loaded\n");
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
            // Exit
        case 'E':
        case 'e':
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

void print_comb_math(uint8_t iq_math)
{
    switch (iq_math) {
    case IQ_MATH_CH_I:
        printf("       I | ");
        break;
    case IQ_MATH_CH_Q:
        printf("       Q | ");
        break;
    case IQ_MATH_I_MINUS_Q:
        printf("     I-Q | ");
        break;
    case IQ_MATH_Q_MINUS_I:
        printf("     Q-I | ");
        break;
    case IQ_MATH_I_PLUS_Q:
        printf("     I+Q | ");
        break;

    default:
        printf("     ??? | ");
        break;
    }
}

void print_matrix_header(void)
{
    int i;
    printf("\nStep Index:                 ");
    for (i = 0; i < FD_THRESH_NUM_STEPS; i++) { printf("%8d | ", i); }
    printf("\n");
    for (i = 0; i < MATRIX_HORIZONTAL_NUM_TICKS; i++) { printf("-"); }
    for (i = 0; i < FD_THRESH_NUM_STEPS; i++) { printf("-----------"); }
    printf("\n");
}

void get_analog_math_value(uint8_t val_array[], char name[])
{
    int i, j;
    int new_val = 0;
    char val_string[64];
    uint8_t key_press = 0;

    printf("Enter index to change, or 'A' for all\n");
    printf("=> ");

    while (1) {
        key_press = get_key_press();
        if (((key_press >= '0') && (key_press <= ((FD_THRESH_NUM_STEPS - 1) + '0')))
            || (key_press == 'A') || (key_press == 'a')) {
            break;
        }

        printf("Invalid selection try again\n");
        printf("=> ");
    }

    if ((key_press == 'A') || (key_press == 'a')) {
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_matrix_header();
            printf("%s", name);
            for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { print_comb_math(val_array[j]); }
            printf("\n");

            sprintf(val_string, "value for index %d", i);

            new_val = dte_get_math_val(val_string);

            val_array[i] = new_val;
        }
    } else {
        i = key_press - '0';
        print_matrix_header();
        printf("%s", name);
        for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { print_comb_math(val_array[j]); }
        printf("\n");

        sprintf(val_string, "value for index %d", i);

        new_val = dte_get_math_val(val_string);

        val_array[i] = new_val;
    }
}

void get_hex_analog_matrix_value(uint32_t val_array[], char name[])
{
    int i, j;
    uint32_t new_val = 0;
    char val_string[64];
    uint8_t key_press = 0;
    int scan_status = 0;

    printf("Enter index to change, or 'A' for all\n");
    printf("=> ");

    while (1) {
        key_press = get_key_press();
        if (((key_press >= '0') && (key_press <= ((FD_THRESH_NUM_STEPS - 1) + '0')))
            || (key_press == 'A') || (key_press == 'a')) {
            break;
        }

        printf("Invalid selection try again\n");
        printf("=> ");
    }

    if ((key_press == 'A') || (key_press == 'a')) {
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_matrix_header();
            printf("%s", name);
            for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { printf("%08X | ", val_array[j]); }
            printf("\n");

            sprintf(val_string, "value for index %d", i);

            do {
                printf("Enter an new value in HEX with no 0x\n");
                printf("=> ");
                dte_get_input_string(input_buffer);
                scan_status = sscanf(input_buffer, "%X", &new_val);
            } while (scan_status != 1);

            val_array[i] = new_val;
        }
    } else {
        i = key_press - '0';

        print_matrix_header();

        printf("%s", name);
        for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { printf("%08X | ", val_array[j]); }
        printf("\n");

        sprintf(val_string, "value for index %d", i);

        do {
            printf("Enter an new value in HEX with no 0x\n");
            printf("=> ");
            dte_get_input_string(input_buffer);
            scan_status = sscanf(input_buffer, "%X", &new_val);
        } while (scan_status != 1);

        val_array[i] = new_val;
    }
}

void get_analog_matrix_value(
    uint8_t val_array[], char name[], int32_t exp_char_count, int32_t min_val, int32_t max_val)
{
    int i, j;
    int new_val = 0;
    char val_string[64];
    uint8_t key_press = 0;

    printf("Enter index to change, or 'A' for all\n");
    printf("=> ");

    while (1) {
        key_press = get_key_press();
        if (((key_press >= '0') && (key_press <= ((FD_THRESH_NUM_STEPS - 1) + '0')))
            || (key_press == 'A') || (key_press == 'a')) {
            break;
        }

        printf("Invalid selection try again\n");
        printf("=> ");
    }

    if ((key_press == 'A') || (key_press == 'a')) {
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_matrix_header();
            printf("%s", name);
            for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { printf("     %3d | ", val_array[j]); }
            printf("\n");

            sprintf(val_string, "value for index %d", i);

            do {
                new_val = dte_get_int_val(val_string, exp_char_count, min_val, max_val);
            } while (new_val == -1);

            val_array[i] = new_val;
        }
    } else {
        i = key_press - '0';

        print_matrix_header();

        printf("%s", name);
        for (j = 0; j < FD_THRESH_NUM_STEPS; j++) { printf("     %3d | ", val_array[j]); }
        printf("\n");

        sprintf(val_string, "value for index %d", i);

        do {
            new_val = dte_get_int_val(val_string, exp_char_count, min_val, max_val);
        } while (new_val == -1);

        val_array[i] = new_val;
    }
}

void force_rf_settings_update(void)
{
    // We will use a dummy transaction to force the actual config update
    int32_t tx_buffer_len = 1;
    uint8_t tx_buffer[1] = { 0x00 };

    uint8_t receive_buffer[256];
    uint32_t receive_len = 0;

    int32_t status = MML_NFC_PCD_E_SUCCESS;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.frametype = FT_STANDARD_CRC_NO_EMD;
    trans_params.tx_buf = tx_buffer;
    trans_params.tx_len = tx_buffer_len;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.early_limit = 4000;
    trans_params.timeout = 5000;
    trans_params.delay_till_send = 3000;

    while (1) {
        printf("\n\nUpdate and set RF config for which protocol?\n\n");

        printf("1. PROTOCOL_ISO14443A\n");
        printf("2. PROTOCOL_ISO14443B\n");
        printf("3. PROTOCOL_TYPE_F\n");
        printf("4. PROTOCOL_ISO15693_100_1OF4_SINGLE_HIGH\n");
        printf("5. PROTOCOL_ISO15693_100_1OF4_SINGLE_LOW\n");
        printf("6. PROTOCOL_ISO15693_100_1OF4_DUAL_HIGH\n");
        printf("7. PROTOCOL_ISO15693_100_1OF4_DUAL_LOW\n");
        printf("8. PROTOCOL_ISO15693_100_1OF256_SINGLE_HIGH\n");
        printf("9. PROTOCOL_ISO15693_100_1OF256_SINGLE_LOW\n");
        printf("A. PROTOCOL_ISO15693_100_1OF256_DUAL_HIGH\n");
        printf("B. PROTOCOL_ISO15693_100_1OF256_DUAL_LOW\n");
        printf("C. PROTOCOL_ISO15693_10_1OF4_SINGLE_HIGH\n");
        printf("D. PROTOCOL_ISO15693_10_1OF4_SINGLE_LOW\n");
        printf("E. PROTOCOL_ISO15693_10_1OF4_DUAL_HIGH\n");
        printf("F. PROTOCOL_ISO15693_10_1OF4_DUAL_LOW\n");
        printf("G. PROTOCOL_ISO15693_10_1OF256_SINGLE_HIGH\n");
        printf("H. PROTOCOL_ISO15693_10_1OF256_SINGLE_LOW\n");
        printf("I. PROTOCOL_ISO15693_10_1OF256_DUAL_HIGH\n");
        printf("J. PROTOCOL_ISO15693_10_1OF256_DUAL_LOW\n");

        switch (get_key_press()) {
        case '1':
            trans_params.protocol = PROTOCOL_ISO14443A;
            break;
        case '2':
            trans_params.protocol = PROTOCOL_ISO14443B;
            break;
        case '3':
            trans_params.protocol = PROTOCOL_TYPE_F;
            break;
        case '4':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_SINGLE_HIGH;
            break;
        case '5':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_SINGLE_LOW;
            break;
        case '6':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_DUAL_HIGH;
            break;
        case '7':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_DUAL_LOW;
            break;
        case '8':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_SINGLE_HIGH;
            break;
        case '9':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_SINGLE_LOW;
            break;
        case 'A':
        case 'a':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_DUAL_HIGH;
            break;
        case 'B':
        case 'b':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_DUAL_LOW;
            break;
        case 'C':
        case 'c':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_SINGLE_HIGH;
            break;
        case 'D':
        case 'd':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_SINGLE_LOW;
            break;
        case 'E':
        case 'e':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_DUAL_HIGH;
            break;
        case 'F':
        case 'f':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_DUAL_LOW;
            break;
        case 'G':
        case 'g':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_SINGLE_HIGH;
            break;
        case 'H':
        case 'h':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_SINGLE_LOW;
            break;
        case 'I':
        case 'i':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_DUAL_HIGH;
            break;
        case 'J':
        case 'j':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_DUAL_LOW;
            break;

        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            continue;
        }

        status = mml_nfc_pcd_transceive(trans_params);

        if ((status != MML_NFC_PCD_E_SUCCESS) && (status != MML_NFC_PCD_E_TIMEOUT)) {
            printf("Error during attempt to set RF for protocol: %d\n", status);
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
        } else {
            printf("RF config set\n");
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
        }

        return;
    }
}

void print_matrix_spacer(void)
{
    int i = 0;

    for (i = 0; i < 27; i++) { printf("-"); }
    for (i = 0; i < FD_THRESH_NUM_STEPS; i++) { printf("-----------"); }
    printf("\n");
}

void dte_analog_settings()
{
    int i = 0;

    while (1) {
        clear_screen();

        printf("Maxim Analog Settings Menu\n");
        print_matrix_header();

        printf("FD Thresholds:              ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_thresholds[i]);
        }
        printf("\n");

        print_matrix_spacer();

        printf("FD Dynamic Combiner Math A: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_comb_math(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_a[i]);
        }
        printf("\n");
        printf("FD Dynamic Combiner Math B: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_comb_math(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_b[i]);
        }
        printf("\n");
        printf("FD Dynamic Combiner Math F: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_comb_math(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_f[i]);
        }
        printf("\n");
        printf("FD Dynamic Combiner Math V: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            print_comb_math(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_v[i]);
        }

        printf("\n");

        printf("\n");
        printf("FD Dynamic STTM_A:          ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%08X | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_a[i]);
        }
        printf("\n");
        printf("FD Dynamic STFM_A:          ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%08X | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_a[i]);
        }
        printf("\n");
        printf("FD Dynamic STTM_BFV:        ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%08X | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_bfv[i]);
        }
        printf("\n");
        printf("FD Dynamic STFM_BFV:        ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%08X | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_bfv[i]);
        }

        printf("\n");

        printf("\n");
        printf("FD Dynamic Trigger Level A: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_a[i]);
        }
        printf("\n");
        printf("FD Dynamic Trigger Level B: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_b[i]);
        }
        printf("\n");
        printf("FD Dynamic Trigger Level F: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_f[i]);
        }

        printf("\n");

        printf("\n");
        printf("FD Dynamic Trigger Level V: ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_v[i]);
        }
        printf("\n");
        printf("FD Dynamic Gain:            ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_gain[i]);
        }
        printf("\n");
        printf("FD Dynamic Attenuation:     ");
        for (i = 0; i < FD_THRESH_NUM_STEPS; i++) {
            printf("%8d | ", mml_nfc_pcd_analog_parameters_matrix.fd_dyn_atten[i]);
        }
        printf("\n");
        print_matrix_spacer();

        printf("1. Change FD Thresholds\n");

        printf("\n");

        printf("2. Change FD Dynamic Combiner Math A\n");
        printf("3. Change FD Dynamic Combiner Math B\n");
        printf("4. Change FD Dynamic Combiner Math F\n");
        printf("5. Change FD Dynamic Combiner Math V\n");

        printf("\n");

        printf("6. Change FD Dynamic STTM_A\n");
        printf("7. Change FD Dynamic STFM_A\n");
        printf("8. Change FD Dynamic STTM_BFV\n");
        printf("9. Change FD Dynamic STFM_BFV\n");

        printf("\n");

        printf("A. Change FD Dynamic Trigger A\n");
        printf("B. Change FD Dynamic Trigger B\n");
        printf("F. Change FD Dynamic Trigger F\n");
        printf("V. Change FD Dynamic Trigger V\n");

        printf("\n");

        printf("G. Change FD Dynamic Gain\n");
        printf("R. Change FD Dynamic Attenuation\n");

        printf("\n");

        printf("T. Test/Display Field Detect Level\n");
        printf("U. Update actual RF settings\n");
        printf("E. Exit back to previous menu\n");
        printf("=> ");

        switch (get_key_press()) {
            // FD Thresholds
        case '1':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_thresholds,
                "FD Thresholds               ", 3, 0, 255);
            break;
            // FD Dynamic STTM_A
        case '6':
            get_hex_analog_matrix_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_a, "FD Dynamic STTM_A:          ");
            break;
            // FD Dynamic STFM_A
        case '7':
            get_hex_analog_matrix_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_a, "FD Dynamic STFM_A           ");
            break;
            // FD Dynamic STTM_BFV
        case '8':
            get_hex_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_bfv,
                "FD Dynamic STTM_BVF         ");
            break;
            // FD Dynamic STFM_BFV
        case '9':
            get_hex_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_bfv,
                "FD Dynamic STFM_BVF         ");
            break;
            // FD Dynamic Trigger A
        case 'A':
        case 'a':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_a,
                "FD Dynamic Trigger A        ", 3, 0, 127);
            break;
            // FD Dynamic Math A
        case '2':
            get_analog_math_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_a, "FD Dynamic Combiner Math A  ");
            break;
            // FD Dynamic Trigger B
        case 'B':
        case 'b':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_b,
                "FD Dynamic Trigger B        ", 3, 0, 127);
            break;
            // FD Dynamic Math B
        case '3':
            get_analog_math_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_b, "FD Dynamic Combiner Math B  ");
            break;
            // FD Dynamic Trigger F
        case 'F':
        case 'f':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_f,
                "FD Dynamic Trigger F        ", 3, 0, 127);
            break;
            // FD Dynamic Math F
        case '4':
            get_analog_math_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_f, "FD Dynamic Combiner Math F  ");
            break;
            // FD Dynamic Trigger V
        case 'V':
        case 'v':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_v,
                "FD Dynamic Trigger V        ", 3, 0, 127);
            break;
            // FD Dynamic Math V
        case '5':
            get_analog_math_value(
                mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_v, "FD Dynamic Combiner Math V  ");
            break;
            // FD Dynamic Gain
        case 'G':
        case 'g':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_gain,
                "FD Dynamic Gain             ", 2, 0, 12);
            break;
            // FD Dynamic Attenuation
        case 'R':
        case 'r':
            get_analog_matrix_value(mml_nfc_pcd_analog_parameters_matrix.fd_dyn_atten,
                "FD Dynamic Attenuation      ", 2, 0, 0x1F);
            break;
            // FD Display
        case 'T':
        case 't':
            printf("\n\nPress any key to quit.\n");
            show_current_field_loading_level();
            break;
            // Update RF Settings
        case 'U':
        case 'u':
            force_rf_settings_update();
            break;
            // Exit
        case 'E':
        case 'e':
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    } // End of while 1
}

void dte_settings(void)
{
    while (1) {
        clear_screen();

        printf("Maxim Settings Menu\n");
        printf("Logging Level: %ld\n", g_logging_level);

        printf("A. View or Change Analog Settings\n");
        printf("N. Select Antenna and Set Analog Defaults\n");
        printf("L. Change Logging Level\n");
        printf("E. Exit back to previous menu\n");
        printf("=> ");

        switch (get_key_press()) {
            // Analog Settings
        case 'A':
        case 'a':
            dte_analog_settings();
            break;
        case 'N':
        case 'n':
            dte_antenna_selection();
            break;
            // Logging level change
        case 'L':
        case 'l':
            printf("\nSelect Logging Level:\n");
            printf("0. None\n");
            printf("1. Logging (Default)\n");
            printf("2. Errors\n");
            printf("3. Warnings\n");
            printf("4. Informational\n");
            printf("5. Debug\n");
            printf("6. Full Debug\n");
            switch (get_key_press()) {
            case '0':
                g_logging_level = DBG_LVL_NON;
                break;
            case '1':
                g_logging_level = DBG_LVL_LOG;
                break;
            case '2':
                g_logging_level = DBG_LVL_ERR;
                break;
            case '3':
                g_logging_level = DBG_LVL_WRN;
                break;
            case '4':
                g_logging_level = DBG_LVL_INF;
                break;
            case '5':
                g_logging_level = DBG_LVL_DBG;
                break;
            case '6':
                g_logging_level = DBG_LVL_FDB;
                break;
            default:
                printf("\n**Invalid selection no change made**\n\n");
                break;
            }

            logging("\nshowing logging messages\n");
            error("showing error messages\n");
            warning("showing warn messages\n");
            info("showing info messages\n");
            debug("showing debug messages\n");
            full_debug("showing full_debug messages\n");

            printf("\nPress any key to continue...\n");
            get_key_press_no_echo();

            break;
            // Exit
        case 'E':
        case 'e':
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

void dte_get_loop_params(int32_t* num_loops)
{
    uint8_t key_press = 0;
    int32_t char_count = 0;
    int32_t temp_count = 0;
    int32_t i;
    uint8_t loop_input[5];

    while (1) {
        clear_screen();
        printf("Loop Setting: Num Loops: %ld\n", *num_loops);
        printf("Enter an new number of loops (1-9999), then press enter\n");
        printf("=> ");

        while (char_count < 5) {
            key_press = get_key_press();

            if ((key_press == '\n') || (key_press == '\r')) {
                break;
            }

            if ((key_press < '0') || (key_press > '9')) {
                printf("\nInvalid digit, only 0-9 allowed.\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                return;
            }

            loop_input[char_count++] = key_press;
        }

        if (char_count > 4) {
            printf("\nMax Allowed 9999. Try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            return;
        }

        if (char_count > 0) {
            break;
        }
    }

    // Back up to last actual digit
    char_count--;

    for (i = 1; char_count >= 0; char_count--, i *= 10) {
        temp_count += (loop_input[char_count] - '0') * i;
    }

    if ((temp_count < 0) || (temp_count > 9999)) {
        printf("Not an allowed number of loops, not changing\n");
        printf("Press any key to retry...\n");
        get_key_press_no_echo();
        return;
    }

    *num_loops = temp_count;

    printf("New Number of Loops: %ld\n", *num_loops);
    printf("Press any key to continue...\n");
    get_key_press_no_echo();

    return;
}

void dte_do_analog_polling(int32_t num_loops)
{
    int32_t i;

    printf("\nAnalog Loop Back Mode\n");
    printf(
        "\n!Test will loop for %ld times or until any key is pressed to stop test!\n\n", num_loops);

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        printf("\nAnalogue %ld\n", i);

        if (singleemvl1exchange(key_has_been_pressed)) {
            nfc_reset();
            break;
        }
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_reset(int32_t num_loops)
{
    int32_t i;

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        if (key_has_been_pressed()) {
            break;
        }

        nfc_pcd_reset_wait();
        printf("\nReset Loop Status: Num Loops: %ld, Count: %ld\n", num_loops, i);
        nfc_reset();
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_wupa(int32_t num_loops)
{
    int32_t i;
    int32_t ret = 0;

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        if (key_has_been_pressed()) {
            break;
        }

        nfc_pcd_reset_wait();
        printf("\nWUPA Loop Status: Num Loops: %ld, Count: %ld\n", num_loops, i);
        printf("WUPA ->\n");
        ret = iso_14443_3a_cmd_req_wupa(ISO_14443_3A_CMD_WUPA, dte_buffer, WAKEUP_NOTRETRY);
        if (ret == ISO14443_3_ERR_SUCCESS) {
            printf("ATQA <-\n");
        }
        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_rats(int32_t num_loops)
{
    int32_t i;
    int32_t ret = 0;

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        if (key_has_been_pressed()) {
            break;
        }

        nfc_pcd_reset_wait();
        printf("\nRATS Loop Status: Num Loops: %ld, Count: %ld\n", num_loops, i);

        printf("ANTICOLISION ->\n");
        ret = iso_14443_3a_collision_detect();

        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
        if (ret != ISO14443_3_ERR_SUCCESS) {
            continue;
        }

        printf("ANTICOLISION RESP <-\n");

        printf("RATS ->\n");
        ret = iso_14443_3a_active();
        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
        if (ret != ISO14443_3_ERR_SUCCESS) {
            continue;
        }

        printf("ATS <-\n");
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_wupb(int32_t num_loops)
{
    int32_t i;
    int32_t ret = 0;
    int32_t atq_len = 0;

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        if (key_has_been_pressed()) {
            break;
        }

        nfc_pcd_reset_wait();
        printf("\nWUPB Loop Status: Num Loops: %ld, Count: %ld\n", num_loops, i);
        printf("WUPB ->\n");
        ret = iso_14443_3b_cmd_req_wup(dte_buffer, &atq_len, WAKEUP_NOTRETRY);
        if (ret == ISO14443_3_ERR_SUCCESS) {
            printf("ATQB <-\n");
        }
        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_attrib(int32_t num_loops)
{
    int32_t i;
    int32_t ret = 0;

    for (i = 0; (i < num_loops) || (num_loops == 0); i++) {
        if (key_has_been_pressed()) {
            break;
        }

        nfc_pcd_reset_wait();
        printf("\nATTRIB Loop Status: Num Loops: %ld, Count: %ld\n", num_loops, i);

        printf("ANTICOLISION ->\n");
        ret = iso_14443_3b_collision_detect();

        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
        if (ret != ISO14443_3_ERR_SUCCESS) {
            continue;
        }

        printf("ANTICOLISION RESP <-\n");

        printf("ATTRIB ->\n");
        ret = iso_14443_3b_active();

        if (ret == ISO14443_3_ERR_ABORTED) {
            printf("Operation Aborted\n");
            break;
        }
        if (ret != ISO14443_3_ERR_SUCCESS) {
            continue;
        }

        printf("ATTRIB RESP <-\n");
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

uint32_t decode_pbm_type(uint8_t sak)
{
    if (sak & 0x08) {
        // Bit 4 == 1
        if (sak & 0x10) {
            // Bit 5 == 1
            return PBM_4K;
        } else {
            // Bit 5 == 0
            if (sak & 0x01) {
                // Bit 1 == 1
                return PBM_MINI;
            } else {
                // Bit 1 == 0
                return PBM_1K;
            }
        }
    } else {
        // Bit 4 == 0
        if (sak & 0x10) {
            // Bit 5 == 1
            if (sak & 0x01) {
                // Bit 1 == 1
                return PBM_PLUS_4K_SL2;
            } else {
                // Bit 1 == 0
                return PBM_PLUS_2K_SL2;
            }
        } else {
            // Bit 5 == 0
            if (sak & 0x20) {
                // Bit 6 == 1
                // This version requires RATS etc
                // Is must therefore be compliant, so we should not be here
                // Do nothing
                return PBM_TYPE_UNKNOWN;
            } else {
                // Bit 6 == 0
                return PBM_UL;
            }
        }
    }
}

// This function assumes a PBM card has been found and identified, and the requested block
// authenticated This function only supports PBM Classic cards
int32_t pbm_create_value_block(uint8_t block_num, uint32_t value)
{
    uint8_t val_blk_buffer[16];

    val_blk_buffer[0] = (value >> 0) & 0xFF;
    val_blk_buffer[1] = (value >> 8) & 0xFF;
    val_blk_buffer[2] = (value >> 16) & 0xFF;
    val_blk_buffer[3] = (value >> 24) & 0xFF;
    val_blk_buffer[4] = ~val_blk_buffer[0];
    val_blk_buffer[5] = ~val_blk_buffer[1];
    val_blk_buffer[6] = ~val_blk_buffer[2];
    val_blk_buffer[7] = ~val_blk_buffer[3];
    val_blk_buffer[8] = val_blk_buffer[0];
    val_blk_buffer[9] = val_blk_buffer[1];
    val_blk_buffer[10] = val_blk_buffer[2];
    val_blk_buffer[11] = val_blk_buffer[3];

    // Address Section
    val_blk_buffer[12] = block_num;
    val_blk_buffer[13] = ~block_num;
    val_blk_buffer[14] = block_num;
    val_blk_buffer[15] = ~block_num;

    return pbm_write_block(block_num, val_blk_buffer);
}

// Decode working time specified by ATQC
void show_response_time(uint8_t PMm_val, int32_t n)
{
    int32_t T_us = 302;
    int32_t A = PMm_val & 0x7;
    int32_t B = (PMm_val >> 3) & 0x7;
    int32_t E = (PMm_val >> 6) & 0x3;
    int32_t time = 0;

    time = T_us * ((B + 1) * n + (A + 1)) * pow(4, E);

    printf("%ldus\n", time);
}

void raw_loop_test(mml_nfc_pcd_transceive_params_t trans_params, int32_t do_reset)
{
    int32_t response = 0;

    int32_t sent = 0;
    int32_t passed = 0;
    int32_t num_loops = 1000;
    int32_t row_passed = 0;

    poweron_operatingfield();
    mml_nfc_pcd_task_sleep(10);

    while (1) {
        if (key_has_been_pressed()) {
            poweroff_operatingfield();
            break;
        }

        response = mml_nfc_pcd_transceive(trans_params);
        sent++;

        if (response == MML_NFC_PCD_E_SUCCESS) {
            printf("o");
            passed++;
            row_passed++;
        } else {
            printf(".");
        }

        if (do_reset) {
            nfc_reset();
        }

        if (sent % 50 == 0) {
            printf("  [ %2ld/ 50] %3ld%%\n", row_passed, ((row_passed * 100) / 50));
            row_passed = 0;
        }

        if (sent >= num_loops) {
            break;
        }
    }

    nfc_reset();
    printf("\n\n(TOTAL) [%4ld/%4ld].... %3ld.%02ld%%\n\n", passed, sent, ((passed * 100) / sent),
        (((passed * 10000) / sent) - (((passed * 100) / sent) * 100)));
    poweroff_operatingfield();
}

void type_a_loop_test(void)
{
    uint8_t send_buffer_wupa[1] = { 0x52 };
    int32_t send_len_wupa = 1;

    uint8_t* receive_buffer = GetCommonBuffer();
    uint32_t receive_len;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_ISO14443A;
    trans_params.frametype = FT_SHORT_NO_CRC_NO_EMD;
    trans_params.tx_buf = send_buffer_wupa;
    trans_params.tx_len = send_len_wupa;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;

    // Loop test resets after every command, so we must enforce our powerup delay (5ms)
    // 5ms / (1/13560000) => 5ms / 73.746ns = 67,800fc
    trans_params.delay_till_send = 67800;

    // Using same timeout as Type F loop test: 200ms instead of the normal ISO14443_FWT_ATQB
    // 200ms / (1/13560000) => 200ms / 73.746ns = 2,712,000fc
    trans_params.timeout = 2712000;
    trans_params.early_limit = ISO14443_FDT_A_EARLY_LIMIT;

    printf("\n\nSending 1000 WUPA, with 200ms timeout\n\n");
    raw_loop_test(trans_params, 1);
}

void type_b_loop_test(void)
{
    uint8_t send_buffer_wupb[3] = { 0x05, 0x00, 0x08 };
    int32_t send_len_wupb = 3;

    uint8_t* receive_buffer = GetCommonBuffer();
    uint32_t receive_len;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_ISO14443B;
    trans_params.frametype = FT_STANDARD_CRC_EMD;
    trans_params.tx_buf = send_buffer_wupb;
    trans_params.tx_len = send_len_wupb;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;

    // Loop test resets after every command, so we must enforce our powerup delay (5ms)
    // 5ms / (1/13560000) => 5ms / 73.746ns = 67,800fc
    trans_params.delay_till_send = 67800;

    // Using same timeout as Type F loop test: 200ms instead of the normal ISO14443_FWT_ATQB
    // 200ms / (1/13560000) => 200ms / 73.746ns = 2,712,000
    trans_params.timeout = 2712000;
    trans_params.early_limit = ISO14443_FDT_B_PICC_MIN;

    printf("\n\nSending 1000 WUPB, with 200ms timeout\n\n");
    raw_loop_test(trans_params, 1);
}

void type_f_loop_test(void)
{
    int32_t reqc_buf_len = 6;
    uint8_t reqc_buf[6] = { 0x06, 0x00, 0xFF, 0xFF, 0x00, 0x00 };

    uint8_t receive_buffer[256];
    uint32_t receive_len = 0;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_TYPE_F;
    trans_params.frametype = FT_STANDARD_CRC_NO_EMD;
    trans_params.tx_buf = reqc_buf;
    trans_params.tx_len = reqc_buf_len;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.delay_till_send = ISO14443_FDT_MIN;

    // ATQC should start in Slot 0, This should happen by the time detailed below.  Currently using
    // Trfw from ISO18092 section 11.1.1, RF Waiting Time, as the margin for detection of ATQC.
    // Alternatively, we could delay till the very end of Slot 0, but this does not seem strict
    // enough. NOTE: RF driver times from end of TX to end of SYNC code, start of LEN byte.  So we
    // need to add this additional time to the timeout.  8 bytes at 64fc per bit.
    //
    // Td = (512 * 64fc = 32768), Time to LEN (T2len) =  (8 * 8 * 64fc = 4096), Trfw = (512fc)
    // Td + T2len + Trfw => 32768 + 4096 + 512 => 37367
    //
    // For RW_RW_Kiteisyo polling loop test, timeout desired is 200ms
    // 200ms / (1/13560000) => 200ms / 73.746ns = 2,712,000
    trans_params.timeout = 2712000;

    // Per Sony Whitepaper: Card Technical Note for Software Development
    //   the Guard Time after transmission of Command Packet Data to when the Reader should
    //   be ready to receive preamble is (42 x 64 - 16)/fc ~197us
    trans_params.early_limit = FDT_F_PICC_MIN_TOLERANCE_EARLY;

    printf("\n\nSending 1000 REQC, with 200ms timeout\n\n");
    raw_loop_test(trans_params, 0);
}

#define ATQC_LEN 18
#define ATQC_LEN_OFFSET 0
#define ATQC_COMMAND_CODE 1
#define ATQC_COMMAND_CODE_OFFSET 1
#define ATQC_NFCID2_OFFSET 2
#define ATQC_NFCID2_LEN 8
#define ATQC_PAD_OFFSET 10
#define ATQC_PAD_LEN 8
#define ATQC_ID_COD_BYTE_1_OFFSET 10
#define ATQC_ID_COD_BYTE_2_OFFSET 11
#define ATQC_RESPONSE_TIME_OFFSET 12

void type_f_commands(void)
{
    int32_t response = 0;

    int32_t reqc_buf_len = 6;
    uint8_t reqc_buf[6] = { 0x06, 0x00, 0xFF, 0xFF, 0x00, 0x00 };

    uint8_t receive_buffer[256];
    uint32_t receive_len = 0;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_TYPE_F;
    trans_params.frametype = FT_STANDARD_CRC_NO_EMD;
    trans_params.tx_buf = reqc_buf;
    trans_params.tx_len = reqc_buf_len;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.delay_till_send = ISO14443_FDT_MIN;

    // ATQC should start in Slot 0, This should happen by the time detailed below.  Currently using
    // Trfw from ISO18092 section 11.1.1, RF Waiting Time, as the margin for detection of ATQC.
    // Alternatively, we could delay till the very end of Slot 0, but this does not seem strict
    // enough. NOTE: RF driver times from end of TX to end of SYNC code, start of LEN byte.  So we
    // need to add this additional time to the timeout.  8 bytes at 64fc per bit.
    //
    // Td = (512 * 64fc = 32768), Time to LEN (T2len) =  (8 * 8 * 64fc = 4096), Trfw = (512fc)
    // Td + T2len + Trfw => 32768 + 4096 + 512 => 37367
    trans_params.timeout = 37367;

    // Per Sony Whitepaper: Card Technical Note for Software Development
    //   the Guard Time after transmission of Command Packet Data to when the Reader should
    //   be ready to receive preamble is (42 x 64 - 16)/fc ~197us
    trans_params.early_limit = FDT_F_PICC_MIN_TOLERANCE_EARLY;

    while (1) {
        clear_screen();

        printf("Type F Commands\n");
        printf("1. Poll for Type F Card (REQC)\n");
        printf("2. Loop Test for Type F Card (REQC)\n");
        printf("E. Exit to previous menu\n");
        printf("M. Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
        case '1':
            poweron_operatingfield();
            // Allow card to wake up
            mml_nfc_pcd_task_sleep(10);

            response = mml_nfc_pcd_transceive(trans_params);
            printf("\n\n");

            if (response == MML_NFC_PCD_E_SUCCESS) {
                printf("Got a response, length in bytes: %ld\n", receive_len);

                hexdump(DBG_LVL_NON, receive_buffer, receive_len, 1);

                printf("ATQC breakdown:\n\n");

                if (receive_buffer[ATQC_LEN_OFFSET] != ATQC_LEN) {
                    printf("ATQC length invalid, expected: %d got: %d\n", ATQC_LEN,
                        receive_buffer[ATQC_LEN_OFFSET]);
                } else {
                    printf("ATQC Length valid\n");
                }

                if (receive_buffer[ATQC_COMMAND_CODE_OFFSET] != ATQC_COMMAND_CODE) {
                    printf("ATQC Command Code invalid, expected: %d got: %d\n", ATQC_COMMAND_CODE,
                        receive_buffer[ATQC_COMMAND_CODE_OFFSET]);
                } else {
                    printf("ATQC Command Code valid\n");
                }

                printf("NFCID2: ");
                hexdump(DBG_LVL_NON, &receive_buffer[ATQC_NFCID2_OFFSET], ATQC_NFCID2_LEN, 1);

                printf("PAD (PMm): ");
                hexdump(DBG_LVL_NON, &receive_buffer[ATQC_PAD_OFFSET], ATQC_PAD_LEN, 1);

                printf("    IC Code: 0x%02X%02X\n", receive_buffer[ATQC_ID_COD_BYTE_1_OFFSET],
                    receive_buffer[ATQC_ID_COD_BYTE_2_OFFSET]);
                printf("Request Service Command (Per Service [n]): ");
                show_response_time(receive_buffer[ATQC_RESPONSE_TIME_OFFSET], 1);
            } // if (response == MML_NFC_PCD_E_SUCCESS)
            else {
                printf("Invalid Response: ");

                if (response == MML_NFC_PCD_E_TIMEOUT) {
                    printf("Timeout (No card found)\n");
                } else if (response == MML_NFC_PCD_E_COLLISION) {
                    printf("Collision Error\n");
                } else if (response == MML_NFC_PCD_E_PROTOCOL) {
                    printf("Protocol Error\n");
                } else if (response == MML_NFC_PCD_E_BAD_PARAM) {
                    printf("Bad Parameter\n");
                } else {
                    printf("Un-Decoded error code: %ld\n", response);
                }
            }
            poweroff_operatingfield();
            printf("Press any key to continue...\n");
            get_key_press();
            break;
        case '2':
            type_f_loop_test();
            printf("Press any key to continue...\n");
            get_key_press();
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
        case 'E':
        case 'e':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press();
            break;
        }
    }
}

void type_v_inventory_test(mml_nfc_pcd_transceive_params_t trans_params)
{
    int32_t response = 0;

    uint8_t v_uid[8];

    poweron_operatingfield();
    // Allow card to wake up
    mml_nfc_pcd_task_sleep(20);

    response = mml_nfc_pcd_transceive(trans_params);
    printf("\n\n");

    if (response == MML_NFC_PCD_E_SUCCESS) {
        if (*trans_params.rx_len == 10) {
            printf("Got a response, length in bytes: %ld\n", *trans_params.rx_len);

            hexdump(DBG_LVL_NON, trans_params.rx_buf, *trans_params.rx_len, 1);

            if (trans_params.rx_buf[0] & 0x1) {
                printf("Card Reported Error\n");
                poweroff_operatingfield();
                printf("Press any key to continue...\n");
                get_key_press();
                return;
            }
            if (trans_params.rx_buf[0] & 0xFE) {
                printf("Unsupported Flags\n");
                poweroff_operatingfield();
                printf("Press any key to continue...\n");
                get_key_press();
                return;
            }
            printf("DSFID: 0x%02x\n", trans_params.rx_buf[1]);
            printf("UID Is : ");
            for (int i = 0; i < 8; i++) {
                v_uid[i] = trans_params.rx_buf[i + 2];
                printf("0x%02x ", v_uid[i]);
            }

            printf("\n");
        } else {
            printf("Apparently, we have success. However, expect 10 bytes but only got: %d\n",
                *trans_params.rx_len);
            printf("Received data");
            hexdump(DBG_LVL_NON, trans_params.rx_buf, *trans_params.rx_len, 1);
        }

    } else {
        printf("Invalid Response: ");

        if (response == MML_NFC_PCD_E_TIMEOUT) {
            printf("Timeout (No card found)\n");
        } else if (response == MML_NFC_PCD_E_COLLISION) {
            printf("Collision Error\n");
        } else if (response == MML_NFC_PCD_E_PROTOCOL) {
            printf("Protocol Error\n");
        } else if (response == MML_NFC_PCD_E_BAD_PARAM) {
            printf("Bad Parameter\n");
        } else if (response == MML_NFC_PCD_E_INVALID_CRC) {
            printf("Bad CRC\n");
        } else {
            printf("Un-Decoded error code: %ld\n", response);
        }
    }
    poweroff_operatingfield();
    printf("Press any key to continue...\n");
    get_key_press();
}

void type_v_commands(void)
{
    int32_t inventory_buf_len = 3;
    uint8_t inventory_buf[3] = { 0x26, 0x01, 0x00 }; // Flags, Inventory OpCode, Optional AFI

    uint8_t receive_buffer[256];
    uint32_t receive_len = 0;

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.frametype = FT_STANDARD_CRC_NO_EMD;
    trans_params.tx_buf = inventory_buf;
    trans_params.tx_len = inventory_buf_len;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.delay_till_send = ISO14443_FDT_MIN;
    trans_params.early_limit = ISO15693_FDT_VICC_MIN;
    trans_params.timeout = ISO15693_FWT_ACTIVATION;

    while (1) {
        clear_screen();

        printf("Type V Commands\n");
        printf("1. Inventory 100% Mod, 1of4, Single Sub, High Data Rate\n");
        printf("2. Inventory 100% Mod, 1of4, Single Sub, Low Data Rate\n");
        printf("3. Inventory 100% Mod, 1of4, Dual Sub, High Data Rate\n");
        printf("4. Inventory 100% Mod, 1of4, Dual Sub, Low Data Rate\n");
        printf("\n");
        printf("5. Inventory 100% Mod, 1of256, Single Sub, High Data Rate\n");
        printf("6. Inventory 100% Mod, 1of256, Single Sub, Low Data Rate\n");
        printf("7. Inventory 100% Mod, 1of256, Dual Sub, High Data Rate\n");
        printf("8. Inventory 100% Mod, 1of256, Dual Sub, Low Data Rate\n");

        printf("\n\n");

        printf("9. Inventory 10% Mod, 1of4, Single Sub, High Data Rate\n");
        printf("A. Inventory 10% Mod, 1of4, Single Sub, Low Data Rate\n");
        printf("B. Inventory 10% Mod, 1of4, Dual Sub, High Data Rate\n");
        printf("C. Inventory 10% Mod, 1of4, Dual Sub, Low Data Rate\n");
        printf("\n");
        printf("D. Inventory 10% Mod, 1of256, Single Sub, High Data Rate\n");
        printf("E. Inventory 10% Mod, 1of256, Single Sub, Low Data Rate\n");
        printf("F. Inventory 10% Mod, 1of256, Dual Sub, High Data Rate\n");
        printf("G. Inventory 10% Mod, 1of256, Dual Sub, Low Data Rate\n");

        printf("\n");

        printf("L. Loop Test for Type V Card (Inventory)\n");
        printf("T. CRC Test\n");
        printf("M. Maxim DTE settings\n");
        printf("X. Exit to previous menu\n");
        printf("=> ");

        switch (get_key_press()) {
        case '1':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_SINGLE_HIGH;
            inventory_buf[0] = 0x26; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '2':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_SINGLE_LOW;
            inventory_buf[0] = 0x24; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '3':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_DUAL_HIGH;
            inventory_buf[0] = 0x27; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '4':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF4_DUAL_LOW;
            inventory_buf[0] = 0x25; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '5':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_SINGLE_HIGH;
            inventory_buf[0] = 0x26; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '6':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_SINGLE_LOW;
            inventory_buf[0] = 0x24; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '7':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_DUAL_HIGH;
            inventory_buf[0] = 0x27; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case '8':
            trans_params.protocol = PROTOCOL_ISO15693_100_1OF256_DUAL_LOW;
            inventory_buf[0] = 0x25; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;

        case '9':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_SINGLE_HIGH;
            inventory_buf[0] = 0x26; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'A':
        case 'a':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_SINGLE_LOW;
            inventory_buf[0] = 0x24; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'B':
        case 'b':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_DUAL_HIGH;
            inventory_buf[0] = 0x27; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'C':
        case 'c':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF4_DUAL_LOW;
            inventory_buf[0] = 0x25; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'D':
        case 'd':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_SINGLE_HIGH;
            inventory_buf[0] = 0x26; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'E':
        case 'e':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_SINGLE_LOW;
            inventory_buf[0] = 0x24; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'F':
        case 'f':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_DUAL_HIGH;
            inventory_buf[0] = 0x27; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'G':
        case 'g':
            trans_params.protocol = PROTOCOL_ISO15693_10_1OF256_DUAL_LOW;
            inventory_buf[0] = 0x25; // Flags
            inventory_buf[1] = 0x01; // Inventory OpCode
            inventory_buf[2] = 0x00; // AFI
            type_v_inventory_test(trans_params);
            break;
        case 'L':
        case 'l':
            // TODO: Implement
            //            type_v_loop_test();
            printf("TBD....\n");
            printf("Press any key to continue...\n");
            get_key_press();
            break;
        case 'T':
        case 't':
            printf("\nCRC Test\n");
            uint8_t crc_test_buf[4] = { 0x01, 0x02, 0x03, 0x04 };
            uint8_t crc1 = 0;
            uint8_t crc2 = 0;
            mml_nfc_pcd_compute_crc(
                PROTOCOL_ISO15693_100_1OF4_SINGLE_HIGH, crc_test_buf, 4, &crc2, &crc1);
            printf("Input: 1,2,3,4\n");
            printf("CRC: 0x%0X%0X\n", crc1, crc2);
            printf("Press any key to continue...\n");
            get_key_press();
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
        case 'X':
        case 'x':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press();
            break;
        }
    }
}

void pbm_commands(void)
{
    int32_t i = 0;
    int32_t uid_start = 0;
    int32_t uid_end = 0;
    int32_t status = 0;
    uid_storage_t uid_store = { { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, 0 };
    uint32_t uid = 0;
    // Default PBM Key
    uint64_t current_key = 0xFFFFFFFFFFFFll;
    uint64_t current_key_b = 0x000000000000ll;
    int32_t new_val = 0;
    uint8_t current_block_num = 12;
    uint32_t current_value_num = 0x100;
    uint8_t data_buffer[64];
    uint32_t pbm_type = 0;

    while (1) {
        clear_screen();

        printf("PBM Command Menu\n\n");
        printf("Current UID: 0x%08lX, Current Block: %d, INC/DEC Value: %ld\n", uid,
            current_block_num, current_value_num);
        printf("\nPOLLING AND AUTHENTICATE COMMANDS\n");
        printf("(P) Poll for PBM Cards          (H) Halt\n");
        printf("(A) Authenticate Block          (X) Re-Authenticate Block\n");
        printf("\nGENERIC BLOCK COMMANDS [%d]\n", current_block_num);
        printf("(B) Change Current Block        (F) Erase Block (Fill with 0x00s)\n");
        printf("(R) Read Block                  (W) Write Block\n");
        printf("\nVALUE BLOCK COMMANDS\n");
        printf(
            "(V) Create Value Block          (C) Change Value amount [%ld]\n", current_value_num);
        printf("(D) Decrement Value Block       (I) Increment Value Block\n");
        printf("(S) Restore Value Block         (T) Transfer Value Block\n");
        printf("\nKEY COMMANDS\n");
        printf("(K) Write trailer to use KEY_B  (L) Authenticate with B\n");
        printf("(M) Re-Authenticate with B\n");
        printf("\n(E) Exit back to main menu\n");
        printf("(Z) Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
        // Polling
        case 'P':
        case 'p':
            // This will poll for a card and store the UID for later use
            printf("\nPBM Polling\n");
            printf("\nTest will loop forever until a card is found,\n");
            printf("Or any key is pressed to stop test.\n\n");

            uid = 0;
            status = 0;

            poweron_operatingfield();
            while (status != TYPE_A_NON_ISO14443_4_READY) {
                status = emvl1_poll_for_card(1);

                if (key_has_been_pressed()) {
                    nfc_pcd_reset_wait();
                    printf("\nStopping Test\n");
                    break;
                }
            }

            if (status == TYPE_A_NON_ISO14443_4_READY) {
                // NOTE: The stored UID contains the BCC as well
                uid_store = get_stored_uid();

                // Is this a valid UID
                if (uid_store.uid_length == 5) {
                    uid_start = 0;
                    uid_end = 4;
                } else if (uid_store.uid_length == 10) {
                    uid_start = 5;
                    uid_end = 9;
                } else if (uid_store.uid_length == 15) {
                    uid_start = 10;
                    uid_end = 14;
                } else {
                    printf("Invalid UID detected, should be length of 5, 10, or 15 including "
                           "the BCC\n");
                    break;
                }

                // Convert UID to a uint32_t
                for (i = uid_start; i < uid_end; i++) {
                    uid <<= 8;
                    uid |= uid_store.uid[i];
                }
                printf("\nFound potential Transport Classic card\n");

                pbm_type = decode_pbm_type(get_last_sak());

                if (pbm_type == PBM_UL) {
                    printf("Found Transport Classic UL\n");
                } else if (pbm_type == PBM_MINI) {
                    printf("Found Transport Classic MINI\n");
                } else if (pbm_type == PBM_1K) {
                    printf("Found Transport Classic 1K\n");
                } else if (pbm_type == PBM_4K) {
                    printf("Found Transport Classic 4K\n");
                } else if (pbm_type == PBM_PLUS_2K_SL2) {
                    printf("Found Transport Classic PLUS 2K SL2\n");
                } else if (pbm_type == PBM_PLUS_4K_SL2) {
                    printf("Found Transport Classic PLUS 4K SL2\n");
                }
            } else {
                printf("No potential cards found\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'A':
        case 'a':
            // once a card has been polled, we can authenticate for a block with it's key,
            // which gets us into the encrypted communication process so we can do other commands
            // this uses current_block_num (set by 'B') and the current_key
            printf("\nBlock Authenticate\n");

            if (uid == 0) {
                printf("\nUID is 0, Poll for a valid card first\n");
                printf("Press any key to continue...\n");
                get_key_press_no_echo();
                break;
            }

            status = pbm_authenticate_block(
                uid, current_block_num, current_key, PBM_KEY_A, FIRST_TIME_AUTH);

            if (status != PBM_SUCCESS) {
                printf("Failed to authenticate. Error: %ld\n", status);
            } else {
                printf("Authentication for block %d successful.\n", current_block_num);
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'B':
        case 'b':
            // switch to another block on the card, this only changes the block we are pointing at
            // NOTE: you need to re-authenticate to actually send commands to the new block
            printf("\nCurrent KEY: 0x%012llX\n", current_key);
            printf("\nEnter new block to use\n");

            new_val = dte_get_int_val("Pbm Block", 3, 0, 256);

            if ((new_val < 0) || (new_val > 256)) {
                printf("\nInvalid selection try again.\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
            } else {
                current_block_num = (uint8_t)new_val;
            }
            break;
        case 'R':
        case 'r':
            // read the data for a block, this requires that you are authenticated
            // this uses current_block_num (set by 'B')
            printf("\nRead Block: %d\n", current_block_num);

            status = pbm_read_block(current_block_num, data_buffer);

            if (status != PBM_SUCCESS) {
                printf("Failed to read block. Error: %ld\n", status);
            } else {
                printf("Successful read block. Data:");

                hexdump(DBG_LVL_NON, data_buffer, 16, 1);
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'W':
        case 'w':
            // write data to a block, this requires that you are authenticated
            // this uses current_block_num (set by 'B')
            printf("\nWrite Block: %d\n", current_block_num);

            for (i = 0; i < 8; i++) { data_buffer[i] = 0; }

            data_buffer[8] = 0xDE;
            data_buffer[9] = 0xAD;
            data_buffer[10] = 0xBE;
            data_buffer[11] = 0xEF;
            data_buffer[12] = 0x01;
            data_buffer[13] = 0x23;
            data_buffer[14] = 0x45;
            data_buffer[15] = 0x67;

            status = pbm_write_block(current_block_num, data_buffer);

            if (status != PBM_SUCCESS) {
                printf("Failed to Write block. Error: %ld\n", status);
            } else {
                printf("Successfully wrote block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'H':
        case 'h':
            // send the HALT command to a card, it should ignore us until we re-poll it
            printf("\nHALT\n");

            status = pbm_halt();

            if (status != PBM_SUCCESS) {
                printf("Failed to Halt. Error: %ld\n", status);
            } else {
                printf("Successfully Halted.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'V':
        case 'v':
            // create a formatted value block, uses PBM formatting
            // NOTE: you need to be authenticated for the block to use this command
            // this uses the current_block_num (set by 'B') and the current_value_num (set by 'C')
            printf("\nCreate Value Block\n");

            status = pbm_create_value_block(current_block_num, current_value_num);

            if (status != PBM_SUCCESS) {
                printf("Failed to create value block. Error: %ld\n", status);
            } else {
                printf("Successfully Created Value Block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'X':
        case 'x':
            // re-authenticate to a new block, requires the correct key to be in 'current_key'
            // the uses current_block_num (set by 'B')
            printf("\nBlock Re-Authenticate\n");

            if (uid == 0) {
                printf("\nUID is 0, Poll for a valid card first\n");
                printf("Press any key to continue...\n");
                get_key_press_no_echo();
                break;
            }

            status = pbm_authenticate_block(
                uid, current_block_num, current_key, PBM_KEY_A, ALREADY_AUTHORIZED_AUTH);

            if (status != PBM_SUCCESS) {
                printf("Failed to re-authenticate. Error: %ld\n", status);
            } else {
                printf("Re-Authentication for block %d successful.\n", current_block_num);
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'D':
        case 'd':
            // send the decrement command for the current block, requires that you are
            // authenticated. this uses the current_block_num (set by 'B') and the current_value_num
            // (set by 'C')
            printf("\nDecrement Value Block\n");

            status = pbm_decrement_block(current_block_num, current_value_num);

            if (status != PBM_SUCCESS) {
                printf("Failed to decrement block. Error: %ld\n", status);
            } else {
                printf("Successfully decremented Block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'I':
        case 'i':
            // send the increment command for the current block, requires that you are
            // authenticated. this uses the current_block_num (set by 'B') and the current_value_num
            // (set by 'C')
            printf("\nIncrement Value Block\n");

            status = pbm_increment_block(current_block_num, current_value_num);

            if (status != PBM_SUCCESS) {
                printf("Failed to increment block. Error: %ld\n", status);
            } else {
                printf("Successfully incremented Block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'S':
        case 's':
            // send the restore command for the current block, requires that you are authenticated
            // this uses current_block_num (set by 'B')
            printf("\nRestore Value Block\n");

            status = pbm_restore_block(current_block_num);

            if (status != PBM_SUCCESS) {
                printf("Failed to restore block. Error: %ld\n", status);
            } else {
                printf("Successfully restored Block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'T':
        case 't':
            // send the transfer command for the current block, requires that you are authenticated
            // this uses current_block_num (set by 'B')
            printf("\nTransfer Value Block\n");

            status = pbm_transfer_block(current_block_num);

            if (status != PBM_SUCCESS) {
                printf("Failed to transfer block. Error: %ld\n", status);
            } else {
                printf("Successfully transfered Block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'C':
        case 'c':
            // change the value (current_value_num) used by the value block command for increment,
            // decrement
            printf("\nChange Value\n");
            printf("\nCurrent Value: %ld\n", current_value_num);

            new_val = dte_get_int_val("Pbm Value", 10, 0, 0x7FFFFFF);

            if ((new_val < 0) || (new_val > 0x7FFFFFFF)) {
                printf("\nInvalid selection try again.\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
            } else {
                current_value_num = new_val;
            }

            break;
        case 'K':
        case 'k':
            printf("\nWrite Trailer for Key B: %d\n", current_block_num);

            for (i = 0; i < 8; i++) { data_buffer[i] = 0; }

            // Key A
            data_buffer[0] = (current_key >> 40) & 0xFF;
            data_buffer[1] = (current_key >> 32) & 0xFF;
            data_buffer[2] = (current_key >> 24) & 0xFF;
            data_buffer[3] = (current_key >> 16) & 0xFF;
            data_buffer[4] = (current_key >> 8) & 0xFF;
            data_buffer[5] = (current_key >> 0) & 0xFF;

            // KEY A will remain the only key to write trailer C1-3 of b001
            // DISALLOWED.  If B can be read it CANNOT serve for auth.  Auth will pass but all
            // memory accesses will fail
            //  So, we need to use another mode, so use b100, Key B can write both a and b, but A
            //  and B can write Access
            // KEY B will have SOLE Write permission for block 2 of current sector (where trailer is
            // block 3) C1-3 of b100 C10, C11, C20, C21, C30, C31 will remain 0, so Transport
            // Configuration (Key A or B has full access) /C2      /C1
            //  C1      /C3
            //  C3       C2
            // 7654     3210    BITS
            // 1111     0011    0xF3
            // 1100     1111    0xCF
            // 0000     0000    0x00
            //
#define KEYB_TEST
#ifdef KEYB_TEST
            data_buffer[6] = 0xF3;
            data_buffer[7] = 0xCF;
            data_buffer[8] = 0x00;
#else
            data_buffer[6] = 0xFF;
            data_buffer[7] = 0x07;
            data_buffer[8] = 0x80;
#endif

            data_buffer[9] = 0xAD; // User data

            // Key B
            data_buffer[10] = (current_key_b >> 40) & 0xFF;
            data_buffer[11] = (current_key_b >> 32) & 0xFF;
            data_buffer[12] = (current_key_b >> 24) & 0xFF;
            data_buffer[13] = (current_key_b >> 16) & 0xFF;
            data_buffer[14] = (current_key_b >> 8) & 0xFF;
            data_buffer[15] = (current_key_b >> 0) & 0xFF;

            status = pbm_write_block(current_block_num, data_buffer);

            if (status != PBM_SUCCESS) {
                printf("Failed to Write block. Error: %ld\n", status);
            } else {
                printf("Successfully wrote block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'L':
        case 'l':
            // perform block authentication using key B instead of key A
            printf("\nBlock Authenticate with B\n");

            if (uid == 0) {
                printf("\nUID is 0, Poll for a valid card first\n");
                printf("Press any key to continue...\n");
                get_key_press_no_echo();
                break;
            }

            status = pbm_authenticate_block(
                uid, current_block_num, current_key_b, PBM_KEY_B, FIRST_TIME_AUTH);

            if (status != PBM_SUCCESS) {
                printf("Failed to authenticate. Error: %ld\n", status);
            } else {
                printf("Authentication for block %d successful.\n", current_block_num);
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'M':
        case 'm':
            // re-authenticate using key B
            printf("\nBlock Re-Authenticate Key B\n");

            if (uid == 0) {
                printf("\nUID is 0, Poll for a valid card first\n");
                printf("Press any key to continue...\n");
                get_key_press_no_echo();
                break;
            }

            status = pbm_authenticate_block(
                uid, current_block_num, current_key_b, PBM_KEY_B, ALREADY_AUTHORIZED_AUTH);

            if (status != PBM_SUCCESS) {
                printf("Failed to re-authenticate. Error: %ld\n", status);
            } else {
                printf("Re-Authentication for block %d successful.\n", current_block_num);
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'F':
        case 'f':
            // erase a block by filling with all 0
            printf("\nErase Block (Fill with 0x00s)\n: %d\n", current_block_num);

            for (i = 0; i < 16; i++) { data_buffer[i] = 0; }

            status = pbm_write_block(current_block_num, data_buffer);

            if (status != PBM_SUCCESS) {
                printf("Failed to Erase block. Error: %ld\n", status);
            } else {
                printf("Successfully Erased block.\n");
            }

            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'Z':
        case 'z':
            // go to dte settings menu
            dte_settings();
            break;
            // Exit
        case 'E':
        case 'e':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

int32_t singleemvl1transac_a(callback_check_for_loop_termination_t callback)
{
    uint8_t send_buffer_wupa[1] = { 0x52 };
    int32_t send_len_wupa = 1;
    uint8_t send_buffer_hlta[2] = { 0x50, 0x00 };
    int32_t send_len_hlta = 2;
    uint8_t send_buffer_wupb[3] = { 0x05, 0x00, 0x08 };
    int32_t send_len_wupb = 3;
    uint8_t send_buffer_anticol[2] = { 0x93, 0x20 };
    int32_t send_len_anticol = 2;
    uint8_t send_buffer_sel[7] = { 0x93, 0x70, 0x27, 0xe9, 0x3b, 0x11, 0xe4 };
    int32_t send_len_sel = 7;
    uint8_t send_buffer_rats[2] = { 0xe0, 0x80 };
    int32_t send_len_rats = 2;
    uint8_t send_buffer_iblock1[21] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };
    int32_t send_len_iblock1 = 21;
    uint8_t send_buffer_iblock2[19] = { 0x03, 0x00, 0xA4, 0x04, 0x00, 0x0C, 0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x00 };
    int32_t send_len_iblock2 = 19;

    uint8_t* receive_buffer = GetCommonBuffer();
    uint32_t receive_len;

    // If callback for early termination exists, call it
    if (callback) {
        if (callback()) {
            mml_nfc_pcd_task_sleep(KEYPRESS_RETURN_DELAY_MS);
            logging("\nStopping Test\n");
            return 1;
        }
    }

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_ISO14443A;
    trans_params.frametype = FT_SHORT_NO_CRC_NO_EMD;
    trans_params.tx_buf = send_buffer_wupa;
    trans_params.tx_len = send_len_wupa;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.delay_till_send = ISO14443_FDT_MIN;
    trans_params.timeout = ISO14443_FWT_A_ACT;
    trans_params.early_limit = ISO14443_FDT_A_EARLY_LIMIT;

    // WUPA
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // HALTA
    trans_params.tx_buf = send_buffer_hlta;
    trans_params.tx_len = send_len_hlta;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // WUPB
    trans_params.protocol = PROTOCOL_ISO14443B;
    trans_params.frametype = FT_STANDARD_CRC_EMD;
    trans_params.tx_buf = send_buffer_wupb;
    trans_params.tx_len = send_len_wupb;
    trans_params.timeout = ISO14443_FWT_ATQB;
    trans_params.early_limit = ISO14443_FDT_B_PICC_MIN;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // WUPA
    trans_params.protocol = PROTOCOL_ISO14443A;
    trans_params.frametype = FT_SHORT_NO_CRC_NO_EMD;
    trans_params.tx_buf = send_buffer_wupa;
    trans_params.tx_len = send_len_wupa;
    trans_params.timeout = ISO14443_FWT_A_ACT;
    trans_params.early_limit = ISO14443_FDT_A_EARLY_LIMIT;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Anticollision
    trans_params.frametype = FT_STANDARD_NO_CRC_NO_EMD;
    trans_params.tx_buf = send_buffer_anticol;
    trans_params.tx_len = send_len_anticol;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Select
    trans_params.frametype = FT_STANDARD_CRC_EMD;
    trans_params.tx_buf = send_buffer_sel;
    trans_params.tx_len = send_len_sel;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // RATS
    trans_params.tx_buf = send_buffer_rats;
    trans_params.tx_len = send_len_rats;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Iblock 1
    trans_params.tx_buf = send_buffer_iblock1;
    trans_params.tx_len = send_len_iblock1;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Iblock 2
    trans_params.tx_buf = send_buffer_iblock2;
    trans_params.tx_len = send_len_iblock2;
    mml_nfc_pcd_transceive(trans_params);

    return 0;
}

int32_t singleemvl1transac_b(callback_check_for_loop_termination_t callback)
{
    uint8_t send_buffer_wupa[1] = { 0x52 };
    int32_t send_len_wupa = 1;
    uint8_t send_buffer_wupb[3] = { 0x05, 0x00, 0x08 };
    int32_t send_len_wupb = 3;
    uint8_t send_buffer_attrib[9] = { 0x1D, 0x46, 0xB5, 0xC7, 0xA0, 0x00, 0x08, 0x01, 0x00 };
    int32_t send_len_attrib = 9;
    uint8_t send_buffer_iblock1[21] = { 0x02, 0x00, 0xA4, 0x04, 0x00, 0x0E, 0x32, 0x50, 0x41, 0x59,
        0x2E, 0x53, 0x59, 0x53, 0x2E, 0x44, 0x44, 0x46, 0x30, 0x31, 0x00 };
    int32_t send_len_iblock1 = 21;
    uint8_t send_buffer_iblock2[19] = { 0x03, 0x00, 0xA4, 0x04, 0x00, 0x0C, 0x01, 0x02, 0x03, 0x04,
        0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x00 };
    int32_t send_len_iblock2 = 19;

    uint8_t* receive_buffer = GetCommonBuffer();
    uint32_t receive_len;

    // If callback for early termination exists, call it
    if (callback) {
        if (callback()) {
            mml_nfc_pcd_task_sleep(KEYPRESS_RETURN_DELAY_MS);
            logging("\nStopping Test\n");
            return 1;
        }
    }

    mml_nfc_pcd_transceive_params_t trans_params;

    trans_params.protocol = PROTOCOL_ISO14443B;
    trans_params.frametype = FT_STANDARD_CRC_EMD;
    trans_params.tx_buf = send_buffer_wupb;
    trans_params.tx_len = send_len_wupb;
    trans_params.rx_buf = receive_buffer;
    trans_params.rx_len = &receive_len;
    trans_params.delay_till_send = ISO14443_FDT_MIN;
    trans_params.timeout = ISO14443_FWT_ATQB;
    trans_params.early_limit = ISO14443_FDT_B_PICC_MIN;

    // WUPB
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // WUPA
    trans_params.protocol = PROTOCOL_ISO14443A;
    trans_params.frametype = FT_SHORT_NO_CRC_NO_EMD;
    trans_params.tx_buf = send_buffer_wupa;
    trans_params.tx_len = send_len_wupa;
    trans_params.timeout = ISO14443_FWT_A_ACT;
    trans_params.early_limit = ISO14443_FDT_A_EARLY_LIMIT;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // WUPB
    trans_params.protocol = PROTOCOL_ISO14443B;
    trans_params.frametype = FT_STANDARD_CRC_EMD;
    trans_params.tx_buf = send_buffer_wupb;
    trans_params.tx_len = send_len_wupb;
    trans_params.timeout = ISO14443_FWT_ATQB;
    trans_params.early_limit = ISO14443_FDT_B_PICC_MIN;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Attrib
    trans_params.tx_buf = send_buffer_attrib;
    trans_params.tx_len = send_len_attrib;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Iblock 1
    trans_params.tx_buf = send_buffer_iblock1;
    trans_params.tx_len = send_len_iblock1;
    mml_nfc_pcd_transceive(trans_params);
    nfc_block_for_us(TIMEOUT_TRANSAC_US);

    // Iblock 2
    trans_params.tx_buf = send_buffer_iblock2;
    trans_params.tx_len = send_len_iblock2;
    mml_nfc_pcd_transceive(trans_params);

    return 0;
}

void dte_do_transac_a(void)
{
    int32_t i = 0;

    printf("\nAnalog TRANSAC_A Mode\n");
    printf("\n!Test will continue to send TRANSAC_A sequence until any key is pressed to stop "
           "test!\n\n");

    while (1) {
        mml_nfc_pcd_task_sleep(PAUSE_TRANSAC_MS);
        printf("\nTransaction A Send Count: %ld\n", i++);

        if (singleemvl1transac_a(key_has_been_pressed)) {
            break;
        }
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_do_transac_b(void)
{
    int32_t i = 0;

    printf("\nAnalog TRANSAC_B Mode\n");
    printf("\n!Test will continue to send TRANSAC_B sequence until any key is pressed to stop "
           "test!\n\n");

    while (1) {
        mml_nfc_pcd_task_sleep(PAUSE_TRANSAC_MS);
        printf("\nTransaction B Send Count: %ld\n", i++);

        if (singleemvl1transac_b(key_has_been_pressed)) {
            break;
        }
    }
    printf("Press any key to continue...\n");
    get_key_press_no_echo();
}

void dte_analogue(void)
{
    int32_t carrier_on = 0;
    int32_t num_loops = 0;

    while (1) {
        clear_screen();
        printf("TTA L1: Analogue Menu\n");
        printf("Loop Setting: Num Loops: %ld\n", num_loops);
        printf("\n! NOTE: by Default Num Loops is 0, Which is Infinite Looping !\n\n");
        printf("Halt test sequences by pressing any key\n");
        printf("Carrier Status: ");
        if (carrier_on) {
            printf("On\n");
        } else {
            printf("Off\n");
        }
        printf("\nSelect Test to continue:\n");
        printf("1. Carrier On/Off\n");
        printf("2. Polling\n");
        printf("3. Reset\n");
        printf("4. WUPA\n");
        printf("5. WUPB\n");
        printf("6. RATS\n");
        printf("7. ATTRIB\n");
        printf("8. TRANSAC_A\n");
        printf("9. TRANSAC_B\n");
        printf("Y. WUPA LOOP TEST\n");
        printf("T. WUPB LOOP TEST\n");
        printf("L. Change Loop Settings\n");
        printf("E. Exit Analogue Tests\n");
        printf("M. Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
            // Carrier On/Off
        case '1':
            if (carrier_on) {
                poweroff_operatingfield();
                printf("Carrier OFF\n");
                carrier_on = 0;
            } else {
                poweron_operatingfield();
                printf("Carrier On\n");
                carrier_on = 1;
            }
            break;
            // Polling specified number of loops
        case '2':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_analog_polling(num_loops);
            break;
            // Reset
        case '3':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_reset(num_loops);
            break;
            // WUPA
        case '4':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_wupa(num_loops);
            break;
            // WUPB
        case '5':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_wupb(num_loops);
            break;
            // RATS
        case '6':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_rats(num_loops);
            break;
            // ATTRIB
        case '7':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_attrib(num_loops);
            break;
            // TRANSAC_A
        case '8':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_transac_a();
            break;
            // TRANSAC_B
        case '9':
            if (!carrier_on) {
                printf("\nInvalid Carrier Must be on\n");
                printf("Press any key to retry...\n");
                get_key_press_no_echo();
                break;
            }
            dte_do_transac_b();
            break;
            // WUPA LOOP TEST
        case 'Y':
        case 'y':
            type_a_loop_test();
            break;
            // WUPB LOOP TEST
        case 'T':
        case 't':
            type_b_loop_test();
            break;
            // Change Loop Count
        case 'L':
        case 'l':
            dte_get_loop_params(&num_loops);
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
            // Exit
        case 'E':
        case 'e':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

void dte_digital(void)
{
    int32_t i = 0;

    while (1) {
        clear_screen();

        printf("TTA L1: Digital Menu\n");
        printf("1. Begin Digital Loop Back Mode\n");
        printf("E. Exit back to main menu\n");
        printf("M. Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
            // Polling
        case '1':
            printf("\nDigital Loop Back Mode\n");
            printf("\n!Test will loop forever until any key is pressed to stop test!\n\n");
            poweron_operatingfield();
            while (1) {
                printf("\n\nDig %ld\n", i++);

                if (singleemvl1exchange(key_has_been_pressed)) {
                    break;
                }
            }
            poweroff_operatingfield();
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
            // Exit
        case 'E':
        case 'e':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

void dte_interop(void)
{
    int32_t i = 0;

    while (1) {
        clear_screen();

        printf("TTA L1: Interoperability Testing Loopback Menu\n");
        printf("1. Begin Interoperability Loop Back Mode\n");
        printf("E. Exit back to main menu\n");
        printf("M. Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
            // Polling
        case '1':
            printf("\nInteroperability Loop Back Mode\n");
            printf("\n!Test will loop forever until any key is pressed to stop test!\n\n");
            poweron_operatingfield();
            while (1) {
                printf("\n\nInterop %ld\n", i++);

                if (singleemvl1interopexchange(key_has_been_pressed)) {
                    break;
                }
            }
            poweroff_operatingfield();
            printf("Press any key to continue...\n");
            get_key_press_no_echo();
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
            // Exit
        case 'E':
        case 'e':
            poweroff_operatingfield();
            return;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press_no_echo();
            break;
        }
    }
}

void dte(void)
{
    clear_screen();

    // Setup the abort callback
    set_abort_check_callback(key_has_been_pressed);

    while (1) {
        clear_screen();

        poweron_operatingfield();
        poweroff_operatingfield();

        printf("Maxim Integrated MAX32572 EMV L1 DTE Main Menu\n\n");
        printf("HW Version: %s\n", PCD_HW_VER);
        printf("SW Version: %s\n", PCD_SW_VER);
        printf("FW Checksum: %s\n", PCD_FW_SUM);

        printf("\nSelect Application to continue:\n");
        printf("1. TTA L1: Analogue\n");
        printf("2. TTA L1: Digital\n");
        printf("3. TTA L1: Interoperability Testing Loopback\n");
        printf("P. Transport Classic (PBM)\n");
        printf("F. Type F\n");
        printf("V. Type V\n");
        printf("M. Maxim DTE settings\n");
        printf("=> ");

        switch (get_key_press()) {
        case '1':
            dte_analogue();
            break;
        case '2':
            dte_digital();
            break;
        case '3':
            dte_interop();
            break;
        case 'P':
        case 'p':
            pbm_commands();
            break;
        case 'F':
        case 'f':
            type_f_commands();
            break;
        case 'V':
        case 'v':
            type_v_commands();
            break;
        case 'M':
        case 'm':
            dte_settings();
            break;
        default:
            printf("\nInvalid selection try again.\n");
            printf("Press any key to retry...\n");
            get_key_press();
            break;
        }
    }
}

void emvl1_main_loop(void)
{
    g_logging_level = DBG_LVL_LOG;

    // Disable line buffering for this serial menu driven DTE
    //  avoid may calls to fflush();
    setvbuf(stdout, NULL, _IONBF, 0);

    if (trim_ro_to_rtc() != E_SUCCESS) {
        printf("Failed to trim Ring Oscillator.\n");
        printf("Timing accuracy suspect, may result in communications failures.\n");

        printf("Press any key to continue...\n");
        get_key_press_no_echo();
    }
    // ********************************************************************
    // Set the desired default antenna analog configuration
    // ********************************************************************
    // FIXME: Restore to EVKIT antenna before release:
    // mml_nfc_pcd_analog_parameters_matrix = evkit_antenna_112x76_matrix;
    mml_nfc_pcd_analog_parameters_matrix = custom_antenna_65x65_matrix;

    dte();
}
