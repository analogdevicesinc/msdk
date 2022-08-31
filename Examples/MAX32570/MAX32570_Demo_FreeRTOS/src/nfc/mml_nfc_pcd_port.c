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

#include "mxc_device.h"

#include <emv_l1_stack/iso14443_3_common.h>
#include <logging.h>
#include <mml_nfc_pcd_port.h>

/**
 * @page NFC_EMV_CONTACTLESS_RF_DRIVER_PORT NFC EMV Contactless PCD RF Driver Porting Functions
 *
 * @remarks This file provides various methods required for the NFC PCD RF driver
 * which integrators may wish to modify for specific porting requirements,
 * including RTOS integrations etc.
 *
 * @see @ref APIOverviewAndUsage
 * @see @ref NFC_EMV_CONTACTLESS_RF_DRIVER
 */

// In single process/thread applications, we may set the NFC interrupt to max priority: 0
#define NFC_INTERRUPT_PRIORITY 0
// Typically, the FWT (timeout) timer is set to one less priority from the NFC interrupt
//#define FWT_TIMER_INTERRUPT_PRIORITY (NFC_INTERRUPT_PRIORITY + 1)
#define FWT_TIMER_INTERRUPT_PRIORITY 0

// Fake semaphore used in single process/thread applications
static volatile uint32_t g_fake_semaphore = 0;

// Enable dump of analog settings before setting for the RF driver
#define DUMP_SELECTED_ANALOG_SETTINGS 0

// Dump the FD index for every transceive, if logging level is at least DBG_LVL_ INFO
#define DUMP_FD_INDEX 0

/** Capture array used for Type B SW based receiver */
// Not needed as of now for the MAX32570, size is 0
uint16_t mml_nfc_pcd_maj_sel_capture_data[CAPTURE_DATA_BUFFER_SIZE];

/** Packing buffer used for internal framing */
uint8_t mml_nfc_pcd_packing_buffer[MAX_PACKING_BUFFER_LEN];

/**
 * Default NFC Analog Configuration values
 */
mml_nfc_pcd_analog_params_matrix_t mml_nfc_pcd_analog_parameters_matrix
    = {
          //                                    0                  1                  2 3 4 5 6 7 8
          //                                    9
          .fd_thresholds = { 215, 190, 165, 140, 110, 100, 70, 70, 70, 70 },
          .fd_dyn_trigger_a = { 20, 20, 40, 50, 100, 100, 110, 110, 110, 110 },
          .fd_dyn_math_a = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I,
              IQ_MATH_CH_Q, IQ_MATH_CH_Q, IQ_MATH_CH_Q, IQ_MATH_CH_Q, IQ_MATH_CH_Q },
          .fd_dyn_trigger_b = { 20, 20, 50, 50, 50, 50, 50, 50, 50, 50 },
          .fd_dyn_math_b = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I,
              IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
          .fd_dyn_trigger_f = { 40, 35, 25, 25, 25, 25, 25, 25, 25, 25 },
          .fd_dyn_math_f = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I,
              IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
          .fd_dyn_trigger_v = { 40, 35, 25, 25, 25, 25, 25, 25, 25, 25 },
          .fd_dyn_math_v = { IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I,
              IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I, IQ_MATH_CH_I },
          .fd_dyn_sttm_a = { 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
              0x00000000, 0x00000000, 0x00000000, 0x00000000 },
          .fd_dyn_stfm_a = { 0x7F000000, 0x7F000000, 0x7F000000, 0x7F000000, 0x07020702, 0x07020702,
              0x7F000000, 0x7F000000, 0x7F000000, 0x7F000000 },
          .fd_dyn_sttm_bfv = { 0x0C0C0C0C, 0x0A0A0A0A, 0x09090909, 0x08080808, 0x07070707,
              0x03040506, 0x07070707, 0x07070707, 0x07070707, 0x07070707 },
          .fd_dyn_stfm_bfv = { 0x7F0C0C0C, 0x7F0A0A0A, 0x7F090909, 0x7F080808, 0x7F070707,
              0x06050403, 0x7F050505, 0x7F050505, 0x7F050505, 0x7F050505 },
          .fd_dyn_gain = { 12, 12, 12, 12, 12, 12, 12, 12, 12, 12 },
          .fd_dyn_atten = { 31, 31, 31, 31, 31, 31, 31, 31, 31, 31 },
      };

/**
 * NFC Analog Configuration values to use for current transaction
 */
mml_nfc_pcd_analog_params_t current_analog_parameters = { .trigger_level = 125,
    .sttm = 0x00000000,
    .stfm = 0x7F000000,
    .gain = 12,
    .iq_math = IQ_MATH_Q_MINUS_I,
    .atten = 0x1F };

void mml_nfc_pcd_enter_critical(void)
{
    __disable_irq();
}

void mml_nfc_pcd_leave_critical(void)
{
    __enable_irq();
}

void mml_nfc_pcd_task_sleep(uint32_t delay_ms)
{
    nfc_block_for_us(1000 * (delay_ms));
}

void mml_nfc_pcd_set_nfc_interrupt_priority(int32_t nfc_irq_num)
{
    NVIC_SetPriority(nfc_irq_num, NFC_INTERRUPT_PRIORITY);
}

void mml_nfc_pcd_set_FWT_timer_interrupt_priority(int32_t fwt_timer_irq_num)
{
    NVIC_SetPriority(fwt_timer_irq_num, FWT_TIMER_INTERRUPT_PRIORITY);
}

void mml_nfc_pcd_create_semaphore()
{
    // We are single threaded, no need for a real semaphore
    g_fake_semaphore = 0;
}

void mml_nfc_pcd_take_semaphore()
{
    while (1) {
        if (g_fake_semaphore) {
            // Need to reset our semaphore
            mml_nfc_pcd_enter_critical();
            g_fake_semaphore = 0;
            mml_nfc_pcd_leave_critical();

            return;
        }
    }
}

void mml_nfc_pcd_give_semaphore_from_isr()
{
    g_fake_semaphore = 1;
}

void mml_nfc_pcd_field_level_detection_callback(uint8_t protocol)
{
    uint8_t sensed_threshold = 0;
    uint32_t field_sensed = 0;
    int32_t return_status = 0;

    // Sweep through and identify the requested level
    for (sensed_threshold = 0; sensed_threshold < FD_THRESH_NUM_STEPS;) {
        return_status = mml_nfc_pcd_detect_loading(
            mml_nfc_pcd_analog_parameters_matrix.fd_thresholds[sensed_threshold], &field_sensed);

        if (return_status != MML_NFC_PCD_E_SUCCESS) {
            error("Failed to detect_loading: %d\n", return_status);
        }

        if (field_sensed) {
            break;
        }

        sensed_threshold++;
    }

    if (DUMP_FD_INDEX) {
        info("FD Index: %d\n", sensed_threshold);
    }

    // Bounds check
    if (sensed_threshold > (FD_THRESH_NUM_STEPS - 1)) {
        sensed_threshold = FD_THRESH_NUM_STEPS - 1;
    }

    switch (protocol) {
    case PROTOCOL_ISO14443A:
        current_analog_parameters.trigger_level
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_a[sensed_threshold];
        current_analog_parameters.iq_math
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_a[sensed_threshold];
        current_analog_parameters.sttm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_a[sensed_threshold];
        current_analog_parameters.stfm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_a[sensed_threshold];
        break;

    case PROTOCOL_ISO14443B:
        current_analog_parameters.trigger_level
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_b[sensed_threshold];
        current_analog_parameters.iq_math
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_b[sensed_threshold];
        current_analog_parameters.sttm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_bfv[sensed_threshold];
        current_analog_parameters.stfm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_bfv[sensed_threshold];
        break;

    case PROTOCOL_TYPE_F:
        current_analog_parameters.trigger_level
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_f[sensed_threshold];
        current_analog_parameters.iq_math
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_f[sensed_threshold];
        current_analog_parameters.sttm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_bfv[sensed_threshold];
        current_analog_parameters.stfm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_bfv[sensed_threshold];
        break;

    case PROTOCOL_ISO15693_100_1OF4_SINGLE_HIGH:
    case PROTOCOL_ISO15693_100_1OF4_SINGLE_LOW:
    case PROTOCOL_ISO15693_100_1OF4_DUAL_HIGH:
    case PROTOCOL_ISO15693_100_1OF4_DUAL_LOW:
    case PROTOCOL_ISO15693_100_1OF256_SINGLE_HIGH:
    case PROTOCOL_ISO15693_100_1OF256_SINGLE_LOW:
    case PROTOCOL_ISO15693_100_1OF256_DUAL_HIGH:
    case PROTOCOL_ISO15693_100_1OF256_DUAL_LOW:
    case PROTOCOL_ISO15693_10_1OF4_SINGLE_HIGH:
    case PROTOCOL_ISO15693_10_1OF4_SINGLE_LOW:
    case PROTOCOL_ISO15693_10_1OF4_DUAL_HIGH:
    case PROTOCOL_ISO15693_10_1OF4_DUAL_LOW:
    case PROTOCOL_ISO15693_10_1OF256_SINGLE_HIGH:
    case PROTOCOL_ISO15693_10_1OF256_SINGLE_LOW:
    case PROTOCOL_ISO15693_10_1OF256_DUAL_HIGH:
    case PROTOCOL_ISO15693_10_1OF256_DUAL_LOW:
        current_analog_parameters.trigger_level
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_v[sensed_threshold];
        current_analog_parameters.iq_math
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_v[sensed_threshold];
        current_analog_parameters.sttm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_bfv[sensed_threshold];
        current_analog_parameters.stfm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_bfv[sensed_threshold];
        break;

    default:
        error("Error when setting analog configuration, MML_NFC_PCD_E_UNKNOWN_PROTOCOL\n");
        current_analog_parameters.trigger_level
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_trigger_a[sensed_threshold];
        current_analog_parameters.sttm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_sttm_a[sensed_threshold];
        current_analog_parameters.stfm
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_stfm_a[sensed_threshold];
        current_analog_parameters.iq_math
            = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_math_a[sensed_threshold];
        return;
    }

    // Implement dynamic gain
    current_analog_parameters.gain
        = mml_nfc_pcd_analog_parameters_matrix.fd_dyn_gain[sensed_threshold];

    if (DUMP_SELECTED_ANALOG_SETTINGS) {
        info("Detected Loading Index: %d\n", sensed_threshold);
        info("Selected Trigger        %3d\n", current_analog_parameters.trigger_level);
        info("Selected STTM           %8X\n", current_analog_parameters.sttm);
        info("Selected STFM           %8X\n", current_analog_parameters.stfm);
        info("Selected Gain           %3d\n", current_analog_parameters.gain);
        info("Selected Atten          %3d\n", current_analog_parameters.atten);

        switch (current_analog_parameters.iq_math) {
        case IQ_MATH_CH_I:
            info("Selected IQ Math      IQ_MATH_CH_I\n");
            break;
        case IQ_MATH_CH_Q:
            info("Selected IQ Math      IQ_MATH_CH_Q\n");
            break;
        case IQ_MATH_I_MINUS_Q:
            info("Selected IQ Math      IQ_MATH_I_MINUS_Q\n");
            break;
        case IQ_MATH_Q_MINUS_I:
            info("Selected IQ Math      IQ_MATH_Q_MINUS_I\n");
            break;
        case IQ_MATH_I_PLUS_Q:
            info("Selected IQ Math      IQ_MATH_I_PLUS_Q\n");
            break;

        default:
            info("Selected IQ Math      IQ_MATH_CH_In");
            break;
        }
    }

    if (mml_nfc_pcd_set_analog_config(current_analog_parameters) != MML_NFC_PCD_E_SUCCESS) {
        error("Error when setting analog configuration\n");
    }
}
