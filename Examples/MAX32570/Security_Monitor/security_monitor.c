/*
 * security_monitor.c
 *
 ******************************************************************************
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

#include <stdio.h>
#include <string.h>
#include <MAX32xxx.h>
#include <trng_regs.h>

#include "security_monitor.h"
#include "keypad.h"
#include "utils.h"

/****************************       DEFINES     ******************************/
// CHIPPER Key type, BB: Battery Backed key
#define BB_KEY   0
#define USER_KEY 1

static unsigned int* nvsram_region0 = (unsigned int*)0x20100000;
static unsigned int* nvsram_region1 = (unsigned int*)0x20100080;
static unsigned int* nvsram_region2 = (unsigned int*)0x20100100;
static unsigned int* nvsram_region3 = (unsigned int*)0x20100180;
static unsigned int* nvsram_region4 = (unsigned int*)0x20100200;
static unsigned int* nvsram_region5 = (unsigned int*)0x20100280;
static unsigned int* nvsram_region6 = (unsigned int*)0x20100300;
static unsigned int* nvsram_region7 = (unsigned int*)0x20100380;

/**************************** Static Functions *******************************/
void NMI_Handler(void)
{
    printf("\n[-----------------------------------------------]");
    printf("\n[                  NMI - DRS                    ]");
    printf("\n[-----------------------------------------------]");

    // Find attack source
    smon_check_tamper();
    /* Perform needed action according to secalm register */
    //...

    /* Acknowledge NMI IRQ */
    NVIC->ICPR[0] = (1 << 2);
    printf("\n\n/* Now restarting device...*/\n ");
    utils_delay_ms(250); // wait until all UART data has been sent

    // Reset device
    MXC_SYS_Reset_Periph(MXC_SYS_RESET0_SYS);
}

void get_aes_result(unsigned int* data, int enc, int chipper_key)
{
    /* Clear CPH_DONE */
    MXC_CTB->crypto_ctrl = 0;
    /* Compute AES encryption with user or secure key, 128-bit key length */
    /* Compute AES encryption with User key, 256-bit key length */

    MXC_CTB->cipher_ctrl = 0;

    if (enc == 0) {
        MXC_CTB->cipher_ctrl |= 1; // means dec
    }

    MXC_CTB->cipher_ctrl |= MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128;

    if (chipper_key == BB_KEY) {
        // user BB key for enc/dec
        MXC_CTB->cipher_ctrl |= 0x08; // USE AES key
    }

    MXC_CTB->crypto_din[0] = data[0];
    MXC_CTB->crypto_din[1] = data[1];
    MXC_CTB->crypto_din[2] = data[2];
    MXC_CTB->crypto_din[3] = data[3];

    /* Poll to wait for AES completion */
    while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_CPH_DONE))
        ;

    /* Clear CPH_DONE */
    MXC_CTB->crypto_ctrl = 0;
    /* Read computed data */
    data[0] = MXC_CTB->crypto_dout[0];
    data[1] = MXC_CTB->crypto_dout[1];
    data[2] = MXC_CTB->crypto_dout[2];
    data[3] = MXC_CTB->crypto_dout[3];
}

static void set_nvsram(void)
{
    const char* str;

    str = "TEST LINE 012345";
    memcpy(nvsram_region0, str, strlen(str));
    str = "TEST LINE 123456";
    memcpy(nvsram_region1, str, strlen(str));
    str = "TEST LINE 234567";
    memcpy(nvsram_region2, str, strlen(str));
    str = "TEST LINE 345678";
    memcpy(nvsram_region3, str, strlen(str));
    str = "TEST LINE 456789";
    memcpy(nvsram_region4, str, strlen(str));
    str = "TEST LINE 567890";
    memcpy(nvsram_region5, str, strlen(str));
    str = "TEST LINE 678901";
    memcpy(nvsram_region6, str, strlen(str));
    str = "TEST LINE 789012";
    memcpy(nvsram_region7, str, strlen(str));

    return;
}

static void dump_nvsram(char* title)
{
    int i, k;
    unsigned char buf[16] = {0};
    unsigned char byt;
    unsigned int nb_regions = 8;
    unsigned int* regions[] = {nvsram_region0, nvsram_region1, nvsram_region2, nvsram_region3,
                               nvsram_region4, nvsram_region5, nvsram_region6, nvsram_region7};

    if (title) {
        printf("%s\n", title);
    }

    for (i = 0; i < nb_regions; i++) {
        memcpy(buf, regions[i], sizeof(buf));

        printf("NVSRAM Region-%d: ", i);

        for (k = 0; k < sizeof(buf); k++) {
            byt = buf[k];
            if ((byt < 0x20) || (byt > 0x7E)) {
                byt = ' ';
            }
            printf("%c", byt);
        }
        printf("\n");
    }

    return;
}

/**************************** Public Functions *******************************/
int smon_init(void)
{
    //
    MXC_SMON_Init();

    //
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CTB);

    return 0;
}

int smon_check_aes_key(void)
{
    unsigned int data[4];

    /*
     * check key erasing
     * define AES data
     */
    data[0] = 0xd44be966;
    data[1] = 0x3b2c8aef;
    data[2] = 0x59fa4c88;
    data[3] = 0x2e2b34ca;

    utils_hex_dump("\nPlain Text:", (unsigned char*)data, 16);

    get_aes_result(data, 0, BB_KEY);
    utils_hex_dump("AES DEC Result:", (unsigned char*)data, 16);

    /* if keywipe has been performed, the secure AES key is 0x0000000...000 */
    if ((data[3] == 0) && (data[2] == 0) && (data[1] == 0) && (data[0] == 0)) {
        printf("\t### keywipe has been performed !!!###\n");
    } else {
        printf("\tNo keywipe\n");
    }

    dump_nvsram("--------------------------------");

    return 0;
}

int smon_rtc_status(void)
{
    /* RTC_RTCCN */
    if (MXC_RTC->ctrl & MXC_F_RTC_CTRL_RTCE) {
        /* RTC running when check occurred */
        printf("\nRTC Running.\n");
    } else {
        printf("\nRTC Stopped.\n");
    }

    return 0;
}

int smon_start_rtc(void)
{
    if (MXC_RTC->ctrl & MXC_F_RTC_CTRL_RTCE) {
        printf("\nRTC Running.\n");
        return 0; // it is already running
    }

    //
    MXC_RTC_Init(0, 0);
    //
    MXC_RTC_Start();

    /* Wait for the busy flag */
    while (((MXC_RTC->ctrl >> MXC_F_RTC_CTRL_BUSY_POS) & 0x1) == 1)
        ;

    /* Read back RTC control register */
    printf("\nRTC control register: 0x%08x", (unsigned int)MXC_RTC->ctrl);

    /* Wait for the busy flag */
    while (((MXC_RTC->ctrl >> MXC_F_RTC_CTRL_BUSY_POS) & 0x1) == 1)
        ;

    printf("\nRTC second register: 0x%08x", (unsigned int)MXC_RTC->sec);

    // check rtc status
    smon_rtc_status();

    return 0;
}

int smon_set_ext_sensors(void)
{
#ifdef _UNLOCK_
    printf("********************************\n");
    printf("*   EXT SENSORS _UNLOCKED       *\n");
    printf("********************************\n");
#endif /* _UNLOCK_ */

    /* External sensors configuration
     *  enable all the external sensors
     *  set external sensor number to 0
     *  set external sensor frequency to the max value: EXTFREQ = 000
     *  don't divide the 8kHz input clock: DIVCLK = 000
     *  lock the register
     */
#ifndef _UNLOCK_
    MXC_SMON->extscn = MXC_F_SMON_EXTSCN_LOCK | MXC_F_SMON_EXTSCN_EXTS_EN0 |
                       MXC_F_SMON_EXTSCN_EXTS_EN1 | MXC_F_SMON_EXTSCN_EXTS_EN2 |
                       MXC_F_SMON_EXTSCN_EXTS_EN3 | MXC_F_SMON_EXTSCN_EXTS_EN4 |
                       MXC_F_SMON_EXTSCN_EXTS_EN5;
#else
    MXC_SMON->extscn = MXC_F_SMON_EXTSCN_EXTS_EN0 | MXC_F_SMON_EXTSCN_EXTS_EN1 |
                       MXC_F_SMON_EXTSCN_EXTS_EN2 | MXC_F_SMON_EXTSCN_EXTS_EN3 |
                       MXC_F_SMON_EXTSCN_EXTS_EN4 | MXC_F_SMON_EXTSCN_EXTS_EN5;
#endif /* _UNLOCK_ */

    printf("\nAll External Sensors Activated\n");
    return 0;
}

int smon_set_int_sensors(void)
{
    /*
     *  Internal sensors configuration
     *  bit 0: Shield
     *  bit 1: Temp
     *  bit 2: Vbat
     */
#ifndef _UNLOCK_
    MXC_SMON->intscn = MXC_F_SMON_INTSCN_LOCK | MXC_F_SMON_INTSCN_SHIELD_EN |
                       MXC_F_SMON_INTSCN_TEMP_EN | MXC_F_SMON_INTSCN_VBAT_EN;
#else
    MXC_SMON->intscn =
        MXC_F_SMON_INTSCN_SHIELD_EN | MXC_F_SMON_INTSCN_TEMP_EN | MXC_F_SMON_INTSCN_VBAT_EN;
#endif

    printf("\nAll Internal Sensors Activated\n");

    return 0;
}

int load_user_chipper_key(void)
{
    unsigned int result       = 0;
    unsigned char aes_key[32] = {
        1, 2, 3, 4, 5, 6,
    };
    unsigned int* ptr;
    unsigned int* ptr_key_register = (unsigned int*)AES_BASE;

    /*
     *
     * Generate a pseudo random and store it into aes_key
     * Then register it on AES register
     *
     */
    // generate_random()

    ptr = (unsigned int*)&aes_key[0];

    /* Load aes_key */
    *ptr_key_register++ = ptr[0];
    *ptr_key_register++ = ptr[1];
    *ptr_key_register++ = ptr[2];
    *ptr_key_register++ = ptr[3];
    *ptr_key_register++ = ptr[4];
    *ptr_key_register++ = ptr[5];
    *ptr_key_register++ = ptr[6];
    *ptr_key_register   = ptr[7];

    return result;
}

int load_aes_bb_key(void)
{
    // generate and transfer AES key
    MXC_TRNG->cn |= MXC_F_TRNG_CN_AESKG;

    while ((MXC_SMON->secdiag & MXC_F_SMON_SECDIAG_AESK_MDU) == 0) {
        // wait until it transfer to AES key register
    }

    return 0;
}

int smon_load_aes_key(void)
{
    load_aes_bb_key();

    /*
	 *	Configure plain/encrypted area of the backed NVSRAM.
 	 *	NVSRAM is divided in 8 regions, Each MEU_CFG bit correspond to one of the 8 regions:
	 *	0 --> region is in plain
	 *	1 --> region is encrypted
     */
    MXC_SMON->meucfg |= (1 << 0); // region0: encrypted
    /*
     * For test purpose keep region1 as plain text
     */
    //MXC_SMON->meucfg |= (1<<1);  // region1: encrypted
    MXC_SMON->meucfg |= (1 << 2); // region2: encrypted
    MXC_SMON->meucfg |= (1 << 3); // region3: encrypted
    MXC_SMON->meucfg |= (1 << 4); // region4: encrypted
    MXC_SMON->meucfg |= (1 << 5); // region5: encrypted
    MXC_SMON->meucfg |= (1 << 6); // region6: encrypted
    MXC_SMON->meucfg |= (1 << 7); // region7: encrypted

    /*
     * Lock Register.
     * 	Once locked, the MEUCFG register can no longer be modified.
     * 	Only a battery disconnect will clear this bit. VBAT powers this register.
     * 			0: Not locked. Writes to this register allowed.
	 *			1: Locked. Writes to this register ignored.
     */
    MXC_SMON->meucfg |= (1 << 31); // LOCK it

    // reset crypto module
    MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_RST;

    // wait until ready
    while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_RDY))
        ;

    if (MXC_SMON->secdiag & MXC_F_SMON_SECDIAG_AESK_MDU) {
        printf("\nAES Key has been transferred to battery backed location\n");

        // Write a test string to demonstrate ENC/Plain feature of NVSRAM
        set_nvsram();
        dump_nvsram("--------------------------------");
    } else {
        printf("\nERROR!, AES Key has NOT been transferred to battery backed location\n");
    }

    return 0;
}

/* This function is used to clear security alarm registers */
int smon_clear_tamper(void)
{
    // clear all flgas
    MXC_SMON_ClearFlags((unsigned int)-1);
    printf("\nTamper Flags Cleared\n");

    return 0;
}

int smon_check_tamper_time(void)
{
    DateTime_t dt;
    unsigned int rtcLog;

    rtcLog = MXC_SMON->dlrtc;

    if (rtcLog) {
        utils_seconds_to_date(&dt, rtcLog);
        printf("\nTamper Time is: %u-%02u-%02u %02u:%02u:%02u\n", dt.year, dt.mon + 1, dt.day + 1,
               dt.hour, dt.min, dt.sec);
    } else {
        printf("\nTamper Time is: 0 (MXC_SMON->DLRRTC=0)\n");
    }

    return 0;
}

int smon_check_tamper(void)
{
    unsigned int reg;

    reg = MXC_SMON_GetFlags();

    printf("\nSECALM register: %X\n", reg);

    printf("\nshield: %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_SHIELDF) >> MXC_F_SMON_SECALM_SHIELDF_POS);
    printf("\nhtf   : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_HITEMP) >> MXC_F_SMON_SECALM_HITEMP_POS);
    printf("\nltf   : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_LOTEMP) >> MXC_F_SMON_SECALM_LOTEMP_POS);
    printf("\nhbf   : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_BATHI) >> MXC_F_SMON_SECALM_BATHI_POS);
    printf("\nlbf   : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_BATLO) >> MXC_F_SMON_SECALM_BATLO_POS);
    printf("\nexts0 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT0) >> MXC_F_SMON_SECALM_EXTSTAT0_POS);
    printf("\nexts1 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT1) >> MXC_F_SMON_SECALM_EXTSTAT1_POS);
    printf("\nexts2 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT2) >> MXC_F_SMON_SECALM_EXTSTAT2_POS);
    printf("\nexts3 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT3) >> MXC_F_SMON_SECALM_EXTSTAT3_POS);
    printf("\nexts4 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT4) >> MXC_F_SMON_SECALM_EXTSTAT4_POS);
    printf("\nexts5 : %d",
           (unsigned int)(reg & MXC_F_SMON_SECALM_EXTSTAT5) >> MXC_F_SMON_SECALM_EXTSTAT5_POS);

    return 0;
}

int smon_clear_tamper_time(void)
{
    printf("\nTo Clear tamper time: Follow below steps"
           "       1: Power off EvKit\n"
           "       2: Remove battery\n"
           "       3: Power on EvKit\n");

    return 0;
}

int smon_secalm_keywipe(void)
{
    MXC_SMON_isBusy(SMON_SECALARM, 0);
    MXC_SMON->secalm |= MXC_F_SMON_SECALM_KEYWIPE;
    MXC_SMON_isBusy(SMON_SECALARM, 0);

    printf("\nSECALM.KEYWIPE bit will be set to 1\n");

    return 0;
}

int smon_create_DRS(void)
{
    printf("\nManual DRS will be created\n");

    MXC_SMON_isBusy(SMON_SECALARM, 0);
    MXC_SMON->secalm |= MXC_F_SMON_SECALM_DRS;
    MXC_SMON_isBusy(SMON_SECALARM, 0);

    printf("Manual DRS created\n");

    return 0;
}
