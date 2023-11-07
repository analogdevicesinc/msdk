/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
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

/* One wire API Test, for use with DS2401 */

#include <stdio.h>
#include <string.h>

#include <MAX32xxx.h>

// Local global variables
uint8_t utilcrc8;
static uint8_t dscrc_table[] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,  65,  157, 195, 33,
    127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220, 35,  125, 159, 193, 66,  28,
    254, 160, 225, 191, 93,  3,   128, 222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124,
    34,  192, 158, 29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,  102,
    229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,  165, 251, 120, 38,  196,
    154, 101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122, 36,  248, 166,
    68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237,
    179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,  204, 146,
    211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206, 144, 114, 44,  109, 51,  209,
    143, 12,  82,  176, 238, 50,  108, 142, 208, 83,  13,  239, 177, 240, 174, 76,  18,  145, 207,
    45,  115, 202, 148, 118, 40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,
    9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233, 183, 85,  11,
    136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168, 116, 42,  200, 150, 21,  75,  169,
    247, 182, 232, 10,  84,  215, 137, 107, 53
};

//--------------------------------------------------------------------------
// Reset crc8 to the value passed in
//
// 'portnum'  - number 0 to MAX_PORTNUM-1.  This number is provided to
//              indicate the symbolic port number.
// 'reset'    - data to set crc8 to
//
void setcrc8(uint8_t reset)
{
    utilcrc8 = reset;
    return;
}

//--------------------------------------------------------------------------
// Update the Dallas Semiconductor One Wire CRC (utilcrc8) from the global
// variable utilcrc8 and the argument.
//
// 'portnum'  - number 0 to MAX_PORTNUM-1.  This number is provided to
//              indicate the symbolic port number.
// 'x'        - data byte to calculate the 8 bit crc from
//
// Returns: the updated utilcrc8.
//
uint8_t docrc8(uint8_t x)
{
    utilcrc8 = dscrc_table[utilcrc8 ^ x];
    return utilcrc8;
}

int32_t ow_romid_test(uint8_t od)
{
    uint8_t buffer[8];
    uint8_t crc8;
    int i;

    /* Set 1-Wire to standard speed */
    MXC_OWM_SetOverdrive(0);

    /* Error if presence pulse not detected. */
    if (MXC_OWM_Reset() == 0) {
        return -2;
    }

    if (od) {
        /* Send the overdrive command */
        buffer[0] = OD_SKIP_ROM_COMMAND;
        MXC_OWM_Write(buffer, 1);
        MXC_OWM_SetOverdrive(1);

        /* Error if presence pulse not detected. */
        if (MXC_OWM_Reset() == 0) {
            return -4;
        }
    }

    /* Send read ROMID command */
    buffer[0] = READ_ROM_COMMAND;
    MXC_OWM_Write(buffer, 1);
    /* Read the ROM ID */
    memset(buffer, 0, sizeof(buffer));

    if (MXC_OWM_Read(buffer, 8) < 0) {
        return -5;
    }

    printf("Buffer: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n", buffer[0],
           buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6], buffer[7]);

    /* Check for zero family code in ROM ID */
    if (buffer[0] == 0) {
        return -6;
    }

    /* Check CRC8 of received ROM ID */
    setcrc8(0);

    for (i = 0; i < 8; i++) {
        crc8 = docrc8(buffer[i]);
    }

    if (crc8 != 0x00) {
        return -7;
    }

    return 0;
}

int main(void)
{
    int retval = 0;
    printf("***** 1-Wire ROM (DS2401) Example *****\n");

    /* Initialize one wire 10us timing */

    mxc_owm_cfg_t owm_cfg;
    owm_cfg.int_pu_en = 1;
    owm_cfg.ext_pu_mode = MXC_OWM_EXT_PU_ACT_HIGH;
    owm_cfg.long_line_mode = 0;
    // owm_cfg.overdrive_spec = MXC_OWM_OVERDRIVE_10US;
    MXC_OWM_Init(&owm_cfg);

    /* Test overdrive */
    if (retval = ow_romid_test(1)) {
        printf("Overdrive results: %d; %08x; %08x \n", retval, MXC_OWM->cfg, MXC_OWM->intfl);
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
