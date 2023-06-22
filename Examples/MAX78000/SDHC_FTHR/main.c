/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
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

/**
 * @file    main.c
 * @brief   read and write sdhc
 * @details This example uses the sdhc and ffat to read/write the file system on
 *          an SD card. The Fat library used supports long filenames (see ffconf.h)
 *          the max length is 256 characters.
 *
 *          You must connect an sd card to the sd card slot.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "gpio.h"
#include "uart.h"

#include "ff.h"
#include "cli.h"
#include "sdhc.h"

#ifdef BOARD_EVKIT_V1
#warning This example is not supported by the MAX78000EVKIT.
#endif

/***** Definitions *****/

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define MAXLEN 256





/******************************************************************************/
int main(void)
{
    FF_ERRORS[0] = "FR_OK";
    FF_ERRORS[1] = "FR_DISK_ERR";
    FF_ERRORS[2] = "FR_INT_ERR";
    FF_ERRORS[3] = "FR_NOT_READY";
    FF_ERRORS[4] = "FR_NO_FILE";
    FF_ERRORS[5] = "FR_NO_PATH";
    FF_ERRORS[6] = "FR_INVLAID_NAME";
    FF_ERRORS[7] = "FR_DENIED";
    FF_ERRORS[8] = "FR_EXIST";
    FF_ERRORS[9] = "FR_INVALID_OBJECT";
    FF_ERRORS[10] = "FR_WRITE_PROTECTED";
    FF_ERRORS[11] = "FR_INVALID_DRIVE";
    FF_ERRORS[12] = "FR_NOT_ENABLED";
    FF_ERRORS[13] = "FR_NO_FILESYSTEM";
    FF_ERRORS[14] = "FR_MKFS_ABORTED";
    FF_ERRORS[15] = "FR_TIMEOUT";
    FF_ERRORS[16] = "FR_LOCKED";
    FF_ERRORS[17] = "FR_NOT_ENOUGH_CORE";
    FF_ERRORS[18] = "FR_TOO_MANY_OPEN_FILES";
    FF_ERRORS[19] = "FR_INVALID_PARAMETER";
    srand(12347439);
    //int run = 1, input = -1;

    printf("\n\n***** " TOSTRING(TARGET) " SDHC FAT Filesystem Example *****\n");

    waitCardInserted();

    printf("Card inserted.\n");

    while(1){
        uint8_t character = getchar();
        line_accumlator(character);
    }
    // while (run) {
    //     f_getcwd(cwd, sizeof(cwd));

    //     printf("\nChoose one of the following options: \n");
    //     printf("0. Find the Size of the SD Card and Free Space\n");
    //     printf("1. Format the Card\n");
    //     printf("2. Manually Mount Card\n");
    //     printf("3. List Contents of Current Directory\n");
    //     printf("4. Create a Directory\n");
    //     printf("5. Move into a Directory (cd)\n");
    //     printf("6. Create a File of Random Data\n");
    //     printf("7. Add Random Data to an Existing File\n");
    //     printf("8. Delete a File\n");
    //     printf("9. Format Card and Run Exmaple of FatFS Operations\n");
    //     printf("10. Unmount Card and Quit\n");
    //     printf("%s>>", cwd);

    //     input = -1;
    //     scanf("%d", &input);
    //     printf("%d\n", input);

    //     err = 0;

    //     switch (input) {
    //     case 0:
    //         getSize();
    //         break;

    //     case 1:
    //         formatSDHC();
    //         break;

    //     case 3:
    //         ls();
    //         break;

    //     case 6:
    //         createFile();
    //         break;

    //     case 7:
    //         appendFile();
    //         break;

    //     case 4:
    //         mkdir();
    //         break;

    //     case 5:
    //         cd();
    //         break;

    //     case 9:
    //         example();
    //         break;

    //     case 10:
    //         umount();
    //         run = 0;
    //         break;

    //     case 2:
    //         mount();
    //         break;

    //     case 8:
    //         delete ();
    //         break;

    //     default:
    //         printf("Invalid Selection %d!\n", input);
    //         err = -1;
    //         break;
    //     }

    //     if (err >= 0 && err <= 20) {
    //         printf("Function Returned with code: %s\n", FF_ERRORS[err]);
    //     } else {
    //         printf("Function Returned with code: %d\n", err);
    //     }

    //     MXC_Delay(MSEC(500));
    // }

    printf("End of example, please try to read the card.\n");
    return 0;
}
