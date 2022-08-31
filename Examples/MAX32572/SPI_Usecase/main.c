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
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   SPI Usecase!
 * @details This example demonstrate how to use SPI master slave feature.
 */

/*******************************      INCLUDES    ****************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "spi_config.h"

/*******************************      DEFINES     ****************************/

/******************************* Type Definitions ****************************/

/******************************* 	 Variables    ****************************/

/******************************* Static Functions ****************************/
static void hex_dump(const char* title, unsigned char* buf, unsigned int len)
{
    unsigned int i;

    if (title) {
        printf("%s", title);
    }

    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
            printf("\n");
        }
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

int main(void)
{
    int ret = 0;
    unsigned int i;
    unsigned char tx_buff[TEST_BUFF_SIZE] = { 0 };
    unsigned char rx_buff[TEST_BUFF_SIZE] = { 0 };
    unsigned char expect_packet[TEST_BUFF_SIZE] = { 0 };
    unsigned int len;

    printf("---------------------------------------------------------------------------------------"
           "--------\n");
    printf("Open all TS_EN jumper to access  SPO%d Line\n", MXC_SPI_GET_IDX(SPIx_MASTER));
    printf("Unplug TFT LCD Display to access SPI%d Line\n", MXC_SPI_GET_IDX(SPIx_SLAVE));
    printf("For MAX32572-QNKIT, Be sure DISP_RS_DC is connected to DISP1 socket\n"
           "\tR53 should be assembled, to DISP_RS_DC line goes to DISP1 socket, Please check it\n");
    printf("\n");
    printf("SPI%d is configured as master\n", MXC_SPI_GET_IDX(SPIx_MASTER));
    printf("SPI%d is configured as slave\n", MXC_SPI_GET_IDX(SPIx_SLAVE));
    printf("Please use jumper to connect these two SPI ports:\n");
    printf("\tSS  : P0.2 (TS_SSEL0) <-->  P1.0  (DISP_RS_DC)\n");
    printf("\tMISO: P0.3 (TS_MISO)  <-->  P1.3  (DISP_MISO)\n");
    printf("\tMOSI: P0.4 (TS_MOSI)  <-->  P1.4  (DISP_MOSI)\n");
    printf("\tCLK : P0.5 (TS_CLK)   <-->  P1.5  (DISP_CLK)\n");
    printf("\n");
    printf(
        "This example will send some test byte from master to slave then from slave to master\n");
    printf("To change number of bytes please change TEST_BUFF_SIZE\n");
    printf("---------------------------------------------------------------------------------------"
           "--------\n\n");

    for (i = 0; i < TEST_BUFF_SIZE; i++) {
        tx_buff[i] = i + 1;
        expect_packet[i] = 0xFF - i;
    }

    ret = spi_slave_init();
    if (ret) {
        return ret;
    }

    ret = spi_master_init();
    if (ret) {
        return ret;
    }

    hex_dump("Master Send Packet", tx_buff, TEST_BUFF_SIZE);
    hex_dump("\nMaster Expect Packet", expect_packet, TEST_BUFF_SIZE);
    printf("-------------------------------------------------------\n");

    spi_slave_send(expect_packet, TEST_BUFF_SIZE);

    ret = spi_master_send_rcv(tx_buff, TEST_BUFF_SIZE, rx_buff);
    if (ret) {
        printf("\nSPI Master Send Failed! (0x%X)\n", ret);
        return ret;
    }
    hex_dump("\nMaster Receive:", rx_buff, TEST_BUFF_SIZE);

    if (memcmp(rx_buff, expect_packet, TEST_BUFF_SIZE) == 0) {
        printf("\nMaster Send Receive Succeeded\n");
    } else {
        printf("\nMaster Read Failed!\n");
    }

    memset(rx_buff, 0, sizeof(rx_buff));

    spi_slave_rcv(rx_buff, TEST_BUFF_SIZE, &len);
    hex_dump("\nSlave Receive:", rx_buff, len);

    if (memcmp(rx_buff, tx_buff, TEST_BUFF_SIZE) == 0) {
        printf("\nSlave Send Receive Succeeded\n");
    } else {
        printf("\nSlave Read Failed!\n");
        ret = -1;
    }

    printf("\nEnd of Example!\n");

    return ret;
}
