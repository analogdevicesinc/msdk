/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2019-02-28 13:54:56 -0600 (Thu, 28 Feb 2019) $
 * $Revision: 41324 $
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   read and write sdhc
 * @details This example uses the sdhc to read and write to an sdhc card
 * You must have an SD card inserted int the card slot on the bottome right corner of the board.
 * Also the VDDIOH needs to have a voltage of 3.3V. To do this connect J11 and disconnect J8.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <MAX32xxx.h>

#include "sdhc_lib.h"

/***** Definitions *****/
#define BLOCK_SIZE 512
#define BLOCK_COUNT 1024
#define MULTI_BLOCK_COUNT 512

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/***** Globals *****/
__attribute__((aligned(4))) uint8_t array[BLOCK_SIZE];    //Array to hold data read and written to card
__attribute__((aligned(4))) uint8_t marray[BLOCK_SIZE * MULTI_BLOCK_COUNT];  //Array to hold data read and written to card

volatile int sdhc_flag = 1;
mxc_gpio_cfg_t SDPowerEnablePin = {MXC_GPIO1, MXC_GPIO_PIN_12, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIO};

/******************************************************************************/
//sdhc callback from async functions
void sdhc_cb(int error)
{
    sdhc_flag = error;
}

/******************************************************************************/
// When interrupt fires
void SDHC_IRQHandler(void)
{
    MXC_SDHC_Lib_Async_Handler();
}

/******************************************************************************/
int check_data(uint8_t* x, uint8_t expected, unsigned int length)
{
    while (length--) {
        if (*x++ != expected) {
            return -1;
        }
    }
    
    return 0;
}

/******************************************************************************/
//Sends write async to card and then does a read async to see if the right data was written
int async_transactions(unsigned int width)
{
    // initialize array
    memset(array, 0xfa, BLOCK_SIZE);
    
    sdhc_flag = 1;
    
    if (MXC_SDHC_Lib_WriteAsync(0, array, 1, width, sdhc_cb) != E_NO_ERROR) {
        return -1;
    }
    
    /* Wait for write to complete */
    while (sdhc_flag == 1);
    
    if (sdhc_flag == E_NO_ERROR) {
        printf("non-blocking write ok\n");
        
    }
    else {
        printf("non-blocking write failed\n");
        return sdhc_flag;
    }
    
    /* Clear data before reading back */
    memset(array, 0, BLOCK_SIZE);
    
    sdhc_flag = 1;
    
    if (MXC_SDHC_Lib_ReadAsync(array, 0, 1, width, sdhc_cb) != E_NO_ERROR) {
        return -1;
    }
    
    /* Wait for read to complete */
    while (sdhc_flag == 1);
    
    if (sdhc_flag == E_NO_ERROR) {
        printf("non-blocking read ok\n");
        
    }
    else {
        printf("non-blocking read failed\n");
        return sdhc_flag;
    }
    
    return check_data(array, 0xfa, BLOCK_SIZE);
    
}

/******************************************************************************/
//erases card with blocking functions
int erase(unsigned int width)
{
    int error;
    
    memset(array, 0, BLOCK_SIZE);
    
    //Write data from array to the card
    if ((error = MXC_SDHC_Lib_Write(0, array, 1, width)) == E_NO_ERROR) {
        printf("blocking erase ok\n");
        
    }
    else {
        printf("blocking erase failed\n");
        return error;
    }
    
    memset(array, 1, BLOCK_SIZE);
    
    //Read data from card and store in array
    if ((error = MXC_SDHC_Lib_Read(array, 0, 1, width)) == E_NO_ERROR) {
        printf("blocking erase read ok\n");
        
    }
    else {
        printf("blocking erase read failed\n");
        return error;
    }
    
    return check_data(array, 0, BLOCK_SIZE);
}

/******************************************************************************/
//Write and then read what was written to card using blocking methods.
int blocking_transactions(unsigned int width)
{
    unsigned int card_block;
    int error;
    
    for (card_block = 0; card_block < BLOCK_COUNT; card_block++) {
    
        /* Write a pattern to SD Card, read it back, check it */
        memset(array, (0xAF + card_block) % 256, BLOCK_SIZE);
        
        if ((error = MXC_SDHC_Lib_Write(card_block, array, 1, width)) != E_NO_ERROR) {
            printf("blocking write failed %d at block %u\n", error, card_block);
            
            while (1);
            
            return error;
        }
        
        memset(array, 0, BLOCK_SIZE);
        
        if ((error = MXC_SDHC_Lib_Read(array, card_block, 1, width)) != E_NO_ERROR) {
            printf("blocking read failed %d at block %u\n", error, card_block);
            return error;
        }
        
        if (check_data(array, (0xAF + card_block) % 256, BLOCK_SIZE)) {
            printf("data compare failed at block %u\n", card_block);
            return -1;
        }
    }
    
    printf("blocking read/write ok\n");
    return 0;
}

int multi_block_check(unsigned int width)
{
    int i, error;
    
    for (i = 0; i < MULTI_BLOCK_COUNT; i++) {
        memset(marray + (i * BLOCK_SIZE), (0x88 + i) % 256, BLOCK_SIZE);
    }
    
    
    if ((error = MXC_SDHC_Lib_Write(0, marray, MULTI_BLOCK_COUNT, width)) != E_NO_ERROR) {
        printf("blocking write failed %d\n", error);
        return error;
    }
    
    memset(marray, 0, BLOCK_SIZE * MULTI_BLOCK_COUNT);
    
    if ((error = MXC_SDHC_Lib_Read(marray, 0, MULTI_BLOCK_COUNT, width)) != E_NO_ERROR) {
        printf("blocking read failed %d\n", error);
        return error;
    }
    
    for (i = 0; i < MULTI_BLOCK_COUNT; i++) {
        if (check_data(marray + (i * BLOCK_SIZE), (0x88 + i) % 256, BLOCK_SIZE)) {
            printf("data compare failed\n");
            return -1;
        }
    }
    
    return 0;
}

/******************************************************************************/
int main(void)
{
    mxc_sdhc_cfg_t cfg;
    int result;
    
    printf("\n\n***** " TOSTRING(TARGET) " SDHC Example *****\n");
    
    // Turn on Power to Card
    MXC_GPIO_Config(&SDPowerEnablePin);
    MXC_GPIO_OutClr(SDPowerEnablePin.port, SDPowerEnablePin.mask);
    // Set up Interupt
    NVIC_EnableIRQ(SDHC_IRQn);
    NVIC_SetVector(SDHC_IRQn, SDHC_IRQHandler);
    
    // Initialize SDHC peripheral
    cfg.bus_voltage = MXC_SDHC_Bus_Voltage_3_3;
    cfg.block_gap = 0;
    cfg.clk_div = 0x0B0; // Maximum divide ratio, frequency must be < 400 kHz during Card Identification phase (SD Specification Part 1 Ch 6.6.6)
    MXC_SDHC_Init(&cfg);
    
    // wait for card to be inserted
    printf("Waiting for card.\n");
    
    while (!MXC_SDHC_Card_Inserted());
    
    printf("Card inserted.\n");
    
    // set up card to get it ready for a transaction
    if (MXC_SDHC_Lib_InitCard(10) == E_NO_ERROR) {
        printf("Card Initialized.\n");
    }
    else {
        printf("No card response!\n");
    }
    
    if (MXC_SDHC_Lib_Get_Card_Type() == CARD_SDHC) {
        printf("Card type: SDHC\n");
    }
    else {
        printf("Card type: MMC/eMMC\n");
    }
    
    /* Configure for fastest possible clock, must not exceed 52 MHz for eMMC */
    if (SystemCoreClock > 96000000)  {
        printf("SD clock ratio (at card) 4:1\n");
        MXC_SDHC_Set_Clock_Config(1);
    }
    else {
        printf("SD clock ratio (at card) 2:1\n");
        MXC_SDHC_Set_Clock_Config(0);
    }
    
    /*** 1-bit data bus ***/
    printf("--> 1-bit data bus example <--\n");
    
    if ((result = blocking_transactions(MXC_SDHC_LIB_SINGLE_DATA)) != 0) {
        printf("blocking error %d\n", result);
    }
    else {
        printf("Passed blocking\n");
    }
    
    if ((result = erase(MXC_SDHC_LIB_SINGLE_DATA)) != 0) {
        printf("Erase failed %d\n", result);
    }
    else {
        printf("Passed erase\n");
    }
    
    if ((result = async_transactions(MXC_SDHC_LIB_SINGLE_DATA)) != 0) {
        printf("async error %d\n", result);
    }
    else {
        printf("Passed async\n");
    }
    
    /*** 4-bit data bus ***/
    printf("--> 4-bit data bus example <--\n");
    
    if ((result = blocking_transactions(MXC_SDHC_LIB_QUAD_DATA)) != 0) {
        printf("blocking error %d\n", result);
    }
    else {
        printf("Passed blocking\n");
    }
    
    if ((result = erase(MXC_SDHC_LIB_QUAD_DATA)) != 0) {
        printf("Erase failed %d\n", result);
    }
    else {
        printf("Passed erase\n");
    }
    
    if ((result = async_transactions(MXC_SDHC_LIB_QUAD_DATA)) != 0) {
        printf("async error %d\n", result);
    }
    else {
        printf("Passed async\n");
    }
    
    printf("--> Blocking, 4-bit data bus, multi-block example <--\n");
    
    if (multi_block_check(MXC_SDHC_LIB_QUAD_DATA)) {
        printf(" FAIL \n");
    }
    else {
        printf(" PASS \n");
    }
    
    
    printf(" *** END OF EXAMPLE *** \n");
}


