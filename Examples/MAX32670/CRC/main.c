/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2025 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/**
 * @file        main.c
 * @brief       Example showing how to use the CRC module. Covers 16 and 32-bit CRC.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_device.h"
#include "nvic_table.h"
#include "crc.h"
#include "dma.h"

/***** Definitions *****/
typedef struct _test_data_t {
    mxc_crc_reflected_t inputReflected; ///< Input reflected or not
    mxc_crc_reflected_t resultReflected; ///< Result reflected or not
    uint32_t polynominal; ///< The polynominal to calculate CRC
    uint32_t initialValue; ///< The initial value to calculate CRC
    uint32_t finalXorValue; ///< The final xor value to calculate CRC
    uint32_t dataLen; ///< Length of the data
    uint32_t beginAddress; ///< Begin address of the data
    uint8_t *dataBuffer; ///< Pointer to the data
    uint32_t expectedResultCRC; ///< Expected CRC result
} test_data_t;

typedef struct _test_case_mxc_crc_full_req_t {
    char *test_object;
    test_data_t *test_req;
} test_case_mxc_crc_full_req_t;

/***** Globals *****/
test_case_mxc_crc_full_req_t test_cases_crc[] = {
    { .test_object = "Case 01 - CRC32 - Test original JIRA issue (Customer claim)",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x4C11DB7, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0xA3 }, ///< Pointer to the data buffer
              0x9DDD1DDF ///< Expected CRC result
          } },
    { .test_object = "Case 02 - CRC32 - Test original JIRA issue (Customer claim)",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x4C11DB7, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x07 }, ///< Pointer to the data
              0x4C667A2E ///< Expected CRC result
          } },
    { .test_object = "Case 03 - CRC32 - Test original JIRA issue (Customer claim)",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x4C11DB7, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              3, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x00, 0x4E, 0xB2 }, ///< Pointer to the data
              0x0A4CA2D4 ///< Expected CRC result
          } },
    { .test_object = "Case 04 - CRC32 - Test original JIRA issue (Customer claim)",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x4C11DB7, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0x89, 0x3D }, ///< Pointer to the data
              0xF3F07DEC ///< Expected CRC result
          } },
    { .test_object = "Case 05 - CRC32 - Test original JIRA issue (Customer claim)",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x4C11DB7, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              5, ///< Length of the data
              2, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0xD4, 0x6A, 0x1E, 0xB9, 0x03 }, ///< Pointer to the data
              0x547AF3D2 ///< Expected CRC result
          } },
    { .test_object = "Case 06 - CRC32 - Extra test: Random complicated test data",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x5FC812A7, ///< The polynominal to calculate CRC
              0x3AE17FC4, ///< The initial value to calculate CRC
              0x760DA8F1, ///< The final xor value to calculate CRC
              23, ///< Length of the data
              1, ///< Begin address of the data
              (
                  uint8_t[]){ 0x00, 0x4D, 0x10, 0xDD, 0x70, 0xB9, 0x43, 0xFE,
                              0xC1, 0x94, 0x1B, 0x09, 0xA0, 0x9C, 0xE8, 0x8D,
                              0xF6, 0x04, 0x85, 0x9D, 0xCD, 0xE8, 0x69, 0x95 }, ///< Pointer to the data
              0x27823A8C ///< Expected CRC result
          } },
    { .test_object = "Case 07 - CRC32 - Extra test Random complicated test data",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x2CF7813E, ///< The polynominal to calculate CRC
              0xE358A17B, ///< The initial value to calculate CRC
              0x6DB42A90, ///< The final xor value to calculate CRC
              17, ///< Length of the data
              2, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x75, 0xDC, 0x92, 0xEB, 0xF6, 0x9D, 0x71, 0xF8, 0x7D, 0x1B,
                           0x70, 0x86, 0xAC, 0x14, 0xF2, 0xF0, 0x8E }, ///< Pointer to the data
              0x0D346433 ///< Expected CRC result
          } },
    { .test_object = "Case 08 - CRC32 - Extra test: Random complicated test data",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0xA53CE719, ///< The polynominal to calculate CRC
              0x4AD26F88, ///< The initial value to calculate CRC
              0xF936B02E, ///< The final xor value to calculate CRC
              13, ///< Length of the data
              3, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x00, 0x8B, 0x1E, 0xD2, 0x7D, 0x7C, 0x8C, 0xA5, 0xAA, 0x06,
                           0x8E, 0xCA, 0xCD, 0x4E }, ///< Pointer to the data
              0xDAA1D40B ///< Expected CRC result
          } },
    { .test_object = "Case 09 - CRC32 - Extra test: Random complicated test data",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0xC72FA218, ///< The polynominal to calculate CRC
              0x7EC359A0, ///< The initial value to calculate CRC
              0xD81CF467, ///< The final xor value to calculate CRC
              79, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x3B, 0xAA, 0x9B, 0xAD, 0xBF, 0x8E, 0x73, 0x2A, 0x95, 0x00, 0x96,
                           0xAF, 0xD0, 0xE4, 0x4D, 0xAB, 0x64, 0xB5, 0xAD, 0x76, 0x9C, 0x38, 0xEE,
                           0xB3, 0xD6, 0x6D, 0x68, 0x43, 0xCB, 0xD9, 0x27, 0xFE, 0x95, 0x9A, 0xAD,
                           0x01, 0x94, 0x84, 0xA8, 0x49, 0xAD, 0xC9, 0x3B, 0x0B, 0x3B, 0x5A, 0x55,
                           0x99, 0xDE, 0xB4, 0x50, 0xAF, 0xE6, 0x85, 0xB4, 0x49, 0x1B, 0xB5, 0x45,
                           0x94, 0x83, 0xAD, 0x68, 0x5D, 0x51, 0x43, 0xAB, 0x6B, 0xCC, 0xCB, 0xB4,
                           0xD3, 0x6D, 0xCD, 0x03, 0x9D, 0x62, 0x1E, 0xAA }, ///< Pointer to the data
              0x8412B11F ///< Expected CRC result
          } },
    { .test_object = "Case 0A - CRC32 - Extra test: Popular Example CRC-32/AUTOSAR",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0xF4ACFB13, ///< The polynominal to calculate CRC
              0xFFFFFFFF, ///< The initial value to calculate CRC
              0xFFFFFFFF, ///< The final xor value to calculate CRC
              47, ///< Length of the data
              2, ///< Begin address of the data
              (uint8_t[]){
                  0x00, 0x00, 0x2E, 0x23, 0x58, 0xC9, 0x34, 0x9C, 0x57, 0xCB, 0x41, 0x77, 0xC2,
                  0x4D, 0x66, 0xAB, 0x16, 0xDC, 0x96, 0xA8, 0x3D, 0xE4, 0x65, 0x10, 0x07, 0x33,
                  0x1B, 0x85, 0x25, 0xED, 0xD3, 0x23, 0x70, 0xBA, 0xE3, 0xF9, 0x73, 0xDA, 0x5D,
                  0x01, 0x90, 0x25, 0x82, 0x73, 0x73, 0x21, 0x9A, 0xD1, 0x7B }, ///< Pointer to the data
              0xF99AAB41 ///< Expected CRC result
          } },
    { .test_object = "Case 11 - CRC16 - MAXIM",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x8005, ///< The polynominal to calculate CRC
              0x00, ///< The initial value to calculate CRC
              0xFFFF, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0x6A }, ///< Pointer to the data buffer
              0xD07F ///< Expected CRC result
          } },
    { .test_object = "Case 12 - CRC16 - AUG - CCITT",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0x1021, ///< The polynominal to calculate CRC
              0x1D0F, ///< The initial value to calculate CRC
              0x0, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0xD4 }, ///< Pointer to the data buffer
              0x4765 ///< Expected CRC result
          } },
    { .test_object = "Case 13 - CRC16 - USB",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x8005, ///< The polynominal to calculate CRC
              0xFFFF, ///< The initial value to calculate CRC
              0xFFFF, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              3, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x00, 0x3F, 0xA6 }, ///< Pointer to the data buffer
              0xC56F ///< Expected CRC result
          } },
    { .test_object = "Case 14 - CRC16 - TELEDISK",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0xA097, ///< The polynominal to calculate CRC
              0x00, ///< The initial value to calculate CRC
              0x00, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0xE9, 0x4C }, ///< Pointer to the data buffer
              0x5794 ///< Expected CRC result
          } },
    { .test_object = "Case 15 - CRC16 - Random 01",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0xB17E, ///< The polynominal to calculate CRC
              0x4FC0, ///< The initial value to calculate CRC
              0xA25B, ///< The final xor value to calculate CRC
              7, ///< Length of the data
              2, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x3C, 0xE7, 0xA9, 0x52, 0x1D, 0xB6,
                           0x80 }, ///< Pointer to the data buffer
              0xE1FF ///< Expected CRC result
          } },
    { .test_object = "Case 16 - CRC16 - Random 02",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0x3AF7, ///< The polynominal to calculate CRC
              0x9C21, ///< The initial value to calculate CRC
              0x6ED4, ///< The final xor value to calculate CRC
              89, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0xDD, 0xF1, 0xCB, 0xFB, 0xE8, 0x83, 0x9B, 0x1C, 0xB7, 0xE3, 0x5B,
                           0xDB, 0x91, 0x4E, 0x3B, 0x05, 0xF3, 0x64, 0x4F, 0x42, 0x70, 0x1B, 0x98,
                           0x2F, 0x71, 0xDF, 0xF2, 0x35, 0xD3, 0xE3, 0x22, 0x3D, 0xD7, 0x34, 0xD1,
                           0x67, 0x49, 0x59, 0x2D, 0x35, 0x8C, 0x4F, 0xC9, 0x81, 0x2B, 0xDB, 0x1F,
                           0xEA, 0x7F, 0x61, 0x78, 0x66, 0xDA, 0xDB, 0x31, 0xE6, 0xE7, 0xBD, 0x74,
                           0xCE, 0x87, 0x9A, 0x72, 0xE7, 0x25, 0x90, 0xA8, 0xDE, 0x0D, 0x3B, 0x43,
                           0xF9, 0x45, 0x7D, 0x43, 0x09, 0x63, 0xF2, 0x74, 0x9B, 0x90, 0x64, 0x8C,
                           0xC1, 0x0A, 0x1C, 0x8D, 0x5D, 0xE7 }, ///< Pointer to the data buffer
              0x385B ///< Expected CRC result
          } },
    { .test_object = "Case 21 - CRC8 - MAXIM",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x31, ///< The polynominal to calculate CRC
              0x00, ///< The initial value to calculate CRC
              0x00, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0xA3 }, ///< Pointer to the data buffer
              0x4D ///< Expected CRC result
          } },
    { .test_object = "Case 22 - CRC8 - MAXIM",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x31, ///< The polynominal to calculate CRC
              0x00, ///< The initial value to calculate CRC
              0x00, ///< The final xor value to calculate CRC
              1, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x4E }, ///< Pointer to the data buffer
              0x59 ///< Expected CRC result
          } },
    { .test_object = "Case 23 - CRC8 - SAE - J1850",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0x1D, ///< The polynominal to calculate CRC
              0xFF, ///< The initial value to calculate CRC
              0xFF, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              3, ///< Begin address of the data
              (uint8_t[]){ 0x00, 0x00, 0x00, 0xB2, 0x7A }, ///< Pointer to the data buffer
              0xA5 ///< Expected CRC result
          } },
    { .test_object = "Case 24 - CRC8 - ITU",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0x7, ///< The polynominal to calculate CRC
              0x00, ///< The initial value to calculate CRC
              0x55, ///< The final xor value to calculate CRC
              2, ///< Length of the data
              0, ///< Begin address of the data
              (uint8_t[]){ 0x3C, 0xE5 }, ///< Pointer to the data buffer
              0xE5 ///< Expected CRC result
          } },
    { .test_object = "Case 25 - CRC8 - Random 01",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_REFLECTED, ///< Result reflected or not
              0x6A, ///< The polynominal to calculate CRC
              0xD8, ///< The initial value to calculate CRC
              0x0F, ///< The final xor value to calculate CRC
              19, ///< Length of the data
              2, ///< Begin address of the data
              (uint8_t[]){
                  0x00, 0x00, 0x6C, 0xC7, 0x85, 0x25, 0x2E, 0xCE, 0x6C, 0xF3, 0x2A,
                  0x25, 0x31, 0xBF, 0x85, 0x2E, 0xCA, 0xED, 0x98, 0x59, 0x12 }, ///< Pointer to the data buffer
              0x2D ///< Expected CRC result
          } },
    { .test_object = "Case 26 - CRC8 - Random 02",
      .test_req =
          (test_data_t[]){
              CRC_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0x7E, ///< The polynominal to calculate CRC
              0xB4, ///< The initial value to calculate CRC
              0x19, ///< The final xor value to calculate CRC
              29, ///< Length of the data
              1, ///< Begin address of the data
              (uint8_t[]){
                  0x00, 0xA3, 0xF2, 0x34, 0x5C, 0xB9, 0x81, 0xF8, 0x0E, 0x54,
                  0xC1, 0xDD, 0x01, 0x70, 0xFF, 0x5F, 0xEB, 0x2E, 0x74, 0xCE,
                  0x31, 0x65, 0xD0, 0xA4, 0xE3, 0x1F, 0xAD, 0x97, 0x3A, 0x46 }, ///< Pointer to the data buffer
              0x57 ///< Expected CRC result
          } },
    { .test_object = "Case 27 - CRC8 - Random 03",
      .test_req =
          (test_data_t[]){
              CRC_NOT_REFLECTED, ///< Input reflected or not
              CRC_NOT_REFLECTED, ///< Result reflected or not
              0xC2, ///< The polynominal to calculate CRC
              0x5A, ///< The initial value to calculate CRC
              0x8F, ///< The final xor value to calculate CRC
              911, ///< Length of the data
              3, ///< Begin address of the data
              (uint8_t[]){
                  0x00, 0x00, 0x00, 0x4A, 0xD2, 0xFE, 0x95, 0xB6, 0x4F, 0xA5, 0xC0, 0x6B, 0x0C,
                  0xF6, 0xD6, 0xA8, 0xDC, 0x3C, 0x9B, 0xED, 0x3C, 0xEB, 0xB9, 0x03, 0x05, 0x41,
                  0x32, 0xF5, 0x91, 0x66, 0xD2, 0x7E, 0x65, 0xD1, 0x09, 0x19, 0x3C, 0xD9, 0xE3,
                  0x58, 0x7B, 0x24, 0xCD, 0xD8, 0xB9, 0xC2, 0x5F, 0xF0, 0xE3, 0xF2, 0x68, 0x37,
                  0xDC, 0xDC, 0xFA, 0x0C, 0x02, 0x2C, 0x7B, 0xC9, 0x60, 0x7F, 0x1E, 0x8F, 0x98,
                  0x41, 0x00, 0x22, 0x66, 0x2C, 0xA8, 0x61, 0x5A, 0x2F, 0xCD, 0x52, 0x01, 0xB3,
                  0xDC, 0xD6, 0x2B, 0x54, 0x3D, 0xA4, 0x86, 0xDE, 0xEB, 0xD8, 0x71, 0xC1, 0x7C,
                  0xDE, 0x36, 0x16, 0x13, 0x29, 0xE9, 0xB7, 0x50, 0x7F, 0x78, 0x08, 0xCF, 0x07,
                  0x7A, 0xF2, 0xAD, 0x36, 0x3A, 0x8E, 0x4A, 0x0C, 0xB0, 0x6C, 0xD2, 0x28, 0xD4,
                  0xE9, 0x09, 0x85, 0x3E, 0xB5, 0x3E, 0x9C, 0x40, 0xE4, 0x38, 0xAE, 0x7B, 0x63,
                  0xE5, 0xF7, 0xE9, 0x47, 0x94, 0xB6, 0x4D, 0x2E, 0xF9, 0x5C, 0x9B, 0xCF, 0xE0,
                  0x16, 0xFC, 0xAF, 0x0B, 0x56, 0xA1, 0x0D, 0xD6, 0xA1, 0x3E, 0xDB, 0x7D, 0xA0,
                  0x32, 0x32, 0xD8, 0x86, 0x30, 0x68, 0x04, 0x98, 0xE5, 0xAE, 0xED, 0xD8, 0xE9,
                  0x92, 0x17, 0xFD, 0xF8, 0xF7, 0x61, 0xCF, 0x11, 0x01, 0x2C, 0x1A, 0xAD, 0x39,
                  0x62, 0xED, 0x86, 0x6E, 0x06, 0xC3, 0x30, 0x86, 0x5B, 0x37, 0x04, 0xC6, 0x3F,
                  0x9F, 0x52, 0x42, 0xA9, 0x3B, 0xE6, 0xCF, 0xD3, 0x63, 0xB7, 0xEE, 0xE6, 0x7A,
                  0x95, 0x91, 0x29, 0x26, 0xF9, 0xE5, 0xC3, 0x84, 0x5B, 0x00, 0xA7, 0x50, 0x5A,
                  0x70, 0xD4, 0x66, 0xAF, 0x88, 0xCC, 0x97, 0xC6, 0x21, 0x9B, 0x63, 0x26, 0x91,
                  0xB2, 0xEB, 0xCF, 0x84, 0x1F, 0xF6, 0x99, 0xC9, 0xC1, 0x91, 0xA7, 0x7F, 0x6A,
                  0x7C, 0x3F, 0x5A, 0x71, 0xE1, 0x59, 0xFE, 0x82, 0x30, 0xEB, 0x76, 0xF8, 0x63,
                  0xB7, 0x94, 0xBD, 0x6E, 0x9A, 0x27, 0x1E, 0x92, 0x0B, 0xBF, 0x3B, 0xDB, 0x4E,
                  0xB0, 0x43, 0xF1, 0x10, 0x07, 0x46, 0x68, 0xDE, 0x6E, 0xE7, 0x69, 0xDB, 0xB3,
                  0xF2, 0xCD, 0xFF, 0x6E, 0x15, 0x24, 0xDA, 0x3A, 0x87, 0x38, 0x4D, 0xC4, 0xD5,
                  0xED, 0x9B, 0x2D, 0xA9, 0xBE, 0xAD, 0x13, 0x1F, 0xF9, 0xB3, 0x3D, 0x30, 0x26,
                  0x80, 0xD9, 0x9B, 0x8D, 0x0E, 0x85, 0xAA, 0x08, 0xF5, 0x82, 0xC6, 0xAB, 0x6D,
                  0x23, 0x58, 0xA1, 0x7E, 0x36, 0xFD, 0xE4, 0x15, 0x73, 0xA5, 0xDE, 0xEB, 0x1E,
                  0xD9, 0x68, 0x5E, 0x48, 0xB3, 0x3E, 0x66, 0x2E, 0x2F, 0x53, 0xFD, 0x4F, 0x2B,
                  0x1B, 0xAB, 0x01, 0x54, 0x62, 0xF5, 0x5A, 0x68, 0x03, 0x26, 0x73, 0x2D, 0xBF,
                  0xE9, 0xE8, 0x5F, 0x80, 0x9B, 0xA6, 0xD3, 0xB1, 0xAE, 0x6C, 0x3C, 0xD9, 0x6B,
                  0x51, 0x16, 0x41, 0xD3, 0xAF, 0x71, 0xC9, 0xE7, 0x97, 0xE6, 0xE3, 0x84, 0x2A,
                  0xA7, 0x86, 0x2C, 0x14, 0x03, 0x33, 0x8C, 0x59, 0xD6, 0xB1, 0xEE, 0xB3, 0xB4,
                  0x76, 0x13, 0x62, 0x7C, 0x96, 0xA8, 0xF7, 0xB3, 0x5B, 0xB6, 0x5C, 0xFE, 0xCB,
                  0xC3, 0x7B, 0xF1, 0x89, 0xD0, 0x09, 0x0E, 0xE9, 0x06, 0x1F, 0x54, 0x81, 0x24,
                  0x23, 0x9E, 0x34, 0x6F, 0xA1, 0x12, 0xA9, 0x32, 0x82, 0x5D, 0x3C, 0xCB, 0x0F,
                  0x5E, 0x7B, 0xE2, 0xB1, 0x5F, 0xF6, 0x45, 0x1A, 0xB6, 0xBD, 0x8C, 0x6E, 0x02,
                  0xE7, 0x1A, 0x2B, 0xE0, 0xDB, 0x18, 0xE3, 0x18, 0x02, 0xD1, 0x90, 0x7B, 0xDE,
                  0xC1, 0xEA, 0x7B, 0x5D, 0x80, 0xE0, 0x41, 0xA7, 0x02, 0xFC, 0xE9, 0x4B, 0xFC,
                  0xC0, 0x0A, 0xD1, 0x79, 0xF5, 0x62, 0x7F, 0x36, 0x98, 0x2F, 0x3E, 0x4B, 0x56,
                  0x53, 0xCC, 0xE1, 0x21, 0x8C, 0x0B, 0x49, 0x32, 0x83, 0xE7, 0x45, 0x50, 0xCC,
                  0xC2, 0xFA, 0xBB, 0x5E, 0x05, 0x88, 0xFD, 0x91, 0xAB, 0x3B, 0xF6, 0x19, 0xD0,
                  0x6F, 0xEA, 0x86, 0x96, 0xEA, 0x71, 0x52, 0xF8, 0xA8, 0x53, 0x7C, 0x8C, 0xA2,
                  0x13, 0xAE, 0xD9, 0x70, 0xF7, 0xA8, 0x97, 0x2E, 0x96, 0x91, 0x23, 0xD1, 0x83,
                  0x5F, 0xCB, 0x9D, 0xA2, 0xB8, 0xD4, 0x81, 0xCA, 0x4B, 0x0E, 0x55, 0x55, 0x6A,
                  0xFC, 0xC3, 0x1E, 0x58, 0x4B, 0x27, 0x03, 0xDD, 0xC1, 0x90, 0xED, 0xC2, 0x02,
                  0xD3, 0xA9, 0x02, 0x76, 0x7A, 0x21, 0x1B, 0xFC, 0x49, 0x51, 0x41, 0x4F, 0xE9,
                  0x42, 0x60, 0x51, 0xB1, 0x3B, 0x7D, 0xE3, 0x69, 0x90, 0x19, 0x16, 0xA7, 0xCE,
                  0x5C, 0x3B, 0x8C, 0x1B, 0x67, 0xD6, 0x7C, 0xD3, 0xB2, 0xC5, 0x2A, 0x10, 0xBA,
                  0x96, 0x69, 0x17, 0xA8, 0x7F, 0x31, 0x39, 0x5A, 0x4E, 0x9C, 0x60, 0xF0, 0xC7,
                  0x64, 0x1B, 0x60, 0x48, 0xEB, 0x8F, 0x15, 0x42, 0xAC, 0xB3, 0xBA, 0x92, 0xBF,
                  0x23, 0x5E, 0x73, 0x4A, 0x17, 0xE6, 0xED, 0x47, 0x84, 0xAF, 0x0F, 0x95, 0xA3,
                  0x52, 0xAB, 0x2B, 0x4C, 0x8F, 0x65, 0xB9, 0x30, 0xC2, 0xDB, 0xD9, 0xD9, 0x58,
                  0x83, 0x83, 0xB1, 0xED, 0x4A, 0x26, 0xAD, 0xA7, 0xEB, 0x86, 0xA2, 0x7B, 0xD4,
                  0x51, 0x42, 0x40, 0x19, 0x42, 0x7A, 0x0E, 0xAC, 0x09, 0xEE, 0xF0, 0x85, 0x05,
                  0x72, 0x9B, 0x25, 0x53, 0x61, 0x91, 0x68, 0x4D, 0x23, 0x61, 0x58, 0xF9, 0xC1,
                  0x3C, 0xCC, 0x01, 0x39, 0xCA, 0xAE, 0x29, 0xD8, 0x68, 0xD3, 0xB6, 0x04, 0x5F,
                  0xEC, 0x19, 0xF4, 0x0D, 0x6A, 0x6B, 0x54, 0x0D, 0x20, 0xA8, 0x78, 0xAB, 0x5B,
                  0x5E, 0x4A, 0xE9, 0x7E, 0x00, 0xE4, 0x6C, 0xB4, 0x70, 0x62, 0x4F, 0x93, 0x78,
                  0x5F, 0x2A, 0x2E, 0xF5, 0xC2, 0x3D, 0xA7, 0xCB, 0x8D, 0x85, 0x23, 0x42, 0xD4,
                  0xA1, 0xA3, 0x02, 0x9D, 0xC4, 0xE9, 0xF9, 0xDD, 0x74, 0x9E, 0x86, 0x5A, 0x92,
                  0x0F, 0x58, 0x00, 0x8E, 0xF6, 0xAB, 0xFA, 0x3A, 0x6A, 0x3B, 0x2D, 0xE9, 0x62,
                  0x60, 0x57, 0x08, 0xB4, 0xBF, 0x6A, 0xA2, 0x97, 0xCF, 0x45, 0x81, 0x6B, 0x1F,
                  0x40, 0xAD, 0x8A, 0xE5, 0xCE, 0x08, 0x30, 0x74, 0x19, 0x67, 0x2C, 0x42, 0x80,
                  0x22, 0xDF, 0x18, 0x46, 0x5D, 0xAB, 0xF1, 0xB5, 0x5D, 0xB7, 0xAE, 0xB6, 0x41,
                  0xFD, 0xA2, 0x9C, 0x30, 0x31, 0x13, 0xE8, 0x31, 0x96, 0x30, 0x7C, 0xF0, 0x14,
                  0xF5, 0x51, 0x1A, 0xD8, 0xFD, 0x6F, 0x7A, 0xC1, 0x00, 0xFD, 0x83, 0x87, 0xF5,
                  0x44, 0x1A, 0xAA, 0x6F, 0x80, 0xED, 0x28, 0x3F, 0x6B, 0x9E, 0xAF, 0xEF, 0x1C,
                  0xD0, 0xA3, 0xAB, 0xE4, 0xDA, 0xC7, 0x19, 0x0B, 0x76, 0x6C, 0x49, 0x28, 0xB7,
                  0xE5, 0x58, 0xDE, 0xCF, 0x2D, 0x69, 0x37, 0x0D, 0xFE, 0x22, 0xC3, 0xA7, 0xF7,
                  0x35, 0xEB, 0x35, 0xBD, 0x1F, 0x55, 0x65, 0x44, 0x98, 0x74, 0xF8, 0x15, 0xFF,
                  0xA3, 0xE5, 0x61, 0x4E, 0x96, 0x27, 0xCB, 0x0E, 0x30, 0x40, 0x00, 0xDB, 0x46,
                  0x43, 0x90, 0x59, 0x37 }, ///< Pointer to the data buffer
              0xC9 ///< Expected CRC result
          } }
};

/***** Functions *****/
int run_demo_crc(int asynchronous)
{
    int fail = 0;
    int total_cases = sizeof(test_cases_crc) / sizeof(test_case_mxc_crc_full_req_t);
    int i = 0;
    int j = 0;
    mxc_crc_full_req_t tmp_full_req;

    printf("Demo MAXIM CRC-32 Accelerator, total = %d cases\n", total_cases);

    printf(asynchronous ? "TEST CRC ASYNC ****************\n" :
                          "TEST CRC SYNC *****************\n");

    for (i = 0; i < total_cases; i++) {
        printf("%s:\n", test_cases_crc[i].test_object);

        if (test_cases_crc[i].test_req->inputReflected == CRC_REFLECTED) {
            printf("    Input reflected: YES\n");
        } else {
            printf("    Input reflected: NO\n");
        }

        if (test_cases_crc[i].test_req->resultReflected == CRC_REFLECTED) {
            printf("    Result reflected: YES\n");
        } else {
            printf("    Result reflected: NO\n");
        }

        printf("    Polynomial: 0x%08X\n", test_cases_crc[i].test_req->polynominal);
        printf("    Initial Value: 0x%08X\n", test_cases_crc[i].test_req->initialValue);
        printf("    Final Xor Value: 0x%08X\n", test_cases_crc[i].test_req->finalXorValue);
        printf("    Length: %d byte(s)\n", test_cases_crc[i].test_req->dataLen);
        printf(
            "    Begin Address: %p\n",
            (void *)&(
                test_cases_crc[i].test_req->dataBuffer[test_cases_crc[i].test_req->beginAddress]));

        printf("    Input Data:", test_cases_crc[i].test_req->dataLen);
        for (j = 0; j < test_cases_crc[i].test_req->dataLen; j++) {
            printf(" 0x%02X",
                   test_cases_crc[i]
                       .test_req->dataBuffer[j + test_cases_crc[i].test_req->beginAddress]);
        }
        printf("\n");

        printf("    Expected CRC Result: 0x%08X (From internet web ...)\n",
               test_cases_crc[i].test_req->expectedResultCRC);

        tmp_full_req.inputReflected = test_cases_crc[i].test_req->inputReflected;
        tmp_full_req.resultReflected = test_cases_crc[i].test_req->resultReflected;
        tmp_full_req.polynominal = test_cases_crc[i].test_req->polynominal;
        tmp_full_req.initialValue = test_cases_crc[i].test_req->initialValue;
        tmp_full_req.finalXorValue = test_cases_crc[i].test_req->finalXorValue;
        tmp_full_req.dataLen = test_cases_crc[i].test_req->dataLen;
        tmp_full_req.dataBuffer =
            &(test_cases_crc[i].test_req->dataBuffer[test_cases_crc[i].test_req->beginAddress]);

        MXC_CRC_Init();

        if (asynchronous == 0) {
            MXC_CRC_Calculate(&tmp_full_req);
        } else {
            MXC_CRC_CalculateAsync(&tmp_full_req);

            while (tmp_full_req.req_state == CRC_NOT_DONE) {}
        }

        printf("    Actual CRC Result:   0x%08X ", tmp_full_req.resultCRC);
        if (test_cases_crc[i].test_req->expectedResultCRC == tmp_full_req.resultCRC) {
            printf("PASSED!\n");
        } else {
            printf("FAILED!\n");
            fail++;
        }

        MXC_CRC_Shutdown();

        printf("\n");
    }

    return fail;
}

void DMA0_IRQHandler(void)
{
    MXC_DMA_Handler();
}

// *****************************************************************************
int main(void)
{
    int fail = 0;

    MXC_NVIC_SetVector(DMA0_IRQn, DMA0_IRQHandler); // TBD
    NVIC_EnableIRQ(DMA0_IRQn);

    fail += run_demo_crc(0);
    fail += run_demo_crc(1);

    if (fail != 0) {
        printf("\nExample Failed\n");
        return E_FAIL;
    }

    printf("\nExample Succeeded\n");
    return E_NO_ERROR;
}
