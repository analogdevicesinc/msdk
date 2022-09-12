/*===========================================================================
 *
 * p192_os.h
 *
 *==========================================================================*/
/*===========================================================================
 *
 * Copyright © 2009 Innova Card. All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Innova Card ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Innova Card.
 *
 * Innova Card makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Innova Card shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*===========================================================================
 *
 * Purpose: 
 *
 *==========================================================================*/
#ifndef P192_OS_H_
#define P192_OS_H_

#define P192_BSIZE 192
#define P192_SIZE 24
#define P192_S 6

// q = 6277101735386680763835789423207666416083908700390324961279
static u8 _q192[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

// SEED = 3045AE6F C8422F64 ED579528 D38120EA E12196D5
static u8 _seed192[] = { 0x30, 0x45, 0xAE, 0x6F, 0xC8, 0x42, 0x2F, 0x64, 0xED, 0x57,
                         0x95, 0x28, 0xD3, 0x81, 0x20, 0xEA, 0xE1, 0x21, 0x96, 0xD5 };

// a = FFFFFFFF FFFFFFFF FFFFFFFF FFFFFFFE FFFFFFFF FFFFFFFC
static u8 _a192[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                      0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };

// b = 64210519 E59C80E7 0FA7E9AB 72243049 FEB8DEEC C146B9B1
static u8 _b192[] = { 0x64, 0x21, 0x05, 0x19, 0xE5, 0x9C, 0x80, 0xE7, 0x0F, 0xA7, 0xE9, 0xAB,
                      0x72, 0x24, 0x30, 0x49, 0xFE, 0xB8, 0xDE, 0xEC, 0xC1, 0x46, 0xB9, 0xB1 };

// G = 03 188DA80E B03090F6 7CBF20EB 43A18800 F4FF0AFD 82FF1012
static u8 _g192[] = { 0x03, 0x18, 0x8D, 0xA8, 0x0E, 0xB0, 0x30, 0x90, 0xF6, 0x7C, 0xBF, 0x20, 0xEB,
                      0x43, 0xA1, 0x88, 0x00, 0xF4, 0xFF, 0x0A, 0xFD, 0x82, 0xFF, 0x10, 0x12 };

// n = 6277101735386680763835789423176059013767194773182842284081
static u8 _n192[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0x99, 0xde, 0xf8, 0x36, 0x14, 0x6b, 0xc9, 0xb1, 0xb4, 0xd2, 0x28, 0x31 };

#endif /*P192_OS_H_*/
