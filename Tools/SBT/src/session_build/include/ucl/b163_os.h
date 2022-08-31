/*===========================================================================
 *
 * b163_os.h
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
#ifndef B163_OS_H_
#define B163_OS_H_

#define B163_BSIZE 163
#define B163_S 6
#define B163_SIZE 24
#define B163_BUFFER_SIZE (sizeof(ucl_ecc_curve_st) + (7 * B163_SIZE) + sizeof(ucl_ecc_point_st))

/* ------------------------------------------- */

static u8 _q163[] = { 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc9 };

// Gx: 3 f0eba162 86a2d57e a0991168 d4994637 e8343e36
// Gy: 0 d51fbc6c 71a0094f a2cdd545 b11c5c0c 797324f1
static u8 _g163[] = { 0x04, 0x03, 0xf0, 0xeb, 0xa1, 0x62, 0x86, 0xa2, 0xd5, 0x7e, 0xa0, 0x99, 0x11,
    0x68, 0xd4, 0x99, 0x46, 0x37, 0xe8, 0x34, 0x3e, 0x36, 0x00, 0xd5, 0x1f, 0xbc, 0x6c, 0x71, 0xa0,
    0x09, 0x4f, 0xa2, 0xcd, 0xd5, 0x45, 0xb1, 0x1c, 0x5c, 0x0c, 0x79, 0x73, 0x24, 0xf1 };

static u8 _seed163[] = { 0x0 };

// a: 00 00000000 00000000 00000000 00000000 00000001
static u8 _a163[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };
// b: 02 0a601907 b8c953ca 1481eb10 512f7874 4a3205fd
static u8 _b163[] = { 0x02, 0x0a, 0x60, 0x19, 0x07, 0xb8, 0xc9, 0x53, 0xca, 0x14, 0x81, 0xeb, 0x10,
    0x51, 0x2f, 0x78, 0x74, 0x4a, 0x32, 0x05, 0xfd };

// n = 5846006549323611672814742442876390689256843201587
static u8 _n163[] = { 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x02, 0x92, 0xfe, 0x77, 0xe7, 0x0c, 0x12, 0xa4, 0x23, 0x4c, 0x33 };

#endif /*B163_OS_H_*/
