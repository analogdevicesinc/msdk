/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
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
 * @file       infoblock.h
 * @brief      Infoblock interface
 * @details    
 *      This driver can be used to interface with the infoblock,
 *      reading and writing select locations.
 */

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_INFOBLOCK_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_INFOBLOCK_H_

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(TRUE) || !defined(FALSE)
#define TRUE 1
#define FALSE 0
#endif
/**
 * @defgroup    infoblock_access Information Block (OTP) read interface
 * @brief       Read access for information block
 * @{
 */

/**
 * @defgroup    infoblock_defines Information Block defines
 * @brief       Registers, Bit Masks and Bit Positions for the Infoblock.
 * @details     Defines used for accessing the information block
 * @{
 */

/** 
 * @brief MAX32655 and some other parts have 128-bit wide flash, but hardware treats it as 64-bits/8 bytes for checksum purposes
 */
#define INFOBLOCK_LINE_SIZE 8

/**
 * @brief Checksum overhead (\#bytes) in an infoblock line, reduces amount of data that can be stored
 */
#define INFOBLOCK_LINE_OVERHEAD 2

/**
 * @brief 8 bytes, arranged as 4 16-bit uints 
 */
#define INFOBLOCK_ICE_LOCK_SIZE 8

/** 
 * @brief MAX32655 has 128-bit wide flash so the permanent line lock bit is at the top of every 16 bytes
 */
#define INFOBLOCK_WRITE_LOCK_LINE_SIZE 16

/** 
 * @brief Unprogrammed flash reads as all 0xFF
 */
#define ICELOCK_UNMODIFIED_VALUE 0xFFFF

/** 
 * @brief The value hardware looks for to lock the SWD in even numbered 16-bit locations (0-based so locations 0 and 2).
 */
#define ICELOCK_EVEN_LOCK_VALUE 0xA5A5

/** 
 * @brief The value hardware looks for to lock the SWD in odd numbered 16-bit locations (0-based so locations 1 and 3)
 */
#define ICELOCK_ODD_LOCK_VALUE 0x5A5A

/** 
 * @brief Maximum information block read size.  Used in infoblock_read() to
 *  set upper limit on bytes read.
 */
#define INFOBLOCK_MAXIMUM_READ_LENGTH 64

/** 
 * @brief The offset inside the information block where the USN (Universal Serial Number) is stored
 */
#define INFOBLOCK_USN_OFFSET 0x00

/** 
 * @brief The offset inside the information block where the SWD locking information is stored
 * @note There are four locking locations starting at 0x30 for location 0, 0x32 for location 1, 0x34 for location 2, and 0x36 for location 3
 */
#define INFOBLOCK_ICE_LOCK_OFFSET 0x30

/**
 * @brief The minimum number of locations with a lock value to cause SWD to be locked out.
 */
#define INFOBLOCK_ICE_LOCK_MINIMUM 1

/**
 * @brief The offset inside the information block for Storage of ECDSA public key
 */
#define INFOBLOCK_KEY_OFFSET 0x1000
/**
 * @brief The length in bytes of the ECDSA public key
 * @note This is the raw length. Once stored, 2 of every 8 bytes is used for CRC15, so the stored length is greater than 64 bytes.
 */
#define INFOBLOCK_KEY_SIZE 64

/** 
 * @brief Storage locations for feature enables.
 *  The value 0x5a5aa5a5_5a5aa5a5 designates enable or disable depending on the function.
 *  No CRC15 format in this case.
 */
#define INFOBLOCK_ENABLE_SIZE (8)

/**
 * @brief Standard information block enable/disable pattern is 0x5A5AA5A5
 */
#define INFOBLOCK_ENABLE_PATTERN 0x5A5AA5A5

/**
 * @brief Three information block line types
 *  
 *  USN: No CRC15, ignore lowest 15 bits [14:0]
 *
 *  RAW: No CRC15
 *
 *  DESIGN: Use CRC15 error detection in bits [62:48].
 */
typedef enum {
    INFOBLOCK_LINE_FORMAT_USN, /**< USN format is unique, ignore lowest 15 bits */
    INFOBLOCK_LINE_FORMAT_RAW, /**< Raw data format, use all bits */
    INFOBLOCK_LINE_FORMAT_DESIGN, /**< Design format, has CRC15 in bits 62:48, bit 63 is a line locking bit */
} lineformat_e;

/**@} end of group infoblock_defines */

/**
 * @brief crc15_highbitinput    Calculate CRC15 on data bits
 * @param[out]  crc15val    Calling routine must supply a pointer to an array of at least 32 btyes. The hash digest will be placed there.
 * @param[in]   input       pointer to the array of data to CRC15
 * @param[in]   bitlength   length in bits of the data to CRC15
 * @return      crc15val
 */
uint16_t crc15_highbitinput(uint16_t crc15val, uint8_t *input, int bitlength);

/**
 * @brief infoblock_readraw    Read raw data from information block
 * @param[in]   offset  location in the infoblock to read, this is a relative offset
 * @param[out]  data    pointer to array where data will be stored.
 * @return      error_code    error if unable to access infoblock
 * @retval      E_NO_ERROR    read was successful
 */
int infoblock_readraw(uint32_t offset, uint8_t *data);

/**
 * @brief infoblock_read    Read formatted data from information block
 * @note        This routine uses the offest to determine the format of the data,
 *              and will read the correct number of raw bytes be able to return
 *              the number of data bytes specified. The CRC15 and formatting will
 *              be removed and the data returned.
 * @param[in]   offset  location in the infoblock to read, this is a relative offset
 * @param[out]  data    pointer to array where data will be stored.
 * @param[in]   length  number of bytes to read
 * @return      error_code    error if unable to access infoblock
 * @retval      E_NO_ERROR    read was successful
 * @retval      E_BAD_PARAM   if incorrect length or offset is supplied
 */
int infoblock_read(uint32_t offset, uint8_t *data, int length);

/**@} end of group infoblock_access */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32675_INFOBLOCK_H_
