/**
 * @file    ctb.h
 * @brief   Crypto Toolbox driver.
 */

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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_CTB_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_CTB_H_

/***** Includes *****/
#include "ctb_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup ctb CTB
 * @ingroup periphlibs
 * @{
 */

/* IN ADDITION TO THIS HEADER, FCL WILL BE SUPPORTED AND PROVIDED IN BINARY FORM */
/***** Definitions *****/

/**
 * @brief Callback funtion for ctb
 * 
 */
typedef void (*mxc_ctb_complete_cb_t)(void *req, int result);

/**
  * @brief  Enumeration type for Crypto Toolbox features
  *
  */
typedef enum {
    MXC_CTB_FEATURE_DMA = 1 << 0,
    MXC_CTB_FEATURE_ECC = 1 << 1,
    MXC_CTB_FEATURE_CRC = 1 << 2,
    MXC_CTB_FEATURE_HASH = 1 << 4,
    MXC_CTB_FEATURE_CIPHER = 1 << 5,
    MXC_CTB_FEATURE_TRNG = 1 << 6
} mxc_ctb_features_t;

/* ************************************************************************* */
/* DMA Definitions                                                           */
/* ************************************************************************* */

/**
  * @brief  Structure for using DMA with CTB
  *
  */
struct _mxc_ctb_dma_req_t {
    uint8_t *sourceBuffer; ///< pointer to source data
    uint8_t *destBuffer; ///< pointer to destination buffer
    uint32_t length; ///< length of source data
    mxc_ctb_complete_cb_t callback; ///< Null callback indicates a blocking operation
} typedef mxc_ctb_dma_req_t;

/**
  * @brief  Enumeration type to select read source channel of DMA
  *
  */
typedef enum {
    MXC_CTB_DMA_READ_FIFO_DMA = MXC_V_CTB_CTRL_RDSRC_DMAORAPB,
    MXC_CTB_DMA_READ_FIFO_RNG = MXC_V_CTB_CTRL_RDSRC_RNG
} mxc_ctb_dma_read_source_t;

/**
  * @brief  Enumeration type to select write source channel of DMA
  *
  */
typedef enum {
    MXC_CTB_DMA_WRITE_FIFO_CIPHER = MXC_V_CTB_CTRL_WRSRC_CIPHEROUTPUT,
    MXC_CTB_DMA_WRITE_FIFO_READ_FIFO = MXC_V_CTB_CTRL_WRSRC_READFIFO,
    MXC_CTB_DMA_WRITE_FIFO_NONE = MXC_V_CTB_CTRL_WRSRC_NONE
} mxc_ctb_dma_write_source_t;

/* ************************************************************************* */
/* ECC Definitions                                                           */
/* ************************************************************************* */

/**
  * @brief  Structure used to set up ECC request
  *
  */
struct _mxc_ctb_ecc_req_t {
    uint8_t *dataBuffer;
    uint32_t dataLen;
    uint32_t checksum;
    mxc_ctb_complete_cb_t callback;
} typedef mxc_ctb_ecc_req_t;

/**
  * @brief  Structure used to set up CRC request
  *
  */
struct _mxc_ctb_crc_req_t {
    uint8_t *dataBuffer;
    uint32_t dataLen;
    uint32_t resultCRC;
    mxc_ctb_complete_cb_t callback;
} typedef mxc_ctb_crc_req_t;

/** 
 * @brief CRC data bit order
 *  
 */
typedef enum { MXC_CTB_CRC_LSB_FIRST, MXC_CTB_CRC_MSB_FIRST } mxc_ctb_crc_bitorder_t;

/* ************************************************************************* */
/* Hash Definitions                                                                            */
/* ************************************************************************* */

/**
  * @brief  Structure used to set up Hash request
  *
  */
struct _mxc_ctb_hash_req_t {
    uint8_t *msg;
    uint32_t len;
    uint8_t *hash;
    mxc_ctb_complete_cb_t callback;
} typedef mxc_ctb_hash_req_t;

/**
  * @brief  Enumeration type to select Hash function
  *
  */
typedef enum {
    MXC_CTB_HASH_DIS = MXC_V_CTB_HASH_CTRL_HASH_DIS, // Disable
    MXC_CTB_HASH_SHA1 = MXC_V_CTB_HASH_CTRL_HASH_SHA1, // Select SHA1
    MXC_CTB_HASH_SHA224 = MXC_V_CTB_HASH_CTRL_HASH_SHA224, // Select SHA224
    MXC_CTB_HASH_SHA256 = MXC_V_CTB_HASH_CTRL_HASH_SHA256, // Select SHA256
    MXC_CTB_HASH_SHA384 = MXC_V_CTB_HASH_CTRL_HASH_SHA384, // Select SHA384
    MXC_CTB_HASH_SHA512 = MXC_V_CTB_HASH_CTRL_HASH_SHA512 // Select SHA384
} mxc_ctb_hash_func_t;

/**
  * @brief  Enumeration type to select FIFO source for Hash
  *
  */
typedef enum {
    MXC_CTB_HASH_SOURCE_INFIFO = 0,
    MXC_CTB_HASH_SOURCE_OUTFIFO = 1
} mxc_ctb_hash_source_t;

/* ************************************************************************* */
/* Cipher Definitions                                                                          */
/* ************************************************************************* */

/**
  * @brief  Structure used to set up Cipher request
  *
  */
struct _mxc_ctb_cipher_req_t {
    uint8_t *plaintext;
    uint32_t ptLen;
    uint8_t *iv;
    uint8_t *ciphertext;
    mxc_ctb_complete_cb_t callback;
} typedef mxc_ctb_cipher_req_t;

/**
  * @brief  Enumeration type to select Cipher mode
  *
  */
typedef enum {
    MXC_CTB_MODE_ECB = MXC_V_CTB_CIPHER_CTRL_MODE_ECB, ///< Electronic Code Book
    MXC_CTB_MODE_CBC = MXC_V_CTB_CIPHER_CTRL_MODE_CBC, ///< Cipher Block Chaining
    MXC_CTB_MODE_CFB = MXC_V_CTB_CIPHER_CTRL_MODE_CFB, ///< Cipher Feedback
    MXC_CTB_MODE_CTR = MXC_V_CTB_CIPHER_CTRL_MODE_CTR, ///< Counter
    MXC_CTB_MODE_OFB = MXC_V_CTB_CIPHER_CTRL_MODE_OFB ///< Output Feedback
} mxc_ctb_cipher_mode_t;

/**
  * @brief  Enumeration type to select Cipher function
  *
  */
typedef enum {
    MXC_CTB_CIPHER_DIS = MXC_V_CTB_CIPHER_CTRL_CIPHER_DIS, ///< Disable
    MXC_CTB_CIPHER_AES128 = MXC_V_CTB_CIPHER_CTRL_CIPHER_AES128, ///< Select AES-128
    MXC_CTB_CIPHER_AES192 = MXC_V_CTB_CIPHER_CTRL_CIPHER_AES192, ///< Select AES-192
    MXC_CTB_CIPHER_AES256 = MXC_V_CTB_CIPHER_CTRL_CIPHER_AES256, ///< Select AES-256
    MXC_CTB_CIPHER_DES = MXC_V_CTB_CIPHER_CTRL_CIPHER_DES, ///< Select DES
    MXC_CTB_CIPHER_TDES = MXC_V_CTB_CIPHER_CTRL_CIPHER_TDES ///< Select TDES
} mxc_ctb_cipher_t;

/**
  * @brief  Enumeration type to select Cipher key
  *
  */
typedef enum {
    MXC_CTB_CIPHER_KEY_SOFTWARE = 0,
    MXC_CTB_CIPHER_KEY_AES_KEY2 = 2,
    MXC_CTB_CIPHER_KEY_AES_KEY3 = 3
} mxc_ctb_cipher_key_t;

/** 
 * @brief Cipher operation
 *  
 */
typedef enum {
    MXC_CTB_CIPHER_ENCRYPTION = 0,
    MXC_CTB_CIPHER_DECRYPTION
} mxc_ctb_cipher_operation_t;

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/**
 * @brief   Enable portions of the CTB
 *
 * @param   features  bit banded value indicating features to enable
 *                    see \ref mxc_ctb_features_t for a list of features
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_Init(uint32_t features);

/**
 * @brief   Detects what CTB features exist, see \ref mxc_ctb_features_t
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
uint32_t MXC_CTB_CheckFeatures(void);

/**
 * @brief   Enable CTB Interrupts
 *
 */
void MXC_CTB_EnableInt(void);

/**
 * @brief   Disable CTB Interrupts
 *
 */
void MXC_CTB_DisableInt(void);

/**
 * @brief   Checks the global CTB Ready Status
 *
 * @return  Nonzero if ready, zero if not ready or \ref MXC_Error_Codes.
 */
int MXC_CTB_Ready(void);

/**
 * @brief   Clears the selected feature's done bits, see \ref mxc_ctb_features_t
 *
 * @param   features   bit banded value indicating features to clear
 */
void MXC_CTB_DoneClear(uint32_t features);

/**
 * @brief   Returns CTB features showing operations complete, see \ref mxc_ctb_features_t
 *
 * @return  CTB features showing operations complete, see \ref mxc_ctb_features_t.
 */
uint32_t MXC_CTB_Done(void);

/**
 * @brief   Resets the selected features, see \ref mxc_ctb_features_t
 *
 * @param   features   bit banded value indicating features to reset
 */
void MXC_CTB_Reset(uint32_t features);

/**
 * @brief   Invalidates the CTB's internal cache.
 * @note    For best security, should be done after every operation
 */
void MXC_CTB_CacheInvalidate(void);

/**
 * @brief   Disable and reset portions of the CTB
 *
 * @param   features  bit banded value indicating features to shutdown
 *                    see \ref mxc_ctb_features_t for a list of features
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_Shutdown(uint32_t features);

/**
 * @brief   Check which CTB features are enabled
 *
 * @return  CTB features showing features enabled, see \ref mxc_ctb_features_t.
 */
uint32_t MXC_CTB_GetEnabledFeatures(void);

/**
 * @brief   This function should be called from the CTB ISR Handler
 *          when using Async functions
 */
void MXC_CTB_Handler(void);

/************************************/
/* CTB DMA - Used for all features  */
/************************************/

/**
 * @brief   Set the source the DMA reads from
 * @note    The DMA is unable to read directly from Flash
 *
 * @param   source    The source of the data for DMA read operations
 *                    see \ref mxc_ctb_dma_read_source_t for a list of sources
 */
void MXC_CTB_DMA_SetReadSource(mxc_ctb_dma_read_source_t source);

/**
 * @brief   Get the source the DMA reads from
 *
 * @return  The source of the data for DMA read operations
 *          see \ref mxc_ctb_dma_read_source_t for a list of sources
 */
mxc_ctb_dma_read_source_t MXC_CTB_DMA_GetReadSource(void);

/**
 * @brief   Set the source the DMA write fifo reads from
 *
 * @param   source    The source of the data for DMA write operations
 *                    see \ref mxc_ctb_dma_write_source_t for a list of sources
 */
void MXC_CTB_DMA_SetWriteSource(mxc_ctb_dma_write_source_t source);

/**
 * @brief   Set the source the DMA write fifo reads from
 *
 * @return  The source of the data for DMA write operations
 *          see \ref mxc_ctb_dma_write_source_t for a list of sources
 */
mxc_ctb_dma_write_source_t MXC_CTB_DMA_GetWriteSource(void);

/**
 * @brief   Set the source address of the DMA
 * @note    This is only applicable when the read source is memory
 *          The DMA is unable to read directly from Flash
 *
 * @param   source    pointer to the source location
 */
void MXC_CTB_DMA_SetSource(uint8_t *source);

/**
 * @brief   Set the destination address of the DMA
 *
 * @param   dest  pointer to destination
 */
void MXC_CTB_DMA_SetDestination(uint8_t *dest);

/**
 * @brief   Set the source and destination addresses of the DMA
 *
 * @param   req   request structure that contains the source and destination
 *                information. A destination address of NULL will indicate
 *                that the Read Source has been set as something other than memory.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_DMA_SetupOperation(mxc_ctb_dma_req_t *req);

/**
 * @brief   Start a DMA transfer defined by the request object
 *          Blocks until completion
 *
 * @param   req   request structure that contains the source and destination
 *                information. A destination address of NULL will indicate
 *                that the Read Source has been set as something other than memory.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_DMA_DoOperation(mxc_ctb_dma_req_t *req);

/**
 * @brief   Start a DMA transfer of fixed size
 *
 * @param   length Number of bytes to transfer
 *
 */
void MXC_CTB_DMA_StartTransfer(uint32_t length);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

/**
 * @brief   Get a random number
 *
 * @return  A random 32-bit number
 */
int MXC_CTB_TRNG_RandomInt(void);

/**
 * @brief   Get a random number of length len
 *
 * @param   data    Pointer to a location to store the number
 * @param   len     Length of random number in bytes
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_TRNG_Random(uint8_t *data, uint32_t len);

/**
 * @brief   Get a random number of length len, do not block while generating data
 * @note    The user must call MXC_CTB_Handler() in the ISR
 *
 * @param   data      Pointer to a location to store the number
 * @param   len       Length of random number in bytes
 * @param   callback  Function that will be called when all data has been generated
 *
 */
void MXC_CTB_TRNG_RandomAsync(uint8_t *data, uint32_t len, mxc_ctb_complete_cb_t callback);

/* ************************************************************************* */
/* Error Correction Code (ECC) functions                                     */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

/**
 * @brief   Enable ECC Calculation
 * @note    ECC calculation is shared with CRC, when ECC is enabled, CRC
 *          computation is not possible
 */
void MXC_CTB_ECC_Enable(void);

/**
 * @brief   Disable ECC Calculation
 * @note    ECC calculation is shared with CRC, when ECC is enabled, CRC
 *          computation is not possible
 */
void MXC_CTB_ECC_Disable(void);

/**
 * @brief   Get the Result of an ECC Calculation
 *
 * @return  The result of the ECC calculation
 */
uint32_t MXC_CTB_ECC_GetResult(void);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Compute the ECC value for a block of data up to 8kB in size
 * @note    This function places the computed ECC value in the appropriate
 *          place in the mxc_ctb_ecc_req_t structure
 *
 * @param   req   Structure containing data for the ECC request
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_ECC_Compute(mxc_ctb_ecc_req_t *req);

/**
 * @brief   Check for single or dual bit errors in a block of data
 * @note    This function will also correct single bit errors as needed
 *
 * @param   req   Structure containing data for the ECC request
 *
 * @return  Positive values for 1 or 2 bit errors, respectively
 *          otherwise, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_ECC_ErrorCheck(mxc_ctb_ecc_req_t *req);

/**
 * @brief   Compute the ECC value for a block of data up to 8kB in size
 * @note    This function places the computed ECC value in the appropriate
 *          place in the mxc_ctb_ecc_req_t structure. The user needs to call
 *          MXC_CTB_Handler() in the ISR
 *
 * @param   req   Structure containing data for the ECC request
 */
void MXC_CTB_ECC_ComputeAsync(mxc_ctb_ecc_req_t *req);

/**
 * @brief   Check for single or dual bit errors in a block of data
 * @note    This function will also correct single bit errors as needed
 *          The user must call MXC_CTB_Handler() in the ISR.
 *
 * @param   req   Structure containing data for the ECC request
 */
void MXC_CTB_ECC_ErrorCheckAsync(mxc_ctb_ecc_req_t *req);

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @param   bitOrder  The direction to perform CRC calculation in, \ref mxc_ctb_crc_bitorder_t
 */
void MXC_CTB_CRC_SetDirection(mxc_ctb_crc_bitorder_t bitOrder);

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @return  The direction of calculation, 1 for MSB first, 0 for LSB first , \ref mxc_ctb_crc_bitorder_t
 */
mxc_ctb_crc_bitorder_t MXC_CTB_CRC_GetDirection(void);

/**
 * @brief   Set the Polynomial for CRC calculation
 *
 * @param   poly  The polynomial to use for CRC calculation
 */
void MXC_CTB_CRC_SetPoly(uint32_t poly);

/**
 * @brief   Get the polynomial for CRC calculation
 *
 * @return  The polynomial used in calculation
 */
uint32_t MXC_CTB_CRC_GetPoly(void);

/**
 * @brief   Get the result of a CRC calculation
 *
 * @return  The calculated CRC value
 */
uint32_t MXC_CTB_CRC_GetResult(void);

/**
 * @brief   Set the intial value used (the seed) when starting a CRC computation.
 *
 * @param   seed  The value to seed the CRC generator with
 */
void MXC_CTB_CRC_SetInitialValue(uint32_t seed);

/**
 * @brief   Set the value that will be bitwise XORed with the final output from the CRC computation.  Use 0 to skip the XOR step.
 *
 * @param   xor  The value that will be XORed with the CRC
 */
void MXC_CTB_CRC_SetFinalXORValue(uint32_t xor);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Perform a CRC computation
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_ctb_crc_req_t structure
 *
 * @param   req   Structure containing the data for calculation
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_CRC_Compute(mxc_ctb_crc_req_t *req);

/**
 * @brief   Perform a CRC computation asynchronously
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_ctb_crc_req_t structure. The user must call
 *          MXC_CTB_Handler() in the ISR
 *
 * @param   req   Structure containing the data for calculation
 */
void MXC_CTB_CRC_ComputeAsync(mxc_ctb_crc_req_t *req);

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/***********************/
/* Low Level Functions */
/***********************/

/**
 * @brief   Get the block size for a given hash function
 *
 * @param   function  See \ref mxc_ctb_hash_func_t for options
 *
 * @return  Block size in bytes
 */
unsigned int MXC_CTB_Hash_GetBlockSize(mxc_ctb_hash_func_t function);

/**
 * @brief   Get the digest size for a given hash function
 *
 * @param   function  See \ref mxc_ctb_hash_func_t for options
 *
 * @return  Digest size in bytes
 */
unsigned int MXC_CTB_Hash_GetDigestSize(mxc_ctb_hash_func_t function);

/**
 * @brief   Set the algorithm to use for hash computation
 *
 * @param   function  See \ref mxc_ctb_hash_func_t for options
 */
void MXC_CTB_Hash_SetFunction(mxc_ctb_hash_func_t function);

/**
 * @brief   Get the algorithm to use for hash computation
 *
 * @return  See \ref mxc_ctb_hash_func_t for options
 */
mxc_ctb_hash_func_t MXC_CTB_Hash_GetFunction(void);

/**
 * @brief   Set whether to use automatic padding of the input data
 * @note    The padding procedure used by hardware is described in the users guide
 *
 * @param   pad   Use hardware padding of the data
 */
void MXC_CTB_Hash_SetAutoPad(int pad);

/**
 * @brief   Get whether to use automatic padding of the input data
 *
 * @return  Using hardware padding of the data
 */
int MXC_CTB_Hash_GetAutoPad(void);

/**
 * @brief   Get the result of a hash computation
 *
 * @param   digest   buffer to store the ouctbt of the hash algorithm
 * @param   len      location to store the length of the digest
 */
void MXC_CTB_Hash_GetResult(uint8_t *digest, int *len);

/**
 * @brief   Set the size of the data input into the hash computation
 * @note    Hash data size is software limited to ~3GB
 *
 * @param   size  Size of the data in bytes
 */
void MXC_CTB_Hash_SetMessageSize(uint32_t size);

/**
 * @brief   Set the source of data for the hash computation
 *
 * @param   source  see \ref mxc_ctb_hash_source_t for options
 */
void MXC_CTB_Hash_SetSource(mxc_ctb_hash_source_t source);

/**
 * @brief   Get the source of data for the hash computation
 *
 * @return  See \ref mxc_ctb_hash_source_t for options
 */
mxc_ctb_hash_source_t MXC_CTB_Hash_GetSource(void);

/**
 * @brief   Initialize the hash computation unit
 * @note    Call this after setting the hash function and message size
 *          This function blocks until load is complete
 *
 */
void MXC_CTB_Hash_InitializeHash(void);

/************************/
/* High Level Functions */
/************************/

/**
 * @brief   Compute a Hash Digest
 * @note    The computed digest will be stored in the req structure
 *
 * @param   req   Structure containing all data needed for a hash computation
 *
 * @return See \ref MXC_Error_Codes for a list of return codes
 */
int MXC_CTB_Hash_Compute(mxc_ctb_hash_req_t *req);

/**
 * @brief   Compute a Hash Digest
 * @note    The computed digest will be stored in the req structure. The user
 *          must call MXC_CTB_Handler() in the ISR.
 *
 * @param   req   Structure containing all data needed for a hash computation
 */
void MXC_CTB_Hash_ComputeAsync(mxc_ctb_hash_req_t *req);

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/************************/
/* Low Level Functions  */
/************************/

/**
 * @brief   Get the key size for a given cipher type
 *
 * @param   cipher   See \ref mxc_ctb_cipher_t for options
 *
 * @return  Size of the key in bytes
 */
unsigned int MXC_CTB_Cipher_GetKeySize(mxc_ctb_cipher_t cipher);

/**
 * @brief   Get the block size for a given cipher type
 *
 * @param   cipher   See \ref mxc_ctb_cipher_t for options
 *
 * @return  Size of the block in bytes
 */
unsigned int MXC_CTB_Cipher_GetBlockSize(mxc_ctb_cipher_t cipher);

/**
 * @brief   Set the block mode used for cipher operations
 *
 * @param   mode   See \ref mxc_ctb_cipher_mode_t for options
 */
void MXC_CTB_Cipher_SetMode(mxc_ctb_cipher_mode_t mode);

/**
 * @brief   Get the block mode used for cipher operations
 *
 * @return  See \ref mxc_ctb_cipher_mode_t for options
 */
mxc_ctb_cipher_mode_t MXC_CTB_Cipher_GetMode(void);

/**
 * @brief   Set the cipher type used for cipher operations
 *
 * @param   cipher   See \ref mxc_ctb_cipher_t for options
 */
void MXC_CTB_Cipher_SetCipher(mxc_ctb_cipher_t cipher);

/**
 * @brief   Get the cipher type used for cipher operations
 *
 * @return  See \ref mxc_ctb_cipher_t for options
 */
mxc_ctb_cipher_t MXC_CTB_Cipher_GetCipher(void);

/**
 * @brief   Set the source of the key used in cipher operations
 *
 * @param   source   See \ref mxc_ctb_cipher_key_t for options
 */
void MXC_CTB_Cipher_SetKeySource(mxc_ctb_cipher_key_t source);

/**
 * @brief   Get the cipher type used for cipher operations
 *
 * @return  See \ref mxc_ctb_cipher_key_t for options
 */
mxc_ctb_cipher_key_t MXC_CTB_Cipher_GetKeySource(void);

/**
 * @brief   Load the cipher key from the selected source
 *
 */
void MXC_CTB_Cipher_LoadKey(void);

/**
 * @brief   Configure for encryption or decryption
 *
 * @param   operation Set to perform encryption/decryption \ref mxc_ctb_cipher_operation_t
 */
void MXC_CTB_Cipher_SetOperation(mxc_ctb_cipher_operation_t operation);

/**
 * @brief   Set the cipher key
 * @note    This only takes effect if software is the selected key source
 *
 * @param   key   buffer containing key
 * @param   len   length of key (dependent on cipher used)
 */
void MXC_CTB_Cipher_SetKey(uint8_t *key, uint32_t len);

/**
 * @brief   Set the initial value used for cipher operations
 *
 * @param   iv   buffer containing iv
 * @param   len  length of initial value
 */
void MXC_CTB_Cipher_SetIV(uint8_t *iv, uint32_t len);

/**
 * @brief   Get the initial value used for cipher operations
 *
 * @param   ivOut   buffer containing iv
 * @param   len     length of buffer
 */
void MXC_CTB_Cipher_GetIV(uint8_t *ivOut, uint32_t len);

/************************/
/* High Level Functions */
/************************/

/**
 * @brief   Perform an encryption using the cipher feature
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the encryption
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_Cipher_Encrypt(mxc_ctb_cipher_req_t *req);

/**
 * @brief   Perform a decryption using the cipher feature
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the decryption
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_CTB_Cipher_Decrypt(mxc_ctb_cipher_req_t *req);

/**
 * @brief   Perform an encryption using the cipher feature
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_CTB_Handler() in the ISR
 *
 * @param   req  Structure containing data for the encryption
 */
void MXC_CTB_Cipher_EncryptAsync(mxc_ctb_cipher_req_t *req);

/**
 * @brief   Perform a decryption using the cipher feature
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_CTB_Handler() in the ISR
 *
 * @param   req  Structure containing data for the decryption
 */
void MXC_CTB_Cipher_DecryptAsync(mxc_ctb_cipher_req_t *req);

#ifdef __cplusplus
}
#endif
/**@} end of group ctb */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32690_CTB_H_
