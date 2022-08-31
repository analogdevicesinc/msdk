/**
 * @file
 * @brief   Trust Protection Unit driver.
 */

/* ****************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date$
 * $Revision$
 *
 *************************************************************************** */

#ifndef _TPU_H_
#define _TPU_H_

/***** Includes *****/
#include "tpu_regs.h"

/* IN ADDITION TO THIS HEADER, FCL WILL BE SUPPORTED AND PROVIDED IN BINARY FORM */
/***** Definitions *****/

typedef void (*mxc_tpu_complete_t)(void* req, int result);

typedef enum {
    MXC_TPU_FEATURE_DMA = 1 << 0,
    MXC_TPU_FEATURE_ECC = 1 << 1,
    MXC_TPU_FEATURE_CRC = 1 << 2,
    MXC_TPU_FEATURE_MAA = 1 << 3,
    MXC_TPU_FEATURE_HASH = 1 << 4,
    MXC_TPU_FEATURE_CIPHER = 1 << 5,
    MXC_TPU_FEATURE_TRNG = 1 << 6
} mxc_tpu_features_t;

/* ************************************************************************* */
/* DMA Definitions                                                           */
/* ************************************************************************* */

struct _mxc_tpu_dma_req_t {
    uint8_t* sourceBuffer;
    uint8_t* destBuffer;
    uint32_t length;
    mxc_tpu_complete_t callback; // Null callback indicates a blocking operation
} typedef mxc_tpu_dma_req_t;

typedef enum {
    MXC_TPU_DMA_READ_FIFO_DMA = MXC_V_TPU_CTRL_RDSRC_DMAORAPB,
    MXC_TPU_DMA_READ_FIFO_RNG = MXC_V_TPU_CTRL_RDSRC_RNG
} mxc_tpu_dma_read_source_t;

typedef enum {
    MXC_TPU_DMA_WRITE_FIFO_CIPHER = MXC_V_TPU_CTRL_WRSRC_CIPHEROUTPUT,
    MXC_TPU_DMA_WRITE_FIFO_READ_FIFO = MXC_V_TPU_CTRL_WRSRC_READFIFO,
    MXC_TPU_DMA_WRITE_FIFO_NONE = MXC_V_TPU_CTRL_WRSRC_NONE
} mxc_tpu_dma_write_source_t;

/* ************************************************************************* */
/* ECC Definitions                                                           */
/* ************************************************************************* */

struct _mxc_tpu_ecc_req_t {
    uint8_t* dataBuffer;
    uint32_t dataLen;
    uint32_t checksum;
    mxc_tpu_complete_t callback;
} typedef mxc_tpu_ecc_req_t;

struct _mxc_tpu_crc_req_t {
    uint8_t* dataBuffer;
    uint32_t dataLen;
    uint32_t resultCRC;
    mxc_tpu_complete_t callback;
} typedef mxc_tpu_crc_req_t;

/* ************************************************************************* */
/* MAA Definitions                                                                             */
/* ************************************************************************* */

typedef enum {
    MXC_TPU_MAA_EXP = MXC_V_TPU_MAA_CTRL_CLC_EXP, // Select exponentiation operation
    MXC_TPU_MAA_SQ = MXC_V_TPU_MAA_CTRL_CLC_SQ, // Select square operation
    MXC_TPU_MAA_MUL = MXC_V_TPU_MAA_CTRL_CLC_MUL, // Select multiplication operation
    MXC_TPU_MAA_SQMUL
    = MXC_V_TPU_MAA_CTRL_CLC_SQMUL, // Select square followed by multiplication operation
    MXC_TPU_MAA_ADD = MXC_V_TPU_MAA_CTRL_CLC_ADD, // Select add operation
    MXC_TPU_MAA_SUB = MXC_V_TPU_MAA_CTRL_CLC_SUB, // Select subtract operation
    MXC_TPU_MAA_INV // Composite operation, modular exponentiation
} mxc_tpu_maa_operation_t;

struct _mxc_tpu_maa_req_t {
    // All operand buffers must be as long as the word size
    uint8_t* opA;
    uint8_t* opB;
    uint8_t* exponent;
    uint8_t* modulus;
    uint8_t* result;
    mxc_tpu_maa_operation_t op;
    mxc_tpu_complete_t callback;
} typedef mxc_tpu_maa_req_t;

typedef enum {
    MXC_TPU_MAA_REG_OPA = MXC_F_TPU_MAA_CTRL_AMS_POS,
    MXC_TPU_MAA_REG_OPB = MXC_F_TPU_MAA_CTRL_BMS_POS,
    MXC_TPU_MAA_REG_EXP = MXC_F_TPU_MAA_CTRL_EMS_POS,
    MXC_TPU_MAA_REG_MOD = MXC_F_TPU_MAA_CTRL_MMS_POS
} mxc_tpu_maa_register_t;

/* ************************************************************************* */
/* Hash Definitions                                                                            */
/* ************************************************************************* */

struct _mxc_tpu_hash_req_t {
    uint8_t* msg;
    uint32_t len;
    uint8_t* hash;
    mxc_tpu_complete_t callback;
} typedef mxc_tpu_hash_req_t;

typedef enum {
    MXC_TPU_HASH_DIS = MXC_V_TPU_HASH_CTRL_HASH_DIS, // Disable
    MXC_TPU_HASH_SHA1 = MXC_V_TPU_HASH_CTRL_HASH_SHA1, // Select SHA1
    MXC_TPU_HASH_SHA224 = MXC_V_TPU_HASH_CTRL_HASH_SHA224, // Select SHA224
    MXC_TPU_HASH_SHA256 = MXC_V_TPU_HASH_CTRL_HASH_SHA256, // Select SHA256
    MXC_TPU_HASH_SHA384 = MXC_V_TPU_HASH_CTRL_HASH_SHA384, // Select SHA384
    MXC_TPU_HASH_SHA512 = MXC_V_TPU_HASH_CTRL_HASH_SHA512 // Select SHA384
} mxc_tpu_hash_func_t;

typedef enum {
    MXC_TPU_HASH_SOURCE_INFIFO = 0,
    MXC_TPU_HASH_SOURCE_OUTFIFO = 1
} mxc_tpu_hash_source_t;

/* ************************************************************************* */
/* Cipher Definitions                                                                          */
/* ************************************************************************* */

struct _mxc_tpu_cipher_req_t {
    uint8_t* plaintext;
    uint32_t ptLen;
    uint8_t* iv;
    uint8_t* ciphertext;
    mxc_tpu_complete_t callback;
} typedef mxc_tpu_cipher_req_t;

typedef enum {
    MXC_TPU_MODE_ECB = MXC_V_TPU_CIPHER_CTRL_MODE_ECB, // Electronic Code Book
    MXC_TPU_MODE_CBC = MXC_V_TPU_CIPHER_CTRL_MODE_CBC, // Cipher Block Chaining
    MXC_TPU_MODE_CFB = MXC_V_TPU_CIPHER_CTRL_MODE_CFB, // Cipher Feedback
    MXC_TPU_MODE_CTR = MXC_V_TPU_CIPHER_CTRL_MODE_CTR, // Counter
    MXC_TPU_MODE_OFB = /*????*/ 0 // Output Feedback
} mxc_tpu_cipher_mode_t;

typedef enum {
    MXC_TPU_CIPHER_DIS = MXC_V_TPU_CIPHER_CTRL_CIPHER_DIS, // Disable
    MXC_TPU_CIPHER_AES128 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES128, // Select AES-128
    MXC_TPU_CIPHER_AES192 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES192, // Select AES-192
    MXC_TPU_CIPHER_AES256 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES256, // Select AES-256
    MXC_TPU_CIPHER_DES = MXC_V_TPU_CIPHER_CTRL_CIPHER_DES, // Select DES
    MXC_TPU_CIPHER_TDES = MXC_V_TPU_CIPHER_CTRL_CIPHER_TDES // Select TDES
} mxc_tpu_cipher_t;

typedef enum {
    MXC_TPU_CIPHER_KEY_SOFTWARE = 0,
    MXC_TPU_CIPHER_KEY_AES_KEY2 = 2,
    MXC_TPU_CIPHER_KEY_AES_KEY3 = 3
} mxc_tpu_cipher_key_t;

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/**
 * @brief   Enable portions of the TPU
 *
 * @param   features  bit banded value indicating features to enable
 *                    see \ref mxc_tpu_features_t for a list of features
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_Init(uint32_t features);

/**
 * @brief   Detects what TPU features exist, see \ref mxc_tpu_features_t
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
uint32_t MXC_TPU_CheckFeatures(void);

/**
 * @brief   Enable TPU Interrupts
 *
 * @param   enable  enable the TPU interrupt
 */
void MXC_TPU_IntEnable(int enable);

/**
 * @brief   Checks the global TPU Ready Status
 *
 * @return  Nonzero if ready, zero if not ready.
 */
int MXC_TPU_Ready(void);

/**
 * @brief   Clears the selected feature's done bits, see \ref mxc_tpu_features_t
 *
 */
void MXC_TPU_DoneClear(uint32_t features);

/**
 * @brief   Returns TPU features showing operations complete, see \ref mxc_tpu_features_t
 *
 * @return  TPU features showing operations complete, see \ref mxc_tpu_features_t.
 */
uint32_t MXC_TPU_Done(void);

/**
 * @brief   Resets the selected features, see \ref mxc_tpu_features_t
 *
 */
void MXC_TPU_Reset(uint32_t features);

/**
 * @brief   Invalidates the TPU's internal cache.
 * @note    For best security, should be done after every operation
 *
 */
void MXC_TPU_CacheInvalidate(void);

/**
 * @brief   Disable and reset portions of the TPU
 *
 * @param   features  bit banded value indicating features to shutdown
 *                    see \ref mxc_tpu_features_t for a list of features
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_Shutdown(uint32_t features);

/**
 * @brief   Check which TPU features are enabled
 *
 * @return  TPU features showing features enabled, see \ref mxc_tpu_features_t.
 */
uint32_t MXC_TPU_GetEnabledFeatures(void);

/**
 * @brief   This function should be called from the TPU ISR Handler
 *          when using Async functions
 */
void MXC_TPU_Handler(void);

/************************************/
/* TPU DMA - Used for all features  */
/************************************/

/**
 * @brief   Set the source the DMA reads from
 * @note    The DMA is unable to read directly from Flash
 *
 * @param   source    The source of the data for DMA read operations
 *                    see \ref mxc_tpu_dma_read_source_t for a list of sources
 */
void MXC_TPU_DMA_SetReadSource(mxc_tpu_dma_read_source_t source);

/**
 * @brief   Get the source the DMA reads from
 *
 * @return  The source of the data for DMA read operations
 *          see \ref mxc_tpu_dma_read_source_t for a list of sources
 */
mxc_tpu_dma_read_source_t MXC_TPU_DMA_GetReadSource(void);

/**
 * @brief   Set the source the DMA write fifo reads from
 *
 * @param   source    The source of the data for DMA write operations
 *                    see \ref mxc_tpu_dma_write_source_t for a list of sources
 */
void MXC_TPU_DMA_SetWriteSource(mxc_tpu_dma_write_source_t source);

/**
 * @brief   Set the source the DMA write fifo reads from
 *
 * @return  The source of the data for DMA write operations
 *          see \ref mxc_tpu_dma_write_source_t for a list of sources
 */
mxc_tpu_dma_write_source_t MXC_TPU_DMA_GetWriteSource(void);

/**
 * @brief   Set the source address of the DMA
 * @note    This is only applicable when the read source is memory
 *          The DMA is unable to read directly from Flash
 *
 * @param   source    pointer to the source location
 */
void MXC_TPU_DMA_SetSource(uint8_t* source);

/**
 * @brief   Set the destination address of the DMA
 *
 * @param   dest  pointer to destination
 */
void MXC_TPU_DMA_SetDestination(uint8_t* dest);

/**
 * @brief   Set the source and destination addresses of the DMA
 *
 * @param   req   request structure that contains the source and destination
 *                information. A destination address of NULL will indicate
 *                that the Read Source has been set as something other than memory.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_DMA_SetupOperation(mxc_tpu_dma_req_t* req);

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
int MXC_TPU_DMA_DoOperation(mxc_tpu_dma_req_t* req);

/**
 * @brief   Start a DMA transfer of fixed size
 *
 * @param   length Number of bytes to transfer
 *
 */
void MXC_TPU_DMA_StartTransfer(uint32_t length);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

/**
 * @brief   Get a random number
 *
 * @return  A random 32-bit number
 */
int MXC_TPU_TRNG_RandomInt(void);

/**
 * @brief   Get a random number of length len
 *
 * @param   data    Pointer to a location to store the number
 * @param   len     Length of random number in bytes
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_TRNG_Random(uint8_t* data, uint32_t len);

/**
 * @brief   Get a random number of length len, do not block while generating data
 * @note    The user must call MXC_TPU_Handler() in the ISR
 *
 * @param   data      Pointer to a location to store the number
 * @param   len       Length of random number in bytes
 * @param   callback  Function that will be called when all data has been generated
 *
 */
void MXC_TPU_TRNG_RandomAsync(uint8_t* data, uint32_t len, mxc_tpu_complete_t callback);

/**
 * @brief   Use the TRNG to generate an AES key and place it in the hardware key store
 * @note    The key generated will not be visible to software.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_TRNG_Generate_AES(void);

/**
 * @brief   Perform an on-demand health test
 * @note    Blocks until health test is done
 *
 * @return  Zero for pass, one for fail
 */
int MXC_TPU_TRNG_HealthTest(void);

/**
 * @brief   Start an on-demand health test, and enable interrupts whenever one fails
 * @note    The user must call MXC_TPU_Handler() in the ISR
 *
 * @param   callback  Function that will be called when a health test fails
 */
void MXC_TPU_TRNG_HealthTestAsync(mxc_tpu_complete_t callback);

/* ************************************************************************* */
/* Error Correction Code (ECC) functions                                     */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

/**
 * @brief   Enable or Disable ECC Calculation
 * @note    ECC calculation is shared with CRC, when ECC is enabled, CRC
 *          computation is not possible
 *
 * @param   enable  enable the ECC calculation
 */
void MXC_TPU_ECC_Enable(int enable);

/**
 * @brief   Get the Result of an ECC Calculation
 *
 * @return  The result of the ECC calculation
 */
uint32_t MXC_TPU_ECC_GetResult(void);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Compute the ECC value for a block of data up to 8kB in size
 * @note    This function places the computed ECC value in the appropriate
 *          place in the mxc_tpu_ecc_req_t structure
 *
 * @param   req   Structure containing data for the ECC request
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_ECC_Compute(mxc_tpu_ecc_req_t* req);

/**
 * @brief   Check for single or dual bit errors in a block of data
 * @note    This function will also correct single bit errors as needed
 *
 * @param   req   Structure containing data for the ECC request
 *
 * @return  Positive values for 1 or 2 bit errors, respectively
 *          otherwise, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_ECC_ErrorCheck(mxc_tpu_ecc_req_t* req);

/**
 * @brief   Compute the ECC value for a block of data up to 8kB in size
 * @note    This function places the computed ECC value in the appropriate
 *          place in the mxc_tpu_ecc_req_t structure. The user needs to call
 *          MXC_TPU_Handler() in the ISR
 *
 * @param   req   Structure containing data for the ECC request
 */
void MXC_TPU_ECC_ComputeAsync(mxc_tpu_ecc_req_t* req);

/**
 * @brief   Check for single or dual bit errors in a block of data
 * @note    This function will also correct single bit errors as needed
 *          The user must call MXC_TPU_Handler() in the ISR.
 *
 * @param   req   Structure containing data for the ECC request
 */
void MXC_TPU_ECC_ErrorCheckAsync(mxc_tpu_ecc_req_t* req);

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @param   msbFirst  The direction to perform CRC calculation in
 */
void MXC_TPU_CRC_SetDirection(int msbFirst);

/**
 * @brief   Set the bit-order of CRC calculation
 *
 * @return  The direction of calculation, 1 for MSB first, 0 for LSB first
 */
int MXC_TPU_CRC_GetDirection(void);

/**
 * @brief   Set the Polynomial for CRC calculation
 *
 * @param   poly  The polynomial to use for CRC calculation
 */
void MXC_TPU_CRC_SetPoly(uint32_t poly);

/**
 * @brief   Get the polynomial for CRC calculation
 *
 * @return  The polynomial used in calculation
 */
uint32_t MXC_TPU_CRC_GetPoly(void);

/**
 * @brief   Get the result of a CRC calculation
 *
 * @return  The calculated CRC value
 */
uint32_t MXC_TPU_CRC_GetResult(void);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Perform a CRC computation
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_tpu_crc_req_t structure
 *
 * @param   req   Structure containing the data for calculation
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_CRC_Compute(mxc_tpu_crc_req_t* req);

/**
 * @brief   Perform a CRC computation asynchronously
 * @note    The result of the CRC calculation will be placed in the
 *          mxc_tpu_crc_req_t structure. The user must call
 *          MXC_TPU_Handler() in the ISR
 *
 * @param   req   Structure containing the data for calculation
 */
void MXC_TPU_CRC_ComputeAsync(mxc_tpu_crc_req_t* req);

/* ************************************************************************* */
/* Modular Arithmetic Accelerator (MAA) functions                           */
/* ************************************************************************* */

/*******************************/
/* Low Level Functions         */
/*******************************/

/**
 * @brief   Get the physical address of an MAA segment
 *
 * @param   segment   index of the segment to use (0-4)
 *
 * @return  Pointer to the instance in memory
 */
uint32_t* MXC_TPU_MAA_GetAddress(int segment);

/**
 * @brief   Set the MAA RAM Segment used for the Temporary register
 *
 * @param   segment   index of the segment to use (0-4)
 */
void MXC_TPU_MAA_SetRAMTemporary(int segment);

/**
 * @brief   Get the MAA RAM Segment used for the Temporary register
 *
 * @return  Index of the segment to use (0-4)
 */
int MXC_TPU_MAA_GetRAMTemporary(void);

/**
 * @brief   Set the MAA RAM Segment used for the Result register
 *
 * @param   segment   index of the segment to use (0-4)
 */
void MXC_TPU_MAA_SetRAMResult(int segment);

/**
 * @brief   Get the MAA RAM Segment used for the Result register
 *
 * @return  Index of the segment to use (0-4)
 */
int MXC_TPU_MAA_GetRAMResult(void);

/**
 * @brief   Set the MAA RAM Segment used for the Op A register
 *
 * @param   segment   index of the segment to use (0-4)
 */
void MXC_TPU_MAA_SetRAMOperandA(int segment);

/**
 * @brief   Get the MAA RAM Segment used for the Op A register
 *
 * @return  Index of the segment to use (0-4)
 */
int MXC_TPU_MAA_GetRAMOperandA(void);

/**
 * @brief   Set the MAA RAM Segment used for the Op B register
 *
 * @param   segment   index of the segment to use (0-4)
 */
void MXC_TPU_MAA_SetRAMOperandB(int segment);

/**
 * @brief   Get the MAA RAM Segment used for the Op B register
 *
 * @return  Index of the segment to use (0-4)
 */
int MXC_TPU_MAA_GetRAMOperandB(void);

/**
 * @brief   Set the Memory Blinding Offset for a particular register
 *
 * @param   reg     The register to set the blind offset of
 * @param   blindIndex  The blinding offset to use
 */
void MXC_TPU_MAA_SetMemoryBlinding(mxc_tpu_maa_register_t reg, int blindIndex);

/**
 * @brief   Get the Memory Blinding Offset for a particular register
 *
 * @param   reg     The register to get the blind offset of
 *
 * @return  The blinding offset used
 */
int MXC_TPU_MAA_GetMemoryBlinding(mxc_tpu_maa_register_t reg);

// Set secure vs speed
/**
 * @brief   Control whether the MAA uses the secure, slower mode or
 *          a higher speed but less secure computation mode
 *
 * @param   secureMode use the secure, but slower, mode for computation
 */
void MXC_TPU_MAA_SetSecureMode(int secureMode);

/**
 * @brief   Get whether the MAA uses the secure, slower mode or
 *          a higher speed but less secure computation mode
 *
 * @return  Use the secure, but slower, mode for computation
 */
int MXC_TPU_MAA_GetSecureMode(void);

/**
 * @brief   Set the operation the MAA will perform
 * @note    Modular inversion is not a native operation,
 *          and not supported in the low level functions.
 *
 * @param   operation The MAA operation to perform
 */
void MXC_TPU_MAA_SetCalculation(mxc_tpu_maa_operation_t operation);

/**
 * @brief   Start an MAA computation, assuming all registers are loaded
 * @note    This function returns immediately, use MXC_TPU_Done to determine if
 *          the MAA operation is complete
 *
 */
void MXC_TPU_MAA_Start(void);

/**
 * @brief   Start an MAA computation, assuming all registers are loaded
 * @note    This function blocks until the operation is complete
 *
 */
void MXC_TPU_MAA_StartBlocking(void);

/**
 * @brief   Set the size of the data used in MAA operations
 *
 * @param   size  The size of the modulus in bits
 */
void MXC_TPU_MAA_SetWordSize(int size);

/**
 * @brief   Get the size in bits of the MAA computations (see MXC_TPU_MAA_SetWordSize())
 *
 * @return  The size in bits of the MAA computations
 */
int MXC_TPU_MAA_GetWordSize(void);

/*******************************/
/* High Level Functions        */
/*******************************/

/**
 * @brief   Start an MAA computation, assuming all registers are loaded
 * @note    This function returns immediately, use MXC_TPU_Done to determine if
 *          the MAA operation is complete. The result will be stored in the request
 *          structure.
 *
 * @param   req   Structure containing all the information for the computation
 *
 * @return  see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_MAA_Compute(mxc_tpu_maa_req_t* req);

/**
 * @brief   Start an MAA computation, assuming all registers are loaded
 * @note    The user needs to call MXC_TPU_Handler() in the ISR. The result
 *          will be stored in the request structure.
 *
 * @param   req   Structure containing all the information for the computation
 */
void MXC_TPU_MAA_ComputeAsync(mxc_tpu_maa_req_t* req);

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/***********************/
/* Low Level Functions */
/***********************/

/**
 * @brief   Get the block size for a given hash function
 *
 * @param   function  See \ref mxc_tpu_hash_func_t for options
 *
 * @return  Block size in bytes
 */
unsigned int MXC_TPU_Hash_GetBlockSize(mxc_tpu_hash_func_t function);

/**
 * @brief   Get the digest size for a given hash function
 *
 * @param   function  See \ref mxc_tpu_hash_func_t for options
 *
 * @return  Digest size in bytes
 */
unsigned int MXC_TPU_Hash_GetDigestSize(mxc_tpu_hash_func_t function);

/**
 * @brief   Set the algorithm to use for hash computation
 *
 * @param   function  See \ref mxc_tpu_hash_func_t for options
 */
void MXC_TPU_Hash_SetFunction(mxc_tpu_hash_func_t function);

/**
 * @brief   Get the algorithm to use for hash computation
 *
 * @return  See \ref mxc_tpu_hash_func_t for options
 */
mxc_tpu_hash_func_t MXC_TPU_Hash_GetFunction(void);

/**
 * @brief   Set whether to use automatic padding of the input data
 * @note    The padding procedure used by hardware is described in the users guide
 *
 * @param   pad   Use hardware padding of the data
 */
void MXC_TPU_Hash_SetAutoPad(int pad);

/**
 * @brief   Get whether to use automatic padding of the input data
 *
 * @return  Using hardware padding of the data
 */
int MXC_TPU_Hash_GetAutoPad(void);

/**
 * @brief   Get the result of a hash computation
 *
 * @param   digest   buffer to store the output of the hash algorithm
 * @param   len      location to store the length of the digest
 */
void MXC_TPU_Hash_GetResult(uint8_t* digest, int* len);

/**
 * @brief   Set the size of the data input into the hash computation
 * @note    Hash data size is software limited to ~3GB
 *
 * @param   size  Size of the data in bytes
 */
void MXC_TPU_Hash_SetMessageSize(uint32_t size);

/**
 * @brief   Set the source of data for the hash computation
 *
 * @param   source  see \ref mxc_tpu_hash_source_t for options
 */
void MXC_TPU_Hash_SetSource(mxc_tpu_hash_source_t source);

/**
 * @brief   Get the source of data for the hash computation
 *
 * @return  See \ref mxc_tpu_hash_source_t for options
 */
mxc_tpu_hash_source_t MXC_TPU_Hash_GetSource(void);

/**
 * @brief   Initialize the hash computation unit
 * @note    Call this after setting the hash function and message size
 *          This function blocks until load is complete
 *
 */
void MXC_TPU_Hash_InitializeHash(void);

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
int MXC_TPU_Hash_Compute(mxc_tpu_hash_req_t* req);

/**
 * @brief   Compute a Hash Digest
 * @note    The computed digest will be stored in the req structure. The user
 *          must call MXC_TPU_Handler() in the ISR.
 *
 * @param   req   Structure containing all data needed for a hash computation
 */
void MXC_TPU_Hash_ComputeAsync(mxc_tpu_hash_req_t* req);

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/************************/
/* Low Level Functions  */
/************************/

/**
 * @brief   Get the key size for a given cipher type
 *
 * @param   cipher   See \ref mxc_tpu_cipher_t for options
 *
 * @return  Size of the key in bytes
 */
unsigned int MXC_TPU_Cipher_GetKeySize(mxc_tpu_cipher_t cipher);

/**
 * @brief   Get the block size for a given cipher type
 *
 * @param   cipher   See \ref mxc_tpu_cipher_t for options
 *
 * @return  Size of the block in bytes
 */
unsigned int MXC_TPU_Cipher_GetBlockSize(mxc_tpu_cipher_t cipher);

/**
 * @brief   Set the block mode used for cipher operations
 *
 * @param   mode   See \ref mxc_tpu_cipher_mode_t for options
 */
void MXC_TPU_Cipher_SetMode(mxc_tpu_cipher_mode_t mode);

/**
 * @brief   Get the block mode used for cipher operations
 *
 * @return  See \ref mxc_tpu_cipher_mode_t for options
 */
mxc_tpu_cipher_mode_t MXC_TPU_Cipher_GetMode(void);

/**
 * @brief   Set the cipher type used for cipher operations
 *
 * @param   cipher   See \ref mxc_tpu_cipher_t for options
 */
void MXC_TPU_Cipher_SetCipher(mxc_tpu_cipher_t cipher);

/**
 * @brief   Get the cipher type used for cipher operations
 *
 * @return  See \ref mxc_tpu_cipher_t for options
 */
mxc_tpu_cipher_t MXC_TPU_Cipher_GetCipher(void);

/**
 * @brief   Set the source of the key used in cipher operations
 *
 * @param   source   See \ref mxc_tpu_cipher_key_t for options
 */
void MXC_TPU_Cipher_SetKeySource(mxc_tpu_cipher_key_t source);

/**
 * @brief   Get the cipher type used for cipher operations
 *
 * @return  See \ref mxc_tpu_cipher_key_t for options
 */
mxc_tpu_cipher_key_t MXC_TPU_Cipher_GetKeySource(void);

/**
 * @brief   Load the cipher key from the selected source
 *
 */
void MXC_TPU_Cipher_LoadKey(void);

/**
 * @brief   Configure for encryption or decryption
 *
 * @param   encrypt Set to perform encryption
 */
void MXC_TPU_Cipher_SetOperation(int encrypt);

/**
 * @brief   Set the cipher key
 * @note    This only takes effect if software is the selected key source
 *
 * @param   key   buffer containing key
 * @param   len   length of key (dependent on cipher used)
 */
void MXC_TPU_Cipher_SetKey(uint8_t* key, int len);

/**
 * @brief   Set the initial value used for cipher operations
 *
 * @param   iv   buffer containing iv
 * @param   len  length of initial value
 */
void MXC_TPU_Cipher_SetIV(uint8_t* iv, int len);

/**
 * @brief   Get the initial value used for cipher operations
 *
 * @param   iv   buffer to store the iv in
 * @param   len  length of buffer
 */
void MXC_TPU_Cipher_GetIV(uint8_t* ivOut, int len);

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
int MXC_TPU_Cipher_Encrypt(mxc_tpu_cipher_req_t* req);

/**
 * @brief   Perform a decryption using the cipher feature
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the decryption
 *
 * @return  See \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_TPU_Cipher_Decrypt(mxc_tpu_cipher_req_t* req);

/**
 * @brief   Perform an encryption using the cipher feature
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_TPU_Handler() in the ISR
 *
 * @param   req  Structure containing data for the encryption
 */
void MXC_TPU_Cipher_EncryptAsync(mxc_tpu_cipher_req_t* req);

/**
 * @brief   Perform a decryption using the cipher feature
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_TPU_Handler() in the ISR
 *
 * @param   req  Structure containing data for the decryption
 */
void MXC_TPU_Cipher_DecryptAsync(mxc_tpu_cipher_req_t* req);

#endif /* _TPU_H_ */
