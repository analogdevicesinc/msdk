/**
 * @file    crypto.h
 * @brief   TPU communications interface driver.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_TPU_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_TPU_H_

#include <stdint.h>
#include "tpu_regs.h"
#include "trng_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup tpu TPU
 * @ingroup periphlibs
 * @{
 */
/* ****** MACROS ****** */
// CRC Polynomials
#define MXC_TPU_CRC32_ETHERNET 0xEDB88320
#define MXC_TPU_CRC_CCITT 0x00008408
#define MXC_TPU_CRC16 0x0000A001
#define MXC_TPU_USBDATA 0x80050000
#define MXC_TPU_PARITY 0x00000001

#define DES_DATA_LEN 8 // The byte length for DES data block
#define AES_DATA_LEN 16 // The byte length for AES data block
#define MAX_KEY_SIZE 32 // Defines maximum key length
#define MXC_AES_DATA_LEN \
    (128 /               \
     8) /**< Number of bytes in an AES plaintext or ciphertext block, which are always 128-bits long. */
#define MXC_AES_KEY_128_LEN (128 / 8) /**< Number of bytes in a AES-128 key. */
#define MXC_AES_KEY_192_LEN (192 / 8) /**< Number of bytes in a AES-192 key. */
#define MXC_AES_KEY_256_LEN (256 / 8) /**< Number of bytes in a AES-256 key. */

//Macros used for MAA
#define MAA_MAX_SIZE 256 // in bytes
#define MAA_MAX_WORD_SIZE 2048 // in bits

/***************************************************************************************************************
                  DATA STRUCTURES FOR TPU INITIALIZATION
***************************************************************************************************************/
/**
  * Enumeration type for the TPU Cipher Operation(128/192/256-bit key)
  */
typedef enum {
    MXC_TPU_CIPHER_DIS = MXC_V_TPU_CIPHER_CTRL_CIPHER_DIS, // Disable
    MXC_TPU_CIPHER_AES128 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES128, // Select AES-128
    MXC_TPU_CIPHER_AES192 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES192, // Select AES-192
    MXC_TPU_CIPHER_AES256 = MXC_V_TPU_CIPHER_CTRL_CIPHER_AES256, // Select AES-256
    MXC_TPU_CIPHER_DES = MXC_V_TPU_CIPHER_CTRL_CIPHER_DES, // Select DES
    MXC_TPU_CIPHER_TDEA = MXC_V_TPU_CIPHER_CTRL_CIPHER_TDEA // Select TDEA
} mxc_tpu_ciphersel_t;

/**
  * Enumeration type to select which key is used in Cipher operations
  */
typedef enum {
    MXC_TPU_KEYSRC_CIPHER_KEY = MXC_S_TPU_CIPHER_CTRL_SRC_CIPHERKEY, // Use key in CIPHER_KEY[0:7]
    MXC_TPU_KEYSRC_AES_KEY0 = MXC_S_TPU_CIPHER_CTRL_SRC_REGFILE, // Use key 0 in AES_KEY registers
    MXC_TPU_KEYSRC_AES_KEY1 =
        MXC_S_TPU_CIPHER_CTRL_SRC_QSPIKEY_REGFILE // Use key 1 in AES_KEY registers
} mxc_tpu_keysrc_t;

/**
  * Enumeration type for the TPU Mode Select
  */
typedef enum {
    MXC_TPU_MODE_ECB = MXC_V_TPU_CIPHER_CTRL_MODE_ECB, // Select ECB
    MXC_TPU_MODE_CBC = MXC_V_TPU_CIPHER_CTRL_MODE_CBC, // Select CBC
    MXC_TPU_MODE_CFB = MXC_V_TPU_CIPHER_CTRL_MODE_CFB, // Select CFB
    MXC_TPU_MODE_CTR = MXC_V_TPU_CIPHER_CTRL_MODE_CTR // Select CTR
} mxc_tpu_modesel_t;

/**
  * Enumeration type for Hash function Select
  */
typedef enum {
    MXC_TPU_HASH_DIS = MXC_V_TPU_HASH_CTRL_HASH_DIS, // Disable
    MXC_TPU_HASH_SHA1 = MXC_V_TPU_HASH_CTRL_HASH_SHA1, // Select SHA1
    MXC_TPU_HASH_SHA224 = MXC_V_TPU_HASH_CTRL_HASH_SHA224, // Select SHA224
    MXC_TPU_HASH_SHA256 = MXC_V_TPU_HASH_CTRL_HASH_SHA256, // Select SHA256
    MXC_TPU_HASH_SHA384 = MXC_V_TPU_HASH_CTRL_HASH_SHA384, // Select SHA384
    MXC_TPU_HASH_SHA512 = MXC_V_TPU_HASH_CTRL_HASH_SHA512 // Select SHA384
} mxc_tpu_hashfunsel_t;

/**
  * Enumeration type for MAA initialization
  */
typedef enum {
    MXC_TPU_MAA_EXP = MXC_V_TPU_MAA_CTRL_CLC_EXP, // Select exponentiation operation
    MXC_TPU_MAA_SQ = MXC_V_TPU_MAA_CTRL_CLC_SQ, // Select square operation
    MXC_TPU_MAA_MUL = MXC_V_TPU_MAA_CTRL_CLC_MULT, // Select multiplication operation
    MXC_TPU_MAA_SQMUL =
        MXC_V_TPU_MAA_CTRL_CLC_SQ_MULT, // Select square followed by multiplication operation
    MXC_TPU_MAA_ADD = MXC_V_TPU_MAA_CTRL_CLC_ADD, // Select add operation
    MXC_TPU_MAA_SUB = MXC_V_TPU_MAA_CTRL_CLC_SUB // Select subtract operation
} mxc_tpu_maa_clcsel_t;

/***************************************************************************************************************
                      DRIVER EXPOSED API's
***************************************************************************************************************/
/**
 * @brief      Init TPU system settings
 * @param      clock    peripheral clock to use
 *
 * @return     E_NO_ERROR if successful, E_TIME_OUT otherwise.
 */
int MXC_TPU_Init(mxc_sys_periph_clock_t clock);

/**
 * @brief      Shutdown TPU system specific settings
 */
int MXC_TPU_Shutdown(mxc_sys_periph_clock_t clock);

/**
 * @brief      Reset the crypto accelerator
 */
void MXC_TPU_Reset(void);

/* ************************************************************************* */
/* Cyclic Redundancy Check (CRC) functions                                   */
/* ************************************************************************* */

/**
 * @brief      Configure crypto CRC operation
 * @return     #E_SUCCESS    CRC algorithm configured successfully
 */
int MXC_TPU_CRC_Config(void);

/**
 * @brief      Test the CRC process
 * @param      src           Pointer to source message
 * @param      len           Specifies size of message in bytes
 * @param      poly          Selects the crc polynomial
 * @param      crc           Pointer to store crc value
 * @return     #E_NULL_PTR   Specified pointers \p src; points to null
 * @return     #E_SUCCESS    CRC process completed successfully
 */
int MXC_TPU_CRC(const uint8_t *src, uint32_t len, uint32_t poly, uint32_t *crc);

/**
 * @brief      Configure crypto HAM operation
 * @return     #E_SUCCESS    HAM algorithm configured successfully
 */
int MXC_TPU_Ham_Config(void);

/**
 * @brief      Test the CRC process
 * @param      src           Pointer to source message
 * @param      len           Specifies size of message in bytes
 * @param      ecc           Pointer to store ecc value
 * @return     #E_NULL_PTR   Specified pointers \p src; points to null
 * @return     #E_SUCCESS    CRC process completed successfully
 */
int MXC_TPU_Ham(const uint8_t *src, uint32_t len, uint32_t *ecc);

/* ************************************************************************* */
/* Cipher functions                                                          */
/* ************************************************************************* */

/**
 * @brief      Get cipher key's size
 */
unsigned int MXC_TPU_Cipher_Get_Key_Size(mxc_tpu_ciphersel_t cipher);

/**
 * @brief      Get cipher block's size
 */
unsigned int MXC_TPU_Cipher_Get_Block_Size(mxc_tpu_ciphersel_t cipher);

/**
 * @brief      Get number of blocks
 */
unsigned int MXC_TPU_Cipher_GetLength(mxc_tpu_ciphersel_t cipher, unsigned int data_size);

/**
 * @brief      Enable Encrypt/Decrypt Cipher Operation
 * @param      enc      Enable Encryption or Decryption
 */
void MXC_TPU_Cipher_EncDecSelect(int enc);

/**
 * @brief      Configure crypto cipher operation for different modes
 * @param      mode      Selects the TPU operation mode
 * @param      cipher    Selects the Cipher Operation mode
 * @return    #E_SUCCESS     Cipher algorithm configured successfully
 */
int MXC_TPU_Cipher_Config(mxc_tpu_modesel_t mode, mxc_tpu_ciphersel_t cipher);

/**
 * @brief      Select the source of the cipher key used in cipher operations
 * @param      key_src      Selects the key used in cipher operations
 * @return     #E_SUCCESS   Cipher key selected successfully
 */
int MXC_TPU_Cipher_KeySelect(mxc_tpu_keysrc_t key_src);

/**
 * @brief      Test Cipher Algorithm
 * @param      src         Pointer to the plaintext/ciphertext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      cipher        Selects the Cipher Operation mode
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_INVALID    DES Encryption process failed
 * @return     #E_SUCCESS    DES Encryption process completed successfully
 * */
int MXC_TPU_Cipher_DoOperation(const char *src, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr);

/**
 * @brief      The DES encryption process
 * @param      plaintext     Pointer to the plaintext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_INVALID    DES Encryption process failed
 * @return     #E_SUCCESS    DES Encryption process completed successfully
 */
int MXC_TPU_Cipher_DES_Encrypt(const char *plaintext, const char *iv, const char *key,
                               mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);

/**
 * @brief      The DES decryption process
 * @param      ciphertext    Pointer to the ciphertext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_INVALID    DES Decryption process failed
 * @return     #E_SUCCESS    DES Decryption process completed successfully
 */
int MXC_TPU_Cipher_DES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                               mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);

/**
 * @brief      The TDES encryption process
 * @param      plaintext     Pointer to the plaintext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_INVALID    TDES Encryption process failed
 * @return     #E_SUCCESS    TDES Encryption process completed successfully
 */
int MXC_TPU_Cipher_TDES_Encrypt(const char *plaintext, const char *iv, const char *key,
                                mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);

/**
 * @brief      The TDES decryption process
 * @param      ciphertext    Pointer to the ciphertext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_INVALID    TDES Decryption process failed
 * @return     #E_SUCCESS    TDES Decryption process completed successfully
 */
int MXC_TPU_Cipher_TDES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                                mxc_tpu_modesel_t mode, unsigned int data_size, char *outptr);

/**
 * @brief      The AES encryption process
 * @param      plaintext     Pointer to the plaintext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      cipher        Selects the Cipher Operation mode
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_BAD_PARAM  Specified Cipher operation mode @cipher is invalid, see #tpu_ciphersel_t
 * @return     #E_INVALID    AES Encryption process failed
 * @return     #E_SUCCESS    AES Encryption process completed successfully
 */
int MXC_TPU_Cipher_AES_Encrypt(const char *plaintext, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr);

/**
 * @brief      The AES decryption process
 * @param      ciphertext    Pointer to the ciphertext data
 * @param      iv            Pointer to the initial vector data
 * @param      key           Pointer to the crypto key
 * @param      cipher        Selects the Cipher Operation mode
 * @param      mode          Selects the TPU operation mode
 * @param      data_size     Specifies length of data in bytes
 * @param      outptr        Output buffer
 * @return     #E_NULL_PTR   Specified pointers @plaintext; @iv; @key points to null
 * @return     #E_BAD_PARAM  Specified Cipher operation mode @cipher is invalid, see #tpu_ciphersel_t
 * @return     #E_INVALID    AES Encryption process failed
 * @return     #E_SUCCESS    AES Encryption process completed successfully
 */
int MXC_TPU_Cipher_AES_Decrypt(const char *ciphertext, const char *iv, const char *key,
                               mxc_tpu_ciphersel_t cipher, mxc_tpu_modesel_t mode,
                               unsigned int data_size, char *outptr);

/* ************************************************************************* */
/* Hash functions                                                            */
/* ************************************************************************* */

/**
 * @brief      Get hash block's size
 */
unsigned int MXC_TPU_Hash_Get_Block_Size_SHA(mxc_tpu_hashfunsel_t func);

/**
 * @brief      Get hash digest's size
 */
unsigned int MXC_TPU_Hash_Get_Dgst_Size(mxc_tpu_hashfunsel_t func);

/**
 * @brief      Get SHA size
 */
void MXC_TPU_Hash_SHA_Size(unsigned int *blocks, unsigned int *length, unsigned int *lbyte,
                           mxc_tpu_hashfunsel_t fun);

/**
 * @brief      Configure     crypto hash operation for different hash functions
 * @param      func          Selects the hash function
 * @return     #E_SUCCESS    Hash algorithm configured successfully
 */
int MXC_TPU_Hash_Config(mxc_tpu_hashfunsel_t func);

/**
 * @brief      Test the SHA process
 * @param      fun           Selects the hash function
 * @param      msg           Pointer to source message
 * @param      byteLen       Specifies size of message in bytes
 * @param      digest        Digest buffer
 * @return     #E_NULL_PTR   Specified pointers \p msg; \p digest points to null
 * @return     #E_SUCCESS    SHA process completed successfully
 */
int MXC_TPU_Hash_SHA(const char *msg, mxc_tpu_hashfunsel_t fun, unsigned int byteLen, char *digest);

/* ************************************************************************* */
/* True Random Number Generator (TRNG) functions                             */
/* ************************************************************************* */

/**
  *@brief Reads 8-bit value stored in the data register.
  *@param trng  Pointer to the trng register structure.
  *@return  8-bit data register value.
  */
uint8_t MXC_TPU_TRNG_Read8BIT(mxc_trng_regs_t *trng);

/**
  *@brief Reads 16-bit value stored in the data register.
  *@param trng  Pointer to the trng register structure.
  *@return  16-bit data register value.
  */
uint16_t MXC_TPU_TRNG_Read16BIT(mxc_trng_regs_t *trng);

/**
  *@brief Reads 32-bit value stored in the data register.
  *@param trng  Pointer to the trng register structure.
  *@return  32-bit data register value.
  */
uint32_t MXC_TPU_TRNG_Read32BIT(mxc_trng_regs_t *trng);

/**
  *@brief Generates Random Number of variable length.
  *@param trng  Pointer to the trng register structure.
  *@param data  Pointer to the Data Buffer.
  *@param len Defines length of data(bytes).
  */
void MXC_TPU_TRNG_Read(mxc_trng_regs_t *trng, uint8_t *data, int len);

/**
  *@brief Generates 256-bits random number automatically.
  *@param trng  Pointer to the trng register structure.
  */
void MXC_TPU_TRNG_Generate_AES(mxc_trng_regs_t *trng);

/* ************************************************************************* */
/* Modular Arithmetic Accelerator (MAA) functions                             */
/* ************************************************************************* */

/**
 * @brief      Initialize memory used for MAA
 */
void MXC_TPU_MAA_Mem_Clear(void);

/**
 * @brief      Reset the TPU accelerator
 */
void MXC_TPU_MAA_Reset(void);

/**
 * @brief      Configure MAA operation with appropriate MAA word size
 * @param      size      Defines the number of bits for modular operation
 * @return     #E_BAD_PARAM  Specified size \p size, out of range
 * @return     #E_SUCCESS    Cipher algorithm configured successfully
 */
int MXC_TPU_MAA_Init(unsigned int size);

/**
 * @brief      Release MAA
 * @details    Shuts down the MAA engine and any associated clocks
 * @return     #E_BAD_PARAM if MAA cannot be stopped
 * @return     #E_NO_ERROR otherwise
 */
int MXC_TPU_MAA_Shutdown(void);

/**
 * @brief      MAA operation
 * @param      clc         Selects the MAA calculation operation
 * @param      multiplier    Pointer to multiplier data
 * @param      multiplicand  Pointer to multiplicand data
 * @param      exp       Pointer to exponent data
 * @param      mod           Pointer to modular data
 * @param      result        Output buffer
 * @param      len           Specifies length to the nearest 32-bit boundary
 * @return     #E_NULL_PTR   Specified pointers \p multiplier; \p multiplicand; \p exp; \p mod points to null
 * @return     #E_INVALID    Specified MAA calculation operation is invalid
 * @return     #E_BAD_STATE  MAA Error occurs
 * @return     #E_SUCCESS    MAA process completed successfully
 *
 * @note       \p multiplier; \p multiplicand; \p exp; \p mod, data must be loaded with zero pad to specified length \p len, or the "garbage bits" will case erroneous results
 */
int MXC_TPU_MAA_Compute(mxc_tpu_maa_clcsel_t clc, char *multiplier, char *multiplicand, char *exp,
                        char *mod, int *result, unsigned int len);

/**@} end of group tpu */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_TPU_H_
