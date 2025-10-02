/**
 * @file    aes.h 
 * @brief   AES driver.
 */

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
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

#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_AES_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_AES_H_

/***** Includes *****/
#include "aes_regs.h"
#include "aeskeys_regs.h"
#include "dma_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup aes AES
 * @ingroup periphlibs
 * @{
 */
/*@} end of group aes */

/***** Definitions *****/

typedef void (*mxc_aes_complete_t)(void *req, int result);

/* ************************************************************************* */
/* Cipher Definitions                                                                          */
/* ************************************************************************* */

/**
  * @brief  Enumeration type to select AES key
  *
  */
typedef enum {
    MXC_AES_128BITS = MXC_S_AES_CTRL_KEY_SIZE_AES128, ///< Select AES-128 bit key
    MXC_AES_192BITS = MXC_S_AES_CTRL_KEY_SIZE_AES192, ///< Select AES-192 bit key
    MXC_AES_256BITS = MXC_S_AES_CTRL_KEY_SIZE_AES256, ///< Select AES-256 bit key
} mxc_aes_keys_t;

/**
  * @brief  Enumeration type to select AES key source and encryption type
  *
  */
typedef enum {
    MXC_AES_ENCRYPT_EXT_KEY = 0, ///< Encryption using External key
    MXC_AES_DECRYPT_EXT_KEY = 1, ///< Encryption using internal key
    MXC_AES_DECRYPT_INT_KEY = 2 ///< Decryption using internal key
} mxc_aes_enc_type_t;

/**
  * @brief  Structure used to set up AES request
  *
  */
typedef struct _mxc_aes_cipher_req_t {
    uint32_t length; ///< Length of the data
    uint32_t *inputData; ///< Pointer to input data. Buffer length MUST be a multiple of 4 words
    uint32_t *resultData; ///< Pointer to encrypted data. Buffer length MUST be a multiple of 4 words
    mxc_aes_keys_t keySize; ///< Size of AES key
    mxc_aes_enc_type_t encryption; ///< Encrytion type or \ref mxc_aes_enc_type_t
    mxc_aes_complete_t callback; ///< Callback function
} mxc_aes_req_t;

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Global Control/Configuration functions                                    */
/* ************************************************************************* */

/**
 * @brief   Enable portions of the AES
 *
 * @param   dma   DMA instance used for AES
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_Init(mxc_dma_regs_t *dma);

/**
 * @brief   Enable AES Interrupts
 * 
 * @param   interrupt interrupt to enable
 */
void MXC_AES_EnableInt(uint32_t interrupt);

/**
 * @brief   Disable AES Interrupts
 * 
 * @param   interrupt interrupt to disable
 */
void MXC_AES_DisableInt(uint32_t interrupt);

/**
 * @brief   Checks the global AES Busy Status
 *
 * @return  E_BUSY if busy and E_NO_ERROR otherwise, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_IsBusy(void);

/**
 * @brief   Disable and reset portions of the AES
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_Shutdown(void);

/**
 * @brief   This function should be called from the DMA Handler
 *          when using Async functions
 */
void MXC_AES_DMACallback(int ch, int error);

/**
 * @brief   This function should be called before encryption to genrate external key
 */
void MXC_AES_GenerateKey(void);

/**
 * @brief   Set Key size for encryption or decryption
 * 
 * @param   key Key size, see \ref mxc_aes_keys_t for a list of keys
 */
void MXC_AES_SetKeySize(mxc_aes_keys_t key);

/**
 * @brief   Get the currently set key size
 * 
 * @return  mxc_aes_keys_t see \ref mxc_aes_keys_t
 */
mxc_aes_keys_t MXC_AES_GetKeySize(void);

/**
 * @brief   Flush Input Data FIFO
 * 
 */
void MXC_AES_FlushInputFIFO(void);

/**
 * @brief   Flush Output Data FIFO
 * 
 */
void MXC_AES_FlushOutputFIFO(void);

/**
 * @brief   Start AES Calculations
 * 
 */
void MXC_AES_Start(void);

/**
 * @brief   Get Interrupt flags set
 * 
 * @return  return the flags set in intfl register
 */
uint32_t MXC_AES_GetFlags(void);

/**
 * @brief   Clear the interrupts
 * 
 * @param   flags flags to be cleared
 */
void MXC_AES_ClearFlags(uint32_t flags);

/**
 * @brief   Perform generic AES operation (no interrupt handling).
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the encryption
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_Generic(mxc_aes_req_t *req);

/**
 * @brief   Perform an encryption (no interrrupt handling)
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the encryption
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_Encrypt(mxc_aes_req_t *req);

/**
 * @brief   Perform a decryption (no interrupt handling)
 * @note    The result will be stored in the req structure
 *
 * @param   req  Structure containing data for the decryption
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_Decrypt(mxc_aes_req_t *req);

/**
 * @brief   Perform encryption or decryption using interrupts
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * 
 * @param   req The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 * @param   enc 0 for encryption and 1 for decryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_GenericAsync(mxc_aes_req_t *req, uint8_t enc);

/**
 * @brief   Perform an encryption using interrupts
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 *
 * @param   req  Structure containing data for the encryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_EncryptAsync(mxc_aes_req_t *req);

/**
 * @brief   Perform a decryption using interrupts.
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 *
 * @param   req  Structure containing data for the decryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_DecryptAsync(mxc_aes_req_t *req);

/**
 * @brief   Performs AES handler operation.
 * @details This function must be called in the AES ISR when using
 *          'MXC_AES_*Async(...)' functions.
 */
void MXC_AES_Handler(void);

/**
 * @brief   Initializes the DMA and acquire channels before AES operations. Simplifies
 *          DMA interrupt handling for AES async operations.
 * @details This optional function is called before MXC_AES_EncryptAsync(...),
 *          MXC_AES_DecryptAsync(...), or MXC_AES_GenericAsync(...) to retrieve the
 *          DMA channels associated with AES transmit and receive operations. The
 *          previous implementation required the application to know the DMA channel
 *          that AES will use at compile time. However, the driver can NOT assume
 *          that the compile time selected DMA channel will always be acquired.
 *          This functipn also uses the DMA instance passed and internally saved
 *          with MXC_AES_Init(...).
 * 
 * @param   rx_channel  Pointer to store RX DMA Channel used in AES DMA operations.
 * @param   tx_channel  Pointer to store TX DMA Channel used in AES DMA operations.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_AES_PreInitDMA(int8_t *rx_channel, int8_t *tx_channel);

/**
 * @brief   Perform AES TX using DMA. Configures DMA request and starts the transmission.
 * 
 * @param   src_addr  source address
 * @param   len       number of words of data
 * @param   dma       DMA instance to configue for AES
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_AES_TXDMAConfig(void *src_addr, int len, mxc_dma_regs_t *dma);

/**
 * @brief   Perform AES RX using DMA. Configures DMA request and receives data from AES FIFO.
 * 
 * @param   dest_addr destination address
 * @param   len       number of words of data
 * @param   dma       DMA instance to configure for AES
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_AES_RXDMAConfig(void *dest_addr, int len, mxc_dma_regs_t *dma);

/**
 * @brief   Get the DMA TX channel assigned to AES DMA (Async) operations.
 * 
 * @param   channel   Pointer to store assigned TX DMA Channel
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_AES_GetTXDMAChannel(int8_t *channel);

/**
 * @brief   Get the DMA RX channel assigned to AES DMA (Async) operations.
 * 
 * @param   channel   Pointer to store assigned RX DMA Channel
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes. 
 */
int MXC_AES_GetRXDMAChannel(int8_t *channel);



/**
 * @brief   Perform encryption or decryption using DMA
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * 
 * @param   req The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 * @param   enc 0 for encryption and 1 for decryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_GenericDMA(mxc_aes_req_t *req, uint8_t enc);

/**
 * @brief   Perform an encryption using DMA
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 *
 * @param   req  Structure containing data for the encryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_EncryptDMA(mxc_aes_req_t *req);

/**
 * @brief   Perform a decryption using DMA
 * @details Although the function name says 'Async', an AES DMA operation does
 *          not use the AES IRQ but rather the DMAn_CHm IRQs. THe AES interrupt is
 *          only used for non-DMA AES operations.
 * @note    The result will be stored in the req structure. The user needs
 *          to call MXC_DMA_Handler() in the appropriate DMAn_CHm ISR
 *
 * @param   req  Structure containing data for the decryption
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_AES_DecryptDMA(mxc_aes_req_t *req);

/**
 * @brief   Set the external key
 * @param   key  Buffer for the key.
 * @param   len  Key size.
 */
void MXC_AES_SetExtKey(const void *key, mxc_aes_keys_t len);

#ifdef __cplusplus
}
#endif
/**@} end of group aes */

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_AES_H_
