/*============================================================================
 *
 * ucl_aes_rw20.h [20-apr-16]
 *
 *==========================================================================*/
/*============================================================================
 *
 * Copyright © 2016 Maxim Integrated. All Rights Reserved. Do not disclose.
 *
 * This software is the confidential and proprietary information of
 * Maxim Integrated ("Confidential Information"). You shall not
 * disclose such Confidential Information and shall use it only in
 * accordance with the terms of the license agreement you entered
 * into with Maxim Integrated.
 *
 * Maxim Integrated makes no representations or warranties about the suitability of
 * the software, either express or implied, including but not limited to
 * the implied warranties of merchantability, fitness for a particular purpose,
 * or non-infrigement. Maxim Integrated shall not be liable for any damages suffered
 * by licensee as the result of using, modifying or distributing this software
 * or its derivatives.
 *
 *==========================================================================*/
/*============================================================================
 *
 * Purpose : AES routines using RW20 AES registers
 *
 *==========================================================================*/
#ifndef _AES_RW20
#define _AES_RW20

#include <ucl/ucl_aes.h>
#include <ucl/ucl_types.h>

#define MK_KEY 0
#define K_KEY 1
#define RW20_CBC 0
#define RW20_CMAC 256

#define RW20_SYS_INIT_BLOCK_ADDRESS 0xABCDEF
#define RW20_SYS_INIT_AES_KEY_SEL RW20_SYS_INIT_BLOCK_ADDRESS + 0x04
#define RW20_SYS_INIT_AES_KEY_STAT RW20_SYS_INIT_BLOCK_ADDRESS + 0x08
#define RW20_SYS_INIT_AES_KEY_1_CTRL RW20_SYS_INIT_BLOCK_ADDRESS + 0x0C
#define RW20_SYS_INIT_AES_KEY_2_CTRL RW20_SYS_INIT_BLOCK_ADDRESS + 0x010
#define RW20_SYS_INIT_AES_KEY_STAT_CRC RW20_SYS_INIT_BLOCK_ADDRESS + 0x34
#define RW20_SYS_INIT_AES_KEY_OEMID_DATA RW20_SYS_INIT_BLOCK_ADDRESS + 0x38
#define KEY_WR_TO_AES_BUSY 1

#define RW20_SECURITY_BLOCK_ADDRESS 0xABCDEF
#define RW20_AESDATAIN0 RW20_SECURITY_BLOCK_ADDRESS + 0x2C
#define RW20_AESDATAIN1 RW20_SECURITY_BLOCK_ADDRESS + 0x30
#define RW20_AESDATAIN2 RW20_SECURITY_BLOCK_ADDRESS + 0x34
#define RW20_AESDATAIN3 RW20_SECURITY_BLOCK_ADDRESS + 0x38

#define RW20_AESDATAOUT0 RW20_SECURITY_BLOCK_ADDRESS + 0x3C
#define RW20_AESDATAOUT1 RW20_SECURITY_BLOCK_ADDRESS + 0x40
#define RW20_AESDATAOUT2 RW20_SECURITY_BLOCK_ADDRESS + 0x44
#define RW20_AESDATAOUT3 RW20_SECURITY_BLOCK_ADDRESS + 0x48
#define RW20_AES_CTRL RW20_SECURITY_BLOCK_ADDRESS + 0x4C
#define RW20_AES_CTRL_MODE 2
#define RW20_AES_CTRL_INIT_DEC_KEY 4
#define RW20_AES_CTRL_START 3
#define RW20_AES_STAT_BUSY 0
#define RW20_AES_STAT_ERR 2
#define RW20_AES_STAT RW20_SECURITY_BLOCK_ADDRESS + 0x50

int ucl_load_oemid(u32 oemid);

// multi-packets AES CBC

// mode est UCL_CIPHER_ENCRYPT ou UCL_CIPHER_DECRYPT
// key_id est 0(MK) ou 1(K)
// len est un nombre de bytes (multiple de 16)
// ucl_aes_ctx_t est défini dans ucl_aes.h

/*============================================================================*/
/** <b>RW20 AES-CBC Init</b>.
 * Initialise AES CBC Context.
 *
 * @param[out] ctx    Pointer to the context
 * @param[in]  key_id    AES Key index: MK_KEY for MK, K_KEY for K
 * @param[in]  IV     Pointer to the initialization vector
 * @param[in]  mode   The mode (Encryption/Decryption) :
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @ingroup UCL_CBC_AES
 */
int ucl_aes_cbc_init_rw20(ucl_aes_ctx_t* ctx, int key_id, u8* IV, int mode);

/*============================================================================*/
/** <b>RW20 AES-CBC Core</b>.
 * Process the Data.
 *
 * @param[out]    dst  Pointer to the processed data
 * @param[out,in] ctx  Pointer to the context
 * @param[in]     src  Pointer to the data
 * @param[in]     len  Data byte length
 *
 * @pre The byte length must be a multiple of #UCL_AES_BLOCKSIZE.
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    The data byte length is not a multiple of
 *                             #UCL_AES_BLOCKSIZE
 *
 * @ingroup UCL_CBC_AES
 */
int ucl_aes_cbc_core_rw20(u8* dst, ucl_aes_ctx_t* ctx, u8* src, u32 len);

/*============================================================================*/
/**<b>RW20 AES-CBC Finish</b>.
 * Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CBC_AES
 */
int ucl_aes_cbc_finish_rw20(ucl_aes_ctx_t* ctx);

/*============================================================================*/
/** <b>RW20 AES-CBC</b>.
 * Encrypt / Decrypt with AES in CBC (Cipher Block Chaining) mode.
 *
 *
 * @param[out] dst    Pointer to the output data
 * @param[in]  src    Pointer to the input data
 * @param[in]  len    Data byte length
 * @param[in]  key_id    AES Key index: MK_KEY for MK, K_KEY for K
 * @param[in] IV      Pointer to the initialization vector
 * @param[in] mode    The mode (Encryption/Decryption):
 *                        @li #UCL_CIPHER_ENCRYPT
 *                        @li #UCL_CIPHER_DECRYPT
 *
 * @return Error code
 *
 * @retval #UCL_OK             no error occurred
 * @retval #UCL_INVALID_INPUT  one of the input is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT one of the output is the pointer NULL
 * @retval #UCL_INVALID_ARG   @p len is not a multiple of
 *                             #UCL_AES_BLOCKSIZE or @p keylen is invalid
 *
 * @ingroup UCL_CBC_AES
 */
int ucl_aes_cbc_rw20(u8* dst, u8* src, u32 len, int key_id, u8* iv, int mode);

// multi-packets AES CMAC

/*============================================================================*/
/** <b>RW20 AES-CMAC verifiy Init</b>.
 * Initialise AES CMAC Context.
 *
 * @param[out] ctx    Pointer to the context
 * @param[in]  key_id    AES Key index: MK_KEY for MK, K_KEY for K
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  The input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CMAC_AES
 */
int ucl_aes_cmac_verify_init_rw20(ucl_aes_ctx_t* ctx, int key_id);

/*============================================================================*/
/** <b>RW20 AES-CMAC verify Core</b>.
 * Process the Data verification.
 *
 * @param[out]    t  Pointer to the CMAC data
 * @param[in]     tlen  CMAC byte length
 * @param[out,in] ctx  Pointer to the context
 * @param[in]     src  Pointer to the data
 * @param[in]     len  Data byte length
 *
 * @pre The byte length must be a multiple of #UCL_AES_BLOCKSIZE.
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 * @retval #UCL_INVALID_ARG    The data byte length is not a multiple of
 *                             #UCL_AES_BLOCKSIZE
 *
 * @ingroup UCL_CMAC_AES
 */
int ucl_aes_cmac_verify_core_rw20(u8* t, int tlen, ucl_aes_ctx_t* ctx, u8* src, u32 len);

int ucl_aes_cmac_verify_rw20(u8* t, int tlen, u8* src, u32 len, int key_id);

/*============================================================================*/
/**<b>RW20 AES-CMAC verify Finish</b>.
 * Finalize the verification and Zeroize the context.
 *
 * @param[out,in] ctx Pointer to the context
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred: CMAC verification is successful
 * @retval #UCL_INVALID_OUTPUT The output is the pointer #NULL
 *
 * @ingroup UCL_CMAC_AES
 */
int ucl_aes_cmac_verify_finish_rw20(ucl_aes_ctx_t* ctx);

#endif //_AES_RW20
