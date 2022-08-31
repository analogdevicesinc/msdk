#ifndef XYSSL_SM4_H
#define XYSSL_SM4_H

#define SM4_ENCRYPT 1
#define SM4_DECRYPT 0
#define SM4_BLOCKSIZE 16

/** @file sm4.h
 * @defgroup UCL_SM4 SM4
 * the SM4, see
 * http://www1.cnnic.cn/ScientificResearch/LeadingEdge/soea/sm4/201312/t20131204_43351.htm
 *
 * @par Header:
 * @link sm4.h sm4.h @endlink
 *
 * <b>SM4 Descriptor:</b>
 * @li Length of Input/Output Block: 128 bits
 * @li Length of Key: 128 bits
 *
 * For messages longer than 128 bits, use an Operation mode.@n
 * @n
 *
 * @see UCL_OPERATION_MODES
 * @ingroup UCL_BLOCK_CIPHER */

/** <b>SM4 Key</b>.
 * @ingroup UCL_SM4 */

typedef struct {
    int mode; //  encrypt/decrypt
    unsigned long sk[32]; // SM4 subkeys
} sm4_context;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          SM4 key schedule (128-bit, encryption)
 *
 * \param ctx      SM4 context to be initialized
 * \param key      16-byte secret key
 */

/*============================================================================*/
/** <b>SM4 key context setup for encryption </b>.
 * The complete SM4 setup for encryption key
 *
 * @param[out] ctx    SM4 key context structure
 * @param[in]  key    128-bit SM4 key
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 *
 * @ingroup UCL_SM4
 */
int sm4_setkey_enc(sm4_context* ctx, unsigned char key[16]);

/**
 * \brief          SM4 key schedule (128-bit, decryption)
 *
 * \param ctx      SM4 context to be initialized
 * \param key      16-byte secret key
 */

/** <b>SM4 key context setup for encryption </b>.
 * The complete SM4 setup for encryption key
 *
 * @param[out] ctx    SM4 key context structure
 * @param[in]  key    128-bit SM4 key
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 *
 * @ingroup UCL_SM4
 */

int sm4_setkey_dec(sm4_context* ctx, unsigned char key[16]);

/*============================================================================*/
/** <b>SM4 for ECB blocks processing </b>.
 * The complete SM4 for several blocks.
 *
 * @param[out] output    Output data (encrypted/decrypted)
 * @param[in]  input    Input data to encrypt/decrypt
 * @param[in]  length   the Input data length in bytes
 * @param[in]  ctx    a SM4 key structure
 * @param[in]  mode   The SM4 mode (SM4_ENCRYPT/SM4_DECRYPT)
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @ingroup UCL_SM4
 */

int sm4_crypt_ecb(
    sm4_context* ctx, int mode, int length, unsigned char* input, unsigned char* output);

/*============================================================================*/
/** <b>SM4 for CBC blocks processing </b>.
 * The complete SM4 for several blocks.
 *
 * @param[out] output    Output data (encrypted/decrypted)
 * @param[in]  input    Input data to encrypt/decrypt
 * @param[in]  length   the Input data length in bytes
 * @param[in]  iv       the 128-bit initial value, IV
 * @param[in]  ctx    a SM4 key structure
 * @param[in]  mode   The SM4 mode (SM4_ENCRYPT/SM4_DECRYPT)
 *
 * @return Error code
 *
 * @retval #UCL_OK             No error occurred
 * @retval #UCL_INVALID_INPUT  One of the input is the pointer #NULL
 * @retval #UCL_INVALID_OUTPUT One of the output is the pointer #NULL
 * @retval #UCL_INVALID_MODE   The mode is not one of those described
 *
 * @ingroup UCL_SM4
 */
int sm4_crypt_cbc(sm4_context* ctx, int mode, int length, unsigned char iv[16],
    unsigned char* input, unsigned char* output);

#ifdef __cplusplus
}
#endif

#endif /* sm4.h */
