#include <ucl/ucl_types.h>
/*============================================================================*/
/** <b>SHA</b>.
 * The complete process of SHA.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out] hash         Pointer to the digest
 * @param[in]  data         Pointer to the data
 * @param[in]  data_byteLen Data byte length
 * @param[in] algo, value of the SHA hash function
 *
 * algo is UCL_SHA1,UCL_SHA224,UCL_SHA256,UCL_SHA384,UCL_SHA512,UCL_SHA3
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is #NULL
 *
 * @ingroup UCL_SHA
 */
int ucl_sha(u8* hash, u8* data, u32 data_byteLen, int algo);

/*============================================================================*/
/** <b>SHA Init</b>.
 * The initialisation of SHA.
 *
 * @param[in] algo, value of the SHA hash function
 *
 * algo is UCL_SHA1,UCL_SHA224,UCL_SHA256,UCL_SHA384,UCL_SHA512,UCL_SHA3
 * @return Error code
 *
 * @retval #UCL_OK            if no error occurred
 * @retval #UCL_INVALID_INPUT if @p context is #NULL
 *
 * @ingroup UCL_SHA
 */
int ucl_sha_init(int algo);

/*============================================================================*/
/** <b>SHA Core</b>.
 * The core of SHA.
 *
 * @param[in]     data         Pointer to the data
 * @param[in]     data_byteLen Data byte length
 *
 * @warning #ucl_sha_init must be processed before, and
 *          #ucl_sha_finish should be processed to get the final hash.
 *
 * @return Error code
 *
 * @retval #UCL_OK  if no error occurred
 * @retval #UCL_NOP if @p data_byteLen = 0 or if @p data is the pointer #NULL
 *
 * @ingroup UCL_SHA
 */
int ucl_sha_core(u8* data, u32 data_byteLen);

/*============================================================================*/
/** <b>SHA Finish</b>.
 * Finish the process of SHA.
 *
 * @pre Hash byte length is equal to 32
 *
 * @param[out]    hash Pointer to the digest
 *
 * @warning #ucl_sha_init and #ucl_sha_core must be processed before.
 *
 * @return Error code
 *
 * @retval #UCL_OK             if no error occurred
 * @retval #UCL_INVALID_OUTPUT if @p hash is the pointer #NULL
 * @retval #UCL_INVALID_INPUT  if @p context is the pointer #NULL
 *
 * @ingroup UCL_SHA
 */
int ucl_sha_finish(u8* hash);
