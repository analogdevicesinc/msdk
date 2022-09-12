#ifndef UCL_SP80056_H_
#define UCL_SP80056_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
#include <ucl/bignum_ecdsa_generic_api.h>
#include <ucl/ecdsa_generic_api.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_types.h>
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_sha1.h>
#include <ucl/ucl_sha224.h>
#include <ucl/ucl_sha256.h>
#include <ucl/ucl_sha384.h>
#include <ucl/ucl_sha512.h>
/** @file ucl_sp800_56.h
 * @defgroup UCL_ECC_SP80056 SP800-56 ECDH computation
 *
 * @par Header:
 * @link ucl_sp800_56.h ucl_sp800_56.h @endlink
 * @n
 * SP800-56A NIST document specifies how to establish a secure link between two entities U and V
 * thanks to the ECDH, Elliptic Curve Diffie-Hellman.
 * These routines implement the Ephemeral Unified Model (section 6.1.2.2)
 * Note only the routines to be used by U are provided here as the protocol and so, the routines
 * can be used symmetrically by replacing U data by V data and reciprocally
 *
 * @ingroup UCL_ECC
 */

/** <b>shared secret computation</b>.
 * this function computes the shared secret Z, using the secret key from U and the public key from V
 * as described in the SP800-56A in the section 5.7.1.2
 * @param[out]  *z: the shared secret Z
 * @param[in]  *d: the secret key for the entity U
 * @param[in]  *xV: the V ephemeral public key X coordinate
 * @param[in]  *yV: the V ephemeral public key Y coordinate
 * @param[in]  configuration: the configuration variable for transmitting the curve identifier
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 *
 * @ingroup UCL_ECC_SP80056 */
int ucl_sp800_56_ecc_cdh(u8 *z, u8 *dU, u8 *xV, u8 *yV, u32 configuration);

/** <b>secret key derivation</b>.
 * this function represents the concatenation KDF, Key Derivation Function, as described in the
 * as described in the SP800-56A in the section 5.8.1
 * @param[out]   *derivingkeymaterial: the derived key
 * @param[in]   keydatalen: the bits length of the derived key material
 * @param[in]  *z: the shared secret
 * @param[in]  zbytelength: the shared secret length in bytes
 * @param[in]  *otherinfo: the OtherInfo field as defined in section 5.8.1.2
 * @param[in] otherinfolength: the OtherInfo field bit length; this number is a multiple of 8
 * @param[in]  configuration: the configuration variable for transmitting the hash function identifier
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 *
 * limitation: the OtherInfo length is expressed in bits but is a multiple of 8 (so bytes).
 * @ingroup UCL_ECC_SP80056 */
int ucl_sp800_56_concat_kdf(u8 *derivedkeyingmaterial, u32 keydatalen, u8 *z, u32 zbytelength,
                            u8 *otherinfo, u32 otherinfobitlength, u32 configuration);

/** <b>derived key material computation and ephemeral key generation</b>.
 * this function computes the derived key material and generates the U ephemeral key
 * the derived key material is used to establish the secure link
 *
 * @param[out] *derivedkeyingmaterial: the shared secret Z
 * @param[out] keydatalen: the length of the shared secret
 * @param[out] *dU: the newly generated secret key for the entity U
 * @param[out] *xU: the newly generated U ephemeral key X coordinate using the ucl_type_ecc_u8_affine_point structure
 * @param[out] *yU: the newly generated U ephemeral key Y coordinate using the ucl_type_ecc_u8_affine_point structure
 * @param[in]  *xV: the V ephemeral public key X coordinate using the ucl_type_ecc_u8_affine_point structure
 * @param[in]  *yV: the V ephemeral public key Y coordinate using the ucl_type_ecc_u8_affine_point structure
 * @param[in]  *otherinfo: the OtherInfo field as defined in section 5.8.1.2
 * @param[in]  otherinfolength: the OtherInfo field bit length; this number is a multiple of 8
 * @param[in] configuration: the configuration variable for transmitting the curve and hash function identifiers
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 * @retval #UCL_ERROR if any problem in key generation or Z shared secret computation
 *
 * limitation: the OtherInfo length is expressed in bits but is a multiple of 8 (so bytes).
 * @ingroup UCL_ECC_SP80056 */
int ucl_sp800_56_eum_ecc_cdh(u8 *derivedkeyingmaterial, u32 keydatalen, u8 *dU, u8 *xU, u8 *yU,
                             u8 *xV, u8 *yV, u8 *otherinfo, u32 otherinfobitlength,
                             u32 configuration);

/** <b>shared secret computation</b>.
 * this function computes the shared secret Z, using the secret key from U and the public key from V
 * as described in the SP800-56A section 5.7.1.2
 * using the ECC API introduced in UCL 2.4.9
 *
 * @param[out]  *z: the shared secret Z
 * @param[in]   *dU: the secret key for the entity U
 * @param[in] V: the V ephemeral public key using the ucl_type_ecc_u8_affine_point structure
 * @param[in] curve_params: a pointer to the curve domain parameters
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 *
 * @ingroup UCL_ECC_SP80056 */
int ucl_ecc_sp800_56_ecc_cdh(u8 *z, u8 *dU, ucl_type_ecc_u8_affine_point V,
                             ucl_type_curve *curve_params);

/** <b>derived key material computation and ephemeral key generation</b>.
 * this function computes the derived key material and generates the U ephemeral key
 * using the ECC API introduced in UCL 2.4.9
 * the derived key material is used to establish the secure link
 *
 * @param[out]   *z: the shared secret Z
 * @param[out]   *d: the newly generated secret key for the entity U
 * @param[out] U: the newly generated U ephemeral key using the ucl_type_ecc_u8_affine_point structure
 * @param[in] V: the V ephemeral public key using the ucl_type_ecc_u8_affine_point structure
 * @param[in]  *otherinfo: the OtherInfo field as defined in section 5.8.1.2
 * @param[in]  otherinfolength: the OtherInfo field bit length; this number is a multiple of 8
 * @param[in] curve_params: a pointer to the curve domain parameters
 * @param[in]  configuration: the configuration variable for transmitting the hash function identifier
 * @return Error code
 * 
 * @retval #UCL_OK              No error occurred
 * @retval #UCL_INVALID_INPUT  if one of the inputs is the pointer NULL
 * @retval #UCL_INVALID_OUTPUT  if one of the outputs is the pointer NULL
 * @retval #UCL_ERROR if any problem in key generation or Z shared secret computation
 *
 * limitation: the OtherInfo length is expressed in bits but is a multiple of 8 (so bytes).
 * @ingroup UCL_ECC_SP80056 */
int ucl_ecc_sp800_56_eum_ecc_cdh(u8 *derivedkeyingmaterial, u32 keydatalen, u8 *dU,
                                 ucl_type_ecc_u8_affine_point U, ucl_type_ecc_u8_affine_point V,
                                 u8 *otherinfo, u32 otherinfobitlength,
                                 ucl_type_curve *curve_params, u32 configuration);
#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif
