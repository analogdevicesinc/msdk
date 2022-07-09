/*============================================================================
 *
 * ucl_dh.h [20-mar-06]
 *
 *==========================================================================*/
/*============================================================================
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
/*============================================================================
 *
 * Purpose :
 *
 *==========================================================================*/
#ifndef _UCL_DH_H_
#define _UCL_DH_H_

#ifdef __cplusplus
extern "C" {
#endif /* _ cplusplus  */

#ifdef __x86
#define MFPA_CTX_SIZE 40
#else
#define MFPA_CTX_SIZE 24
#endif

/** @file ucl_dh.h
 * @defgroup UCL_DH Diffie-Hellman
 * Diffie-Hellman Key Exchange Protocol.
 *
 * @par Header:
 * @link ucl_dh.h ucl_dh.h @endlink
 *
 * Diffie-Hellman key agreement provided the first practical solution to the
 * key distribution problem, allowing two parties, never having met in advance
 * or shared keying material, to establish a shared secret by exchanging
 * messages over an open channel. The security rests on the intractability of
 * the Diffie-Hellman problem and the related problem of computing discrete
 * logarithms. @n
 * @n
 * The basic version provides protection in the form of secrecy of the
 * resulting key from passive adversaries, but not from active adversaries
 * capable of intercepting, modifying, or injecting messages. Neither party
 * has assurances of the source identity of the incoming message or the
 * identity of the party which may know the resulting key, i.e., entity
 * authentication or key authentication.@n
 * @n
 * The Diffie-Hellman protocol, and those based on it, can be carried out in
 * any group in which both the discrete logarithm problem is hard and
 * exponentiation is efficient. The most common examples of such groups used in
 * practice are the multiplicative group @f$ Z_{p}^\star @f$ of @f$ Z_p @f$, the
 * analogous multiplicative group of @f$ F_{2^m} @f$, and the group of points
 * defined by an elliptic curve over a finite field.@n
 * In our case, we choose the group @f$ Z_p @f$ where @a p is a large prime.@n
 *
 * @n
 * <b>Protocol</b>:@n
 * The goal: @a A and @a B want to share a secret key @a K.@n
 * @a A and @a B use the public elements @a p and @a g where @a p (the
 * modulus) is a large prime and @a g (the base) is an element of @f$ Z_p @f$.
 * @n
 * @n
 * @li @a A and @a B generate each a secret element @f$ s_A,\ s_B \in Z_p @f$.
 * @li @a A calculate and send @f$ m_A = g^{s_A} \ mod\ p @f$ to @a B
 * @li @a B calculate and send @f$ m_B = g^{s_B} \ mod\ p @f$ to @a A
 * @li then @f$ K = m_A^{s_B} \ mod\ p = m_B^{s_A} \ mod\ p@f$
 * 
 * @ingroup UCL_PKC
 */


/* ========================================================================== */

/** <b>DH Context</b>.
 *
 * @ingroup UCL_DH
 */
struct ucl_dh_ctx
{
    u8 *module; /**< The module                             */
    u8 *base;   /**< The base                               */
    u8 *secret; /**< The secret                             */
    u32 len;    /**< Module (base and secret) byte length   */
    void *ctx;  /**< The precalculation context             */
};

/** <b>DH Context typedef</b>.
 *
 * @ingroup UCL_DH
 */
typedef struct ucl_dh_ctx ucl_dh_ctx_t;


/* ========================================================================== */

/** <b>Modular exponentiation use Montgomery method</b>.
 * @ingroup UCL_DH
 */
#define UCL_MOD_EXP_METHOD_MONTY 1

/** <b>Modular exponentiation use FBEM method</b>.
 * @ingroup UCL_DH
 */
#define UCL_MOD_EXP_METHOD_FBEM  2

/** <b>Specific error</b>.
 * @ingroup UCL_DH
 */
#define UCL_BUFFER_TOO_SMALL -101

/** <b>Specific error</b>.
 * @ingroup UCL_DH
 */
#define UCL_DATA_TOO_SMALL -102


/* ========================================================================== */

/** <b>Save the Diffie-Hellman Context</b>.
 *
 * @param[out] data    The saved context
 * @param[in]  datalen Data byte length (See the note)
 * @param[in]  dh_ctx  Pointer to a Diffie-Hellman context
 *
 * @note Data length is (s * (4 + s)) + 8
 * 
 * @warning Only for FBEM method
 * 
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_save_ctx(u8 *data, u32 datalen, ucl_dh_ctx_t dh_ctx);


/** <b>Restore the Diffie-Hellman Context</b>.
 *
 * @param[out] dh_ctx  The Diffie-Hellman context
 * @param[in]  data    A saved context
 * @param[in]  datalen Data byte length
 * @param[in]  buffer  Buffer
 * @param[in]  size    Buffer 32-bits words length (See the note)
 * 
 * @note Data length is (s * (4 + s)) + 8
 * @note Buffer size is (((2 + 2*s) * s) + 6)
 * 
 * @warning Only for FBEM method
 *
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_restore_ctx(ucl_dh_ctx_t *dh_ctx, u8 *data, u32 datalen,
                       u32 *buffer, u32 size);


/** <b>Diffie-Hellman Initialization</b>.
 * Init the context.
 * The Diffie-Hellman protocol needs some pre-computations.
 *
 * @param[out] dh_ctx Pointer to a Diffie-Hellman context
 * @param[in]  buffer Pointer to a buffer
 * @param[in]  size   Buffer 32-bit words length (See the note)
 * @param[in]  option Define the exponentiation method:
 *                      @li #UCL_MOD_EXP_METHOD_MONTY Montgomery method
 *                      @li #UCL_MOD_EXP_METHOD_FBEM  Fixed-base Euclidian Method
 * 
 * @note The size of the buffer depends on the computation methods:
 *     @li #UCL_MOD_EXP_METHOD_FBEM (((2 + 2*s) * s) + MFPA_CTX_SIZE)
 *     @li #UCL_MOD_EXP_METHOD_MONTY  ((2*s) + +MFPA_CTX_SIZE)
 * 
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_init(ucl_dh_ctx_t *dh_ctx, u32 *buffer, u32 size, int option);


/** <b>Diffie-Hellman Secret Generation</b>.
 * Generate a secret.
 *
 * @param[out] dh_ctx Pointer to a Diffie-Hellman context
 *
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_gen_secret(ucl_dh_ctx_t *dh_ctx);


/** <b>Diffie-Hellman Message Generation</b>.
 * Generate the message from the secret.
 *
 * @f$ m = g^x @f$
 *
 * @param[out] m      The message
 * @param[in]  dh_ctx Pointer to a Diffie-Hellman context
 *
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_gen_mess(u8 *m, ucl_dh_ctx_t *dh_ctx);


/** <b>Diffie-Hellman Shared-Key Generation</b>.
 * Generate the key from the second parts message and the secret.
 *
 * @f$ key = m^y @f$
 *
 * @param[out] key    The shared key
 * @param[in]  keylen The key length
 * @param[in]  m      The second part message
 * @param[in]  dh_ctx Pointer to a Diffie-Hellman context
 *
 * @return Error code
 *
 * @ingroup UCL_DH
 */
int ucl_dh_gen_key(u8 *key, u32 keylen, u8 *m, ucl_dh_ctx_t *dh_ctx);


/* ========================================================================== */
#ifdef __cplusplus
}
#endif /* _ cplusplus  */

#endif /*UCL_DH_H_*/
