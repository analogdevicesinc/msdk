#ifndef UCL_X931_SHA256_H_
#define UCL_X931_SHA256_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus  */
#include <ucl/ucl_config.h>
#include <ucl/ucl_defs.h>
#include <ucl/ucl_retdefs.h>
#include <ucl/ucl_rsa.h>
#include <ucl/ucl_sha256.h>
#include <ucl/ucl_sys.h>
#include <ucl/ucl_types.h>

int ucl_x931_sha1_encode(u8* EM, u8* message, u32 message_length, u32 k);
int ucl_x931_sha256_encode(u8* EM, u8* message, u32 message_length, u32 k);

int ucl_x931_ssa_sha256_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_private_key_t* keyPr);
int ucl_x931_ssa_sha256_crt_sign(
    u8* signature, u8* message, u32 message_length, ucl_rsa_crt_private_key_t* keyPr);
int ucl_x931_ssa_sha256_verify(
    u8* signature, u8* message, u32 message_length, ucl_rsa_public_key_t* keyPu);
int ucl_x931_sha256_verify(u8* EM, u8* message, u32 message_length, u32 k);
#ifdef __cplusplus
}
#endif /* __cplusplus  */

#endif // X931
