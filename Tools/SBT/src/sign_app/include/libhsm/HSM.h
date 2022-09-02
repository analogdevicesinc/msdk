/*******************************************************************************
* Copyright (C) 2017 Maxim Integrated Products, Inc., All rights Reserved.
* * This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************
*
* @author : Benjamin VINOT - <benjamin.vinot@maximintegrated.com>
* @file : HSM.h
* @brief : 
*
*/

#ifndef __LIB_HSM_H__
#define __LIB_HSM_H__

#include "pkcs11/cryptoki.h"
#include "HSMU.h"

#define LIBHSM_VERS_MAJOR 1
#define LIBHSM_VERS_MINOR 0
#define LIBHSM_VERS_PATCH 1
#define LIBHSM_VERSION \
    STR_VALUE(LIBHSM_VERS_MAJOR) "." STR_VALUE(LIBHSM_VERS_MINOR) "." STR_VALUE(LIBHSM_VERS_PATCH)

#ifdef WIN32

#include "windows.h"

#define _HSM_LIBIMPEXP_

//#define CALL __cdecl
#define CALL

#else

#define _HSM_LIBIMPEXP_
#define CALL

#endif

#ifdef __cplusplus
extern "C" {
#endif

_HSM_LIBIMPEXP_ char* CALL HSM_GetVersion(void);

_HSM_LIBIMPEXP_ char* CALL HSM_GetBuildDate(void);

/**
	  * @brief Perform Token login in order to open a session
	  * @param session  Session Handler
	  * @param slotID ID of the smartcard slot used to insert OCS card, can be determined with ListSlot
	  * @retval rv
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_Login(CK_SESSION_HANDLE_PTR session, CK_ULONG slotID);

/**
	  * @brief Find a Key object according to the LABEL
	  * @param session  Session Handler
	  * @param keyname name of the key to find
	  * @param obj object handler for the found key
	  * @retval rv
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_FindKey(CK_SESSION_HANDLE session, const char* keyName,
                                       CK_BBOOL private, CK_OBJECT_HANDLE_PTR obj);

/**
	  * @brief Initialise DLL, giving the path of Thales cknfast DLL
	  * @param path path of Thales cknfast DLL
	  * @retval rv
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_InitDLL(char* path);

/**
	  * @brief Close the HSM Session
	  * @param session  Session Handler
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_Close(CK_SESSION_HANDLE session);

/**
	  * @brief List All Slot Available on stdout
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_ListSlot(void);

/**
	  * @brief List all Keys available on stdout
	  * @param session  Session Handler
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_ListKey(CK_SESSION_HANDLE session);

/**
	  * @brief Generate a HMAC SHA256 key inside the HSM ( Cannot be exported )
	  * @param session  Session Handler
	  * @param keyname name of the key to generate
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_GenerateHMACSHA256Key(CK_SESSION_HANDLE session,
                                                     const char* keyName);

/**
	  * @brief Sign Data with SHA-256 HMAC Algorithm
	  * @param session  Session Handler
	  * @param data  pointer to data to sign
	  * @param data_len length of the data to sign
	  * @param sig pointer of a buffer for the resulting signature
	  * @param siglen length of the sig buffer, return the size of the signature, should be 32
	  * @param objkey Object handle of the key to use
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_SignHMACSHA256(CK_SESSION_HANDLE session, CK_BYTE_PTR data,
                                              CK_ULONG data_len, CK_BYTE_PTR sig,
                                              CK_ULONG_PTR siglen, CK_OBJECT_HANDLE objkey);

/**
	  * @brief Import a HMAC SHA256 key inside the HSM ( need to modify CKNFAST_OVERRIDE_SECURITY_ASSURANCES=import only during this step )
	  * @param session  Session Handler
	  * @param keyvalue the key
	  * @param label name of the key to import
	  * @param objkey return an object to the imported key
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL Import_HMACSHA256Key(CK_SESSION_HANDLE session, CK_BYTE_PTR keyvalue,
                                                CK_CHAR_PTR label, CK_OBJECT_HANDLE_PTR objkey);

/**
	  * @brief Print a Human explicite meaning of error code
	  * @param rv
	  * @retval rv
	  *
	  */
_HSM_LIBIMPEXP_ CK_RV CALL HSM_pError(CK_RV rv);

_HSM_LIBIMPEXP_ CK_RV CALL HSM_EncryptRSA(CK_SESSION_HANDLE session, CK_BYTE_PTR data_in,
                                          CK_ULONG data_in_len, CK_BYTE_PTR data_out,
                                          CK_ULONG_PTR data_out_len, CK_OBJECT_HANDLE objkey);

_HSM_LIBIMPEXP_ CK_RV CALL HSM_DecryptRSA(CK_SESSION_HANDLE session, CK_BYTE_PTR data_in,
                                          CK_ULONG data_in_len, CK_BYTE_PTR data_out,
                                          CK_ULONG_PTR data_out_len, CK_OBJECT_HANDLE objkey);

_HSM_LIBIMPEXP_ CK_RV CALL HSM_GetRSAPublicKey(CK_SESSION_HANDLE session, CK_OBJECT_HANDLE obj,
                                               CK_BYTE_PTR pModulus, CK_ULONG_PTR pModulus_len,
                                               CK_BYTE_PTR pExponent, CK_ULONG_PTR pExponent_len);

CK_RV CALL HSM_GetModulusbit(CK_SESSION_HANDLE session, CK_OBJECT_HANDLE obj);

_HSM_LIBIMPEXP_ CK_RV CALL HSM_SignECDSA(CK_SESSION_HANDLE session, CK_BYTE_PTR data,
                                         CK_ULONG data_len, CK_BYTE_PTR sig, CK_ULONG_PTR siglen,
                                         CK_OBJECT_HANDLE objkey);

CK_RV CALL HSM_VerifyECDSA(CK_SESSION_HANDLE session, CK_BYTE_PTR data, CK_ULONG data_len,
                           CK_BYTE_PTR sig, CK_ULONG siglen, CK_OBJECT_HANDLE objkey);

#ifdef __cplusplus
}
#endif

#endif /* __LIB_HSM_H__ */
