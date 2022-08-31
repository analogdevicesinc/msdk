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
 *
 */

#include "cryptoki.h"

#ifndef __PKCS_H__
#define __PKCS_H__

#ifndef __IMPORT
#define __extern extern
#else
#define __extern
#endif

__extern CK_C_OpenSession pC_OpenSession; // Function pointer
__extern CK_C_CloseSession pC_CloseSession; // Function pointer
__extern CK_C_Initialize pC_Initialize; // Function pointer
__extern CK_C_Finalize pC_Finalize; // Function pointer
__extern CK_C_GetSlotList pC_GetSlotList; // Function pointer
__extern CK_C_GetSlotInfo pC_GetSlotInfo; // Function pointer

__extern CK_C_GetTokenInfo pC_GetTokenInfo;
__extern CK_C_WaitForSlotEvent pC_WaitForSlotEvent;

__extern CK_C_LoginBegin pC_LoginBegin;
__extern CK_C_LoginNext pC_LoginNext;
__extern CK_C_LoginEnd pC_LoginEnd;

__extern CK_C_SignInit pC_SignInit;
__extern CK_C_Sign pC_Sign;
__extern CK_C_SignUpdate pC_SignUpdate;
__extern CK_C_SignFinal pC_SignFinal;

__extern CK_C_EncryptInit pC_EncryptInit;
__extern CK_C_Encrypt pC_Encrypt;
__extern CK_C_EncryptUpdate pC_EncryptUpdate;
__extern CK_C_EncryptFinal pC_EncryptFinal;

__extern CK_C_FindObjectsInit pC_FindObjectsInit;
__extern CK_C_FindObjects pC_FindObjects;
__extern CK_C_FindObjectsFinal pC_FindObjectsFinal;
__extern CK_C_GetAttributeValue pC_GetAttributeValue;
__extern CK_C_SetAttributeValue pC_SetAttributeValue;
__extern CK_C_CreateObject pC_CreateObject;
__extern CK_C_GenerateKey pC_GenerateKey;
__extern CK_C_GenerateKeyPair pC_GenerateKeyPair;

__extern CK_C_WrapKey pC_WrapKey;
__extern CK_C_UnwrapKey pC_UnwrapKey;

#endif
