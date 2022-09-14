#ifndef CRYPTOKI_H
#define CRYPTOKI_H
/*
 * (C) Copyright nCipher Corporation Limited 1998-2000.
 * All rights reserved.
 */

/* Parts of this file are derived from the RSA Data Security, Inc.
   Cryptoki Cryptographic Token Interface */
/* Before including pkcs11.h, 6 platform-specific macros must be defined.
 * These macros are described in pkcs11.h, and defined here.
 */

#ifdef _MSC_VER
#if defined(_WIN32) /* win32 */
#define CK_PTR *

/* #define STATIC_BUILD to avoid __declspec(dllimport) etc.
   even when compiling with /MD */
#ifdef STATIC_BUILD
#define CK_DEFINE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType(*name)
#else
#ifdef _DLL
#ifdef BUILDING_CRYPTOKI
/* BUILDING_CRYPTOKI should be defined iff we are compiling
	the library contents, not just using it. */
#define CK_DEFINE_FUNCTION(returnType, name) returnType __declspec(dllexport) name
#define CK_DECLARE_FUNCTION(returnType, name) returnType __declspec(dllexport) name
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType __declspec(dllexport)(*name)
#else
#define CK_DEFINE_FUNCTION(returnType, name) returnType __declspec(dllexport) name
#define CK_DECLARE_FUNCTION(returnType, name) returnType __declspec(dllimport) name
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType __declspec(dllimport)(*name)
#endif
#else
/* Win32, not DLL */
#define CK_DEFINE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType(*name)
#endif
#endif

#define CK_CALLBACK_FUNCTION(returnType, name) returnType(*name)

#else /* win16 */
/* We don't support win16. This may be wrong or incomplete,
        it's just taken from the example in the spec. */
#define CK_PTR far *
#define CK_DEFINE_FUNCTION(returnType, name) returnType __export _far _pascal name
#ifdef BUILDING_CRYPTOKI
#define CK_DECLARE_FUNCTION(returnType, name) returnType __export _far _pascal name
#else
#define CK_DECLARE_FUNCTION(returnType, name) returnType __import _far _pascal name
#endif
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType __export _far _pascal(*name)
#define CK_CALLBACK_FUNCTION(returnType, name) returnType _far _pascal(*name)
#endif

#else /* not windows */

#define CK_PTR *
#define CK_DEFINE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION(returnType, name) returnType name
#define CK_DECLARE_FUNCTION_POINTER(returnType, name) returnType(*name)
#define CK_CALLBACK_FUNCTION(returnType, name) returnType(*name)
#endif

#ifndef NULL_PTR
#define NULL_PTR 0
#endif

/* I tried "#pragma PACK_PRAGMA" inside an #ifdef, but
   the preprocessor doesn't necessarily substitute
   defines inside a #pragma directive.
   The pack(1) style is used by SUNWspro as well, but
   the library won't actually compile packed on Solaris.
*/
#if defined(_WIN32)
#pragma pack(push, cryptoki, 1)
#else
#ifdef PACK_PRAGMA_STYLE_GCC
#pragma pack(1)
#endif
#ifdef PACK_PRAGMA_STYLE_HPUX
#pragma pack 1
#endif
#ifdef PACK_PRAGMA_STYLE_AIX
#pragma options align = packed
#endif
#endif

/* The standard RSA supplied header */
#include "pkcs11.h"

/* Non-standard API entry points, vendor defined constants */
#include "pkcs11extra.h"

#if defined(_WIN32)
#pragma pack(pop, cryptoki)
/* Leave 4103 disabled, for VC8 */
/*#pragma warning(default: 4103)*/
#else
#ifdef PACK_PRAGMA_STYLE_GCC
#pragma pack(4)
#endif
#ifdef PACK_PRAGMA_STYLE_HPUX
#pragma pack 4
#endif
#ifdef PACK_PRAGMA_STYLE_AIX
#pragma options align = full
#endif
#endif

#endif /* CRYPTOKI_H */
