/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      Crypto driver implementation.
 *
 * Copyright (c) 2019-2020 Packetcraft, Inc.  All rights reserved.
 * Packetcraft, Inc. confidential and proprietary.
 *
 * IMPORTANT.  Your use of this file is governed by a Software License Agreement
 * ("Agreement") that must be accepted in order to download or otherwise receive a
 * copy of this file.  You may not use or copy this file for any purpose other than
 * as described in the Agreement.  If you do not agree to all of the terms of the
 * Agreement do not use this file and delete all copies in your possession or control;
 * if you do not have a copy of the Agreement, you must contact Packetcraft, Inc. prior
 * to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <string.h>
#include "mxc_device.h"
#include "pal_crypto.h"
#include "pal_sys.h"
#include "pal_bb_ble.h"
#include "pal_timer.h"
#include "ctb_regs.h"
#include "ctb.h"
#include "trng.h"
#include "util/bstream.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/
/*! \brief convert little endian byte buffer to uint16_t. */
#define BYTES_TO_UINT16(n, p)                             \
    {                                                     \
        n = ((uint16_t)(p)[0] + ((uint16_t)(p)[1] << 8)); \
    }

/*! \brief      Data PDU header length. */
#define BB_DATA_PDU_HDR_LEN 2

/*! \brief      Block size. */
#define BB_AES_BLOCK_SIZE 16

/*! \brief      Length field data PDU offset. */
#define BB_DATA_PDU_LEN_OFFSET 1

/*! \brief      Encryption modes. */
enum {
    PAL_CRYPTO_MODE_ENC, /*!< Encryption mode. */
    PAL_CRYPTO_MODE_DEC, /*!< Decryption mode. */
    PAL_CRYPTO_MODE_TOTAL /*!< Total modes. */
};

#ifdef DEBUG

/*! \brief      Parameter check. */
#define PAL_CRYPTO_PARAM_CHECK(expr) \
    {                                \
        if (!(expr)) {               \
            return;                  \
        }                            \
    }

/*! \brief      Parameter check, with return value. */
#define PAL_CRYPTO_PARAM_CHECK_RET(expr, rv) \
    {                                        \
        if (!(expr)) {                       \
            return (rv);                     \
        }                                    \
    }

#else

/*! \brief      Parameter check (disabled). */
#define PAL_CRYPTO_PARAM_CHECK(expr)

/*! \brief      Parameter check, with return value (disabled). */
#define PAL_CRYPTO_PARAM_CHECK_RET(expr, rv)

#endif

#ifndef PAL_CRYPTO_MAX_ID
#define PAL_CRYPTO_MAX_ID 14 /*!< Absolute maximum number of cipher blocks. */
#endif

#ifndef BB_ENABLE_INLINE_ENC_TX
#define BB_ENABLE_INLINE_ENC_TX FALSE
#endif

#ifndef BB_ENABLE_INLINE_DEC_RX
#define BB_ENABLE_INLINE_DEC_RX FALSE
#endif

#ifdef __riscv
uint32_t __REV(uint32_t rev)
{
    uint32_t retval = 0;

    retval |= (rev & 0x000000FF) << 24;
    retval |= (rev & 0x0000FF00) << 8;
    retval |= (rev & 0x00FF0000) >> 8;
    retval |= (rev & 0xFF000000) >> 24;

    return retval;
}
#endif

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! \brief   Encryption cipher block table. */
typedef union {
    uint8_t b[BB_AES_BLOCK_SIZE]; /*!< Byte access block. */
    uint32_t w[BB_AES_BLOCK_SIZE / sizeof(uint32_t)]; /*!< Word access block. */

    struct {
        uint8_t flags[1]; /*!< Flags. */
        uint8_t pctr[5]; /*!< Control. */
        uint8_t iv[8]; /*!< IV. */
    } f; /*!< Field access. */
} palCryptoCipherBlk_t;

/*! \brief   ECB data. */
typedef union {
    struct {
        uint8_t key[BB_AES_BLOCK_SIZE]; /*!< Key. */
        uint8_t clear[BB_AES_BLOCK_SIZE]; /*!< Clear. */
        uint8_t cipher[BB_AES_BLOCK_SIZE]; /*!< Cipher. */
    } b; /*!< Byte access block. */
    struct {
        uint32_t key[BB_AES_BLOCK_SIZE / sizeof(uint32_t)]; /*!< Key. */
        uint32_t clear[BB_AES_BLOCK_SIZE / sizeof(uint32_t)]; /*!< Clear. */
        uint32_t cipher[BB_AES_BLOCK_SIZE / sizeof(uint32_t)]; /*!< Cipher. */
    } w; /*!< Word acess block. */
} palCryptoEcbData_t;

struct {
    uint32_t rngW;
    uint32_t rngX;
    uint32_t rngY;
    uint32_t rngZ;
} palCryptoCb;

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

#if (!BB_ENABLE_INLINE_ENC_TX && !BB_ENABLE_INLINE_DEC_RX)
/*! \brief      Cipher block context. */
static palCryptoCipherBlk_t palCryptoCipherBlkTbl[PAL_CRYPTO_MAX_ID][PAL_CRYPTO_MODE_TOTAL];
#endif

/*! \brief      ECB encryption data block. */
static palCryptoEcbData_t palCryptoEcb;

/**************************************************************************************************
  Functions
**************************************************************************************************/

static void palCryptoAesEcb(const uint8_t *key, const uint8_t *pt, uint8_t *ct)
{
    uint32_t key32[16 / 4];
    uint32_t pt32[16 / 4];
    uint32_t ct32[16 / 4];

    /* Align the key and pt */
    memcpy(key32, (uint32_t *)key, 16);
    memcpy(pt32, (uint32_t *)pt, 16);

    MXC_CTB_Init(MXC_CTB_FEATURE_CIPHER);

    /* Reset the CTB */
    MXC_CTB->ctrl = MXC_F_CTB_CTRL_RST;

    /* Set the legacy bit */
    MXC_CTB->ctrl |= MXC_F_CTB_CTRL_FLAG_MODE;

    /* Byte swap the input and output */
    MXC_CTB->ctrl |= MXC_F_CTB_CTRL_BSO;
    MXC_CTB->ctrl |= MXC_F_CTB_CTRL_BSI;

    /* Clear interrupt flags */
    MXC_CTB->ctrl |= MXC_F_CTB_CTRL_CPH_DONE;

    /* Setup the key source */
    MXC_CTB->cipher_ctrl = MXC_S_CTB_CIPHER_CTRL_SRC_CIPHERKEY;

    /* Setup the CT calculation */
    MXC_CTB->cipher_ctrl |= MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128;

    MXC_CTB->cipher_key[0] = key32[3];
    MXC_CTB->cipher_key[1] = key32[2];
    MXC_CTB->cipher_key[2] = key32[1];
    MXC_CTB->cipher_key[3] = key32[0];

    /* Wait for the ready flag */
    while (!(MXC_CTB->ctrl & MXC_F_CTB_CTRL_RDY)) {}

    /* Copy data to start the operation */
    MXC_CTB->din[0] = pt32[3];
    MXC_CTB->din[1] = pt32[2];
    MXC_CTB->din[2] = pt32[1];
    MXC_CTB->din[3] = pt32[0];

    /* Wait for and clear the done flag */
    while (!(MXC_CTB->ctrl & MXC_F_CTB_CTRL_CPH_DONE)) {}
    MXC_CTB->ctrl |= MXC_F_CTB_CTRL_CPH_DONE;

    ct32[3] = MXC_CTB->dout[0];
    ct32[2] = MXC_CTB->dout[1];
    ct32[1] = MXC_CTB->dout[2];
    ct32[0] = MXC_CTB->dout[3];

    /* Copy the data out */
    memcpy((uint32_t *)ct, (uint32_t *)ct32, 16);

    MXC_CTB_Shutdown(MXC_CTB_FEATURE_CIPHER);
}

/*************************************************************************************************/
/*!
 *  \brief      Generate random number.
 *
 *  \return     32-bit random number.
 *
 *  This software implementation uses a xorshift random number generator.
 *      George Marsaglia (2003), "Xorshift RNGs", Journal of Statistical Software
 *
 *  \note       This routine is not a cryptographic grade random number generator.
 */
/*************************************************************************************************/
static uint32_t prandNum(void)
{
    uint32_t t;

    t = palCryptoCb.rngX ^ (palCryptoCb.rngX << 11);
    palCryptoCb.rngX = palCryptoCb.rngY;
    palCryptoCb.rngY = palCryptoCb.rngZ;
    palCryptoCb.rngZ = palCryptoCb.rngW;
    palCryptoCb.rngW = palCryptoCb.rngW ^ (palCryptoCb.rngW >> 19) ^ (t ^ (t >> 8));

    return palCryptoCb.rngW;
}

/*************************************************************************************************/
/*!
 *  \brief  XOR block.
 */
/*************************************************************************************************/
static void palXor128(const uint8_t *pInA, const uint8_t *pInB, uint8_t *pOut)
{
    const uint32_t *pInA_w = (uint32_t *)pInA;
    const uint32_t *pInB_w = (uint32_t *)pInB;
    uint32_t *pOut_w = (uint32_t *)pOut;

    pOut_w[0] = pInA_w[0] ^ pInB_w[0];
    pOut_w[1] = pInA_w[1] ^ pInB_w[1];
    pOut_w[2] = pInA_w[2] ^ pInB_w[2];
    pOut_w[3] = pInA_w[3] ^ pInB_w[3];
}

/*************************************************************************************************/
/*!
 *  \brief  Shift block left by 1 bit.
 */
/*************************************************************************************************/
void palShiftLeft128(const uint8_t *pIn, uint8_t *pOut)
{
    uint8_t of = 0;

    for (int i = 15; i >= 0; i--) {
        pOut[i] = pIn[i] << 1;
        pOut[i] |= of;

        of = pIn[i] >> 7;
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Generate subkeys.
 */
/*************************************************************************************************/
static void palGenSubkey(const uint8_t *pKey, uint8_t *pK1, uint8_t *pK2)
{
    static const uint8_t Rb[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87 };

    unsigned char L[16];
    unsigned char Z[16] = { 0 };
    unsigned char t[16];

    palCryptoAesEcb((const uint8_t *)pKey, (const uint8_t *)Z, (uint8_t *)L);

    if ((L[0] & 0x80) == 0) {
        palShiftLeft128(L, pK1);
    } else {
        palShiftLeft128(L, t);
        palXor128(t, Rb, pK1);
    }

    if ((pK1[0] & 0x80) == 0) {
        palShiftLeft128(pK1, pK2);
    } else {
        palShiftLeft128(pK1, t);
        palXor128(t, Rb, pK2);
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Pad block.
 */
/*************************************************************************************************/
static void palPadBlock(const uint8_t *pIn, uint8_t *pOut, uint8_t len)
{
    for (size_t i = 0; i < BB_AES_BLOCK_SIZE; i++) {
        if (i < len) {
            pOut[i] = pIn[i];
        } else if (i == len) {
            pOut[i] = 0x80;
        } else {
            pOut[i] = 0x00;
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Execute AES ECB.
 */
/*************************************************************************************************/
static void palCryptoExecuteAesEcb(void)
{
    palCryptoAesEcb((const uint8_t *)palCryptoEcb.w.key, (const uint8_t *)palCryptoEcb.w.clear,
                    (uint8_t *)palCryptoEcb.w.cipher);
}

/*************************************************************************************************/
/*!
 *  \brief  Load Nordic AES ECB data.
 *
 *  \param  pEnc        Encryption parameters.
 */
/*************************************************************************************************/
static inline void palCryptoLoadEcbData(PalCryptoEnc_t *pEnc)
{
    uint32_t *pSkW = (uint32_t *)pEnc->sk;
    palCryptoEcb.w.key[0] = __REV(pSkW[3]);
    palCryptoEcb.w.key[1] = __REV(pSkW[2]);
    palCryptoEcb.w.key[2] = __REV(pSkW[1]);
    palCryptoEcb.w.key[3] = __REV(pSkW[0]);

    /* NRF_ECB->ECBDATAPTR = (uint32_t)palCryptoEcb.w.key; */
}

/*************************************************************************************************/
/*!
 *  \brief  Encrypt/decrypt PDU.
 *
 *  \param  pAx         Ax cipher block template.
 *  \param  pMic        Inplace MIC buffer.
 *  \param  pBuf        Inplace cleartext/ciphertext buffer.
 *  \param  pldLen      Length of buffer payload.
 */
/*************************************************************************************************/
static void palCryptoPdu(palCryptoCipherBlk_t *pAx, uint8_t *pMic, uint8_t *pBuf, uint16_t pldLen)
{
    /* X_1 := ECB(K, A_0) */
    palCryptoEcb.w.clear[0] = pAx->w[0];
    palCryptoEcb.w.clear[1] = pAx->w[1];
    palCryptoEcb.w.clear[2] = pAx->w[2];
    palCryptoEcb.w.clear[3] = pAx->w[3];
    palCryptoExecuteAesEcb();
    pMic[0] ^= palCryptoEcb.b.cipher[0]; /* MIC */
    pMic[1] ^= palCryptoEcb.b.cipher[1];
    pMic[2] ^= palCryptoEcb.b.cipher[2];
    pMic[3] ^= palCryptoEcb.b.cipher[3];

    uint16_t actLen = 0;

    /* Preload static Ax values. */
    palCryptoEcb.w.clear[0] = pAx->w[0];
    palCryptoEcb.w.clear[1] = pAx->w[1];
    palCryptoEcb.w.clear[2] = pAx->w[2];
    palCryptoEcb.w.clear[3] = pAx->w[3];

    for (unsigned int offs = 0; offs < pldLen; offs += 16) {
        /* X_i := ECB(K, X_i XOR A_i, for i=1..n */
        palCryptoEcb.b.clear[15]++;
        palCryptoExecuteAesEcb();

        for (unsigned int i = 0; i < sizeof(palCryptoEcb.b.cipher); i++) {
            pBuf[actLen++] ^= palCryptoEcb.b.cipher[i];

            if (actLen == pldLen) {
                break;
            }
        }
    }
}

/*************************************************************************************************/
/*!
 *  \brief  Authenticate PDU.
 *
 *  \param  type        Encryption type.
 *  \param  pBx         Bx cipher block template.
 *  \param  pMic        Storage for computed MIC.
 *  \param  pHdr        Header buffer.
 *  \param  pBuf        Inplace cleartext/ciphertext buffer.
 *  \param  pldLen      Length of payload.
 */
/*************************************************************************************************/
static void palCryptoAuthPdu(uint8_t type, palCryptoCipherBlk_t *pBx, uint8_t *pMic, uint8_t *pHdr,
                             uint8_t *pBuf, uint16_t pldLen)
{
    const uint8_t IGNORE_BITS_ACL = 0x1C; /* NESN, SN and MD */
    const uint8_t IGNORE_BITS_CIS = 0x5C; /* NESN, SN, NPI and CIE */
    const uint8_t IGNORE_BITS_BIS = 0x3C; /* CSSN and CSTF */

    uint8_t ignoreBits;

    switch (type) {
    case PAL_BB_TYPE_ACL:
        ignoreBits = IGNORE_BITS_ACL;
        break;
    case PAL_BB_TYPE_CIS:
        ignoreBits = IGNORE_BITS_CIS;
        break;
    default: /* PAL_BB_TYPE_BIS */
        ignoreBits = IGNORE_BITS_BIS;
        break;
    }

    /* X_1 := ECB(K, B_0) */
    palCryptoEcb.w.clear[0] = pBx->w[0];
    palCryptoEcb.w.clear[1] = pBx->w[1];
    palCryptoEcb.w.clear[2] = pBx->w[2];
    palCryptoEcb.w.clear[3] = pBx->w[3];
    palCryptoEcb.b.clear[0] = 0x49; /* Flags */
    palCryptoEcb.b.clear[15] = pldLen; /* Length[LSO] */
    palCryptoExecuteAesEcb();

    /* X_2 := ECB(K, X_1 XOR B_1) */
    palCryptoEcb.b.clear[0] = 0x00; /* AAD_Length[MSO] */
    palCryptoEcb.b.clear[1] = 0x01; /* AAD_Length[LSO] */
    palCryptoEcb.b.clear[2] = pHdr[0] & ~ignoreBits; /* AAD */
    palCryptoEcb.b.clear[3] = 0x00;
    palCryptoEcb.w.clear[0] ^= palCryptoEcb.w.cipher[0];
    palCryptoEcb.w.clear[1] = palCryptoEcb.w.cipher[1];
    palCryptoEcb.w.clear[2] = palCryptoEcb.w.cipher[2];
    palCryptoEcb.w.clear[3] = palCryptoEcb.w.cipher[3];
    palCryptoExecuteAesEcb();

    for (unsigned int offset = 0; offset < pldLen; offset += 16) {
        size_t len = (pldLen < (offset + 16)) ? (pldLen - offset) : 16;

        /* X_i := ECB(K, X_(i-1) XOR B_(i-1)  for i=3..n */
        palCryptoEcb.w.clear[0] = 0;
        palCryptoEcb.w.clear[1] = 0;
        palCryptoEcb.w.clear[2] = 0;
        palCryptoEcb.w.clear[3] = 0;
        memcpy(palCryptoEcb.b.clear, pBuf + offset, len);
        palCryptoEcb.w.clear[0] ^= palCryptoEcb.w.cipher[0];
        palCryptoEcb.w.clear[1] ^= palCryptoEcb.w.cipher[1];
        palCryptoEcb.w.clear[2] ^= palCryptoEcb.w.cipher[2];
        palCryptoEcb.w.clear[3] ^= palCryptoEcb.w.cipher[3];
        palCryptoExecuteAesEcb();
    }

    /* Store MIC. */
    pMic[0] = palCryptoEcb.b.cipher[0];
    pMic[1] = palCryptoEcb.b.cipher[1];
    pMic[2] = palCryptoEcb.b.cipher[2];
    pMic[3] = palCryptoEcb.b.cipher[3];
}

/*************************************************************************************************/
/*!
 *  \brief  Increment cipher block packet counter.
 *
 *  \param  pCb         Cipher block.
 */
/*************************************************************************************************/
static inline void palCryptoIncPktCnt(palCryptoCipherBlk_t *pCb)
{
    /* Unpack packetCounter. */
    uint64_t pktCnt = ((uint64_t)pCb->f.pctr[0] << 0) + ((uint64_t)pCb->f.pctr[1] << 8) +
                      ((uint64_t)pCb->f.pctr[2] << 16) + ((uint64_t)pCb->f.pctr[3] << 24) +
                      ((uint64_t)(pCb->f.pctr[4] & 0x7F) << 32);

    /* Increment packetCounter. */
    pktCnt++;

    /* Pack packetCounter. */
    pCb->f.pctr[0] = pktCnt >> 0;
    pCb->f.pctr[1] = pktCnt >> 8;
    pCb->f.pctr[2] = pktCnt >> 16;
    pCb->f.pctr[3] = pktCnt >> 24;

    /* Preserve directionBit. */
    pCb->f.pctr[4] = (pCb->f.pctr[4] & 0x80) | ((pktCnt >> 32) & 0x7F);
}

/*************************************************************************************************/
/*!
 *  \brief  Load event counter.
 *
 *  \param  pCb         Cipher block.
 *  \param  evtCnt      Connection event counter.
 */
/*************************************************************************************************/
static inline void palCryptoLoadPktCnt(palCryptoCipherBlk_t *pCb, uint16_t evtCnt)
{
    /* Pack connEventCounter. */
    pCb->f.pctr[0] = evtCnt >> 0;
    pCb->f.pctr[1] = evtCnt >> 8;
    pCb->f.pctr[2] = 0;
    pCb->f.pctr[3] = 0;

    /* Preserve directionBit. */
    pCb->f.pctr[4] &= 0x80;
}

/*************************************************************************************************/
/*!
 *  \brief  Load event counter.
 *
 *  \param  pCb         Cipher block.
 *  \param  evtCnt      Connection event counter.
 */
/*************************************************************************************************/
static inline void palCryptoLoadIsoPktCnt(palCryptoCipherBlk_t *pCb, uint64_t pktCnt)
{
    /* Pack connEventCounter. */
    pCb->f.pctr[0] = pktCnt >> 0;
    pCb->f.pctr[1] = pktCnt >> 8;
    pCb->f.pctr[2] = pktCnt >> 16;
    pCb->f.pctr[3] = pktCnt >> 32;
    /* Preserve directionBit. */
    pCb->f.pctr[4] = (pCb->f.pctr[4] & 0x80) | ((pktCnt >> 32) & 0x7F);
}

/*************************************************************************************************/
/*!
 *  \brief      Initialize crypto resources.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalCryptoInit(void)
{
    uint32_t key[4];
    uint32_t in[4];
    uint32_t out[4];

    /* Seed RNG */
    palCryptoCb.rngW = 0x733c9fe6;
    palCryptoCb.rngX = 0x90b15126;
    palCryptoCb.rngY = 0x4a497de3;
    palCryptoCb.rngZ = 0xf3a82809;

    /* Use TRNG to seed the RNG */
    MXC_CTB_Init(MXC_CTB_FEATURE_TRNG);

    MXC_CTB_TRNG_Random((uint8_t *)key, 16);
    MXC_CTB_TRNG_Random((uint8_t *)in, 16);

    key[0] ^= 0xa0cc5ea4;
    key[1] ^= 0x2980e0c4;
    key[2] ^= 0xdd617602;
    key[3] ^= 0xe1e21e73;

    in[0] ^= 0x7a7f37da;
    in[1] ^= 0x017e0fd9;
    in[2] ^= 0x133e23cc;
    in[3] ^= 0x46a4e91a;

    MXC_CTB_Shutdown(MXC_CTB_FEATURE_TRNG);

    PalCryptoAesEcb((const uint8_t *)key, (uint8_t *)out, (const uint8_t *)in);

    palCryptoCb.rngW ^= out[0];
    palCryptoCb.rngX ^= out[1];
    palCryptoCb.rngY ^= out[2];
    palCryptoCb.rngZ ^= out[3];
}

/*************************************************************************************************/
/*!
 *  \brief      De-Initialize crypto resources.
 *
 *  \return     None.
 */
/*************************************************************************************************/
void PalCryptoDeInit(void)
{
    MXC_CTB_Shutdown(MXC_CTB_FEATURE_TRNG | MXC_CTB_FEATURE_CIPHER);
}

/*************************************************************************************************/
/*!
 *  \fn     PalCryptoAesEcb
 *
 *  \brief  Calculate AES CBC.
 *
 *  \param  pKey        Encryption key.
 *  \param  pOut        Output data.
 *  \param  pIn         Input data.
 *
 *  \return None.
 *
 *  \note   Packet length is 16 bytes. pOut could be equal to pIn.
 */
/*************************************************************************************************/
void PalCryptoAesEcb(const uint8_t *pKey, uint8_t *pOut, const uint8_t *pIn)
{
    palCryptoAesEcb((const uint8_t *)pKey, (const uint8_t *)pIn, (uint8_t *)pOut);
}

/*************************************************************************************************/
/*!
 *  \fn     PalCryptoAesCmac
 *
 *  \brief  Calculate AES CMAC.
 *
 *  \param  pKey        Encryption key.
 *  \param  pOut        Output data.
 *  \param  pIn         Input data.
 *
 *  \note   Packet length is 16 bytes.
 */
/*************************************************************************************************/
void PalCryptoAesCmac(const uint8_t *pKey, uint8_t *pOut, const uint8_t *pIn, uint16_t len)
{
    uint32_t alignKey[4];
    memcpy(alignKey, pKey, sizeof(alignKey));

    uint32_t revKey[4];
    revKey[0] = __REV(alignKey[3]);
    revKey[1] = __REV(alignKey[2]);
    revKey[2] = __REV(alignKey[1]);
    revKey[3] = __REV(alignKey[0]);

    uint32_t *pIn_w = (uint32_t *)pIn;
    uint32_t revIn[4];
    if (len == 16) {
        revIn[0] = __REV(pIn_w[3]);
        revIn[1] = __REV(pIn_w[2]);
        revIn[2] = __REV(pIn_w[1]);
        revIn[3] = __REV(pIn_w[0]);
    } else {
        revIn[0] = __REV(pIn_w[0]);
    }

    uint8_t K1[BB_AES_BLOCK_SIZE], K2[BB_AES_BLOCK_SIZE];
    palGenSubkey((const uint8_t *)revKey, K1, K2);

    uint32_t alignM[4];
    if (len == BB_AES_BLOCK_SIZE) {
        /* Complete block. */
        palXor128((uint8_t *)revIn, K1, (uint8_t *)alignM);
    } else {
        uint32_t alignInPad[4];
        /* Partial block. */
        palPadBlock((uint8_t *)revIn, (uint8_t *)alignInPad, len);
        palXor128((uint8_t *)alignInPad, K2, (uint8_t *)alignM);
    }

    const uint32_t alignX[4] = { 0 };
    uint32_t alignY[4];
    palXor128((const uint8_t *)alignX, (uint8_t *)alignM, (uint8_t *)alignY);

    uint32_t alignOut[4];

    palCryptoAesEcb((const uint8_t *)revKey, (const uint8_t *)alignY, (uint8_t *)alignOut);

    uint32_t revOut[4];
    revOut[0] = __REV(alignOut[3]);
    revOut[1] = __REV(alignOut[2]);
    revOut[2] = __REV(alignOut[1]);
    revOut[3] = __REV(alignOut[0]);

    memcpy(pOut, revOut, sizeof(revOut));
}

/*************************************************************************************************/
/*!
 *  \brief  Execute AES CBC transformation on payload and add 4 byte MIC.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  id          Context ID.
 *  \param  localDir    Direction bit of local device (0=slave, 1=master).
 *
 *  This routine completes the transformation in a blocking manner.
 *
 *  \note   Leave this implementation empty if inline hardware encryption is available.
 */
/*************************************************************************************************/
#if (!BB_ENABLE_INLINE_ENC_TX || !BB_ENABLE_INLINE_DEC_RX)
void PalCryptoAesEnable(PalCryptoEnc_t *pEnc, uint8_t id, uint8_t localDir)
{
    unsigned int mode;

    if (id > PAL_CRYPTO_MAX_ID) {
        PAL_SYS_ASSERT(0);
        return;
    }

    /* Clear */
    memset(&palCryptoCipherBlkTbl[id], 0, sizeof(palCryptoCipherBlkTbl[id]));

    /* Loop through Tx/encrypt and Rx/decrypt cipher blocks. */
    for (mode = 0; mode < PAL_CRYPTO_MODE_TOTAL; mode++) {
        /* Compute A0 and B0 */
        uint8_t *pBlk = palCryptoCipherBlkTbl[id][mode].b;
        pBlk[0] = 0x01; /* Flags: initialize Ax */
        /* pBlk[1..5] = { 0 }; */ /* Nonce::packetCounter clear to 0 */

        if (pEnc->type == PAL_BB_TYPE_ACL || pEnc->type == PAL_BB_TYPE_CIS) {
            pBlk[5] = (mode == PAL_CRYPTO_MODE_ENC) ? (localDir << 7) :
                                                      (!localDir << 7); /* Nonce::directionBit */
        } else { /* PAL_BB_TYPE_BIS */
            pBlk[5] = (localDir << 7); /* Nonce::directionBit */
        }

        memcpy(&pBlk[6], pEnc->iv, PAL_CRYPTO_LL_IV_LEN); /* Nonce::IV */
        /* pBlk[14] = 0x00; */ /* Length[MSO]: always 0 */
        /* pBlk[15] = 0x00; */ /* Length[LSO]: set on use */
    }

    /* Store context. */
    pEnc->pEncryptCtx = &palCryptoCipherBlkTbl[id][PAL_CRYPTO_MODE_ENC];
    pEnc->pDecryptCtx = &palCryptoCipherBlkTbl[id][PAL_CRYPTO_MODE_DEC];
}

/*************************************************************************************************/
/*!
 *  \brief  Disable AES cipher block.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  id          Context ID.
 *  \param  localDir    Direction bit of local device (0=slave, 1=master).
 *
 *  \return None.
 */
/*************************************************************************************************/
void PalCryptoAesDisable(PalCryptoEnc_t *pEnc, uint8_t id, uint8_t dir) {}
#endif

/*************************************************************************************************/
/*!
 *  \brief  Execute AES CBC transformation on payload and add 4 byte MIC.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  pHdr        Packet header.
 *  \param  pBuf        Packet data.
 *  \param  pMic        Storage for MIC.
 *
 *  \return TRUE if the MIC was set.
 *
 *  This routine completes the transformation in a blocking manner. This routine modifies the
 *  length field of the PDU to account for the MIC.
 *
 *  \note   Leave this implementation empty if inline hardware encryption is available.
 */
/*************************************************************************************************/
bool_t PalCryptoAesCcmEncrypt(PalCryptoEnc_t *pEnc, uint8_t *pHdr, uint8_t *pBuf, uint8_t *pMic)
{
    PAL_CRYPTO_PARAM_CHECK_RET(pEnc && pBuf, FALSE);

    if (!pEnc->enaEncrypt) {
        return FALSE;
    }

    PAL_CRYPTO_PARAM_CHECK_RET(pEnc->pEncryptCtx, FALSE); /* Cipher blocks must be initialized */
    palCryptoCipherBlk_t *pCb = pEnc->pEncryptCtx;

    PAL_CRYPTO_PARAM_CHECK_RET(pHdr[BB_DATA_PDU_LEN_OFFSET] != 0,
                               FALSE); /* Zero length LE-C or LE-U is not possible */

    const uint16_t pldLen = pHdr[BB_DATA_PDU_LEN_OFFSET];

    if (pEnc->enaAuth) {
        pHdr[BB_DATA_PDU_LEN_OFFSET] +=
            PAL_CRYPTO_LL_DATA_MIC_LEN; /* Add length of MIC to payload. */
    }

    if ((pEnc->nonceMode == PAL_BB_NONCE_MODE_EXT16_CNTR) && (pEnc->pEventCounter)) {
        palCryptoLoadPktCnt(pCb, *pEnc->pEventCounter + 1);
    }

    if ((pEnc->nonceMode == PAL_BB_NONCE_MODE_EXT64_CNTR) && (pEnc->pTxPktCounter)) {
        palCryptoLoadIsoPktCnt(pCb, *pEnc->pTxPktCounter);
    }

    palCryptoLoadEcbData(pEnc);

    if (pEnc->enaAuth) {
        palCryptoAuthPdu(pEnc->type, pCb, pMic, pHdr, pBuf, pldLen);
    }

    palCryptoPdu(pCb, pMic, pBuf, pldLen);

    if (pEnc->nonceMode == PAL_BB_NONCE_MODE_PKT_CNTR) {
        palCryptoIncPktCnt(pCb);
    }

    return pEnc->enaAuth;
}

/*************************************************************************************************/
/*!
 *  \brief  Execute AES CBC transformation on payload and return MIC evaluation status.
 *
 *  \param  pEnc        Encryption parameters.
 *  \param  pBuf       Packet data.
 *
 *  \return TRUE if authentication successful, FALSE otherwise.
 *
 *  This routine completes the transformation in a blocking manner. This routine modifies the
 *  length field of the PDU to account for the MIC.
 *
 *  \note   Leave this implementation empty if inline hardware encryption is available.
 */
/*************************************************************************************************/
bool_t PalCryptoAesCcmDecrypt(PalCryptoEnc_t *pEnc, uint8_t *pBuf)
{
    PAL_CRYPTO_PARAM_CHECK_RET(pEnc && pBuf, FALSE);

    if (!pEnc->enaDecrypt) {
        /* Always successful if not enabled. */
        return TRUE;
    }

    PAL_CRYPTO_PARAM_CHECK_RET(pEnc->pDecryptCtx, FALSE); /* Cipher blocks must be initialized */
    palCryptoCipherBlk_t *pCb = pEnc->pDecryptCtx;

    uint8_t actMic[PAL_CRYPTO_LL_DATA_MIC_LEN] = { 0 };
    uint8_t *pHdr = pBuf;
    uint16_t pldLen = pHdr[BB_DATA_PDU_LEN_OFFSET];
    pBuf += BB_DATA_PDU_HDR_LEN;

    if (pEnc->enaAuth) {
        if (pldLen <= PAL_CRYPTO_LL_DATA_MIC_LEN) {
            /* No decryption required with no payload. */
            return TRUE;
        }

        pldLen -= PAL_CRYPTO_LL_DATA_MIC_LEN;
        pHdr[BB_DATA_PDU_LEN_OFFSET] = pldLen; /* Remove length of MIC from payload. */
    }

    uint8_t *pMic = pBuf + pldLen;
    if ((pEnc->nonceMode == PAL_BB_NONCE_MODE_EXT16_CNTR) && (pEnc->pEventCounter)) {
        /* Synchronized event counter stored in packet headroom. */
        uint16_t eventCounter;
        uint8_t *pEvtCntr = pHdr - sizeof(eventCounter);
        BYTES_TO_UINT16(eventCounter, pEvtCntr);

        palCryptoLoadPktCnt(pCb, eventCounter);
    }

    if ((pEnc->nonceMode == PAL_BB_NONCE_MODE_EXT64_CNTR) && (pEnc->pRxPktCounter)) {
        palCryptoLoadIsoPktCnt(pCb, *pEnc->pRxPktCounter);
    }

    palCryptoLoadEcbData(pEnc);
    palCryptoPdu(pCb, pMic, pBuf, pldLen);

    if (pEnc->enaAuth) {
        palCryptoAuthPdu(pEnc->type, pCb, actMic, pHdr, pBuf, pldLen);
    }

    if (pEnc->nonceMode == PAL_BB_NONCE_MODE_PKT_CNTR) {
        palCryptoIncPktCnt(pCb);
    }

    /* Verify MIC. */
    if (pEnc->enaAuth) {
        if ((actMic[0] != pMic[0]) || (actMic[1] != pMic[1]) || (actMic[2] != pMic[2]) ||
            (actMic[3] != pMic[3])) {
            return FALSE;
        }
    }

    return TRUE;
}

/*************************************************************************************************/
/*!
 *  \brief  Generate cryptographic grade random number.
 *
 *  \param  pBuf        Buffer to store random number.
 *  \param  len         Number of bytes.
 *
 *  \return None.
 */
/*************************************************************************************************/
void PalCryptoGenerateRandomNumber(uint8_t *pBuf, uint8_t len)
{
    while (len > 0) {
        uint32_t randNum;

        randNum = prandNum();
        if (len >= 4) {
            UINT32_TO_BSTREAM(pBuf, randNum);
            len -= 4;
        } else {
            while (len > 0) {
                UINT8_TO_BSTREAM(pBuf, randNum);
                len -= 1;
                randNum >>= 8;
            }
        }
    }
}
