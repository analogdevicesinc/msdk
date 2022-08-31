/*
 * max32631_crypto.h --
 *
 * ----------------------------------------------------------------------------
 * Copyright (c) 2016, Maxim Integrated Products
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MAXIM INTEGRATED PRODUCTS ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL MAXIM INTEGRATED PRODUCTS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* [INTERNAL] ------------------------------------------------------------------
 * Created on: 08-Jun-2012
 * Author:
 *
 * ---- Subversion keywords (need to set the keyword property)
 * $Revision:: 2996     $:  Revision of last commit
 * $Author:: pramod.hud#$:  Author of last commit
 * $Date:: 2012-06-08 1#$:  Date of last commit
 * [/INTERNAL] -------------------------------------------------------------- */

#if defined(__max32631)

#ifndef MAX32631_CRYPTO_H_
#define MAX32631_CRYPTO_H_

// MAA operand types
typedef enum { OPERAND_A, OPERAND_B, OPERAND_E, OPERAND_N, OPERAND_R } E_OPERANDS;

// MAA operations
typedef enum {
    A_POW_E_MODN,
    B_SQR_MODN,
    A_MULT_B_MODN,
    B_SQR_A_MODN,
    A_PLUS_B_MODN,
    A_MINUS_B_MODN,
    //        A_POW_E_MODN_OCALC,                     // with OCALC eset
    //        FAIL_TEST                               // FAILURE

} E_OPERATIONS;

#define MAX_INCOMING_DATA (256 + 3)

// MAX32631 CRYPTO CTL bits
// on MAX32631 MAA_CTRL is used for reset, starting a calculation and checking the calculation is
// completed

#define CRYPTO_MAA_DONE (1 << 5)
#define CRYPTO_DONE (unsigned int)(1 << 0)
#define CRYPTO_READY (1 << 30)
#define CRYPTO_BUSERR (1 << 29)

#define UCL_MAA_CLR_DONE (0xffffffdf)

#define UCL_MAA_BASE 0x40000000

// MAX32631 CRYPTO block address
#define PRNG_BASE 0x40007000
#define PRNG_RND_NUM (PRNG_BASE + 4)
#define AES_KEY_GEN 0x43

#define RESET_VALUE 0x00
#define UCL_MAA_REG_WORD_SIZE 32

// Crypto
// on max32631, there is no distinct crypto_ctl and maa_ctl: both are the same
#define CRYPTO_CTL (UCL_MAA_BASE + 0x7800)
#define CRYPTO_BASE 0x40000000

// MAX32631 MAA memory addresses
#define UCL_MAA_CTL (UCL_MAA_BASE + 0x7800) // MAA CTRL offset, equivalent to CRYPTO_CTL
#define UCL_MAA_MAWS (UCL_MAA_BASE + 0x7804) // MAWS
#define UCL_MAA_A (UCL_MAA_BASE + 0x0102800) // MAA_A memory offset
#define UCL_MAA_B (UCL_MAA_BASE + 0x0102880) // MAA_B memory offset
#define UCL_MAA_R (UCL_MAA_BASE + 0x0102900) // MAA_R memory offset
#define UCL_MAA_T (UCL_MAA_BASE + 0x0102980) // MAA_T memory offset
#define UCL_MAA_E (UCL_MAA_BASE + 0x0102A00) // MAA_E memory offset
#define UCL_MAA_M (UCL_MAA_BASE + 0x0102A80) // MAA_M memory offset

#define UCL_MAA_SEGA 0
#define UCL_MAA_SEGB 2
#define UCL_MAA_SEGR 4
#define UCL_MAA_SEGT 6
#define UCL_MAA_SEGE 8

#define UCL_MAA_SHIFT_RMA 24
#define UCL_MAA_SHIFT_AMA 16
#define UCL_MAA_SHIFT_BMA 20
#define UCL_MAA_SHIFT_TMA 28

// MAX32631 AES memory address
#define AES_CTRL CRYPTO_BASE + 0x7400
#define AES_CTRL_START 1
#define AES_CTRL_BUSY_MASK 1
#define AES_CTRL_ENCRYPT 0
#define AES_CTRL_DECRYPT 2

#define AES_CTRL_AES_128 0
#define AES_CTRL_AES_192 (1 << 3)
#define AES_CTRL_AES_256 (2 << 3)

#define AES_DATA_IN_0 CRYPTO_BASE + 0x102000
#define AES_DATA_IN_1 CRYPTO_BASE + 0x102004
#define AES_DATA_IN_2 CRYPTO_BASE + 0x102008
#define AES_DATA_IN_3 CRYPTO_BASE + 0x10200C

#define AES_KEY_0 CRYPTO_BASE + 0x102010
#define AES_KEY_1 CRYPTO_BASE + 0x102014
#define AES_KEY_2 CRYPTO_BASE + 0x102018
#define AES_KEY_3 CRYPTO_BASE + 0x10201C
#define AES_KEY_4 CRYPTO_BASE + 0x102020
#define AES_KEY_5 CRYPTO_BASE + 0x102024
#define AES_KEY_6 CRYPTO_BASE + 0x102028
#define AES_KEY_7 CRYPTO_BASE + 0x10202C

#define AES_DATA_OUT_0 CRYPTO_BASE + 0x102030
#define AES_DATA_OUT_1 CRYPTO_BASE + 0x102034
#define AES_DATA_OUT_2 CRYPTO_BASE + 0x102038
#define AES_DATA_OUT_3 CRYPTO_BASE + 0x10203C

#define MXC_BASE_AES ((uint32_t)0x40007400UL)
#define MXC_AES ((mxc_aes_regs_t*)MXC_BASE_AES)
#define MXC_BASE_AES_MEM ((uint32_t)0x40102000UL)
#define MXC_AES_MEM ((mxc_aes_mem_regs_t*)MXC_BASE_AES_MEM)

#endif /* MAX32631_CRYPTO_H_ */
#endif //_max32631
