/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
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
*/

#include <emv_l1_stack/iso14443_3_common.h>
#include <emv_l1_stack/iso14443_4_transitive.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <mml_nfc_pcd_rf_driver.h>
#include "logging.h"

#define RSEQNUM(pcb) (pcb & 1)
#define ISEQNUM(pcb) (pcb & 1)

#define ISRNAK(pcb) (pcb & 0x10) /*is R(NAK) or not*/

#define CHAINING(pcb) ((pcb >> 4) & 1) // Retrieves More data bit info from pcb

#define MAXRETRY_SENDBLOCK 3
#define MAXRESENDIBLOCK 2

#define BLOCKTYPE(p) (p & 0xC2)
#define BLOCK_I (0x02)
#define BLOCK_R (0x82)
#define BLOCK_S (0xC2)

#define PCBS 0xF2
#define WTXM_MAX_VALUE (59)

#define ACK 0
#define NAK 1
#define PCB_I(b5, b1) (0x2 | b1 | (b5 << 4))
#define PCB_R(nak, b1) (0xA2 | b1 | (nak << 4))
#define PCB_S(b56) (0xC0 | (b56 << 4))

typedef enum { False = 0, True } Bool;

#define PCBLEN 1
#define CRCLEN 2

const uint16_t FSCTable[9] = { 16, 24, 32, 40, 48, 64, 96, 128, 256 };
const uint16_t *FSDTable = FSCTable;

static uint8_t gPCDSeqNum = 0;

/**
 * @brief Reset block number
 *
 * Resets the block number for block transmission to 0
 */
void seqnuminit(void)
{
    gPCDSeqNum = 0;
}

static int32_t nfc_pcd_transceive_withpcb(uint8_t SPCB, uint8_t *RPCB, uint8_t protocol,
                                          uint8_t frametype, uint8_t *tx_buf, int32_t tx_len,
                                          uint8_t *rx_buf, int32_t *rx_len, uint32_t timeout)
{
    int32_t ret, slen = 0;
    uint8_t sbuf[MAX_BUFFER_LEN], rbuf[MAX_BUFFER_LEN];
    uint32_t rlen = MAX_BUFFER_LEN;

    sbuf[slen++] = SPCB;
    if (tx_len > 0) {
        memcpy(sbuf + 1, tx_buf, tx_len);
        slen += tx_len;
    }

    debug("trans TX: ");
    hexdump(DBG_LVL_DBG, sbuf, slen, 1);

    ret = nfc_pcd_transceive(protocol, frametype, sbuf, slen, rbuf, &rlen, timeout);

    if (ret != ISO14443_3_ERR_SUCCESS) {
        return ret;
    }

    full_debug("trans RX: ");
    hexdump(DBG_LVL_FDB, rbuf, (rlen + CRCLEN), 0);

    /*4.7.3.1 (protocol error) if it receives a frame with more than FSD data bytes,2 bytes crc not include of rx_buf*/
    if (rlen > (uint32_t)(FSDTable[FSDI_DEFAULT_VALUE] - CRCLEN))
        return ISO14443_3_ERR_PROTOCOL;

    if (rlen == 0)
        return ISO14443_3_ERR_PROTOCOL;

    *RPCB = rbuf[0];
    memcpy(rx_buf, (rbuf + 1), rlen - 1);
    *rx_len = rlen - 1;

    return ISO14443_3_ERR_SUCCESS;
}

/****************************************************************


  ________________Send______________________
  012345 789ABCD EF
  -------------------
  |PCB |   INF     |EDC |
  -------------------
  | 12  | 012345 |xxxx |
  -------------------
  I(0) |                    ^
  |                     |
  v                     |R(ACK)
  ----------
  |PCB |EDC|
  ----------

 ****************************************************************/
/****************************************************************
  Transmit and Receive APDU to and from a PICC (Card).

Pres:
capdu(out):
capdu_len(out):
rapdu(in):
rapdu_len(in):
Return:
Status of excute.
 ****************************************************************/
int32_t SendAPDU(uint8_t *capdu, int32_t capdu_len, uint8_t *rapdu, int32_t *rapdu_len)
{
    int32_t sendiblocklen; /*send length of one I-Block*/

    uint8_t readPCB; /*one byte PCB from PICC*/
    int32_t readLen; /*read length of bytes from PICC*/
    int8_t retry; /*for all type of the blocks*/
    int8_t iblockresend; /*i block resend times*/

    uint8_t rSINF; /*one byte INF of S-Block*/
    int8_t morechain; /*more chain for I-Block*/

    uint32_t fwt, fwttmp; /*frame wait time*/
    uint8_t WTXM = 0; /*10.2.2 1~59*/
    uint8_t rbuf[MAX_BUFFER_LEN]; /*receive buffer*/
    uint16_t fsc; /*Table 5.17,get from FSCI  16~256(FSCI 0~8)bytes*/
    int32_t ret = 0;

    Bool ApduDone = False; /*apdu done.*/
    ATSConfig_t ATS; /*answer to select*/
    Bool getSblock = False;

    get_ats(&ATS);

    fwt = (4096 * (1 << ATS.FWI) + ISO14443_FWT_DELTA) + 1 + 54;
    fsc = (FSCTable[ATS.FSCI] - PCBLEN - CRCLEN);

    *rapdu_len = 0;

    /*start to send command.*/
    while (capdu_len) {
        sendiblocklen = (capdu_len < fsc) ? capdu_len : fsc;
        capdu_len -= sendiblocklen;
        morechain = capdu_len ? 1 : 0;

        //10.3.4.1 The first block shall be sent by the PCD.
        ret = nfc_pcd_transceive_withpcb(PCB_I(morechain, gPCDSeqNum), &readPCB, ATS.Pro_Type,
                                         FT_STANDARD_CRC_EMD, capdu, sendiblocklen, rbuf, &readLen,
                                         fwt);

        retry = MAXRETRY_SENDBLOCK;
        iblockresend = 0;
        while (retry--) {
            /*10.3.2.1 When an I-block indicating chaining is received, the block shall be acknowledged by an R(ACK) block.*/
            if (ret == ISO14443_3_ERR_SUCCESS) {
                switch (BLOCKTYPE(readPCB)) {
                    /*I Block*/
                case BLOCK_I:
                    if ((readPCB & 0x2E) != 0x02) {
                        error("Bad I block: 10.3.2.1\n");
                        ret = ISO14443_3_ERR_PROTOCOL;
                        break;
                    }

                    /* already send all cmd,start get response from here*/
                    if (((ISEQNUM(readPCB) == gPCDSeqNum) && (!morechain))) {
                        /*10.3.3.3 */

                        break;
                    }
                    error("10.3.2.1 readPCB %x %d\n", ISEQNUM(readPCB), readPCB);
                    ret = ISO14443_3_ERR_PROTOCOL;
                    break;

                    /*R Block*/
                case BLOCK_R:
                    if (ISRNAK(readPCB)) {
                        /*10.3.4.6 reject R(NAK)*/
                        error("Get RNAK 10.3.4.6\n");
                        ret = ISO14443_3_ERR_PROTOCOL;
                        break;
                    }

                    /*update B6 b4 B3must be*/
                    if (!(readPCB & 0x20) || (readPCB & 0x08) || (readPCB & 0x04) ||
                        !(readPCB & 0x02)) {
                        error("ERR RACK 10.3.4.6\n");
                        ret = ISO14443_3_ERR_PROTOCOL;
                        break;
                    }

                    if (RSEQNUM(readPCB) == gPCDSeqNum) {
                        if (morechain) {
                            /*10.3.3.3 last block is chaining block 10.3.4.5 */
                            gPCDSeqNum ^= 1;

                            break;
                        } else {
                            /*10.3.4.5 if last block is not chaining block,protocol error*/
                            error("Last block not a chaining block\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }
                    } else {
                        /*10.3.4.3  received block number is different.*/
                        ret = ISO14443_3_ERR_PROTOCOL;
                        /*PCD shall re-transmit the last I-block if this R(ACK) block is received in response to an R(NAK) block sent by the PCD to notify a time-out.*/

                        if (iblockresend > MAXRESENDIBLOCK) {
                            /*10.3.4.4 max resend 2 time.*/
                            while (1) {}
                            error("IBlock resend max 2 times %d\n", retry);
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        } else {
                            warning("IBlock resend R(ACK) #%d\n", iblockresend);
                            /*resend last i block.*/
                            iblockresend++;
                            retry++;
                            ret = nfc_pcd_transceive_withpcb(PCB_I(morechain, gPCDSeqNum), &readPCB,
                                                             ATS.Pro_Type, FT_STANDARD_CRC_EMD,
                                                             capdu, sendiblocklen, rbuf, &readLen,
                                                             fwt);
                            continue;
                        }
                    }
                    break;

                    /*S Block*/
                case BLOCK_S:

                    /*10.3.4.2*/
                    do {
                        if (readPCB == PCBS) {
                            rSINF = *rbuf;

                            if (!rSINF || (rSINF > WTXM_MAX_VALUE)) {
                                error("WTX 00 or more than 59\n");
                                ret = ISO14443_3_ERR_PROTOCOL;
                                break;
                            }

                            //10.2.2.1
                            WTXM = (rSINF < WTXM_MAX_VALUE) ? rSINF : WTXM_MAX_VALUE;
                            /*wtx=fwt*wtxm+deltafwt*/
                            fwttmp = ((fwt - ISO14443_FWT_DELTA) * WTXM + ISO14443_FWT_DELTA) <=
                                             ISO14443_FWT_MAX ?
                                         ((fwt - ISO14443_FWT_DELTA) * WTXM + ISO14443_FWT_DELTA) :
                                         ISO14443_FWT_MAX;

                            //send back same 1 byte INF  to comfirm picc.
                            ret = nfc_pcd_transceive_withpcb(readPCB, &readPCB, ATS.Pro_Type,
                                                             FT_STANDARD_CRC_EMD, &rSINF, 1, rbuf,
                                                             &readLen, fwttmp);

                            if (ret != ISO14443_3_ERR_SUCCESS)
                                break;

                            getSblock = True;

                            //check if is next s block.
                            if (BLOCKTYPE(readPCB) != BLOCK_S)
                                break;

                        } else {
                            //maybe DESELECT something,DO NOT support.
                            error("ERR SBLOCK: 10.3.4.2\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }
                    } while (1);

                    if (getSblock) {
                        full_debug("getout from s\n");
                        retry += 1;
                        continue;
                    }
                    break;
                default:
                    // its a protocol error block
                    error("Unknown block type.\n");
                    ret = ISO14443_3_ERR_PROTOCOL;
                    break;
                }
            }

            //no need retry and error handle.
            if (ret == ISO14443_3_ERR_SUCCESS)
                break;

            /*power of pcd for protocol error.*/
            if (ret == ISO14443_3_ERR_PROTOCOL) {
                //9.6.12
                error("Protocol Error: 9.6.12\n");
                return ISO14443_3_ERR_PROTOCOL;
            }

            /* Return to application if they aborted this transaction */
            if (ret == ISO14443_3_ERR_ABORTED) {
                info("Transaction Aborted\n");
                return ret;
            }

            debug("iblock tx retry: %d, iblockresend: %d\n", retry, iblockresend);

            if ((retry) || (iblockresend == MAXRESENDIBLOCK)) {
                if (ret == ISO14443_3_ERR_TIMEOUT) {
                    warning("ISO14443_3_ERR_TIMEOUT\n");
                } else if (ret == ISO14443_3_ERR_TRANSMISSION) {
                    warning("ISO14443_3_ERR_TRANSMISSION\n");
                } else if (ret == ISO14443_3_ERR_COLLISION) {
                    warning("ISO14443_3_ERR_COLLISION\n");
                } else if (ret == ISO14443_3_ERR_EARLY_RESPONSE) {
                    warning("ISO14443_3_ERR_EARLY_RESPONSE\n");
                } else if (ret == ISO14443_3_ERR_PROTOCOL) {
                    /*9.6.1.2 return to polling*/
                    error("Protocol Error: 9.6.1.2\n");
                    return ISO14443_3_ERR_PROTOCOL;
                } else {
                    error("ISO14443_3_ERR_OTHER unexpected in retry check\n");
                    return ISO14443_3_ERR_OTHER;
                }

                //send nak for timeout
                debug("RNAK %d\n", retry);
                /*10.3.5.3 send RNAK to picc*/
                ret = nfc_pcd_transceive_withpcb(PCB_R(NAK, gPCDSeqNum), &readPCB, ATS.Pro_Type,
                                                 FT_STANDARD_CRC_EMD, NULL, 0, rbuf, &readLen, fwt);
            }
        }

        /* MAX 3 time for one same block*/
        if (retry < 0) {
            error("retried 3 times\n");
            if (ret == ISO14443_3_ERR_TIMEOUT) {
                return ISO14443_3_ERR_TIMEOUT;
            }
            return ISO14443_3_ERR_PROTOCOL;
        }
        capdu += sendiblocklen;
    }

    /*start to receive response.*/
    while (1) {
        if (readLen > 0) {
            memcpy(rapdu + *rapdu_len, rbuf, readLen);
            *rapdu_len += readLen;
            debug("*rapdu_len: %d\n", *rapdu_len);
        }

        /*10.3.3.3 */
        gPCDSeqNum ^= 1;

        // Check whether there are morechain chained APDU responses blocks
        if (CHAINING(readPCB)) {
            //10.3.2.1 When an I-block indicating chaining is received, the block shall be acknowledged by an R(ACK) block.
            ret = nfc_pcd_transceive_withpcb(PCB_R(ACK, gPCDSeqNum), &readPCB, ATS.Pro_Type,
                                             FT_STANDARD_CRC_EMD, NULL, 0, rbuf, &readLen, fwt);

            /*you have 3 chances to retry one block. */
            retry = MAXRETRY_SENDBLOCK;
            ApduDone = False;
            while (retry && !ApduDone) {
                /*receive next chaining i block.*/
                if (ret == ISO14443_3_ERR_SUCCESS) {
                    switch (BLOCKTYPE(readPCB)) {
                    case BLOCK_I:
                        /*update RFU to must be*/
                        if ((readPCB & 0x20) || (readPCB & 0x08) || (readPCB & 0x04) ||
                            !(readPCB & 0x02)) {
                            error("ERR IBLOCK 10.3.2.1\n");
                            return ISO14443_3_ERR_PROTOCOL;
                        }

                        if (ISEQNUM(readPCB) == gPCDSeqNum) {
                            /* A valid I-Block,break  retry loop */
                            ApduDone = True;
                        } else {
                            /*error block num*/
                            ret = ISO14443_3_ERR_PROTOCOL;
                        }

                        break;
                    case BLOCK_R:
                        if (ISRNAK(readPCB)) {
                            /*10.3.4.6 reject R(NAK)*/
                            error("R(NAK) reject: 10.3.4.6\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }

                        /*update B6 b4 B3must be*/
                        if (!(readPCB & 0x20) || (readPCB & 0x08) || (readPCB & 0x04) ||
                            !(readPCB & 0x02)) {
                            error("ERR RACK bad PCB bits: 10.3.4.6\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }
                        /*do not retranmit because R block only has 3 bytes,less than 4 bytes.*/
                        if (RSEQNUM(readPCB) == gPCDSeqNum) {
                            /*10.3.4.5*/
                            ret = ISO14443_3_ERR_PROTOCOL;
                        } else {
                            /*10.3.4.3  need more test*/
                            ret = ISO14443_3_ERR_PROTOCOL;
                        }
                        break;
                    case BLOCK_S:
                        if (readPCB != PCBS) {
                            error("ERR SBLOCK not allowed during chaining\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }

                        /*10.3.4.2 get wtxm*/
                        rSINF = *rbuf;
                        if (!rSINF || (rSINF > WTXM_MAX_VALUE)) {
                            error("WTX 00 or more than 59 during chaining\n");
                            ret = ISO14443_3_ERR_PROTOCOL;
                            break;
                        }

                        /*10.2.2.1  10.3.5.8*/
                        WTXM = (rSINF < WTXM_MAX_VALUE) ? rSINF : WTXM_MAX_VALUE;

                        /*wtx=fwt*wtxm+deltafwt*/
                        fwttmp = ((fwt - ISO14443_FWT_DELTA) * WTXM + ISO14443_FWT_DELTA) <=
                                         ISO14443_FWT_MAX ?
                                     ((fwt - ISO14443_FWT_DELTA) * WTXM + ISO14443_FWT_DELTA) :
                                     ISO14443_FWT_MAX;

                        /*send back same 1 byte INF  to comfirm picc.*/
                        ret = nfc_pcd_transceive_withpcb(readPCB, &readPCB, ATS.Pro_Type,
                                                         FT_STANDARD_CRC_EMD, &rSINF, 1, rbuf,
                                                         &readLen, fwttmp);

                        break;
                    default:
                        // its a protocol error block
                        error("Bad Block Type during chaining\n");
                        ret = ISO14443_3_ERR_PROTOCOL;
                        break;
                    }
                } else if ((ret == ISO14443_3_ERR_TIMEOUT) ||
                           (ret == ISO14443_3_ERR_TRANSMISSION) ||
                           (ret == ISO14443_3_ERR_COLLISION) ||
                           (ret == ISO14443_3_ERR_EARLY_RESPONSE)) {
                    /*10.3.5.8  ReSend last RACK */
                    /*10.3.5.6 this transmission error block*/

                    if (--retry) {
                        /*send RACK block*/
                        ret = nfc_pcd_transceive_withpcb(PCB_R(ACK, gPCDSeqNum), &readPCB,
                                                         ATS.Pro_Type, FT_STANDARD_CRC_EMD, NULL, 0,
                                                         rbuf, &readLen, fwt);
                    }
                } else {
                    error("protocol or other error during chaining\n");
                    return ret;
                }
            }

            /*10.3.5.6 max resend RACK twice. total 3 time.*/
            if (!retry) {
                if (ret == ISO14443_3_ERR_TIMEOUT) {
                    return ISO14443_3_ERR_TIMEOUT;
                }
                return ISO14443_3_ERR_PROTOCOL;
            }
        } else {
            /*10.3.5.3 response i block  is no-link block   Done!*/
            info("apdu done!\n");
            break;
        }
    }
    return ret;
}
