/*******************************************************************************
* Copyright (C) 2017 Maxim Integrated Products, Inc., All rights Reserved.
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

#ifndef _MXC_NFC_REGS_H_
#define _MXC_NFC_REGS_H_

#ifdef __cplusplus
extern "C" {
#endif

// This version to be used on ME09

#include <stdint.h>

/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif


/*
   Typedefed structure(s) for module registers (per instance or section) with direct 8-bit
   access to each register in module.
*/

/*                                                          Offset          Register Description
                                                            =============   ============================================================================ */
typedef struct {
    __IO uint32_t  softrstn;                             /*  0x0000          Software Reset Register                                                      */
    __IO uint32_t  dpp_cntl1;                            /*  0x0001          DPP Control Register 1                                                       */
    __IO uint32_t  dpp_cntl2;                            /*  0x0002          DPP Control Register 2                                                       */
    __IO uint32_t  dpp_cntl3;                            /*  0x0003          DPP Control Register 3                                                       */
    __IO uint32_t  dpp_capinit;                          /*  0x0004          DPP Initial Capture State Register                                           */
    __IO uint32_t  lc_tx_cntl0;                          /*  0x0005          LC TX Control Register 0                                                     */
    __R  uint32_t  rsv006;                               /*  0x0006                                                                                       */
    __IO uint32_t  lc_soflen;                            /*  0x0007          LC Type A Start of Frame Length Register                                     */
    __IO uint32_t  lc_cmd;                               /*  0x0008          TBD                                                                          */
    __IO uint32_t  lc_txrx_cfg;                          /*  0x0009          TBD                                                                          */
    __IO uint32_t  lc_typeb_cfg1;                        /*  0x000A          TBD                                                                          */
    __IO uint32_t  lc_typeb_cfg2;                        /*  0x000B          TBD                                                                          */
    __IO uint32_t  lc_rxb_mintr2;                        /*  0x000C          TBD                                                                          */
    __IO uint32_t  lc_rxb_maxtr2;                        /*  0x000D          TBD                                                                          */
    __R  uint32_t  rsv00E[2];                            /*  0x000E-0x000F                                                                                */
    __IO uint32_t  lc_status;                            /*  0x0010          TBD                                                                          */
    __IO uint32_t  lc_rxtx_status;                       /*  0x0011          TBD                                                                          */
    __IO uint32_t  lc_fifo_lvl;                          /*  0x0012          TBD                                                                          */
    __IO uint32_t  lc_fifo_af;                           /*  0x0013          TBD                                                                          */
    __IO uint32_t  lc_fifo_ae;                           /*  0x0014          TBD                                                                          */
    __IO uint32_t  lc_timer_init;                        /*  0x0015          TBD                                                                          */
    __IO uint32_t  lc_timer_reload;                      /*  0x0016          TBD                                                                          */
    __IO uint32_t  lc_timer_cntl;                        /*  0x0017          TBD                                                                          */
    __R  uint32_t  lc_irq_status;                        /*  0x0018          TBD                                                                          */
    __IO uint32_t  lc_irq_mask;                          /*  0x0019          TBD                                                                          */
    __IO uint32_t  lc_irq_clr;                           /*  0x001A          TBD                                                                          */
    __IO uint32_t  lc_tx_millerpw;                       /*  0x001B          TBD                                                                          */
    __IO uint32_t  lc_timer_prescaler;                   /*  0x001C          TBD                                                                          */
    __IO uint32_t  reserved1;                            /*  0x001D          DPP TBUS and new ME13 TM config                                              */
    __IO uint32_t  lc_fifo_wdata;                        /*  0x001E          TBD                                                                          */
    __IO uint32_t  lc_fifo_rdata;                        /*  0x001F          TBD                                                                          */
    __IO uint32_t  lc_typeb_eof_cfg;                     /*  0x0020          TBD                                                                          */
    __IO uint32_t  lc_timer_val;                         /*  0x0021          LC Timer Current Value Register                                              */
    __IO uint32_t  lc_fifo_status;                       /*  0x0022          LC FIFO Status Register                                                      */
    __IO uint32_t  lc_rx_cfg;                            /*  0x0023          LC RX Configuration Register                                                 */
    __IO uint32_t  lc_rx_cfg2;                           /*  0x0023          LC RX Configuration Register 2                                               */
    __IO uint32_t  clkcfg;                               /*  0x0025          Clock Configuration Register                                                 */
    __IO uint32_t  lc_rx_cfg3;                           /*  0x0026          LC RX Configuration Register 3                                               */
    __IO uint32_t  afe_tx_cfg10;                         /*  0x0027          AFE TX Configuration Register 10                                             */
    __IO uint32_t  afe_tx_cfg8;                          /*  0x0028          AFE TX Configuration Register 8                                              */
    __IO uint32_t  afe_tx_cfg9;                          /*  0x0029          AFE TX Configuration Register 9                                              */
    __IO uint32_t  afe_tx_cfg1;                          /*  0x002A          AFE TX Configuration Register 1                                              */
    __IO uint32_t  afe_tx_cfg2;                          /*  0x002B          AFE TX Configuration Register 2                                              */
    __IO uint32_t  afe_rx_cfg1;                          /*  0x002C          AFE RX Configuration Register 1                                              */
    __IO uint32_t  afe_rx_cfg2;                          /*  0x002D          AFE RX Configuration Register 2                                              */
    __IO uint32_t  afe_rx_time_refh;                     /*  0x002E          AFE RX Time Reference High Register 0                                        */
    __IO uint32_t  afe_rx_time_refl;                     /*  0x002F          AFE RX Time Reference Low Register                                           */
    __IO uint32_t  afe_rx_dc_i;                          /*  0x0030          AFE RX DC I Offset Register                                                  */
    __IO uint32_t  afe_rx_dc_q;                          /*  0x0031          AFE RX DC Q Offset Register                                                  */
    __IO uint32_t  afe_rx_azcfdvselh;                    /*  0x0032          AFE RX AZC Field Detector VSEL High Register                                 */
    __IO uint32_t  afe_rx_azcfdvsell;                    /*  0x0033          AFE RX AZC Field Detector VSEL Low Register                                  */
    __IO uint32_t  afe_rx_azcattfsel;                    /*  0x0034          AFE RX AZC ATT FSEL INIT Register                                            */
    __IO uint32_t  afe_rx_azcstatus;                     /*  0x0035          AFE RX AZC Status Register                                                   */
    __IO uint32_t  offs_comp0;                           /*  0x0036          DC Offset Compensation 0 Register                                            */
    __IO uint32_t  offs_comp1;                           /*  0x0037          DC Offset Compensation 1 Register                                            */
    __IO uint32_t  offs_comp2;                           /*  0x0038          DC Offset Compensation 2 Register                                            */
    __IO uint32_t  afe_rx_cfg8;                          /*  0x0039          AFE RX Configuration Register 8                                              */
    __IO uint32_t  afe_rx_cfg9;                          /*  0x003A          AFE RX Configuration Register 9                                              */
    __IO uint32_t  afe_rx_cfg10;                         /*  0x003B          AFE RX Configuration Register 10                                             */
    __IO uint32_t  afe_tx_cfg3;                          /*  0x003C          AFE TX Configuration Register 3                                              */
    __IO uint32_t  afe_tx_cfg4;                          /*  0x003D          AFE TX Configuration Register 4                                              */
    __IO uint32_t  afe_tx_cfg5;                          /*  0x003E          AFE TX Configuration Register 5                                              */
    __IO uint32_t  afe_tx_cfg6;                          /*  0x003F          AFE TX Configuration Register 6                                              */
    __IO uint32_t  afe_rx_cfg3;                          /*  0x0040          AFE RX Configuration Register 3                                              */
    __IO uint32_t  afe_rx_cfg4;                          /*  0x0041          AFE RX Configuration Register 4                                              */
    __IO uint32_t  afe_rx_attcfg;                        /*  0x0042          AFE RX Attenuation Configuration Register                                    */
    __IO uint32_t  afe_rx_cfg5;                          /*  0x0043          AFE RX Configuration Register 5                                              */
    __IO uint32_t  afe_rx_cfg6;                          /*  0x0044          AFE RX Configuration Register 6                                              */
    __IO uint32_t  afe_cal_status;                       /*  0x0045          AFE Calibration Status Register                                              */
    __IO uint32_t  afe_misc_test;                        /*  0x0046          AFE Test Configuration Register                                              */
    __IO uint32_t  afe_rx_cfg7;                          /*  0x0047          AFE RX Configuration Register 7                                              */
    __IO uint32_t  matchpat0;                            /*  0x0048          IQ Decoder Match Pattern Register 0                                          */
    __IO uint32_t  matchpat1;                            /*  0x0049          IQ Decoder Match Pattern Register 1                                          */
    __IO uint32_t  matchpat2;                            /*  0x004A          IQ Decoder Match Pattern Register 2                                          */
    __IO uint32_t  matchpat3;                            /*  0x004B          IQ Decoder Match Pattern Register 3                                          */
    __IO uint32_t  matchpatm0;                           /*  0x004C          IQ Decoder Match Pattern Mask Register 0                                     */
    __IO uint32_t  matchpatm1;                           /*  0x004D          IQ Decoder Match Pattern Mask Register 1                                     */
    __IO uint32_t  matchpatm2;                           /*  0x004E          IQ Decoder Match Pattern Mask Register 2                                     */
    __IO uint32_t  matchpatm3;                           /*  0x004F          IQ Decoder Match Pattern Mask Register 3                                     */
    __IO uint32_t  afe_tx_cfg7;                          /*  0x0050          AFE TX Configuration Register 7                                              */
    __IO uint32_t  afe_tx_stat2;                         /*  0x0051          AFE TX Status Register 2                                                     */
    __IO uint32_t  afe_tx_dsel;                          /*  0x0052          AFE TX Driver Strength Selection Register                                    */
    __IO uint32_t  afe_rx_lsbsel;                        /*  0x0053          AFE RX LSB Selection Register                                                */
    __IO uint32_t  fircoef0;                             /*  0x0054          I/Q FIR Coefficients                                                         */
    __IO uint32_t  fircoef1;                             /*  0x0055          I/Q FIR Coefficients                                                         */
    __IO uint32_t  fircoef2;                             /*  0x0056          I/Q FIR Coefficients                                                         */
    __IO uint32_t  fircoef3;                             /*  0x0057          I/Q FIR Coefficients                                                         */
    __IO uint32_t  fircoef4;                             /*  0x0058          I/Q FIR Coefficients                                                         */
    __R  uint32_t  rsv059[13];                           /*  0x0059-0x0065                                                                                */
    __IO uint32_t  firskipms;                            /*  0x0066          FIR Filter Skip Mux Select Register                                          */
    __IO uint32_t  firsfactr;                            /*  0x0067          FIR Filter Scaling Factor Register                                           */
    __IO uint32_t  iqc_ibufsel;                          /*  0x0068          I-Buffer Selection Register                                                  */
    __IO uint32_t  iqc_qbufsel;                          /*  0x0069          Q-Buffer Selection Register                                                  */
    __IO uint32_t  iqc_math;                             /*  0x006A          I/Q Math Control Register                                                    */
    __IO uint32_t  iqc_iqsel;                            /*  0x006B          I/Q Selection Control Register                                               */
    __IO uint32_t  iqc_iqbufsel0;                        /*  0x006C          Combined IQ Buffer Selection Register 0                                      */
    __IO uint32_t  iqc_iqbufsel1;                        /*  0x006D          Combined IQ Buffer Selection Register 1                                      */
    __IO uint32_t  iqc_thrsel;                           /*  0x006E          Moving Average Length Selection Register                                     */
    __IO uint32_t  iqc_hys;                              /*  0x006F          Hysteresis Constant Register                                                 */
    __IO uint32_t  iqc_datasel;                          /*  0x0070          Decoder Data Selection Register                                              */
    __IO uint32_t  iqc_det_cfg;                          /*  0x0071                                                                                       */
    __IO uint32_t  misc_test_cfg1;                       /*  0x0072          Misc. Test Configuration Register 1                                          */
    __IO uint32_t  misc_test_cfg2;                       /*  0x0073          Misc. Test Configuration Register 2                                          */
    __IO uint32_t  iqc_minmax_iq;                        /*  0x0074                                                                                       */
    __R  uint32_t  rsv075;                               /*  0x0075                                                                                       */
    __IO uint32_t  lc_mansmp106;                         /*  0x0076          LC 106kbps Manchester Sampling Decode Threshold Register                     */
    __IO uint32_t  lc_mansmp212;                         /*  0x0077          LC 212kbps Manchester Sampling Decode Threshold Register                     */
    __IO uint32_t  lc_mansmp424;                         /*  0x0078          LC 424kbps Manchester Sampling Decode Threshold Register                     */
    __IO uint32_t  lc_nfc_cntl;                          /*  0x0079          LC NFC Control Register                                                      */
    __IO uint32_t  lc_nfc_txsosl;                        /*  0x007A          LC NFC Type F TX Start of Sequence Length Register                           */
    __IO uint32_t  lc_nfc_rxeof106;                      /*  0x007B          LC NFC 106kps RX End of Frame Count Cycles Register                          */
    __IO uint32_t  lc_nfc_rxeof212;                      /*  0x007C          LC NFC 212kps RX End of Frame Count Cycles Register                          */
    __IO uint32_t  lc_nfc_rxeof424;                      /*  0x007D          LC NFC 424kps RX End of Frame Count CycRegister                              */
    __IO uint32_t  lc_nfc_rxeof848;                      /*  0x007E          LC NFC 848kps RX End of Frame Count Cycles Register                          */
    __IO uint32_t  lc_rx_typbegt;                        /*  0x007F          LC Type B RX EGT Limit Register                                              */
    __IO uint32_t  lc_cntl1;                             /*  0x0080          LC Control Register 1                                                        */
    __IO uint32_t  dpp_cntl4;                            /*  0x0081          DPP Control Register 4                                                       */
    __IO uint32_t  dpp_cntl5;                            /*  0x0082          DPP Control Register 5                                                       */
    __IO uint32_t  lc_tasof1c128;                        /*  0x0083          LC Type A Card Emulation 106kbps Start of Frame Ones Count Register          */
    __IO uint32_t  lc_tasof1c64;                         /*  0x0084          LC Type A Card Emulation 212kbps Start of Frame Ones Count Register          */
    __IO uint32_t  lc_tasof1c32;                         /*  0x0085          LC Type A Card Emulation 424kbps Start of Frame Ones Count Register          */
    __IO uint32_t  lc_tasof1c16;                         /*  0x0086          LC Type A Card Emulation 848kbps Start of Frame Ones Count Register          */
    __IO uint32_t  lc_typev_cntl1;                       /*  0x0087          LC Type Vicinity Control Register 1                                          */
    __IO uint32_t  lc_typev_cntl2;                       /*  0x0088          LC Type Vicinity Control Register 2                                          */
    __IO uint32_t  lc_vrxumssl0;                         /*  0x0089          LC Type Vicinity RX Unmodulated Single Subcarrier Length Register 0          */
    __R  uint32_t  rsv08A;                               /*  0x008A                                                                                       */
    __IO uint32_t  lc_vrxumdsl0;                         /*  0x008B          LC Type Vicinity RX Unmodulated Double Subcarrier Length Register 0          */
    __R  uint32_t  rsv08C;                               /*  0x008C                                                                                       */
    __IO uint32_t  lc_vrxspcfd;                          /*  0x008D          LC Type Vicinity RX Subcarrier Frame Delimiters Pulse Cycles Register        */
    __IO uint32_t  lc_vrxsplfd0;                         /*  0x008E          LC Type Vicinity RX Subcarrier Pulse Frame Delimiters Length Register 0      */
    __R  uint32_t  rsv08F;                               /*  0x008F                                                                                       */
    __IO uint32_t  lc_vrxsplfdt;                         /*  0x0090          LC Type Vicinity RX Subcarrier Frame Delimiters Pulse Length Tolerance Register*/
    __IO uint32_t  lc_vrxssc;                            /*  0x0091          LC Type Vicinity RX Single Subcarrier Cycle Register                         */
    __IO uint32_t  lc_vrxdsc;                            /*  0x0092          LC Type Vicinity RX Double Subcarrier Cycle Register                         */
    __IO uint32_t  lc_vrxssct;                           /*  0x0093          LC Type Vicinity RX Single Subcarrier Cycle Tolerance Register               */
    __IO uint32_t  lc_vrxeofw;                           /*  0x0094          LC Type Vicinity RX EOF Warning Register                                     */
    __IO uint32_t  lc_vrxdshbs0;                         /*  0x0095          LC Type Vicinity RX Double Subcarrier Half Bit Subtraction Register 0        */
    __R  uint32_t  rsv096;                               /*  0x0096                                                                                       */
    __IO uint32_t  lc_vrxdshbl0;                         /*  0x0097          LC Type Vicinity RX Double Subcarrier Half Bit Length Register 0             */
    __R  uint32_t  rsv098;                               /*  0x0098                                                                                       */
    __IO uint32_t  lc_vtxpl0;                            /*  0x0099          LC Type Vicinity TX Pause Length Register 0                                  */
    __R  uint32_t  rsv09A;                               /*  0x009A                                                                                       */
    __IO uint32_t  lc_vtx38sofht0;                       /*  0x009B          LC Type Vicinity TX 3/8 SOF High Time Register 0                             */
    __R  uint32_t  rsv09C;                               /*  0x009C                                                                                       */
    __IO uint32_t  lc_vtx18soft0;                        /*  0x009D          LC Type Vicinity TX 1/8 SOF Length Register 0                                */
    __R  uint32_t  rsv09E;                               /*  0x009E                                                                                       */
    __IO uint32_t  lc_vtx28soft0;                        /*  0x009F          LC Type Vicinity TX 2/8 SOF Length Register 0                                */
    __R  uint32_t  rsv0A0;                               /*  0x00A0                                                                                       */
    __IO uint32_t  lc_vtx38soft0;                        /*  0x00A1          LC Type Vicinity TX 3/8 SOF Length Register 0                                */
    __R  uint32_t  rsv0A2;                               /*  0x00A2                                                                                       */
    __IO uint32_t  lc_vtx48soft0;                        /*  0x00A3          LC Type Vicinity TX 4/8 SOF Length Register 0                                */
    __R  uint32_t  rsv0A4;                               /*  0x00A4                                                                                       */
    __IO uint32_t  lc_vtx14ht0;                          /*  0x00A5          LC Type Vicinity TX 1/4 High Time Register 0                                 */
    __R  uint32_t  rsv0A6;                               /*  0x00A6                                                                                       */
    __IO uint32_t  lc_vtx18p0;                           /*  0x00A7          LC Type Vicinity TX 1/8 Point Register 0                                     */
    __R  uint32_t  rsv0A8;                               /*  0x00A8                                                                                       */
    __IO uint32_t  lc_vtx38p0;                           /*  0x00A9          LC Type Vicinity TX 3/8 Point Register 0                                     */
    __R  uint32_t  rsv0AA;                               /*  0x00AA                                                                                       */
    __IO uint32_t  lc_vtx48p0;                           /*  0x00AB          LC Type Vicinity TX 4/8 Point Register 0                                     */
    __R  uint32_t  rsv0AC;                               /*  0x00AC                                                                                       */
    __IO uint32_t  lc_vtx58p0;                           /*  0x00AD          LC Type Vicinity TX 5/8 Point Register 0                                     */
    __R  uint32_t  rsv0AE;                               /*  0x00AE                                                                                       */
    __IO uint32_t  lc_vtx68p0;                           /*  0x00AF          LC Type Vicinity TX 6/8 Point Register 0                                     */
    __R  uint32_t  rsv0B0;                               /*  0x00B0                                                                                       */
    __IO uint32_t  lc_vtx78p0;                           /*  0x00B1          LC Type Vicinity TX 7/8 Point Register 0                                     */
    __R  uint32_t  rsv0B2;                               /*  0x00B2                                                                                       */
    __IO uint32_t  pll_cfg1;                             /*  0x00B3                                                                                       */
    __IO uint32_t  pll_cfg2;                             /*  0x00B4                                                                                       */
    __IO uint32_t  pll_cfg3;                             /*  0x00B5                                                                                       */
    __IO uint32_t  pll_cfg4;                             /*  0x00B6                                                                                       */
    __IO uint32_t  pll_status;                           /*  0x00B7                                                                                       */
    __IO uint32_t  env_det_mon0;                         /*  0x00B8                                                                                       */
    __IO uint32_t  env_det_mon1;                         /*  0x00B9                                                                                       */
    __IO uint32_t  env_det_mon2;                         /*  0x00BA                                                                                       */
    __IO uint32_t  env_det_mon3;                         /*  0x00BB                                                                                       */
    __IO uint32_t  tx_mod_tm_pass;                       /*  0x00BC                                                                                       */
    __IO uint32_t  autocorr_config;                      /*  0x00BD                                                                                       */
    __IO uint32_t  autocorr_pattern;                     /*  0x00BE                                                                                       */
    __IO uint32_t  autocorr_mask;                        /*  0x00BF                                                                                       */
    __R  uint32_t  rsv0C0;                               /*  0x00C0                                                                                       */
    __IO uint32_t  lc_nfc_rxsosl;                        /*  0x00C1          LC NFC Type F RX SOS Length Register                                         */
    __IO uint32_t  lc_nfc_rxpw;                          /*  0x00C2          LC NFC Type F RX Pulse Width Register                                        */
    __IO uint32_t  lc_nfc_rxpwt;                         /*  0x00C3          LC NFC Type F RX Pulse Width Tolerance Register                              */
    __IO uint32_t  lc_nfc_rxeofl;                        /*  0x00C4          LC NFC Type F RX EOF Length Register                                         */
    __IO uint32_t  lc_rxacdmpt0;                         /*  0x00C5          LC Type A Card Emulation RX Miller Pulse Tolerance Register 0                */
    __R  uint32_t  rsv0C6;                               /*  0x00C6                                                                                       */
    __IO uint32_t  lc_vrxcdpw;                           /*  0x00C7          LC Type Vicinity Card Emulation RX Pulse Width Register                      */
    __IO uint32_t  dpp_cmpsel;                           /*  0x00C8          DPP Comparator Calibration Register                                          */
    __IO uint32_t  dpp_vicc_cntl0;                       /*  0x00C9          DPP Vicinity Card Emulation Control 0                                        */
    __R  uint32_t  rsv0CA[2];                            /*  0x00CA-0x0CB                                                                                 */
    __IO uint32_t  apc_min_low_cnt0;                     /*  0x00CC                                                                                       */
    __IO uint32_t  apc_min_low_cnt1;                     /*  0x00CD                                                                                       */
    __IO uint32_t  apc_min_hi_cnt0;                      /*  0x00CE                                                                                       */
    __IO uint32_t  apc_min_hi_cnt1;                      /*  0x00CF                                                                                       */
    __IO uint32_t  apc_fd_wait_time0;                    /*  0x00D0                                                                                       */
    __IO uint32_t  apc_fd_wait_time1;                    /*  0x00D1                                                                                       */
    __IO uint32_t  apc_fd_rf_loop;                       /*  0x00D2                                                                                       */
    __IO uint32_t  apc_fd_vsel_hi_lowz;                  /*  0x00D3                                                                                       */
    __IO uint32_t  apc_fd_vsel_min_lowz;                 /*  0x00D4                                                                                       */
    __IO uint32_t  apc_fd_vsel_min_hiz;                  /*  0x00D5                                                                                       */
    __IO uint32_t  apc_fd_vsel_lowz;                     /*  0x00D6                                                                                       */
    __IO uint32_t  fircombiq_config;                     /*  0x00D7                                                                                       */
    __IO uint32_t  fircombiqcoef0;                       /*  0x00D8                                                                                       */
    __IO uint32_t  fircombiqcoef1;                       /*  0x00D9                                                                                       */
    __IO uint32_t  fircombiqcoef2;                       /*  0x00DA                                                                                       */
    __IO uint32_t  fircombiqcoef3;                       /*  0x00DB                                                                                       */
    __IO uint32_t  fircombiqcoef4;                       /*  0x00DC                                                                                       */
    __R  uint32_t  rsv0DD;                               /*  0x00DD                                                                                       */
    __IO uint32_t  lc_errorhis0;                         /*  0x00DE          LC Error Code History Register 0                                             */
    __R  uint32_t  rsv0DF;                               /*  0x00DF                                                                                       */
    __IO uint32_t  lc_cdtxstgdly;                        /*  0x00E0          LC Type A Card Emulation TX Start Gate Delay Register                        */
    __IO uint32_t  hw_tmr_cntl;                          /*  0x00E1          Hardware Timer Control Register                                              */
    __IO uint32_t  hw_tmrpscal;                          /*  0x00E2          Hardware Timer Pre-Scalar Register                                           */
    __IO uint32_t  hw_tmrmatchval;                       /*  0x00E3          Hardware Timer Match Value (timeout)                                         */
    __R  uint32_t  rsv0E4[3];                            /*  0x00E4-0x00E6                                                                                */
    __IO uint32_t  hw_tmrval0;                           /*  0x00E7          Hardware Timer Current Value Register 0                                      */
    __R  uint32_t  rsv0E8[3];                            /*  0x00E8-0x00EA                                                                                */
    __IO uint32_t  hw_tmrrxstastp0;                      /*  0x00EB          Hardware Timer RX Start Timestamp Register 0                                 */
    __IO uint32_t  hw_tmrrxendstp0;                      /*  0x00EC          Hardware Timer RX End Timestamp Register 0                                   */
    __R  uint32_t  rsv0EA[2];                            /*  0x00ED-0x00EE                                                                                */
    __IO uint32_t  hw_tmrtxstastp0;                      /*  0x00EF          Hardware Timer TX Start Timestamp Register 0                                 */
    __IO uint32_t  hw_tmrtxendstp0;                      /*  0x00F0          Hardware Timer TX End Timestamp Register 0                                   */
    __R  uint32_t  rsv0F1[2];                            /*  0x00F1-0x00F2                                                                                */
    __IO uint32_t  lc_nfc_auto_det;                      /*  0x00F3          LC NFC Auto Detection Status Register                                        */
    __IO uint32_t  dpp_nfcrxadly;                        /*  0x00F4          Type A Card Emulation RX Delay Register                                      */
    __IO uint32_t  afe_rx_cfg11;                         /*  0x00F5                                                                                       */
    __IO uint32_t  sttm_clk_sel;                         /*  0x00F6                                                                                       */
    __IO uint32_t  sttm_clk_sel_f;                       /*  0x00F7                                                                                       */
    __IO uint32_t  stfm_clk_sel;                         /*  0x00F8                                                                                       */
    __IO uint32_t  stfm_clk_sel_f;                       /*  0x00F9                                                                                       */
    __IO uint32_t  sttm_driver_strength;                 /*  0x00FA                                                                                       */
    __IO uint32_t  stfm_driver_strength;                 /*  0x00FB                                                                                       */
    __IO uint32_t  sttm_firmware;                        /*  0x00FC                                                                                       */
    __IO uint32_t  reg_reset;                            /*  0x00FD          Reset Register                                                               */
    __IO uint32_t  stfm_firmware;                        /*  0x00FE                                                                                       */
    __IO uint32_t  tx_mod_mode_en;                       /*  0x00FF                                                                                       */
    __IO uint32_t  nfc_clk_src_sel;                      /*  0x0100                                                                                       */
    __IO uint32_t  nfc_clk_src_stat;                     /*  0x0101                                                                                       */
    __IO uint32_t  lc_rx_ai_config;                      /*  0x0102                                                                                       */
    __IO uint32_t  lc_rx_i_stat;                         /*  0x0103                                                                                       */
    __IO uint32_t  lc_rx_q_stat;                         /*  0x0104                                                                                       */
    __IO uint32_t  lc_rx_i_stat_bf;                      /*  0x0105                                                                                       */
    __IO uint32_t  lc_rx_q_stat_bf;                      /*  0x0106                                                                                       */
    __R  uint32_t  rsv107[9];                            /*  0x0107--0x010F                                                                               */
    __IO uint32_t  maxim_rx_phy_cfg1;                    /*  0x0110                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg2;                    /*  0x0111                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg3;                    /*  0x0112                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg4;                    /*  0x0113                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg5;                    /*  0x0114                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg6;                    /*  0x0115                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg7;                    /*  0x0116                                                                                       */
    __IO uint32_t  maxim_rx_phy_cfg8;                    /*  0x0117                                                                                       */
    __R  uint32_t  maxim_rx_phy_status1;                 /*  0x0118                                                                                       */
    __R  uint32_t  maxim_rx_phy_status2;                 /*  0x0119                                                                                       */
    __IO uint32_t  maxim_rx_lnk_cfg1;                    /*  0x011A                                                                                       */
    __IO uint32_t  maxim_rx_lnk_cfg2;                    /*  0x011B                                                                                       */
    __R  uint32_t  maxim_rx_lnk_status1;                 /*  0x011C                                                                                       */
    __IO uint32_t  maxim_tx_cfg1;                        /*  0x011D                                                                                       */
    __IO uint32_t  maxim_tx_cfg2;                        /*  0x011E                                                                                       */
    __IO uint32_t  maxim_tx_cfg3;                        /*  0x011F                                                                                       */
    __IO uint32_t  maxim_tx_cfg4;                        /*  0x0120                                                                                       */
    __IO uint32_t  maxim_tx_cfg5;                        /*  0x0121                                                                                       */
    __IO uint32_t  maxim_tx_cfg6;                        /*  0x0122                                                                                       */
    __R  uint32_t  maxim_tx_status1;                     /*  0x0123                                                                                       */
    __IO uint32_t  maxim_timer_cfg1;                     /*  0x0124                                                                                       */
    __IO uint32_t  maxim_timer_cfg2;                     /*  0x0125                                                                                       */
    __IO uint32_t  maxim_timer_cfg3;                     /*  0x0126                                                                                       */
    __IO uint32_t  maxim_timer_cfg4;                     /*  0x0127                                                                                       */
    __IO uint32_t  maxim_timer_cfg5;                     /*  0x0128                                                                                       */
    __IO uint32_t  maxim_timer_cfg6;                     /*  0x0129                                                                                       */
    __R  uint32_t  maxim_timer_status1;                  /*  0x012A                                                                                       */
    __R  uint32_t  maxim_timer_status2;                  /*  0x012B                                                                                       */
    __R  uint32_t  maxim_timer_status3;                  /*  0x012C                                                                                       */
    __R  uint32_t  maxim_timer_status4;                  /*  0x012D                                                                                       */
    __R  uint32_t  maxim_timer_status5;                  /*  0x012E                                                                                       */
    __R  uint32_t  maxim_timer_status6;                  /*  0x012F                                                                                       */
    __R  uint32_t  maxim_timer_status7;                  /*  0x0130                                                                                       */
    __R  uint32_t  maxim_timer_status8;                  /*  0x0131                                                                                       */
    __R  uint32_t  rsv132[14];                           /*  0x0132--0x013F                                                                               */
    __IO uint32_t  maxim_int_cfg1;                       /*  0x0140                                                                                       */
    __IO uint32_t  maxim_int_cfg2;                       /*  0x0141                                                                                       */
    __IO uint32_t  maxim_int_cfg3;                       /*  0x0142                                                                                       */
    __R  uint32_t  rsv143[13];                           /*  0x0143--0x014F                                                                               */
    __R  uint32_t  maxim_int_status1;                    /*  0x150                                                                                        */
    __R  uint32_t  maxim_int_status2;                    /*  0x151                                                                                       */
    __R  uint32_t  maxim_int_status3;                    /*  0x152                                                                                       */
} mxc_nfc_regs_t;


/*
   Register offsets for module NFC.
   FIXME these are not updated for ME09, but I don't think they are used anywhere
*/

#define MXC_R_NFC_OFFS_SOFTRSTN                             ((uint32_t)0x00000000UL)
#define MXC_R_NFC_OFFS_DPP_CNTL1                            ((uint32_t)0x00000001UL)
#define MXC_R_NFC_OFFS_DPP_CNTL2                            ((uint32_t)0x00000002UL)
#define MXC_R_NFC_OFFS_DPP_CNTL3                            ((uint32_t)0x00000003UL)
#define MXC_R_NFC_OFFS_DPP_CAPINIT                          ((uint32_t)0x00000004UL)
#define MXC_R_NFC_OFFS_LC_TX_CNTL0                          ((uint32_t)0x00000005UL)
#define MXC_R_NFC_OFFS_LC_TX_CNTL1                          ((uint32_t)0x00000006UL)
#define MXC_R_NFC_OFFS_LC_SOFLEN                            ((uint32_t)0x00000007UL)
#define MXC_R_NFC_OFFS_LC_CMD                               ((uint32_t)0x00000008UL)
#define MXC_R_NFC_OFFS_LC_TXRX_CFG                          ((uint32_t)0x00000009UL)
#define MXC_R_NFC_OFFS_LC_TYPEB_CFG1                        ((uint32_t)0x0000000AUL)
#define MXC_R_NFC_OFFS_LC_TYPEB_CFG2                        ((uint32_t)0x0000000BUL)
#define MXC_R_NFC_OFFS_LC_RXB_MINTR2                        ((uint32_t)0x0000000CUL)
#define MXC_R_NFC_OFFS_LC_RXB_MAXTR2                        ((uint32_t)0x0000000DUL)
#define MXC_R_NFC_OFFS_LC_STATUS                            ((uint32_t)0x00000010UL)
#define MXC_R_NFC_OFFS_LC_RXTX_STATUS                       ((uint32_t)0x00000011UL)
#define MXC_R_NFC_OFFS_LC_FIFO_LVL                          ((uint32_t)0x00000012UL)
#define MXC_R_NFC_OFFS_LC_FIFO_AF                           ((uint32_t)0x00000013UL)
#define MXC_R_NFC_OFFS_LC_FIFO_AE                           ((uint32_t)0x00000014UL)
#define MXC_R_NFC_OFFS_LC_TIMER_INIT                        ((uint32_t)0x00000015UL)
#define MXC_R_NFC_OFFS_LC_TIMER_RELOAD                      ((uint32_t)0x00000016UL)
#define MXC_R_NFC_OFFS_LC_TIMER_CNTL                        ((uint32_t)0x00000017UL)
#define MXC_R_NFC_OFFS_LC_IRQ_STATUS                        ((uint32_t)0x00000018UL)
#define MXC_R_NFC_OFFS_LC_IRQ_MASK                          ((uint32_t)0x00000019UL)
#define MXC_R_NFC_OFFS_LC_IRQ_CLR                           ((uint32_t)0x0000001AUL)
#define MXC_R_NFC_OFFS_LC_TX_MILLERPW                       ((uint32_t)0x0000001BUL)
#define MXC_R_NFC_OFFS_LC_TIMER_PRESCALER                   ((uint32_t)0x0000001CUL)
#define MXC_R_NFC_OFFS_LC_FIFO_WDATA                        ((uint32_t)0x0000001EUL)
#define MXC_R_NFC_OFFS_LC_FIFO_RDATA                        ((uint32_t)0x0000001FUL)
#define MXC_R_NFC_OFFS_LC_TIMER_VAL                         ((uint32_t)0x00000021UL)
#define MXC_R_NFC_OFFS_LC_FIFO_STATUS                       ((uint32_t)0x00000022UL)
#define MXC_R_NFC_OFFS_LC_RX_CFG                            ((uint32_t)0x00000023UL)
#define MXC_R_NFC_OFFS_CLKCFG                               ((uint32_t)0x00000025UL)
#define MXC_R_NFC_OFFS_AFE_TX_CFG1                          ((uint32_t)0x0000002AUL)
#define MXC_R_NFC_OFFS_AFE_TX_CFG2                          ((uint32_t)0x0000002BUL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG1                          ((uint32_t)0x0000002CUL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG2                          ((uint32_t)0x0000002DUL)
#define MXC_R_NFC_OFFS_AFE_RX_TIME_REFH                     ((uint32_t)0x0000002EUL)
#define MXC_R_NFC_OFFS_AFE_RX_TIME_REFL                     ((uint32_t)0x0000002FUL)
#define MXC_R_NFC_OFFS_AFE_RX_DC_I                          ((uint32_t)0x00000030UL)
#define MXC_R_NFC_OFFS_AFE_RX_DC_Q                          ((uint32_t)0x00000031UL)
#define MXC_R_NFC_OFFS_AFE_RX_AZCFDVSELH                    ((uint32_t)0x00000032UL)
#define MXC_R_NFC_OFFS_AFE_RX_AZCFDVSELL                    ((uint32_t)0x00000033UL)
#define MXC_R_NFC_OFFS_AFE_RX_AZCATTFSEL                    ((uint32_t)0x00000034UL)
#define MXC_R_NFC_OFFS_AFE_RX_AZCSTATUS                     ((uint32_t)0x00000035UL)
#define MXC_R_NFC_OFFS_OFFS_COMP0                           ((uint32_t)0x00000036UL)
#define MXC_R_NFC_OFFS_OFFS_COMP1                           ((uint32_t)0x00000037UL)
#define MXC_R_NFC_OFFS_OFFS_COMP2                           ((uint32_t)0x00000038UL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG3                          ((uint32_t)0x00000040UL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG4                          ((uint32_t)0x00000041UL)
#define MXC_R_NFC_OFFS_AFE_RX_ATTCFG                        ((uint32_t)0x00000042UL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG5                          ((uint32_t)0x00000043UL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG6                          ((uint32_t)0x00000044UL)
#define MXC_R_NFC_OFFS_AFE_CAL_STATUS                       ((uint32_t)0x00000045UL)
#define MXC_R_NFC_OFFS_AFE_MISC_TEST                        ((uint32_t)0x00000046UL)
#define MXC_R_NFC_OFFS_AFE_RX_CFG7                          ((uint32_t)0x00000047UL)
#define MXC_R_NFC_OFFS_MATCHPAT0                            ((uint32_t)0x00000048UL)
#define MXC_R_NFC_OFFS_MATCHPAT1                            ((uint32_t)0x00000049UL)
#define MXC_R_NFC_OFFS_MATCHPAT2                            ((uint32_t)0x0000004AUL)
#define MXC_R_NFC_OFFS_MATCHPAT3                            ((uint32_t)0x0000004BUL)
#define MXC_R_NFC_OFFS_MATCHPATM0                           ((uint32_t)0x0000004CUL)
#define MXC_R_NFC_OFFS_MATCHPATM1                           ((uint32_t)0x0000004DUL)
#define MXC_R_NFC_OFFS_MATCHPATM2                           ((uint32_t)0x0000004EUL)
#define MXC_R_NFC_OFFS_MATCHPATM3                           ((uint32_t)0x0000004FUL)
#define MXC_R_NFC_OFFS_AFE_TX_DSEL                          ((uint32_t)0x00000052UL)
#define MXC_R_NFC_OFFS_AFE_RX_LSBSEL                        ((uint32_t)0x00000053UL)
#define MXC_R_NFC_OFFS_FIRSKIPMS                            ((uint32_t)0x00000066UL)
#define MXC_R_NFC_OFFS_FIRSFACTR                            ((uint32_t)0x00000067UL)
#define MXC_R_NFC_OFFS_IQC_IBUFSEL                          ((uint32_t)0x00000068UL)
#define MXC_R_NFC_OFFS_IQC_QBUFSEL                          ((uint32_t)0x00000069UL)
#define MXC_R_NFC_OFFS_IQC_MATH                             ((uint32_t)0x0000006AUL)
#define MXC_R_NFC_OFFS_IQC_IQSEL                            ((uint32_t)0x0000006BUL)
#define MXC_R_NFC_OFFS_IQC_IQBUFSEL0                        ((uint32_t)0x0000006CUL)
#define MXC_R_NFC_OFFS_IQC_IQBUFSEL1                        ((uint32_t)0x0000006DUL)
#define MXC_R_NFC_OFFS_IQC_THRSEL                           ((uint32_t)0x0000006EUL)
#define MXC_R_NFC_OFFS_IQC_HYS                              ((uint32_t)0x0000006FUL)
#define MXC_R_NFC_OFFS_IQC_DATASEL                          ((uint32_t)0x00000070UL)
#define MXC_R_NFC_OFFS_IQC_DETCFG                           ((uint32_t)0x00000071UL)
#define MXC_R_NFC_OFFS_MISC_TEST_CFG1                       ((uint32_t)0x00000072UL)
#define MXC_R_NFC_OFFS_MISC_TEST_CFG2                       ((uint32_t)0x00000073UL)
#define MXC_R_NFC_OFFS_LC_MANSMP106                         ((uint32_t)0x00000076UL)
#define MXC_R_NFC_OFFS_LC_MANSMP212                         ((uint32_t)0x00000077UL)
#define MXC_R_NFC_OFFS_LC_MANSMP424                         ((uint32_t)0x00000078UL)
#define MXC_R_NFC_OFFS_LC_NFC_CNTL                          ((uint32_t)0x00000079UL)
#define MXC_R_NFC_OFFS_LC_NFC_TXSOSL                        ((uint32_t)0x0000007AUL)
#define MXC_R_NFC_OFFS_LC_NFC_RXEOF106                      ((uint32_t)0x0000007BUL)
#define MXC_R_NFC_OFFS_LC_NFC_RXEOF212                      ((uint32_t)0x0000007CUL)
#define MXC_R_NFC_OFFS_LC_NFC_RXEOF424                      ((uint32_t)0x0000007DUL)
#define MXC_R_NFC_OFFS_LC_NFC_RXEOF848                      ((uint32_t)0x0000007EUL)
#define MXC_R_NFC_OFFS_LC_RX_TYPBEGT                        ((uint32_t)0x0000007FUL)
#define MXC_R_NFC_OFFS_LC_CNTL1                             ((uint32_t)0x00000080UL)
#define MXC_R_NFC_OFFS_DPP_CNTL4                            ((uint32_t)0x00000081UL)
#define MXC_R_NFC_OFFS_DPP_CNTL5                            ((uint32_t)0x00000082UL)
#define MXC_R_NFC_OFFS_LC_TASOF1C128                        ((uint32_t)0x00000083UL)
#define MXC_R_NFC_OFFS_LC_TASOF1C64                         ((uint32_t)0x00000084UL)
#define MXC_R_NFC_OFFS_LC_TASOF1C32                         ((uint32_t)0x00000085UL)
#define MXC_R_NFC_OFFS_LC_TASOF1C16                         ((uint32_t)0x00000086UL)
#define MXC_R_NFC_OFFS_LC_TYPEV_CNTL1                       ((uint32_t)0x00000087UL)
#define MXC_R_NFC_OFFS_LC_TYPEV_CNTL2                       ((uint32_t)0x00000088UL)
#define MXC_R_NFC_OFFS_LC_VRXUMSSL0                         ((uint32_t)0x00000089UL)
#define MXC_R_NFC_OFFS_LC_VRXUMSSL1                         ((uint32_t)0x0000008AUL)
#define MXC_R_NFC_OFFS_LC_VRXUMDSL0                         ((uint32_t)0x0000008BUL)
#define MXC_R_NFC_OFFS_LC_VRXUMDSL1                         ((uint32_t)0x0000008CUL)
#define MXC_R_NFC_OFFS_LC_VRXSPCFD                          ((uint32_t)0x0000008DUL)
#define MXC_R_NFC_OFFS_LC_VRXSPLFD0                         ((uint32_t)0x0000008EUL)
#define MXC_R_NFC_OFFS_LC_VRXSPLFD1                         ((uint32_t)0x0000008FUL)
#define MXC_R_NFC_OFFS_LC_VRXSPLFDT                         ((uint32_t)0x00000090UL)
#define MXC_R_NFC_OFFS_LC_VRXSSC                            ((uint32_t)0x00000091UL)
#define MXC_R_NFC_OFFS_LC_VRXDSC                            ((uint32_t)0x00000092UL)
#define MXC_R_NFC_OFFS_LC_VRXSSCT                           ((uint32_t)0x00000093UL)
#define MXC_R_NFC_OFFS_LC_VRXEOFW                           ((uint32_t)0x00000094UL)
#define MXC_R_NFC_OFFS_LC_VRXDSHBS0                         ((uint32_t)0x00000095UL)
#define MXC_R_NFC_OFFS_LC_VRXDSHBS1                         ((uint32_t)0x00000096UL)
#define MXC_R_NFC_OFFS_LC_VRXDSHBL0                         ((uint32_t)0x00000097UL)
#define MXC_R_NFC_OFFS_LC_VRXDSHBL1                         ((uint32_t)0x00000098UL)
#define MXC_R_NFC_OFFS_LC_VTXPL0                            ((uint32_t)0x00000099UL)
#define MXC_R_NFC_OFFS_LC_VTXPL1                            ((uint32_t)0x0000009AUL)
#define MXC_R_NFC_OFFS_LC_VTX38SOFHT0                       ((uint32_t)0x0000009BUL)
#define MXC_R_NFC_OFFS_LC_VTX38SOFHT1                       ((uint32_t)0x0000009CUL)
#define MXC_R_NFC_OFFS_LC_VTX18SOFT0                        ((uint32_t)0x0000009DUL)
#define MXC_R_NFC_OFFS_LC_VTX18SOFT1                        ((uint32_t)0x0000009EUL)
#define MXC_R_NFC_OFFS_LC_VTX14HT0                          ((uint32_t)0x000000A5UL)
#define MXC_R_NFC_OFFS_LC_VTX14HT1                          ((uint32_t)0x000000A6UL)
#define MXC_R_NFC_OFFS_LC_VTX18P0                           ((uint32_t)0x000000A7UL)
#define MXC_R_NFC_OFFS_LC_VTX18P1                           ((uint32_t)0x000000A8UL)
#define MXC_R_NFC_OFFS_LC_VTX38P0                           ((uint32_t)0x000000A9UL)
#define MXC_R_NFC_OFFS_LC_VTX38P1                           ((uint32_t)0x000000AAUL)
#define MXC_R_NFC_OFFS_LC_VTX48P0                           ((uint32_t)0x000000ABUL)
#define MXC_R_NFC_OFFS_LC_VTX48P1                           ((uint32_t)0x000000ACUL)
#define MXC_R_NFC_OFFS_LC_VTX58P0                           ((uint32_t)0x000000ADUL)
#define MXC_R_NFC_OFFS_LC_VTX58P1                           ((uint32_t)0x000000AEUL)
#define MXC_R_NFC_OFFS_LC_VTX68P0                           ((uint32_t)0x000000AFUL)
#define MXC_R_NFC_OFFS_LC_VTX68P1                           ((uint32_t)0x000000B0UL)
#define MXC_R_NFC_OFFS_LC_VTX78P0                           ((uint32_t)0x000000B1UL)
#define MXC_R_NFC_OFFS_LC_VTX78P1                           ((uint32_t)0x000000B2UL)
#define MXC_R_NFC_OFFS_LC_NFC_RXSOSL                        ((uint32_t)0x000000C1UL)
#define MXC_R_NFC_OFFS_LC_NFC_RXPW                          ((uint32_t)0x000000C2UL)
#define MXC_R_NFC_OFFS_LC_NFC_RXPWT                         ((uint32_t)0x000000C3UL)
#define MXC_R_NFC_OFFS_LC_NFC_RXEOFL                        ((uint32_t)0x000000C4UL)
#define MXC_R_NFC_OFFS_LC_RXACDMPT0                         ((uint32_t)0x000000C5UL)
#define MXC_R_NFC_OFFS_LC_RXACDMPT1                         ((uint32_t)0x000000C6UL)
#define MXC_R_NFC_OFFS_LC_VRXCDPW                           ((uint32_t)0x000000C7UL)
#define MXC_R_NFC_OFFS_DPP_CMPSEL                           ((uint32_t)0x000000C8UL)
#define MXC_R_NFC_OFFS_DPP_VICC_CNTL0                       ((uint32_t)0x000000C9UL)
#define MXC_R_NFC_OFFS_DPP_VICC_CNTL1                       ((uint32_t)0x000000CAUL)
#define MXC_R_NFC_OFFS_DPP_VICC_CNTL2                       ((uint32_t)0x000000CBUL)
#define MXC_R_NFC_OFFS_LC_ERRORHIS0                         ((uint32_t)0x000000DEUL)
#define MXC_R_NFC_OFFS_LC_ERRORHIS1                         ((uint32_t)0x000000DFUL)
#define MXC_R_NFC_OFFS_LC_CDTXSTGDLY                        ((uint32_t)0x000000E0UL)
#define MXC_R_NFC_OFFS_HW_TMR_CNTL                          ((uint32_t)0x000000E1UL)
#define MXC_R_NFC_OFFS_HW_TMRPSCAL                          ((uint32_t)0x000000E2UL)
#define MXC_R_NFC_OFFS_HW_TMRVAL0                           ((uint32_t)0x000000E7UL)
#define MXC_R_NFC_OFFS_HW_TMRVAL1                           ((uint32_t)0x000000E8UL)
#define MXC_R_NFC_OFFS_HW_TMRVAL2                           ((uint32_t)0x000000E9UL)
#define MXC_R_NFC_OFFS_HW_TMRVAL3                           ((uint32_t)0x000000EAUL)
#define MXC_R_NFC_OFFS_HW_TMRRXSTP0                         ((uint32_t)0x000000EBUL)
#define MXC_R_NFC_OFFS_HW_TMRRXSTP1                         ((uint32_t)0x000000ECUL)
#define MXC_R_NFC_OFFS_HW_TMRRXSTP2                         ((uint32_t)0x000000EDUL)
#define MXC_R_NFC_OFFS_HW_TMRRXSTP3                         ((uint32_t)0x000000EEUL)
#define MXC_R_NFC_OFFS_HW_TMRTXSTP0                         ((uint32_t)0x000000EFUL)
#define MXC_R_NFC_OFFS_HW_TMRTXSTP1                         ((uint32_t)0x000000F0UL)
#define MXC_R_NFC_OFFS_HW_TMRTXSTP2                         ((uint32_t)0x000000F1UL)
#define MXC_R_NFC_OFFS_HW_TMRTXSTP3                         ((uint32_t)0x000000F2UL)
#define MXC_R_NFC_OFFS_LC_NFC_AUTO_DET                      ((uint32_t)0x000000F3UL)
#define MXC_R_NFC_OFFS_DPP_NFCRXADLY                        ((uint32_t)0x000000F4UL)
#define MXC_R_NFC_OFFS_REG_RESET                            ((uint32_t)0x000000FDUL)


/*
   Field positions and masks for module NFC.
*/

#define MXC_F_NFC_SOFTRSTN_DPPSRN_POS                       0
#define MXC_F_NFC_SOFTRSTN_DPPSRN                           ((uint32_t)(0x00000001UL << MXC_F_NFC_SOFTRSTN_DPPSRN_POS))
#define MXC_F_NFC_SOFTRSTN_LCSRN_POS                        1
#define MXC_F_NFC_SOFTRSTN_LCSRN                            ((uint32_t)(0x00000001UL << MXC_F_NFC_SOFTRSTN_LCSRN_POS))
#define MXC_F_NFC_SOFTRSTN_AFETCALSRN_POS                   2
#define MXC_F_NFC_SOFTRSTN_AFETCALSRN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_SOFTRSTN_AFETCALSRN_POS))
#define MXC_F_NFC_SOFTRSTN_AFEFDESRN_POS                    3
#define MXC_F_NFC_SOFTRSTN_AFEFDESRN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_SOFTRSTN_AFEFDESRN_POS))

#define MXC_F_NFC_DPP_CNTL1_DPPBYP_POS                      0
#define MXC_F_NFC_DPP_CNTL1_DPPBYP                          ((uint32_t)(0x00000007UL << MXC_F_NFC_DPP_CNTL1_DPPBYP_POS))

#define MXC_F_NFC_DPP_CNTL2_DPPPOL0_POS                     0
#define MXC_F_NFC_DPP_CNTL2_DPPPOL0                         ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL2_DPPPOL0_POS))
#define MXC_F_NFC_DPP_CNTL2_DPPPOL1_POS                     1
#define MXC_F_NFC_DPP_CNTL2_DPPPOL1                         ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL2_DPPPOL1_POS))
#define MXC_F_NFC_DPP_CNTL2_DPPSLEN_POS                     2
#define MXC_F_NFC_DPP_CNTL2_DPPSLEN                         ((uint32_t)(0x00000003UL << MXC_F_NFC_DPP_CNTL2_DPPSLEN_POS))
#define MXC_F_NFC_DPP_CNTL2_AFEFDESRN_POS                   4
#define MXC_F_NFC_DPP_CNTL2_AFEFDESRN                       ((uint32_t)(0x00000007UL << MXC_F_NFC_DPP_CNTL2_AFEFDESRN_POS))

#define MXC_F_NFC_DPP_CNTL3_DPPLDLY_POS                     0
#define MXC_F_NFC_DPP_CNTL3_DPPLDLY                         ((uint32_t)(0x00000003UL << MXC_F_NFC_DPP_CNTL3_DPPLDLY_POS))
#define MXC_F_NFC_DPP_CNTL3_DPPDGLIT_POS                    2
#define MXC_F_NFC_DPP_CNTL3_DPPDGLIT                        ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_CNTL3_DPPDGLIT_POS))

#define MXC_F_NFC_DPP_CAPINIT_DPPCAPI_POS                   0
#define MXC_F_NFC_DPP_CAPINIT_DPPCAPI                       ((uint32_t)(0x000000FFUL << MXC_F_NFC_DPP_CAPINIT_DPPCAPI_POS))

#define MXC_F_NFC_LC_TX_CNTL0_TXFRLEN_POS                   0
#define MXC_F_NFC_LC_TX_CNTL0_TXFRLEN                       ((uint32_t)(0x000001FFUL << MXC_F_NFC_LC_TX_CNTL0_TXFRLEN_POS))
#define MXC_F_NFC_LC_TX_CNTL0_TXADDBIT_POS                  9
#define MXC_F_NFC_LC_TX_CNTL0_TXADDBIT                      ((uint32_t)(0x00000007UL << MXC_F_NFC_LC_TX_CNTL0_TXADDBIT_POS))
#define MXC_F_NFC_LC_TX_CNTL0_TXIPOL_POS                    12
#define MXC_F_NFC_LC_TX_CNTL0_TXIPOL                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TX_CNTL0_TXIPOL_POS))

#define MXC_F_NFC_LC_SOFLEN_SOFLEN_POS                      0
#define MXC_F_NFC_LC_SOFLEN_SOFLEN                          ((uint32_t)(0x0000003FUL << MXC_F_NFC_LC_SOFLEN_SOFLEN_POS))

#define MXC_F_NFC_LC_CMD_TIMERMD_POS                        0
#define MXC_F_NFC_LC_CMD_TIMERMD                            ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_CMD_TIMERMD_POS))
#define MXC_F_NFC_LC_CMD_BYPASSLC_POS                       2
#define MXC_F_NFC_LC_CMD_BYPASSLC                           ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CMD_BYPASSLC_POS))
#define MXC_F_NFC_LC_CMD_STARTTX_POS                        3
#define MXC_F_NFC_LC_CMD_STARTTX                            ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CMD_STARTTX_POS))
#define MXC_F_NFC_LC_CMD_STARTRX_POS                        4
#define MXC_F_NFC_LC_CMD_STARTRX                            ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CMD_STARTRX_POS))
#define MXC_F_NFC_LC_CMD_FIFOWEN_POS                        5
#define MXC_F_NFC_LC_CMD_FIFOWEN                            ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CMD_FIFOWEN_POS))
#define MXC_F_NFC_LC_CMD_CDME_POS                           7
#define MXC_F_NFC_LC_CMD_CDME                               ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CMD_CDME_POS))

#define MXC_F_NFC_LC_TXRX_CFG_DRATE_POS                     0
#define MXC_F_NFC_LC_TXRX_CFG_DRATE                         ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_TXRX_CFG_DRATE_POS))
#define MXC_F_NFC_LC_TXRX_CFG_COMTYPE_POS                   2
#define MXC_F_NFC_LC_TXRX_CFG_COMTYPE                       ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_COMTYPE_POS))
#define MXC_F_NFC_LC_TXRX_CFG_BYPSTART_POS                  3
#define MXC_F_NFC_LC_TXRX_CFG_BYPSTART                      ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_BYPSTART_POS))
#define MXC_F_NFC_LC_TXRX_CFG_BYPEND_POS                    4
#define MXC_F_NFC_LC_TXRX_CFG_BYPEND                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_BYPEND_POS))
#define MXC_F_NFC_LC_TXRX_CFG_BYPEOF_POS                    5
#define MXC_F_NFC_LC_TXRX_CFG_BYPEOF                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_BYPEOF_POS))
#define MXC_F_NFC_LC_TXRX_CFG_BYPSOF_POS                    6
#define MXC_F_NFC_LC_TXRX_CFG_BYPSOF                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_BYPSOF_POS))
#define MXC_F_NFC_LC_TXRX_CFG_BYPCRC_POS                    7
#define MXC_F_NFC_LC_TXRX_CFG_BYPCRC                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TXRX_CFG_BYPCRC_POS))

#define MXC_F_NFC_LC_TYPEB_CFG1_RXBSOF1LEN_POS              2
#define MXC_F_NFC_LC_TYPEB_CFG1_RXBSOF1LEN                  ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_TYPEB_CFG1_RXBSOF1LEN_POS))
#define MXC_F_NFC_LC_TYPEB_CFG1_STSIRQSEL_POS               4
#define MXC_F_NFC_LC_TYPEB_CFG1_STSIRQSEL                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEB_CFG1_STSIRQSEL_POS))

#define MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMAX_POS              0
#define MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMAX                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMAX_POS))
#define MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMIN_POS              4
#define MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMIN                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_TYPEB_CFG2_RXBSOFZMIN_POS))

#define MXC_F_NFC_LC_RXB_MINTR2_RXBMINTR2_POS               0
#define MXC_F_NFC_LC_RXB_MINTR2_RXBMINTR2                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_RXB_MINTR2_RXBMINTR2_POS))

#define MXC_F_NFC_LC_RXB_MAXTR2_RXBMAXTR2_POS               0
#define MXC_F_NFC_LC_RXB_MAXTR2_RXBMAXTR2                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_RXB_MAXTR2_RXBMAXTR2_POS))

#define MXC_F_NFC_LC_STATUS_LCTMREXPF_POS                   0
#define MXC_F_NFC_LC_STATUS_LCTMREXPF                       ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_STATUS_LCTMREXPF_POS))
#define MXC_F_NFC_LC_STATUS_LCTMRRUNF_POS                   1
#define MXC_F_NFC_LC_STATUS_LCTMRRUNF                       ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_STATUS_LCTMRRUNF_POS))
#define MXC_F_NFC_LC_STATUS_COLLPOS_POS                     2
#define MXC_F_NFC_LC_STATUS_COLLPOS                         ((uint32_t)(0x00000007UL << MXC_F_NFC_LC_STATUS_COLLPOS_POS))

// Error code values are overloaded based on transaction type
#define MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS                0
#define MXC_F_NFC_LC_RXTX_STATUS_ERRCODE                    ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS))
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_NONE                     ((uint32_t)(0x00000000UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_NONE                     (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_NONE                     << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SOF               ((uint32_t)(0x00000001UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SOF               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SOF               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_TX_GAP            ((uint32_t)(0x00000002UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_TX_GAP            (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_TX_GAP            << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_PARITY            ((uint32_t)(0x00000003UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_PARITY            (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_PARITY            << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_EOF               ((uint32_t)(0x00000004UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_EOF               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_EOF               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_COLLISION         ((uint32_t)(0x00000005UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_COLLISION         (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_COLLISION         << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC               ((uint32_t)(0x00000006UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SAME_SAMPLE_UNDEF ((uint32_t)(0x00000007UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SAME_SAMPLE_UNDEF (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_SAME_SAMPLE_UNDEF << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_FSM_UNDEF         ((uint32_t)(0x00000008UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_FSM_UNDEF         (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_FSM_UNDEF         << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC_OR_COLL0      ((uint32_t)(0x0000000CUL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC_OR_COLL0      (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_A_CRC_OR_COLL0      << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOS             ((uint32_t)(0x0000000DUL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOS             (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOS             << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOF             ((uint32_t)(0x0000000EUL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOF             (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_NFC_SOF             << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SOF               ((uint32_t)(0x00000001UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SOF               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SOF               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_START_BIT         ((uint32_t)(0x00000002UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_START_BIT         (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_START_BIT         << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_STOP_BIT          ((uint32_t)(0x00000003UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_STOP_BIT          (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_STOP_BIT          << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_EOF               ((uint32_t)(0x00000004UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_EOF               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_EOF               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CHAR_GAP          ((uint32_t)(0x00000005UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CHAR_GAP          (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CHAR_GAP          << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CRC               ((uint32_t)(0x00000006UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CRC               (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_CRC               << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SAME_SAMPLE_UNDEF ((uint32_t)(0x00000007UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SAME_SAMPLE_UNDEF (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_SAME_SAMPLE_UNDEF << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_PICC_GAP      ((uint32_t)(0x00000008UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_PICC_GAP      (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_PICC_GAP      << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
#define MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_TR2           ((uint32_t)(0x00000009UL))
#define MXC_S_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_TR2           (MXC_V_NFC_LC_RXTX_STATUS_ERRCODE_TYPE_B_PCD_TR2           << MXC_F_NFC_LC_RXTX_STATUS_ERRCODE_POS)
/* TODO: Vicinity Errors
`define TYPE_V_SOF_UNMOD_ERROR         `ERR_CODE_MAX_LIMITP1'd1
`define TYPE_V_SOF_PULSE_ERROR         `ERR_CODE_MAX_LIMITP1'd2
`define TYPE_V_SOF_LOG_ERROR           `ERR_CODE_MAX_LIMITP1'd3
`define TYPE_V_SOF_DECODE_ERROR        `ERR_CODE_MAX_LIMITP1'd4
`define TYPE_V_EOF_UNMOD_ERROR         `ERR_CODE_MAX_LIMITP1'd5
`define TYPE_V_EOF_PULSE_ERROR         `ERR_CODE_MAX_LIMITP1'd6
`define TYPE_V_EOF_LOG_ERROR           `ERR_CODE_MAX_LIMITP1'd7
`define TYPE_V_EOF_DECODE_ERROR        `ERR_CODE_MAX_LIMITP1'd8
`define TYPE_V_CARD_SOF_PULSE_POS      `ERR_CODE_MAX_LIMITP1'd9
`define TYPE_V_CARD_SOF_PULSE_WIDTH    `ERR_CODE_MAX_LIMITP1'd10
`define	TYPE_V_SAMPLE_ERROR	           `ERR_CODE_MAX_LIMITP1'd11
`define TYPE_V_CRC_ERROR	             `ERR_CODE_MAX_LIMITP1'd12 */
#define MXC_F_NFC_LC_RXTX_STATUS_TXBUSY_POS                 4
#define MXC_F_NFC_LC_RXTX_STATUS_TXBUSY                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_RXTX_STATUS_TXBUSY_POS))
#define MXC_F_NFC_LC_RXTX_STATUS_RXBUSY_POS                 5
#define MXC_F_NFC_LC_RXTX_STATUS_RXBUSY                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_RXTX_STATUS_RXBUSY_POS))
#define MXC_F_NFC_LC_RXTX_STATUS_FIELDDET_POS               6
#define MXC_F_NFC_LC_RXTX_STATUS_FIELDDET                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_RXTX_STATUS_FIELDDET_POS))
#define MXC_F_NFC_LC_RXTX_STATUS_SUBCAROFF_POS              7
#define MXC_F_NFC_LC_RXTX_STATUS_SUBCAROFF                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_RXTX_STATUS_SUBCAROFF_POS))

#define MXC_F_NFC_LC_FIFO_LVL_FIFOLVL_POS                   0
#define MXC_F_NFC_LC_FIFO_LVL_FIFOLVL                       ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_FIFO_LVL_FIFOLVL_POS))

#define MXC_F_NFC_LC_FIFO_AF_FIFOAF_POS                     0
#define MXC_F_NFC_LC_FIFO_AF_FIFOAF                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_FIFO_AF_FIFOAF_POS))

#define MXC_F_NFC_LC_FIFO_AE_FIFOAE_POS                     0
#define MXC_F_NFC_LC_FIFO_AE_FIFOAE                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_FIFO_AE_FIFOAE_POS))

#define MXC_F_NFC_LC_TIMER_INIT_LCTMRINIT_POS               0
#define MXC_F_NFC_LC_TIMER_INIT_LCTMRINIT                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_TIMER_INIT_LCTMRINIT_POS))

#define MXC_F_NFC_LC_TIMER_RELOAD_LCTMRRLV_POS              0
#define MXC_F_NFC_LC_TIMER_RELOAD_LCTMRRLV                  ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_TIMER_RELOAD_LCTMRRLV_POS))

#define MXC_F_NFC_LC_TIMER_CNTL_FDFLTREN_POS                3
#define MXC_F_NFC_LC_TIMER_CNTL_FDFLTREN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TIMER_CNTL_FDFLTREN_POS))
#define MXC_F_NFC_LC_TIMER_CNTL_FDEN_POS                    4
#define MXC_F_NFC_LC_TIMER_CNTL_FDEN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TIMER_CNTL_FDEN_POS))
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRASEN_POS               5
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRASEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TIMER_CNTL_LCTMRASEN_POS))
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTOP_POS               6
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTOP                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTOP_POS))
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTART_POS              7
#define MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTART                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TIMER_CNTL_LCTMRSTART_POS))

#define MXC_F_NFC_LC_IRQ_STATUS_FDOFFIRQ_POS                0
#define MXC_F_NFC_LC_IRQ_STATUS_FDOFFIRQ                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_FDOFFIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_FIFOAFIRQ_POS               1
#define MXC_F_NFC_LC_IRQ_STATUS_FIFOAFIRQ                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_FIFOAFIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_FDONIRQ_POS                 2
#define MXC_F_NFC_LC_IRQ_STATUS_FDONIRQ                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_FDONIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_FIFOAEIRQ_POS               3
#define MXC_F_NFC_LC_IRQ_STATUS_FIFOAEIRQ                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_FIFOAEIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_LCTMREXIRQ_POS              4
#define MXC_F_NFC_LC_IRQ_STATUS_LCTMREXIRQ                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_LCTMREXIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_RXDONEIRQ_POS               5
#define MXC_F_NFC_LC_IRQ_STATUS_RXDONEIRQ                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_RXDONEIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_TXDONEIRQ_POS               6
#define MXC_F_NFC_LC_IRQ_STATUS_TXDONEIRQ                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_TXDONEIRQ_POS))
#define MXC_F_NFC_LC_IRQ_STATUS_STARTRXIRQ_POS              7
#define MXC_F_NFC_LC_IRQ_STATUS_STARTRXIRQ                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_STATUS_STARTRXIRQ_POS))

#define MXC_F_NFC_LC_IRQ_MASK_FDOFFIRQEN_POS                0
#define MXC_F_NFC_LC_IRQ_MASK_FDOFFIRQEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_FDOFFIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_FIFOAFIRQEN_POS               1
#define MXC_F_NFC_LC_IRQ_MASK_FIFOAFIRQEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_FIFOAFIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_FDONIRQEN_POS                 2
#define MXC_F_NFC_LC_IRQ_MASK_FDONIRQEN                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_FDONIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_FIFOAEIRQEN_POS               3
#define MXC_F_NFC_LC_IRQ_MASK_FIFOAEIRQEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_FIFOAEIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_LCTMREXIRQEN_POS              4
#define MXC_F_NFC_LC_IRQ_MASK_LCTMREXIRQEN                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_LCTMREXIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_RXDONEIRQEN_POS               5
#define MXC_F_NFC_LC_IRQ_MASK_RXDONEIRQEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_RXDONEIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_TXDONEIRQEN_POS               6
#define MXC_F_NFC_LC_IRQ_MASK_TXDONEIRQEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_TXDONEIRQEN_POS))
#define MXC_F_NFC_LC_IRQ_MASK_STARTRXIRQEN_POS              7
#define MXC_F_NFC_LC_IRQ_MASK_STARTRXIRQEN                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_MASK_STARTRXIRQEN_POS))

#define MXC_F_NFC_LC_IRQ_CLR_FDOFFIRQCLR_POS                0
#define MXC_F_NFC_LC_IRQ_CLR_FDOFFIRQCLR                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_FDOFFIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_FIFOAFIRQCLR_POS               1
#define MXC_F_NFC_LC_IRQ_CLR_FIFOAFIRQCLR                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_FIFOAFIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_FDONIRQCLR_POS                 2
#define MXC_F_NFC_LC_IRQ_CLR_FDONIRQCLR                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_FDONIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_FIFOAEIRQCLR_POS               3
#define MXC_F_NFC_LC_IRQ_CLR_FIFOAEIRQCLR                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_FIFOAEIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_LCTMREXIRQCLR_POS              4
#define MXC_F_NFC_LC_IRQ_CLR_LCTMREXIRQCLR                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_LCTMREXIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_RXDONEIRQCLR_POS               5
#define MXC_F_NFC_LC_IRQ_CLR_RXDONEIRQCLR                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_RXDONEIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_TXDONEIRQCLR_POS               6
#define MXC_F_NFC_LC_IRQ_CLR_TXDONEIRQCLR                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_TXDONEIRQCLR_POS))
#define MXC_F_NFC_LC_IRQ_CLR_STARTRXIRQCLR_POS              7
#define MXC_F_NFC_LC_IRQ_CLR_STARTRXIRQCLR                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_IRQ_CLR_STARTRXIRQCLR_POS))

#define MXC_F_NFC_LC_TX_MILLERPW_TXMILLERPW_POS             0
#define MXC_F_NFC_LC_TX_MILLERPW_TXMILLERPW                 ((uint32_t)(0x0000003FUL << MXC_F_NFC_LC_TX_MILLERPW_TXMILLERPW_POS))

#define MXC_F_NFC_LC_TIMER_PRESCALER_LCTMRPSCAL_POS         0
#define MXC_F_NFC_LC_TIMER_PRESCALER_LCTMRPSCAL             ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_TIMER_PRESCALER_LCTMRPSCAL_POS))

#define MXC_F_NFC_LC_FIFO_WDATA_FIFOWDATA_POS               0
#define MXC_F_NFC_LC_FIFO_WDATA_FIFOWDATA                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_FIFO_WDATA_FIFOWDATA_POS))

#define MXC_F_NFC_LC_FIFO_RDATA_FIFORDATA_POS               0
#define MXC_F_NFC_LC_FIFO_RDATA_FIFORDATA                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_FIFO_RDATA_FIFORDATA_POS))

#define MXC_F_NFC_LC_TIMER_VAL_LCTMRVAL_POS                 0
#define MXC_F_NFC_LC_TIMER_VAL_LCTMRVAL                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_TIMER_VAL_LCTMRVAL_POS))

#define MXC_F_NFC_LC_FIFO_STATUS_FIFOAE_POS                 0
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOAE                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_FIFOAE_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOEMP_POS                1
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOEMP                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_FIFOEMP_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOAF_POS                 2
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOAF                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_FIFOAF_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOFULL_POS               3
#define MXC_F_NFC_LC_FIFO_STATUS_FIFOFULL                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_FIFOFULL_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_VICC100ERR_POS             4
#define MXC_F_NFC_LC_FIFO_STATUS_VICC100ERR                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_VICC100ERR_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_VICC10ERR_POS              5
#define MXC_F_NFC_LC_FIFO_STATUS_VICC10ERR                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_VICC10ERR_POS))
#define MXC_F_NFC_LC_FIFO_STATUS_DPPLOCK_POS                6
#define MXC_F_NFC_LC_FIFO_STATUS_DPPLOCK                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_FIFO_STATUS_DPPLOCK_POS))

#define MXC_F_NFC_LC_RX_CFG_RXNADDBIT_POS                   0
#define MXC_F_NFC_LC_RX_CFG_RXNADDBIT                       ((uint32_t)(0x00000007UL << MXC_F_NFC_LC_RX_CFG_RXNADDBIT_POS))

#define MXC_F_NFC_CLKCFG_LCCLKPSEL_POS                      4
#define MXC_F_NFC_CLKCFG_LCCLKPSEL                          ((uint32_t)(0x00000001UL << MXC_F_NFC_CLKCFG_LCCLKPSEL_POS))
#define MXC_F_NFC_CLKCFG_LCCLKSEL_POS                       5
#define MXC_F_NFC_CLKCFG_LCCLKSEL                           ((uint32_t)(0x00000001UL << MXC_F_NFC_CLKCFG_LCCLKSEL_POS))
#define MXC_F_NFC_CLKCFG_DPPCLKSEL_POS                      6
#define MXC_F_NFC_CLKCFG_DPPCLKSEL                          ((uint32_t)(0x00000001UL << MXC_F_NFC_CLKCFG_DPPCLKSEL_POS))
#define MXC_F_NFC_CLKCFG_DPPCLKPSEL_POS                     7
#define MXC_F_NFC_CLKCFG_DPPCLKPSEL                         ((uint32_t)(0x00000001UL << MXC_F_NFC_CLKCFG_DPPCLKPSEL_POS))

#define MXC_F_NFC_AFE_TX_CFG1_AFETXMODI_POS                 0
#define MXC_F_NFC_AFE_TX_CFG1_AFETXMODI                     ((uint32_t)(0x0000007FUL << MXC_F_NFC_AFE_TX_CFG1_AFETXMODI_POS))
#define MXC_F_NFC_AFE_TX_CFG1_AFETXEN_POS                   7
#define MXC_F_NFC_AFE_TX_CFG1_AFETXEN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG1_AFETXEN_POS))

#define MXC_F_NFC_AFE_TX_CFG2_AFETXELEN_POS                 0
#define MXC_F_NFC_AFE_TX_CFG2_AFETXELEN                     ((uint32_t)(0x00000007UL << MXC_F_NFC_AFE_TX_CFG2_AFETXELEN_POS))
#define MXC_F_NFC_AFE_TX_CFG2_AFETXSEMD_POS                 3
#define MXC_F_NFC_AFE_TX_CFG2_AFETXSEMD                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG2_AFETXSEMD_POS))
#define MXC_F_NFC_AFE_TX_CFG2_AFETXCLKMD_POS                4
#define MXC_F_NFC_AFE_TX_CFG2_AFETXCLKMD                    ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG2_AFETXCLKMD_POS))
#define MXC_F_NFC_AFE_TX_CFG2_AFETXCLKSEL_POS               5
#define MXC_F_NFC_AFE_TX_CFG2_AFETXCLKSEL                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG2_AFETXCLKSEL_POS))
#define MXC_F_NFC_AFE_TX_CFG2_AFETXDSEL_POS                 6
#define MXC_F_NFC_AFE_TX_CFG2_AFETXDSEL                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG2_AFETXDSEL_POS))
#define MXC_F_NFC_AFE_TX_CFG2_AFETXEOVR_POS                 7
#define MXC_F_NFC_AFE_TX_CFG2_AFETXEOVR                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_CFG2_AFETXEOVR_POS))

#define MXC_F_NFC_AFE_RX_CFG1_RXSPEN_POS                    0
#define MXC_F_NFC_AFE_RX_CFG1_RXSPEN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXSPEN_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXDBEN_POS                    1
#define MXC_F_NFC_AFE_RX_CFG1_RXDBEN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXDBEN_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXMXENB_POS                   2
#define MXC_F_NFC_AFE_RX_CFG1_RXMXENB                       ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXMXENB_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXLPFEN_POS                   3
#define MXC_F_NFC_AFE_RX_CFG1_RXLPFEN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXLPFEN_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXDCDACEN_POS                 4
#define MXC_F_NFC_AFE_RX_CFG1_RXDCDACEN                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXDCDACEN_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXCLKADCEN_POS                5
#define MXC_F_NFC_AFE_RX_CFG1_RXCLKADCEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXCLKADCEN_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXADCFS_POS                   6
#define MXC_F_NFC_AFE_RX_CFG1_RXADCFS                       ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXADCFS_POS))
#define MXC_F_NFC_AFE_RX_CFG1_RXADCFSF_POS                  7
#define MXC_F_NFC_AFE_RX_CFG1_RXADCFSF                      ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG1_RXADCFSF_POS))

#define MXC_F_NFC_AFE_RX_CFG2_RX_LPF_GAIN_POS               0
#define MXC_F_NFC_AFE_RX_CFG2_RX_LPF_GAIN                   ((uint32_t)(0x0000000FUL << MXC_F_NFC_AFE_RX_CFG2_RX_LPF_GAIN_POS))
#define MXC_F_NFC_AFE_RX_CFG2_RX_LPF_BW_POS                 4
#define MXC_F_NFC_AFE_RX_CFG2_RX_LPF_BW                     ((uint32_t)(0x00000007UL << MXC_F_NFC_AFE_RX_CFG2_RX_LPF_BW_POS))

#define MXC_F_NFC_AFE_RX_TIME_REFH_RXTIMERREFHSEL_POS       0
#define MXC_F_NFC_AFE_RX_TIME_REFH_RXTIMERREFHSEL           ((uint32_t)(0x0000007FUL << MXC_F_NFC_AFE_RX_TIME_REFH_RXTIMERREFHSEL_POS))

#define MXC_F_NFC_AFE_RX_TIME_REFL_RXTIMERREFLSEL_POS       0
#define MXC_F_NFC_AFE_RX_TIME_REFL_RXTIMERREFLSEL           ((uint32_t)(0x0000007FUL << MXC_F_NFC_AFE_RX_TIME_REFL_RXTIMERREFLSEL_POS))

#define MXC_F_NFC_AFE_RX_DC_I_DC_I_POS                      0
#define MXC_F_NFC_AFE_RX_DC_I_DC_I                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_AFE_RX_DC_I_DC_I_POS))

#define MXC_F_NFC_AFE_RX_DC_Q_DC_Q_POS                      0
#define MXC_F_NFC_AFE_RX_DC_Q_DC_Q                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_AFE_RX_DC_Q_DC_Q_POS))

#define MXC_F_NFC_AFE_RX_AZCFDVSELH_RXAZCFDVSELH_POS        0
#define MXC_F_NFC_AFE_RX_AZCFDVSELH_RXAZCFDVSELH            ((uint32_t)(0x000000FFUL << MXC_F_NFC_AFE_RX_AZCFDVSELH_RXAZCFDVSELH_POS))

#define MXC_F_NFC_AFE_RX_AZCFDVSELL_RXAZCFDVSELL_POS        0
#define MXC_F_NFC_AFE_RX_AZCFDVSELL_RXAZCFDVSELL            ((uint32_t)(0x000000FFUL << MXC_F_NFC_AFE_RX_AZCFDVSELL_RXAZCFDVSELL_POS))

#define MXC_F_NFC_AFE_RX_AZCATTFSEL_RXAZCATTFSELI_POS       0
#define MXC_F_NFC_AFE_RX_AZCATTFSEL_RXAZCATTFSELI           ((uint32_t)(0x0000000FUL << MXC_F_NFC_AFE_RX_AZCATTFSEL_RXAZCATTFSELI_POS))

#define MXC_F_NFC_AFE_RX_AZCSTATUS_AZCDONE_POS              0
#define MXC_F_NFC_AFE_RX_AZCSTATUS_AZCDONE                  ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_AZCSTATUS_AZCDONE_POS))
#define MXC_F_NFC_AFE_RX_AZCSTATUS_MINFSELFAIL_POS          1
#define MXC_F_NFC_AFE_RX_AZCSTATUS_MINFSELFAIL              ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_AZCSTATUS_MINFSELFAIL_POS))
#define MXC_F_NFC_AFE_RX_AZCSTATUS_MAXFSELFAIL_POS          2
#define MXC_F_NFC_AFE_RX_AZCSTATUS_MAXFSELFAIL              ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_AZCSTATUS_MAXFSELFAIL_POS))

#define MXC_F_NFC_OFFS_COMP0_OFSCMPEN_POS                   0
#define MXC_F_NFC_OFFS_COMP0_OFSCMPEN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP0_OFSCMPEN_POS))
#define MXC_F_NFC_OFFS_COMP0_OFSCMPDIGIQEN_POS              1
#define MXC_F_NFC_OFFS_COMP0_OFSCMPDIGIQEN                  ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP0_OFSCMPDIGIQEN_POS))
#define MXC_F_NFC_OFFS_COMP0_OFSCMPDONE_POS                 7
#define MXC_F_NFC_OFFS_COMP0_OFSCMPDONE                     ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP0_OFSCMPDONE_POS))

#define MXC_F_NFC_OFFS_COMP1_OFSCMPSAVGCNT_POS              0
#define MXC_F_NFC_OFFS_COMP1_OFSCMPSAVGCNT                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_OFFS_COMP1_OFSCMPSAVGCNT_POS))
#define MXC_F_NFC_OFFS_COMP1_OFSCMPOWID_POS                 4
#define MXC_F_NFC_OFFS_COMP1_OFSCMPOWID                     ((uint32_t)(0x0000000FUL << MXC_F_NFC_OFFS_COMP1_OFSCMPOWID_POS))

#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACEN_POS                0
#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP2_OFSCMPLACEN_POS))
#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACRST_POS               1
#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACRST                   ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP2_OFSCMPLACRST_POS))
#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACF_POS                 3
#define MXC_F_NFC_OFFS_COMP2_OFSCMPLACF                     ((uint32_t)(0x0000000FUL << MXC_F_NFC_OFFS_COMP2_OFSCMPLACF_POS))
#define MXC_F_NFC_OFFS_COMP2_OFSCMPOFSDIR_POS               7
#define MXC_F_NFC_OFFS_COMP2_OFSCMPOFSDIR                   ((uint32_t)(0x00000001UL << MXC_F_NFC_OFFS_COMP2_OFSCMPOFSDIR_POS))

#define MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXSEL_POS              0
#define MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXSEL                  ((uint32_t)(0x0000001FUL << MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXSEL_POS))
#define MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXE_POS                5
#define MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXE                    ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG3_AFERXAMUXE_POS))
#define MXC_F_NFC_AFE_RX_CFG3_AFERXSWADCEN_POS              6
#define MXC_F_NFC_AFE_RX_CFG3_AFERXSWADCEN                  ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG3_AFERXSWADCEN_POS))
#define MXC_F_NFC_AFE_RX_CFG3_AFERXFDTEN_POS                7
#define MXC_F_NFC_AFE_RX_CFG3_AFERXFDTEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG3_AFERXFDTEN_POS))

#define MXC_F_NFC_AFE_RX_CFG4_AFERXMUXSEL_POS               0
#define MXC_F_NFC_AFE_RX_CFG4_AFERXMUXSEL                   ((uint32_t)(0x0000003FUL << MXC_F_NFC_AFE_RX_CFG4_AFERXMUXSEL_POS))
#define MXC_F_NFC_AFE_RX_CFG4_AFERXTMCLKADC_POS             6
#define MXC_F_NFC_AFE_RX_CFG4_AFERXTMCLKADC                 ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG4_AFERXTMCLKADC_POS))
#define MXC_F_NFC_AFE_RX_CFG4_AFERXFDEN_POS                 7
#define MXC_F_NFC_AFE_RX_CFG4_AFERXFDEN                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG4_AFERXFDEN_POS))

#define MXC_F_NFC_AFE_RX_ATTCFG_RXATTFSEL_POS               0
#define MXC_F_NFC_AFE_RX_ATTCFG_RXATTFSEL                   ((uint32_t)(0x0000001FUL << MXC_F_NFC_AFE_RX_ATTCFG_RXATTFSEL_POS))

#define MXC_F_NFC_AFE_RX_CFG9_RXATTRESMODSEL_POS            4
#define MXC_F_NFC_AFE_RX_CFG9_RXATTRESMODSEL                ((uint32_t)(0x0000000FUL << MXC_F_NFC_AFE_RX_CFG9_RXATTRESMODSEL_POS))

#define MXC_F_NFC_AFE_RX_CFG5_AFERFDVSEL_POS                0
#define MXC_F_NFC_AFE_RX_CFG5_AFERFDVSEL                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_AFE_RX_CFG5_AFERFDVSEL_POS))

#define MXC_F_NFC_AFE_RX_CFG6_AFERFDPD_POS                  0
#define MXC_F_NFC_AFE_RX_CFG6_AFERFDPD                      ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG6_AFERFDPD_POS))
#define MXC_F_NFC_AFE_RX_CFG6_AFERAZCPD_POS                 1
#define MXC_F_NFC_AFE_RX_CFG6_AFERAZCPD                     ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG6_AFERAZCPD_POS))
#define MXC_F_NFC_AFE_RX_CFG6_AZCEN_POS                     2
#define MXC_F_NFC_AFE_RX_CFG6_AZCEN                         ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG6_AZCEN_POS))
#define MXC_F_NFC_AFE_RX_CFG6_AFERTREFPD_POS                3
#define MXC_F_NFC_AFE_RX_CFG6_AFERTREFPD                    ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG6_AFERTREFPD_POS))
#define MXC_F_NFC_AFE_RX_CFG6_AFEFRCVSEL_POS                4
#define MXC_F_NFC_AFE_RX_CFG6_AFEFRCVSEL                    ((uint32_t)(0x00000007UL << MXC_F_NFC_AFE_RX_CFG6_AFEFRCVSEL_POS))
#define MXC_F_NFC_AFE_RX_CFG6_AFERXATTRESMODBYP_POS         7
#define MXC_F_NFC_AFE_RX_CFG6_AFERXATTRESMODBYP             ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG6_AFERXATTRESMODBYP_POS))

#define MXC_F_NFC_AFE_CAL_STATUS_AFETDCALST_POS             0
#define MXC_F_NFC_AFE_CAL_STATUS_AFETDCALST                 ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_CAL_STATUS_AFETDCALST_POS))
#define MXC_F_NFC_AFE_CAL_STATUS_AFERTHLDST_POS             1
#define MXC_F_NFC_AFE_CAL_STATUS_AFERTHLDST                 ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_CAL_STATUS_AFERTHLDST_POS))
#define MXC_F_NFC_AFE_CAL_STATUS_AFERFDST_POS               2
#define MXC_F_NFC_AFE_CAL_STATUS_AFERFDST                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_CAL_STATUS_AFERFDST_POS))

#define MXC_F_NFC_AFE_MISC_TEST_AFERMONSW_POS               0
#define MXC_F_NFC_AFE_MISC_TEST_AFERMONSW                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_MISC_TEST_AFERMONSW_POS))
#define MXC_F_NFC_AFE_MISC_TEST_AFERD2SEN_POS               1
#define MXC_F_NFC_AFE_MISC_TEST_AFERD2SEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_MISC_TEST_AFERD2SEN_POS))
#define MXC_F_NFC_AFE_MISC_TEST_AFERD2SMUX_POS              2
#define MXC_F_NFC_AFE_MISC_TEST_AFERD2SMUX                  ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_MISC_TEST_AFERD2SMUX_POS))
#define MXC_F_NFC_AFE_MISC_TEST_AFEMBIAS_POS                3
#define MXC_F_NFC_AFE_MISC_TEST_AFEMBIAS                    ((uint32_t)(0x00000007UL << MXC_F_NFC_AFE_MISC_TEST_AFEMBIAS_POS))

#define MXC_F_NFC_AFE_RX_CFG7_AFERXFDCVHSEL_POS             0
#define MXC_F_NFC_AFE_RX_CFG7_AFERXFDCVHSEL                 ((uint32_t)(0x00000003UL << MXC_F_NFC_AFE_RX_CFG7_AFERXFDCVHSEL_POS))
#define MXC_F_NFC_AFE_RX_CFG7_AFERXIBOP_POS                 2
#define MXC_F_NFC_AFE_RX_CFG7_AFERXIBOP                     ((uint32_t)(0x00000003UL << MXC_F_NFC_AFE_RX_CFG7_AFERXIBOP_POS))
#define MXC_F_NFC_AFE_RX_CFG7_AFERXVCM_POS                  4
#define MXC_F_NFC_AFE_RX_CFG7_AFERXVCM                      ((uint32_t)(0x00000007UL << MXC_F_NFC_AFE_RX_CFG7_AFERXVCM_POS))
#define MXC_F_NFC_AFE_RX_CFG7_AFERXBIASEN_POS               7
#define MXC_F_NFC_AFE_RX_CFG7_AFERXBIASEN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_RX_CFG7_AFERXBIASEN_POS))

#define MXC_F_NFC_MATCHPAT0_IQMPAT_POS                      0
#define MXC_F_NFC_MATCHPAT0_IQMPAT                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPAT0_IQMPAT_POS))

#define MXC_F_NFC_MATCHPAT1_IQMPAT_POS                      0
#define MXC_F_NFC_MATCHPAT1_IQMPAT                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPAT1_IQMPAT_POS))

#define MXC_F_NFC_MATCHPAT2_IQMPAT_POS                      0
#define MXC_F_NFC_MATCHPAT2_IQMPAT                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPAT2_IQMPAT_POS))

#define MXC_F_NFC_MATCHPAT3_IQMPAT_POS                      0
#define MXC_F_NFC_MATCHPAT3_IQMPAT                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPAT3_IQMPAT_POS))

#define MXC_F_NFC_MATCHPATM0_IQMPAT_POS                     0
#define MXC_F_NFC_MATCHPATM0_IQMPAT                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPATM0_IQMPAT_POS))

#define MXC_F_NFC_MATCHPATM1_IQMPAT_POS                     0
#define MXC_F_NFC_MATCHPATM1_IQMPAT                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPATM1_IQMPAT_POS))

#define MXC_F_NFC_MATCHPATM2_IQMPAT_POS                     0
#define MXC_F_NFC_MATCHPATM2_IQMPAT                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPATM2_IQMPAT_POS))

#define MXC_F_NFC_MATCHPATM3_IQMPAT_POS                     0
#define MXC_F_NFC_MATCHPATM3_IQMPAT                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_MATCHPATM3_IQMPAT_POS))

#define MXC_F_NFC_AFE_TX_DSEL_AFETDSEL_POS                  0
#define MXC_F_NFC_AFE_TX_DSEL_AFETDSEL                      ((uint32_t)(0x0000000FUL << MXC_F_NFC_AFE_TX_DSEL_AFETDSEL_POS))
#define MXC_F_NFC_AFE_TX_DSEL_AFETMODTYPE_POS               6
#define MXC_F_NFC_AFE_TX_DSEL_AFETMODTYPE                   ((uint32_t)(0x00000001UL << MXC_F_NFC_AFE_TX_DSEL_AFETMODTYPE_POS))

#define MXC_F_NFC_AFE_RX_LSBSEL_AFERXLSBS_POS               0
#define MXC_F_NFC_AFE_RX_LSBSEL_AFERXLSBS                   ((uint32_t)(0x00000003UL << MXC_F_NFC_AFE_RX_LSBSEL_AFERXLSBS_POS))

#define MXC_F_NFC_FIRSKIPMS_FRSKIPMS_POS                    0
#define MXC_F_NFC_FIRSKIPMS_FRSKIPMS                        ((uint32_t)(0x0000003FUL << MXC_F_NFC_FIRSKIPMS_FRSKIPMS_POS))

#define MXC_F_NFC_FIRSFACTR_FIRSF_POS                       0
#define MXC_F_NFC_FIRSFACTR_FIRSF                           ((uint32_t)(0x0000000FUL << MXC_F_NFC_FIRSFACTR_FIRSF_POS))
#define MXC_F_NFC_FIRSFACTR_FIR_IBYP_POS                    6
#define MXC_F_NFC_FIRSFACTR_FIR_IBYP                        ((uint32_t)(0x00000001UL << MXC_F_NFC_FIRSFACTR_FIR_IBYP_POS))
#define MXC_F_NFC_FIRSFACTR_FIR_QBYP_POS                    7
#define MXC_F_NFC_FIRSFACTR_FIR_QBYP                        ((uint32_t)(0x00000001UL << MXC_F_NFC_FIRSFACTR_FIR_QBYP_POS))

#define MXC_F_NFC_IQC_IBUFSEL_BUFSELI0_POS                  0
#define MXC_F_NFC_IQC_IBUFSEL_BUFSELI0                      ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IBUFSEL_BUFSELI0_POS))
#define MXC_F_NFC_IQC_IBUFSEL_BUFSELI1_POS                  4
#define MXC_F_NFC_IQC_IBUFSEL_BUFSELI1                      ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IBUFSEL_BUFSELI1_POS))

#define MXC_F_NFC_IQC_QBUFSEL_BUFSELQ0_POS                  0
#define MXC_F_NFC_IQC_QBUFSEL_BUFSELQ0                      ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_QBUFSEL_BUFSELQ0_POS))
#define MXC_F_NFC_IQC_QBUFSEL_BUFSELQ1_POS                  4
#define MXC_F_NFC_IQC_QBUFSEL_BUFSELQ1                      ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_QBUFSEL_BUFSELQ1_POS))

#define MXC_F_NFC_IQC_MATH_ADDSUBI_POS                      0
#define MXC_F_NFC_IQC_MATH_ADDSUBI                          ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ADDSUBI_POS))
#define MXC_F_NFC_IQC_MATH_ADDSUBIBYP_POS                   1
#define MXC_F_NFC_IQC_MATH_ADDSUBIBYP                       ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ADDSUBIBYP_POS))
#define MXC_F_NFC_IQC_MATH_ABSI_POS                         2
#define MXC_F_NFC_IQC_MATH_ABSI                             ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ABSI_POS))
#define MXC_F_NFC_IQC_MATH_ADDSUBQ_POS                      4
#define MXC_F_NFC_IQC_MATH_ADDSUBQ                          ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ADDSUBQ_POS))
#define MXC_F_NFC_IQC_MATH_ADDSUBQBYP_POS                   5
#define MXC_F_NFC_IQC_MATH_ADDSUBQBYP                       ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ADDSUBQBYP_POS))
#define MXC_F_NFC_IQC_MATH_ABSQ_POS                         6
#define MXC_F_NFC_IQC_MATH_ABSQ                             ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_MATH_ABSQ_POS))

#define MXC_F_NFC_IQC_IQSEL_IQOPSEL_POS                     0
#define MXC_F_NFC_IQC_IQSEL_IQOPSEL                         ((uint32_t)(0x00000003UL << MXC_F_NFC_IQC_IQSEL_IQOPSEL_POS))
#define MXC_F_NFC_IQC_IQSEL_MUXSELIQ0_POS                   2
#define MXC_F_NFC_IQC_IQSEL_MUXSELIQ0                       ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_IQSEL_MUXSELIQ0_POS))
#define MXC_F_NFC_IQC_IQSEL_MUXSELIQ1_POS                   3
#define MXC_F_NFC_IQC_IQSEL_MUXSELIQ1                       ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_IQSEL_MUXSELIQ1_POS))

#define MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ0_POS               0
#define MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ0                   ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ0_POS))
#define MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ1_POS               4
#define MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ1                   ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IQBUFSEL0_BUFSELIQ1_POS))

#define MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ2_POS               0
#define MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ2                   ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ2_POS))
#define MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ3_POS               4
#define MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ3                   ((uint32_t)(0x00000007UL << MXC_F_NFC_IQC_IQBUFSEL1_BUFSELIQ3_POS))

#define MXC_F_NFC_IQC_THRSEL_MAVGSEL_POS                    0
#define MXC_F_NFC_IQC_THRSEL_MAVGSEL                        ((uint32_t)(0x00000003UL << MXC_F_NFC_IQC_THRSEL_MAVGSEL_POS))
#define MXC_F_NFC_IQC_THRSEL_MAVGSELPK_POS                  3
#define MXC_F_NFC_IQC_THRSEL_MAVGSELPK                      ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_THRSEL_MAVGSELPK_POS))
#define MXC_F_NFC_IQC_THRSEL_THRSELPK_POS                   4
#define MXC_F_NFC_IQC_THRSEL_THRSELPK                       ((uint32_t)(0x00000003UL << MXC_F_NFC_IQC_THRSEL_THRSELPK_POS))

#define MXC_F_NFC_IQC_HYS_HYSCON_POS                        0
#define MXC_F_NFC_IQC_HYS_HYSCON                            ((uint32_t)(0x000000FFUL << MXC_F_NFC_IQC_HYS_HYSCON_POS))

#define MXC_F_NFC_IQC_DATASEL_DATASEL_POS                   0
#define MXC_F_NFC_IQC_DATASEL_DATASEL                       ((uint32_t)(0x00000003UL << MXC_F_NFC_IQC_DATASEL_DATASEL_POS))
#define MXC_F_NFC_IQC_DATASEL_POLSEL_POS                    4
#define MXC_F_NFC_IQC_DATASEL_POLSEL                        ((uint32_t)(0x00000001UL << MXC_F_NFC_IQC_DATASEL_POLSEL_POS))

#define MXC_F_NFC_MISC_TEST_CFG1_PICCTXDSEL_POS             0
#define MXC_F_NFC_MISC_TEST_CFG1_PICCTXDSEL                 ((uint32_t)(0x00000001UL << MXC_F_NFC_MISC_TEST_CFG1_PICCTXDSEL_POS))
#define MXC_F_NFC_MISC_TEST_CFG1_PCDTXDSEL_POS              1
#define MXC_F_NFC_MISC_TEST_CFG1_PCDTXDSEL                  ((uint32_t)(0x00000001UL << MXC_F_NFC_MISC_TEST_CFG1_PCDTXDSEL_POS))

#define MXC_F_NFC_MISC_TEST_CFG2_LCIRQEN_POS                5
#define MXC_F_NFC_MISC_TEST_CFG2_LCIRQEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_MISC_TEST_CFG2_LCIRQEN_POS))

#define MXC_F_NFC_LC_MANSMP106_MANSMP106_POS                0
#define MXC_F_NFC_LC_MANSMP106_MANSMP106                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_MANSMP106_MANSMP106_POS))

#define MXC_F_NFC_LC_MANSMP212_MANSMP212_POS                0
#define MXC_F_NFC_LC_MANSMP212_MANSMP212                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_MANSMP212_MANSMP212_POS))

#define MXC_F_NFC_LC_MANSMP424_MANSMP424_POS                0
#define MXC_F_NFC_LC_MANSMP424_MANSMP424                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_MANSMP424_MANSMP424_POS))

#define MXC_F_NFC_LC_NFC_CNTL_NFCTYP_POS                    0
#define MXC_F_NFC_LC_NFC_CNTL_NFCTYP                        ((uint32_t)(0x00000007UL << MXC_F_NFC_LC_NFC_CNTL_NFCTYP_POS))
#define MXC_F_NFC_LC_NFC_CNTL_NFCDPOLVAL_POS                3
#define MXC_F_NFC_LC_NFC_CNTL_NFCDPOLVAL                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_CNTL_NFCDPOLVAL_POS))
#define MXC_F_NFC_LC_NFC_CNTL_NFCDPOLOVR_POS                4
#define MXC_F_NFC_LC_NFC_CNTL_NFCDPOLOVR                    ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_CNTL_NFCDPOLOVR_POS))
#define MXC_F_NFC_LC_NFC_CNTL_MSBMD_POS                     5
#define MXC_F_NFC_LC_NFC_CNTL_MSBMD                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_CNTL_MSBMD_POS))
#define MXC_F_NFC_LC_NFC_CNTL_NFCMD_POS                     6
#define MXC_F_NFC_LC_NFC_CNTL_NFCMD                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_CNTL_NFCMD_POS))
#define MXC_F_NFC_LC_NFC_CNTL_NFCSOSBYP_POS                 7
#define MXC_F_NFC_LC_NFC_CNTL_NFCSOSBYP                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_CNTL_NFCSOSBYP_POS))

#define MXC_F_NFC_LC_NFC_TXSOSL_NFCTXSOSL_POS               0
#define MXC_F_NFC_LC_NFC_TXSOSL_NFCTXSOSL                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_TXSOSL_NFCTXSOSL_POS))

#define MXC_F_NFC_LC_NFC_RXEOF106_NFCRXEOF106_POS           0
#define MXC_F_NFC_LC_NFC_RXEOF106_NFCRXEOF106               ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXEOF106_NFCRXEOF106_POS))

#define MXC_F_NFC_LC_NFC_RXEOF212_NFCRXEOF212_POS           0
#define MXC_F_NFC_LC_NFC_RXEOF212_NFCRXEOF212               ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXEOF212_NFCRXEOF212_POS))

#define MXC_F_NFC_LC_NFC_RXEOF424_NFCRXEOF424_POS           0
#define MXC_F_NFC_LC_NFC_RXEOF424_NFCRXEOF424               ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXEOF424_NFCRXEOF424_POS))

#define MXC_F_NFC_LC_NFC_RXEOF848_NFCRXEOF848_POS           0
#define MXC_F_NFC_LC_NFC_RXEOF848_NFCRXEOF848               ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXEOF848_NFCRXEOF848_POS))

#define MXC_F_NFC_LC_RX_TYPBEGT_RXTYPBEGT_POS               0
#define MXC_F_NFC_LC_RX_TYPBEGT_RXTYPBEGT                   ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RX_TYPBEGT_RXTYPBEGT_POS))

#define MXC_F_NFC_LC_CNTL1_NFCSOSLKE_POS                    0
#define MXC_F_NFC_LC_CNTL1_NFCSOSLKE                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_NFCSOSLKE_POS))
#define MXC_F_NFC_LC_CNTL1_DPPLKVAL_POS                     1
#define MXC_F_NFC_LC_CNTL1_DPPLKVAL                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_DPPLKVAL_POS))
#define MXC_F_NFC_LC_CNTL1_DPPLKOVR_POS                     2
#define MXC_F_NFC_LC_CNTL1_DPPLKOVR                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_DPPLKOVR_POS))
#define MXC_F_NFC_LC_CNTL1_NFCASWEN_POS                     3
#define MXC_F_NFC_LC_CNTL1_NFCASWEN                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_NFCASWEN_POS))
#define MXC_F_NFC_LC_CNTL1_NFCP2PEN_POS                     4
#define MXC_F_NFC_LC_CNTL1_NFCP2PEN                         ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_NFCP2PEN_POS))
#define MXC_F_NFC_LC_CNTL1_CDTXSTGEN_POS                    5
#define MXC_F_NFC_LC_CNTL1_CDTXSTGEN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_CDTXSTGEN_POS))
#define MXC_F_NFC_LC_CNTL1_NFCADOVREN_POS                   6
#define MXC_F_NFC_LC_CNTL1_NFCADOVREN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_NFCADOVREN_POS))
#define MXC_F_NFC_LC_CNTL1_NFCSOFBYP_POS                    7
#define MXC_F_NFC_LC_CNTL1_NFCSOFBYP                        ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CNTL1_NFCSOFBYP_POS))

#define MXC_F_NFC_DPP_CNTL4_DPPDIV2_POS                     0
#define MXC_F_NFC_DPP_CNTL4_DPPDIV2                         ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPDIV2_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPTYPADSEL_POS                 1
#define MXC_F_NFC_DPP_CNTL4_DPPTYPADSEL                     ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPTYPADSEL_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPTYPASMPEN_POS                2
#define MXC_F_NFC_DPP_CNTL4_DPPTYPASMPEN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPTYPASMPEN_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPVPHSYNMD_POS                 3
#define MXC_F_NFC_DPP_CNTL4_DPPVPHSYNMD                     ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPVPHSYNMD_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPPHSYNEN_POS                  4
#define MXC_F_NFC_DPP_CNTL4_DPPPHSYNEN                      ((uint32_t)(0x00000003UL << MXC_F_NFC_DPP_CNTL4_DPPPHSYNEN_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPDOVREN_POS                   6
#define MXC_F_NFC_DPP_CNTL4_DPPDOVREN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPDOVREN_POS))
#define MXC_F_NFC_DPP_CNTL4_DPPDOVRVAL_POS                  7
#define MXC_F_NFC_DPP_CNTL4_DPPDOVRVAL                      ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CNTL4_DPPDOVRVAL_POS))

#define MXC_F_NFC_DPP_CNTL5_TASMPTHR_POS                    0
#define MXC_F_NFC_DPP_CNTL5_TASMPTHR                        ((uint32_t)(0x0000003FUL << MXC_F_NFC_DPP_CNTL5_TASMPTHR_POS))

#define MXC_F_NFC_LC_TASOF1C128_TASOF1C128_POS              0
#define MXC_F_NFC_LC_TASOF1C128_TASOF1C128                  ((uint32_t)(0x0000007FUL << MXC_F_NFC_LC_TASOF1C128_TASOF1C128_POS))

#define MXC_F_NFC_LC_TASOF1C64_TASOF1C64_POS                0
#define MXC_F_NFC_LC_TASOF1C64_TASOF1C64                    ((uint32_t)(0x0000007FUL << MXC_F_NFC_LC_TASOF1C64_TASOF1C64_POS))

#define MXC_F_NFC_LC_TASOF1C32_TASOF1C32_POS                0
#define MXC_F_NFC_LC_TASOF1C32_TASOF1C32                    ((uint32_t)(0x0000007FUL << MXC_F_NFC_LC_TASOF1C32_TASOF1C32_POS))

#define MXC_F_NFC_LC_TASOF1C16_TASOF1C16_POS                0
#define MXC_F_NFC_LC_TASOF1C16_TASOF1C16                    ((uint32_t)(0x0000007FUL << MXC_F_NFC_LC_TASOF1C16_TASOF1C16_POS))

#define MXC_F_NFC_LC_TYPEV_CNTL1_VMEN_POS                   0
#define MXC_F_NFC_LC_TYPEV_CNTL1_VMEN                       ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VMEN_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL1_VCEMEN_POS                 1
#define MXC_F_NFC_LC_TYPEV_CNTL1_VCEMEN                     ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VCEMEN_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL1_VDSCAREN_POS               2
#define MXC_F_NFC_LC_TYPEV_CNTL1_VDSCAREN                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VDSCAREN_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL1_VSCAROVREN_POS             3
#define MXC_F_NFC_LC_TYPEV_CNTL1_VSCAROVREN                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VSCAROVREN_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL1_VEOFDETEN_POS              4
#define MXC_F_NFC_LC_TYPEV_CNTL1_VEOFDETEN                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VEOFDETEN_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL1_VEOFCNTRST_POS             5
#define MXC_F_NFC_LC_TYPEV_CNTL1_VEOFCNTRST                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL1_VEOFCNTRST_POS))

#define MXC_F_NFC_LC_TYPEV_CNTL2_VRXDGLTBYP_POS             0
#define MXC_F_NFC_LC_TYPEV_CNTL2_VRXDGLTBYP                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL2_VRXDGLTBYP_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL2_VTXDATABYP_POS             1
#define MXC_F_NFC_LC_TYPEV_CNTL2_VTXDATABYP                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL2_VTXDATABYP_POS))
#define MXC_F_NFC_LC_TYPEV_CNTL2_VRXARBYP_POS               2
#define MXC_F_NFC_LC_TYPEV_CNTL2_VRXARBYP                   ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_TYPEV_CNTL2_VRXARBYP_POS))

#define MXC_F_NFC_LC_VRXUMSSL0_VRXUMSSL_POS                 0
#define MXC_F_NFC_LC_VRXUMSSL0_VRXUMSSL                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXUMSSL0_VRXUMSSL_POS))

#define MXC_F_NFC_LC_VRXUMSSL1_VRXUMSSL_POS                 0
#define MXC_F_NFC_LC_VRXUMSSL1_VRXUMSSL                     ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VRXUMSSL1_VRXUMSSL_POS))

#define MXC_F_NFC_LC_VRXUMDSL0_VRXUMDSL_POS                 0
#define MXC_F_NFC_LC_VRXUMDSL0_VRXUMDSL                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXUMDSL0_VRXUMDSL_POS))

#define MXC_F_NFC_LC_VRXUMDSL1_VRXUMDSL_POS                 0
#define MXC_F_NFC_LC_VRXUMDSL1_VRXUMDSL                     ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VRXUMDSL1_VRXUMDSL_POS))

#define MXC_F_NFC_LC_VRXSPCFD_VRXSPCFD_POS                  0
#define MXC_F_NFC_LC_VRXSPCFD_VRXSPCFD                      ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXSPCFD_VRXSPCFD_POS))

#define MXC_F_NFC_LC_VRXSPLFD0_VRXSPLFD_POS                 0
#define MXC_F_NFC_LC_VRXSPLFD0_VRXSPLFD                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXSPLFD0_VRXSPLFD_POS))

#define MXC_F_NFC_LC_VRXSPLFD1_VRXSPLFD_POS                 0
#define MXC_F_NFC_LC_VRXSPLFD1_VRXSPLFD                     ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VRXSPLFD1_VRXSPLFD_POS))

#define MXC_F_NFC_LC_VRXSPLFDT_VRXSPLFDT_POS                0
#define MXC_F_NFC_LC_VRXSPLFDT_VRXSPLFDT                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXSPLFDT_VRXSPLFDT_POS))

#define MXC_F_NFC_LC_VRXSSC_VRXSSC_POS                      0
#define MXC_F_NFC_LC_VRXSSC_VRXSSC                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXSSC_VRXSSC_POS))

#define MXC_F_NFC_LC_VRXDSC_VRXDSC_POS                      0
#define MXC_F_NFC_LC_VRXDSC_VRXDSC                          ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXDSC_VRXDSC_POS))

#define MXC_F_NFC_LC_VRXSSCT_VRXSSCT_POS                    0
#define MXC_F_NFC_LC_VRXSSCT_VRXSSCT                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXSSCT_VRXSSCT_POS))

#define MXC_F_NFC_LC_VRXEOFW_VRXEOFW_POS                    0
#define MXC_F_NFC_LC_VRXEOFW_VRXEOFW                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXEOFW_VRXEOFW_POS))

#define MXC_F_NFC_LC_VRXDSHBS0_VRXDSHBS_POS                 0
#define MXC_F_NFC_LC_VRXDSHBS0_VRXDSHBS                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXDSHBS0_VRXDSHBS_POS))

#define MXC_F_NFC_LC_VRXDSHBS1_VRXDSHBS_POS                 0
#define MXC_F_NFC_LC_VRXDSHBS1_VRXDSHBS                     ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_VRXDSHBS1_VRXDSHBS_POS))

#define MXC_F_NFC_LC_VRXDSHBL0_VRXDSHBL_POS                 0
#define MXC_F_NFC_LC_VRXDSHBL0_VRXDSHBL                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXDSHBL0_VRXDSHBL_POS))

#define MXC_F_NFC_LC_VRXDSHBL1_VRXDSHBL_POS                 0
#define MXC_F_NFC_LC_VRXDSHBL1_VRXDSHBL                     ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_VRXDSHBL1_VRXDSHBL_POS))

#define MXC_F_NFC_LC_VTXPL0_VTXPL_POS                       0
#define MXC_F_NFC_LC_VTXPL0_VTXPL                           ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTXPL0_VTXPL_POS))

#define MXC_F_NFC_LC_VTXPL1_VTXPL_POS                       0
#define MXC_F_NFC_LC_VTXPL1_VTXPL                           ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTXPL1_VTXPL_POS))

#define MXC_F_NFC_LC_VTX38SOFHT0_VTX38SOFHT_POS             0
#define MXC_F_NFC_LC_VTX38SOFHT0_VTX38SOFHT                 ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX38SOFHT0_VTX38SOFHT_POS))

#define MXC_F_NFC_LC_VTX38SOFHT1_VTX38SOFHT_POS             0
#define MXC_F_NFC_LC_VTX38SOFHT1_VTX38SOFHT                 ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX38SOFHT1_VTX38SOFHT_POS))

#define MXC_F_NFC_LC_VTX18SOFT0_VTX18SOFT_POS               0
#define MXC_F_NFC_LC_VTX18SOFT0_VTX18SOFT                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX18SOFT0_VTX18SOFT_POS))

#define MXC_F_NFC_LC_VTX18SOFT1_VTX18SOFT_POS               0
#define MXC_F_NFC_LC_VTX18SOFT1_VTX18SOFT                   ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX18SOFT1_VTX18SOFT_POS))

#define MXC_F_NFC_LC_VTX14HT0_VTX14HT_POS                   0
#define MXC_F_NFC_LC_VTX14HT0_VTX14HT                       ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX14HT0_VTX14HT_POS))

#define MXC_F_NFC_LC_VTX14HT1_VTX14HT_POS                   0
#define MXC_F_NFC_LC_VTX14HT1_VTX14HT                       ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX14HT1_VTX14HT_POS))

#define MXC_F_NFC_LC_VTX18P0_VTX18P_POS                     0
#define MXC_F_NFC_LC_VTX18P0_VTX18P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX18P0_VTX18P_POS))

#define MXC_F_NFC_LC_VTX18P1_VTX18P_POS                     0
#define MXC_F_NFC_LC_VTX18P1_VTX18P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX18P1_VTX18P_POS))

#define MXC_F_NFC_LC_VTX38P0_VTX38P_POS                     0
#define MXC_F_NFC_LC_VTX38P0_VTX38P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX38P0_VTX38P_POS))

#define MXC_F_NFC_LC_VTX38P1_VTX38P_POS                     0
#define MXC_F_NFC_LC_VTX38P1_VTX38P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX38P1_VTX38P_POS))

#define MXC_F_NFC_LC_VTX48P0_VTX48P_POS                     0
#define MXC_F_NFC_LC_VTX48P0_VTX48P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX48P0_VTX48P_POS))

#define MXC_F_NFC_LC_VTX48P1_VTX48P_POS                     0
#define MXC_F_NFC_LC_VTX48P1_VTX48P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX48P1_VTX48P_POS))

#define MXC_F_NFC_LC_VTX58P0_VTX58P_POS                     0
#define MXC_F_NFC_LC_VTX58P0_VTX58P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX58P0_VTX58P_POS))

#define MXC_F_NFC_LC_VTX58P1_VTX58P_POS                     0
#define MXC_F_NFC_LC_VTX58P1_VTX58P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX58P1_VTX58P_POS))

#define MXC_F_NFC_LC_VTX68P0_VTX68P_POS                     0
#define MXC_F_NFC_LC_VTX68P0_VTX68P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX68P0_VTX68P_POS))

#define MXC_F_NFC_LC_VTX68P1_VTX68P_POS                     0
#define MXC_F_NFC_LC_VTX68P1_VTX68P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX68P1_VTX68P_POS))

#define MXC_F_NFC_LC_VTX78P0_VTX78P_POS                     0
#define MXC_F_NFC_LC_VTX78P0_VTX78P                         ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VTX78P0_VTX78P_POS))

#define MXC_F_NFC_LC_VTX78P1_VTX78P_POS                     0
#define MXC_F_NFC_LC_VTX78P1_VTX78P                         ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_VTX78P1_VTX78P_POS))

#define MXC_F_NFC_LC_NFC_RXSOSL_NFCRXSOSL_POS               0
#define MXC_F_NFC_LC_NFC_RXSOSL_NFCRXSOSL                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXSOSL_NFCRXSOSL_POS))

#define MXC_F_NFC_LC_NFC_RXPW_NFCRXPW_POS                   0
#define MXC_F_NFC_LC_NFC_RXPW_NFCRXPW                       ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXPW_NFCRXPW_POS))

#define MXC_F_NFC_LC_NFC_RXPWT_NFCRXPWT_POS                 0
#define MXC_F_NFC_LC_NFC_RXPWT_NFCRXPWT                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXPWT_NFCRXPWT_POS))

#define MXC_F_NFC_LC_NFC_RXEOFL_NFCRXEOFL_POS               0
#define MXC_F_NFC_LC_NFC_RXEOFL_NFCRXEOFL                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_NFC_RXEOFL_NFCRXEOFL_POS))

#define MXC_F_NFC_LC_RXACDMPT0_RXACDMPT106_POS              0
#define MXC_F_NFC_LC_RXACDMPT0_RXACDMPT106                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RXACDMPT0_RXACDMPT106_POS))
#define MXC_F_NFC_LC_RXACDMPT0_RXACDMPT212_POS              4
#define MXC_F_NFC_LC_RXACDMPT0_RXACDMPT212                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RXACDMPT0_RXACDMPT212_POS))

#define MXC_F_NFC_LC_RXACDMPT1_RXACDMPT424_POS              0
#define MXC_F_NFC_LC_RXACDMPT1_RXACDMPT424                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RXACDMPT1_RXACDMPT424_POS))
#define MXC_F_NFC_LC_RXACDMPT1_RXACDMPT848_POS              4
#define MXC_F_NFC_LC_RXACDMPT1_RXACDMPT848                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_LC_RXACDMPT1_RXACDMPT848_POS))

#define MXC_F_NFC_LC_VRXCDPW_VRXCDPW_POS                    0
#define MXC_F_NFC_LC_VRXCDPW_VRXCDPW                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_VRXCDPW_VRXCDPW_POS))

#define MXC_F_NFC_DPP_CMPSEL_CMP_SEL_EN_POS                 0
#define MXC_F_NFC_DPP_CMPSEL_CMP_SEL_EN                     ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_CMPSEL_CMP_SEL_EN_POS))
#define MXC_F_NFC_DPP_CMPSEL_CMP_SEL_VAL_POS                1
#define MXC_F_NFC_DPP_CMPSEL_CMP_SEL_VAL                    ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_CMPSEL_CMP_SEL_VAL_POS))

#define MXC_F_NFC_DPP_VICC_CNTL0_VICCME_POS                 0
#define MXC_F_NFC_DPP_VICC_CNTL0_VICCME                     ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_VICC_CNTL0_VICCME_POS))
#define MXC_F_NFC_DPP_VICC_CNTL0_VICEBTHR_POS               1
#define MXC_F_NFC_DPP_VICC_CNTL0_VICEBTHR                   ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_VICC_CNTL0_VICEBTHR_POS))
#define MXC_F_NFC_DPP_VICC_CNTL0_VICEBITHR_POS              5
#define MXC_F_NFC_DPP_VICC_CNTL0_VICEBITHR                  ((uint32_t)(0x00000007UL << MXC_F_NFC_DPP_VICC_CNTL0_VICEBITHR_POS))

#define MXC_F_NFC_DPP_VICC_CNTL1_VICEBITHR_POS              0
#define MXC_F_NFC_DPP_VICC_CNTL1_VICEBITHR                  ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_VICC_CNTL1_VICEBITHR_POS))
#define MXC_F_NFC_DPP_VICC_CNTL1_VICEBSTHR_POS              1
#define MXC_F_NFC_DPP_VICC_CNTL1_VICEBSTHR                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_VICC_CNTL1_VICEBSTHR_POS))
#define MXC_F_NFC_DPP_VICC_CNTL1_VICDSELDIS_POS             5
#define MXC_F_NFC_DPP_VICC_CNTL1_VICDSELDIS                 ((uint32_t)(0x00000001UL << MXC_F_NFC_DPP_VICC_CNTL1_VICDSELDIS_POS))
#define MXC_F_NFC_DPP_VICC_CNTL1_VICDSELOVR_POS             6
#define MXC_F_NFC_DPP_VICC_CNTL1_VICDSELOVR                 ((uint32_t)(0x00000003UL << MXC_F_NFC_DPP_VICC_CNTL1_VICDSELOVR_POS))

#define MXC_F_NFC_DPP_VICC_CNTL2_VICINVCAR_POS              0
#define MXC_F_NFC_DPP_VICC_CNTL2_VICINVCAR                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_VICC_CNTL2_VICINVCAR_POS))

#define MXC_F_NFC_LC_ERRORHIS0_ERRORHIS_POS                 0
#define MXC_F_NFC_LC_ERRORHIS0_ERRORHIS                     ((uint32_t)(0x000000FFUL << MXC_F_NFC_LC_ERRORHIS0_ERRORHIS_POS))

#define MXC_F_NFC_LC_ERRORHIS1_ERRORHIS_POS                 0
#define MXC_F_NFC_LC_ERRORHIS1_ERRORHIS                     ((uint32_t)(0x0000003FUL << MXC_F_NFC_LC_ERRORHIS1_ERRORHIS_POS))

#define MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGDLY_POS              0
#define MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGDLY                  ((uint32_t)(0x0000007FUL << MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGDLY_POS))
#define MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGAOS_POS              7
#define MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGAOS                  ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_CDTXSTGDLY_CDTXSTGAOS_POS))

#define MXC_F_NFC_HW_TMR_CNTL_HTRESTART_POS                 0
#define MXC_F_NFC_HW_TMR_CNTL_HTRESTART                     ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_HTRESTART_POS))
#define MXC_F_NFC_HW_TMR_CNTL_HTSTOP_POS                    1
#define MXC_F_NFC_HW_TMR_CNTL_HTSTOP                        ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_HTSTOP_POS))
#define MXC_F_NFC_HW_TMR_CNTL_HTSTART_POS                   2
#define MXC_F_NFC_HW_TMR_CNTL_HTSTART                       ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_HTSTART_POS))
#define MXC_F_NFC_HW_TMR_CNTL_HTEN_POS                      3
#define MXC_F_NFC_HW_TMR_CNTL_HTEN                          ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_HTEN_POS))
#define MXC_F_NFC_HW_TMR_CNTL_TXENDCAPE_POS                 4
#define MXC_F_NFC_HW_TMR_CNTL_TXENDCAPE                     ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_TXENDCAPE_POS))
#define MXC_F_NFC_HW_TMR_CNTL_TXSTARTCAPE_POS               5
#define MXC_F_NFC_HW_TMR_CNTL_TXSTARTCAPE                   ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_TXSTARTCAPE_POS))
#define MXC_F_NFC_HW_TMR_CNTL_RXENDCAPE_POS                 6
#define MXC_F_NFC_HW_TMR_CNTL_RXENDCAPE                     ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_RXENDCAPE_POS))
#define MXC_F_NFC_HW_TMR_CNTL_RXSTARTCAPE_POS               7
#define MXC_F_NFC_HW_TMR_CNTL_RXSTARTCAPE                   ((uint32_t)(0x00000001UL << MXC_F_NFC_HW_TMR_CNTL_RXSTARTCAPE_POS))

#define MXC_F_NFC_HW_TMRPSCAL_TMRPSCAL_POS                  0
#define MXC_F_NFC_HW_TMRPSCAL_TMRPSCAL                      ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRPSCAL_TMRPSCAL_POS))

#define MXC_F_NFC_HW_TMRVAL0_HTMRVAL_POS                    0
#define MXC_F_NFC_HW_TMRVAL0_HTMRVAL                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRVAL0_HTMRVAL_POS))

#define MXC_F_NFC_HW_TMRVAL1_HTMRVAL_POS                    0
#define MXC_F_NFC_HW_TMRVAL1_HTMRVAL                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRVAL1_HTMRVAL_POS))

#define MXC_F_NFC_HW_TMRVAL2_HTMRVAL_POS                    0
#define MXC_F_NFC_HW_TMRVAL2_HTMRVAL                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRVAL2_HTMRVAL_POS))

#define MXC_F_NFC_HW_TMRVAL3_HTMRVAL_POS                    0
#define MXC_F_NFC_HW_TMRVAL3_HTMRVAL                        ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRVAL3_HTMRVAL_POS))

#define MXC_F_NFC_HW_TMRRXSTP0_HTMRRXSTP_POS                0
#define MXC_F_NFC_HW_TMRRXSTP0_HTMRRXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRRXSTP0_HTMRRXSTP_POS))

#define MXC_F_NFC_HW_TMRRXSTP1_HTMRRXSTP_POS                0
#define MXC_F_NFC_HW_TMRRXSTP1_HTMRRXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRRXSTP1_HTMRRXSTP_POS))

#define MXC_F_NFC_HW_TMRRXSTP2_HTMRRXSTP_POS                0
#define MXC_F_NFC_HW_TMRRXSTP2_HTMRRXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRRXSTP2_HTMRRXSTP_POS))

#define MXC_F_NFC_HW_TMRRXSTP3_HTMRRXSTP_POS                0
#define MXC_F_NFC_HW_TMRRXSTP3_HTMRRXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRRXSTP3_HTMRRXSTP_POS))

#define MXC_F_NFC_HW_TMRTXSTP0_HTMRTXSTP_POS                0
#define MXC_F_NFC_HW_TMRTXSTP0_HTMRTXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRTXSTP0_HTMRTXSTP_POS))

#define MXC_F_NFC_HW_TMRTXSTP1_HTMRTXSTP_POS                0
#define MXC_F_NFC_HW_TMRTXSTP1_HTMRTXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRTXSTP1_HTMRTXSTP_POS))

#define MXC_F_NFC_HW_TMRTXSTP2_HTMRTXSTP_POS                0
#define MXC_F_NFC_HW_TMRTXSTP2_HTMRTXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRTXSTP2_HTMRTXSTP_POS))

#define MXC_F_NFC_HW_TMRTXSTP3_HTMRTXSTP_POS                0
#define MXC_F_NFC_HW_TMRTXSTP3_HTMRTXSTP                    ((uint32_t)(0x000000FFUL << MXC_F_NFC_HW_TMRTXSTP3_HTMRTXSTP_POS))

#define MXC_F_NFC_LC_NFC_AUTO_DET_SETTING_DONE_IRQ_POS      0
#define MXC_F_NFC_LC_NFC_AUTO_DET_SETTING_DONE_IRQ          ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_NFC_AUTO_DET_SETTING_DONE_IRQ_POS))
#define MXC_F_NFC_LC_NFC_AUTO_DET_NFC_TYPE_POS              2
#define MXC_F_NFC_LC_NFC_AUTO_DET_NFC_TYPE                  ((uint32_t)(0x00000003UL << MXC_F_NFC_LC_NFC_AUTO_DET_NFC_TYPE_POS))
#define MXC_F_NFC_LC_NFC_AUTO_DET_NFC_A_F_MODE_POS          4
#define MXC_F_NFC_LC_NFC_AUTO_DET_NFC_A_F_MODE              ((uint32_t)(0x00000007UL << MXC_F_NFC_LC_NFC_AUTO_DET_NFC_A_F_MODE_POS))
#define MXC_F_NFC_LC_NFC_AUTO_DET_DATA_RATE_POS             7
#define MXC_F_NFC_LC_NFC_AUTO_DET_DATA_RATE                 ((uint32_t)(0x00000001UL << MXC_F_NFC_LC_NFC_AUTO_DET_DATA_RATE_POS))

#define MXC_F_NFC_DPP_NFCRXADLY_NFCRXADLY_POS               0
#define MXC_F_NFC_DPP_NFCRXADLY_NFCRXADLY                   ((uint32_t)(0x0000000FUL << MXC_F_NFC_DPP_NFCRXADLY_NFCRXADLY_POS))

#define MXC_F_NFC_REG_RESET_REGRST_POS                      0
#define MXC_F_NFC_REG_RESET_REGRST                          ((uint32_t)(0x00000001UL << MXC_F_NFC_REG_RESET_REGRST_POS))

/*
   Field values and shifted values for module NFC.
*/

#define MXC_V_NFC_DPP_CNTL2_DPPPOL0_NO_INVERSION                                ((uint32_t)(0x00000000UL))
#define MXC_V_NFC_DPP_CNTL2_DPPPOL0_INVERT_ORDER                                ((uint32_t)(0x00000001UL))

#define MXC_S_NFC_DPP_CNTL2_DPPPOL0_NO_INVERSION                                ((uint8_t)(MXC_V_NFC_DPP_CNTL2_DPPPOL0_NO_INVERSION   << MXC_F_NFC_DPP_CNTL2_DPPPOL0_POS))
#define MXC_S_NFC_DPP_CNTL2_DPPPOL0_INVERT_ORDER                                ((uint8_t)(MXC_V_NFC_DPP_CNTL2_DPPPOL0_INVERT_ORDER   << MXC_F_NFC_DPP_CNTL2_DPPPOL0_POS))


/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG1_Register
 * @brief    Maxim RX PHY Configuration 1 Register
 * @{
 */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_EN_POS                (0) /**< MXM_RX_PHY_CFG1_PHY_EN (PHY Enable) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_EN                    ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_EN_POS)) /**< MXM_RX_PHY_CFG1_PHY_EN Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_EN_DISABLED           ((uint32_t)(0x00000000UL)) /**< MXM_RX_PHY_CFG1_PHY_EN_DISABLED Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_EN_DISABLED           ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_EN_DISABLED << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_EN_POS)) /**< MXM_RX_PHY_CFG1_PHY_EN_DISABLED Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_EN_ENABLED            ((uint32_t)(0x00000001UL)) /**< MXM_RX_PHY_CFG1_PHY_EN_ENABLED Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_EN_ENABLED            ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_EN_ENABLED  << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_EN_POS)) /**< MXM_RX_PHY_CFG1_PHY_EN_ENABLED Setting */
                   
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_POS        (1) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N (PHY PCD/PICC_N Select) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N            ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_POS)) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_POS Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PICC       ((uint32_t)(0x00000000UL)) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PICC Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PICC       ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PICC << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_POS))) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PICC Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PCD        ((uint32_t)(0x00000001UL)) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PCD Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PCD        ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PCD  << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_POS)) /**< MXM_RX_PHY_CFG1_PHY_PCD_PICC_N_PCD Setting */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS           (2) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE (PHY RX Type) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE               ((uint32_t)(0x00000003UL << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_106         ((uint32_t)(0x00000000UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_106 Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_106         ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_106 << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_106 Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_212_424     ((uint32_t)(0x00000001UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_212_424 Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_212_424     ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_212_424 << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_A_212_424 Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_B             ((uint32_t)(0x00000001UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_B Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_B             ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_B << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_B Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_F             ((uint32_t)(0x00000002UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_F Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_F             ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_F << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_F Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_V             ((uint32_t)(0x00000003UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_V Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_V             ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_V << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_TYPE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_TYPE_V Setting */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS      (4) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE (PHY RX Data Rate) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE          ((uint32_t)(0x00000003UL << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_106      ((uint32_t)(0x00000000UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_106 Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_106      ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_106 << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_106 Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_212      ((uint32_t)(0x00000001UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_212 Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_212      ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_212 << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_212 Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_424      ((uint32_t)(0x00000002UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_F Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_424      ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_424 << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_424 Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_RFU      ((uint32_t)(0x00000003UL)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_V Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_RFU      ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_RFU << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_POS)) /**< MXM_RX_PHY_CFG1_PHY_RX_DATA_RATE_RFU Setting */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_POS           (6) /**< MXM_RX_PHY_CFG1_LOCK_BYPASS (Lock Bypass) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS               ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_POS)) /**< MXM_RX_PHY_CFG1_LOCK_BYPASS Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_ENABLE_LOCK   ((uint32_t)(0x00000000UL)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_ENABLE_LOCK Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_ENABLE_LOCK   ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_ENABLE_LOCK << MXC_F_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_POS)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_ENABLE_LOCK Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_BYPASS_LOCK   ((uint32_t)(0x00000001UL)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_BYPASS_LOCK Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_BYPASS_LOCK   ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_BYPASS_LOCK << MXC_F_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_POS)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_LOCK_BYPASS_BYPASS_LOCK Setting */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_POS             (7) /**< MXM_RX_PHY_CFG1_MXM_RX_EN (Maxim RX Enable) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN                 ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_POS)) /**< MXM_RX_PHY_CFG1_MXM_RX_EN Mask */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_DISABLE         ((uint32_t)(0x00000000UL)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_DISABLE Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_DISABLE         ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_DISABLE << MXC_F_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_POS)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_DISABLE Setting */
#define MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_ENABLE          ((uint32_t)(0x00000001UL)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_ENABLE Value */
#define MXC_S_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_ENABLE          ((uint32_t)(MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_ENABLE << MXC_F_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_POS)) /**< MXC_V_NFC_MXM_RX_PHY_CFG1_MXM_RX_EN_ENABLE Setting */

#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_ERR_MASK_POS          (8) /**< MXM_RX_PHY_CFG1_PHY_ERR_MASK (PHY Error Mask) Position */
#define MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_ERR_MASK              ((uint32_t)(0x000000FFUL << MXC_F_NFC_MXM_RX_PHY_CFG1_PHY_ERR_MASK_POS)) /**< MXM_RX_PHY_CFG1_PHY_ERR_MASK Mask */

/* TODO... */

/**@} end of group MXM_RX_PHY_CFG1_Register */


/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG2_Register
 * @brief    Maxim RX PHY Configuration 2 Register
 * @{
 */

/* TODO... */
 
/**@} end of group MXM_RX_PHY_CFG2_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG3_Register
 * @brief    Maxim RX PHY Configuration 3 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_PHY_CFG3_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG4_Register
 * @brief    Maxim RX PHY Configuration 4 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_PHY_CFG4_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG5_Register
 * @brief    Maxim RX PHY Configuration 5 Register
 * @{
 */

/* TODO... */
 
/**@} end of group MXM_RX_PHY_CFG5_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG6_Register
 * @brief    Maxim RX PHY Configuration 6 Register
 * @{
 */

/* TODO... */
 
/**@} end of group MXM_RX_PHY_CFG6_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG7_Register
 * @brief    Maxim RX PHY Configuration 7 Register
 * @{
 */

/* TODO... */
 
/**@} end of group MXM_RX_PHY_CFG7_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_CFG8_Register
 * @brief    Maxim RX PHY Configuration 8 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_PHY_CFG8_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_STATUS1_Register
 * @brief    Maxim RX PHY Status 1 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_PHY_STATUS1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_PHY_STATUS2_Register
 * @brief    Maxim RX PHY Status 2 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_PHY_STATUS2_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_LNK_CFG1_Register
 * @brief    Maxim RX LNK Configuration 1 Register
 * @{
 */

/* TODO... */

/**@} end of group MXM_RX_LNK_CFG1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_LNK_CFG2_Register
 * @brief    Maxim RX LNK Configuration 2 Register
 * @{
 */

/* TODO... */
 
/**@} end of group MXM_RX_LNK_CFG2_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_RX_LNK_STATUS1_Register
 * @brief    Maxim RX Link Status 1 Register
 * @{
 */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_POS           (0) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE (RX Complete) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE               ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_POS)) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE Mask */
#define MXC_V_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_INCOMPLETE    ((uint32_t)(0x00000000UL)) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE_INCOMPLETE Value */
#define MXC_S_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_INCOMPLETE    ((uint32_t)(MXC_V_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_INCOMPLETE << MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_POS)) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE_INCOMPLETE Setting */
#define MXC_V_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_COMPLETE      ((uint32_t)(0x00000001UL)) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE_COMPLETE Value */
#define MXC_S_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_COMPLETE      ((uint32_t)(MXC_V_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_COMPLETE  << MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_COMPLETE_POS)) /**< MXM_RX_LNK_STATUS1_RX_COMPLETE_COMPLETE Setting */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_POS           (1) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY (Link RX Busy) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY               ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_POS)) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY_POS Mask */
#define MXC_V_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_IDLE          ((uint32_t)(0x00000000UL)) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY_IDLE Value */
#define MXC_S_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_IDLE          ((uint32_t)(MXC_V_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_IDLE << MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_POS))) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY_IDLE Setting */
#define MXC_V_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_BUSY          ((uint32_t)(0x00000001UL)) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY_BUSY Value */
#define MXC_S_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_BUSY          ((uint32_t)(MXC_V_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_BUSY  << MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_RX_BUSY_POS)) /**< MXM_RX_LNK_STATUS1_LNK_RX_BUSY_BUSY Setting */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_SOF_RX_POS                (2) /**< MXM_RX_LNK_STATUS1_SOF_RX (Start of Frame Received) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_SOF_RX                    ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_SOF_RX_POS)) /**< MXM_RX_LNK_STATUS1_SOF_RX_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_EOF_RX_POS                (3) /**< MXM_RX_LNK_STATUS1_EOF_RX (End of Frame Received) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_EOF_RX                    ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_EOF_RX_POS)) /**< MXM_RX_LNK_STATUS1_EOF_RX_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_CNT_FIFO_BIT_POS       (4) /**< MXM_RX_LNK_STATUS1_RX_CNT_FIFO_BIT (Count of Received FIFO Bits) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_CNT_FIFO_BIT           ((uint32_t)(0x0000000FUL << MXC_F_NFC_MXM_RX_LNK_STATUS1_RX_CNT_FIFO_BIT_POS)) /**< MXM_RX_LNK_STATUS1_RX_CNT_FIFO_BIT_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_ERR_POS               (8) /**< MXM_RX_LNK_STATUS1_LNK_ERR (Link Errors) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_ERR                   ((uint32_t)(0x000000FFUL << MXC_F_NFC_MXM_RX_LNK_STATUS1_LNK_ERR_POS)) /**< MXM_RX_LNK_STATUS1_LNK_ERR_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS1_POS   (16) /**< MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS1 (tPICC,S,1 Count in Subcarrier Cycles) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS1       ((uint32_t)(0x0000007FUL << MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS1_POS)) /**< MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS1_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS2_POS   (24) /**< MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS2 (tPICC,S,2 Count in Subcarrier Cycles) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS2       ((uint32_t)(0x0000001FUL << MXC_F_NFC_MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS2_POS)) /**< MXM_RX_LNK_STATUS1_CNT_SUBCARR_TPICCS2_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC0_POS            (30) /**< MXM_RX_LNK_STATUS1_VCD_LOGIC0 (VCD Logic 0) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC0                ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC0_POS)) /**< MXM_RX_LNK_STATUS1_VCD_LOGIC0_POS Mask */

#define MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC1_POS            (31) /**< MXM_RX_LNK_STATUS1_VCD_LOGIC1 (VCD Logic 1) Position */
#define MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC1                ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_RX_LNK_STATUS1_VCD_LOGIC1_POS)) /**< MXM_RX_LNK_STATUS1_VCD_LOGIC1_POS Mask */

 /**@} end of group MXM_RX_LNK_STATUS1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TX_CFG1_Register
 * @brief    Maxim TX Configuration 1 Register
 * @{
 */

#define MXC_F_NFC_MXM_TX_CFG1_TX_EN_POS                        (0) /**< MXM_TX_CFG1_TX_EN (Transmitter Enable) Position */
#define MXC_F_NFC_MXM_TX_CFG1_TX_EN                            ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_TX_CFG1_TX_EN_POS)) /**< MXM_TX_CFG1_TX_EN_POS Mask */

#define MXC_F_NFC_MXM_TX_CFG1_TX_VCD_VICC_N_POS                (1) /**< MXM_TX_CFG1_TX_VCD_VICC_N (Transmitter VCD/VICC_N Select) Position */
#define MXC_F_NFC_MXM_TX_CFG1_TX_VCD_VICC_N                    ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_TX_CFG1_TX_VCD_VICC_N_POS)) /**< MXM_TX_CFG1_TX_VCD_VICC_N_POS Mask */

#define MXC_F_NFC_MXM_TX_CFG1_TX_VIC_EN_POS                    (2) /**< MXM_TX_CFG1_TX_VIC_EN (Vicinity Transmitter Enable) Position */
#define MXC_F_NFC_MXM_TX_CFG1_TX_VIC_EN                        ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_TX_CFG1_TX_VIC_EN_POS)) /**< MXM_TX_CFG1_TX_VIC_EN_POS Mask */

#define MXC_F_NFC_MXM_TX_CFG1_TX_DIR_SEL_POS                   (3) /**< MXM_TX_CFG1_TX_DIR_SEL (Transmitter Direction Select) Position */
#define MXC_F_NFC_MXM_TX_CFG1_TX_DIR_SEL                       ((uint32_t)(0x00000001UL << MXC_F_NFC_MXM_TX_CFG1_TX_DIR_SEL_POS)) /**< MXM_TX_CFG1_TX_DIR_SEL_POS Mask */
#define MXC_V_NFC_MXM_TX_CFG1_TX_DIR_SEL_LSB_FIRST             ((uint32_t)(0x00000000UL)) /**< MXM_TX_CFG1_TX_DIR_SEL_LSB_FIRST Value */
#define MXC_S_NFC_MXM_TX_CFG1_TX_DIR_SEL_LSB_FIRST             ((uint32_t)(MXC_V_NFC_MXM_TX_CFG1_TX_DIR_SEL_LSB_FIRST << MXC_F_NFC_MXM_TX_CFG1_TX_DIR_SEL_POS))) /**< MXM_TX_CFG1_TX_DIR_SEL_LSB_FIRST Setting */
#define MXC_V_NFC_MXM_TX_CFG1_TX_DIR_SEL_MSB_FIRST             ((uint32_t)(0x00000001UL)) /**< MXM_TX_CFG1_TX_DIR_SEL_MSB_FIRST Value */
#define MXC_S_NFC_MXM_TX_CFG1_TX_DIR_SEL_MSB_FIRST             ((uint32_t)(MXC_V_NFC_MXM_TX_CFG1_TX_DIR_SEL_MSB_FIRST  << MXC_F_NFC_MXM_TX_CFG1_TX_DIR_SEL_POS)) /**< MXM_TX_CFG1_TX_DIR_SEL_MSB_FIRST Setting */

/* TODO bits 11, 10, 9, 8, 7, 6, and 5:4 */

#define MXC_F_NFC_MXM_TX_CFG1_TX_CNT_FIFO_BIT_POS              (12) /**< MXM_TX_CFG1_TX_CNT_FIFO_BIT (Count of FIFO Bits to Transmit) Position */
#define MXC_F_NFC_MXM_TX_CFG1_TX_CNT_FIFO_BIT                  ((uint32_t)(0x0000000FUL << MXC_F_NFC_MXM_TX_CFG1_TX_CNT_FIFO_BIT_POS)) /**< MXM_TX_CFG1_TX_CNT_FIFO_BIT_POS Mask */
 
/**@} end of group MXM_TX_CFG1_Register */


/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG1_Register
 * @brief    Maxim Timer Configuration 1 Register
 * @{
 */
 
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_EN_POS                (0) /**< MXM_TMR_CFG1_TMR_A_EN (Timer A Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_EN                    ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_EN_POS)) /**< MXM_TMR_CFG1_TMR_A_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_CLR_POS               (1) /**< MXM_TMR_CFG1_TMR_A_CLR (Timer A Clear) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_CLR                   ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_CLR_POS)) /**< MXM_TMR_CFG1_TMR_A_CLR Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_ROLL_EN_POS           (2) /**< MXM_TMR_CFG1_TMR_A_ROLL_EN (Timer A Roll Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_ROLL_EN               ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_ROLL_EN_POS)) /**< MXM_TMR_CFG1_TMR_A_ROLL_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_RETRIG_EN_POS         (3) /**< MXM_TMR_CFG1_TMR_A_RETRIG_EN (Timer A Retrigger Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_RETRIG_EN             ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_RETRIG_EN_POS)) /**< MXM_TMR_CFG1_TMR_A_RETRIG_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_IRQ_EN_POS            (4) /**< MXM_TMR_CFG1_TMR_A_IRQ_EN (Timer A Interrupt Request Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_IRQ_EN                ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_IRQ_EN_POS)) /**< MXM_TMR_CFG1_TMR_A_IRQ_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TX_GATE_EN_POS        (5) /**< MXM_TMR_CFG1_TMR_A_TX_GATE_EN (Timer A Transmit Gate Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TX_GATE_EN            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TX_GATE_EN_POS)) /**< MXM_TMR_CFG1_TMR_A_TX_GATE_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_OPC_BYPASS_POS        (6) /**< MXM_TMR_CFG1_TMR_A_OPC_BYPASS (Timer A Operation Complete Bypass) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_OPC_BYPASS            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_OPC_BYPASS_POS)) /**< MXM_TMR_CFG1_TMR_A_OPC_BYPASS Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_MXM_TMR_EN_POS              (7) /**< MXM_TMR_CFG1_MXM_TMR_EN (Maxim Timer Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_MXM_TMR_EN                  ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG1_MXM_TMR_EN_POS)) /**< MXM_TMR_CFG1_MXM_TMR_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS          (8) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL (Timer A Trigger Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL              ((uint32_t)(0x7 << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS)) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL Mask */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_NONE         ((uint32_t)0x0) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_NONE Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_NONE         (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_NONE << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_NONE Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RFU          ((uint32_t)0x1) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RFU Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RFU          (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RFU << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RFU Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_COMPLETE  ((uint32_t)0x2) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_COMPLETE Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_MOD_CHG   ((uint32_t)0x3) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_TX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_PHASE_CHG ((uint32_t)0x4) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_PHASE_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_PHASE_CHG (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_PHASE_CHG << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_PHASE_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_ETU_END   ((uint32_t)0x5) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_ETU_END Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_ETU_END   (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_ETU_END << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_ETU_END Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_MOD_CHG   ((uint32_t)0x6) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_COMPLETE  ((uint32_t)0x7) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TRIG_SEL_POS) /**< MXM_TMR_CFG1_TMR_A_TRIG_SEL_RX_COMPLETE Setting */

#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TARG_POS              (16) /**< MXM_TMR_CFG1_TMR_A_TARG (Timer A Target) Position */
#define MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TARG_SEL              ((uint32_t)(0xFFFF << MXC_F_NFC_MXM_TMR_CFG1_TMR_A_TARG_POS)) /**< MXM_TMR_CFG1_TMR_A_TARG Mask */

/**@} end of group MXM_TMR_CFG1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG2_Register
 * @brief    Maxim Timer Configuration 2 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_EN_POS                (0) /**< MXM_TMR_CFG2_TMR_B_EN (Timer B Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_EN                    ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_EN_POS)) /**< MXM_TMR_CFG2_TMR_B_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_CLR_POS               (1) /**< MXM_TMR_CFG2_TMR_B_CLR (Timer B Clear) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_CLR                   ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_CLR_POS)) /**< MXM_TMR_CFG2_TMR_B_CLR Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_ROLL_EN_POS           (2) /**< MXM_TMR_CFG2_TMR_B_ROLL_EN (Timer B Roll Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_ROLL_EN               ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_ROLL_EN_POS)) /**< MXM_TMR_CFG2_TMR_B_ROLL_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_RETRIG_EN_POS         (3) /**< MXM_TMR_CFG2_TMR_B_RETRIG_EN (Timer B Retrigger Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_RETRIG_EN             ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_RETRIG_EN_POS)) /**< MXM_TMR_CFG2_TMR_B_RETRIG_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_IRQ_EN_POS            (4) /**< MXM_TMR_CFG2_TMR_B_IRQ_EN (Timer B Interrupt Request Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_IRQ_EN                ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_IRQ_EN_POS)) /**< MXM_TMR_CFG2_TMR_B_IRQ_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TX_GATE_EN_POS        (5) /**< MXM_TMR_CFG2_TMR_B_TX_GATE_EN (Timer B Transmit Gate Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TX_GATE_EN            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TX_GATE_EN_POS)) /**< MXM_TMR_CFG2_TMR_B_TX_GATE_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_OPC_BYPASS_POS        (6) /**< MXM_TMR_CFG2_TMR_B_OPC_BYPASS (Timer B Operation Complete Bypass) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_OPC_BYPASS            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_OPC_BYPASS_POS)) /**< MXM_TMR_CFG2_TMR_B_OPC_BYPASS Mask */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS          (8) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL (Timer B Trigger Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL              ((uint32_t)(0x7 << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS)) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL Mask */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_NONE         ((uint32_t)0x0) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_NONE Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_NONE         (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_NONE << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_NONE Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RFU          ((uint32_t)0x1) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RFU Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RFU          (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RFU << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RFU Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_COMPLETE  ((uint32_t)0x2) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_COMPLETE Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_MOD_CHG   ((uint32_t)0x3) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_TX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_PHASE_CHG ((uint32_t)0x4) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_PHASE_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_PHASE_CHG (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_PHASE_CHG << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_PHASE_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_ETU_END   ((uint32_t)0x5) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_ETU_END Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_ETU_END   (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_ETU_END << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_ETU_END Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_MOD_CHG   ((uint32_t)0x6) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_COMPLETE  ((uint32_t)0x7) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TRIG_SEL_POS) /**< MXM_TMR_CFG2_TMR_B_TRIG_SEL_RX_COMPLETE Setting */

#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TARG_POS              (16) /**< MXM_TMR_CFG2_TMR_B_TARG (Timer B Target) Position */
#define MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TARG_SEL              ((uint32_t)(0xFFFF << MXC_F_NFC_MXM_TMR_CFG2_TMR_B_TARG_POS)) /**< MXM_TMR_CFG2_TMR_B_TARG Mask */

/**@} end of group MXM_TMR_CFG1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG3_Register
 * @brief    Maxim Timer Configuration 3 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_EN_POS                (0) /**< MXM_TMR_CFG3_TMR_C_EN (Timer C Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_EN                    ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_EN_POS)) /**< MXM_TMR_CFG3_TMR_C_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_CLR_POS               (1) /**< MXM_TMR_CFG3_TMR_C_CLR (Timer C Clear) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_CLR                   ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_CLR_POS)) /**< MXM_TMR_CFG3_TMR_C_CLR Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_ROLL_EN_POS           (2) /**< MXM_TMR_CFG3_TMR_C_ROLL_EN (Timer C Roll Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_ROLL_EN               ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_ROLL_EN_POS)) /**< MXM_TMR_CFG3_TMR_C_ROLL_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_RETRIG_EN_POS         (3) /**< MXM_TMR_CFG3_TMR_C_RETRIG_EN (Timer C Retrigger Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_RETRIG_EN             ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_RETRIG_EN_POS)) /**< MXM_TMR_CFG3_TMR_C_RETRIG_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_IRQ_EN_POS            (4) /**< MXM_TMR_CFG3_TMR_C_IRQ_EN (Timer C Interrupt Request Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_IRQ_EN                ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_IRQ_EN_POS)) /**< MXM_TMR_CFG3_TMR_C_IRQ_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_OPC_BYPASS_POS        (6) /**< MXM_TMR_CFG3_TMR_C_OPC_BYPASS (Timer C Operation Complete Bypass) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_OPC_BYPASS            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_OPC_BYPASS_POS)) /**< MXM_TMR_CFG3_TMR_C_OPC_BYPASS Mask */

#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS          (8) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL (Timer C Trigger Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL              ((uint32_t)(0x7 << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS)) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL Mask */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_NONE         ((uint32_t)0x0) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_NONE Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_NONE         (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_NONE << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_NONE Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RFU          ((uint32_t)0x1) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RFU Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RFU          (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RFU << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RFU Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_COMPLETE  ((uint32_t)0x2) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_COMPLETE Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_MOD_CHG   ((uint32_t)0x3) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_TX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_PHASE_CHG ((uint32_t)0x4) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_PHASE_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_PHASE_CHG (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_PHASE_CHG << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_PHASE_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_ETU_END   ((uint32_t)0x5) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_ETU_END Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_ETU_END   (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_ETU_END << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_ETU_END Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_MOD_CHG   ((uint32_t)0x6) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_COMPLETE  ((uint32_t)0x7) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG3_TMR_C_TRIG_SEL_POS) /**< MXM_TMR_CFG3_TMR_C_TRIG_SEL_RX_COMPLETE Setting */

/**@} end of group MXM_TMR_CFG3_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG4_Register
 * @brief    Maxim Timer Configuration 4 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_CFG4_TMR_C_TARG_POS              (0) /**< MXM_TMR_CFG4_TMR_C_TARG (Timer C Target) Position */
#define MXC_F_NFC_MXM_TMR_CFG4_TMR_C_TARG_SEL              ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_CFG4_TMR_C_TARG_POS)) /**< MXM_TMR_CFG4_TMR_C_TARG Mask */

/**@} end of group MXM_TMR_CFG4_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG5_Register
 * @brief    Maxim Timer Configuration 5 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_EN_POS                (0) /**< MXM_TMR_CFG5_TMR_D_EN (Timer D Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_EN                    ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_EN_POS)) /**< MXM_TMR_CFG5_TMR_D_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_CLR_POS               (1) /**< MXM_TMR_CFG5_TMR_D_CLR (Timer D Clear) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_CLR                   ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_CLR_POS)) /**< MXM_TMR_CFG5_TMR_D_CLR Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_ROLL_EN_POS           (2) /**< MXM_TMR_CFG5_TMR_D_ROLL_EN (Timer D Roll Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_ROLL_EN               ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_ROLL_EN_POS)) /**< MXM_TMR_CFG5_TMR_D_ROLL_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_RETRIG_EN_POS         (3) /**< MXM_TMR_CFG5_TMR_D_RETRIG_EN (Timer D Retrigger Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_RETRIG_EN             ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_RETRIG_EN_POS)) /**< MXM_TMR_CFG5_TMR_D_RETRIG_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_IRQ_EN_POS            (4) /**< MXM_TMR_CFG5_TMR_D_IRQ_EN (Timer D Interrupt Request Enable) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_IRQ_EN                ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_IRQ_EN_POS)) /**< MXM_TMR_CFG5_TMR_D_IRQ_EN Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_OPC_BYPASS_POS        (6) /**< MXM_TMR_CFG5_TMR_D_OPC_BYPASS (Timer D Operation Complete Bypass) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_OPC_BYPASS            ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_OPC_BYPASS_POS)) /**< MXM_TMR_CFG5_TMR_D_OPC_BYPASS Mask */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS          (8) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL (Timer D Trigger Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL              ((uint32_t)(0x7 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS)) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL Mask */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_NONE         ((uint32_t)0x0) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_NONE Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_NONE         (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_NONE << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_NONE Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RFU          ((uint32_t)0x1) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RFU Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RFU          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RFU << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RFU Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_COMPLETE  ((uint32_t)0x2) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_COMPLETE Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_MOD_CHG   ((uint32_t)0x3) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_TX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_PHASE_CHG ((uint32_t)0x4) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_PHASE_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_PHASE_CHG (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_PHASE_CHG << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_PHASE_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_ETU_END   ((uint32_t)0x5) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_ETU_END Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_ETU_END   (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_ETU_END << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_ETU_END Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_MOD_CHG   ((uint32_t)0x6) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_MOD_CHG Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_MOD_CHG   (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_MOD_CHG << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_MOD_CHG Setting */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_COMPLETE  ((uint32_t)0x7) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_COMPLETE Value */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_COMPLETE  (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_COMPLETE << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TRIG_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TRIG_SEL_RX_COMPLETE Setting */

#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS                      ((uint32_t)0x0) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0                     ((uint32_t)0x1) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1                     ((uint32_t)0x2) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG               ((uint32_t)0x3) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0                 ((uint32_t)0x4) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1                 ((uint32_t)0x5) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 ((uint32_t)0x6) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 ((uint32_t)0x7) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0              ((uint32_t)0x8) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1              ((uint32_t)0x9) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1 Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_PHASE_CHG      ((uint32_t)0xA) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_TYPE_B_PHASE_CHG Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F          ((uint32_t)0xB) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E          ((uint32_t)0xC) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D          ((uint32_t)0xD) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END               ((uint32_t)0xE) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END Value */
#define MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG               ((uint32_t)0xF) /**< MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG Value */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS                      (16) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL (Timer D Timestamp 0 Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL                          ((uint32_t)(0xF << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS)) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL Mask */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_DIS                      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS                      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_DIS                      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RFU0                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RFU0                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RFU1                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RFU1                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_TX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_TX_MOD_CHG               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_VCD_0                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_VCD_0                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_VCD_1                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_VCD_1                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_0_OR_VCD_SUB28 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_0_OR_VCD_SUB28 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_1_OR_VCD_SUB32 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_1_OR_VCD_SUB32 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_FELICA_0              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_FELICA_0              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_FELICA_1              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_FELICA_1              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_PHASE_CHG      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_PHASE_CHG      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_B_PHASE_CHG      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_F          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_F          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_E          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_E          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_D          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_TYPE_A_SEQ_D          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_ETU_END               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_ETU_END               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS0_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS0_SEL_RX_MOD_CHG               Setting */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS                      (20) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL (Timer D Timestamp 1 Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL                          ((uint32_t)(0xF << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS)) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL Mask */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_DIS                      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS                      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_DIS                      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RFU0                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RFU0                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RFU1                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RFU1                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_TX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_TX_MOD_CHG               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_VCD_0                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_VCD_0                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_VCD_1                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_VCD_1                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_0_OR_VCD_SUB28 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_0_OR_VCD_SUB28 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_1_OR_VCD_SUB32 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_1_OR_VCD_SUB32 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_FELICA_0              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_FELICA_0              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_FELICA_1              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_FELICA_1              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_PHASE_CHG      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_PHASE_CHG      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_B_PHASE_CHG      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_F          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_F          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_E          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_E          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_D          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_TYPE_A_SEQ_D          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_ETU_END               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_ETU_END               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS1_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS1_SEL_RX_MOD_CHG               Setting */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS                      (24) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL (Timer D Timestamp 2 Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL                          ((uint32_t)(0xF << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS)) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL Mask */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_DIS                      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS                      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_DIS                      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RFU0                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RFU0                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RFU1                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RFU1                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_TX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_TX_MOD_CHG               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_VCD_0                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_VCD_0                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_VCD_1                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_VCD_1                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_0_OR_VCD_SUB28 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_0_OR_VCD_SUB28 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_1_OR_VCD_SUB32 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_1_OR_VCD_SUB32 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_FELICA_0              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_FELICA_0              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_FELICA_1              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_FELICA_1              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_PHASE_CHG      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_PHASE_CHG      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_B_PHASE_CHG      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_F          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_F          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_E          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_E          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_D          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_TYPE_A_SEQ_D          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_ETU_END               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_ETU_END               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS2_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS2_SEL_RX_MOD_CHG               Setting */

#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS                      (28) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL (Timer D Timestamp 3 Select) Position */
#define MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL                          ((uint32_t)(0xF << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS)) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL Mask */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_DIS                      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_DIS                      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_DIS                      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RFU0                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU0                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RFU0                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RFU1                     (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RFU1                     << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RFU1                     Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_TX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_TX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_TX_MOD_CHG               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_VCD_0                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_0                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_VCD_0                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_VCD_1                 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_VCD_1                 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_VCD_1                 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_0_OR_VCD_SUB28 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_0_OR_VCD_SUB28 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_0_OR_VCD_SUB28 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_1_OR_VCD_SUB32 (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_1_OR_VCD_SUB32 << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_1_OR_VCD_SUB32 Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_FELICA_0              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_0              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_FELICA_0              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_FELICA_1              (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_FELICA_1              << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_FELICA_1              Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_PHASE_CHG      (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_B_PHASE_CHG      << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_B_PHASE_CHG      Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_F          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_F          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_F          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_E          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_E          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_E          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_D          (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_TYPE_A_SEQ_D          << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_TYPE_A_SEQ_D          Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_ETU_END               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_ETU_END               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_ETU_END               Setting */
#define MXC_S_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_MOD_CHG               (MXC_V_NFC_MXM_TMR_CFG5_TMR_D_TSX_SEL_RX_MOD_CHG               << MXC_F_NFC_MXM_TMR_CFG5_TMR_D_TS3_SEL_POS) /**< MXM_TMR_CFG5_TMR_D_TS3_SEL_RX_MOD_CHG               Setting */

/**@} end of group MXM_TMR_CFG5_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_CFG6_Register
 * @brief    Maxim Timer Configuration 6 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_CFG6_TMR_D_TARG_POS              (0) /**< MXM_TMR_CFG6_TMR_D_TARG (Timer D Target) Position */
#define MXC_F_NFC_MXM_TMR_CFG6_TMR_D_TARG                  ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_CFG6_TMR_D_TARG_POS)) /**< MXM_TMR_CFG6_TMR_D_TARG Mask */

/**@} end of group MXM_TMR_CFG6_Register */


/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS1_Register
 * @brief    Maxim Timer Status 1 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_CR_POS              (0) /**< MXM_TMR_STATUS1_TMR_A_CR (Timer A Complete/Rolled) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_CR                  ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_CR_POS)) /**< MXM_TMR_STATUS1_TMR_A_CR Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_RUN_POS             (1) /**< MXM_TMR_STATUS1_TMR_A_RUN (Timer A Running) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_RUN                 ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_RUN_POS)) /**< MXM_TMR_STATUS1_TMR_A_RUN Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_CR_POS              (2) /**< MXM_TMR_STATUS1_TMR_B_CR (Timer B Complete/Rolled) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_CR                  ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_CR_POS)) /**< MXM_TMR_STATUS1_TMR_B_CR Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_RUN_POS             (3) /**< MXM_TMR_STATUS1_TMR_B_RUN (Timer B Running) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_RUN                 ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_B_RUN_POS)) /**< MXM_TMR_STATUS1_TMR_B_RUN Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_CR_POS              (4) /**< MXM_TMR_STATUS1_TMR_C_CR (Timer C Complete/Rolled) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_CR                  ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_CR_POS)) /**< MXM_TMR_STATUS1_TMR_C_CR Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_RUN_POS             (5) /**< MXM_TMR_STATUS1_TMR_C_RUN (Timer C Running) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_RUN                 ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_C_RUN_POS)) /**< MXM_TMR_STATUS1_TMR_C_RUN Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_CR_POS              (6) /**< MXM_TMR_STATUS1_TMR_D_CR (Timer D Complete/Rolled) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_CR                  ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_CR_POS)) /**< MXM_TMR_STATUS1_TMR_D_CR Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_RUN_POS             (7) /**< MXM_TMR_STATUS1_TMR_D_RUN (Timer D Running) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_RUN                 ((uint32_t)(0x1 << MXC_F_NFC_MXM_TMR_STATUS1_TMR_D_RUN_POS)) /**< MXM_TMR_STATUS1_TMR_D_RUN Mask */

#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_COUNT_POS           (16) /**< MXM_TMR_STATUS1_TMR_A_COUNT (Timer A Count) Position */
#define MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_COUNT               ((uint32_t)(0xFFFF << MXC_F_NFC_MXM_TMR_STATUS1_TMR_A_COUNT_POS)) /**< MXM_TMR_STATUS1_TMR_A_COUNT Mask */

/**@} end of group MXM_TMR_STATUS1_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS2_Register
 * @brief    Maxim Timer Status 2 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS2_TMR_B_COUNT_POS           (16) /**< MXM_TMR_STATUS2_TMR_B_COUNT (Timer B Count) Position */
#define MXC_F_NFC_MXM_TMR_STATUS2_TMR_B_COUNT               ((uint32_t)(0xFFFF << MXC_F_NFC_MXM_TMR_STATUS2_TMR_B_COUNT_POS)) /**< MXM_TMR_STATUS2_TMR_B_COUNT Mask */

/**@} end of group MXM_TMR_STATUS2_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS3_Register
 * @brief    Maxim Timer Status 3 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS3_TMR_C_COUNT_POS           (0) /**< MXM_TMR_STATUS3_TMR_C_COUNT (Timer C Count) Position */
#define MXC_F_NFC_MXM_TMR_STATUS3_TMR_C_COUNT               ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS3_TMR_C_COUNT_POS)) /**< MXM_TMR_STATUS3_TMR_C_COUNT Mask */

/**@} end of group MXM_TMR_STATUS3_Register */


/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS4_Register
 * @brief    Maxim Timer Status 4 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS4_TMR_D_COUNT_POS           (0) /**< MXM_TMR_STATUS4_TMR_D_COUNT (Timer D Count) Position */
#define MXC_F_NFC_MXM_TMR_STATUS4_TMR_D_COUNT               ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS4_TMR_D_COUNT_POS)) /**< MXM_TMR_STATUS4_TMR_D_COUNT Mask */

/**@} end of group MXM_TMR_STATUS4_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS5_Register
 * @brief    Maxim Timer Status 5 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS5_TMR_D_TS0_POS (0) /**< MXM_TMR_STATUS5_TMR_D_TS0 (Timer D Timestamp 0) Position */
#define MXC_F_NFC_MXM_TMR_STATUS5_TMR_D_TS0     ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS5_TMR_D_TS0_POS)) /**< MXM_TMR_STATUS5_TMR_D_TS0 Mask */

/**@} end of group MXM_TMR_STATUS5_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS6_Register
 * @brief    Maxim Timer Status 6 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS6_TMR_D_TS1_POS (0) /**< MXM_TMR_STATUS6_TMR_D_TS1 (Timer D Timestamp 1) Position */
#define MXC_F_NFC_MXM_TMR_STATUS6_TMR_D_TS1     ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS6_TMR_D_TS1_POS)) /**< MXM_TMR_STATUS6_TMR_D_TS1 Mask */

/**@} end of group MXM_TMR_STATUS6_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS7_Register
 * @brief    Maxim Timer Status 7 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS7_TMR_D_TS2_POS (0) /**< MXM_TMR_STATUS7_TMR_D_TS2 (Timer D Timestamp 2) Position */
#define MXC_F_NFC_MXM_TMR_STATUS7_TMR_D_TS2     ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS7_TMR_D_TS2_POS)) /**< MXM_TMR_STATUS7_TMR_D_TS2 Mask */

/**@} end of group MXM_TMR_STATUS7_Register */

/**
 * @ingroup  nfc_registers
 * @defgroup MXM_TMR_STATUS8_Register
 * @brief    Maxim Timer Status 8 Register
 * @{
 */

#define MXC_F_NFC_MXM_TMR_STATUS8_TMR_D_TS3_POS (0) /**< MXM_TMR_STATUS8_TMR_D_TS3 (Timer D Timestamp 3) Position */
#define MXC_F_NFC_MXM_TMR_STATUS8_TMR_D_TS3     ((uint32_t)(0xFFFFFFFF << MXC_F_NFC_MXM_TMR_STATUS8_TMR_D_TS3_POS)) /**< MXM_TMR_STATUS8_TMR_D_TS3 Mask */

/**@} end of group MXM_TMR_STATUS8_Register */


#ifdef __cplusplus
}
#endif

#endif   /* _MXC_NFC_REGS_H_ */

