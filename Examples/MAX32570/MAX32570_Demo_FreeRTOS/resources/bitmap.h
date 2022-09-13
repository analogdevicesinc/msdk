/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
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
 *
 ******************************************************************************/

#ifndef _BITMAP_H_
#define _BITMAP_H_

// bitmaps id
#define arrow_left_bmp 0
#define arrow_right_bmp 1
#define check_bmp 2
#define home_bmp 3
#define information_bmp 4
#define integrated_only_small_bmp 5
#define keypad_large_bmp 6
#define keypad_bmp 7
#define key_0_bg_white_bmp 8
#define key_00_bg_white_bmp 9
#define key_1_bg_white_bmp 10
#define key_2_bg_white_bmp 11
#define key_3_bg_white_bmp 12
#define key_4_bg_white_bmp 13
#define key_5_bg_white_bmp 14
#define key_6_bg_white_bmp 15
#define key_7_bg_white_bmp 16
#define key_8_bg_white_bmp 17
#define key_9_bg_white_bmp 18
#define key_cancel_bg_white_bmp 19
#define key_clear_bg_white_bmp 20
#define key_enter_bg_white_bmp 21
#define logo_white_bg_white_bmp 22
#define magstripe_large_bmp 23
#define magstripe_bmp 24
#define maxim_integrated_large_bmp 25
#define maxim_logo_only_small_000_bmp 26
#define maxim_logo_only_small_001_bmp 27
#define maxim_logo_only_small_002_bmp 28
#define maxim_logo_only_small_003_bmp 29
#define maxim_logo_only_small_004_bmp 30
#define maxim_logo_only_small_005_bmp 31
#define maxim_logo_only_small_006_bmp 32
#define maxim_logo_only_small_007_bmp 33
#define maxim_logo_only_small_008_bmp 34
#define maxim_logo_only_small_009_bmp 35
#define maxim_logo_only_small_010_bmp 36
#define maxim_logo_only_small_011_bmp 37
#define maxim_logo_only_small_012_bmp 38
#define maxim_logo_only_small_013_bmp 39
#define maxim_logo_only_small_014_bmp 40
#define maxim_logo_only_small_015_bmp 41
#define maxim_logo_only_small_016_bmp 42
#define maxim_logo_only_small_017_bmp 43
#define maxim_logo_only_small_018_bmp 44
#define maxim_logo_only_small_019_bmp 45
#define maxim_logo_only_small_020_bmp 46
#define maxim_logo_only_small_021_bmp 47
#define maxim_logo_only_small_022_bmp 48
#define maxim_logo_only_small_023_bmp 49
#define maxim_logo_only_small_024_bmp 50
#define maxim_logo_only_small_025_bmp 51
#define maxim_logo_only_small_026_bmp 52
#define maxim_logo_only_small_027_bmp 53
#define maxim_logo_only_small_028_bmp 54
#define maxim_logo_only_small_029_bmp 55
#define maxim_logo_only_small_030_bmp 56
#define maxim_logo_only_small_031_bmp 57
#define maxim_logo_only_small_bmp 58
#define medium_logo_000_bmp 59
#define medium_logo_001_bmp 60
#define medium_logo_002_bmp 61
#define medium_logo_003_bmp 62
#define medium_logo_004_bmp 63
#define medium_logo_005_bmp 64
#define medium_logo_006_bmp 65
#define medium_logo_007_bmp 66
#define medium_logo_008_bmp 67
#define medium_logo_009_bmp 68
#define medium_logo_010_bmp 69
#define medium_logo_011_bmp 70
#define medium_logo_012_bmp 71
#define medium_logo_013_bmp 72
#define medium_logo_014_bmp 73
#define medium_logo_015_bmp 74
#define medium_logo_016_bmp 75
#define medium_logo_017_bmp 76
#define medium_logo_018_bmp 77
#define medium_logo_019_bmp 78
#define medium_logo_020_bmp 79
#define medium_logo_021_bmp 80
#define medium_logo_022_bmp 81
#define medium_logo_023_bmp 82
#define medium_logo_024_bmp 83
#define medium_logo_025_bmp 84
#define medium_logo_026_bmp 85
#define medium_logo_027_bmp 86
#define medium_logo_028_bmp 87
#define medium_logo_029_bmp 88
#define medium_logo_030_bmp 89
#define medium_logo_031_bmp 90
#define medium_logo_032_bmp 91
#define medium_logo_033_bmp 92
#define medium_logo_034_bmp 93
#define medium_logo_035_bmp 94
#define nfc_large_bmp 95
#define nfc_bmp 96
#define slideshow_large_bmp 97
#define slideshow_bmp 98
#define smartcard_large_bmp 99
#define smartcard_bmp 100
#define mpos_bmp 101
#define parrot_bmp 102

// fonts id
#define urw_gothic_12_white_bg_grey 0
#define urw_gothic_13_grey_bg_white 1
#define urw_gothic_16_bleu_bg_grey 2
#define urw_gothic_16_white_bg_grey 3

#endif //_BITMAP_H_
