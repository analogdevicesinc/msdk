/*************************************************************************************************/
/*!
 * \file
 *
 * \brief      Audio codec driver implementation.
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

#include "pal_codec.h"
#include "hci_defs.h"

/**************************************************************************************************
  Functions: Control and Status
**************************************************************************************************/

/*************************************************************************************************/
/*!
 * \brief   Read local supported codecs.
 *
 * \param   pNumStd     Input is size of \a stdCodecs and output is actual number of standard codecs.
 * \param   stdCodecs   Standard codec info.
 * \param   pNumVs      Input is size of \a vsCodecs and output is actual number of vendor specific codecs.
 * \param   vsCodecs    Vendor specific codec info.
 *
 * \return  None.
 */
/*************************************************************************************************/
void PalCodecReadLocalSupportedCodecs(uint8_t *pNumStd, AudioStdCodecInfo_t stdCodecs[],
                                      uint8_t *pNumVs, AudioVsCodecInfo_t vsCodecs[])
{
  if (*pNumStd > 1) {
    *pNumStd = 1;
    stdCodecs[0].codecId = HCI_ID_LC3;
  }

  /* No VS codecs. */
  *pNumVs = 0;
}

/*************************************************************************************************/
/*!
 * \brief   Read local supported codec capabilities.
 *
 * \param   codingFmt   Coding format.
 * \param   compId      Company ID.
 * \param   vsCodecId   Vendor specific codec ID.
 * \param   dir         Direction.
 *
 * \return  TRUE if valid, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t PalCodecReadLocalSupportedCodecCapabilities(uint8_t codingFmt, uint16_t compId, uint16_t vsCodecId, PalAudioDir_t dir)
{
  switch (codingFmt) {
    case HCI_ID_LC3:
      return TRUE;

    case HCI_ID_VS:
    default:
      return FALSE;
  }
}

/*************************************************************************************************/
/*!
 * \brief   Read local supported codecs.
 *
 * \param   codingFmt   Coding format.
 * \param   compId      Company ID.
 * \param   vsCodecId   Vendor specific codec ID.
 * \param   dir         Direction.
 *
 * \return  TRUE if valid, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t PalCodecReadLocalSupportedControllerDelay(uint8_t codingFmt, uint16_t compId, uint16_t vsCodecId, PalAudioDir_t dir,
    uint32_t *pMinDly, uint32_t *pMaxDly)
{
  switch (codingFmt) {
    case HCI_ID_LC3:
      switch (dir) {
        case PAL_CODEC_DIR_INPUT:
          *pMinDly = 1000;
          *pMaxDly = 2000;
          break;
        case PAL_CODEC_DIR_OUTPUT:
          *pMinDly = 100;
          *pMaxDly = 200;
          break;
        default:
          return HCI_ERR_UNSUP_FEAT;
      }
      return TRUE;

    case HCI_ID_VS:
    default:
      return FALSE;
  }
}

/*************************************************************************************************/
/*!
 * \brief   Read local supported codecs.
 *
 * \param   dir         Direction.
 * \param   dataPathId  Data Path ID.
 *
 * \return  TRUE if valid, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t PalCodecConfigureDataPath(PalAudioDir_t dir, uint8_t dataPathId)
{
  /* TODO needs implementation. */
  return TRUE;
}
