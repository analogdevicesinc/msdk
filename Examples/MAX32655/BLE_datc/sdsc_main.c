#include "app_api.h"
#include "sdsc_api.h"
#include "util/bstream.h"
#include "wsf_assert.h"
#include "wsf_types.h"
#include <string.h>

/**************************************************************************************************
  Secure  Service and Data UUIDs
**************************************************************************************************/
#define ATT_UUID_SEC_DATA_SERVICE                                                                  \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x42, 0xD9, 0x32, 0x7E, 0x36, 0xFC, 0x42
/* MCS service GATT characteristic UUIDs*/
#define ATT_UUID_SEC_DATA                                                                          \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x41, 0xD9, 0x31, 0x3E, 0x56, 0xFC, 0x42
/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/* UUIDs */
static const uint8_t SecDatSvcUuid[] = { ATT_UUID_SEC_DATA_SERVICE }; /*! Secured service */
static const uint8_t SecDatChUuid[] = { ATT_UUID_SEC_DATA }; /*! Secured data  */

/* Characteristics for discovery */

/*! Secured data */
static const attcDiscChar_t secDat = { SecDatChUuid, ATTC_SET_REQUIRED | ATTC_SET_UUID_128 };

/*! Secured data descriptor */
static const attcDiscChar_t secDatCcc
    = { attCliChCfgUuid, ATTC_SET_REQUIRED | ATTC_SET_DESCRIPTOR };

/*! List of characteristics to be discovered; order matches handle index enumeration  */
static const attcDiscChar_t* secDatDiscCharList[] = {
    &secDat, /*! Secured data */
    &secDatCcc /*! Secured data descriptor */
};

/* sanity check:  make sure handle list length matches characteristic list length */
WSF_CT_ASSERT(SEC_HDL_LIST_LEN == ((sizeof(secDatDiscCharList) / sizeof(attcDiscChar_t*))));

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for custom secured service .
 *          Parameter pHdlList must point to an array of length SEC_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SecDatSvcDiscover(dmConnId_t connId, uint16_t* pHdlList)
{
    AppDiscFindService(connId, ATT_128_UUID_LEN, (uint8_t*)SecDatSvcUuid, SEC_HDL_LIST_LEN,
        (attcDiscChar_t**)secDatDiscCharList, pHdlList);
}
