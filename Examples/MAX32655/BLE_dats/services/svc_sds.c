#include "wsf_types.h"
#include "att_api.h"
#include "wsf_trace.h"
#include "util/bstream.h"
#include "svc_ch.h"
#include "svc_sds.h"
#include "svc_cfg.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read/write permissions */
#ifndef SEC_DATA_PERMIT_READ
#define SEC_DATA_PERMIT_READ SVC_SEC_PERMIT_READ
#endif

#ifndef SEC_DATA_PERMIT_READ_ENC
#define SEC_DATA_PERMIT_READ_ENC ATTS_PERMIT_READ_ENC
#endif

#ifndef SEC_DATA_PERMIT_READ_AUTH
#define SEC_DATA_PERMIT_READ_AUTH ATTS_PERMIT_READ_AUTH
#endif

#ifndef SEC_DATA_PERMIT_WRITE
#define SEC_DATA_PERMIT_WRITE SVC_SEC_PERMIT_WRITE
#endif

#ifndef SEC_DATA_PERMIT_WRITE_ENC
#define SEC_DATA_PERMIT_WRITE_ENC ATTS_PERMIT_WRITE_ENC
#endif

#ifndef SEC_DATA_PERMIT_WRITE_AUTH
#define SEC_DATA_PERMIT_WRITE_AUTH ATTS_PERMIT_WRITE_AUTH
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/* UUIDs */
static const uint8_t svcSecDatUuid[] = { ATT_UUID_SEC_DATA };

/* Proprietary service declaration */
static const uint8_t secDatSvc[] = { ATT_UUID_SEC_DATA_SERVICE };
static const uint16_t secDatLenSvc = sizeof(secDatSvc);

/* Secured data characteristic */
static const uint8_t secDatCh[] = { ATT_PROP_NOTIFY | ATT_PROP_WRITE_NO_RSP | ATT_PROP_WRITE,
                                    UINT16_TO_BYTES(SEC_DAT_HDL), ATT_UUID_SEC_DATA };

static const uint16_t secDatLenDatCh = sizeof(secDatCh);

/* Secured data */
static const uint8_t secDatVal[] = { 0 };
static const uint16_t secDatLenDat = sizeof(secDatVal);

/* Secured data client characteristic configuration */
static uint8_t secDatChCcc[] = { UINT16_TO_BYTES(0x0000) };
static const uint16_t secDatLenDatChCcc = sizeof(secDatChCcc);

/* Attribute list for Secured Data group */
static const attsAttr_t secDatList[] = {
    /* Service declaration */
    { attPrimSvcUuid, (uint8_t *)secDatSvc, (uint16_t *)&secDatLenSvc, sizeof(secDatSvc), 0,
      ATTS_PERMIT_READ },

    /* Secure data characteristic declaration */
    { attChUuid, (uint8_t *)secDatCh, (uint16_t *)&secDatLenDatCh, sizeof(secDatCh), 0,
      ATTS_PERMIT_READ },

    /* Secure data characteristic value */
    { svcSecDatUuid, (uint8_t *)secDatVal, (uint16_t *)&secDatLenDat, ATT_VALUE_MAX_LEN,
      (ATTS_SET_UUID_128 | ATTS_SET_VARIABLE_LEN | ATTS_SET_WRITE_CBACK),
      SEC_DATA_PERMIT_WRITE | SEC_DATA_PERMIT_WRITE_ENC | SEC_DATA_PERMIT_WRITE_AUTH },

    /*Secure data characteristic value descriptor*/
    { attCliChCfgUuid, (uint8_t *)secDatChCcc, (uint16_t *)&secDatLenDatChCcc, sizeof(secDatChCcc),
      ATTS_SET_CCC, (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE) }
};

/* Secured Data group structure */
static attsGroup_t svcSecDatGroup = { NULL, (attsAttr_t *)secDatList, NULL,
                                      NULL, SEC_DATA_START_HDL,       SEC_DATA_END_HDL };

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataAddGroup(void)
{
    AttsAddGroup(&svcSecDatGroup);
}

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataRemoveGroup(void)
{
    AttsRemoveGroup(SEC_DATA_START_HDL);
}

/*************************************************************************************************/
/*!
 *  \brief  Register callbacks for the service.
 *
 *  \param  readCback   Read callback function.
 *  \param  writeCback  Write callback function.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback)
{
    svcSecDatGroup.readCback = readCback;
    svcSecDatGroup.writeCback = writeCback;
}
