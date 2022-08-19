#ifndef SVC_SEC_DATA_H
#define SVC_SEC_DATA_H

#ifdef __cplusplus
extern "C" {
#endif


/**************************************************************************************************
  UUIDs
**************************************************************************************************/
#define ATT_UUID_SEC_DATA_SERVICE \
    0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x42, 0xD9, 0x32, 0x7E, 0x36, 0xFC, 0x42
/* MCS service GATT characteristic UUIDs*/
#define ATT_UUID_SEC_DATA  0xBE, 0x35, 0xD1, 0x24, 0x99, 0x33, 0xC6, 0x87, 0x85, 0x41, 0xD9, 0x31, 0x3E, 0x56, 0xFC, 0x42

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/*! \brief Secured Data Service */
#define SEC_DATA_START_HDL               0x300            /*!< \brief Start handle. */
#define SEC_DATA_END_HDL                 (SEC_DAT_MAX_HDL - 1) /*!< \brief End handle. */

/**************************************************************************************************
 Handles
**************************************************************************************************/

/*! \brief Proprietary Service Handles */
enum
{
  SEC_DATA_SVC_HDL = SEC_DATA_START_HDL, /*!< \brief Secured Data service declaration */
  SEC_DAT_CH_HDL,                   /*!< \brief Secured Data characteristic */
  SEC_DAT_HDL,                      /*!< \brief Secured Data  */
  SEC_DAT_CH_CCC_HDL,               /*!< \brief Secured Data client characteristic configuration */
  SEC_DAT_MAX_HDL                       /*!< \brief Maximum handle. */
};
/**@}*/

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataAddGroup(void);

/*************************************************************************************************/
/*!
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcSecDataRemoveGroup(void);

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
void SvcSecDataCbackRegister(attsReadCback_t readCback, attsWriteCback_t writeCback);

/*! \} */    /* WP_SERVICE */

#ifdef __cplusplus
};
#endif

#endif /* SVC_SEC_DATA_H */
