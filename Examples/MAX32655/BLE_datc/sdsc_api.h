/*************************************************************************************************/
/*! Secure Data Service Client
*   Implements the necessary handles list to perform service 
*   and characteristic discovery of custom secured service .
*
 */
/*************************************************************************************************/

#ifndef SDSC_API_H
#define SDSC_API_H

#include "att_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Secured service  enumeration of handle indexes of characteristics to be discovered */
enum {
    SEC_DAT_HDL_IDX, /*!< \brief Secured data */
    SEC_DAT_CCC_HDL_IDX, /*!< \brief Secured data client characteristic configuration descriptor */
    SEC_HDL_LIST_LEN /*!< \brief Handle list length */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \brief  Perform service and characteristic discovery for Secured service .
 *          Parameter pHdlList must point to an array of length \ref SEC_HDL_LIST_LEN.
 *          If discovery is successful the handles of discovered characteristics and
 *          descriptors will be set in pHdlList.
 *
 *  \param  connId    Connection identifier.
 *  \param  pHdlList  Characteristic handle list.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SecDatSvcDiscover(dmConnId_t connId, uint16_t *pHdlList);

#ifdef __cplusplus
};
#endif

#endif /* SDSC_API_H */
