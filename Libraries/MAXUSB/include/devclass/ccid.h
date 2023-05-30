/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
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

#ifndef LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_CCID_H_
#define LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_CCID_H_

/* CCID Class-specific requests */
#define CCID_CONTROL_ABORT                  0x01
#define CCID_CONTROL_GET_CLOCK_FREQUENCIES  0x02
#define CCID_CONTROL_GET_DATA_RATES         0x03

/* CCID command types */
#define PC_to_RDR_IccPowerOn        0x62
#define PC_to_RDR_IccPowerOff       0x63
#define PC_to_RDR_GetSlotStatus     0x65
#define PC_to_RDR_XfrBlock          0x6f
#define PC_to_RDR_GetParameters     0x6c
#define PC_to_RDR_ResetParameters   0x6d
#define PC_to_RDR_SetParameters     0x61
#define PC_to_RDR_Escape            0x6b
#define PC_to_RDR_IccClock          0x6e
#define PC_to_RDR_T0APDU            0x6a
#define PC_to_RDR_Secure            0x69
#define PC_to_RDR_Mechanical        0x71
#define PC_to_RDR_Abort             0x72
#define PC_to_RDR_SetDataRateAndClockFrequency 0x73

/* CCID response types */
#define RDR_to_PC_DataBlock         0x80
#define RDR_to_PC_SlotStatus        0x81
#define RDR_to_PC_Parameters        0x82
#define RDR_to_PC_Escape            0x83
#define RDR_to_PC_DataRateAndClockFrequency 0x84
#define RDR_to_PC_NotifySlotChange  0x50
#define RDR_to_PC_HardwareError     0x51

/* CCID Error Status */
/* All gaps are considered RFU  */
#define CMD_ABORTED                 -1
#define ICC_MUTE                    -2
#define XFR_PARITY_ERROR            -3
#define XFR_OVERRUN                 -4
#define HW_ERROR                    -5
#define BAD_ATR_TS                  -8      /*  6,7 not in 6.2.6 */
#define BAD_ATR_TCK                 -9
#define ICC_PROTOCOL_NOT_SUPPORTED  -10
#define ICC_CLASS_NOT_SUPPORTED     -11
#define PROCEDURE_BYTE_CONFLICT     -12
#define DEACTIVATED_PROTOCOL        -13
#define BUSY_WITH_AUTO_SEQUENCE     -14
#define PIN_TIMEOUT                 -16     /*  15 not in 6.2.6 */
#define PIN_CANCELLED               -17
#define CMD_SLOT_BUSY               -32     /*  18-31 not in 6.2.6 */
#define USER_DEFINED_BEGIN          -64     /*  Reserved to user defined errors */
#define USER_DEFINED_END            -127    /*  Reserved to user defined errors */
#define NOT_SUPPORTED_BEGIN         0x7F    /*  Incorrect message param */
#define NOT_SUPPORTED_END           0x01    /*  Incorrect message param */
#define COMMAND_NOT_SUPPORTED       0

/* Configuration structure */
typedef struct {
  uint8_t out_ep;           /*  endpoint to be used for OUT packets */
  uint8_t out_maxpacket;    /*  max packet size for OUT endpoint */
  uint8_t in_ep;            /*  endpoint to be used for IN packets */
  uint8_t in_maxpacket;     /*  max packet size for IN endpoint */
  uint8_t notify_ep;        /*  endpoint to be used for notifications */
  uint8_t notify_maxpacket; /*  max packet size for notifications */
} ccid_cfg_t;

#define PC_to_RDR_XfrBlock_bMessageType_OFFSET     0
#define PC_to_RDR_XfrBlock_dwLength_OFFSET         1
#define PC_to_RDR_XfrBlock_bSlot_OFFSET            5
#define PC_to_RDR_XfrBlock_bSeq_OFFSET             6
#define PC_to_RDR_XfrBlock_bBWI_OFFSET             7
#define PC_to_RDR_XfrBlock_wLevelParameter_OFFSET   8
#define PC_to_RDR_XfrBlock_abData_OFFSET           10

#define PC_to_RDR_XfrBlock_T0_T1_abData_MAX_LEN     261

#define PC_to_RDR_Parameters_T0_T1_bMessageType_OFFSET     0
#define PC_to_RDR_Parameters_T0_T1_dwLength_OFFSET         1
#define PC_to_RDR_Parameters_T0_T1_bSlot_OFFSET            5
#define PC_to_RDR_Parameters_T0_T1_bSeq_OFFSET             6
#define PC_to_RDR_Parameters_T0_T1_bProtocolNum_OFFSET     7
#define PC_to_RDR_Parameters_T0_T1_abRFU_OFFSET            8
#define PC_to_RDR_Parameters_T0_bmFindexDindex_OFFSET      10
#define PC_to_RDR_Parameters_T0_bmTCCKST0_OFFSET           11
#define PC_to_RDR_Parameters_T0_bmGuardTimeT0_OFFSET       12
#define PC_to_RDR_Parameters_T0_bWaitingIntegerT0_OFFSET   13
#define PC_to_RDR_Parameters_T0_bClockStop_OFFSET          14
#define PC_to_RDR_Parameters_T1_bmFindexDindex_OFFSET      10
#define PC_to_RDR_Parameters_T1_bmTCCKST1_OFFSET           11
#define PC_to_RDR_Parameters_T1_bmGuardTimeT1_OFFSET       12
#define PC_to_RDR_Parameters_T1_bWaitingIntegerT1_OFFSET   13
#define PC_to_RDR_Parameters_T1_bClockStop_OFFSET          14
#define PC_to_RDR_Parameters_T1_bIFSC_OFFSET               15
#define PC_to_RDR_Parameters_T1_bNadValue_OFFSET           16

#define PC_to_RDR_SetDataRateAndClockFrequency_bMessageType_OFFSET      0
#define PC_to_RDR_SetDataRateAndClockFrequency_dwLength_OFFSET          1
#define PC_to_RDR_SetDataRateAndClockFrequency_bSlot_OFFSET             5
#define PC_to_RDR_SetDataRateAndClockFrequency_bSeq_OFFSET              6
#define PC_to_RDR_SetDataRateAndClockFrequency_abRFU_OFFSET             7
#define PC_to_RDR_SetDataRateAndClockFrequency_dwClockFrequency_OFFSET  10
#define PC_to_RDR_SetDataRateAndClockFrequency_dwDataRate_OFFSET        14

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bChainParameter;
    uint8_t     *abData;
} RDR_to_PC_DataBlock_t;

#define RDR_to_PC_DataBlock_bMessageType_OFFSET     0
#define RDR_to_PC_DataBlock_dwLength_OFFSET         1
#define RDR_to_PC_DataBlock_bSlot_OFFSET            5
#define RDR_to_PC_DataBlock_bSeq_OFFSET             6
#define RDR_to_PC_DataBlock_bStatus_OFFSET          7
#define RDR_to_PC_DataBlock_bError_OFFSET           8
#define RDR_to_PC_DataBlock_bChainParamter_OFFSET   9
#define RDR_to_PC_DataBlock_abData_OFFSET           10

#define RDR_to_PC_DataBlock_Len_no_abData           RDR_to_PC_DataBlock_abData_OFFSET

#define bmCommandStatus_Failed_MASK                 0x40
#define bmICCStatus_ICC_pres_and_active_MASK        0x00
#define bmICCStatus_ICC_pres_and_inactive_MASK      0x01
#define bmICCStatus_ICC_not_pres_MASK               0x02

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bClockStatus;
} RDR_to_PC_SlotStatus_t;

#define RDR_to_PC_SlotStatus_bMessageType_OFFSET     0
#define RDR_to_PC_SlotStatus_dwLength_OFFSET         1
#define RDR_to_PC_SlotStatus_bSlot_OFFSET            5
#define RDR_to_PC_SlotStatus_bSeq_OFFSET             6
#define RDR_to_PC_SlotStatus_bStatus_OFFSET          7
#define RDR_to_PC_SlotStatus_bError_OFFSET           8
#define RDR_to_PC_SlotStatus_bClockStatus_OFFSET     9

#define RDR_to_PC_SlotStatus_Len                     RDR_to_PC_SlotStatus_bClockStatus_OFFSET + 1

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bProtocolNum;
    uint8_t     bmFindexDindex;
    uint8_t     bmTCCKST0;
    uint8_t     bGuardTimeT0;
    uint8_t     bWaitingIntegerT0;
    uint8_t     bClockStop;
} RDR_to_PC_Parameters_T0_t;

#define RDR_to_PC_Parameters_T0_bMessageType_OFFSET        0
#define RDR_to_PC_Parameters_T0_dwLength_OFFSET            1
#define RDR_to_PC_Parameters_T0_bSlot_OFFSET               5
#define RDR_to_PC_Parameters_T0_bSeq_OFFSET                6
#define RDR_to_PC_Parameters_T0_bStatus_OFFSET             7
#define RDR_to_PC_Parameters_T0_bError_OFFSET              8
#define RDR_to_PC_Parameters_T0_bProtocolNum_OFFSET        9
#define RDR_to_PC_Parameters_T0_bmFindexDindex_OFFSET      10
#define RDR_to_PC_Parameters_T0_bmTCCKST0_OFFSET           11
#define RDR_to_PC_Parameters_T0_bmGuardTimeT0_OFFSET       12
#define RDR_to_PC_Parameters_T0_bWaitingIntegerT0_OFFSET   13
#define RDR_to_PC_Parameters_T0_bClockStop_OFFSET          14

#define RDR_to_PC_Parameters_T0_Len                        RDR_to_PC_Parameters_T0_bClockStop_OFFSET + 1
#define RDR_to_PC_Parameters_Protocol_Num_T0               0x00
#define RDR_to_PC_Parameters_T0_CLOCK_STOP_NOT_ALLOWED     0x00

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bProtocolNum;
    uint8_t     bmFindexDindex;
    uint8_t     bmTCCKST1;
    uint8_t     bGuardTimeT1;
    uint8_t     bWaitingIntegerT1;
    uint8_t     bClockStop;
    uint8_t     bIFSC;
    uint8_t     bNadValue;
} RDR_to_PC_Parameters_T1_t;

#define RDR_to_PC_Parameters_T1_bMessageType_OFFSET        0
#define RDR_to_PC_Parameters_T1_dwLength_OFFSET            1
#define RDR_to_PC_Parameters_T1_bSlot_OFFSET               5
#define RDR_to_PC_Parameters_T1_bSeq_OFFSET                6
#define RDR_to_PC_Parameters_T1_bStatus_OFFSET             7
#define RDR_to_PC_Parameters_T1_bError_OFFSET              8
#define RDR_to_PC_Parameters_T1_bProtocolNum_OFFSET        9
#define RDR_to_PC_Parameters_T1_bmFindexDindex_OFFSET      10
#define RDR_to_PC_Parameters_T1_bmTCCKST1_OFFSET           11
#define RDR_to_PC_Parameters_T1_bmGuardTimeT1_OFFSET       12
#define RDR_to_PC_Parameters_T1_bWaitingIntegerT1_OFFSET   13
#define RDR_to_PC_Parameters_T1_bClockStop_OFFSET          14
#define RDR_to_PC_Parameters_T1_bIFSC_OFFSET               15
#define RDR_to_PC_Parameters_T1_bNadValue_OFFSET           16

#define RDR_to_PC_Parameters_T1_Len                        RDR_to_PC_Parameters_T1_bNadValue_OFFSET + 1
#define RDR_to_PC_Parameters_Protocol_Num_T1               0x01
#define RDR_to_PC_Parameters_T1_CLOCK_STOP_NOT_ALLOWED     0x00

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bRFU;
    uint32_t    dwClockFrequency;
    uint32_t    dwDataRate;
} RDR_to_PC_DataRateAndClockFrequency_t;

#define RDR_to_PC_DataRateAndClockFrequency_bMessageType_OFFSET     0
#define RDR_to_PC_DataRateAndClockFrequency_dwLength_OFFSET         1
#define RDR_to_PC_DataRateAndClockFrequency_bSlot_OFFSET            5
#define RDR_to_PC_DataRateAndClockFrequency_bSeq_OFFSET             6
#define RDR_to_PC_DataRateAndClockFrequency_bStatus_OFFSET          7
#define RDR_to_PC_DataRateAndClockFrequency_bError_OFFSET           8
#define RDR_to_PC_DataRateAndClockFrequency_bRFU_OFFSET             9
#define RDR_to_PC_DataRateAndClockFrequency_dwClockFrequency_OFFSET 10
#define RDR_to_PC_DataRateAndClockFrequency_dwDataRate_OFFSET       14

#define RDR_to_PC_DataRateAndClockFrequency_Len                     RDR_to_PC_DataRateAndClockFrequency_dwDataRate_OFFSET + 4

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint32_t    dwLength;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bStatus;
    uint8_t     bError;
    uint8_t     bRFU;
    uint8_t     *abData;
} RDR_to_PC_Escape_t;

#define RDR_to_PC_Escape_bMessageType_OFFSET     0
#define RDR_to_PC_Escape_dwLength_OFFSET         1
#define RDR_to_PC_Escape_bSlot_OFFSET            5
#define RDR_to_PC_Escape_bSeq_OFFSET             6
#define RDR_to_PC_Escape_bStatus_OFFSET          7
#define RDR_to_PC_Escape_bError_OFFSET           8
#define RDR_to_PC_Escape_bRFU_OFFSET             9
#define RDR_to_PC_Escape_abData_OFFSET           10

#define RDR_to_PC_Escape_Len_no_abData           RDR_to_PC_Escape_abData_OFFSET

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint8_t     bmSlotICCState;
} RDR_to_PC_NotifySlotChange_t;

#define RDR_to_PC_NotifySlotChange_bMessageType_OFFSET     0
#define RDR_to_PC_NotifySlotChange_bSlotICCState_OFFSET    1

#define RDR_to_PC_NotifySlotChange_Len                     RDR_to_PC_NotifySlotChange_bSlotICCState_OFFSET + 1

#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
    uint8_t     bMessageType;
    uint8_t     bSlot;
    uint8_t     bSeq;
    uint8_t     bHardwareErrorCode;
} RDR_to_PC_HardwareError_t;

#define RDR_to_PC_HardwareError_bMessageType_OFFSET         0
#define RDR_to_PC_HardwareError_bSlot_OFFSET                1
#define RDR_to_PC_HardwareError_bSeq_OFFSET                 2
#define RDR_to_PC_HardwareError_bHardwareErrorCode_OFFSET   3

#define RDR_to_PC_HardwareError_Len                         RDR_to_PC_HardwareError_bHardwareErrorCode_OFFSET + 1

#define RDR_to_PC_MAX_Message_Len                           (PC_to_RDR_XfrBlock_abData_OFFSET + PC_to_RDR_XfrBlock_T0_T1_abData_MAX_LEN)


#if defined(__GNUC__)
typedef struct __attribute__((packed)) {
#else
typedef __packed struct {
#endif
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  bcdCCID;
  uint8_t   bMaxSlotIndex;
  uint8_t   bVoltageSupport;
  uint32_t  dwProtocols;
  uint32_t  dwDefaultClock;
  uint32_t  dwMaximumClock;
  uint8_t   bNumClockSupported;
  uint32_t  dwDataRate;
  uint32_t  dwMaxDataRate;
  uint8_t   bNumDataRatesSupported;
  uint32_t  dwMaxIFSD;
  uint32_t  dwSynchProtocols;
  uint32_t  dwMechanical;
  uint32_t  dwFeatures;
  uint32_t   dwMaxCCIDMessageLength;
  uint8_t   bClassGetResponse;
  uint8_t   bClassEnvelope;
  uint16_t  wLcdLayout;
  uint8_t   bPINSupport;
  uint8_t   bMaxCCIDBusySlots;
} ccid_descriptor_t;

/* CCID callback events, PC_to_RDR recieved on OUT EP */
typedef enum {
  CCID_ICC_POWER_ON,
  CCID_ICC_POWER_OFF,
  CCID_GET_SLOT_STATUS,
  CCID_XFR_BLOCK,
  CCID_GET_PARAMETERS,
  CCID_RESET_PARAMETERS,
  CCID_SET_PARAMETERS,
  CCID_ESCAPE,
  CCID_ICC_CLOCK,
  CCID_T0APDU,
  CCID_SECURE,
  CCID_MECHANICAL,
  CCID_ABORT,
  CCID_SET_DATA_RATE_AND_CLOCK_FREQUENCY,
  CCID_CONFIGURED,
  CCID_DECONFIGURED,
  CCID_NUM_CALLBACKS                          /* Do not use */
} ccid_callback_t;

/**
 *  \brief    Initialize the class driver
 *  \details  Initialize the class driver.
 *  \param    interface  The interface number for this class, from the Configuration Descriptor
 *  \return   Zero (0) for success, non-zero for failure
 */
int ccid_init(unsigned int interface);

/**
 *  \brief    Add a function to be called for unhandled class requests
 *  \details  Register a function to be called for unhandled class requests. To disable the
 *            callback, call this function with a NULL parameter.
 */
void ccid_chain_class_req(int (*func)(MXC_USB_SetupPkt *, void *), void *cbdata);

/**
 *  \brief    Set the specified configuration
 *  \details  Configures the class and endpoints and starts operation. This function should be
 *            called upon configuration from the host.
 *  \param    cfg   configuration to be set
 *  \return   Zero (0) for success, non-zero for failure
 */
int ccid_configure(const ccid_cfg_t *cfg);

/**
 *  \brief    Clear the current configuration and resets endpoints
 *  \details  Clear the current configuration and resets endpoints.
 *  \return   Zero (0) for success, non-zero for failure
 */
int ccid_deconfigure(void);

/**
 *  \brief    Checks if ccid message received
 *  \details  Set variable if a message received
 *  \return   One(1) if received, Zero(0) if not
 */
int ccid_is_received();

/**
 *  \brief    Dispatch the callbacks
 *  \details  Dispatch according to last received message
 */
void ccid_dispatcher(void *cbdata);

/**
 *  \brief    Register a callback to be called upon the specified event.
 *  \details  Register a callback to be called upon the specified event. To disable the
 *            callback, call this function with a NULL parameter.
 *  \return   Zero (0) for success, non-zero for failure
 *  \note     Callbacks are executed in interrupt context
 */
int ccid_register_callback(ccid_callback_t cbnum, int (*func)(MXC_USB_Req_t *));

/**
 *
 *  \brief    Sends response data to USB host
 *  \return   Zero (0) for success, non-zero for failure
 *  \note     Storage for buffer must exist until ccid_deconfigure() is called (i.e. not on caller stack)
 *
 */
int ccid_rdr_to_pc(uint8_t *buffer, unsigned int len);

/**
 *
 *  \brief    Sends notification data to USB host on the Interrupt endpoing
 *  \return   Zero (0) for success, non-zero for failure
 *  \note     Storage for buffer must exist until ccid_deconfigure() is called (i.e. not on caller stack)
 *
 */
int ccid_notify(uint8_t *buffer, unsigned int len);

#endif // LIBRARIES_MAXUSB_INCLUDE_DEVCLASS_CCID_H_
