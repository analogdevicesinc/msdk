/*******************************************************************************
* Copyright (C) 2009-2018 Maxim Integrated Products, Inc., All Rights Reserved.
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
*******************************************************************************
*
* @author: Benjamin VINOT <benjamin.vinot@maximintegrated.com>
*
*/

#ifndef __SCP_DEFINITIONS_H__
#define __SCP_DEFINITIONS_H__
#include <session_build.h>
/***************************
 *    SCP - frame format
 **************************/

/***************************
 *    SCP - targets
 **************************/

#define HOST 0
#define TARGET 1

/* -- SCP - modes  -- */
#define SCP_RSA 0x03
#define SCP_FLORA_RSA 0x04
#define MSP_MAXQ1852_ECDSA 0x06
#define SCP_ANGELA_ECDSA 0x07
#define SCP_LITE_ECDSA 0x08
#define SCP_PAOLA 0x09
#define SCP_SESSION_MODE_UNK 0xFF

#define SCP_PAOLA_MSK (1 << SCP_PAOLA)
#define SCP_ANGELA_ECDSA_MSK (1 << SCP_ANGELA_ECDSA)
#define SCP_RSA_MSK (1 << SCP_RSA)
#define SCP_FLORA_RSA_MSK (1 << SCP_FLORA_RSA)
#define MSP_MAXQ1852_ECDSA_MSK (1 << MSP_MAXQ1852_ECDSA)
#define SCP_LITE_ECDSA_MSK (1 << SCP_LITE_ECDSA)

/* -- SCP - security profiles  -- */
#define SCP_PP_PAOLA_4096 0x08
#define SCP_PP_RSA 0x09
#define SCP_PP_PAOLA_2048 0x09

#define SCP_PP_ECDSA 0x0A
#define SCP_PP_UNK 0xFF

#define MAX_IDF 400

#define HEADER_LEN 8
#define CRC_LEN 4
#define COMMAND_LEN 4

#define DATA_CHECKSUM_LEN 4

/* -- SCP - physical layer parameters -- */

/* USIP-SBL and JIBE-SCP commands, synchros, .... */
#define SYNCH1 0xBE
#define SYNCH2 0xEF
#define SYNCH3 0xED

/* -- SCP - transport layer parameters -- */

#define HELLO_SCP_REQ_BLOCK_LEN 0x0E
#define HELLO_SCP_REQ_LEN 0x0A
#define HELLO_SCP_REQ_CONST_LEN 0x09
#define HELLO_SCP_REP_BLOCK_LEN 0x32
#define HELLO_SCP_REP_CONST_LEN 0x0A
#define HELLO_REQ_BLOCK_LEN 0x0C
#define HELLO_REP_BLOCK_LEN 0x32
#define HELLO_OFF_REQ_BLOCK_LEN 0x0C
#define HELLO_OFF_REP_BLOCK_LEN 0x22
#define CHALLENGE_BLOCK_LEN 0x14
#define SUCCESS_BLOCK_LEN 0x4
#define FAILURE_BLOCK_LEN 0x4

#define HELLO_REQ_LEN 0x08
#define HELLO_REQ_CONST_LEN 0x08
#define HELLO_REP_LEN 0x2E
#define HELLO_REP_CONST_LEN 0x07
#define HELLO_REP_LC 0x0B
#define HELLO_REP_UMV 0x0C
#define HELLO_REP_UmV 0x0D
#define HELLO_REP_SMV 0x0E
#define HELLO_REP_SmV 0x0F
#define HELLO_REP_HMV 0x10
#define HELLO_REP_HmV 0x11
#define HELLO_OFF_REQ_LEN 0x08
#define HELLO_OFF_REQ_CONST_LEN 0x08
#define HELLO_OFF_REP_LEN 0x1E
#define HELLO_OFF_REP_CONST_LEN 0x07
#define HELLO_OFF_REP_LC 0x0B
#define HELLO_OFF_REP_UMV 0x0C
#define HELLO_OFF_REP_UmV 0x0D
#define HELLO_OFF_REP_SMV 0x0E
#define HELLO_OFF_REP_SmV 0x0F
#define HELLO_OFF_REP_HMV 0x10
#define HELLO_OFF_REP_HmV 0x11
#define CHALLENGE_VALUE 4
#define CHALLENGE_LEN 0x10
#define HELLO_REP_RANDOM 0x22
#define HELLO_REP_USN 0x12
#define HELLO_OFF_REP_USN 0x12

/**
 * SCP Session Layer Command
 */
typedef enum {
    HELLO_REQ = 0x1, //!< Hello-Request
    HELLO_REP = 0x2, //!< Hello-Reply
    HELLO_OFF_REQ = 0x8, //!< HELLO_OFF_REQ
    HELLO_OFF_REP = 0x9, //!< HELLO_OFF_REP
    CHALLENGE = 0x7, //!< Challenge-Request (applicable only for AES)
    SUCCESS = 0x3, //!< Operation successful (applicable only for AES)
    FAILURE = 0x4, //!< Operation failed (applicable only for AES)
    DATA = 0x5 //!< Data Transfer
} session_cmd_t;

/**
 * SCP Data Link Control Code (CTL)
 * The “Control Code” field is one byte and identifies the type of segment.
 */
typedef enum {
    CON_REQ = 0x01, /*!< Connection request  */
    CON_REP = 0x02, /*!< Connection reply */
    DISC_REQ = 0x03, /*!< Disconnection request */
    DISC_REP = 0x04, /*!< Disconnection reply */
    DATA_TRANSFER = 0x05, /*!< Data Exchange */
    ACK = 0x06, /*!< Acknowledge */
    CON_REF = 0x09, /*!< Connection refused */
    ECHO_REQ = 0x0B, /*!< Echo request */
    ECHO_REP = 0x0C /*!< Echo reply */
} ctl_code_t;

#define USER_FLASH_CHECK 0x02
#define RESERVED_SECTOR_CHECK 0x03

/* SCP-MAXQ1852 commands, synchro, ... */
#define MAXQ1852_SCDESIGNATOR 0x80
#define MAXQ1852_SCPROMPT '>'
#define MAXQ1852_SCOFFSET 0x00
#define MAXQ1852_SC_LEN_BYTE1 1
#define MAXQ1852_SC_LEN_BYTE2 2

/*----------------------------------------------------------------------------------------------*/

/* -- SCP - application layer parameters -- */
#define ADMIN 0x02
#define USER 0x01

/* -- SCP - application layer commands --*/
/**
  *
  */
typedef enum {
    /* confused with FLORA ERASE-DATA */
    ERASE_MEM = 0x4401, /*!< ERASE_MEM */
    /* confused with FLORA WRITE-DATA */
    WRITE_MEM = 0x2402, //!< WRITE_MEM
    /* confused with FLORA COMPARE-DATA */
    VERIFY_MEM = 0x2403, //!< VERIFY_MEM
    /*  FLORA commands */
    WRITE_CRK = 0x470A, //!< WRITE_CRK
    WRITE_APP_VER = 0x470B, //!< WRITE_APP_VER
    REWRITE_CRK = 0x461A, //!< REWRITE_CRK
    WRITE_OTP = 0x4714, //!< WRITE_OTP
    WRITE_TIMEOUT = 0x4426, //!< WRITE_TIMEOUT
    WRITE_PARAMS = 0x4427, //!< WRITE_PARAMS
    WRITE_STIM = 0x4428, //!< WRITE_STIM
    WRITE_DEACT = 0x4429, //!< WRITE_DEACT
    KILL_CHIP = 0x4538, //!< KILL_CHIP
    KILL_CHIP2 = 0x4539, //!< KILL_CHIP2
    EXECUTE_CODE = 0x2101 //!< EXECUTE_CODE
} scp_app_cmd_t; /**
 *
 */
typedef enum {
    /* SCP */
    COMMAND_WRITE_FILE, //!< COMMAND_WRITE_FILE
    COMMAND_VERIFY_FILE, //!< COMMAND_VERIFY_FILE
    COMMAND_WRITE_CRK, //!< COMMAND_WRITE_CRK
    COMMAND_WRITE_OTP, //!< COMMAND_WRITE_OTP
    COMMAND_WRITE_TIMEOUT, //!< COMMAND_WRITE_TIMEOUT
    COMMAND_WRITE_PARAMS, //!< COMMAND_WRITE_PARAMS
    COMMAND_WRITE_STIM, //!< COMMAND_WRITE_STIM
    COMMAND_WRITE_DEACT, //!< COMMAND_WRITE_DEACT
    COMMAND_KILL_CHIP2, //!< COMMAND_KILL_CHIP2
    COMMAND_KILL_CHIP, //!< COMMAND_KILL_CHIP
    COMMAND_EXECUTE_CODE, //!< COMMAND_EXECUTE_CODE
    COMMAND_UNKNOWN, //!< COMMAND_UNKNOWN
    COMMAND_WRITE_ONLY, //!< COMMAND_WRITE_ONLY
    COMMAND_ERASE_DATA, //!< COMMAND_ERASE_DATA
    COMMAND_WRITE_DATA, //!< COMMAND_ERASE_DATA
    COMMAND_WRITE_APP_VER,
    /* MSP 1852 */
    COMMAND_MAXQ1852_LOAD_CUSTOMER_KEY, //!< COMMAND_MAXQ1852_LOAD_CUSTOMER_KEY
    COMMAND_MAXQ1852_ERASE_CODE_FLASH_AREA, //!< COMMAND_MAXQ1852_ERASE_CODE_FLASH_AREA
    COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS, //!< COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS
    COMMAND_MAXQ1852_LOAD_CODE, //!< COMMAND_MAXQ1852_LOAD_CODE
    COMMAND_MAXQ1852_LOAD_DATA, //!< COMMAND_MAXQ1852_LOAD_DATA
    COMMAND_MAXQ1852_VERIFY_CODE, //!< COMMAND_MAXQ1852_VERIFY_CODE
    COMMAND_MAXQ1852_VERIFY_DATA, //!< COMMAND_MAXQ1852_VERIFY_DATA
    COMMAND_MAXQ1852_WRITE_REGISTER, //!< COMMAND_MAXQ1852_WRITE_REGISTER
    COMMAND_MAXQ1852_READ_REGISTER, //!< COMMAND_MAXQ1852_READ_REGISTER
    COMMAND_MAXQ1852_ENGAGE_PLLO, //!< COMMAND_MAXQ1852_ENGAGE_PLLO
    COMMAND_MAXQ1852_VERIFY_CUSTOMER_KEY, //!< COMMAND_MAXQ1852_VERIFY_CUSTOMER_KEY
    COMMAND_MAXQ1852_ACTIVATE_CUSTOMER_KEY, //!< COMMAND_MAXQ1852_ACTIVATE_CUSTOMER_KEY
    COMMAND_MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE, //!< COMMAND_MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE
    COMMAND_MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE, //!< COMMAND_MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE
    COMMAND_MAXQ1852_LOAD_FILE, //!< COMMAND_MAXQ1852_LOAD_FILE
    COMMAND_MAXQ1852_VERIFY_FILE, //!< COMMAND_MAXQ1852_VERIFY_FILE
    COMMAND_REWRITE_CRK, //!< COMMAND_REWRITE_CRK
    COMMAND_SCP_LITE_LOAD_RAM, //!< COMMAND_SCP_LITE_LOAD_RAM
    COMMAND_ECHO, //!< COMMAND_ECHO
    MAX_SCP_COMMAND //!< MAX_SCP_COMMAND
} script_cmd_t;
#define MAXQ1852_LOAD_CUSTOMER_KEY 0x01
#define MAXQ1852_VERIFY_CUSTOMER_KEY 0x02
#define MAXQ1852_ACTIVATE_CUSTOMER_KEY 0x03
#define MAXQ1852_ERASE_CODE_FLASH_AREA 0x10
#define MAXQ1852_ERASE_ALL_FLASH_AREAS 0x11
#define MAXQ1852_LOAD_CODE 0x20
#define MAXQ1852_LOAD_DATA 0x21
#define MAXQ1852_VERIFY_CODE 0x30
#define MAXQ1852_VERIFY_DATA 0x31
#define MAXQ1852_WRITE_REGISTER 0x40
#define MAXQ1852_READ_REGISTER 0x41
#define MAXQ1852_ENGAGE_PLLO 0x50
#define MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE 0x60
#define MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE 0x61

#define SCP_LITE_SYNC1 0xCE
#define SCP_LITE_SYNC2 0xD1

/*----------------------------------------------------------------------------------------------*/
static const char idf_ctl[MAX_IDF][30] = {
    [CON_REQ] = "CON_REQ",   [CON_REP] = "CON_REP",   [CON_REF] = "CON_REF",
    [DISC_REQ] = "DISC_REQ", [DISC_REP] = "DISC_REP", [DATA_TRANSFER] = "DATA_TRANSFER",
    [ACK] = "ACK",           [ECHO_REQ] = "ECHO_REQ", [ECHO_REP] = "ECHO_REP"
};

/*----------------------------------------------------------------------------------------------*/

/**
 *
 */
typedef struct {
    char timeout_char;
    uint8_t target;
} scp_timeout_t;

/**
 *
 */
typedef enum {
    SCP_TARGET_UART, //!< SCP_TARGET_UART
    SCP_TARGET_USB, //!< SCP_TARGET_USB
    SCP_TARGET_VBUS, //!< SCP_TARGET_VBUS
    SCP_TARGET_ETH, //!< SCP_TARGET_ETH
    SCP_TARGET_SPI, //!< SCP_TARGET_SPI
    _SCP_TARGET_MAX //!< _SCP_TARGET_MAX
} scp_target_t;

static const scp_timeout_t timeout_c[] = { [SCP_TARGET_UART] = { '0', 0x00 },
                                           [SCP_TARGET_USB] = { 'U', 0x55 },
                                           [SCP_TARGET_VBUS] = { 'V', 0x56 },
                                           [SCP_TARGET_ETH] = { 'E', 0x45 },
                                           [SCP_TARGET_SPI] = { 'S', 0x53 } };

typedef struct {
    option_type_t type;
    uint8_t mandatory;
    int min;
    int max;
} scp_script_param_t;

typedef struct {
    char *name;
    scp_script_param_t param[3];
} scp_script_cmd_def_t;

typedef struct {
    script_cmd_t cmd;
    char param[3][MAX_LINE];
} script_cmd_list_t;
static const scp_script_cmd_def_t idf_scp_cmd[MAX_SCP_COMMAND + 1] = {
    [COMMAND_UNKNOWN] = { "unknown-command", { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_FILE] = { "write-file",
                             { { OT_FILE, TRUE, 0, 0 }, { OT_LONGHEX, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_ONLY] = { "write-only",
                             { { OT_FILE, TRUE, 0, 0 }, { OT_LONGHEX, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_ERASE_DATA] = { "erase-data",
                             { { OT_LONGHEX, TRUE, 0, 0 },
                               { OT_LONGHEX, TRUE, 0, 0 },
                               { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_DATA] = { "write-data",
                             { { OT_LONGHEX, TRUE, 0, 0 },
                               { OT_LONGHEX, TRUE, 0, 0 },
                               { 0, 0, 0, 0 } } },
    [COMMAND_VERIFY_FILE] = { "verify-file",
                              { { OT_FILE, TRUE, 0, 0 },
                                { OT_LONGHEX, 0, 0, 0 },
                                { OT_YESNO, 0, 0, 0 } } },
    [COMMAND_WRITE_CRK] = { "write-crk",
                            { { OT_FILE, TRUE, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_REWRITE_CRK] = { "renew-crk",
                              { { OT_FILE, TRUE, 0, 0 }, { OT_FILE, TRUE, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_OTP] = { "write-otp",
                            { { OT_DATAHEX, TRUE, 2, 0 },
                              { OT_DATAHEX, TRUE, 4, 0 },
                              { 0, 0, 0, 0 } } },
    [COMMAND_ECHO] = { "echo", { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_TIMEOUT] = { "write-timeout",
                                { { OT_SCP_TARGET, TRUE, 0, 0 },
                                  { OT_DATAHEX, TRUE, 2, 0 },
                                  { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_PARAMS] = { "write-param",
                               { { OT_SCP_TARGET, TRUE, 0, 0 },
                                 { OT_DATAHEX, TRUE, 4, 0 },
                                 { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_STIM] = { "write-stim",
                             { { OT_SCP_TARGET, TRUE, 0, 0 },
                               { OT_DATAHEX, TRUE, 4, 0 },
                               { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_DEACT] = { "write-deact",
                              { { OT_SCP_TARGET, TRUE, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_KILL_CHIP] = { "kill-chip", { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_KILL_CHIP2] = { "kill-chip2",
                             { { OT_DATAHEX, TRUE, USN_LEN, USN_LEN },
                               { 0, 0, 0, 0 },
                               { 0, 0, 0, 0 } } },
    [COMMAND_EXECUTE_CODE] = { "execute-code",
                               { { OT_LONGHEX, TRUE, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_WRITE_APP_VER] = { "write-app-ver",
                                { { OT_LONGHEX, TRUE, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_LOAD_CUSTOMER_KEY] = { "load-customer-key",
                                             { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_ERASE_CODE_FLASH_AREA] = { "erase-code-flash-area",
                                                 { { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_ERASE_ALL_FLASH_AREAS] = { "erase-all-flash-areas",
                                                 { { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_LOAD_CODE] = { "load-code",
                                     { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_LOAD_FILE] = { "load-file",
                                     { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_VERIFY_FILE] = { "verify-1852-file",
                                       { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_LOAD_DATA] = { "load-data",
                                     { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_VERIFY_CODE] = { "verify-code",
                                       { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_VERIFY_DATA] = { "verify-data",
                                       { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_WRITE_REGISTER] = { "write-register",
                                          { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_READ_REGISTER] = { "read-register",
                                         { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_ENGAGE_PLLO] = { "engage-pllo",
                                       { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_VERIFY_CUSTOMER_KEY] = { "verify-customer-key",
                                               { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_ACTIVATE_CUSTOMER_KEY] = { "activate-customer-key",
                                                 { { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 },
                                                   { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_GENERATE_APPLICATION_STARTUP_SIGNATURE] = { "generate-application-startup-signature",
                                                                  { { 0, 0, 0, 0 },
                                                                    { 0, 0, 0, 0 },
                                                                    { 0, 0, 0, 0 } } },
    [COMMAND_MAXQ1852_VERIFY_APPLICATION_STARTUP_SIGNATURE] = { "verify-application-startup-signature",
                                                                { { 0, 0, 0, 0 },
                                                                  { 0, 0, 0, 0 },
                                                                  { 0, 0, 0, 0 } } },

    [COMMAND_SCP_LITE_LOAD_RAM] = { "load-ram",
                                    { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } },
    [MAX_SCP_COMMAND] = { 0, { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } } }
};

#endif /* __SCP_DEFINITIONS_H__ */
