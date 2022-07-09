__author__ = 'bvinot'


K_MODULE_LCM = 0x01
K_MODULE_SH = 0x02
K_MODULE_PM = 0x03
K_MODULE_MM = 0x04
K_MODULE_BM = 0x05
K_MODULE_TE = 0x06
K_MODULE_TS = 0x07
K_MODULE_TL = 0x08
K_MODULE_CM = 0x09
K_MODULE_RCE = 0x0a
K_MODULE_COMMON = 0x0b

ERROR_MSG = x = [['                                                                        ' for i in range(35)] for j in range(14)]

#################################### CM ###########################################

ERROR_MSG[K_MODULE_CM][1] = "ERR_NOT_INITIALIZED : CM module not initialized"
ERROR_MSG[K_MODULE_CM][2] = "ERR_NOT_AVAILABLE : Parameter is not available"
ERROR_MSG[K_MODULE_CM][3] = "ERR_CORRUPTED : Parameter is corrupted"
ERROR_MSG[K_MODULE_CM][4] = "ERR_NO_PARAM: Asked parameter does not exist"
ERROR_MSG[K_MODULE_CM][5] = "ERR_BAD_LENGTH : Length of parameters does not match"
ERROR_MSG[K_MODULE_CM][6] = "ERR_LOCK_PARAM : Parameter has been read as OTP LOCK value"
ERROR_MSG[K_MODULE_CM][7] = "ERR_USN_CV_NO_MATCH : USN CV does not match"
ERROR_MSG[K_MODULE_CM][8] = "ERR_PARAM_INVALID_OP : Operation on parameter is invalid"
ERROR_MSG[K_MODULE_CM][9] = "ERR_PARAM_NO_OP : Operation on parameter is not needed"
ERROR_MSG[K_MODULE_CM][10] = "ERR_PARAM_BAD_VALUE : Value of parameter is not appropriated"
ERROR_MSG[K_MODULE_CM][11] = "ERR_CANT_PROCEED : Command can not be executed"
ERROR_MSG[K_MODULE_CM][12] = "ERR_UNKNOWN : Generic error for unknown behavior"


################################### COMMON ########################################

ERROR_MSG[K_MODULE_COMMON][1] = "ERR_INVAL : Value is not appropriate"
ERROR_MSG[K_MODULE_COMMON][2] = "ERR_NULL_PTR : Pointer is null"
ERROR_MSG[K_MODULE_COMMON][3] = "ERR_OUT_OF_RANGE : Value is out of expected range"
ERROR_MSG[K_MODULE_COMMON][4] = "ERR_NOT_INITIALIZED : Module not initialized"
ERROR_MSG[K_MODULE_COMMON][5] = "ERR_ALREADY_INITIALIZED : Module already initialized"
ERROR_MSG[K_MODULE_COMMON][6] = "ERR_FATAL_ERROR : Critical error"
ERROR_MSG[K_MODULE_COMMON][7] = "ERR_RUNNING: Still processing"
ERROR_MSG[K_MODULE_COMMON][8] = "ERR_BAD_STATE: Action not allowed in this state"
ERROR_MSG[K_MODULE_COMMON][9] = "ERR_NO_MATCH: Data does not match"
ERROR_MSG[K_MODULE_COMMON][10] = "ERR_NOT_SUPPORTED: Operation is not supported"
ERROR_MSG[K_MODULE_COMMON][11] = "ERR_UNKNOWN: Generic error for unkown behavior"

################################## BM #############################################

ERROR_MSG[K_MODULE_BM][1] = "ERR_NOT_INITIALIZED : BM module not initialized"
ERROR_MSG[K_MODULE_BM][2] = "ERR_NOT_AVAILABLE : Bus is not available"
ERROR_MSG[K_MODULE_BM][3] = "ERR_TX : Bus TX error"
ERROR_MSG[K_MODULE_BM][4] = "ERR_RX: Bus RX error"
ERROR_MSG[K_MODULE_BM][5] = "ERR_CONN_LOST: Bus connection lost"
ERROR_MSG[K_MODULE_BM][6] = "ERR_UART_NOT_INITIALIZED: UART can not be intialized"
ERROR_MSG[K_MODULE_BM][7] = "ERR_USB_INIT_ERROR : USB initialisation error"
ERROR_MSG[K_MODULE_BM][8] = "ERR_USB_CLOSE_ERROR : USB close error"
ERROR_MSG[K_MODULE_BM][9] = "ERR_SPI_NO_PARAMS : Miss parameters"
ERROR_MSG[K_MODULE_BM][10] = "ERR_UNKNOWN : Generic error for unknown behavior"


##################################### LCM #########################################

ERROR_MSG[K_MODULE_LCM][1] = "ERR_NOT_INITIALIZED : LCM module not initialized"
ERROR_MSG[K_MODULE_LCM][2] = "ERR_NO_LIFECYCLE : Life Cycle Phase can not be retrieved"
ERROR_MSG[K_MODULE_LCM][3] = "ERR_BAD_LIFECYCLE : Life Cycle action does not match Platform's Life Cycle"
ERROR_MSG[K_MODULE_LCM][4] = "ERR_LIFECYCLE_NO_UP : Life cycle value is not to be updated"
ERROR_MSG[K_MODULE_LCM][5] = "ERR_NO_FTM : This error indicates that no FTM's magic value is OTP"
ERROR_MSG[K_MODULE_LCM][6] = "ERR_NO_JTAG : This error indicates that JTAG is not available"
ERROR_MSG[K_MODULE_LCM][7] = "ERR_FATAL_ERROR : Critical error"
ERROR_MSG[K_MODULE_LCM][8] = "ERR_LIFECYCLE_UPDATE : Life Cycle Phase value has to be updated !!! not really an error"
ERROR_MSG[K_MODULE_LCM][9] = "ERR_UNKNOWN: Generic error for unknown behavior"


#################################### MM ###########################################

ERROR_MSG[K_MODULE_MM][1] = "ERR_NOT_INITIALIZED : BM module not initialized"
ERROR_MSG[K_MODULE_MM][2] = "ERR_LENGTH_NOT_ALIGNED : Data is not aligned according to memory specification"
ERROR_MSG[K_MODULE_MM][3] = "ERR_ADDRESS_NOT_ALIGNED : Address is not aligned according to memory specification"
ERROR_MSG[K_MODULE_MM][4] = "ERR_NOT_ALLOWED : Memory area is not allowed"
ERROR_MSG[K_MODULE_MM][5] = "ERR_NOT_ACCESSIBLE: Memory is not accessible"
ERROR_MSG[K_MODULE_MM][6] = "ERR_OVERFLOW: Data length is out of memory capacity"
ERROR_MSG[K_MODULE_MM][7] = "ERR_UCI_ENCR_WRONG_KEY: Wrong UCI encryption key source"
ERROR_MSG[K_MODULE_MM][8] = "ERR_UCI_INTEG_WRONG_KEY: Wrong UCI integrity key source"
# OTP part #
ERROR_MSG[K_MODULE_MM][9] = "ERR_OTP_PGM_FAILED: OTP programmation failed"
ERROR_MSG[K_MODULE_MM][10] = "ERR_OTP_FAILURE: OTP internal failure"
# NAND part #
ERROR_MSG[K_MODULE_MM][11] = "ERR_NAND_ECC_ONEBITCORRECTED"
ERROR_MSG[K_MODULE_MM][12] = "ERR_NAND_ECC_ONEBITERRORINECC"
ERROR_MSG[K_MODULE_MM][13] = "ERR_NAND_ECC_FAIL"
ERROR_MSG[K_MODULE_MM][14] = "ERR_NAND_GENERIC: Generic error in NAND source code"
# Secure NVSRAM part #
ERROR_MSG[K_MODULE_MM][15] = "ERR_AES_GENERATION_FAILED: AES key generation failed"
ERROR_MSG[K_MODULE_MM][16] = "ERR_UNKNOWN : Generic error for unknown behavior"

####################################### PM ########################################

ERROR_MSG[K_MODULE_PM][0] = "ERR_NOT_INITIALIZED : PM module not initialized"
ERROR_MSG[K_MODULE_PM][1] = "ERR_NO_PROTOCOL : Wanted protocol does not exist"

# STP errors #
ERROR_MSG[K_MODULE_PM][2] = "ERR_STP_NOT_INITIALIZED : STP can not be initialized"
ERROR_MSG[K_MODULE_PM][3] = "ERR_NO_STP_CMD : No STP command corresponds to this identifier"
ERROR_MSG[K_MODULE_PM][4] = "ERR_NO_SCP_CMD : No SCP command corresponds to this identifier"
ERROR_MSG[K_MODULE_PM][5] = "ERR_STP_BAD_CMD_SEQ : Bad STP command sequence"
ERROR_MSG[K_MODULE_PM][6] = "ERR_STP_TIMEOUT : STP listening window times out"
ERROR_MSG[K_MODULE_PM][7] = "ERR_STP_NO_RWK_MW : No Magic Word read"

# SCP errors #
ERROR_MSG[K_MODULE_PM][8] = "ERR_SCP_NOT_INITIALIZED : SCP can not be initialized"
ERROR_MSG[K_MODULE_PM][9] = "ERR_SCP_NO_MORE_MEMORY : No more space left for SCP "
ERROR_MSG[K_MODULE_PM][10] = "ERR_SCP_NO_TIMEOUT : No timeout value, use ROM code's one"
ERROR_MSG[K_MODULE_PM][11] = "ERR_SCP_WRONG_CHECKSUM : Wrong Checksum"
ERROR_MSG[K_MODULE_PM][12] = "ERR_SCP_WRONG_SEC_LVL"
ERROR_MSG[K_MODULE_PM][13] = "ERR_SCP_WRONG_MODE"
ERROR_MSG[K_MODULE_PM][14] = "ERR_SCP_WRONG_CMD : Received command does not match with platform context"
ERROR_MSG[K_MODULE_PM][15] = "ERR_SCP_WRONG_STRING: Wrong characters string received"
ERROR_MSG[K_MODULE_PM][16] = "ERR_SCP_AUTOBOOT_TIMEOUT_END"
ERROR_MSG[K_MODULE_PM][17] = "ERR_SCP_NETWORK_DOWN"
ERROR_MSG[K_MODULE_PM][18] = "ERR_SCP_NOT_CONNECTED"
ERROR_MSG[K_MODULE_PM][19] = "ERR_SCP_CNX_TIMEOUT"
ERROR_MSG[K_MODULE_PM][20] = "ERR_SCP_NET_BAD_CONFIG"
ERROR_MSG[K_MODULE_PM][21] = "ERR_SCP_NET_WRONG_PROFILE"
ERROR_MSG[K_MODULE_PM][22] = "ERR_SCP_NET_WRONG_AUTHENT"
ERROR_MSG[K_MODULE_PM][23] = "ERR_SCP_NET_WRONG_SEQ"
ERROR_MSG[K_MODULE_PM][24] = "ERR_SCP_NET_UNKNOWN"
ERROR_MSG[K_MODULE_PM][25] = "ERR_SCP_BAD_PARAMS"

# SCP Applet errors #
ERROR_MSG[K_MODULE_PM][26] = "ERR_SCP_APLT_NO_SYNC : Synchronization pattern is not present in header"
ERROR_MSG[K_MODULE_PM][27] = "ERR_SCP_APLT_VERSION_MISMATCH : Applet target version does not match ROM code's one"
ERROR_MSG[K_MODULE_PM][28] = "ERR_SCP_APLT_WD_ERROR : an error occurs with WRITE_DATA primitive treatment"
ERROR_MSG[K_MODULE_PM][29] = "ERR_SCP_APLT_CD_ERROR : an error occurs with COMPARE_DATA primitive treatment"
ERROR_MSG[K_MODULE_PM][30] = "ERR_SCP_APLT_ED_ERROR : an error occurs with ERASE_DATA primitive treatment"
ERROR_MSG[K_MODULE_PM][31] = "ERR_SCP_APLT_NO_REGISTERED : No applet registered"
ERROR_MSG[K_MODULE_PM][32] = "ERR_SCP_APLT_NOT_HANDLED : Not handled by applet"

# SCP application command errors #
ERROR_MSG[K_MODULE_PM][33] = "ERR_SCP_APPLI_COMPARE_FAILURE : Compare failed"
ERROR_MSG[K_MODULE_PM][34] = "ERR_UNKNOWN : Generic error for unknown behavior"


####################################### RCE #######################################

ERROR_MSG[K_MODULE_RCE][1] = "ERR_NOT_INITIALIZED : RCE module not initialized"
ERROR_MSG[K_MODULE_RCE][2] = "ERR_NO_H_MV : Header magic value not found"
ERROR_MSG[K_MODULE_RCE][3] = "ERR_BAD_SIGNATURE : Digital signature does not match"
ERROR_MSG[K_MODULE_RCE][4] = "ERR_BAD_LA : Bad load address"
ERROR_MSG[K_MODULE_RCE][5] = "ERR_BAD_BIN_LENGTH : Bad binary length"
ERROR_MSG[K_MODULE_RCE][6] = "ERR_BAD_FORMAT : Binary format is not respected"
ERROR_MSG[K_MODULE_RCE][7] = "ERR_ARGS_FAILURE : Arguments for 2nd level application unavailable"
ERROR_MSG[K_MODULE_RCE][8] = "ERR_BAD_BIN_FMT_VER : Binary format version does not match"
ERROR_MSG[K_MODULE_RCE][9] = "ERR_BOOTSRC_NOT_SUPPORTED : Boot source not supported"
ERROR_MSG[K_MODULE_RCE][10] = "ERR_UNKNOWN : Generic error for unknown behavior"

###################################################################################
