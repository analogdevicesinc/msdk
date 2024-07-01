/**
* @file     i3c.h
* @brief    Improved Inter Integrated Circuit (I3C) communications interface driver.
*/

/******************************************************************************
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I3C_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I3C_H_

#include <stdint.h>
#include <stdbool.h>
#include "mxc_sys.h"
#include "i3c_regs.h"
#include "dma_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup i3c I3C
 * @ingroup periphlibs
 * @{
 */

/***** Definitions *****/
typedef struct _i3c_target_t mxc_i3c_target_t;
typedef struct _i2c_target_t mxc_i3c_i2c_target_t;

/**
 * @brief Maximum supported IBI bytes.
 *
 */
#define MXC_I3C_MAX_IBI_BYTES 7U

/**
 * @brief Invalid I2C/I3C address.
 *
 */
#define MXC_I3C_ADDR_INVALID 0U

/**
 * @brief   The list of high-keeper options.
 *
 * This setting should match the high-keeper implementation of the device.
 */
typedef enum {
    MXC_I3C_HIGH_KEEPER_OFF = MXC_S_I3C_CONT_CTRL0_HKEEP_OFF, ///< No high-keeper support
    MXC_I3C_HIGH_KEEPER_ON_CHIP = MXC_S_I3C_CONT_CTRL0_HKEEP_ON_CHIP, ///< SCL and SDA pads
    ///< have weak pull-ups
    MXC_I3C_HIGH_KEEPER_EXT_SDA = MXC_S_I3C_CONT_CTRL0_HKEEP_EXT_SDA, ///< External high-keeper
    ///< support for SDA signal
    MXC_I3C_HIGH_KEEPER_EXT_SCL_SDA = MXC_S_I3C_CONT_CTRL0_HKEEP_EXT_SCL_SDA,
    ///< External high-keeper support for SCL and SDA signals
} mxc_i3c_high_keeper_t;

/**
 * @brief   The list of receive FIFO trigger levels.
 */
typedef enum {
    MXC_I3C_RX_TH_NOT_EMPTY = MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_NOT_EMPTY, ///<
    MXC_I3C_RX_TH_QUARTER_FULL = MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_QUARTER_FULL, ///<
    MXC_I3C_RX_TH_HALF_FULL = MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_HALF_FULL, ///<
    MXC_I3C_RX_TH_3_4_FULL = MXC_V_I3C_CONT_FIFOCTRL_RX_THD_LVL_3_QUARTER_FULL, ///<
} mxc_i3c_rx_threshold_t;

/**
 * @brief   The list of transmit FIFO trigger levels.
 */
typedef enum {
    MXC_I3C_TX_TH_EMPTY = MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_EMPTY, ///<
    MXC_I3C_TX_TH_QUARTER_FULL = MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_QUARTER_FULL, ///<
    MXC_I3C_TX_TH_HALF_FULL = MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_HALF_FULL, ///<
    MXC_I3C_TX_TH_ALMOST_FULL = MXC_V_I3C_CONT_FIFOCTRL_TX_THD_LVL_ALMOST_FULL, ///<
} mxc_i3c_tx_threshold_t;

/**
 * @brief IBI types.
 *
 */
typedef enum {
    MXC_I3C_IBI_TYPE_NONE = MXC_V_I3C_CONT_STATUS_IBITYPE_NONE, ///<
    MXC_I3C_IBI_TYPE_IBI = MXC_V_I3C_CONT_STATUS_IBITYPE_IBI, ///<
    MXC_I3C_IBI_TYPE_CONTROLLER_REQ = MXC_V_I3C_CONT_STATUS_IBITYPE_CONT_REQ, ///<
    MXC_I3C_IBI_TYPE_HOTJOIN_REQ = MXC_V_I3C_CONT_STATUS_IBITYPE_HOTJOIN_REQ, ///<
} mxc_i3c_ibi_type_t;

/**
 * @brief   IBI callback.
 *
 * When a target wins address arbitration and generates an IBI, this callback
 * function is called to get the application decision to ACK/NACK the IBI.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     The byte received.
 * @param   ibiType     IBI type. See \ref mxc_i3c_ibi_type_t for possible values.
 *
 * @return  0 if the IBI should not be acknowledged (NACK), non-zero to
 *          acknowledge the IBI.
 */
typedef int (*mxc_i3c_ibi_ack_t)(mxc_i3c_regs_t *i3c, unsigned char dynAddr,
                                 mxc_i3c_ibi_type_t ibiType);

/**
 * @brief   IBI request callback. Called after an IBI is acknowledged by the application
 * and mandatory and additional data bytes are read.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   target      Pointer to I3C target requesting an IBI.
 */
typedef void (*mxc_i3c_ibi_req_t)(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *target);

/**
 * @brief   IBI payload request callback. Write additional byte to \a byte.
 *
 * This function will be called as long as non-zero is returned.
 *
 * @return  Non-zero if a byte is written to \a byte, 0 to indicate no more additional
 * bytes left to send.
 */
typedef int (*mxc_i3c_ibi_getbyte_t)(mxc_i3c_regs_t *i3c, unsigned char *byte);

/**
 * @brief   CCC request callback.
 *
 * Called when the received CCC is not handled automatically.
 */
typedef void (*mxc_i3c_ccc_cb_t)(mxc_i3c_regs_t *i3c, unsigned char ccc);

/**
 * @brief   The information required to perform a complete I2C transaction as
 *          the bus master.
 *
 * The information required to perform a complete I2C transaction as the bus
 * master. This structure is used by the MXC_I2C_MasterTransaction() and
 * MXC_I2C_MasterTransactionAsync() functions.
 */
struct _i3c_target_t {
    uint8_t dynAddr; ///< Dynamic address of the I3C target.
    uint8_t staticAddr; ///< Static address of the I3C target. Set to 0 if target does
    ///< not have an I2C-style static address.
    uint64_t pid; ///< Provisioned ID.
    uint8_t bcr; ///< Bus characteristics register.
    uint8_t dcr; ///< Device characteristics register.
    uint8_t data[1 + MXC_I3C_MAX_IBI_BYTES]; ///< Mandatory byte plus additional bytes.
    uint8_t numBytes; ///< Number of data bytes.
};

struct _i2c_target_t {
    uint8_t staticAddr; ///< Target address of the I2C target.
};

/***** Function Prototypes *****/

/* ************************************************************************* */
/* Control/Configuration functions                                           */
/* ************************************************************************* */

/**
 * @brief   Initialize and enable I3C peripheral.
 *
 * @note    On default this function enables I3C peripheral clock and I3C gpio pins.
 *          If you wish to skip clock and gpio initialization, define MSDK_NO_GPIO_CLK_INIT
 *          flag in project.mk file.
 *
 * @param   i3c         Pointer to I3C registers (selects the I3C block used).
 * @param   targetMode  Whether to put the device in controller or target mode. Use
 *                      non-zero.
 * @param   ppHz        I3C push-pull frequency. Only valid in controller mode.
 * @param   odHz        I3C open-drain frequency. Only valid in controller mode.
 * @param   i2cHz       I2C frequency. Only valid in controller mode.
 * @param   staticAddr  I2C-style static address. Has no effect if the address is hardwired.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, int targetMode, uint8_t staticAddr, uint32_t ppHz,
                 uint32_t odHz, uint32_t i2cHz);

/**
 * @brief   Disable and shutdown I3C peripheral.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Shutdown(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I3C push-pull operation.
 *
 * Recommended value for push-pull frequency is fclk / 2, where fclk is the I3C
 * peripheral clock. Note that I3C supports a maximum frequency of 12.5MHz.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 *
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetPPFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C push-pull mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The I3C push-pull frequency in hertz.
 */
unsigned int MXC_I3C_GetPPFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I3C open-drain operation.
 *
 * Note that open-drain SCL also depends on push-pull SCL settings. See
 * MXC_I3C_SetPPFrequency().
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 * @param   highPP        Set SCL high period to high period in push-pull mode.
 *                        This is used to prevent legacy I2C devices from detecting
 *                        I3C messages.
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetODFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency, bool highPP);

/**
 * @brief   Get the frequency of the I3C open-drain mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The I3C open-drain frequency in hertz.
 */
unsigned int MXC_I3C_GetODFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I2C mode.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 *
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetI2CFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C in I2C mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The frequency of I2C mode in hertz.
 */
unsigned int MXC_I3C_GetI2CFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the skew value for I3C push-pull operation.
 *
 * Note that this setting requires peripheral clock (fclk) to SCL ratio to be at
 * least 4. See MXC_I3C_SetPPFrequency().
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   skew          Skew value in units of peripheral clock cycles. Cannot
 *                        be greater than 7.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetSkew(mxc_i3c_regs_t *i3c, uint8_t skew);

/**
 * @brief   Sets the high-keeper implementation for the device.
 *
 * See \ref mxc_i3c_high_keeper_t.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   hkeep         High-keeper option.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetHighKeeperMode(mxc_i3c_regs_t *i3c, mxc_i3c_high_keeper_t hkeep);

/**
 * @brief   Set the I3C targets connected to this controller instance.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   targets     Pointer to I3C targets.
 * @param   numTargets  Number of I3C targets.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetI3CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_target_t *targets, uint8_t numTargets);

/**
 * @brief   Set the I3C targets connected to this controller instance.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   targets     Pointer to I2C targets.
 * @param   numTargets  Number of I2C targets.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetI2CTargets(mxc_i3c_regs_t *i3c, mxc_i3c_i2c_target_t *targets, uint8_t numTargets);

/**
 * @brief   Set IBI handlers. Only required for controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   ackCb       Called when an IBI needs to be ACKed or NACKed.
 * @param   reqCb       Called when mandatory and additional bytes are read.
 */
void MXC_I3C_SetIBICallback(mxc_i3c_regs_t *i3c, mxc_i3c_ibi_ack_t ackCb, mxc_i3c_ibi_req_t reqCb);

/**
 * @brief   Set IBI payload callback. Only required for target mode.
 *
 * Application will write IBI payload using this callback.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   payloadCb   Called when application needs provide data bytes to send with the
 *                      IBI.
 */
void MXC_I3C_SetIBIPayloadCallback(mxc_i3c_regs_t *i3c, mxc_i3c_ibi_getbyte_t payloadCb);

/**
 * @brief   Set CCC command callback. Only required for target mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   cccCb       Called when application needs to handle a CCC command.
 */
void MXC_I3C_SetCCCCallback(mxc_i3c_regs_t *i3c, mxc_i3c_ccc_cb_t cccCb);

/**
 * @brief   Emit an I2C or I3C start.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   i2c         True if I2C, false if I3C.
 * @param   readWrite   Direction of transfer. 1 if read, 0 if write.
 * @param   addr        Target address.
 * @param   readCount   Number of bytes to read if this is a read message.
 *
 * @return  Success/Fail. E_SUCCESS if request is successful, E_BUSY if an IBI occurrs,
 *          one of \ref MXC_Error_Codes otherwise.
 */
int MXC_I3C_Start(mxc_i3c_regs_t *i3c, bool i2c, uint8_t readWrite, uint8_t addr,
                  uint8_t readCount);

/**
 * @brief   Broadcast a Common Command Code (CCC) to all I3C targets.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   ccc         Common command code to broadcast.
 * @param   defByte     Optional defining byte. Only LSB 8 bits will be used.
 *                      Set to -1 if not required.
 * @param   data        Optional data to send with broadcast CCC.
 * @param   len         Length of optional data. Can only be 0, 1 or 2.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_BroadcastCCC(mxc_i3c_regs_t *i3c, unsigned char ccc, int defByte, unsigned char *data,
                         int len);

/**
 * @brief   Perform dynamic address assignment.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_PerformDAA(mxc_i3c_regs_t *i3c);

/**
 * @brief   Generate a hot-join request in target mode.
 *
 * Hot-Join will only be generated if the target is powered on after the bus is
 * configured or physically connected to an already configured bus. If the active
 * controller disables the Hot-Join events through DISEC CCCs, it will not be
 * generated either.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  E_SUCCESS if request is submitted. E_BAD_STATE if operation is not allowed in
 *          the current state. E_NOT_SUPPORTED is returned in case HotJoin generation is not
 *          supported. Note that return value of E_SUCCESS does not guarantee a successful
 *          Hot-Join since the decision is up to the controller.
 */
int MXC_I3C_HotJoin(mxc_i3c_regs_t *i3c);

/**
 * @brief   Generate an In-Band Interrupt in target mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mdb         Mandatory data byte to be sent if IBI is acknowledged.
 * @param   getByteCb   Callback to get additional databytes from the application. Pass NULL
 *                      if no additional data bytes except \a mdb will be sent with the IBI
 *                      request.
 *
 * @return  E_SUCCESS if request is submitted. E_BAD_STATE if operation is not allowed in
 *          the current state. E_NOT_SUPPORTED is returned in case IBI generation is not
 *          supported. Note that return value of E_SUCCESS does not guarantee that the
 *          request has been acknowledged.
 */
int MXC_I3C_RequestIBI(mxc_i3c_regs_t *i3c, unsigned char mdb, mxc_i3c_ibi_getbyte_t getByteCb);

/**
 * @brief   Enter offline mode and stop participating on the bus.
 *
 * Offline mode is only allowed if the target has already been assigned a dynamic address.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  E_SUCCESS if device switches to offline mode, E_BAD_STATE otherwise.
 */
int MXC_I3C_Standby(mxc_i3c_regs_t *i3c);

/**
 * @brief   Exit offline mode using previously assigned dynamic address.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  E_SUCCESS if device comes out of offline mode, E_BAD_STATE otherwise.
 */
int MXC_I3C_Wakeup(mxc_i3c_regs_t *i3c);

/**
 * @brief   Read multiple bytes from an I2C target in blocking mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   staticAddr  7-bit target address to read from.
 * @param   bytes       The buffer to read data into.
 * @param   len         The number of bytes to read. On return from this function,
 *                      this will be set to the number of bytes actually received.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_ReadI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes,
                            unsigned int *len);

/**
 * @brief   Write multiple bytes to an I2C target in blocking mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   staticAddr  7-bit target address to read from.
 * @param   bytes       The buffer containing bytes to transmit.
 * @param   len         The number of bytes to transmit. On return from this function,
 *                      this will be set to the number of bytes actually received.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_WriteI2CBlocking(mxc_i3c_regs_t *i3c, unsigned char staticAddr, unsigned char *bytes,
                             unsigned int *len);

/**
 * @brief   Read multiple bytes from an I3C target in blocking mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     7-bit target dynamic address to read from.
 * @param   len         The number of bytes to read. On return from this function,
 *                      this will be set to the number of bytes actually received.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_ReadSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                            unsigned int *len);

/**
 * @brief   Write multiple bytes to an I3C target in blocking mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   dynAddr     7-bit target dynamic address to write to.
 * @param   len         The number of bytes to write. On return from this function,
 *                      this will be set to the number of bytes actually sent.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_WriteSDRBlocking(mxc_i3c_regs_t *i3c, unsigned char dynAddr, unsigned char *bytes,
                             unsigned int *len);

/**
 * @brief   Unloads bytes from the receive FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 *
 * @return  The number of bytes actually read.
 */
int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len);

/**
 * @brief   Loads bytes into the transmit FIFO.
 *
 * @param   i2c         Pointer to I3C registers.
 * @param   bytes       The buffer containing the bytes to write.
 * @param   len         The number of bytes to write.
 *
 * @return  The number of bytes actually written.
 */
int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len);

/**
 * @brief   Emit an I2C STOP.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_I2CStop(mxc_i3c_regs_t *i3c);

/**
 * @brief   Emit an I3C STOP.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_Stop(mxc_i3c_regs_t *i3c);

/**
 * @brief   Return the current error code if any.
 *
 * @param   i3c         Pointer to I3C registers.
 * @return  E_NO_ERROR if no error found.
 * E_NO_RESPONSE if request NACKed.
 * E_ABORT if write aborted.
 * E_BAD_STATE if request is invalid or a message mode error occurs.
 * E_TIME_OUT if a timeout error occurred.
 * E_OVERFLOW in case of read data underrun or write data overflow.
 * E_FAIL otherwise.
 */
int MXC_I3C_GetError(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear the error register.
 *
 * @param   i3c         Pointer to I3c registers.
 */
void MXC_I3C_ClearError(mxc_i3c_regs_t *i3c);

/**
 * @brief   Set the transmit threshold level.
 *
 * When operating as a controller, the function sets the transmit threshold level
 * for when the master should add additional bytes to the transmit FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   rxth        Receive threshold level to set. See \ref mxc_i3c_rx_threshold_t
 *                      for available options.
 * @param   txth        Transmit threshold level to set. See \ref mxc_i3c_tx_threshold_t
 *                      for available options.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetRXTXThreshold(mxc_i3c_regs_t *i3c, mxc_i3c_rx_threshold_t rxth,
                             mxc_i3c_tx_threshold_t txth);

/**
 * @brief   Returns the dynamic address in target mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Dynamic address set by bus controller. If the address is invalid or
 *          device is not running in target mode, then 0 is returned.
 */
uint8_t MXC_I3C_GetDynamicAddress(mxc_i3c_regs_t *i3c);

/**
 * @brief   Interrupt handler.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_IRQHandler(mxc_i3c_regs_t *i3c);

/**
 * @brief   Removes and discards all bytes currently in the receive FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_ClearRXFIFO(mxc_i3c_regs_t *i3c);

/**
 * @brief   Removes and discards all bytes currently in the transmit FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_ClearTXFIFO(mxc_i3c_regs_t *i3c);

/**
 * @brief   Return the number of bytes in receive buffer or FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Number of bytes in receive buffer or FIFO.
 */
unsigned int MXC_I3C_ControllerGetRXCount(mxc_i3c_regs_t *i3c);

/**
 * @brief   Return the number of bytes in transmit buffer or FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Number of bytes in transmit buffer or FIFO.
 */
unsigned int MXC_I3C_ControllerGetTXCount(mxc_i3c_regs_t *i3c);

/**
 * @brief   Enable controller interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_ControllerEnableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Disable controller interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_ControllerDisableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Get the presently set interrupt flags.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  See \ref MXC_Error_Codes for a list of return values.
 */
unsigned int MXC_I3C_ControllerGetFlags(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear controller interrupts.
 *
 * Note that some bits cannot be cleared manually and self-clear only when their
 * respective conditions occur.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to clear.
 */
void MXC_I3C_ControllerClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Enable target interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_TargetEnableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Disable target interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_TargetDisableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Get the presently set target mode interrupt flags.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  See \ref MXC_Error_Codes for a list of return values.
 */
int MXC_I3C_TargetGetFlags(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear target interrupts.
 *
 * Note that some bits cannot be cleared manually and self-clear only when their
 * respective conditions occur.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to clear.
 */
void MXC_I3C_TargetClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask);

/**@} end of group i3c */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I3C_H_
