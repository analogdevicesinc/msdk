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

typedef struct _i3c_ccc_req_t mxc_i3c_ccc_req_t;
typedef struct _i3c_req_t mxc_i3c_req_t;

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
 * @brief   Transfer type enumeration.
 * 
 */
typedef enum {
    MXC_I3C_TRANSFER_TYPE_READ = 0,
    MXC_I3C_TRANSFER_TYPE_WRITE = 1
} mxc_i3c_transfer_type_t;

/**
 * @brief   IBI types.
 *
 */
typedef enum {
    MXC_I3C_IBI_TYPE_NONE = MXC_V_I3C_CONT_STATUS_IBITYPE_NONE, ///<
    MXC_I3C_IBI_TYPE_IBI = MXC_V_I3C_CONT_STATUS_IBITYPE_IBI, ///<
    MXC_I3C_IBI_TYPE_CONTROLLER_REQ = MXC_V_I3C_CONT_STATUS_IBITYPE_CONT_REQ, ///<
    MXC_I3C_IBI_TYPE_HOTJOIN_REQ = MXC_V_I3C_CONT_STATUS_IBITYPE_HOTJOIN_REQ, ///<
} mxc_i3c_ibi_type_t;

/* CCC request flags */
#define MXC_I3C_CCC_HAS_DEFINING_BYTE (1U << 0)
#define MXC_I3C_CCC_HAS_SUB_COMMAND (1U << 1)
#define MXC_I3C_CCC_HAS_DATA (1U << 2)

/**
 * @brief I3C configuration options used during initialization.
 * 
 */
typedef struct {
    bool target_mode; ///< If the driver should be initialized in target mode.
    uint8_t static_addr; ///< Static address to use if target mode is used.
    uint32_t pp_hz; ///< SCL frequency to be used in push-pull operation.
    uint32_t od_hz; ///< SCL frequency to be used in open-drain operation.
    uint32_t i2c_hz; ///< SCL frequency to be used in I2C operation.
} mxc_i3c_config_t;

/**
 * @brief Common Command Code request structure.
 *
 * Broadcast and direct commands are distinguished by MSB, i.e. if MSB being set
 * implies a direct CCC while MSB being 0 implies a broadcast CCC.
 *
 */
struct _i3c_ccc_req_t {
    uint8_t ccc; ///< CCC command to send.
    uint8_t target_addr; ///< Target address if CCC is a direct command. Ignored
    ///< if CCC is a broadcast command.
    mxc_i3c_transfer_type_t xfer_type; ///< Transfer type. Ignored for broadcast CCCs.
    uint8_t flags; ///< See request flags above.
    uint8_t def_byte; ///< Optional defining byte. Defined by CCC.
    uint8_t sub_cmd; ///< Optional sub-command. Defined by CCC. Only used by direct
    ///< CCCs.
    unsigned char *tx_buf; ///< Optional data bytes to send.
    uint8_t tx_len; ///< Length of optional data.
    unsigned char *rx_buf; ///< Optional data bytes to read.
    uint8_t rx_len; ///< Length of optional data.
};

/**
 * @brief Private SDR/I2C request structure.
 *
 * SDR/I2C read and write to an I3C target.
 *
 */
struct _i3c_req_t {
    uint8_t target_addr; ///< Target address.
    bool is_i2c; ///< If this is a legacy I2C transfer.
    bool stop; ///< Send a STOP after the transaction.
    unsigned char *tx_buf; ///< Optional data bytes to send.
    uint16_t tx_len; ///< Length of optional data.
    unsigned char *rx_buf; ///< Optional data bytes to read.
    uint16_t rx_len; ///< Length of optional data.
};

/**
 * @brief   IBI payload request callback. Write additional byte to \a byte.
 *
 * This function will be called as long as non-zero is returned.
 *
 * @return  Non-zero if a byte is written to \a byte, 0 to indicate no more additional
 * bytes left to send.
 */
typedef int (*mxc_i3c_ibi_getbyte_t)(mxc_i3c_regs_t *i3c, unsigned char *byte);

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
 * @param   config      Config options. See \ref mxc_i3c_config_t for details.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Init(mxc_i3c_regs_t *i3c, mxc_i3c_config_t *config);

/**
 * @brief   Recover I3C bus if necessary. Should only be used in controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Recover(mxc_i3c_regs_t *i3c);

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
 * Should only be used in controller mode.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 *
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetPPFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C push-pull mode. Should only be used in 
 * controller mode.
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
 * MXC_I3C_SetPPFrequency(). Should only be used in controller mode.
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
 * @brief   Get the frequency of the I3C open-drain mode. Should only be used in 
 * controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  The I3C open-drain frequency in hertz.
 */
unsigned int MXC_I3C_GetODFrequency(mxc_i3c_regs_t *i3c);

/**
 * @brief   Sets the SCL frequency for I2C mode. Should only be used in controller mode.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   frequency     Frequency in hertz.
 *
 * @return  Negative if error, otherwise actual speed set. See \ref
 *          MXC_Error_Codes for the list of error return codes.
 */
int MXC_I3C_SetI2CFrequency(mxc_i3c_regs_t *i3c, unsigned int frequency);

/**
 * @brief   Get the frequency of the I3C in I2C mode. Should only be used in controller 
 * mode.
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
 * least 4. See MXC_I3C_SetPPFrequency(). Should only be used in controller mode.
 *
 * @param   i3c           Pointer to I3C registers (selects the I3C block used).
 * @param   skew          Skew value in units of peripheral clock cycles. Cannot
 *                        be greater than 7.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_SetSkew(mxc_i3c_regs_t *i3c, uint8_t skew);

/**
 * @brief   Sets the high-keeper implementation for the device. Should only be used 
 * in controller mode.
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
 * @brief   Emit an I2C or I3C start in controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   isI2C       True if I2C, false if I3C.
 * @param   xferType    Transfer type. See \ref mxc_i3c_transfer_type_t.
 * @param   addr        Target address.
 * @param   readCount   Number of bytes to read if this is a read message.
 *
 * @return  Success/Fail. E_SUCCESS if request is successful, E_BUSY if an IBI occurrs,
 *          one of \ref MXC_Error_Codes otherwise.
 */
int MXC_I3C_EmitStart(mxc_i3c_regs_t *i3c, bool isI2C, mxc_i3c_transfer_type_t xferType,
                      uint8_t addr, uint8_t readCount);

/**
 * @brief   Emit an I3C target reset pattern in controller mode.
 * 
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Success/Fail. See \ref MXC_Error_Codes for possible return codes.
 */
int MXC_I3C_ResetTarget(mxc_i3c_regs_t *i3c);

/**
 * @brief   Emit an I2C STOP in controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_EmitI2CStop(mxc_i3c_regs_t *i3c);

/**
 * @brief   Emit an I3C STOP in controller mode.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_EmitStop(mxc_i3c_regs_t *i3c);

/**
 * @brief   Send or broadcast a Common Command Code (CCC).
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   req         CCC request data.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Controller_CCC(mxc_i3c_regs_t *i3c, const mxc_i3c_ccc_req_t *req);

/**
 * @brief   Perform a private SDR or a legacy I2C transfer.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   req         Request data.
 *
 * @return  Success/Fail, see \ref MXC_Error_Codes for a list of return codes.
 */
int MXC_I3C_Controller_Transaction(mxc_i3c_regs_t *i3c, const mxc_i3c_req_t *req);

/**
 * @brief   Start dynamic address assignment.
 *
 * If E_SUCCESS is returned, PID, BCR and DCR are read into their corresponding buffers.
 * Call MXC_I3C_ControllerDAA again with a valid \a addr to assign a dynamic address to
 * the target and read PID, BCR and DCR of the next target. If E_SHUTDOWN is returned,
 * DAA process is complete
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   pid         Buffer to store provisioned ID of a target.
 * @param   bcr         Buffer to store Bus Characteristics Register of a target.
 * @param   dcr         Buffer to store Device Characteristics Register of a target.
 *
 * @return  Success/Fail, E_SUCCESS if request is successful, E_SHUTDOWN if DAA is
 *          finished, E_FAIL if an error occurs during DAA, one of \ref MXC_Error_Codes
 *          otherwise.
 */
int MXC_I3C_Controller_DAA(mxc_i3c_regs_t *i3c, uint8_t addr, uint8_t *pid, uint8_t *bcr,
                           uint8_t *dcr);

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
 * @brief   Enter offline mode and stop participating on the bus. Valid only in target mode.
 *
 * Offline mode is only allowed if the target has already been assigned a dynamic address.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  E_SUCCESS if device switches to offline mode, E_BAD_STATE otherwise.
 */
int MXC_I3C_Standby(mxc_i3c_regs_t *i3c);

/**
 * @brief   Exit offline mode using previously assigned dynamic address. Valid only in
 * target mode.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  E_SUCCESS if device comes out of offline mode, E_BAD_STATE otherwise.
 */
int MXC_I3C_Wakeup(mxc_i3c_regs_t *i3c);

/**
 * @brief   Unloads bytes from the receive FIFO. Both controller and target mode can
 * use this function to read the receive FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   bytes       The buffer to read the data into.
 * @param   len         The number of bytes to read.
 * @param   timeout     Timeout in microseconds to wait. Use -1 to wait indefinitely.
 *
 * @return  The number of bytes actually read.
 */
int MXC_I3C_ReadRXFIFO(mxc_i3c_regs_t *i3c, volatile unsigned char *bytes, unsigned int len,
                       int timeout);

/**
 * @brief   Loads bytes into the transmit FIFO. Both controller and target mode can
 * use this function to write into the transmit FIFO.
 *
 * @param   i2c         Pointer to I3C registers.
 * @param   bytes       The buffer containing the bytes to write.
 * @param   len         The number of bytes to write.
 * @param   end         If set to true, last byte in \a bytes is marked as end-of-data.
 * @param   timeout     Timeout in microseconds to wait. Use -1 to wait indefinitely.
 *
 * @return  The number of bytes actually written.
 */
int MXC_I3C_WriteTXFIFO(mxc_i3c_regs_t *i3c, const unsigned char *bytes, unsigned int len, bool end,
                        int timeout);

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
int MXC_I3C_Controller_GetError(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear the error register.
 *
 * @param   i3c         Pointer to I3c registers.
 */
void MXC_I3C_Controller_ClearError(mxc_i3c_regs_t *i3c);

/**
 * @brief   Set the transmit threshold level.
 *
 * When operating as a controller, the function sets the transmit threshold level
 * for when the master should add additional bytes to the transmit FIFO. Can be used in
 * both controller and target mode.
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
 * @brief   Removes and discards all bytes currently in the receive FIFO. Valid in
 * both controller and target mode.
 *
 * @param   i3c         Pointer to I3C registers.
 */
void MXC_I3C_ClearRXFIFO(mxc_i3c_regs_t *i3c);

/**
 * @brief   Removes and discards all bytes currently in the transmit FIFO. Valid in
 * both controller and target mode.
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
unsigned int MXC_I3C_Controller_GetRXCount(mxc_i3c_regs_t *i3c);

/**
 * @brief   Return the number of bytes in transmit buffer or FIFO.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  Number of bytes in transmit buffer or FIFO.
 */
unsigned int MXC_I3C_Controller_GetTXCount(mxc_i3c_regs_t *i3c);

/**
 * @brief   Enable controller interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_Controller_EnableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Disable controller interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_Controller_DisableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Get the presently set interrupt flags.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  See \ref MXC_Error_Codes for a list of return values.
 */
unsigned int MXC_I3C_Controller_GetFlags(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear controller interrupts.
 *
 * Note that some bits cannot be cleared manually and self-clear only when their
 * respective conditions occur.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to clear.
 */
void MXC_I3C_Controller_ClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Enable target interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_Target_EnableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Disable target interrupts.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to set.
 */
void MXC_I3C_Target_DisableInt(mxc_i3c_regs_t *i3c, uint32_t mask);

/**
 * @brief   Get the presently set target mode interrupt flags.
 *
 * @param   i3c         Pointer to I3C registers.
 *
 * @return  See \ref MXC_Error_Codes for a list of return values.
 */
int MXC_I3C_Target_GetFlags(mxc_i3c_regs_t *i3c);

/**
 * @brief   Clear target interrupts.
 *
 * Note that some bits cannot be cleared manually and self-clear only when their
 * respective conditions occur.
 *
 * @param   i3c         Pointer to I3C registers.
 * @param   mask        Interrupt mask to clear.
 */
void MXC_I3C_Target_ClearFlags(mxc_i3c_regs_t *i3c, uint32_t mask);

/**@} end of group i3c */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32657_I3C_H_
