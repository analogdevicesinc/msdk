/*******************************************************************************
* Copyright (C) 2022 Maxim Integrated Products, Inc., All rights Reserved.
* 
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*/

#ifndef I2C_MNGR_H
#define I2C_MNGR_H

#include <stdbool.h>

#include "i2c_regs.h"
#include "mxc_device.h"

/*
 * @brief I2C slave device configuration
 */
typedef struct {
    mxc_i2c_regs_t *i2c_instance; ///< I2C peripheral instance
    uint8_t slave_addr; ///< I2C slave
    uint32_t freq; ///< I2C bus frequency
    uint32_t timeout; ///< I2C transaction timeout
    bool clock_stretching; ///< I2C clock stretching flag
} i2c_mngr_slv_config_t;

/*
 * @brief I2C transaction
 */
typedef struct {
    i2c_mngr_slv_config_t *slave_config; ///< I2C device configuration
    uint8_t *p_rx_data; ///< RX data buffer pointer
    uint32_t rx_len; ///< RX data length
    uint8_t *p_tx_data; ///< TX data buffer pointer
    uint32_t tx_len; ///< TX data length
} i2c_mngr_txn_t;

/* Function prototypes */

/*
 * @brief Initializes I2C transaction manager
 * @return #E_NO_ERROR if succeeded, error code otherwise
 */
int I2C_MNGR_Init();

/*
 * @brief Executes I2C transaction
 * @param transaction The trancaction to execute
 * @return #E_NO_ERROR if succeeded, error code otherwise
 */
int I2C_MNGR_Transact(const i2c_mngr_txn_t *transaction);

#endif // I2C_MNGR_H
