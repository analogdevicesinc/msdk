#ifndef I2C_MNGR_H
#define I2C_MNGR_H

#include <stdbool.h>

#include "i2c_regs.h"
#include "mxc_device.h"

/*
 * @brief I2C slave device configuration
 */
typedef struct {
	mxc_i2c_regs_t* i2c_instance;  ///< I2C peripheral instance
	uint8_t slave_addr;            ///< I2C slave
	uint32_t freq;                 ///< I2C bus frequency
	uint32_t timeout;              ///< I2C transaction timeout
	bool clock_stretching;         ///< I2C clock stretching flag
} i2c_mngr_slv_config_t;

/*
 * @brief I2C transaction
 */
typedef struct {
	i2c_mngr_slv_config_t* slave_config;  	///< I2C device configuration
	uint8_t* p_rx_data;                    	///< RX data buffer pointer
	uint32_t rx_len;                       	///< RX data length
	uint8_t* p_tx_data;                    	///< TX data buffer pointer
	uint32_t tx_len;                       	///< TX data length
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
int I2C_MNGR_Transact(const i2c_mngr_txn_t* transaction);

#endif // I2C_MNGR_H
