#ifndef I2C_SENSOR_H
#define I2C_SENSOR_H

#include <stdint.h>
#include <stdio.h>
#include "i2c_regs.h"
#include "i2c.h"

/******************************* Definitions *******************************/
#define SET_BIT(cmd, bit)   cmd |= (1 << bit)
#define GET_BIT(cmd, bit)   cmd&(1 << bit)
#define CLEAR_BIT(cmd, bit) cmd &= ~(1 << bit)

/******************************* Type Definitions *******************************/
/**
 * @brief Structure with sensor function pointers
 */
typedef struct {
    int (*init)(mxc_i2c_regs_t* i2c, uint8_t addr);
    int (*read)(void* buf);
} mxc_i2c_sensor_driver_t;

/******************************* Functions *******************************/
/**
 * @brief I2C data transfer (read/write)
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to sending buffer
 * @param txSize number of bytes to write
 * @param rxData pointer receiving buffer
 * @param rxSize number of bytes to read
 * @return int transaction result
 */
int i2c_transfer(mxc_i2c_req_t* req, uint8_t* txData, int txSize, uint8_t* rxData, int rxSize);

/**
 * @brief I2C data write
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to sending buffer
 * @param txSize number of bytes to write
 * @return int transaction result
 */
int i2c_write(mxc_i2c_req_t* req, uint8_t* txData, int txSize);

/**
 * @brief I2C data read
 * 
 * @param req pointer to I2C request instance 
 * @param txData pointer to one byte
 * @param rxData pointer receiving buffer
 * @param rxSize number of bytes to read
 * @return int transaction result
 */
int i2c_read(mxc_i2c_req_t* req, uint8_t* txData, uint8_t* rxData, int rxSize);

#endif /* I2C_SENSOR_H */