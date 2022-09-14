#include "i2c_sensor.h"

int i2c_write(mxc_i2c_req_t* req, uint8_t* txData, int txSize)
{
    return i2c_transfer(req, txData, txSize, NULL, 0); // Create I2C write request
}

int i2c_read(mxc_i2c_req_t* req, uint8_t* txData, uint8_t* rxData, int rxSize)
{
    return i2c_transfer(req, txData, 1, rxData, rxSize); // Create I2C read request
}

int i2c_transfer(mxc_i2c_req_t* req, uint8_t* txData, int txSize, uint8_t* rxData, int rxSize)
{
    req->tx_buf = txData; // Write data buffer
    req->tx_len = txSize; // Number of bytes to write
    req->rx_buf = rxData; // Read data buffer
    req->rx_len = rxSize; // Number of bytes to read
    return MXC_I2C_MasterTransaction(req);
}