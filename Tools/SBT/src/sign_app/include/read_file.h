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

#ifndef __READ_FILE_H__
#define __READ_FILE_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Read and parse an Intel HEX file
 * @param filename HEX file to read
 * @param data buffer to store parsed data
 * @param data_len available space in data and output data length read
 * @param addr buffer to store storing address of data
 * @return ERR_OK if success, else return error code
 */
int read_hex_file(const char *filename, uint8_t *data, size_t *data_len, size_t *addr);

/**
 * Read and parse an S19 file, only S3 record type are considered
 * @param filename S19 file to read
 * @param address_offset offset to add to S19 data address
 * @param data buffer to store parsed data
 * @param data_len available space in data and output data length read
 * @param addr buffer to store storing address of data
 * @return ERR_OK if success, else return error code
 */
int read_s19_file(const char *filename, size_t address_offset, uint8_t *data, size_t *data_len,
                  size_t *addr);

/**
 * Read a S19 file and extract the start address and the end address of the data
 * @param filename S19 file to read
 * @param start_addr output start address of the data
 * @param end_addr output end address of the data
 * @return ERR_OK if success, else return error code
 */
int get_start_addr_and_length_s19(const char *filename, size_t *start_addr, size_t *end_addr);

/**
 * Check if the filename have the selected extension
 * @param ext extension we are looking for
 * @param name filename to test
 * @return TRUE or FALSE
 */
int extension(const char *ext, const char *name);

/**
 * Determine size of a binary file
 * @param size size of the file
 * @param filename file to determine the size
 * @return ERR_OK if success otherwise error code
 */
int read_file_size(size_t *size, char *filename);

/**
 * Read Data from a binary file
 * @param p_pucData read data
 * @param size read size
 * @param filename file to read
 * @return ERR_OK if success otherwise error code
 */
int read_binary_file(char *filename, u8 *p_pucData, size_t *file_size);

/**
 * Read Hexadecimal ASCII encoded data on one line in a file
 * @param file_ptr File pointer to read from
 * @param data_length Available space in the buffer and return the length of data read
 * @param data_buffer output buffer for the data read
 * @return ERR_OK if success, else return error code
 */
int read_line_ascii_data(FILE *file_ptr, size_t *data_length, unsigned char *data_buffer);

#ifdef __cplusplus
}
#endif

#endif /* __READ_FILE_H__ */
