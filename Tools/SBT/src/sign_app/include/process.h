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

#ifndef __PROCESS_H__
#define __PROCESS_H__

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * Option type
 */
typedef enum{
	OT_HEX = 1,      //!< One byte hex
	OT_LONGHEX,  //!< Four bytes Hex
	OT_DATAHEX,  //!< Variable Length Hex
	OT_STRING,   //!< String
	OT_INT,      //!< Decimal Integer
	OT_FILE,     //!< File name or path
	OT_YESNO,    //!< Yes or No
	OT_ALGO,     //!< Signing Algorithm
	OT_BOOTMETHOD//!< Boot Method
}option_type_t;


typedef enum {bm_direct, bm_cmsis} bootmethod_t;

/**
 * Parse a string for hex encoded data
 * @param ptr parsed value
 * @param str string to parse
 * @param length expected length
 * @return ERR_OK if success otherwise error code
 */
int parse_datahex(unsigned char * ptr, const char * str, int length);

/**
 * Parse a String for 4 bytes hex encoded data
 * @param ptr parsed value
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_longhex(unsigned int * ptr, const char * str);


/**
 * Parse a String for 1 bytes hex encoded data
 * @param ptr parsed value
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_hex(unsigned int * ptr, const char * str);

/**
 * Parse a string for yes or no keywords
 * @param ptr parsed value ( TRUE for yes and FALSE for no )
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_yesno(unsigned char * ptr, const char * str);

/**
 * Parse a String for decimal encoded integer data
 * @param ptr parsed value
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_int(unsigned int * ptr, const char * str);


/**
 * Parse a String for clean string
 * @param ptr parsed value
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_string(char * ptr, const char * str);


/**
 * Parse a String for a signing algorithm keyword
 * @param ptr parsed algorithm
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_algo(unsigned char * ptr, const char * str);


/**
 * Parse a String for a boot method keyword
 * @param ptr parsed method
 * @param str string to parse
 * @return ERR_OK if success otherwise error code
 */
int parse_bootmethod(bootmethod_t * ptr, const char * str);


/**
 * Parse an option according to its type and store its value in the corresponding variable
 * @param type Option type
 * @param ptr pointer to the variable to store the parsed value
 * @param value option string to parse
 * @param min parsing option
 * @return ERR_OK if success otherwise error code
 */
int parse_store (option_type_t type, void * ptr, const char * value, int min);


#ifdef __cplusplus
}
#endif

#endif	/* __PROCESS_H__ */
