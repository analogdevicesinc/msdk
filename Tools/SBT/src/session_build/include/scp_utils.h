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

#ifndef __SCP_UTILS_H__
#define __SCP_UTILS_H__

#ifdef __cplusplus
extern "C"
{
#endif


/**
 * Display an SCP packet
 * @param frame packet
 * @param frame_size packet size
 */
void display_frame(uint8_t * frame, size_t frame_size);

/**
 * Change current side to HOST
 */
void host (void);

/**
 * Return current side  value
 * @return
 */
int whoami (void);

/**
 * Change current side to TARGET
 */
void target (void);


/**
 * Open the packet list file for writing
 * @return ERR_OK if success, error code otherwise
 */
int open_packetlist_file(void);


/**
 * Close the packet list file for writing
 * @return ERR_OK if success, error code otherwise
 */
int close_packetlist_file(void);


/**
 * Write an SCP packet to a file
 * @param frame packet data
 * @param frame_size data length
 * @param name_file file name to write of the packet file
 * @return
 */
int write_packet(uint8_t * frame, size_t frame_size, const char * name_file);

/**
 * Generate an SCP packet ( write to file and display )
 * @param frame packet data
 * @param frame_size data length
 * @param message Display Message - packet Name
 * @param name_file file name to write of the packet file
 * @return
 */
int packet_send (uint8_t * frame, size_t frame_size, const char * message, const char * name_file);


/**
 * Replace EXTRA_PARAM tag in the provided parameter
 * @param param line to replace extra param tag
 * @return
 */
int replace_extra_params(char * param);


/**
 * DEPRECATED Process script line
 * @param line line to process
 * @return
 */
int process_command (char *line);


#ifdef __cplusplus
}
#endif

#endif	/* __SCP_UTILS_H__ */
