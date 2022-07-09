/******************************************************************************
 * Copyright (C) 2022 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *
 ******************************************************************************/

/*******************************      INCLUDES    ****************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "bootloader.h"

/*******************************      DEFINES     ****************************/
#ifndef MIN
	#define MIN(x,y)	( (x<y) ? x: y )
#endif

#ifndef MAX
	#define MAX(x,y)	( (x>y) ? x: y )
#endif

/******************************* Type Definitions ****************************/
typedef struct {
	unsigned char  magic[4];
	unsigned int   formatVersion;
	unsigned char  target[16];
	unsigned char  enc_type[16];
	unsigned char  nonce[11];
	unsigned char  resv0;
	unsigned char  auth[16];
	unsigned short numPages;
	unsigned short pageSize;
	unsigned char  crcSize;
	unsigned char  resv1[3];
} MsblHeader_t;

/******************************* 	Variables 	  ****************************/
static bl_conf_struct_t g_plt_funcs;

/******************************* Static Functions ****************************/
static void hexdump(const char* title, unsigned char* buf, unsigned int len)
{
    unsigned int i;

    if (title) {
    	g_plt_funcs.printf("%s", title);
    }

    /* Print buffer bytes */
    for (i = 0; i < len; i++) {
        if (!(i % 16)) {
        	g_plt_funcs.printf("\r\n");
        }
        g_plt_funcs.printf("%02X ", buf[i]);
    }

    g_plt_funcs.printf("\r\n");
}

static int send_rcv(unsigned char *tx, int txLen, unsigned char *rx, int rxLen, int delay_ms, int delay_step_ms)
{
	int ret = 0;
	int i;

	if (delay_step_ms == 0) {
		delay_step_ms = 100; // 100ms default
	}

	for (i=0; i<2; i++) {
		ret = g_plt_funcs.write(tx, txLen);
		if (ret == 0) {
			break;
		}
		g_plt_funcs.delay_ms(100);
	}

	if (ret == 0) {
        int nb_of_retry = (delay_ms / delay_step_ms) + 1;
        for (i = 0; i < nb_of_retry; i++) {
        	if (delay_ms > 0) {
        		g_plt_funcs.delay_ms(delay_step_ms);
        	}

        	// read
        	ret = g_plt_funcs.read(rx, rxLen);

			if ( (ret == 0) && (rx[0] != BL_RET_ERR_TRY_AGAIN) ) {
				break;
			}
        }

        // Convert BL return value
    	if (rx[0] == BL_RET_SUCCESS) {
    		ret = 0; // zero means success
    	} else if (rx[0] == 0) {
    		ret = -1;
    	} else {
    		ret = rx[0]; // first byte is BL response
    	}
	}

	return ret;
}

static int update_bl_cfg (unsigned char item, unsigned char cmd)
{
	int ret = 0;
	unsigned char req[4];
	unsigned char rsp[1] = {0xFF, };

	req[0] = BLCmdConfigWrite_MAIN_CMD;
	req[1] = BLCmdConfigWrite_ENTRY_CONFIG;
	req[2] = item;
	req[3] = cmd;

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

/******************************* Public Functions ****************************/
int bl_init(bl_conf_struct_t *plt_funcs)
{
	int ret = 0;

	g_plt_funcs = *plt_funcs;

	return ret;
}

int bl_update_interface(comm_read_t read_func,  comm_write_t write_func)
{
	int ret = 0;

	g_plt_funcs.read  	  = read_func;
	g_plt_funcs.write 	  = write_func;

	return ret;
}

int bl_hard_reset_then_enter_bl_mode(void)
{
	int ret = 0;

	g_plt_funcs.gpio_set(GPIO_IDX_BL0, 0);
	g_plt_funcs.delay_ms(10);
	g_plt_funcs.gpio_set(GPIO_IDX_BL0, 1);
	g_plt_funcs.delay_ms(10);

	ret = bl_enter_bl_mode();

	return ret;
}

int bl_enter_bl_mode(void)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdDevSetMode_MAIN_CMD, BLCmdDevSetMode_SET_MODE, 0x08};
	unsigned char rsp[1] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_exit_bl_mode(void)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdDevSetMode_MAIN_CMD, BLCmdDevSetMode_SET_MODE, 0x00};
	unsigned char rsp[1] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_get_partnumber(char *buf, unsigned int maxLen)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdDeviceInfo_MAIN_CMD, BLCmdDeviceInfo_GET_PLATFORM_TYPE};
	unsigned char rsp[2] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 2, 0, 0);

    if (ret == 0) {
    	switch(rsp[1]) {
			case 1: memcpy(buf, "MAX32660", MIN(strlen("MAX32660")+1, maxLen)); break;
			case 3: memcpy(buf, "MAX32670", MIN(strlen("MAX32670")+1, maxLen)); break;
			case 5: memcpy(buf, "MAX78000", MIN(strlen("MAX78000")+1, maxLen)); break;
			case 6: memcpy(buf, "MAX32655", MIN(strlen("MAX32655")+1, maxLen)); break;
			default:
				memcpy(buf, "UNKNOWN", MIN(strlen("UNKNOWN")+1, maxLen));
				break;
    	}
    }

	return ret;
}

int bl_get_version(char *buf, unsigned int maxLen)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdInfo_MAIN_CMD, BLCmdInfo_GET_VERSION};
	unsigned char rsp[4] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 4, 0, 0);

    if (ret == 0) {
    	snprintf(buf, maxLen, "v%d.%d.%d", rsp[1], rsp[2], rsp[3]);
    }

	return ret;
}

int bl_get_usn(char *buf, unsigned int maxLen)
{
	int ret = 0;
	unsigned char req[ ]  = {BLCmdInfo_MAIN_CMD, BLCmdInfo_GET_USN};
	unsigned char rsp[25] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 25, 0, 0);

    if (ret == 0) {
    	memcpy(buf, &rsp[1], MIN(24, maxLen));
    }

	return ret;
}

int bl_get_page_size(unsigned int *page_size)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdInfo_MAIN_CMD, BLCmdInfo_GET_PAGE_SIZE};
	unsigned char rsp[3] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 3, 0, 0);

    if (ret == 0) {
    	*page_size =  (rsp[1]<<8) | rsp[2];
    }

	return ret;
}

int bl_erase_app(void)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdFlash_MAIN_CMD,  BLCmdFlash_ERASE_APP_MEMORY};
	unsigned char rsp[1] = {0xFF, };

    ret = send_rcv(req, sizeof(req), rsp, 1, 2000, 500);

	return ret;
}

int bl_set_num_pages(int page_num)
{
	int ret = 0;
	unsigned char req[4];
	unsigned char rsp[1] = {0xFF, };

    req[0] = BLCmdFlash_MAIN_CMD;
    req[1] = BLCmdFlash_SET_NUM_PAGES;
    req[2] = (page_num>>8) & 0xff;
    req[3] = (page_num>>0) & 0xff;

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_set_iv(unsigned char *iv)
{
	int ret = 0;
	unsigned char req[2+AES_IV_SIZE];
	unsigned char rsp[1] = {0xFF, };

    req[0] = BLCmdFlash_MAIN_CMD;
    req[1] = BLCmdFlash_SET_IV;
    memcpy(&req[2], iv, AES_IV_SIZE);

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_set_auth(unsigned char *auth)
{
	int ret = 0;
	unsigned char req[2+AES_AUTH_SIZE];
	unsigned char rsp[1] = {0xFF, };

    req[0] = BLCmdFlash_MAIN_CMD;
    req[1] = BLCmdFlash_SET_AUTH;
    memcpy(&req[2], auth, AES_AUTH_SIZE);

    ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_write_page(const char *page, unsigned int page_len)
{
	int ret = 0;
	static unsigned char req[8*1024+32];// use static buffer
	unsigned char rsp[1] = {0xFF, };

	if (page_len > (sizeof(req)-2) ) {
		return -1;
	}

    req[0] = BLCmdFlash_MAIN_CMD;
    req[1] = BLCmdFlash_WRITE_PAGE;
    memcpy(&req[2], page, page_len);

    ret = send_rcv(req, page_len+2, rsp, 1, 1000, 100);

	return ret;
}

int bl_flash_image(const char *image)
{
	int ret;
	int i;
	MsblHeader_t header;
	int checksum_size = 16; // checksum value at the end of page

	if (image == NULL) {
		return -1;
	}

	memcpy(&header, image, sizeof(MsblHeader_t));

    g_plt_funcs.printf("MSBL Info\r\n");
	g_plt_funcs.printf("------------------------------------------------\r\n");
	g_plt_funcs.printf("%-15s: %c%c%c%c\r\n", 	"magic", header.magic[0], header.magic[1], header.magic[2], header.magic[3]);
	g_plt_funcs.printf("%-15s: %d\r\n", 		"formatVersion", header.formatVersion);
	g_plt_funcs.printf("%-15s: %s\r\n", 		"target", header.target);
	g_plt_funcs.printf("%-15s: %s\r\n", 		"EncType", header.enc_type);
	g_plt_funcs.printf("%-15s: %d\r\n", 		"numPages", header.numPages);
	g_plt_funcs.printf("%-15s: %d\r\n", 		"pageSize", header.pageSize);
	g_plt_funcs.printf("%-15s: %d\r\n", 		"crcSize", header.crcSize);
	g_plt_funcs.printf("%-15s: %d\r\n", 		"Header Size", sizeof(header));
	g_plt_funcs.printf("%-15s: %d\r\n", 		"resv0", header.resv0);
	//
	hexdump("nonce", header.nonce, 	AES_IV_SIZE);
	hexdump("auth",  header.auth,   AES_AUTH_SIZE);
	hexdump("resv1", header.resv1,  3);

	ret = bl_hard_reset_then_enter_bl_mode();
    if (ret) {
        return ret;
    }

	ret = bl_set_num_pages(header.numPages);
    if (ret) {
    	g_plt_funcs.printf("Error! bl_set_num_pages\r\n");
        return ret;
    }

	ret = bl_set_iv(header.nonce);
    if (ret) {
    	g_plt_funcs.printf("Error! bl_set_iv\r\n");
        return ret;
    }

	ret = bl_set_auth(header.auth);
    if (ret) {
    	g_plt_funcs.printf("Error! bl_set_auth\r\n");
        return ret;
    }

	ret = bl_erase_app();
    if (ret) {
    	g_plt_funcs.printf("Error! bl_erase_app\r\n");
        return ret;
    }

    int page_len = header.pageSize + checksum_size;
    for (i=0; i < header.numPages; i++) {
		ret = bl_write_page( &image[sizeof(MsblHeader_t) + i*page_len], page_len);
		if (ret) {
			g_plt_funcs.printf("Flashing page %d/%d  [FAILED] err:%d\r\n", i+1, header.numPages, ret);
			return ret;
		}
		g_plt_funcs.printf("Flashing page %d/%d  [SUCCESS]\r\n", i+1, header.numPages);
    }

    bl_exit_bl_mode();

	return ret;
}

int bl_get_target_configure(const char *target_bl_version, void *bl_cfg_struct)
{
	int ret = 0;
	unsigned char req[3];
	unsigned char rsp[9] = {0xFF, };

    req[0] = BLCmdConfigRead_MAIN_CMD;
    req[1] = BLCmdConfigRead_ALL_CFG;
    req[2] = 0x00; // dummy byte

    if (strcmp(target_bl_version, "v3.4.1") <= 0) {
		ret = send_rcv(req, sizeof(req), rsp, 5, 0, 0);

		if (ret == 0) {
			//
			((boot_config_t_before_v342 *)bl_cfg_struct)->v[0] = rsp[4];
			((boot_config_t_before_v342 *)bl_cfg_struct)->v[1] = rsp[3];
			((boot_config_t_before_v342 *)bl_cfg_struct)->v[2] = rsp[2];
			((boot_config_t_before_v342 *)bl_cfg_struct)->v[3] = rsp[1];
		}
    } else {
    	ret = send_rcv(req, sizeof(req), rsp, 9, 0, 0);

		if (ret == 0) {
			//
			((boot_config_t *)bl_cfg_struct)->v[0] = rsp[8];
			((boot_config_t *)bl_cfg_struct)->v[1] = rsp[7];
			((boot_config_t *)bl_cfg_struct)->v[2] = rsp[6];
			((boot_config_t *)bl_cfg_struct)->v[3] = rsp[5];
			((boot_config_t *)bl_cfg_struct)->v[4] = rsp[4];
			((boot_config_t *)bl_cfg_struct)->v[5] = rsp[3];
			((boot_config_t *)bl_cfg_struct)->v[6] = rsp[2];
			((boot_config_t *)bl_cfg_struct)->v[7] = rsp[1];
		}
    }

	return ret;
}

int bl_update_cfg_enter_bl_check(int enable)
{
    return update_bl_cfg(BL_CFG_EBL_CHECK,  enable?1:0);
}

int bl_update_cfg_set_ebl_pin(int port, int pin)
{
    return update_bl_cfg(BL_CFG_EBL_PIN, (port<<5) | pin);
}

int bl_update_cfg_set_ebl_pin_polarity(int polarity)
{
    return update_bl_cfg(BL_CFG_EBL_POL, polarity?1:0);
}

int bl_update_cfg_set_valid_mark_check(int enable)
{
    return update_bl_cfg(BL_CFG_VALID_MARK_CHK, enable?1:0);
}

int bl_update_cfg_enable_uart(int enable)
{
    return update_bl_cfg(BL_CFG_UART_ENABLE, enable?1:0);
}

int bl_update_cfg_enable_i2c(int enable)
{
    return update_bl_cfg(BL_CFG_I2C_ENABLE, enable?1:0);
}

int bl_update_cfg_enable_spi(int enable)
{
    return update_bl_cfg(BL_CFG_SPI_ENABLE, enable?1:0);
}

int bl_update_cfg_set_i2c_addr(const char *target_bl_version, unsigned char addr)
{
	int ret = -1; // on default error

    if (strcmp(target_bl_version, "v3.4.1") <= 0) {
             if (addr == 0x58) ret = update_bl_cfg(BL_CFG_I2C_SLAVE_ADDR, 0);
        else if (addr == 0x5A) ret = update_bl_cfg(BL_CFG_I2C_SLAVE_ADDR, 1);
        else if (addr == 0x5C) ret = update_bl_cfg(BL_CFG_I2C_SLAVE_ADDR, 2);
        else if (addr == 0xAA) ret = update_bl_cfg(BL_CFG_I2C_SLAVE_ADDR, 3);
        else {
            g_plt_funcs.printf("%s version bootloader does not support 0x%X I2C addr", target_bl_version, addr);
        }
    } else {
    	ret = update_bl_cfg(BL_CFG_I2C_SLAVE_ADDR, addr);
    }

    return ret;
}

int bl_update_cfg_set_crc_check(int enable)
{
    return update_bl_cfg(BL_CFG_CRC_CHECK, enable?1:0);
}

int bl_update_cfg_lock_swd(int enable)
{
    return update_bl_cfg(BL_CFG_LOCK_SWD, enable?1:0);
}

int bl_update_cfg_set_bl_exit_mode(BLExitMode_t mode)
{
	int ret = 0;
	unsigned char req[4];
	unsigned char rsp[1] = {0xFF, };

    req[0] = BLCmdConfigWrite_MAIN_CMD;
    req[1] = BLCmdConfigWrite_EXIT_CONFIG;
    req[2] = 0x00; // means exit mode
    req[3] = mode; //

	ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int b_update_cfg_set_bl_exit_timeout(unsigned short timeout)
{
	int ret = 0;
	unsigned char req[4];
	unsigned char rsp[1] = {0xFF, };
	int i;

    req[0] = BLCmdConfigWrite_MAIN_CMD;
    req[1] = BLCmdConfigWrite_EXIT_CONFIG;
    req[2] = BL_EXIT_TIMEOUT;

    // Log2(timeout) then round
    for (i = 15; i > 0; i--) {
    	if (timeout & (1<<i)) { // Does MSB bit set?

    		// Check Round
			if ( ( timeout & ~(1<<i) ) >= (1<<i)/2 ) {
			    req[3] = i+1; // add plus one
			} else {
			    req[3] = i;
			}
			break;
    	}
    }

	ret = send_rcv(req, sizeof(req), rsp, 1, 0, 0);

	return ret;
}

int bl_flash_bl_cfg(void)
{
	int ret = 0;
	unsigned char req[ ] = {BLCmdConfigWrite_MAIN_CMD, BLCmdConfigWrite_SAVE_SETTINGS};
	unsigned char rsp[1] = {0xFF, };

	ret = send_rcv(req, sizeof(req), rsp, 1, 1000, 0);

	return ret;
}
