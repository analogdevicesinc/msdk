#include <stdio.h>
#include <stdlib.h>
#include "led.h"
#include "camera.h"
#include "fastspi.h"
#include "tft_st7789v.h"
#include "tmr.h"

unsigned int g_index = 0;

int error;
int id;

int CSI2_line_handler(uint8_t *data, unsigned int len)
{
    ram_write_quad(g_index, data, len);
    g_index += len;
    return E_NO_ERROR;
}

void camera_capture(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    printf("Capturing image...\n");
    spi_init();
    ram_enter_quadmode();

    g_index = 0;
    MXC_TMR_SW_Start(MXC_TMR1);
    int error = mipi_camera_capture();
    unsigned int elapsed = MXC_TMR_SW_Stop(MXC_TMR1);
    if (error) {
        mxc_csi2_capture_stats_t stats = MXC_CSI2_GetCaptureStats();
        printf("Failed!\n");
        printf("CTRL Error flags: 0x%x\tPPI Error flags: 0x%x\tVFIFO Error flags: 0x%x\n",
               stats.ctrl_err, stats.ppi_err, stats.vfifo_err);
        return;
    }
    printf("Done! (took %i us)\n", elapsed);

    // Get the details of the image from the camera driver.
    MXC_CSI2_GetImageDetails(&raw, &imgLen, &w, &h);

#ifdef CONSOLE
    MXC_TMR_SW_Start(MXC_TMR1);
    clear_serial_buffer();
    snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
             "*IMG* %s %i %i %i", // Format img info into a string
             mipi_camera_get_pixel_format(STREAM_PIXEL_FORMAT), imgLen, w, h);
    send_msg(g_serial_buffer);

    for (int i = 0; i < imgLen; i += SERIAL_BUFFER_SIZE) {
        // cnn_addr = read_bytes_from_cnn_sram((uint8_t *)g_serial_buffer, transfer_len, cnn_addr);
        ram_read_quad(i, (uint8_t *)g_serial_buffer, SERIAL_BUFFER_SIZE);
        MXC_UART_WriteBytes(Con_Uart, (uint8_t *)g_serial_buffer, SERIAL_BUFFER_SIZE);
    }

    elapsed = MXC_TMR_SW_Stop(MXC_TMR1);
    printf("Done! (serial transmission took %i us)\n", elapsed);
#endif
}

#ifdef CONSOLE
void service_console(cmd_t cmd)
{
    // Process the received command...
    if (cmd == CMD_UNKNOWN) {
        printf("Uknown command '%s'\n", g_serial_buffer);
    } else if (cmd == CMD_HELP) {
        print_help();
    } else if (cmd == CMD_RESET) {
        // Issue a soft reset
        MXC_GCR->rst0 |= MXC_F_GCR_RST0_SYS;
    } else if (cmd == CMD_CAPTURE) {
        camera_capture();
    } else if (cmd == CMD_SETREG) {
        // Set a camera register
        unsigned int reg;
        unsigned int val;
        // ^ Declaring these as unsigned ints instead of uint8_t
        // avoids some issues caused by type-casting inside of sscanf.

        sscanf(g_serial_buffer, "%s %u %u", cmd_table[cmd], &reg, &val);
        printf("Writing 0x%x to camera reg 0x%x\n", val, reg);
        mipi_camera_write_reg((uint16_t)reg, (uint16_t)val);
    } else if (cmd == CMD_GETREG) {
        // Read a camera register
        unsigned int reg;
        uint8_t val;
        sscanf(g_serial_buffer, "%s %u", cmd_table[cmd], &reg);
        mipi_camera_read_reg((uint16_t)reg, &val);
        snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE, "Camera reg 0x%x=0x%x", reg, val);
        send_msg(g_serial_buffer);
    }
}
#endif

bool camera_init()
{
    mipi_camera_settings_t camera_settings = {
        .width = IMAGE_WIDTH,
        .height = IMAGE_HEIGHT,
        .camera_format = {
            .pixel_format = PIXEL_FORMAT,
            .pixel_order = PIXEL_ORDER
        },
        .line_handler = CSI2_line_handler
    };

    mipi_camera_init(camera_settings);

    mipi_camera_get_product_id(&id);
    printf("Camera ID = %x\n", id);
    if (id != CAMERA_ID) {
        printf("Incorrect camera.\n");
        LED_On(1);
        return false;
    }

    printf("Initializing SRAM...\n");
    error = ram_init();
    if (error) {
        printf("Failed to initialize SRAM with error %i\n", error);
        return false;
    }

    ram_id_t ram_id;
    error = ram_read_id(&ram_id);
    if (error) {
        printf("Failed to read expected SRAM ID!\n");
        return false;
    }
    printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", ram_id.MFID,
           ram_id.KGD, ram_id.density, ram_id.EID);
    
    return true;
}