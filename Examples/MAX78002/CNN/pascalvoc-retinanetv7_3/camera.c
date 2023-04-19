#include "camera.h"

unsigned int g_index = 0;

int error;
int id;
mxc_csi2_req_t req;
mxc_csi2_ctrl_cfg_t ctrl_cfg;
mxc_csi2_vfifo_cfg_t vfifo_cfg;

void CSI2_line_handler(uint8_t* data, unsigned int len)
{
    ram_write_quad(g_index, data, len);
    g_index += len;
}

void process_img(void)
{
    uint8_t *raw;
    uint32_t imgLen;
    uint32_t w, h;

    printf("Capturing image...\n");

    g_index = 0;
    MXC_TMR_SW_Start(MXC_TMR0);
    int error = MXC_CSI2_CaptureFrameDMA();
    unsigned int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    if(error) {
        printf("Failed!\n");
        mxc_csi2_capture_stats_t stats = MXC_CSI2_GetCaptureStats();
        printf("CTRL Error flags: 0x%x\tPPI Error flags: 0x%x\tVFIFO Error flags: 0x%x\n", stats.ctrl_err, stats.ppi_err, stats.vfifo_err);
        return;
    }
    printf("Done! (took %i us)\n", elapsed);

    // Get the details of the image from the camera driver.
    MXC_CSI2_GetImageDetails(&raw, &imgLen, &w, &h);

    // MXC_TMR_SW_Start(MXC_TMR0);
    // clear_serial_buffer();
    // snprintf(g_serial_buffer, SERIAL_BUFFER_SIZE,
    //          "*IMG* %s %i %i %i", // Format img info into a string
    //          mipi_camera_get_pixel_format(STREAM_PIXEL_FORMAT), imgLen, w, h);
    // send_msg(g_serial_buffer);

    // for (int i = 0; i < imgLen; i += SERIAL_BUFFER_SIZE) {
    //     // cnn_addr = read_bytes_from_cnn_sram((uint8_t *)g_serial_buffer, transfer_len, cnn_addr);
    //     ram_read_quad(i, (uint8_t*)g_serial_buffer, SERIAL_BUFFER_SIZE);
    //     MXC_UART_WriteBytes(Con_Uart, (uint8_t *)g_serial_buffer, SERIAL_BUFFER_SIZE);
    // }

    // elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
    // printf("Done! (serial transmission took %i us)\n", elapsed);
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
        process_img();
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

bool camera_init() {
    mipi_camera_init();
    // Confirm correct camera is connected
    mipi_camera_get_product_id(&id);
    printf("Camera ID = %x\n", id);
    if (id != CAMERA_ID) {
        printf("Incorrect camera.\n");
        LED_On(1);
        return false;
    }

    mipi_camera_setup(IMAGE_WIDTH, IMAGE_HEIGHT, PIXEL_FORMAT, OUT_SEQ, MUX_CTRL);

    // Configure RX Controller and PPI (D-PHY)
    ctrl_cfg.invert_ppi_clk = MXC_CSI2_PPI_NO_INVERT;
    ctrl_cfg.num_lanes = NUM_DATA_LANES;
    ctrl_cfg.payload0 = PAYLOAD0_DATA_TYPE;
    ctrl_cfg.payload1 = PAYLOAD1_DATA_TYPE;
    ctrl_cfg.flush_cnt = FLUSH_COUNT;

    ctrl_cfg.lane_src.d0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L0;
    ctrl_cfg.lane_src.d1_swap_sel = MXC_CSI2_PAD_CDRX_PN_L1;
    ctrl_cfg.lane_src.d2_swap_sel = MXC_CSI2_PAD_CDRX_PN_L2;
    ctrl_cfg.lane_src.d3_swap_sel = MXC_CSI2_PAD_CDRX_PN_L3;
    ctrl_cfg.lane_src.c0_swap_sel = MXC_CSI2_PAD_CDRX_PN_L4;

    // Image Data
    req.img_addr = NULL;
    req.pixels_per_line = IMAGE_WIDTH;
    req.lines_per_frame = IMAGE_HEIGHT;
    req.bits_per_pixel_odd = BITS_PER_PIXEL_ODD;
    req.bits_per_pixel_even = BITS_PER_PIXEL_EVEN;
    req.frame_num = 1;

    // Convert RAW to RGB
    req.process_raw_to_rgb = false;
    req.rgb_type = RGB_TYPE;
    req.raw_format = RAW_FORMAT;
    req.autoflush = MXC_CSI2_AUTOFLUSH_ENABLE;
    req.line_handler = CSI2_line_handler;
    // req.callback = CSI2_Callback;

    // Configure VFIFO
    vfifo_cfg.virtual_channel = VIRTUAL_CHANNEL;
    vfifo_cfg.rx_thd = RX_THRESHOLD;
    vfifo_cfg.wait_cyc = WAIT_CYCLE;
    vfifo_cfg.flow_ctrl = FLOW_CTRL;
    vfifo_cfg.err_det_en = MXC_CSI2_ERR_DETECT_DISABLE;
    vfifo_cfg.fifo_rd_mode = MXC_CSI2_READ_ONE_BY_ONE;
    vfifo_cfg.dma_whole_frame = MXC_CSI2_DMA_LINE_BY_LINE;
    vfifo_cfg.dma_mode = MXC_CSI2_DMA_FIFO_ABV_THD;
    vfifo_cfg.bandwidth_mode = MXC_CSI2_NORMAL_BW;
    vfifo_cfg.wait_en = MXC_CSI2_AHBWAIT_ENABLE;

    error = MXC_CSI2_Init(&req, &ctrl_cfg, &vfifo_cfg);
    if (error != E_NO_ERROR) {
        printf("Error Initializating.\n\n");
        while (1) {}
    }

    printf("Initializing SRAM...\n");
    error = ram_init();
    if (error) {
        printf("Failed to initialize SRAM with error %i\n", error);
        return error;
    }

    ram_id_t ram_id;
    error = ram_read_id(&ram_id);
    if (error) {
        printf("Failed to read expected SRAM ID!\n");
        return error;
    }
    printf("RAM ID:\n\tMFID: 0x%.2x\n\tKGD: 0x%.2x\n\tDensity: 0x%.2x\n\tEID: 0x%x\n", ram_id.MFID, ram_id.KGD, ram_id.density, ram_id.EID);
}