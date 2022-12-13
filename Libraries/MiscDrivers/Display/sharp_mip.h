#ifndef DRIVER_SHARP_MIPH_
#define DRIVER_SHARP_MIPH_

#include <stdio.h>
#include <stdint.h>
#include "display.h"

/********************************* DEFINES ******************************/
#define BUF_SIZE(w,h)  (((h) * (2 + (w >> 3))) + 2)													\

/********************************* TYPE DEFINES ******************************/
typedef struct {
	uint8_t 					row;
	uint8_t 					col;
}sharp_mip_init_param_t;

typedef struct {
	display_comm_api comm_api;
	sharp_mip_init_param_t init_param;
}sharp_mip_dev;

/********************************* Function Prototypes **************************/
int sharp_mip_configure(sharp_mip_dev* dev, sharp_mip_init_param_t* init_param, display_comm_api* comm_api);
int sharp_mip_init(sharp_mip_dev* dev);
void sharp_mip_flush_area(sharp_mip_dev *dev, const display_area_t* area, const uint8_t* data);
void sharp_mip_set_buffer_pixel_util(sharp_mip_dev *dev, uint8_t* buf, uint16_t buf_w, uint16_t x, uint16_t y, uint8_t color, uint8_t is_opaque);
void sharp_mip_com_inversion(sharp_mip_dev *dev, int inversion_on);

#endif /* DRIVER_SHARP_MIPH_ */
