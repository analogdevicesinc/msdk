
#include <stdint.h>

// Return an x coordinate given an array index and x resolution
unsigned int _x(unsigned int i, unsigned int xres);

// Return a y coordinate given an array index and x resolution
unsigned int _y(unsigned int i, unsigned int xres);

// Return an array index given an x and y coordinate and an x resolution
// This function also enforces array boundaries.
unsigned int _i(unsigned int x, unsigned int y, unsigned int xres, unsigned int yres);

// Convert an RGB value to RGB565
uint16_t rgb_to_rgb565(uint8_t r, uint8_t g, uint8_t b);

// Clamp a float to uint8_t
uint8_t clamp_f_u8(float val);

// Clamp an int to uint8_t
uint8_t clamp_i_u8(int val);

void color_correct(uint8_t *srcimg, unsigned int w, unsigned int h);

/**
* @brief Formulate a bayer "passthrough" image that splits an HM0360 bayer pattern into its RGB channels while preserving the bayer pattern.  Useful for debugging and demosaicing algorithm development.
* @param[in] srcimg Pointer to the raw bayer pattern
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
* @param[out] dstimg Output pointer for converted RGB565 image.
****************************************************************************/
void bayer_passthrough(uint8_t *srcimg, uint32_t w, uint32_t h, uint16_t *dstimg);

/**
* @brief Color-correct and demosaic a raw HM0360 bayer-patterned image array and convert to RGB565.
* @param[in] srcimg Pointer to the raw bayer pattern
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
* @param[out] dstimg Output pointer for converted RGB565 image.
****************************************************************************/
void bayer_bilinear_demosaicing(uint8_t *srcimg, uint32_t w, uint32_t h, uint16_t *dstimg);

/**
* @brief Color-correct and demosaic a raw HM0360 bayer-patterned image array and convert to RGB565.
* @param[in] srcimg Pointer to the raw bayer pattern
* @param[in] w Width of the bayer pattern (in pixels)
* @param[in] h Height of the bayer pattern (in pixels)
* @param[out] dstimg Output pointer for converted RGB565 image.
****************************************************************************/
void bayer_bilinear_demosaicing_crop(uint8_t *srcimg, uint32_t src_width, uint32_t w_offset, uint32_t src_height, uint32_t h_offset, uint16_t *dstimg, uint32_t dst_width, uint32_t dst_height);

void bayer_bilinear_demosaicing_crop_vertical(uint8_t *srcimg, uint32_t src_width, uint32_t w_offset, uint32_t src_height, uint32_t h_offset, uint16_t *dstimg, uint32_t dst_width, uint32_t dst_height);

void bayer_malvarhe_demosaicing_crop_vertical(uint8_t *srcimg, uint32_t w, uint32_t w_offset, uint32_t h, uint32_t h_offset, uint16_t *dstimg, uint32_t dst_width, uint32_t dst_height);