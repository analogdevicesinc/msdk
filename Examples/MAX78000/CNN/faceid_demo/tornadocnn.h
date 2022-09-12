/*
 * On-device execution
 */

#include <stdint.h>
typedef int32_t q31_t;
typedef int16_t q15_t;

#define CNN_START LED_On(0)
#define CNN_COMPLETE LED_Off(0)

void softmax_q17p14_q15(const q31_t *vec_in, const uint16_t dim_vec, q15_t *p_out);
