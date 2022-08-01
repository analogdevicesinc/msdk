#include <stdint.h>
#include "mxc.h"
#include "gcfr_regs.h"

int cnn_enable(uint32_t clock_source, uint32_t clock_divider)
{
  // Reset all domains, restore power to CNN
  MXC_GCFR->reg3 = 0xf; // Reset
  MXC_GCFR->reg1 = 0xf; // Mask memory
  MXC_GCFR->reg0 = 0xf; // Power
  MXC_GCFR->reg2 = 0x0; // Iso
  MXC_GCFR->reg3 = 0x0; // Reset

  MXC_GCR->pclkdiv = (MXC_GCR->pclkdiv & ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL))
                     | clock_divider | clock_source;
  MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

  return 1;
}

int cnn_init(void)
{
  *((volatile uint32_t *) 0x50001000) = 0x00000000; // AON control
  // Quadrant 0
  *((volatile uint32_t *) 0x50100000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50100004) = 0x0000040e; // SRAM control
  // Quadrant 1
  *((volatile uint32_t *) 0x50500000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50500004) = 0x0000040e; // SRAM control
  // Quadrant 2
  *((volatile uint32_t *) 0x50900000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50900004) = 0x0000040e; // SRAM control
  // Quadrant 3
  *((volatile uint32_t *) 0x50D00000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50D00004) = 0x0000040e; // SRAM control

  return 1;
}

static inline uint32_t* increment_cnn_sram_ptr(uint32_t* ptr) {
    int val = (int)ptr;
    if (val != 0x5041FFFC && val != 0x5081FFFC && val != 0x50C1FFFC && val != 0x5101FFFC ) {
        return ptr + 1;
    }
    else if (val == 0x5041FFFC) { // Quadrant 0 end
        return (uint32_t*)0x50800000; // Quadrant 1 start
    }
    else if (val == 0x5081FFFC) { // Quadrant 1 end
        return (uint32_t*)0x50C00000; // Quadrant 2 start
    }
    else if (val == 0x50C1FFFC) { // Quadrant 2 end
        return (uint32_t*)0x51000000; // Quadrant 3 start
    }
    else if (val >= 0x5101FFFC) { // Quadrant 3 end
        return NULL; // End of CNN SRAM, return NULL
    }
    else {
        return NULL;
    }
}

union bytes_to_word {
    uint8_t* b;
    uint32_t* word;
};

static inline uint32_t* write_bytes_to_cnn_sram(uint8_t* bytes, int len, uint32_t* addr) {
    int i = 0;
    union bytes_to_word u;

    while (i < len) {
      u.b = &bytes[i];
      // *addr = bytes[i] | (bytes[i+1] << 8) | (bytes[i+2] << 16) | (bytes[i+3] << 24);
      *addr = *u.word;
      // ^ Casting through the union will reverse the bytes.  ARM core has a dedicated
      // bytes reversal instruction we can leverage here.
      i += 4;
      addr = increment_cnn_sram_ptr(addr);
    }

    return addr;
}

static inline uint32_t* read_bytes_from_cnn_sram(uint8_t* out_bytes, int len, uint32_t* addr) {
    int i = 0;
    uint32_t word = *addr;

    while (i < len) {
        out_bytes[i] = word & 0xFF;
        out_bytes[i+1] = (word >> 8) & 0xFF;
        out_bytes[i+2] = (word >> 16) & 0xFF;
        out_bytes[i+3] = (word >> 24) & 0xFF;
        addr = increment_cnn_sram_ptr(addr);
        word = *addr;
        i+=4;
    }

    return addr;
}