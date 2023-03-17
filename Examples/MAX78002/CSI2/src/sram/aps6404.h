#include <stdint.h>

#define MFID_EXPECTED 0x0D
#define KGD_EXPECTED 0x5D
#define DENSITY_EXPECTED 0b010

typedef struct {
    uint8_t MFID;
    uint8_t KGD;
    uint8_t density;
    int EID;
} ram_id_t;

int ram_init();

int ram_reset();

int ram_enter_quadmode();

int ram_exit_quadmode();

int ram_read_id(ram_id_t *out);

int ram_read_slow(uint32_t address, uint8_t *out, unsigned int len);

int ram_read_quad(uint32_t address, uint8_t *out, unsigned int len);

int ram_write(uint32_t address, uint8_t * data, unsigned int len);

int ram_write_quad(uint32_t address, uint8_t * data, unsigned int len);

int benchmark_dma_overhead(unsigned int *out);