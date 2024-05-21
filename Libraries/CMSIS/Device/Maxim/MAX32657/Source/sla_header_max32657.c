/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

extern uint32_t _application_end;

#define SLA_HEADER_MAGIC0    0xBF1421E4
#define SLA_HEADER_MAGIC1    0x461A8CF5
#define SLA_HEADER_VERSION   0x00000001
#define SLA_HEADER_ALGORITHM_ECDSA 0x516A0001
#define SLA_HEADER_RESERVED  0x00000000

typedef struct
{
    uint32_t magic0;
    uint32_t magic1;
    uint32_t version;
    uint32_t verifytype;
    uint32_t sigaddress;
    uint32_t reserved5;
    uint32_t reserved6;
    uint32_t reserved7;
} flash_app_header_t;

__attribute__ ((section(".sla_header"))) __attribute__ ((__used__))
const flash_app_header_t sla_header =
{
    .magic0 = SLA_HEADER_MAGIC0,
    .magic1 = SLA_HEADER_MAGIC1,
    .version = SLA_HEADER_VERSION,
    .verifytype = SLA_HEADER_ALGORITHM_ECDSA,
    .sigaddress = (uint32_t)&_application_end,
    .reserved5 = SLA_HEADER_RESERVED,
    .reserved6 = SLA_HEADER_RESERVED,
    .reserved7 = SLA_HEADER_RESERVED
};

