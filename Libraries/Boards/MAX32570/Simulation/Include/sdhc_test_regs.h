#ifndef _SDHC_TEST_REGS_H_
#define _SDHC_TEST_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     sdhc
 * @defgroup    sdhc_registers Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SDHC Peripheral Module.
 * @description SDHC/SDIO Controller
 */

/**
 * @ingroup sdhc_registers
 * Structure type to access the SDHC Registers.
 */
typedef struct {
    __IO uint32_t reg_00;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_04;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_08;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_0C;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_10;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_14;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_18;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_1C;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_20;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_24;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_28;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_2C;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_30;                 /**< <tt>\b 0x00:<\tt> SDHC */
    __IO uint32_t reg_34;                 /**< <tt>\b 0x00:<\tt> SDHC */
} mxc_sdhc_test_regs_t;

#define MXC_F_SDHC_TEST_00_ITAPDLYENA_POS                (30)
#define MXC_F_SDHC_TEST_00_ITAPDLYENA                    ((uint32_t)(0x1 << MXC_F_SDHC_TEST_00_ITAPDLYENA_POS))

#define MXC_F_SDHC_TEST_00_ITAPDLYSEL_POS                (25)
#define MXC_F_SDHC_TEST_00_ITAPDLYSEL                    ((uint32_t)(0x1F << MXC_F_SDHC_TEST_00_ITAPDLYSEL_POS))

#define MXC_F_SDHC_TEST_00_ITAPCHGWIN_POS                (24)
#define MXC_F_SDHC_TEST_00_ITAPCHGWIN                    ((uint32_t)(0x1 << MXC_F_SDHC_TEST_00_ITAPCHGWIN_POS))

#define MXC_F_SDHC_TEST_00_OTAPDLYENA_POS                (23)
#define MXC_F_SDHC_TEST_00_OTAPDLYENA                    ((uint32_t)(0x1 << MXC_F_SDHC_TEST_00_OTAPDLYENA_POS))

#define MXC_F_SDHC_TEST_00_OTAPDLYSEL_POS                (19)
#define MXC_F_SDHC_TEST_00_OTAPDLYSEL                    ((uint32_t)(0xF << MXC_F_SDHC_TEST_00_OTAPDLYSEL_POS))

#define MXC_F_SDHC_TEST_04_SLOTTYPE_POS                  (30)
#define MXC_F_SDHC_TEST_04_SLOTTYPE                      ((uint32_t)(0x3 << MXC_F_SDHC_TEST_04_SLOTTYPE_POS))

#define MXC_F_SDHC_TEST_04_TUNINGCOUNT_POS               (24)
#define MXC_F_SDHC_TEST_04_TUNINGCOUNT                   ((uint32_t)(0x3F << MXC_F_SDHC_TEST_04_TUNINGCOUNT_POS))

#define MXC_F_SDHC_TEST_04_TIMEOUTCLKUNIT_POS            (22)
#define MXC_F_SDHC_TEST_04_TIMEOUTCLKUNIT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_04_TIMEOUTCLKUNIT_POS))

#define MXC_F_SDHC_TEST_04_TIMEOUTCLKFREQ_POS            (16)
#define MXC_F_SDHC_TEST_04_TIMEOUTCLKFREQ                ((uint32_t)(0x3F << MXC_F_SDHC_TEST_04_TIMEOUTCLKFREQ_POS))

#define MXC_F_SDHC_TEST_04_ASYNCWKUPENA_POS              (10)
#define MXC_F_SDHC_TEST_04_ASYNCWKUPENA                  ((uint32_t)(0x3 << MXC_F_SDHC_TEST_04_ASYNCWKUPENA_POS))

#define MXC_F_SDHC_TEST_04_MAXBLKLENGTH_POS              (8)
#define MXC_F_SDHC_TEST_04_MAXBLKLENGTH                  ((uint32_t)(0x3 << MXC_F_SDHC_TEST_04_MAXBLKLENGTH_POS))

#define MXC_F_SDHC_TEST_04_BASECLKFREQ_POS               (0)
#define MXC_F_SDHC_TEST_04_BASECLKFREQ                   ((uint32_t)(0xFF << MXC_F_SDHC_TEST_04_BASECLKFREQ_POS))

#define MXC_F_SDHC_TEST_08_8BITSUPPORT_POS               (31)
#define MXC_F_SDHC_TEST_08_8BITSUPPORT                   ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_8BITSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_ADMA2SUPPORT_POS              (30)
#define MXC_F_SDHC_TEST_08_ADMA2SUPPORT                  ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_ADMA2SUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SDMASUPPORT_POS               (29)
#define MXC_F_SDHC_TEST_08_SDMASUPPORT                   ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SDMASUPPORT_POS))

#define MXC_F_SDHC_TEST_08_HIGHSPEEDSUPPORT_POS          (28)
#define MXC_F_SDHC_TEST_08_HIGHSPEEDSUPPORT              ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_HIGHSPEEDSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SUSPRESSUPPORT_POS            (27)
#define MXC_F_SDHC_TEST_08_SUSPRESSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SUSPRESSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_64BITSUPPORT_POS              (26)
#define MXC_F_SDHC_TEST_08_64BITSUPPORT                  ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_64BITSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_ASYNCINTRSUPPORT_POS          (25)
#define MXC_F_SDHC_TEST_08_ASYNCINTRSUPPORT              ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_ASYNCINTRSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_3P3VOLTSUPPORT_POS            (24)
#define MXC_F_SDHC_TEST_08_3P3VOLTSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_3P3VOLTSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_3P0VOLTSUPPORT_POS            (23)
#define MXC_F_SDHC_TEST_08_3P0VOLTSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_3P0VOLTSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_1P8VOLTSUPPORT_POS            (22)
#define MXC_F_SDHC_TEST_08_1P8VOLTSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_1P8VOLTSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SDR50SUPPORT_POS              (21)
#define MXC_F_SDHC_TEST_08_SDR50SUPPORT                  ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SDR50SUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SDR104SUPPORT_POS             (20)
#define MXC_F_SDHC_TEST_08_SDR104SUPPORT                 ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SDR104SUPPORT_POS))

#define MXC_F_SDHC_TEST_08_DDR50SUPPORT_POS              (19)
#define MXC_F_SDHC_TEST_08_DDR50SUPPORT                  ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_DDR50SUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SPISUPPORT_POS                (18)
#define MXC_F_SDHC_TEST_08_SPISUPPORT                    ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SPISUPPORT_POS))

#define MXC_F_SDHC_TEST_08_SPIBLKMODE_POS                (17)
#define MXC_F_SDHC_TEST_08_SPIBLKMODE                    ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_SPIBLKMODE_POS))

#define MXC_F_SDHC_TEST_08_ADRIVERSUPPORT_POS            (2)
#define MXC_F_SDHC_TEST_08_ADRIVERSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_ADRIVERSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_CDRIVERSUPPORT_POS            (1)
#define MXC_F_SDHC_TEST_08_CDRIVERSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_CDRIVERSUPPORT_POS))

#define MXC_F_SDHC_TEST_08_DDRIVERSUPPORT_POS            (0)
#define MXC_F_SDHC_TEST_08_DDRIVERSUPPORT                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_08_DDRIVERSUPPORT_POS))

#define MXC_F_SDHC_TEST_0C_RETUNINGMODES_POS             (0)
#define MXC_F_SDHC_TEST_0C_RETUNINGMODES                 ((uint32_t)(0x3 << MXC_F_SDHC_TEST_0C_RETUNINGMODES_POS))

#define MXC_F_SDHC_TEST_0C_TUNINGFORSDR50_POS            (2)
#define MXC_F_SDHC_TEST_0C_TUNINGFORSDR50                ((uint32_t)(0x1 << MXC_F_SDHC_TEST_0C_TUNINGFORSDR50_POS))

#define MXC_F_SDHC_TEST_0C_RETUNINGTIMERCNT_POS          (24)
#define MXC_F_SDHC_TEST_0C_RETUNINGTIMERCNT              ((uint32_t)(0xF << MXC_F_SDHC_TEST_0C_RETUNINGTIMERCNT_POS))

#define MXC_F_SDHC_TEST_20_MAXCURRENT1P8V_POS            (24)
#define MXC_F_SDHC_TEST_20_MAXCURRENT1P8V                ((uint32_t)(0xFF << MXC_F_SDHC_TEST_20_MAXCURRENT1P8V_POS))

#define MXC_F_SDHC_TEST_20_MAXCURRENT3P0V_POS            (16)
#define MXC_F_SDHC_TEST_20_MAXCURRENT3P0V                ((uint32_t)(0xFF << MXC_F_SDHC_TEST_20_MAXCURRENT3P0V_POS))

#define MXC_F_SDHC_TEST_20_MAXCURRENT3P3V_POS            (8)
#define MXC_F_SDHC_TEST_20_MAXCURRENT3P3V                ((uint32_t)(0xFF << MXC_F_SDHC_TEST_20_MAXCURRENT3P3V_POS))

#define MXC_F_SDHC_TEST_20_CLOCKMULTIPLIER_POS           (0)
#define MXC_F_SDHC_TEST_20_CLOCKMULTIPLIER               ((uint32_t)(0xFF << MXC_F_SDHC_TEST_20_CLOCKMULTIPLIER_POS))

#ifdef __cplusplus
}
#endif

/******************************************************************************/
/*                                                                       SDHC */
#define MXC_BASE_SDHC_TEST              ((uint32_t)0x40037000UL)
#define MXC_SDHC_TEST                   ((mxc_sdhc_test_regs_t*)MXC_BASE_SDHC_TEST)
#endif /* _SDHC_TEST_REGS_H_ */
