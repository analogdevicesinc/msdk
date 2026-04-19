/**
 * @file    hpb.h
 * @brief   HyperBus (HPB) function prototypes and data types.
 */

/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. (now owned by 
 * Analog Devices, Inc.),
 * Copyright (C) 2023-2024 Analog Devices, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************/

/* Define to prevent redundant inclusion */
#ifndef LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_HPB_H_
#define LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_HPB_H_

/* **** Includes **** */
#include "hpb_regs.h"
#include "mxc_sys.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup hpb HyperBus (HPB)
 * @ingroup periphlibs
 * @{
 */

/* **** Definitions **** */
/**
 *  Structure type containing register offset-value pairs.
 */
typedef struct {
    uint32_t addr; /**< Offset from port base  */
    uint32_t val; /* < Value                  */
} mxc_hpb_cfg_reg_val_t;

/**
 * Enumeration type to select the number of cycles CS is high. 
 */
typedef enum {
    MXC_HPB_CS_HIGH_1_5 = 0, /**< CS High 1.5 clock cycles */
    MXC_HPB_CS_HIGH_2_5, /**< CS High 2.5 clock cycles */
    MXC_HPB_CS_HIGH_3_5, /**< CS High 3.5 clock cycles */
    MXC_HPB_CS_HIGH_4_5, /**< CS High 4.5 clock cycles */
    MXC_HPB_CS_HIGH_5_5, /**< CS High 5.5 clock cycles */
    MXC_HPB_CS_HIGH_6_5, /**< CS High 6.5 clock cycles */
    MXC_HPB_CS_HIGH_7_5, /**< CS High 7.5 clock cycles */
    MXC_HPB_CS_HIGH_8_5, /**< CS High 8.5 clock cycles */
    MXC_HPB_CS_HIGH_9_5, /**< CS High 9.5 clock cycles */
    MXC_HPB_CS_HIGH_10_5, /**< CS High 10.5 clock cycles */
    MXC_HPB_CS_HIGH_11_5, /**< CS High 11.5 clock cycles */
    MXC_HPB_CS_HIGH_12_5, /**< CS High 12.5 clock cycles */
    MXC_HPB_CS_HIGH_13_5, /**< CS High 13.5 clock cycles */
    MXC_HPB_CS_HIGH_14_5, /**< CS High 14.5 clock cycles */
    MXC_HPB_CS_HIGH_15_5, /**< CS High 15.5 clock cycles */
    MXC_HPB_CS_HIGH_16_5, /**< CS High 16.5 clock cycles */
} mxc_hpb_cs_high_t;

/**
 * Enumeration type to select the number of clock cycles between asserting the CS signal and the first clock cycle. 
 */
typedef enum {
    MXC_HPB_CS_SETUP_HOLD_1 = 0x0, /**< CS Setup/Hold 1 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_2, /**< CS Setup/Hold 2 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_3, /**< CS Setup/Hold 3 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_4, /**< CS Setup/Hold 4 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_5, /**< CS Setup/Hold 5 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_6, /**< CS Setup/Hold 6 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_7, /**< CS Setup/Hold 7 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_8, /**< CS Setup/Hold 8 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_9, /**< CS Setup/Hold 9 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_10, /**< CS Setup/Hold 10 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_11, /**< CS Setup/Hold 11 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_12, /**< CS Setup/Hold 12 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_13, /**< CS Setup/Hold 13 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_14, /**< CS Setup/Hold 14 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_15, /**< CS Setup/Hold 15 clock cycles */
    MXC_HPB_CS_SETUP_HOLD_16, /**< CS Setup/Hold 16 clock cycles */
} mxc_hpb_cs_setup_hold_t;

/**
 * Enumeration type to select the number of clock cycles for the latency of RAM operations.
 */
typedef enum {
    MXC_HPB_LATENCY_5 = MXC_V_HPB_MTR_LATENCY_5CLK, /**< 5 clock latency for RAM */
    MXC_HPB_LATENCY_6 = MXC_V_HPB_MTR_LATENCY_6CLK, /**< 6 clock latency for RAM */
    MXC_HPB_LATENCY_3 = MXC_V_HPB_MTR_LATENCY_3CLK, /**< 3 clock latency for RAM */
    MXC_HPB_LATENCY_4 = MXC_V_HPB_MTR_LATENCY_4CLK, /**< 4 clock latency for RAM */
} mxc_hpb_latency_t;

/**
 * Enumeration type to select the type of device connected to the HPB controller.
 */
typedef enum {
    MXC_HPB_DEV_HYPER_FLASH = MXC_V_HPB_MCR_DEV_TYPE_HYPERFLASH,
    MXC_HPB_DEV_XCCELA_PSRAM = MXC_V_HPB_MCR_DEV_TYPE_XCCELAPSRAM,
    MXC_HPB_DEV_HYPER_RAM = MXC_V_HPB_MCR_DEV_TYPE_HYPERRAM,
} mxc_hpb_device_t;

/**
 * Structure type to configure the HPB controller.
 */
typedef struct {
    /** The base address for memory space */
    uint32_t base_addr;

    /** Type of device attached to controller */
    mxc_hpb_device_t device_type;

    /** Pointer to array of address offset/value pairs */
    const mxc_hpb_cfg_reg_val_t *cfg_reg_val;

    /** number of configuration pairs */
    unsigned int cfg_reg_val_len;

    /** Before the read access, this setting inserts the CK cycles to the chip select */
    /** high period. */
    mxc_hpb_cs_high_t read_cs_high;

    /** Before the write access, this setting inserts the CK cycles to the chip select */
    /** high period. */
    mxc_hpb_cs_high_t write_cs_high;

    /** In the read access, this setting inserts the CK cycles, between the falling edge */
    /** of chip select and the rising edge of first CK. */
    mxc_hpb_cs_setup_hold_t read_cs_setup;

    /** In the write access, this setting inserts the CK cycles, between the falling */
    /** edge of chip select and the rising edge of first CK. */
    mxc_hpb_cs_setup_hold_t write_cs_setup;

    /** In the read access, this setting inserts the CK cycles, between the falling */
    /** edge of last CK and the rising edge of chip select. */
    mxc_hpb_cs_setup_hold_t read_cs_hold;

    /** In the write access, this setting inserts the CK cycles, between the falling */
    /** edge of last CK and the rising edge of chip select */
    mxc_hpb_cs_setup_hold_t write_cs_hold;

    /** Latency Cycle for HyperRAM mode, ignored when the connected device is HyperFlash */
    mxc_hpb_latency_t latency_cycle;

    unsigned int fixed_latency;
} mxc_hpb_mem_config_t;

/* **** Function Prototypes **** */
/**
 * @brief Read a variable
 * @param cfg_reg_val  Pointer to configuration struct to read a variable
 * @param base_addr    Base address
 * @param index        0 or 1 to determine which configuration settings    
*/
void MXC_HPB_RegRead8(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index);

/**
 * @brief Read a variable
 * @param cfg_reg_val  Pointer to configuration struct to read a variable
 * @param base_addr    Base address
 * @param index        0 or 1 to determine which configuration settings    
*/
void MXC_HPB_RegWrite8(const mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr,
                       unsigned int index);

/**
 * @brief Read a variable
 * @param cfg_reg_val  Pointer to configuration struct to read a variable
 * @param base_addr    Base address
 * @param index        0 or 1 to determine which configuration settings    
*/
void MXC_HPB_RegRead16(mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr, unsigned int index);

/**
 * @brief Read a variable
 * @param cfg_reg_val  Pointer to configuration struct to read a variable
 * @param base_addr    Base address
 * @param index        0 or 1 to determine which configuration settings    
*/
void MXC_HPB_RegWrite16(const mxc_hpb_cfg_reg_val_t *cfg_reg_val, uint32_t base_addr,
                        unsigned int index);

/**
 * @brief Configure the HyperBus peripheral.
 * @param      mem0 Pointer to configuration struct for mem0 (may be NULL if
 *             this memory not used)
 * @param      mem1 Pointer to configuration struct for mem1 (may be NULL if
 *             this memory not used)
 * @return #E_BAD_PARAM if configuration error, #E_NO_ERROR otherwise
 */
int MXC_HPB_Init(const mxc_hpb_mem_config_t *mem0, const mxc_hpb_mem_config_t *mem1);

/**
 * @brief   Returns the contents of the status register.
 * @note    Use MXC_F_HPB_CSR macros to filter out specific status bits.
 * @return  HPB status register.
 */
uint32_t MXC_HPB_GetStatus(void);

/**
 * @brief   Enable HPB interrupt.
 * @param   polarity  1 to use active high, 0 to use active low.
 */
void MXC_HPB_EnableInt(unsigned polarity);

/**
 * @brief   Get HPB interrupt status.
 * @return  1 if flag is set, 0 if not.
 */
unsigned MXC_HPB_GetFlag(void);

/**@} end of group hpb */

#ifdef __cplusplus
}
#endif

#endif // LIBRARIES_PERIPHDRIVERS_INCLUDE_MAX32650_HPB_H_
