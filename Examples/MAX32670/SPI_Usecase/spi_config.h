/******************************************************************************
 *
 * Copyright (C) 2022-2023 Maxim Integrated Products, Inc. All Rights Reserved.
 * (now owned by Analog Devices, Inc.),
 * Copyright (C) 2023 Analog Devices, Inc. All Rights Reserved. This software
 * is proprietary to Analog Devices, Inc. and its licensors.
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

#ifndef EXAMPLES_MAX32670_SPI_USECASE_SPI_CONFIG_H_
#define EXAMPLES_MAX32670_SPI_USECASE_SPI_CONFIG_H_

#include "max32670.h"
#include "spi_regs.h"
#include "spi.h"

/*** SPI Master Configuration ***/
#define SPIx_MASTER MXC_SPI0
#define SPI_BAUD_RATE 1000000

/*** SPI Slave Configuration ***/
#define SPIx_SLAVE MXC_SPI1
#define SPIx_IRQn SPI1_IRQn
#define SPIx_IRQHandler SPI1_IRQHandler

//
#define TEST_BUFF_SIZE 64

/*** Functions ***/
int spi_master_init(void);
int spi_master_send_rcv(unsigned char *src, unsigned int srcLen, unsigned char *dst);

int spi_slave_init(void);
int spi_slave_send(unsigned char *src, unsigned int srcLen);
int spi_slave_rcv(unsigned char *dst, unsigned int maxLen, unsigned int *len);

#endif // EXAMPLES_MAX32670_SPI_USECASE_SPI_CONFIG_H_
