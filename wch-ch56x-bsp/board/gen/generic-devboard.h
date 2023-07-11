/********************************** (C) COPYRIGHT *******************************
* File Name          : hydrausb3_v1.h
* Author             : bvernoux
* Version            : V1.0
* Date               : 2022/08/20
* Description        : This file contains all the functions prototypes for
*                      Board Support Package(BSP) for HydraUSB3 v1 Dev Board
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __HYDRAUSB3_V1_H__
#define __HYDRAUSB3_V1_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "core_riscv.h"
#include "CH56x_bsp.h"
void generic_bsp_gpio_init(void);
void generic_uled_on(void);
void generic_uled_off(void);
void generic_uled2_on(void);
void generic_uled2_off(void);
void generic_uled3_on(void);
void generic_uled3_off(void);
#ifdef __cplusplus
}
#endif

#endif  // __BSP_V1_H__
