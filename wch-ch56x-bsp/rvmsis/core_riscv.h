/********************************** (C) COPYRIGHT  *******************************
* File Name          : core_riscv.h
* Author             : WCH, bvernoux
* Version            : V1.1
* Date               : 2022/08/07
* Description        : RISC-V Core Peripheral Access Layer Header File
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#ifndef __CORE_RV3A_H__
#define __CORE_RV3A_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "CH56xSFR.h"
#include <stdint.h>
/* IO definitions */
#ifdef __cplusplus
#define     __I     volatile                /*!< defines 'read only' permissions      */
#else
#define     __I     volatile const          /*!< defines 'read only' permissions     */
#endif
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */
#define   RV_STATIC_INLINE  static  inline

//typedef enum {SUCCESS = 0, ERROR = !SUCCESS} ErrorStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

/* memory mapped structure for Program Fast Interrupt Controller (PFIC) */
typedef struct __attribute__((packed))
{
	__I  uint32_t ISR[8];
	__I  uint32_t IPR[8];
	__IO uint32_t ITHRESDR;
	__IO uint32_t FIBADDRR;
	__IO uint32_t CFGR;
	__I  uint32_t GISR;
	uint8_t RESERVED0[0x10];
	__IO uint32_t FIOFADDRR[4];
	uint8_t RESERVED1[0x90];
	__O  uint32_t IENR[8];
	uint8_t RESERVED2[0x60];
	__O  uint32_t IRER[8];
	uint8_t RESERVED3[0x60];
	__O  uint32_t IPSR[8];
	uint8_t RESERVED4[0x60];
	__O  uint32_t IPRR[8];
	uint8_t RESERVED5[0x60];
	__IO uint32_t IACTR[8];
	uint8_t RESERVED6[0xE0];
	__IO uint8_t IPRIOR[256];
	uint8_t RESERVED7[0x810];
	__IO uint32_t SCTLR;
}
PFIC_Type;

/* memory mapped structure for SysTick */
typedef struct __attribute__((packed))
{
	__IO uint32_t CTLR;
	__IO uint64_t CNT;
	__IO uint64_t CMP;
	__IO uint32_t CNTFG;
}
SysTick_Type;

#define PFIC            ((PFIC_Type *) 0xE000E000 )
#define SysTick         ((SysTick_Type *) 0xE000F000)

#define PFIC_KEY1       ((uint32_t)0xFA050000)
#define	PFIC_KEY2		((uint32_t)0xBCAF0000)
#define	PFIC_KEY3		((uint32_t)0xBEEF0000)

/*********************************************************************
 * @fn      __NOP
 *
 * @brief   nop
 *
 * @return  none
 */
RV_STATIC_INLINE void __NOP()
{
	__asm volatile ("nop");
}

/*********************************************************************
 * @fn      PFIC_EnableIRQ
 *
 * @brief   Enable Interrupt
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  none
 */
RV_STATIC_INLINE void PFIC_EnableIRQ(IRQn_Type IRQn)
{
	PFIC->IENR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      PFIC_DisableIRQ
 *
 * @brief   Disable Interrupt
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  none
 */
RV_STATIC_INLINE void PFIC_DisableIRQ(IRQn_Type IRQn)
{
	uint32_t t;

	t = PFIC->ITHRESDR;
	PFIC->ITHRESDR = 0x10;
	PFIC->IRER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
	PFIC->ITHRESDR = t;
}

/*********************************************************************
 * @fn      PFIC_GetStatusIRQ
 *
 * @brief   Get Interrupt Enable State
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  1 - Interrupt Enable
 *          0 - Interrupt Disable
 */
RV_STATIC_INLINE uint32_t PFIC_GetStatusIRQ(IRQn_Type IRQn)
{
	return((uint32_t) ((PFIC->ISR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      PFIC_GetPendingIRQ
 *
 * @brief   Get Interrupt Pending State
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  1 - Interrupt Pending Enable
 *          0 - Interrupt Pending Disable
 */
RV_STATIC_INLINE uint32_t PFIC_GetPendingIRQ(IRQn_Type IRQn)
{
	return((uint32_t) ((PFIC->IPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      PFIC_SetPendingIRQ
 *
 * @brief   Set Interrupt Pending
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  None
 */
RV_STATIC_INLINE void PFIC_SetPendingIRQ(IRQn_Type IRQn)
{
	PFIC->IPSR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      PFIC_ClearPendingIRQ
 *
 * @brief   Clear Interrupt Pending
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  None
 */
RV_STATIC_INLINE void PFIC_ClearPendingIRQ(IRQn_Type IRQn)
{
	PFIC->IPRR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}

/*********************************************************************
 * @fn      PFIC_GetActive
 *
 * @brief   Get Interrupt Active State
 *
 * @param   IRQn: Interrupt Numbers
 *
 * @return  1 - Interrupt Active
 *          0 - Interrupt No Active
 */
RV_STATIC_INLINE uint32_t PFIC_GetActive(IRQn_Type IRQn)
{
	return((uint32_t)((PFIC->IACTR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}

/*********************************************************************
 * @fn      PFIC_SetPriority
 *
 * @brief   Set Interrupt Priority
 *
 * @param   IRQn - Interrupt Numbers
 *          priority -
 *              bit7 - pre-emption priority
 *              bit6~bit4 - subpriority
 * @return  None
 */
RV_STATIC_INLINE void PFIC_SetPriority(IRQn_Type IRQn, uint8_t priority)
{
	PFIC->IPRIOR[(uint32_t)(IRQn)] = priority;
}

/*********************************************************************
 * @fn      __SEV
 *
 * @brief   Wait for Events
 *
 * @return  None
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __SEV(void)
{
	PFIC->SCTLR |= (1<<3);
}

/*********************************************************************
 * @fn      __WFI
 *
 * @brief   Wait for Interrupt
 *
 * @return  None
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __WFI(void)
{
	PFIC->SCTLR &= ~(1<<3);	// wfi
	asm volatile ("wfi");
}

/*********************************************************************
 * @fn      __WFE
 *
 * @brief   Wait for Events
 *
 * @return  None
 */
__attribute__( ( always_inline ) ) RV_STATIC_INLINE void __WFE(void)
{
	PFIC->SCTLR |= (1<<3)|(1<<5);		// (wfi->wfe)+(__sev)
	asm volatile ("wfi");
	PFIC->SCTLR |= (1<<3);
	asm volatile ("wfi");
}

/*********************************************************************
 * @fn      PFIC_SetFastIRQ
 *
 * @brief   Set VTF Interrupt
 *
 * @param   add - VTF interrupt service function base address.
 *          IRQn -Interrupt Numbers
 *          num - VTF Interrupt Numbers
 * @return  None
 */
RV_STATIC_INLINE void PFIC_SetFastIRQ(uint32_t addr, IRQn_Type IRQn, uint8_t num)
{
	if(num > 3)  return ;
	PFIC->FIBADDRR = addr;
	PFIC->FIOFADDRR[num] = ((uint32_t)IRQn<<24)|(addr&0xfffff);
}

/*********************************************************************
 * @fn      PFIC_SystemReset
 *
 * @brief   Initiate a system reset request
 *
 * @return  None
 */
RV_STATIC_INLINE void PFIC_SystemReset(void)
{
	PFIC->CFGR = PFIC_KEY3|(1<<7);
}

/*********************************************************************
 * @fn      PFIC_HaltPushCfg
 *
 * @brief   Enable Hardware Stack
 *
 * @param   NewState - DISABLE or ENABLE

 * @return  None
 */
RV_STATIC_INLINE void PFIC_HaltPushCfg(FunctionalState NewState)
{
	if (NewState != DISABLE)
	{
		PFIC->CFGR = PFIC_KEY1;
	}
	else
	{
		PFIC->CFGR = PFIC_KEY1|(1<<0);
	}
}

/*********************************************************************
 * @fn      PFIC_INTNestCfg
 *
 * @brief   Enable Interrupt Nesting
 *
 * @param   NewState - DISABLE or ENABLE

 * @return  None
 */
RV_STATIC_INLINE void PFIC_INTNestCfg(FunctionalState NewState)
{
	if (NewState != DISABLE)
	{
		PFIC->CFGR = PFIC_KEY1;
	}
	else
	{
		PFIC->CFGR = PFIC_KEY1|(1<<1);
	}
}

#define SysTick_LOAD_RELOAD_Msk            (0xFFFFFFFFFFFFFFFFULL)
#define SysTick_CTRL_RELOAD_Msk            (1 << 8)
#define SysTick_CTRL_CLKSOURCE_Msk         (1 << 2)
#define SysTick_CTRL_TICKINT_Msk           (1 << 1)
#define SysTick_CTRL_ENABLE_Msk            (1 << 0)

RV_STATIC_INLINE uint32_t SysTick_Config( uint64_t ticks )
{
	if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)  return (1); /* Reload value impossible */

	SysTick->CMP  = ticks - 1; /* set reload register */
	PFIC_EnableIRQ( SysTick_IRQn );
	SysTick->CTLR  = SysTick_CTRL_RELOAD_Msk    |
					 SysTick_CTRL_CLKSOURCE_Msk |
					 SysTick_CTRL_TICKINT_Msk   |
					 SysTick_CTRL_ENABLE_Msk; /* Enable SysTick IRQ and SysTick Timer */
	return (0); /* Function successful */
}

/* Core_Exported_Functions */
extern uint32_t __get_FFLAGS(void);
extern void __set_FFLAGS(uint32_t value);
extern uint32_t __get_FRM(void);
extern void __set_FRM(uint32_t value);
extern uint32_t __get_FCSR(void);
extern void __set_FCSR(uint32_t value);

/* Requires Machine privilege */
extern uint32_t __get_MSTATUS(void);
extern void __set_MSTATUS(uint32_t value);
extern uint32_t __get_MISA(void);
extern void __set_MISA(uint32_t value);
extern uint32_t __get_MIE(void);
extern void __set_MIE(uint32_t value);
extern uint32_t __get_MTVEC(void);
extern void __set_MTVEC(uint32_t value);
extern uint32_t __get_MSCRATCH(void);
extern void __set_MSCRATCH(uint32_t value);
extern uint32_t __get_MEPC(void);
extern void __set_MEPC(uint32_t value);
extern uint32_t __get_MCAUSE(void);
extern void __set_MCAUSE(uint32_t value);
extern uint32_t __get_MTVAL(void);
extern void __set_MTVAL(uint32_t value);
extern uint32_t __get_MIP(void);
extern void __set_MIP(uint32_t value);
/*
// Disabled / Not supported on CH569
extern uint32_t __get_MCYCLE(void);
extern void __set_MCYCLE(uint32_t value);
extern uint32_t __get_MCYCLEH(void);
extern void __set_MCYCLEH(uint32_t value);
extern uint32_t __get_MINSTRET(void);
extern void __set_MINSTRET(uint32_t value);
extern uint32_t __get_MINSTRETH(void);
extern void __set_MINSTRETH(uint32_t value);
*/
extern uint32_t __get_MVENDORID(void);
extern uint32_t __get_MARCHID(void);
extern uint32_t __get_MIMPID(void);
extern uint32_t __get_MHARTID(void);
extern uint32_t __get_SP(void);

#ifdef __cplusplus
}
#endif


#endif/* __CORE_RV3A_H__ */





