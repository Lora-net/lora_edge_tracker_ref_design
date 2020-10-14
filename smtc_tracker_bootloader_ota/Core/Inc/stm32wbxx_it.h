/**
  ******************************************************************************
  * @file    stm32wbxx_it.h
  * @author  MCD Application Team
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32WBxx_IT_H
#define __STM32WBxx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

void PUSH_BUTTON_SW1_EXTI_IRQHandler(void);
void PUSH_BUTTON_SW2_EXTI_IRQHandler(void);
void PUSH_BUTTON_SW3_EXTI_IRQHandler(void);
void USART1_IRQHandler(void);
void CFG_HW_USART1_DMA_TX_IRQHandler( void );
void LPUART1_IRQHandler(void);
void CFG_HW_LPUART1_DMA_TX_IRQHandler( void );
void RTC_WKUP_IRQHandler(void);
void IPCC_C1_TX_IRQHandler(void);
void IPCC_C1_RX_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32WBxx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
