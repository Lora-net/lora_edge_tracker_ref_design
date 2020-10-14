/** 
  ******************************************************************************
  * @file    stm32wbxx_nucleo.h
  * @author  MCD Application Team
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32WBXX-Nucleo Kit 
  *            from STMicroelectronics
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics. 
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the 
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32WBXX_TRACKER_H
#define __STM32WBXX_TRACKER_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32WBXX_TRACKER
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
   
/** @defgroup STM32WBXX_NUCLEO_Exported_Types Exported Types
  * @{
  */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  /* Color led aliases */
  LED_GREEN  = LED1,
  LED_RED    = LED2
}Led_TypeDef;

/**
  * @}
  */ 

/** @defgroup STM32WBXX_NUCLEO_Exported_Constants Exported Constants
  * @{
  */ 

/** 
  * @brief Define for STM32WBXX_NUCLEO board  
  */ 
#if !defined (USE_STM32WBXX_TRACKER)
 #define USE_STM32WBXX_TRACKER
#endif

/** @defgroup STM32WBXX_NUCLEO_LED LED Constants
  * @{
  */
#define LEDn                                    2

#define LED1_PIN                                GPIO_PIN_5
#define LED1_GPIO_PORT                          GPIOB
#define LED1_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()

#define LED2_PIN                                GPIO_PIN_4
#define LED2_GPIO_PORT                          GPIOE
#define LED2_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()                 __HAL_RCC_GPIOE_CLK_ENABLE()


/** @addtogroup STM32WBXX_NUCLEO_Exported_Functions
  * @{
  */
uint32_t         BSP_GetVersion(void);  

/** @addtogroup STM32WBXX_NUCLEO_LED_Functions
  * @{
  */
void BSP_LED_Init(Led_TypeDef Led);
void BSP_LED_DeInit(Led_TypeDef Led);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);
void BSP_Default_Board_Init( void );

#ifdef __cplusplus
}
#endif

#endif /* __STM32WBXX_TRACKER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

