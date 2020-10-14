/**
  ******************************************************************************
  * @file    stm32wbxx_nucleo.c
  * @author  MCD Application Team
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32WBXX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
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

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_tracker.h"

/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32WBXX_NUCLEO STM32WBxx-Nucleo
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32WBxx-Nucleo Kit from STMicroelectronics.
  *        It provides also LCD, joystick and uSD functions to communicate with 
  *        Adafruit 1.8" TFT LCD shield (reference ID 802)
  * @{
  */ 

/** @defgroup STM32WBXX_NUCLEO_Private_Defines Private Defines
  * @{
  */ 

/**
  * @brief STM32WBxx NUCLEO BSP Driver
  */
#define __STM32WBxx_NUCLEO_BSP_VERSION_MAIN   (0x01U) /*!< [31:24] main version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB1   (0x00U) /*!< [23:16] sub1 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_SUB2   (0x01U) /*!< [15:8]  sub2 version */
#define __STM32WBxx_NUCLEO_BSP_VERSION_RC     (0x00U) /*!< [7:0]  release candidate */ 
#define __STM32WBxx_NUCLEO_BSP_VERSION        ((__STM32WBxx_NUCLEO_BSP_VERSION_MAIN << 24)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32WBxx_NUCLEO_BSP_VERSION_RC))   


/**
  * @}
  */ 

/** @defgroup STM32WBXX_NUCLEO_LOW_LEVEL_Private_Variables Private Variables
  * @{
  */ 
GPIO_TypeDef* GPIO_PORT[LEDn] = {LED1_GPIO_PORT, LED2_GPIO_PORT};
const uint16_t GPIO_PIN[LEDn] = {LED1_PIN, LED2_PIN};

/** @defgroup STM32WBXX_NUCLEO_Exported_Functions Exported Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32WBxx NUCLEO BSP Driver revision
  * @retval version: 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32WBxx_NUCLEO_BSP_VERSION;
}

/** @defgroup STM32WBXX_NUCLEO_LED_Functions LED Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: LED to be configured. 
  *          This parameter can be one of the following values:
  *            @arg LED1
  *            @arg LED2
  *            @arg LED3
  * @retval None
  */
void BSP_LED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  
  /* Enable the GPIO_LED Clock */
  LED1_GPIO_CLK_ENABLE();
  LED2_GPIO_CLK_ENABLE();  
  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin = GPIO_PIN[Led];
  gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(GPIO_PORT[Led], &gpioinitstruct);
  
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  LED1
  *     @arg  LED2
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  * @retval None
  */
void BSP_LED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = GPIO_PIN[Led];
  HAL_GPIO_DeInit(GPIO_PORT[Led], gpio_init_structure.Pin);
	
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  * @retval None
  */
void BSP_LED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  * @retval None
  */
void BSP_LED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  * @retval None
  */
void BSP_LED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}
    
void BSP_Default_Board_Init( void )
{
	GPIO_InitTypeDef  gpioinitstruct = {0};
  
	/* Enable VCC for SPDT */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpioinitstruct.Pin = GPIO_PIN_3;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull = GPIO_NOPULL;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); 
  
	/* turn the SPDT on BLE PATH */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpioinitstruct.Pin = GPIO_PIN_15;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull = GPIO_NOPULL;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);   
	
	/* Minimal configuration for LR1110 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpioinitstruct.Pin = GPIO_PIN_0 | GPIO_PIN_4;
	gpioinitstruct.Mode = GPIO_MODE_OUTPUT_PP;
	gpioinitstruct.Pull = GPIO_NOPULL;
	gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  
	HAL_GPIO_Init(GPIOA, &gpioinitstruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_SET);   
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
