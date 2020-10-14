/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @brief   BLE application with BLE core
 *
  @verbatim
  ==============================================================================
                    ##### IMPORTANT NOTE #####
  ==============================================================================

  This application requests having the stm32wb5x_BLE_Stack_fw.bin binary
  flashed on the Wireless Coprocessor.
  If it is not the case, you need to use STM32CubeProgrammer to load the appropriate
  binary.

  All available binaries are located under following directory:
  /Projects/STM32_Copro_Wireless_Binaries

  Refer to UM2237 to learn how to use/install STM32CubeProgrammer.
  Refer to /Projects/STM32_Copro_Wireless_Binaries/ReleaseNote.html for the
  detailed procedure to change the Wireless Coprocessor binary.

  @endverbatim
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


/* Includes ------------------------------------------------------------------*/
#include "app_common.h"

#include "app_entry.h"
#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "dbg_trace.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Global variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc = { 0 }; /**< RTC handler declaration */

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void Reset_BackupDomain( void );
static void Init_RTC( void );
static void SystemClock_Config( void );
static void Reset_Device( void );
static void Reset_IPCC( void );
static void Init_Exti( void );

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main( void )
{
  HAL_Init();

  Reset_Device();

  BSP_Default_Board_Init();

  /**
   * When the application is expected to run at higher speed, it should be better to set the correct system clock
   * in system_stm32yyxx.c so that the initialization phase is running at max speed.
   */
  SystemClock_Config(); /**< Configure the system clock */

  Init_Exti( );

  Init_RTC();

  APPE_Init( );

  while(1)
  {
    UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
static void Init_Exti( void )
{
  /**< Disable all wakeup interrupt on CPU1  except IPCC(36), HSEM(38) */
  LL_EXTI_DisableIT_0_31(~0);
  LL_EXTI_DisableIT_32_63( (~0) & (~(LL_EXTI_LINE_36 | LL_EXTI_LINE_38)) );

  return;
}

static void Reset_Device( void )
{
#if ( CFG_HW_RESET_BY_FW == 1 )
  Reset_BackupDomain();

  Reset_IPCC();
#endif

  return;
}

static void Reset_IPCC( void )
{
  LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_IPCC);

  LL_C1_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_ClearFlag_CHx(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableTransmitChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C1_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  LL_C2_IPCC_DisableReceiveChannel(
      IPCC,
      LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4
      | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6);

  return;
}

static void Reset_BackupDomain( void )
{
  if ((LL_RCC_IsActiveFlag_PINRST() != FALSE) && (LL_RCC_IsActiveFlag_SFTRST() == FALSE))
  {
    HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

    /**
     *  Write twice the value to flush the APB-AHB bridge
     *  This bit shall be written in the register before writing the next one
     */
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
  }

  return;
}

static void Init_RTC( void )
{
  HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */

  /**
   *  Write twice the value to flush the APB-AHB bridge
   *  This bit shall be written in the register before writing the next one
   */
  HAL_PWR_EnableBkUpAccess();

  __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE); /**< Select LSI as RTC Input */

  __HAL_RCC_RTC_ENABLE(); /**< Enable RTC */

  hrtc.Instance = RTC; /**< Define instance */

  /**
   * Set the Asynchronous prescaler
   */
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  HAL_RTC_Init(&hrtc);

  /* Disable RTC registers write protection */
  LL_RTC_DisableWriteProtection(RTC);

  LL_RTC_WAKEUP_SetClock(RTC, CFG_RTC_WUCKSEL_DIVIDER);

  /* Enable RTC registers write protection */
  LL_RTC_EnableWriteProtection(RTC);

  return;
}

/**
 * @brief  Configure the system clock
 *
 * @note   This API configures
 *         - The system clock source
 *           - The AHBCLK, APBCLK dividers
 *           - The flash latency
 *           - The PLL settings (when required)
 *
 * @param  None
 * @retval None
 */
void SystemClock_Config( void )
{
#if (CFG_USB_INTERFACE_ENABLE != 0)
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
  RCC_CRSInitTypeDef RCC_CRSInitStruct = { 0 };

  /**
   * This prevents the CPU2 to disable the HSI48 oscillator when
   * it does not use anymore the RNG IP
   */
  LL_HSEM_1StepLock( HSEM, 5 );

  LL_RCC_HSI48_Enable();

  while(!LL_RCC_HSI48_IsReady());

  /* Select HSI48 as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /*Configure the clock recovery system (CRS)**********************************/

  /* Enable CRS Clock */
  __HAL_RCC_CRS_CLK_ENABLE();

  /* Default Synchro Signal division factor (not divided) */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;

  /* Set the SYNCSRC[1:0] bits according to CRS_Source value */
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;

  /* HSI48 is synchronized with USB SOF at 1KHz rate */
  RCC_CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
  RCC_CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;

  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;

  /* Set the TRIM[5:0] to the default value*/
  RCC_CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

  /* Start automatic synchronization */
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
#endif

        /**
         *  Write twice the value to flush the APB-AHB bridge to ensure the  bit is written
         */
        HAL_PWR_EnableBkUpAccess(); /**< Enable access to the RTC registers */
        HAL_PWR_EnableBkUpAccess();

  /**
   * Select LSE clock
   */
  LL_RCC_LSE_Enable();
  while(!LL_RCC_LSE_IsReady());

  /**
   * Select wakeup source of BLE RF
   */
  LL_RCC_SetRFWKPClockSource(LL_RCC_RFWKP_CLKSOURCE_LSE);

  return;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += HAL_GetTickFreq();
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
    /************************************************************************************
     * ENTER SLEEP MODE
     ***********************************************************************************/
    LL_LPM_EnableSleep( ); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

    /**
     * This option is used to ensure that store operations are completed
     */
  #if defined ( __CC_ARM)
    __force_stores();
  #endif

    __WFI( );
  }
}

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
