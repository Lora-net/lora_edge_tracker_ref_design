/*!
 * \file      smtc_hal_mcu.c
 *
 * \brief     Board specific package MCU API implementation.
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32wbxx_hal.h"
#include "stm32wbxx_ll_utils.h"
#include "lr1110_tracker_board.h"
#include "smtc_hal.h"

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#endif

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief Radio hardware and global parameters
 */
lr1110_t lr1110;

/*!
 * \brief Low Power options
 */
typedef enum low_power_mode_e
{
    LOW_POWER_ENABLE,
    LOW_POWER_DISABLE,
    LOW_POWER_DISABLE_ONCE
} low_power_mode_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

static volatile bool             hal_exit_wait        = false;
static volatile low_power_mode_t hal_lp_current_mode  = LOW_POWER_ENABLE;
static bool                      partial_sleep_enable = false;

/*!
 * \brief Timer to handle the software watchdog
 */
static timer_event_t soft_watchdog;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief init the MCU clock tree
 */
static void hal_mcu_system_clock_config( void );

/*!
 * \brief reinit the MCU clock tree after a stop mode
 */
static void hal_mcu_system_clock_re_config_after_stop( void );

/*!
 * \brief init the GPIO
 */
static void hal_mcu_gpio_init( void );

/*!
 * \brief deinit the GPIO
 */
static void hal_mcu_gpio_deinit( void );

/*!
 * \brief init the power voltage detector
 */
static void hal_mcu_pvd_config( void );

/*!
 * \brief Deinit the MCU
 */
static void hal_mcu_deinit( void );

/*!
 * \brief reinit the peripherals
 */
static void hal_mcu_reinit_periph( void );

/*!
 * \brief deinit the peripherals 
 */
static void hal_mcu_deinit_periph( void );

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
/*!
 * \brief printf
 */
static void vprint( const char* fmt, va_list argp );
#endif

/*!
 * \brief Function executed on software watchdog event
 */
static void on_soft_watchdog_event( void* context );

/*!
 * \brief Configure the STM32WB SMPS
 */
static void hal_mcu_smps_config( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hal_mcu_critical_section_begin( uint32_t* mask )
{
    *mask = __get_PRIMASK( );
    __disable_irq( );
}

void hal_mcu_critical_section_end( uint32_t* mask ) { __set_PRIMASK( *mask ); }

void hal_mcu_init_periph( void )
{
    // Init TX & RX Leds
    leds_init( );

    // Enable user button
    usr_button_init( );

    // External supplies
    external_supply_init( LNA_SUPPLY_MASK | SPDT_2G4_MASK | VCC_SENSORS_SUPPLY_MASK );

    // Switch
    pe4259_wifi_ble_init( );

    // LIS2DE12 accelerometer
    accelerometer_init( INT_1 );

    // Effect Hall sensor
    lr1110_modem_board_hall_effect_enable( true );
}

static void hal_mcu_reinit_periph( void )
{
    // Leds
    leds_init( );

    // Enable user button
    usr_button_init( );

    // External supplies
    external_supply_init( LNA_SUPPLY_MASK | SPDT_2G4_MASK );

    // Switch
    pe4259_wifi_ble_init( );
}

void hal_mcu_deinit_periph( void )
{
    // Leds
    leds_deinit( );

    // Disable bothe user button
    usr_button_deinit( );

    // Disable external supply
    external_supply_deinit( LNA_SUPPLY_MASK | SPDT_2G4_MASK );

    // Switch
    pe4259_wifi_ble_deinit( );

    hal_mcu_gpio_deinit( );
}

void hal_mcu_init( void )
{
    // Initialize MCU HAL library
    HAL_Init( );

    // Initialize clocks
    hal_mcu_system_clock_config( );

    // Initialize GPIOs
    hal_mcu_gpio_init( );

    // Initialize low power timer
    hal_tmr_init( );

    // Initialize the user flash
    flash_init( );

    // Init power voltage voltage detector
    hal_mcu_pvd_config( );

    // Initialize UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, BOARD_DBG_PIN_TX, BOARD_DBG_PIN_RX );
#endif

    // Initialize SPI
    hal_spi_init( HAL_RADIO_SPI_ID, RADIO_MOSI, RADIO_MISO, RADIO_SCLK );
    lr1110_modem_board_init_io_context( &lr1110 );
    // Init LR1110 IO
    lr1110_modem_board_init_io( &lr1110 );

    // Initialize RTC
    hal_rtc_init( );
    
    // Initialize ADC
    hal_adc_init( );

    // Initialize I2C
    hal_i2c_init( HAL_I2C_ID, I2C_SDA, I2C_SCL );
}

void hal_mcu_disable_irq( void ) { __disable_irq( ); }

void hal_mcu_enable_irq( void ) { __enable_irq( ); }

void hal_mcu_reset( void )
{
    __disable_irq( );

    // Restart system
    NVIC_SystemReset( );
}

void hal_mcu_panic( void )
{
    CRITICAL_SECTION_BEGIN( );

    HAL_DBG_TRACE_ERROR( "%s\n", __FUNCTION__ );
    HAL_DBG_TRACE_ERROR( "PANIC" );

    // reset the board
    hal_mcu_reset( );
}

void hal_mcu_wait_us( const int32_t microseconds )
{
    const uint32_t nb_nop = microseconds * 1000 / 561;
    for( uint32_t i = 0; i < nb_nop; i++ )
    {
        __NOP( );
    }
}

void hal_mcu_init_software_watchdog( uint32_t value )
{
    timer_init( &soft_watchdog, on_soft_watchdog_event );
    timer_set_value( &soft_watchdog, value );
    timer_start( &soft_watchdog );
}

void hal_mcu_set_software_watchdog_value( uint32_t value )
{
    timer_set_value( &soft_watchdog, value );
}

void hal_mcu_start_software_watchdog( void )
{
    timer_start( &soft_watchdog );
}

void hal_mcu_reset_software_watchdog( void )
{
    timer_reset( &soft_watchdog );
}

uint16_t hal_mcu_get_vref_level( void )
{
    return hal_adc_get_vref_int( );
}

void hal_mcu_disable_low_power_wait( void )
{
    hal_exit_wait       = true;
    hal_lp_current_mode = LOW_POWER_DISABLE;
}

void hal_mcu_enable_low_power_wait( void )
{
    hal_exit_wait       = false;
    hal_lp_current_mode = LOW_POWER_ENABLE;
}

void hal_mcu_disable_once_low_power_wait( void )
{
    hal_exit_wait       = true;
    hal_lp_current_mode = LOW_POWER_DISABLE_ONCE;
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
}

void hal_mcu_trace_print( const char* fmt, ... )
{
#if HAL_DBG_TRACE == HAL_FEATURE_ON
    va_list argp;
    va_start( argp, fmt );
    vprint( fmt, argp );
    va_end( argp );
#endif
}

#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line
 * number where the assert_param error has occurred. Input          : - file:
 * pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    // User can add his own implementation to report the file name and line
    // number,
    // ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line)

    HAL_DBG_TRACE_PRINTF( "Wrong parameters value: file %s on line %lu\r\n", ( const char* ) file, line );
    // Infinite loop
    while( 1 )
    {
    }
}
#endif

void hal_mcu_partial_sleep_enable( bool enable ) { partial_sleep_enable = enable; }

void hal_mcu_system_clock_forward_LSE( bool enable )
{
    if( enable )
    {
        HAL_RCCEx_EnableLSCO( RCC_LSCOSOURCE_LSE );
    }
    else
    {
        HAL_RCCEx_DisableLSCO( );
    }
}

void hal_mcu_smps_enable( bool enable )
{
    if( enable )
    {
        LL_PWR_SMPS_Enable( );
    }
    else
    {
        LL_PWR_SMPS_Disable( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void hal_mcu_system_clock_config( void )
{
    RCC_OscInitTypeDef       RCC_OscInitStruct   = { 0 };
    RCC_ClkInitTypeDef       RCC_ClkInitStruct   = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );
    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE ;
    RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState            = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State          = RCC_HSI48_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        hal_mcu_panic( );
    }
    /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4 | RCC_CLOCKTYPE_HCLK2 | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 ) != HAL_OK )
    {
        hal_mcu_panic( );
    }
    /** Initializes the peripherals clocks
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS | RCC_PERIPHCLK_RFWAKEUP | RCC_PERIPHCLK_RTC |
                                               RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USB |
                                               RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.Usart1ClockSelection   = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInitStruct.I2c1ClockSelection     = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInitStruct.UsbClockSelection      = RCC_USBCLKSOURCE_HSI48;
    PeriphClkInitStruct.RTCClockSelection      = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
    PeriphClkInitStruct.SmpsClockSelection     = RCC_SMPSCLKSOURCE_HSE;
    PeriphClkInitStruct.SmpsDivSelection       = RCC_SMPSCLKDIV_RANGE1;

    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInitStruct ) != HAL_OK )
    {
        hal_mcu_panic( );
    }
    
    /* Configure the SMPS */
    hal_mcu_smps_config( );

    /*Configure GPIO pin : PA2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_LSCO;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    hal_mcu_system_clock_forward_LSE( true );
}

static void hal_mcu_smps_config( void )
{
   /* 
    *   See AN5246 from ST
    *
    *   TX level of +0 dBm (Tx Code = 25) and a digital
    *   activity at 20 mA maximum, then we have to set the VFBSMPS at a voltage higher than
    *   1.4 V + 15 mV (load impact) + 10 mV (trimming accuracy), that is VFBSMPS > 1.425 V, which
    *   gives 1.450 V
    *
    *   SMPSVOS = (VFBSMPS – 1.5 V) / 50 mV + SMPS_coarse_engi_trim
    *
    *   uint8_t smps_coarse_engi_trim = (*( __IO uint32_t* ) 0x1FFF7559) & 0x0F;
    *   uint8_t smps_fine_engi_trim = (*( __IO uint32_t* ) 0x1FFF7549) & 0x0F;
    *   float vfbsmps = 1.45;
    *
    *   smpsvos = ( ( vfbsmps - 1.5 ) / 0.050 ) + smps_coarse_engi_trim;
    */

    /* Configure the SMPS */
    LL_PWR_SMPS_SetStartupCurrent(LL_PWR_SMPS_STARTUP_CURRENT_80MA);
    LL_PWR_SMPS_SetOutputVoltageLevel(LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50); // smpsvos should be 6 meaning LL_PWR_SMPS_OUTPUT_VOLTAGE_1V50
    LL_PWR_SMPS_SetMode(LL_PWR_SMPS_STEP_DOWN);
}

static void hal_mcu_pvd_config( void )
{
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_1;
    sConfigPVD.Mode     = PWR_PVD_MODE_IT_RISING;
    if( HAL_PWR_ConfigPVD( &sConfigPVD ) != HAL_OK )
    {
        assert_param( FAIL );
    }

    // Enable PVD
    HAL_PWR_EnablePVD( );

    // Enable and set PVD Interrupt priority
    HAL_NVIC_SetPriority( PVD_PVM_IRQn, 0, 0 );
    HAL_NVIC_EnableIRQ( PVD_PVM_IRQn );
}

static void hal_mcu_gpio_init( void )
{
    // GPIO Ports Clock Enable
    __HAL_RCC_GPIOA_CLK_ENABLE( );
    __HAL_RCC_GPIOB_CLK_ENABLE( );
    __HAL_RCC_GPIOC_CLK_ENABLE( );
    __HAL_RCC_GPIOE_CLK_ENABLE( );
    __HAL_RCC_GPIOH_CLK_ENABLE( );

#if( HAL_HW_DEBUG_PROBE == HAL_FEATURE_ON )
    // Enable debug in sleep/stop/standby
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );
#endif
}

static void hal_mcu_gpio_deinit( void )
{
    /* Disable GPIOs clock */
    __HAL_RCC_GPIOA_CLK_DISABLE( );
    __HAL_RCC_GPIOB_CLK_DISABLE( );
    __HAL_RCC_GPIOC_CLK_DISABLE( );
    __HAL_RCC_GPIOD_CLK_DISABLE( );
    __HAL_RCC_GPIOE_CLK_DISABLE( );
    __HAL_RCC_GPIOH_CLK_DISABLE( );
}

void HAL_MspInit( void )
{
    HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );

    /* HSEM Clock enable */
    __HAL_RCC_HSEM_CLK_ENABLE( );

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( MemoryManagement_IRQn, 0, 0 );
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( BusFault_IRQn, 0, 0 );
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( UsageFault_IRQn, 0, 0 );
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SVCall_IRQn, 0, 0 );
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( DebugMonitor_IRQn, 0, 0 );
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( PendSV_IRQn, 0, 0 );
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
}

/**
 * \brief Enters Low Power Stop Mode
 *
 * \note ARM exits the function when waking up
 */
static void hal_mcu_lpm_enter_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on MSI
    CRITICAL_SECTION_BEGIN( );

    if( partial_sleep_enable == true )
    {
        hal_mcu_deinit( );
    }
    else
    {
        hal_mcu_deinit_periph( );
        hal_mcu_deinit( );
    }

    CRITICAL_SECTION_END( );

    /* 	In case of debugger probe attached, work-around of issue specified in "ES0394 - STM32WB55Cx/Rx/Vx device
       errata": 2.2.9 Incomplete Stop 2 mode entry after a wakeup from debug upon EXTI line 48 event "With the JTAG
       debugger enabled on GPIO pins and after a wakeup from debug triggered by an event on EXTI line 48 (CDBGPWRUPREQ),
       the device may enter in a state in which attempts to enter Stop 2 mode are not fully effective ..."
    */
    LL_EXTI_DisableIT_32_63( LL_EXTI_LINE_48 );
    LL_C2_EXTI_DisableIT_32_63( LL_EXTI_LINE_48 );

    LL_C2_PWR_SetPowerMode( LL_PWR_MODE_SHUTDOWN );

    // Enter Stop Mode
    HAL_PWREx_EnterSTOP2Mode( PWR_STOPENTRY_WFI );
}

/*!
 * \brief Exists Low Power Stop Mode
 */
static void hal_mcu_lpm_exit_stop_mode( void )
{
    // Disable IRQ while the MCU is not running on MSI
    CRITICAL_SECTION_BEGIN( );

    // Reinitializes the mcu
    hal_mcu_reinit( );

    if( partial_sleep_enable == false )
    {
        // Reinitializes the peripherals
        hal_mcu_reinit_periph( );
    }

    CRITICAL_SECTION_END( );
}

/*!
 * \brief handler low power (TODO: put in a new smtc_hal_lpm with option)
 */
void hal_mcu_low_power_handler( void )
{
#if( HAL_LOW_POWER_MODE == HAL_FEATURE_ON )
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending
     * and cortex will not enter low power anyway
     */

    hal_mcu_lpm_enter_stop_mode( );
    hal_mcu_lpm_exit_stop_mode( );

    __enable_irq( );
#endif
}

static void hal_mcu_deinit( void )
{
    hal_spi_deinit( HAL_RADIO_SPI_ID );
    lr1110_modem_board_deinit_io( &lr1110 );
    // Disable I2C
    hal_i2c_deinit( HAL_I2C_ID );
    // Disable UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_deinit( HAL_PRINTF_UART_ID );
#endif
}

void hal_mcu_reinit( void )
{
    // Reconfig needed OSC and PLL
    hal_mcu_system_clock_re_config_after_stop( );

    // Initialize I2C
    hal_i2c_init( HAL_I2C_ID, I2C_SDA, I2C_SCL );

    // Initialize UART
#if( HAL_USE_PRINTF_UART == HAL_FEATURE_ON )
    hal_uart_init( HAL_PRINTF_UART_ID, BOARD_DBG_PIN_TX, BOARD_DBG_PIN_RX );
#endif

    // Initialize SPI
    hal_spi_init( HAL_RADIO_SPI_ID, RADIO_MOSI, RADIO_MISO, RADIO_SCLK );
    // Init LR1110 IO
    lr1110_modem_board_init_io( &lr1110 );
}

static void hal_mcu_system_clock_re_config_after_stop( void )
{
    CRITICAL_SECTION_BEGIN( );

    /**Configure LSE Drive Capability
     */
    __HAL_RCC_LSEDRIVE_CONFIG( RCC_LSEDRIVE_LOW );

    // Enable HSE
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    // Wait till HSE is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    // Enable HSI
    __HAL_RCC_HSI_ENABLE( );

    // Wait till MSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    // Select HSE as system clock source
    __HAL_RCC_SYSCLK_CONFIG( RCC_SYSCLKSOURCE_HSE );

    // Wait till HSE is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_HSE )
    {
    }

    CRITICAL_SECTION_END( );
}

#if( HAL_DBG_TRACE == HAL_FEATURE_ON )
static void vprint( const char* fmt, va_list argp )
{
    char string[HAL_PRINT_BUFFER_SIZE];
    if( 0 < vsprintf( string, fmt, argp ) )  // build string
    {
        hal_uart_tx( 1, ( uint8_t* ) string, strlen( string ) );
    }
}
#endif

static void on_soft_watchdog_event( void* context )
{
    HAL_DBG_TRACE_INFO( "###### ===== WATCHDOG RESET ==== ######\r\n\r\n" );
    /* System reset */
    hal_mcu_reset( );
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler( void )
{
    HAL_DBG_TRACE_ERROR( "HardFault_Handler\n\r" );

    // reset the board
    hal_mcu_reset( );
}

/*!
 * \brief  This function handles PVD interrupt request.
 * \param  None
 * \retval None
 */
void PVD_PVM_IRQHandler( void )
{
    HAL_DBG_TRACE_ERROR( "PVD_PVM_IRQHandler\n\r" );
    // Loop inside the handler to prevent the Cortex from using the Flash,
    // allowing the flash interface to finish any ongoing transfer.
    while( __HAL_PWR_GET_FLAG( PWR_FLAG_PVDO ) != RESET )
    {
    }
    // Then reset the board
    hal_mcu_reset( );
}

/* --- EOF ------------------------------------------------------------------ */
