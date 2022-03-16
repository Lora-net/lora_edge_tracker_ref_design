/*!
 * @file      ble_thread.c
 *
 * @brief     BLE thread implementation
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

#include "lr1110_tracker_board.h"
#include "ble_thread.h"
#include "app_ble.h"
#include "app_entry.h"
#include "app_common.h"

#include "stm32_lpm.h"
#include "stm32_seq.h"
#include "dbg_trace.h"
#include "hw_conf.h"
#include "otp.h"
#include "tracker_utility.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Defines the connection timeout
 */
#define CONNECTION_TIMEOUT 120000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Timer to handle the advertisement timeout
 */
static timer_event_t advertisement_timeout_timer;

/*!
 * @brief Timer to handle the connection timeout
 */
static timer_event_t connection_timeout_timer;

/*!
 * @brief advertisement timeout flag
 */
bool advertisement_timeout = false;

/*!
 * @brief connection timeout flag
 */
bool connection_timeout = false;

/*!
 * @brief BLE WPAN Initalized flag
 */
bool ble_is_initialized = false;

/*!
 * @brief Tracker context structure
 */
extern tracker_ctx_t tracker_ctx;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

void        SystemClock_Config( void );
static void Reset_Device( void );
static void Init_BLE_Exti( void );
static void Deinit_BLE_Exti( void );
static void Config_HSE( void );
#if( CFG_HW_RESET_BY_FW == 1 )
static void Reset_IPCC( void );
static void Reset_BackupDomain( void );
#endif

/*!
 * @brief Function executed on advertising timeout event
 */
static void on_advertisement_timeout_event( void* context );

/*!
 * @brief Function executed on connection timeout event
 */
static void on_connection_timeout_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void start_ble_thread( uint32_t adv_timeout )
{
    HAL_DBG_TRACE_INFO( "###### ===== START BLE THREAD ==== ######\r\n\r\n" );

    /* Stop Hall Effect sensors while the tracker is in BLE mode */
    lr1110_tracker_board_hall_effect_enable( false );

    Reset_Device( );
    Config_HSE( );

    /* Init BLE IRQ */
    Init_BLE_Exti( );

    /* Init code for STM32_WPAN */
    if( ble_is_initialized == false )
    {
        APPE_Init( );
        ble_is_initialized               = true;
        tracker_ctx.ble_advertisement_on = false;
    }
    else
    {
        Adv_Request( APP_BLE_FAST_ADV );
        /* In this case switch the flag to true here because it won't pass by APP_BLE_Init funtion */
        tracker_ctx.ble_advertisement_on = true;
    }

    /* Reset BLE related flags */
    advertisement_timeout        = false;
    tracker_ctx.ble_connected    = false;
    tracker_ctx.ble_disconnected = false;
    tracker_ctx.ble_cmd_received = false;

    /* Init the BLE related timer */
    if( adv_timeout != NO_ADV_TIMEOUT )
    {
        timer_init( &advertisement_timeout_timer, on_advertisement_timeout_event );
        timer_set_value( &advertisement_timeout_timer, adv_timeout );
        timer_start( &advertisement_timeout_timer );

        timer_init( &connection_timeout_timer, on_connection_timeout_event );
        timer_set_value( &connection_timeout_timer, CONNECTION_TIMEOUT );

        /* change the value of the watchog to BLE operation */
        hal_mcu_set_software_watchdog_value( CONNECTION_TIMEOUT + tracker_ctx.app_scan_interval );
        hal_mcu_start_software_watchdog( );
    }

    /* Turn on the 2G4 SPDT and set it into the right direction */
    spdt_2g4_on( );
    set_ble_antenna( );

    /* Start SMPS step down converter */
    hal_mcu_smps_enable( true );

    while( ( tracker_ctx.ble_disconnected == false ) &&
           ( tracker_ctx.ble_connected == true || advertisement_timeout == false ) )
    {
        /* Enter in the BLE sequencer */
        UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );

        if( ( adv_timeout != NO_ADV_TIMEOUT ) && ( tracker_ctx.ble_connected == true ) )
        {
            if( connection_timeout_timer.is_started == 0 )
            {
                /* Connection is established, stop the advertisement timeout \
                timer and start the connection timeout timer */
                timer_stop( &advertisement_timeout_timer );
                timer_start( &connection_timeout_timer );
            }
            else
            {
                if( tracker_ctx.ble_cmd_received == true )
                {
                    tracker_ctx.ble_cmd_received = false;

                    /* Reload the connection timeout timer */
                    timer_reset( &connection_timeout_timer );

                    /* Reload the software watchdog */
                    hal_mcu_reset_software_watchdog( );
                }
            }
        }
    }

    /* Shut down the spdt */
    spdt_2g4_off( );

    /* Stop the advertisement once the connection terminated */
    while( tracker_ctx.ble_advertisement_on == true )
    {
        Adv_Cancel_Req( );
        UTIL_SEQ_Run( UTIL_SEQ_DEFAULT );
    }

    /* Stop SMPS step down converter */
    hal_mcu_smps_enable( false );

    leds_off( LED_TX_MASK );

    /* Store the new values here only if a reset board is asked */
    if( ( tracker_ctx.new_value_to_set == true ) && ( tracker_ctx.has_lr1110_firmware == true ) )
    {
        tracker_ctx.new_value_to_set = false;
        tracker_store_app_ctx( );
    }

    /* Erase the internal log flash once the BLE thread terminated if a flush internal log is asked */
    if( tracker_ctx.internal_log_flush_request == true )
    {
        HAL_DBG_TRACE_INFO( "###### ===== FLUSH INTERNAL LOG ==== ######\r\n\r\n" );

        tracker_ctx.internal_log_flush_request = false;
        tracker_reset_internal_log( );
    }

    /* Stop the BLE advertiser and connection timer in case of quick disconnection*/
    timer_stop( &advertisement_timeout_timer );
    timer_stop( &connection_timeout_timer );

    /* Deinit BLE IRQ */
    Deinit_BLE_Exti( );

    HAL_DBG_TRACE_INFO( "###### ===== LEAVE BLE THREAD ==== ######\r\n\r\n" );

    if( tracker_ctx.lorawan_parameters_have_changed == true )
    {
        /* reset device because of LoRaWAN Parameters */
        HAL_DBG_TRACE_INFO( "###### ===== RESET TRACKER ==== ######\r\n\r\n" );
        hal_mcu_reset( );
    }

    /* set the watchdog to the right value for application operation */
    hal_mcu_set_software_watchdog_value( tracker_ctx.app_scan_interval * 3 );
    hal_mcu_start_software_watchdog( );

    /* Start Hall Effect sensors when the tracker leaves the BLE mode */
    lr1110_tracker_board_hall_effect_enable( true );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/*!
 * @brief Function executed on advertising timeout event
 */
static void on_advertisement_timeout_event( void* context ) { advertisement_timeout = true; }

/*!
 * @brief Function executed on connection timeout event
 */
static void on_connection_timeout_event( void* context ) { connection_timeout = true; }

static void Config_HSE( void )
{
    OTP_ID0_t* p_otp;

    /**
     * Read HSE_Tuning from OTP
     */
    p_otp = ( OTP_ID0_t* ) OTP_Read( 0 );
    if( p_otp )
    {
        LL_RCC_HSE_SetCapacitorTuning( p_otp->hse_tuning );
    }

    return;
}

static void Reset_Device( void )
{
#if( CFG_HW_RESET_BY_FW == 1 )
    Reset_BackupDomain( );

    Reset_IPCC( );
#endif

    return;
}

#if( CFG_HW_RESET_BY_FW == 1 )

static void Reset_IPCC( void )
{
    LL_AHB3_GRP1_EnableClock( LL_AHB3_GRP1_PERIPH_IPCC );

    LL_C1_IPCC_ClearFlag_CHx( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4 |
                                        LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    LL_C2_IPCC_ClearFlag_CHx( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 | LL_IPCC_CHANNEL_4 |
                                        LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    LL_C1_IPCC_DisableTransmitChannel( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 |
                                                 LL_IPCC_CHANNEL_4 | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    LL_C2_IPCC_DisableTransmitChannel( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 |
                                                 LL_IPCC_CHANNEL_4 | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    LL_C1_IPCC_DisableReceiveChannel( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 |
                                                LL_IPCC_CHANNEL_4 | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    LL_C2_IPCC_DisableReceiveChannel( IPCC, LL_IPCC_CHANNEL_1 | LL_IPCC_CHANNEL_2 | LL_IPCC_CHANNEL_3 |
                                                LL_IPCC_CHANNEL_4 | LL_IPCC_CHANNEL_5 | LL_IPCC_CHANNEL_6 );

    return;
}

static void Reset_BackupDomain( void )
{
    if( ( LL_RCC_IsActiveFlag_PINRST( ) != FALSE ) && ( LL_RCC_IsActiveFlag_SFTRST( ) == FALSE ) )
    {
        HAL_PWR_EnableBkUpAccess( ); /**< Enable access to the RTC registers */

        /**
         *  Write twice the value to flush the APB-AHB bridge
         *  This bit shall be written in the register before writing the next one
         */
        HAL_PWR_EnableBkUpAccess( );

        __HAL_RCC_BACKUPRESET_FORCE( );
        __HAL_RCC_BACKUPRESET_RELEASE( );
    }

    return;
}

#endif

static void Init_BLE_Exti( void )
{
    /* Enable wakeup interrupt IPCC(36), HSEM(38) */
    LL_EXTI_EnableIT_32_63( LL_EXTI_LINE_36 | LL_EXTI_LINE_38 );

    return;
}

static void Deinit_BLE_Exti( void )
{
    /* Disable wakeup interrupt IPCC(36), HSEM(38) */
    LL_EXTI_DisableIT_32_63( LL_EXTI_LINE_36 | LL_EXTI_LINE_38 );

    return;
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/
void HAL_Delay( uint32_t Delay )
{
    uint32_t tickstart = HAL_GetTick( );
    uint32_t wait      = Delay;

    /* Add a freq to guarantee minimum wait */
    if( wait < HAL_MAX_DELAY )
    {
        wait += HAL_GetTickFreq( );
    }

    while( ( HAL_GetTick( ) - tickstart ) < wait )
    {
        /************************************************************************************
         * ENTER SLEEP MODE
         ***********************************************************************************/
        LL_LPM_EnableSleep( ); /**< Clear SLEEPDEEP bit of Cortex System Control Register */

/**
 * This option is used to ensure that store operations are completed
 */
#if defined( __CC_ARM )
        __force_stores( );
#endif

        __WFI( );
    }
}

/* --- EOF ------------------------------------------------------------------ */
