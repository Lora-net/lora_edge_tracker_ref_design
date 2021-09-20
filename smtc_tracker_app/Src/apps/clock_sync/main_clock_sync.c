/*!
 * @ingroup   apps_clock_sync
 * @file      main_clock_sync.c
 *
 * @brief     LR1110 Simple LoRaWAN clock synchronization example implementation
 *
 * @copyright
 * @parblock
 * Revised BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
 * @endparblock
 */

/*!
 * @addtogroup apps_clock_sync
 * LR1110 Simple LoRaWAN clock synchronization example
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_clock_sync.h"
#include "apps_utilities.h"
#include "lorawan_commissioning.h"
#include "lr1110_tracker_board.h"
#include "utilities.h"

#include <stdio.h>

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
 * @brief LR1110 Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Lorawan default init
 *
 * @param [in] region LoRaWAN region to use \ref lr1110_modem_regions_t
 * @param [in] lorawan_class LoRaWAN class to use \ref lr1110_modem_classes_t
 *
 * @returns Operation status
 */
static lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class );

/*!
 * @brief Executes the network Join request
 */
static void join_network( void );

/*!
 * @brief Start a the Modem-E Alarm to read the GPS clock after a delay
 */
static void start_modeme_alarm( void );

/*!
 * @addtogroup lr1110_evt_callback
 * LR1110 event callbacks
 * @{
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void lr1110_modem_network_joined( void );

/*!
 * @brief Join Fail event callback
 */
static void lr1110_modem_join_fail( void );

/*!
 * @brief Alarm event callback
 */
static void lr1110_modem_alarm( void );

/*!
 * @brief  Time Updated by application layer clock synchronization.
 *
 * @param [in] alc_sync_state \ref lr1110_modem_alc_sync_state_t
 */
void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state );

/*!
 * @}
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_version_t        modem_version;
    lr1110_modem_event_callback_t lr1110_modem_event_callback = { NULL };

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO(
        "###### ===== LoRa Basics Modem-E Application Layer Clock Synchronization example ==== ######\r\n\r\n" );

    /* Register LR1110 modem-e event callback functions */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.reset                 = lr1110_modem_reset_event;
    lr1110_modem_event_callback.alarm                 = lr1110_modem_alarm;
    lr1110_modem_event_callback.joined                = lr1110_modem_network_joined;
    lr1110_modem_event_callback.join_fail             = lr1110_modem_join_fail;
    lr1110_modem_event_callback.time_updated_alc_sync = lr1110_modem_time_updated_alc_sync;

    if( lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n" );
    }

    /* Display the LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &modem_version );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E VERSION ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem_version.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem_version.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n", modem_version.bootloader );

    /* Proceed to basic LoRaWAN configuration */
    if( lorawan_init( LORAWAN_REGION_USED, LORAWAN_CLASS_USED ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n" );
    }

    /* Activate ALC Sync */
    modem_response_code = lr1110_modem_set_alc_sync_mode( &lr1110, LR1110_MODEM_ALC_SYNC_MODE_ENABLE );
    if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== SET ALC SYNC MODE FAILED ==== ######\r\n\r\n" );
    }

    /* Join the LoRaWAN Network */
    join_network( );

    while( 1 )
    {
        /* Process LR1110 Events */
        if( lr1110.event.callback != NULL )
        {
            lr1110_modem_event_process( &lr1110 );
        }

        /* Activate the Low Power Mode, this will put the MCU to sleep.
         * Events from the LR1110 modem-e will wake the MCU up. */
        if( lr1110_tracker_board_read_event_line( &lr1110 ) == false )
        {
            hal_mcu_low_power_handler( );
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    uint32_t                     pin;
    uint8_t                      dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
    uint8_t                      join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
    uint8_t                      app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

    modem_response_code |= lr1110_modem_set_class( &lr1110, lorawan_class );

    if( lorawan_class == LR1110_LORAWAN_CLASS_A )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : A\r\n" );
    }
    if( lorawan_class == LR1110_LORAWAN_CLASS_C )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : C\r\n" );
    }

    modem_response_code |= lr1110_modem_set_region( &lr1110, region );

    switch( region )
    {
    case LR1110_LORAWAN_REGION_EU868:
    {
        HAL_DBG_TRACE_MSG( "REGION      : EU868\r\n\r\n" );
        modem_response_code |= lr1110_modem_activate_duty_cycle( &lr1110, LORAWAN_DUTYCYCLE_ON );

        modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, 0 );
        break;
    }
    case LR1110_LORAWAN_REGION_US915:
    {
        HAL_DBG_TRACE_MSG( "REGION      : US915\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AU915:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AU915\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP1:
    {
        if( LORAWAN_COUNTRY_JAPAN == 1 )
        {
            HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
            /* Activate LBT for 5ms before each transmission with a threshold at -80 dBm */
            modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -80, 5, 1250000 );
        }

        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP1\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_CN470:
    {
        HAL_DBG_TRACE_MSG( "REGION      : CN470\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP2:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP2\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP3:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP3\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_IN865:
    {
        HAL_DBG_TRACE_MSG( "REGION      : IN865\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_KR920:
    {
        HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
        HAL_DBG_TRACE_MSG( "REGION      : KR920\r\n\r\n" );

        /* Activate LBT for 5ms before each transmission with a threshold at -65 dBm */
        modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -65, 5, 1250000 );
        break;
    }
    case LR1110_LORAWAN_REGION_RU864:
    {
        HAL_DBG_TRACE_MSG( "REGION      : RU864\r\n\r\n" );
        break;
    }
    default:
        HAL_DBG_TRACE_ERROR( "No supported region selected\r\n\r\n" );
        break;
    }

#if( USE_PRODUCTION_KEYS == 1 )
    modem_response_code |= lr1110_modem_get_dev_eui( &lr1110, dev_eui );
    modem_response_code |= lr1110_modem_get_join_eui( &lr1110, join_eui );
#endif
    /* Set Keys */
    modem_response_code |= lr1110_modem_set_dev_eui( &lr1110, dev_eui );
    modem_response_code |= lr1110_modem_set_join_eui( &lr1110, join_eui );

#if( USE_SEMTECH_JOIN_SERVER == 0 )
    modem_response_code |= lr1110_modem_set_app_key( &lr1110, app_key );
#else
    modem_response_code |= lr1110_modem_derive_keys( &lr1110 );
    modem_response_code |= lr1110_modem_get_pin( &lr1110, &pin );
#endif
    print_lorawan_keys( dev_eui, join_eui, app_key, pin, USE_SEMTECH_JOIN_SERVER );

    return modem_response_code;
}
static void join_network( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    /* Starts the join procedure */
    modem_response_code = lr1110_modem_join( &lr1110 );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== JOINING CMD ERROR %d==== ######\r\n\r\n", modem_response_code );
    }
}

static void start_modeme_alarm( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== START MODEM-E ALARM ==== ######\r\n\r\n" );

    do
    {
        modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, APP_GETTIME_PERIOD_MS / 1000 );
        if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
        {
            HAL_DBG_TRACE_ERROR( "lr1110_modem_set_alarm_timer failed (%d)\r\n\r\n", modem_response_code );
        }
        /* Make sure to retry after a Busy/Timeout error,
         *  you don't want to go to sleep without setting the alarm. */
    } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );
}

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_tracker_board_is_ready( ) == true )
    {
        /* System reset */
        hal_mcu_reset( );
    }
    else
    {
        lr1110_tracker_board_set_ready( true );
    }
}

void lr1110_modem_network_joined( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== JOINED ==== ######\r\n\r\n" );

    /* Set the ADR profile once joined */
    lr1110_modem_set_adr_profile( &lr1110, LORAWAN_DEFAULT_DATARATE, adr_custom_list );

    /* Start displaying the GPS time */
    start_modeme_alarm( );
}

void lr1110_modem_join_fail( void ) { HAL_DBG_TRACE_INFO( "###### ===== JOINED FAIL ==== ######\r\n\r\n" ); }

void lr1110_modem_alarm( void )
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== LR1110 ALARM ==== ######\r\n\r\n" );

    modem_response_code = lr1110_modem_get_status( &lr1110, &modem_status );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        modem_status_to_string( modem_status );

        if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
        {
            hal_mcu_reset( );
        }
        else
        {
            /* Get and display the LR1110 GPS time. */
            uint32_t gps_time = 0;
            lr1110_modem_get_gps_time( &lr1110, &gps_time );
            HAL_DBG_TRACE_PRINTF( "lr1110_modem_get_gps_time: %d\r\n\r\n", gps_time );
            /* Restart the alarm */
            start_modeme_alarm( );
        }
    }
}

void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state )
{
    if( alc_sync_state == LR1110_MODEM_ALC_SYNC_DESYNCHRONIZED )
    {
        HAL_DBG_TRACE_INFO( "###### ===== LR1110 CLOCK DESYNCHRONIZED ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "###### ===== LR1110 CLOCK SYNCHRONIZED ==== ######\r\n\r\n" );
    }
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
