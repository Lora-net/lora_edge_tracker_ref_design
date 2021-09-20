/*!
 * @ingroup   simple_gnss_example
 * @file      main_simple_gnss_example.c
 *
 * @brief     GNSS Example implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_simple_gnss_example.h"
#include "gnss_scan.h"
#include "lr1110_tracker_board.h"
#include "smtc_hal_options.h"
#include <stdint.h>

/*!
 * @addtogroup simple_gnss_example
 * LR1110 Simple GNSS example
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief Number of expected input characters when reading the Unix epoch time.
 */
#define EPOCH_STR_LEN 10

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*!
 * @brief Read and return the current Unix epoch time from the input console
 */
static uint32_t get_unix_time( void );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_response_code_t  modem_response_code         = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_event_callback_t lr1110_modem_event_callback = { NULL };
    lr1110_modem_version_t        modem;
    gnss_settings_t               gnss_settings;
    uint32_t                      unix_time    = 0;
    antenna_t                     gnss_antenna = GNSS_PATCH_ANTENNA;
    gnss_scan_single_result_t     capture_result;

    /* Init board */
    hal_mcu_init( );

    hal_mcu_init_periph( );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.gnss_scan_done = lr1110_modem_gnss_scan_done;
    lr1110_modem_event_callback.reset          = lr1110_modem_reset_event;
    modem_response_code                        = lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );
    if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "lr1110_tracker_board_init failed (%d)\r\n", modem_response_code );
    }

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E GNSS demo application ==== ######\r\n\r\n" );

    /* LR1110 modem-e version */
    memset( &modem, 0, sizeof( modem ) );
    modem_response_code = lr1110_modem_get_version( &lr1110, &modem );
    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
        HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
        HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n\r\n", modem.bootloader );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "lr1110_modem_get_version failed (%d)\r\n", modem_response_code );
    }

    /* GNSS Parameters */
    memset( &gnss_settings, 0, sizeof( gnss_settings ) );
    gnss_settings.enabled              = true;
    gnss_settings.constellation_to_use = GNSS_CONSTELLATION;
    gnss_settings.scan_type            = GNSS_SCAN_TYPE;
    gnss_settings.search_mode          = LR1110_MODEM_GNSS_OPTION_BEST_EFFORT;

    if( gnss_settings.scan_type == ASSISTED_MODE )
    {
        /* Set approximate position for assisted mode */
        gnss_settings.assistance_position.latitude  = GNSS_ASSIST_LATITUDE;
        gnss_settings.assistance_position.longitude = GNSS_ASSIST_LONGITUDE;

        modem_response_code = lr1110_modem_gnss_set_assistance_position( &lr1110, &gnss_settings.assistance_position );
        if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
        {
            HAL_DBG_TRACE_ERROR( "lr1110_modem_gnss_set_assistance_position failed (%d)\r\n", modem_response_code );
        }

        /* Get Unix time from user */
        unix_time = get_unix_time( );

        modem_response_code =
            lr1110_modem_set_gps_time( &lr1110, unix_time - GNSS_EPOCH_SECONDS + GNSS_LEAP_SECONDS_OFFSET );
        if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
        {
            HAL_DBG_TRACE_ERROR( "lr1110_modem_set_gps_time failed (%d)\r\n", modem_response_code );
        }
    }

    while( 1 )
    {
        gnss_scan_result_t scan_result;

        HAL_DBG_TRACE_MSG( "\r\nSCAN...\r\n" );
        if( gnss_antenna == GNSS_PATCH_ANTENNA )
        {
            gnss_antenna = GNSS_PCB_ANTENNA;
            HAL_DBG_TRACE_MSG( "GNSS_PCB_ANTENNA\r\n" );
        }
        else
        {
            gnss_antenna = GNSS_PATCH_ANTENNA;
            HAL_DBG_TRACE_MSG( "GNSS_PATCH_ANTENNA\r\n" );
        }

        /* Activate the partial low power mode to don't shut down lna during low power */
        hal_mcu_partial_sleep_enable( true );

        memset( &capture_result, 0, sizeof( capture_result ) );
        scan_result = gnss_scan_execute( &lr1110, gnss_antenna, &gnss_settings, &capture_result );
        if( scan_result == GNSS_SCAN_SUCCESS )
        {
            gnss_scan_display_results( &capture_result );
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "gnss_scan_execute failed (%d)\r\n", scan_result );
        }

        /* Deactivate the partial low power mode */
        hal_mcu_partial_sleep_enable( false );

        /* Wait between two scans */
        HAL_Delay( GNSS_SCAN_PERIOD_MS );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

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

static uint32_t get_unix_time( void )
{
    uint32_t unix_time = 0;
    uint8_t  c         = 0;

    HAL_DBG_TRACE_INFO( "###### ===== PLEASE ENTER THE CURRENT UNIX EPOCH TIME IN ASCII ==== ######\r\n\r\n" );

    while( unix_time == 0 )
    {
        for( uint8_t i = 0; i < EPOCH_STR_LEN; i++ )
        {
            hal_uart_rx( HAL_PRINTF_UART_ID, &c, 1 );
#if defined( CONSOLE_INPUT_ECHO ) && ( CONSOLE_INPUT_ECHO != 0 )
            /* Input echo */
            hal_uart_tx( HAL_PRINTF_UART_ID, &c, 1 );
#endif
            if( ( c >= '0' ) && ( c <= '9' ) )
            {
                unix_time = unix_time * 10 + ( c - '0' );
            }
            else
            {
                HAL_DBG_TRACE_ERROR( "Invalid decimal character (%c, 0x%.2x)\r\n", c, c );
                unix_time = 0;
                break;
            }
        }

        if( ( unix_time != 0 ) && ( unix_time < GNSS_EPOCH_SECONDS ) )
        {
            HAL_DBG_TRACE_ERROR( "Invalid Unix epoch time (%d)\r\n", unix_time );
            unix_time = 0;
        }
    }

    HAL_DBG_TRACE_PRINTF( "\r\nEpoch time: %u\n\r", unix_time );

    return unix_time;
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
