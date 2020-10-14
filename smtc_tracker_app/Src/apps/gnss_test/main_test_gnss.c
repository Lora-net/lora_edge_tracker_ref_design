/*!
 * \file      main_test_gnss.c
 *
 * \brief     GNSS Test implementation
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

#include "gnss_scan.h"
#include "lr1110_tracker_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */
 
 #define EPOCH_BUFFER_LEN 10

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
extern lr1110_t lr1110;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Reset event callback
 *
 * \param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * \brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_event_t         lr1110_modem_event;
    lr1110_modem_version_t       modem;
    gnss_settings_t              gnss_settings;
    uint32_t                     unix_date     = 0;
    uint8_t                      rx_buffer[EPOCH_BUFFER_LEN] = { 0 };

    // Init board
    hal_mcu_init( );

    hal_mcu_init_periph( );

    // Init LR1110 modem event
    lr1110_modem_event.gnss_scan_done = lr1110_modem_gnss_scan_done;
    lr1110_modem_event.reset          = lr1110_modem_reset_event;
    lr1110_modem_board_init( &lr1110, &lr1110_modem_event );

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 Modem GNSS demo application ==== ######\r\n\r\n" );

    // LR1110 modem version
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n\r\n", modem.bootloader );

    // GNSS Parameters
    gnss_settings.enabled              = true;
    gnss_settings.constellation_to_use = LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK;
    gnss_settings.scan_type            = ASSISTED_MODE;
    gnss_settings.search_mode          = LR1110_MODEM_GNSS_OPTION_DEFAULT;

    // Set default position to Semtech France
    gnss_settings.assistance_position.latitude  = 45.208;
    gnss_settings.assistance_position.longitude = 5.781;
    
    lr1110_modem_gnss_set_assistance_position( &lr1110, &gnss_settings.assistance_position );

    // Wait Unix time from user
    HAL_DBG_TRACE_INFO( "###### ===== PLEASE ENTER AN UNIX DATE IN ASCII ==== ######\r\n\r\n" );

    while( unix_date == 0 )
    {
        hal_uart_rx( 2, rx_buffer, EPOCH_BUFFER_LEN );

        for( uint8_t i = 0; i < EPOCH_BUFFER_LEN; i++ )
        {
            unix_date += ( rx_buffer[9 - i] - 48 ) * pow( 10, i );
        }
    }
    modem_response_code =
        lr1110_modem_set_gps_time( &lr1110, unix_date - GNSS_EPOCH_SECONDS + GNSS_LEAP_SECONDS_OFFSET );

    HAL_DBG_TRACE_PRINTF( "\r\nEpoch time: %u\n\r", unix_date );

    while( 1 )
    {
        HAL_DBG_TRACE_MSG( "\r\nSCAN...\r\n" );
        
        /* Activate the partial low power mode to don't shut down lna during low power */
        hal_mcu_partial_sleep_enable( true );

        gnss_scan_init( &lr1110, gnss_settings );
        gnss_scan_set_antenna( GNSS_PATCH_ANTENNA );
        gnss_scan_execute( &lr1110 );
        gnss_scan_display_results( );

        HAL_Delay( 1000 );

        gnss_scan_init( &lr1110, gnss_settings );
        gnss_scan_set_antenna( GNSS_PCB_ANTENNA );
        gnss_scan_execute( &lr1110 );
        gnss_scan_display_results( );
        
        /* Deactivate the partial low power mode */
        hal_mcu_partial_sleep_enable( false );

        HAL_Delay( 1000 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_modem_board_is_ready( ) == true )
    {
        // System reset
        hal_mcu_reset( );
    }
    else
    {
        lr1110_modem_board_set_ready( true );
    }
}
