/*!
 * \file      main_test_wifi.c
 *
 * \brief     Wi-Fi test implementation
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

#include "wifi_scan.h"
#include "lr1110_tracker_board.h"

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
    lr1110_modem_event_t   lr1110_modem_event;
    lr1110_modem_version_t modem;
    wifi_settings_t        wifi_settings;

    // Init board
    hal_mcu_init( );

    hal_mcu_init_periph( );

    // Init LR1110 modem event
    lr1110_modem_event.wifi_scan_done = lr1110_modem_wifi_scan_done;
    lr1110_modem_event.reset          = lr1110_modem_reset_event;
    lr1110_modem_board_init( &lr1110, &lr1110_modem_event );

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 Modem Wi-Fi demo application ==== ######\r\n\r\n" );

    // LR1110 modem version
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n\r\n", modem.bootloader );

    // Wi-Fi Parameters
    wifi_settings.enabled       = true;
    wifi_settings.channels      = 0x3FFF;  // by default enable all channels
    wifi_settings.types         = LR1110_MODEM_WIFI_TYPE_SCAN_B;
    wifi_settings.scan_mode     = LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PACKET;
    wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS_DEFAULT;
    wifi_settings.max_results   = WIFI_MAX_RESULTS_DEFAULT;
    wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS_DEFAULT;
    wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    while( 1 )
    {
        wifi_init( &lr1110, wifi_settings );

        // Turn on the 2G4 SPDT and set it into the right direction
        spdt_2g4_on( );
        set_wifi_antenna( );

        wifi_execute_scan( &lr1110 );

        // Turn off the 2G4 SPDT
        spdt_2g4_off( );

        lr1110_display_wifi_scan_results( );

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
