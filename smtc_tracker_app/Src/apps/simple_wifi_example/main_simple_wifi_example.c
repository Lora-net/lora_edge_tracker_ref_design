/*!
 * @ingroup   simple_wifi_example
 * @file      main_simple_wifi_example.c
 *
 * @brief     Wi-Fi Example implementation
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
 * @addtogroup simple_wifi_example
 * LR1110 Simple Wi-Fi test application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "main_simple_wifi_example.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_event_callback_t  lr1110_modem_event_callback = { NULL };
    lr1110_modem_version_t         modem;
    wifi_settings_t                wifi_settings;
    static wifi_scan_all_results_t capture_result;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    /* Init LR1110 modem-e event */
    lr1110_modem_event_callback.wifi_scan_done = lr1110_modem_wifi_scan_done;
    lr1110_modem_event_callback.reset          = lr1110_modem_reset_event;
    lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E Wi-Fi demo application ==== ######\r\n\r\n" );

    /* LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n\r\n", modem.bootloader );

    /* Wi-Fi Parameters */
    wifi_settings.enabled       = true;
    wifi_settings.channels      = 0x3FFF;  // by default enable all channels
    wifi_settings.types         = WIFI_TYPE_SCAN;
    wifi_settings.scan_mode     = WIFI_SCAN_MODE;
    wifi_settings.nbr_retrials  = WIFI_NBR_RETRIALS;
    wifi_settings.max_results   = WIFI_MAX_RESULTS;
    wifi_settings.timeout       = WIFI_TIMEOUT_IN_MS;
    wifi_settings.result_format = LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL;

    while( 1 )
    {
        HAL_DBG_TRACE_INFO( "###### ===== Wi-FI SCAN ==== ######\r\n\r\n" );

        /* Activate the partial low power mode */
        hal_mcu_partial_sleep_enable( true );

        /* Turn on the 2G4 SPDT and set it into the right direction */
        spdt_2g4_on( );
        set_wifi_antenna( );

        if( wifi_execute_scan( &lr1110, &wifi_settings, &capture_result ) == WIFI_SCAN_SUCCESS )
        {
            lr1110_modem_display_wifi_scan_results( &capture_result );
        }
        else
        {
            HAL_DBG_TRACE_MSG( "Wi-Fi Scan error\n\r" );
        }

        /* Turn off the 2G4 SPDT */
        spdt_2g4_off( );

        /* Deactivate the partial low power mode */
        hal_mcu_partial_sleep_enable( false );

        HAL_Delay( WIFI_SCAN_PERIOD_MS );
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

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
