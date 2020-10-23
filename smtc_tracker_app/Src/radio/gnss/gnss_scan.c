/*!
 * \file      gnss_scan.c
 *
 * \brief     GNSS scan implementation.
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

/*!
 * \brief GNSS scan state machine timeout
 */
#define GNSS_SCAN_TIMEOUT ( 15000 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief GNSS global parameters
 */
gnss_t gnss;

/*!
 * \brief GNSS scan type parameter
 */
static uint8_t scan_type = ASSISTED_MODE;

/*!
 * \brief GNSS scan timeout flag
 */
static bool gnss_scan_timeout = false;

/*!
 * \brief Timer to handle the scan timeout
 */
static timer_event_t gnss_scan_timeout_timer;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief configure by default the gnss scanner
 *
 * \param [in] context Radio abstraction
 *
 * \param [in] settings gnss settings to apply \ref gnss_settings_t
 */
static void gnss_scan_configure( const void* context, gnss_settings_t settings );

/*!
 * \brief Function executed on gnss scan timeout event
 *
 * \param [in] context Radio abstraction
 */
static void on_gnss_scan_timeout_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_gnss_scan_done( uint8_t* buffer, uint16_t size )
{
    memcpy( gnss.capture_result.result_buffer, buffer, size );
    gnss.capture_result.result_size = size;

    gnss.state = GNSS_GET_RESULTS;
}

void gnss_scan_set_type( uint8_t type )
{
    if( type == ASSISTED_MODE )
    {
        scan_type = ASSISTED_MODE;
    }
    else
    {
        scan_type = AUTONOMOUS_MODE;
    }
}

void gnss_scan_set_antenna( antenna_t antenna )
{
    if( antenna == GNSS_PATCH_ANTENNA )
    {
        set_gnss_patch_antenna( );
    }
    else
    {
        set_gnss_pcb_antenna( );
    }

    gnss.capture_result.antenna = antenna;
}

void gnss_scan_init( const void* context, gnss_settings_t settings )
{
    gnss.state                      = GNSS_START_SCAN;
    gnss.capture_result.result_size = 0;

    timer_init( &gnss_scan_timeout_timer, on_gnss_scan_timeout_event );
    timer_set_value( &gnss_scan_timeout_timer, GNSS_SCAN_TIMEOUT );

    gnss_scan_configure( context, settings );
}

gnss_scan_result_t gnss_scan_execute( const void* context )
{
    bool                         gnss_scan_done = false;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    uint8_t                      nb_detected_satellites = 0;
    gnss_scan_result_t           scan_result            = GNSS_SCAN_SUCCESS;

    gnss_scan_timeout = false;

    timer_start( &gnss_scan_timeout_timer );

    while( ( gnss_scan_done != true ) && ( gnss_scan_timeout != true ) )
    {
        // Process Event
        if( ( ( lr1110_t* ) context )->event.callback != NULL )
        {
            lr1110_modem_event_process( context );
        }

        switch( gnss.state )
        {
        case GNSS_START_SCAN:

            /* Switch on the LNA */
            lr1110_modem_board_lna_on( );

            if( scan_type == AUTONOMOUS_MODE )
            {
                modem_response_code = lr1110_modem_gnss_scan_autonomous_md( context, gnss.settings.search_mode,
                                                        gnss.settings.input_paramaters, gnss.settings.nb_sat );
            }
            else
            {
                modem_response_code = lr1110_modem_gnss_scan_assisted_md(
                    context, gnss.settings.search_mode, gnss.settings.input_paramaters, gnss.settings.nb_sat );
            }

            // If response code different than RESPONSE_CODE_OK leave
            if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_NO_TIME )
            {
                gnss_scan_done = true;
                scan_result    = GNSS_SCAN_NO_TIME;
            }
            else if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
            {
                gnss_scan_done = true;
                scan_result    = GNSS_SCAN_FAIL;
            }

            gnss.state = GNSS_LOW_POWER;
            break;

        case GNSS_GET_RESULTS:

            modem_response_code = lr1110_modem_gnss_get_nb_detected_satellites( context, &nb_detected_satellites );
            gnss.capture_result.nb_detected_satellites = nb_detected_satellites;
            modem_response_code = lr1110_modem_gnss_get_detected_satellites( context, nb_detected_satellites,
                                                                             gnss.capture_result.detected_satellites );

            gnss.state = GNSS_TERMINATED;

            break;

        case GNSS_TERMINATED:
            gnss.state     = GNSS_START_SCAN;
            gnss_scan_done = true;
            break;

        case GNSS_LOW_POWER:
            // The MCU wakes up through events
            hal_mcu_low_power_handler( );
            break;
        }
    }

    /* Switch off the LNA */
    lr1110_modem_board_lna_off( );

    timer_stop( &gnss_scan_timeout_timer );

    if( gnss_scan_timeout == true )
    {
        scan_result = GNSS_SCAN_FAIL;
    }

    return scan_result;
}

void gnss_scan_display_results( void )
{
    uint8_t i = 0;

    /* Antenna */
    if( gnss.capture_result.antenna == GNSS_PATCH_ANTENNA )
    {
        HAL_DBG_TRACE_MSG( "CAPTURE ON PATCH ANTENNA\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "CAPTURE ON PCB ANTENNA\r\n" );
    }

    /* Satellites infos */

    HAL_DBG_TRACE_PRINTF( "Nb Detected satellites : %d \r\n", gnss.capture_result.nb_detected_satellites );

    HAL_DBG_TRACE_MSG( "Satellites infos : \r\n" );

    for( i = 0; i < gnss.capture_result.nb_detected_satellites; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "ID = %d -- CN = %d \r\n", gnss.capture_result.detected_satellites[i].satellite_id,
                              gnss.capture_result.detected_satellites[i].cnr );
    }

    /*  NAV Message */

    HAL_DBG_TRACE_MSG( "NAV = " );

    for( i = 0; i < gnss.capture_result.result_size; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%02X", gnss.capture_result.result_buffer[i] );
    }

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_gnss_scan_timeout_event( void* context ) { gnss_scan_timeout = true; }

static void gnss_scan_configure( const void* context, gnss_settings_t settings )
{
    gnss.settings.search_mode = settings.search_mode;
    gnss.settings.input_paramaters =
        LR1110_MODEM_GNSS_BIT_CHANGE_MASK | LR1110_MODEM_GNSS_DOPPLER_MASK | LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK;
    lr1110_modem_gnss_set_constellations_to_use( context, settings.constellation_to_use );
    gnss.settings.nb_sat = 0;

    gnss_scan_set_type( settings.scan_type );
}

/* --- EOF ------------------------------------------------------------------ */

