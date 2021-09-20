/*!
 * @file      apps_utilities.c
 *
 * @brief     Common Application Helper function implementations
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

#include <stdlib.h>
#include <stdio.h>
#include "apps_utilities.h"
#include "lr1110_modem_lorawan.h"
#include "smtc_hal_dbg_trace.h"

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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void print_hex_buffer( const uint8_t* buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            HAL_DBG_TRACE_PRINTF( "\r\n" );
            newline = 0;
        }

        HAL_DBG_TRACE_PRINTF( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
}

void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin,
                         const bool use_semtech_join_server )
{
    HAL_DBG_TRACE_PRINTF( "DevEui      : %02X", dev_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", dev_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    HAL_DBG_TRACE_PRINTF( "AppEui      : %02X", join_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", join_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    if( use_semtech_join_server )
    {
        HAL_DBG_TRACE_MSG( "AppKey      : Semtech join server used\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "AppKey      : %02X", app_key[0] );
        for( int i = 1; i < 16; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "-%02X", app_key[i] );
        }
        HAL_DBG_TRACE_PRINTF( "\r\n" );
    }

    HAL_DBG_TRACE_PRINTF( "Pin         : %08X\r\n\r\n", pin );
}

void modem_status_to_string( lr1110_modem_status_t modem_status )
{
    HAL_DBG_TRACE_MSG( "Modem status : " );

    if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
    {
        HAL_DBG_TRACE_MSG( "CRASH " );
    }
    if( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE )
    {
        HAL_DBG_TRACE_MSG( "MUTE " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
    {
        HAL_DBG_TRACE_MSG( "JOINED " );
    }
    if( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND )
    {
        HAL_DBG_TRACE_MSG( "SUSPEND " );
    }
    if( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD )
    {
        HAL_DBG_TRACE_MSG( "UPLOAD " );
    }
    if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
    {
        HAL_DBG_TRACE_MSG( "JOINING " );
    }
    if( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM )
    {
        HAL_DBG_TRACE_MSG( "STREAM " );
    }

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
