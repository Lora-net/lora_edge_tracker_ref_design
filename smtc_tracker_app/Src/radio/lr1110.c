/*!
 * \file      lr1110.c
 *
 * \brief     LR1110 top level implementation
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

#include "lr1110.h"
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
 * \brief lr1110 modem event callback functions
 */
lr1110_modem_event_t* lr1110_modem_event;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Hardware INT IRQ callback initialization
 */
void radio_event_callback( void* obj );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */
 
void radio_event_init( lr1110_modem_event_t* event ) { lr1110_modem_event = event; }

void lr1110_modem_event_process( const void* context )
{
    if( lr1110_modem_board_read_event_line( context ) == 1 )
    {
        lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
        lr1110_modem_event_fields_t  event_fields;

        do
        {
            modem_response_code = lr1110_modem_get_event( context, &event_fields );

            if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
            {
                switch( event_fields.event_type )
                {
                case LR1110_MODEM_LORAWAN_EVENT_RESET:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->reset != NULL ) )
                    {
                        lr1110_modem_event->reset( ( event_fields.buffer[0] << 8 ) + event_fields.buffer[1] );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_ALARM:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->alarm != NULL ) )
                    {
                        lr1110_modem_event->alarm( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_JOINED:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->joined != NULL ) )
                    {
                        lr1110_modem_event->joined( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->join_fail != NULL ) )
                    {
                        lr1110_modem_event->join_fail( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_TX_DONE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->tx_done != NULL ) )
                    {
                        lr1110_modem_event->tx_done( ( lr1110_modem_tx_done_event_t ) event_fields.buffer[0] );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_DOWN_DATA:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->down_data != NULL ) )
                    {
                        int8_t  rssi  = ( ( int8_t ) event_fields.buffer[0] ) - 64;
                        int8_t  snr   = ( ( ( int8_t ) event_fields.buffer[1] ) >> 2 );
                        uint8_t flags = event_fields.buffer[2];
                        uint8_t port  = event_fields.buffer[3];
                        uint8_t buffer_size = event_fields.buffer_len - 4;  // remove rssi/snr/flags and port from buffer

                        for( uint8_t i = 0; i < buffer_size; i++ )
                        {
                            event_fields.buffer[i] = event_fields.buffer[i + 4];
                        }

                        lr1110_modem_event->down_data( rssi, snr, ( lr1110_modem_down_data_flag_t ) flags, port, event_fields.buffer, buffer_size );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_UPLOAD_DONE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->upload_done != NULL ) )
                    {
                        uint8_t session_id      = ( event_fields.buffer[0] >> 4 ) & 0x03;
                        uint8_t session_counter = event_fields.buffer[0] & 0x0F;

                        lr1110_modem_event->upload_done( session_id, session_counter );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_SET_CONF:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->set_conf != NULL ) )
                    {
                        lr1110_modem_event->set_conf( event_fields.buffer[0] );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_MUTE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->mute != NULL ) )
                    {
                        lr1110_modem_event->mute( ( lr1110_modem_mute_t ) event_fields.buffer[0] );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_STREAM_DONE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->stream_done != NULL ) )
                    {
                        lr1110_modem_event->stream_done( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_WIFI_SCAN_DONE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->wifi_scan_done != NULL ) )
                    {
                        lr1110_modem_event->wifi_scan_done( event_fields.buffer, event_fields.buffer_len );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->gnss_scan_done != NULL ) )
                    {
                        lr1110_modem_event->gnss_scan_done( event_fields.buffer, event_fields.buffer_len );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_TIME_UPDATED_ALC_SYNC:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->time_updated_alc_sync != NULL ) )
                    {
                        uint8_t sync_state = event_fields.buffer[0];
                        lr1110_modem_event->time_updated_alc_sync( ( lr1110_modem_alc_sync_state_t ) sync_state );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_ADR_MOBILE_TO_STATIC:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->adr_mobile_to_static != NULL ) )
                    {
                        lr1110_modem_event->adr_mobile_to_static( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_NEW_LINK_ADR:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->new_link_adr != NULL ) )
                    {
                        lr1110_modem_event->new_link_adr( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_NO_EVENT:
                    if( ( lr1110_modem_event != NULL ) && ( lr1110_modem_event->no_event != NULL ) )
                    {
                        lr1110_modem_event->no_event( );
                    }
                    break;
                default:
                    break;
                }
            }
        } while( ( lr1110_modem_board_read_event_line( context ) == 1 ) && ( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK ) );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void radio_event_callback( void* obj ) {}

/* --- EOF ------------------------------------------------------------------ */
