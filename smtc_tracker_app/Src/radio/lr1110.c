/*!
 * @file      lr1110.c
 *
 * @brief     LR1110 top level implementation
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
 * @brief lr1110 modem-e event callback functions
 */
lr1110_modem_event_callback_t* lr1110_modem_event_callback;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Hardware INT IRQ callback initialization
 */
void radio_event_callback( void* obj );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void radio_event_init( lr1110_modem_event_callback_t* event ) { lr1110_modem_event_callback = event; }

void lr1110_modem_event_process( const void* context )
{
    if( lr1110_tracker_board_read_event_line( context ) == 1 )
    {
        lr1110_modem_helper_status_t modem_response_code = LR1110_MODEM_HELPER_STATUS_OK;
        lr1110_modem_event_t         modem_event;

        do
        {
            modem_response_code = lr1110_modem_helper_get_event_data( context, &modem_event );

            if( modem_response_code == LR1110_MODEM_HELPER_STATUS_OK )
            {
                switch( modem_event.event_type )
                {
                case LR1110_MODEM_LORAWAN_EVENT_RESET:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->reset != NULL ) )
                    {
                        lr1110_modem_event_callback->reset( modem_event.event_data.reset.count );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_ALARM:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->alarm != NULL ) )
                    {
                        lr1110_modem_event_callback->alarm( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_JOINED:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->joined != NULL ) )
                    {
                        lr1110_modem_event_callback->joined( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_JOIN_FAIL:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->join_fail != NULL ) )
                    {
                        lr1110_modem_event_callback->join_fail( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_TX_DONE:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->tx_done != NULL ) )
                    {
                        lr1110_modem_event_callback->tx_done( modem_event.event_data.txdone.status );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_DOWN_DATA:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->down_data != NULL ) )
                    {
                        lr1110_modem_event_callback->down_data(
                            modem_event.event_data.downdata.rssi, modem_event.event_data.downdata.snr,
                            modem_event.event_data.downdata.flag, modem_event.event_data.downdata.fport,
                            modem_event.event_data.downdata.data, modem_event.event_data.downdata.length );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_UPLOAD_DONE:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->upload_done != NULL ) )
                    {
                        lr1110_modem_event_callback->upload_done( modem_event.event_data.upload.status );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_SET_CONF:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->set_conf != NULL ) )
                    {
                        lr1110_modem_event_callback->set_conf( modem_event.event_data.setconf.tag );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_MUTE:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->mute != NULL ) )
                    {
                        lr1110_modem_event_callback->mute( modem_event.event_data.mute.status );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_STREAM_DONE:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->stream_done != NULL ) )
                    {
                        lr1110_modem_event_callback->stream_done( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_WIFI_SCAN_DONE:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->wifi_scan_done != NULL ) )
                    {
                        lr1110_modem_event_callback->wifi_scan_done( modem_event.event_data.wifi.buffer,
                                                                     modem_event.event_data.wifi.len );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_GNSS_SCAN_DONE:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->gnss_scan_done != NULL ) )
                    {
                        lr1110_modem_event_callback->gnss_scan_done( modem_event.event_data.gnss.nav_message,
                                                                     modem_event.event_data.gnss.len );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_TIME_UPDATED_ALC_SYNC:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->time_updated_alc_sync != NULL ) )
                    {
                        lr1110_modem_event_callback->time_updated_alc_sync( modem_event.event_data.time.status );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_ADR_MOBILE_TO_STATIC:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->adr_mobile_to_static != NULL ) )
                    {
                        lr1110_modem_event_callback->adr_mobile_to_static( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_NEW_LINK_ADR:
                    if( ( lr1110_modem_event_callback != NULL ) &&
                        ( lr1110_modem_event_callback->new_link_adr != NULL ) )
                    {
                        lr1110_modem_event_callback->new_link_adr( );
                    }
                    break;
                case LR1110_MODEM_LORAWAN_EVENT_NO_EVENT:
                    if( ( lr1110_modem_event_callback != NULL ) && ( lr1110_modem_event_callback->no_event != NULL ) )
                    {
                        lr1110_modem_event_callback->no_event( );
                    }
                    break;
                default:
                    break;
                }
            }
            else
            {
                HAL_DBG_TRACE_ERROR( "lr1110_modem_helper_get_event_data RC = %d\r\n\r\n", modem_response_code );
            }
        } while( ( lr1110_tracker_board_read_event_line( context ) == 1 ) &&
                 ( modem_response_code == LR1110_MODEM_HELPER_STATUS_OK ) );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void radio_event_callback( void* obj ) {}

/* --- EOF ------------------------------------------------------------------ */
