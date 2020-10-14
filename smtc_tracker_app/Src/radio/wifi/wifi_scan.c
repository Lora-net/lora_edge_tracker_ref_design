/*!
 * \file      wifi_scan.c
 *
 * \brief     Wifi scan implementation.
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

#define WIFI_SCAN_ABORT_ON_TIMEOUT ( true )
#define WIFI_CONSUMPTION_DCDC_CORRELATION_MA ( 12 )
#define WIFI_CONSUMPTION_DCDC_DEMODULATION_MA ( 4 )
#define WIFI_CONSUMPTION_LDO_CORRELATION_MA ( 24 )
#define WIFI_CONSUMPTION_LDO_DEMODULATION_MA ( 8 )
#define WIFI_MAX_BASIC_RESULTS_PER_SCAN ( 32 )
#define WIFI_MAX_COMPLETE_RESULTS_PER_SCAN ( 12 )

/*!
 * \brief Wi-Fi scan state machine timeout
 */
#define WIFI_SCAN_TIMEOUT ( 10000 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief Wi-Fi global parameters
 */
wifi_t wifi;

/*!
 * \brief Wi-Fi scan timeout flag
 */
static bool wifi_scan_timeout = false;

/*!
 * \brief Timer to handle the scan timeout
 */
static timer_event_t wifi_scan_timeout_timer;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief configure the Wi-Fi scanner parameter
 *
 * \param [in] wifi_settings structure containing the Wi-Fi parameter \see wifi_settings_t
 */
void wifi_configure( wifi_settings_t wifi_settings );

/*!
 * \brief Compute consumption based on cumulative timings
 *
 * \param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 *
 * \param [in] timing \ref lr1110_modem_wifi_cumulative_timings_t
 *
 * \return Consumption in micro ampere second (uas)
 */
uint32_t wifi_compute_consumption( const lr1110_modem_system_reg_mode_t         reg_mode,
                                   const lr1110_modem_wifi_cumulative_timings_t timing );

/*!
 * \brief copy complete mac results into wi-fi result structure
 *
 * \param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 *
 * \param [out] results structure containing the results \ref wifi_scan_all_result_t
 *
 * \param [in] scan_result to copy \ref lr1110_modem_wifi_basic_complete_result_t
 *
 * \param [in] nbr_results nb result to copy
 *
 * \param [in] timing scan timing
 */
void wifi_add_complete_mac_to_results( lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                       lr1110_modem_wifi_basic_complete_result_t* scan_result, uint8_t nbr_results,
                                       lr1110_modem_wifi_cumulative_timings_t timing );

/*!
 * \brief copy basic mac results into wi-fi result structure
 *
 * \param [in] reg_mode \ref lr1110_modem_system_reg_mode_t
 *
 * \param [out] results structure containing the results \ref wifi_scan_all_result_t
 *
 * \param [in] scan_result to copy \ref lr1110_modem_wifi_basic_mac_type_channel_result_t
 *
 * \param [in] nbr_results nb result to copy
 *
 * \param [in] timing scan timing
 */
void wifi_add_basic_mac_to_results( lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                    lr1110_modem_wifi_basic_mac_type_channel_result_t* scan_result, uint8_t nbr_results,
                                    lr1110_modem_wifi_cumulative_timings_t timing );

/*!
 * \brief Function executed on Wi-Fi scan timeout event
 */
static void on_wifi_scan_timeout_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_wifi_scan_done( uint8_t* buffer, uint16_t size )
{
    memcpy( wifi.results.raw_buffer, buffer, size );
    wifi.results.raw_buffer_size = size;

    wifi.state = WIFI_GET_RESULTS;
}

void wifi_init( const void* context, wifi_settings_t wifi_settings )
{
    wifi.state                          = WIFI_INIT;
    wifi.results.nbr_results            = 0;
    wifi.results.global_consumption_uas = 0;

    timer_init( &wifi_scan_timeout_timer, on_wifi_scan_timeout_event );
    timer_set_value( &wifi_scan_timeout_timer, WIFI_SCAN_TIMEOUT );

    wifi_configure( wifi_settings );
}

wifi_scan_result_t wifi_execute_scan( const void* context )
{
    bool                         wifi_scan_done = false;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    wifi_scan_result_t           scan_result = WIFI_SCAN_SUCCESS;
    
    wifi_scan_timeout = false;

    timer_start( &wifi_scan_timeout_timer );

    while( ( wifi_scan_done != true ) && ( wifi_scan_timeout != true ) )
    {
        // Process Event
        if( ( ( lr1110_t* ) context )->event.callback != NULL )
        {
            lr1110_modem_event_process( context );
        }

        switch( wifi.state )
        {
            case WIFI_INIT:
            {
                wifi.state          = WIFI_SCAN;
                modem_response_code = lr1110_modem_wifi_reset_cumulative_timing( context );
                break;
            }

            case WIFI_SCAN:
            {
                modem_response_code = lr1110_modem_wifi_cfg_hardware_debarker( context, true );
                modem_response_code = lr1110_modem_wifi_passive_scan(
                    context, wifi.settings.types, wifi.settings.channels, wifi.settings.scan_mode,
                    wifi.settings.max_results, wifi.settings.nbr_retrials, wifi.settings.timeout,
                    WIFI_SCAN_ABORT_ON_TIMEOUT, wifi.settings.result_format );

                // If response code different than RESPONSE_CODE_OK leave
                if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
                {
                    wifi_scan_done = true;
                    scan_result    = WIFI_SCAN_FAIL;
                }

                wifi.state = WIFI_WAIT_FOR_SCAN;
                break;
            }

            case WIFI_GET_RESULTS:
            {
                lr1110_modem_wifi_cumulative_timings_t wifi_results_timings = { 0 };
                uint8_t                                nb_result;

                lr1110_modem_wifi_read_cumulative_timing( context, &wifi_results_timings );

                if( wifi.settings.result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL )
                {
                    lr1110_modem_wifi_basic_mac_type_channel_result_t
                        wifi_results_mac_addr[WIFI_MAX_BASIC_RESULTS_PER_SCAN] = { 0 };
                    lr1110_modem_wifi_read_basic_results( wifi.results.raw_buffer, wifi.results.raw_buffer_size,
                                                          wifi_results_mac_addr, &nb_result );
                    wifi_add_basic_mac_to_results( LR1110_MODEM_SYSTEM_REG_MODE_DCDC, &wifi.results, wifi_results_mac_addr,
                                                   nb_result, wifi_results_timings );
                }
                else if( wifi.settings.result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE )
                {
                    lr1110_modem_wifi_basic_complete_result_t wifi_results_mac_addr[WIFI_MAX_COMPLETE_RESULTS_PER_SCAN] = {
                        0
                    };
                    lr1110_modem_wifi_read_complete_results( wifi.results.raw_buffer, wifi.results.raw_buffer_size,
                                                             wifi_results_mac_addr, &nb_result );
                    wifi_add_complete_mac_to_results( LR1110_MODEM_SYSTEM_REG_MODE_DCDC, &wifi.results,
                                                      wifi_results_mac_addr, nb_result, wifi_results_timings );
                }

                wifi_scan_done = true;
                wifi.state = WIFI_INIT;

                break;
            }

            case WIFI_WAIT_FOR_SCAN:
            {
                // The MCU wakes up through events
                hal_mcu_low_power_handler( );
                break;
            }
        }
    }

    timer_stop( &wifi_scan_timeout_timer );

    if( wifi_scan_timeout == true )
    {
        scan_result = WIFI_SCAN_FAIL;
    }

    return scan_result;
}

void lr1110_display_wifi_scan_results( void )
{
    if( wifi.results.nbr_results != 0 )
    {
        HAL_DBG_TRACE_PRINTF( "nb MAC scanned : %d \r\n", wifi.results.nbr_results );
        for( uint8_t i = 0; i < wifi.results.nbr_results; i++ )
        {
            HAL_DBG_TRACE_MSG( "MAC addr : " );
            for( uint8_t j = 0; j < 6; j++ )
            {
                HAL_DBG_TRACE_PRINTF( "%#02X ", wifi.results.results[i].mac_address[j] );
            }
            HAL_DBG_TRACE_PRINTF( " -- Channel : %d ", wifi.results.results[i].channel );
            HAL_DBG_TRACE_PRINTF( " -- Type : %d ", wifi.results.results[i].type );
            HAL_DBG_TRACE_PRINTF( " -- RSSI : %d \r\n", wifi.results.results[i].rssi );
        }
        HAL_DBG_TRACE_MSG( "\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "No MAC address found \r\n\r\n" );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_wifi_scan_timeout_event( void* context ) { wifi_scan_timeout = true; }

void wifi_configure( wifi_settings_t wifi_settings )
{
    wifi.settings.enabled       = wifi_settings.enabled;
    wifi.settings.channels      = wifi_settings.channels;
    wifi.settings.types         = wifi_settings.types;
    wifi.settings.scan_mode     = wifi_settings.scan_mode;
    wifi.settings.nbr_retrials  = wifi_settings.nbr_retrials;
    wifi.settings.result_format = wifi_settings.result_format;
    wifi.settings.timeout       = wifi_settings.timeout;

    // if format is LR1110_WIFI_RESULT_FORMAT_BASIC_COMPLETE max result available is 12 otherwise it's 32.
    if( ( wifi.settings.result_format == LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE ) &&
        ( wifi_settings.max_results > 12 ) )
    {
        wifi.settings.max_results = 12;
    }
    else
    {
        wifi.settings.max_results = wifi_settings.max_results;
    }
}

uint32_t wifi_compute_consumption( const lr1110_modem_system_reg_mode_t         reg_mode,
                                   const lr1110_modem_wifi_cumulative_timings_t timing )
{
    uint32_t consumption_uas = 0;

    switch( reg_mode )
    {
    case LR1110_MODEM_SYSTEM_REG_MODE_DCDC:
        consumption_uas = ( ( timing.rx_correlation_us + timing.rx_capture_us ) * WIFI_CONSUMPTION_LDO_CORRELATION_MA +
                            timing.demodulation_us * WIFI_CONSUMPTION_LDO_DEMODULATION_MA ) /
                          1000;
        break;
    case LR1110_MODEM_SYSTEM_REG_MODE_LDO:
        consumption_uas = ( ( timing.rx_correlation_us + timing.rx_capture_us ) * WIFI_CONSUMPTION_DCDC_CORRELATION_MA +
                            timing.demodulation_us * WIFI_CONSUMPTION_DCDC_DEMODULATION_MA ) /
                          1000;
        break;
    }
    return consumption_uas;
}

void wifi_add_basic_mac_to_results( lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                    lr1110_modem_wifi_basic_mac_type_channel_result_t* scan_result, uint8_t nbr_results,
                                    lr1110_modem_wifi_cumulative_timings_t timing )
{
    for( uint8_t index = 0; index < nbr_results; index++ )
    {
        results->results[index].channel =
            lr1110_modem_extract_channel_from_info_byte( scan_result[index].channel_info_byte );
        results->results[index].type =
            lr1110_modem_extract_signal_type_from_data_rate_info( scan_result[index].data_rate_info_byte );
        memcpy( results->results[index].mac_address, scan_result[index].mac_address,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH );
        results->results[index].rssi = scan_result[index].rssi;
    }

    results->timings = timing;
    results->global_consumption_uas += wifi_compute_consumption( reg_mode, timing );
    results->nbr_results = nbr_results;
}

void wifi_add_complete_mac_to_results( lr1110_modem_system_reg_mode_t reg_mode, wifi_scan_all_result_t* results,
                                       lr1110_modem_wifi_basic_complete_result_t* scan_result, uint8_t nbr_results,
                                       lr1110_modem_wifi_cumulative_timings_t timing )
{
    for( uint8_t index = 0; index < nbr_results; index++ )
    {
        results->results[index].channel =
            lr1110_modem_extract_channel_from_info_byte( scan_result[index].channel_info_byte );
        results->results[index].type =
            lr1110_modem_extract_signal_type_from_data_rate_info( scan_result[index].data_rate_info_byte );
        memcpy( results->results[index].mac_address, scan_result[index].mac_address,
                LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH );
        results->results[index].rssi         = scan_result[index].rssi;
        results->results[index].phi_offset   = scan_result[index].phi_offset;
        results->results[index].timestamp_us = scan_result[index].timestamp_us;
    }

    results->timings = timing;
    results->global_consumption_uas += wifi_compute_consumption( reg_mode, timing );
    results->nbr_results = nbr_results;
}

/* --- EOF ------------------------------------------------------------------ */
