/*!
 * @file      gnss_scan.c
 *
 * @brief     GNSS scan implementation.
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
 * @brief GNSS scan state machine timeout
 */
#define GNSS_SCAN_TIMEOUT ( 15000 )

/*!
 * @brief GNSS scan timing description
 *
 * Assisted mode
          |                                               |<----- GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL --------->|                                            |
          |<-- ALMANAC_CRC_CHECK -->|<-- TCXCO_STARTUP -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|<-- IDLE -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|
          |<--------------------(lna off)---------------->|<-------(lna on)-------->|<-------------(lna off)------->|<-------(lna on)-------->|<---(lna off)---->|
          |                                               |                         |                               |                         |                  |
  Scan command Received                             Start Scan GPS      End Scan GPS/Start Comput GPS         Start Scan Beidou      End Scan Beidou/Start Comput Beidou
 *
 * Autonomous mode
          |                     |<----- GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL --------->|                                            |
          |<-- TCXCO_STARTUP -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|<-- IDLE -->|<-- RADIO_ACQUISITION -->|<-- PROCESSING -->|
          |<-------(lna off)--->|<-------(lna on)-------->|<-------------(lna off)------->|<-------(lna on)-------->|<---(lna off)---->|
          |                     |                         |                               |                         |                  |
  Scan command Received   Start Scan GPS      End Scan GPS/Start Comput GPS         Start Scan Beidou      End Scan Beidou/Start Comput Beidou
 *
 * For the very first scan after a reset of the modem, an additional "CALIBRATION" step is added before the first "RADIO_ACQUISITION".
 * The LNA must be on during the "CALIBRATION" step, which lasts 5ms plus the TCXO startup time.
 * Note that the "CALIBRATION" steps only occurs on the first scan after reset (even in case of double constellation scan), and not for other scans.
 */

/*!
 * @brief GNSS scan radio acquisition duration for GPS constellation
 */
#define GNSS_SCAN_RADIO_GPS_ACQUISITION ( 260 )

/*!
 * @brief GNSS scan radio acquisition duration for BEIDOU constellation
 */
#define GNSS_SCAN_RADIO_BEIDOU_ACQUISITION ( 255 )

/*!
 * @brief GNSS scan radio check almanac crc in assisted mode
 */
#define GNSS_SCAN_ALMANAC_CRC_CHECK_ASSISTED_MODE ( 6 )

/*!
 * @brief GNSS scan radio calibration, performed only on the first scan after Modem-E reset
 */
#define GNSS_SCAN_RADIO_CALIBRATION ( 7 )

/*!
 * @brief GNSS scan radio acquisition interval in case of double constellation
 */
#define GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL ( 4000 )

/*!
 * @brief Timer application adaptation
 */
#define GNSS_SCAN_LNA_TIMER_APP_ADAPTATION ( 2 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*!
 * @brief GNSS state used in the state machine
 */
typedef enum
{
    GNSS_START_SCAN,
    GNSS_GET_RESULTS,
    GNSS_TERMINATED,
    GNSS_LOW_POWER,
} gnss_state_t;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief GNSS state \ref gnss_state_t
 */
static gnss_state_t gnss_state;

/*!
 * @brief Buffer scan result buffer
 */
static uint8_t gnss_scan_result_buffer[GNSS_BUFFER_MAX_SIZE];

/*!
 * @brief Buffer scan result buffer size
 */
static uint16_t gnss_scan_result_buffer_size;

/*!
 * @brief GNSS scan type parameter
 */
static uint8_t scan_type = ASSISTED_MODE;

/*!
 * @brief GNSS scan timeout flag
 */
static bool gnss_scan_timeout = false;

/*!
 * @brief GNSS first scan done, during the first scan the LR1110 performs a radio calibration and the LNA timing
 * management are not adapted for this step
 */
static bool gnss_first_scan_done = false;

/*!
 * @brief GNSS Almanac update flag, if true an almanac update is ongoing.
 */
static bool full_almanac_udapte_flag;

/*!
 * @brief GNSS Almanac updated flag, if true an almanac is updated.
 */
static bool almanac_udapted;

/*!
 * @brief Timer to handle the scan timeout
 */
static timer_event_t gnss_scan_timeout_timer;

/*!
 * @brief Timer to handle the LNA shutdown
 */
static timer_event_t gnss_lna_shutdown_timer;

/*!
 * @brief Timer to handle the first radio acquisition LNA start
 */
static timer_event_t gnss_first_radio_acquisition_lna_start_timer;

/*!
 * @brief Timer to handle the second radio acquisition LNA start
 */
static timer_event_t gnss_second_radio_acquisition_lna_start_timer;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Analyze the received NAV message and determine if it's a valid one
 *
 * @param [in] settings Gnss settings used \ref gnss_settings_t
 * @param [in] capture_result Structure containing the capture result
 * @param [out] is_valid_nav_message If true, the NAV message is valid if false, it's not valid
 * @param [out] average_cn Average CN of the detected satellites
 */
static void gnss_analyse_nav_message( const gnss_settings_t* settings, const gnss_scan_single_result_t* capture_result,
                                      bool* is_valid_nav_message, uint8_t* average_cn );

/*!
 * @brief configure by default the gnss scanner
 *
 * @param [in] context Radio abstraction
 * @param [in] settings gnss settings to apply \ref gnss_settings_t
 */
static void gnss_scan_configure( const void* context, const gnss_settings_t* settings );

/*!
 * @brief Choose scan mode between assisted and autonomous
 *
 * @param [in] type between ASSISTED_MODE or AUTONOMOUS_MODE
 */
static void gnss_scan_set_type( uint8_t type );

/*!
 * @brief configure by default the gnss scanner
 *
 * @param [in] antenna selection between GNSS_PATCH_ANTENNA or GNSS_PCB_ANTENNA
 */
static void gnss_scan_set_antenna( antenna_t antenna );

/*!
 * @brief init the gnss state machine
 *
 * @param [in] context Radio abstraction
 * @param [in] settings GNSS settings to apply \ref gnss_settings_t
 */
static void gnss_scan_init( const void* context, const gnss_settings_t* settings );

/*!
 * @brief Check if the setting mask indicates a single constellation GNSS scan
 *
 * @param [in] settings GNSS settings used \ref gnss_settings_t
 */
static bool is_single_constellation_setting( const gnss_settings_t* settings );

/* Following function are necessary in the case of the MCU control the LNA */

/*!
 * @brief configure the timers managing the LNA supply
 *
 * @param [in] settings gnss settings to apply \ref gnss_settings_t
 */
static void gnss_scan_lna_management_timers_configure( const gnss_settings_t* settings, bool is_gnss_first_scan_done );

/*!
 * @brief start the timers managing the LNA supply
 *
 * @param [in] settings gnss settings to apply \ref gnss_settings_t
 */
static void gnss_scan_lna_management_timers_start( const gnss_settings_t* settings );

/*!
 * @brief Function executed on gnss scan timeout event
 *
 * @param [in] context Radio abstraction
 */
static void on_gnss_scan_timeout_event( void* context );

/*!
 * @brief Function executed on LNA shutdown event
 *
 * @param [in] context Radio abstraction
 */
static void on_gnss_lna_shutdown_event( void* context );

/*!
 * @brief Function executed on first radio acquisition LNA start event
 *
 * @param [in] context Radio abstraction
 */
static void on_gnss_lna_first_radio_acquisition_start_event( void* context );

/*!
 * @brief Function executed on second radio acquisition LNA start event
 *
 * @param [in] context Radio abstraction
 */
static void on_gnss_lna_second_radio_acquisition_start_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_gnss_scan_done( uint8_t* buffer, uint16_t size )
{
    if( buffer[0] == LR1110_MODEM_GNSS_DESTINATION_HOST )
    {
        switch( buffer[1] )
        {
        case LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_UPDATE_FAILS_CRC_ERROR:
        case LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_UPDATE_FAILS_FLASH_INTEGRITY_ERROR:
        case LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_VERSION_NOT_SUPPORTED:
        {
            full_almanac_udapte_flag = false;
            almanac_udapted          = false;
            break;
        }
        case LR1110_MODEM_GNSS_SCAN_DONE_PROCESS_OK:
        {
            full_almanac_udapte_flag = false;
            almanac_udapted          = true;
        }
        case LR1110_MODEM_GNSS_SCAN_DONE_IQ_FAILS:
        case LR1110_MODEM_GNSS_SCAN_DONE_NO_TIME:
        case LR1110_MODEM_GNSS_SCAN_DONE_NO_SATELLITE_DETECTED:
        case LR1110_MODEM_GNSS_SCAN_DONE_GLOBAL_ALMANAC_CRC_ERROR:
        case LR1110_MODEM_GNSS_SCAN_DONE_ALMANAC_TOO_OLD:
        default:
        {
            memcpy( gnss_scan_result_buffer, buffer, size );
            gnss_scan_result_buffer_size = size;
            gnss_state                   = GNSS_GET_RESULTS;
        }
        break;
        }
    }
    else if( buffer[0] == LR1110_MODEM_GNSS_DESTINATION_SOLVER )
    {
        memcpy( gnss_scan_result_buffer, buffer, size );
        gnss_scan_result_buffer_size = size;
        gnss_state                   = GNSS_GET_RESULTS;
    }
}

void gnss_set_full_almanac_udapte_flag( bool enable ) { full_almanac_udapte_flag = enable; }

bool gnss_get_full_almanac_udapte_flag( void ) { return full_almanac_udapte_flag; }

bool gnss_get_full_almanac_udapted( void ) { return almanac_udapted; }

gnss_scan_result_t gnss_scan_execute( const void* context, antenna_t antenna, const gnss_settings_t* settings,
                                      gnss_scan_single_result_t* capture_result )
{
    bool                         gnss_scan_done      = false;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    gnss_scan_result_t           scan_result         = GNSS_SCAN_SUCCESS;

    /* Reset parameters */
    gnss_scan_timeout                    = false;
    capture_result->is_valid_nav_message = false;

    /* Select the antenna */
    gnss_scan_set_antenna( antenna );
    capture_result->antenna = antenna;

    /* Init the GNSS parameters */
    gnss_scan_init( context, settings );

    /* Start the timeout timer */
    timer_start( &gnss_scan_timeout_timer );

    while( ( gnss_scan_done != true ) && ( gnss_scan_timeout != true ) )
    {
        /* Process Event */
        if( ( ( lr1110_t* ) context )->event.callback != NULL )
        {
            lr1110_modem_event_process( context );
        }

        switch( gnss_state )
        {
        case GNSS_START_SCAN:

            if( scan_type == AUTONOMOUS_MODE )
            {
                modem_response_code = lr1110_modem_gnss_scan_autonomous(
                    context, settings->search_mode, LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK, settings->nb_sat );
            }
            else
            {
                modem_response_code = lr1110_modem_gnss_scan_assisted(
                    context, settings->search_mode, LR1110_MODEM_GNSS_PSEUDO_RANGE_MASK, settings->nb_sat );
            }

            /* If response code different than RESPONSE_CODE_OK leave */
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
            else
            {
                /* Start the timers managing the LNA supply */
                gnss_scan_lna_management_timers_start( settings );

                gnss_state = GNSS_LOW_POWER;
            }

            break;

        case GNSS_GET_RESULTS:

            /* Store the NAV message */
            memcpy( capture_result->nav_message, gnss_scan_result_buffer, gnss_scan_result_buffer_size );
            capture_result->nav_message_size = gnss_scan_result_buffer_size;

            modem_response_code =
                lr1110_modem_gnss_get_nb_detected_satellites( context, &capture_result->nb_detected_satellites );
            modem_response_code = lr1110_modem_gnss_get_detected_satellites(
                context, capture_result->nb_detected_satellites, capture_result->detected_satellites );
            lr1110_modem_gnss_get_timings( context, &capture_result->timings );

            /* Analyze the received NAV message */
            gnss_analyse_nav_message( settings, capture_result, &capture_result->is_valid_nav_message,
                                      &capture_result->average_cn );

            /* update the gnss_first_scan_done flag */
            gnss_first_scan_done = true;

            gnss_state = GNSS_TERMINATED;

            break;

        case GNSS_TERMINATED:
            gnss_state     = GNSS_START_SCAN;
            gnss_scan_done = true;
            break;

        case GNSS_LOW_POWER:
            /* The MCU wakes up through events */
            hal_mcu_low_power_handler( );
            break;
        }
    }

    timer_stop( &gnss_scan_timeout_timer );

    if( gnss_scan_timeout == true )
    {
        scan_result = GNSS_SCAN_FAIL;
    }

    return scan_result;
}

void gnss_scan_display_results( const gnss_scan_single_result_t* capture_result )
{
    uint8_t i = 0;

    /* Antenna */
    if( capture_result->antenna == GNSS_PATCH_ANTENNA )
    {
        HAL_DBG_TRACE_MSG( "CAPTURE ON PATCH ANTENNA\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "CAPTURE ON PCB ANTENNA\r\n" );
    }

    /* Satellites infos */

    HAL_DBG_TRACE_PRINTF( "Nb Detected satellites : %d \r\n", capture_result->nb_detected_satellites );

    if( capture_result->nb_detected_satellites > 0 )
    {
        HAL_DBG_TRACE_MSG( "Satellites infos : \r\n" );

        for( i = 0; i < capture_result->nb_detected_satellites; i++ )
        {
            HAL_DBG_TRACE_PRINTF( "ID = %d -- CN = %d \r\n", capture_result->detected_satellites[i].satellite_id,
                                  capture_result->detected_satellites[i].cnr );
        }
    }

    /* Scan Timings */
    HAL_DBG_TRACE_PRINTF( "Scan timing radio_ms : %d\r\n", capture_result->timings.radio_ms );
    HAL_DBG_TRACE_PRINTF( "Scan timing computation_ms : %d\r\n", capture_result->timings.computation_ms );

    /* NAV Message */

    HAL_DBG_TRACE_MSG( "NAV = " );

    for( i = 0; i < capture_result->nav_message_size; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%02X", capture_result->nav_message[i] );
    }

    HAL_DBG_TRACE_PRINTF( "\r\nIs NAV message valid : %d\r\n", capture_result->is_valid_nav_message );

    if( capture_result->is_valid_nav_message == true )
    {
        /* Average CN */
        HAL_DBG_TRACE_PRINTF( "Average CN : %d\r\n", capture_result->average_cn );
    }

    HAL_DBG_TRACE_MSG( "\r\n" );
}

void gnss_scan_determine_best_nav_message( gnss_scan_single_result_t* pcb_capture_result,
                                           gnss_scan_single_result_t* patch_capture_result )
{
    if( ( pcb_capture_result->is_valid_nav_message == true ) && ( patch_capture_result->is_valid_nav_message == true ) )
    {
        float pcb_nb_sv   = pcb_capture_result->nb_detected_satellites;
        float patch_nb_sv = patch_capture_result->nb_detected_satellites;

        if( fabs( pcb_nb_sv - patch_nb_sv ) > 1 )
        {
            if( pcb_capture_result->nb_detected_satellites > patch_capture_result->nb_detected_satellites )
            {
                patch_capture_result->is_valid_nav_message = false;
            }
            else
            {
                pcb_capture_result->is_valid_nav_message = false;
            }
        }
        else
        {
            if( pcb_capture_result->average_cn > patch_capture_result->average_cn )
            {
                patch_capture_result->is_valid_nav_message = false;
            }
            else
            {
                pcb_capture_result->is_valid_nav_message = false;
            }
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_gnss_scan_timeout_event( void* context ) { gnss_scan_timeout = true; }

static void gnss_scan_configure( const void* context, const gnss_settings_t* settings )
{
    lr1110_modem_gnss_set_constellations_to_use( context, settings->constellation_to_use );
    gnss_scan_set_type( settings->scan_type );
}

static bool is_single_constellation_setting( const gnss_settings_t* settings )
{
    return settings->constellation_to_use != ( LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK );
}

static void gnss_analyse_nav_message( const gnss_settings_t* settings, const gnss_scan_single_result_t* capture_result,
                                      bool* is_valid_nav_message, uint8_t* average_cn )
{
    uint16_t average_cn_tmp = 0;

    /* Analyse the NAV message:
    Check the validity which is defined by having :
    at least 2 sv per constellation (BEIDOU and GNSS only) and 6 detected satellites in the case of double
    constellation. if there are 5 sv in a same constellation in case of double constellation the NAV message is valid
    5 detected satellites (BEIDOU and GNSS only) in the case of single constellation.
    GPS satellites ID [0 31], SBAS satellites ID [32 63] but not used, BEIDOU satellites ID [64 128].
    Calcul the average CN.
    */
    if( capture_result->nb_detected_satellites >= 5 )
    {
        uint8_t gps_sv_cnt    = 0;
        uint8_t beidou_sv_cnt = 0;

        for( uint8_t i = 0; i < capture_result->nb_detected_satellites; i++ )
        {
            average_cn_tmp += capture_result->detected_satellites[i].cnr;

            /* Remove the SBAS from the count */
            /* Check if it's a GPS satellite */
            if( capture_result->detected_satellites[i].satellite_id <= 31 )
            {
                gps_sv_cnt++;
            }
            /* Check if it's a BEIDOU satellite */
            if( ( capture_result->detected_satellites[i].satellite_id >= 64 ) &&
                ( capture_result->detected_satellites[i].satellite_id <= 128 ) )
            {
                beidou_sv_cnt++;
            }
        }

        /* Calcul the average CN */
        *average_cn = average_cn_tmp / capture_result->nb_detected_satellites;

        /* Check if the NAV message is valid */
        if( ( is_single_constellation_setting( settings ) == true ) )
        {
            if( ( gps_sv_cnt >= 5 ) || ( beidou_sv_cnt >= 5 ) )
            {
                *is_valid_nav_message = true;
            }
            else
            {
                *is_valid_nav_message = false;
            }
        }
        else
        {
            if( ( ( gps_sv_cnt >= 2 ) && ( beidou_sv_cnt >= 2 ) && ( ( gps_sv_cnt + beidou_sv_cnt ) >= 6 ) ) ||
                ( ( ( gps_sv_cnt >= 5 ) || ( beidou_sv_cnt >= 5 ) ) && ( ( gps_sv_cnt + beidou_sv_cnt ) >= 5 ) ) )
            {
                *is_valid_nav_message = true;
            }
            else
            {
                *is_valid_nav_message = false;
            }
        }
    }
    else
    {
        *is_valid_nav_message = false;
    }
}

static void gnss_scan_lna_management_timers_configure( const gnss_settings_t* settings, bool is_gnss_first_scan_done )
{
    uint16_t gnss_lna_shutdown_timer_value                       = 0;
    uint16_t gnss_first_radio_acquisition_lna_start_timer_value  = 0;
    uint16_t gnss_second_radio_acquisition_lna_start_timer_value = 0;

    /* Init the timers managing the LNA supply */
    timer_init( &gnss_lna_shutdown_timer, on_gnss_lna_shutdown_event );
    timer_init( &gnss_first_radio_acquisition_lna_start_timer, on_gnss_lna_first_radio_acquisition_start_event );
    timer_init( &gnss_second_radio_acquisition_lna_start_timer, on_gnss_lna_second_radio_acquisition_start_event );

    /* Set the timer timings according to the GNSS parameter */
    if( settings->scan_type == ASSISTED_MODE )
    {
        if( ( settings->constellation_to_use == ( LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK ) ) ||
            settings->constellation_to_use == ( LR1110_MODEM_GNSS_GPS_MASK ) )
        {
            gnss_lna_shutdown_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + GNSS_SCAN_RADIO_GPS_ACQUISITION +
                                            lr1110_tracker_board_get_tcxo_wakeup_time( ) +
                                            GNSS_SCAN_ALMANAC_CRC_CHECK_ASSISTED_MODE;
        }
        else
        {
            gnss_lna_shutdown_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + GNSS_SCAN_RADIO_BEIDOU_ACQUISITION +
                                            lr1110_tracker_board_get_tcxo_wakeup_time( ) +
                                            GNSS_SCAN_ALMANAC_CRC_CHECK_ASSISTED_MODE;
        }
        gnss_first_radio_acquisition_lna_start_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION +
                                                             lr1110_tracker_board_get_tcxo_wakeup_time( ) +
                                                             GNSS_SCAN_ALMANAC_CRC_CHECK_ASSISTED_MODE;
        gnss_second_radio_acquisition_lna_start_timer_value =
            GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL +
            lr1110_tracker_board_get_tcxo_wakeup_time( ) + GNSS_SCAN_ALMANAC_CRC_CHECK_ASSISTED_MODE;
    }
    else
    {
        if( ( settings->constellation_to_use == ( LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK ) ) ||
            settings->constellation_to_use == ( LR1110_MODEM_GNSS_GPS_MASK ) )
        {
            gnss_lna_shutdown_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + GNSS_SCAN_RADIO_GPS_ACQUISITION +
                                            lr1110_tracker_board_get_tcxo_wakeup_time( );
        }
        else
        {
            gnss_lna_shutdown_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + GNSS_SCAN_RADIO_BEIDOU_ACQUISITION +
                                            lr1110_tracker_board_get_tcxo_wakeup_time( );
        }
        gnss_first_radio_acquisition_lna_start_timer_value =
            GNSS_SCAN_LNA_TIMER_APP_ADAPTATION + lr1110_tracker_board_get_tcxo_wakeup_time( );
        gnss_second_radio_acquisition_lna_start_timer_value = GNSS_SCAN_LNA_TIMER_APP_ADAPTATION +
                                                              GNSS_SCAN_DOUBLE_CONSTELLATION_INTERVAL +
                                                              lr1110_tracker_board_get_tcxo_wakeup_time( );
    }

    if( is_gnss_first_scan_done == false )
    {
        gnss_lna_shutdown_timer_value += GNSS_SCAN_RADIO_CALIBRATION + lr1110_tracker_board_get_tcxo_wakeup_time( );
        gnss_second_radio_acquisition_lna_start_timer_value +=
            GNSS_SCAN_RADIO_CALIBRATION + lr1110_tracker_board_get_tcxo_wakeup_time( );
    }

    timer_set_value( &gnss_lna_shutdown_timer, gnss_lna_shutdown_timer_value );
    timer_set_value( &gnss_first_radio_acquisition_lna_start_timer,
                     gnss_first_radio_acquisition_lna_start_timer_value );
    timer_set_value( &gnss_second_radio_acquisition_lna_start_timer,
                     gnss_second_radio_acquisition_lna_start_timer_value );
}

static void gnss_scan_lna_management_timers_start( const gnss_settings_t* settings )
{
    /* start the first timer to start the LNA on the first radio acquisition */
    timer_start( &gnss_first_radio_acquisition_lna_start_timer );
    /* Start the LNA shut down timer */
    timer_start( &gnss_lna_shutdown_timer );

    if( settings->constellation_to_use == ( LR1110_MODEM_GNSS_GPS_MASK | LR1110_MODEM_GNSS_BEIDOU_MASK ) )
    {
        /* start the second timer in case of dual constellation scan to restart the LNA */
        timer_start( &gnss_second_radio_acquisition_lna_start_timer );
    }
}

static void on_gnss_lna_shutdown_event( void* context )
{
    /* Switch off the LNA */
    lr1110_tracker_board_lna_off( );
}

static void on_gnss_lna_first_radio_acquisition_start_event( void* context )
{
    /* Turn on the LNA */
    lr1110_tracker_board_lna_on( );
}

static void on_gnss_lna_second_radio_acquisition_start_event( void* context )
{
    /* Turn on the LNA */
    lr1110_tracker_board_lna_on( );

    /* Adapt the timer for the second scan */
    timer_set_value( &gnss_lna_shutdown_timer, GNSS_SCAN_RADIO_BEIDOU_ACQUISITION );

    /* Start the LNA shut down timer */
    timer_start( &gnss_lna_shutdown_timer );
}

static void gnss_scan_set_type( uint8_t type )
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

static void gnss_scan_set_antenna( antenna_t antenna )
{
    if( antenna == GNSS_PATCH_ANTENNA )
    {
        set_gnss_patch_antenna( );
    }
    else
    {
        set_gnss_pcb_antenna( );
    }
}

static void gnss_scan_init( const void* context, const gnss_settings_t* settings )
{
    gnss_state                   = GNSS_START_SCAN;
    gnss_scan_result_buffer_size = 0;

    timer_init( &gnss_scan_timeout_timer, on_gnss_scan_timeout_event );
    timer_set_value( &gnss_scan_timeout_timer, GNSS_SCAN_TIMEOUT );

    gnss_scan_lna_management_timers_configure( settings, gnss_first_scan_done );

    gnss_scan_configure( context, settings );
}

/* --- EOF ------------------------------------------------------------------ */
