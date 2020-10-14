/*!
 * \file      wifi_scan.h
 *
 * \brief     Wi-Fi scan definition
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

#ifndef __WIFI_SCAN_H__
#define __WIFI_SCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1110_modem_wifi.h"
#include "lr1110_modem_system.h"
#include <stdint.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

#define WIFI_NBR_RETRIALS_DEFAULT 5
#define WIFI_MAX_RESULTS_DEFAULT 10
#define WIFI_TIMEOUT_IN_MS_DEFAULT 110
#define WIFI_MAX_RESULT_TOTAL 32

#define WIFI_SCAN_SUCCESS 1
#define WIFI_SCAN_FAIL 0

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \brief Wi-Fi scan result type
 */
typedef uint8_t wifi_scan_result_t;

/*!
 * \brief Wi-Fi state used in the state machine
 */
typedef enum
{
    WIFI_INIT,
    WIFI_SCAN,
    WIFI_WAIT_FOR_SCAN,
    WIFI_GET_RESULTS,
} wifi_state_t;

/*!
 * \brief Wi-Fi single scan result structure
 */
typedef struct
{
    lr1110_modem_wifi_mac_address_t        mac_address;
    lr1110_modem_wifi_channel_t            channel;
    lr1110_modem_wifi_signal_type_result_t type;
    int8_t                                 rssi;
    int16_t                                phi_offset;
    uint64_t                               timestamp_us;
    uint16_t                               beacon_period_tu;
    uint8_t                                country_code[LR1110_MODEM_WIFI_STR_COUNTRY_CODE_SIZE];
} wifi_scan_single_result_t;

/*!
 * \brief Wi-Fi single all result structure
 */
typedef struct
{
    uint8_t                                nbr_results;
    wifi_scan_single_result_t              results[WIFI_MAX_RESULT_TOTAL];
    lr1110_modem_wifi_cumulative_timings_t timings;
    uint32_t                               global_consumption_uas;
    uint8_t                                raw_buffer[288];
    uint16_t                               raw_buffer_size;
    bool                                   error;
} wifi_scan_all_result_t;

/*!
 * \brief Wi-Fi settings stucture parameters
 */
typedef struct
{
    bool                                 enabled;
    lr1110_modem_wifi_channel_mask_t     channels;
    lr1110_modem_wifi_signal_type_scan_t types;
    lr1110_modem_wifi_mode_t             scan_mode;
    uint8_t                              nbr_retrials;
    uint8_t                              max_results;
    uint32_t                             timeout;
    lr1110_modem_wifi_result_format_t    result_format;
} wifi_settings_t;

/*!
 * \brief Wi-Fi global stucture parameters
 */
typedef struct
{
    lr1110_modem_system_reg_mode_t reg_mode;
    wifi_scan_all_result_t         results;
    wifi_settings_t                settings;
    wifi_state_t                   state;
} wifi_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Function executed on Wifi Scan done event
 *
 * \param [in] buffer Buffer containing the raw data
 *
 * \param [in] size Size of the raw data buffer
 */
void lr1110_modem_wifi_scan_done( uint8_t* buffer, uint16_t size );

/*!
 * \brief Display the last Wi-Fi scan results
 */
void lr1110_display_wifi_scan_results( void );

/*!
 * \brief execute the wifi scan state machine
 *
 * \param [in] context Radio abstraction
 */
wifi_scan_result_t wifi_execute_scan( const void* context );

/*!
 * \brief init the wifi scan state machine
 *
 * \param [in] context Radio abstraction
 *
 * \param [in] wifi_settings structure containing the Wi-Fi parameter \see wifi_settings_t
 */
void wifi_init( const void* context, wifi_settings_t wifi_settings );

#ifdef __cplusplus
}
#endif

#endif  // __WIFI_SCAN_H__

/* --- EOF ------------------------------------------------------------------ */
