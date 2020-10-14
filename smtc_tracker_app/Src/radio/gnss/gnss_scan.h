/*!
 * \file      gnss_scan.h
 *
 * \brief     GNSS scan definition
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

#ifndef __GNSS_SCAN_H__
#define __GNSS_SCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>
#include "lr1110_modem_gnss.h"
#include "lr1110_modem_system.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define GNSS_BUFFER_MAX_SIZE 255

#define GNSS_LEAP_SECONDS_OFFSET 18
#define GNSS_EPOCH_SECONDS 315964800  // 6/01/1980
#define SECS_PER_WEEK 604800          //(60L*60*24*7)

#define ASSISTED_MODE 1
#define AUTONOMOUS_MODE 2

#define GNSS_SCAN_NO_TIME 2
#define GNSS_SCAN_SUCCESS 1
#define GNSS_SCAN_FAIL 0

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \brief GNSS scan result type
 */
typedef uint8_t gnss_scan_result_t;

/*!
 * \brief GNSS state used in the state machine
 */
typedef enum
{
    GNSS_START_SCAN,
    GNSS_GET_RESULTS,
    GNSS_TERMINATED,
    GNSS_LOW_POWER,
} gnss_state_t;

/*!
 * \brief GNSS Antenna type
 */
typedef enum
{
    GNSS_PATCH_ANTENNA = 1,
    GNSS_PCB_ANTENNA,
} antenna_t;

/*!
 * \brief GNSS general settings
 */
typedef struct
{
    bool                                           enabled;
    uint8_t                                        scan_type;
    uint8_t                                        inter_capture_delay_second;
    lr1110_modem_gnss_search_mode_t                search_mode;
    uint8_t                                        input_paramaters;
    uint8_t                                        constellation_to_use;
    lr1110_modem_gnss_solver_assistance_position_t assistance_position;
    uint8_t                                        nb_sat;
} gnss_settings_t;

/*!
 * \brief GNSS scan results
 */
typedef struct
{
    bool                                   double_scan_first_scan_done;
    uint16_t                               result_size;
    antenna_t                              antenna;
    uint8_t                                result_buffer[GNSS_BUFFER_MAX_SIZE];
    uint8_t                                nb_detected_satellites;
    lr1110_modem_gnss_detected_satellite_t detected_satellites[32];
} gnss_scan_single_result_t;

/*!
 * \brief GNSS scan general handler
 */
typedef struct
{
    gnss_scan_single_result_t capture_result;
    gnss_settings_t           settings;
    gnss_state_t              state;
} gnss_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Function executed on GNSS Scan done event
 *
 * \param [in] buffer Buffer containing the NAV Message
 *
 * \param [in] size Size of the NAV message
 */
void lr1110_modem_gnss_scan_done( uint8_t* buffer, uint16_t size );

/*!
 * \brief Display the last scan results
 */
void gnss_scan_display_results( void );

/*!
 * \brief start the gnss capture state machine
 *
 * \param [in] context Radio abstraction
 */
gnss_scan_result_t gnss_scan_execute( const void* context );

/*!
 * \brief init the gnss state machine
 *
 * \param [in] context Radio abstraction
 *
 * \param [in] settings GNSS settings to apply \ref gnss_settings_t
 */
void gnss_scan_init( const void* context, gnss_settings_t settings );

/*!
 * \brief Choose scan mode between assisted and autonomous
 *
 * \param [in] type between
    ASSISTED_MODE or AUTONOMOUS_MODE
 */
void gnss_scan_set_type( uint8_t type );

/*!
 * \brief configure by default the gnss scanner
 *
 * \param [in] antenna selection between
 * GNSS_PATCH_ANTENNA or GNSS_PCB_ANTENNA
 */
void gnss_scan_set_antenna( antenna_t antenna );

#ifdef __cplusplus
}
#endif

#endif  // __GNSS_SCAN_H__

/* --- EOF ------------------------------------------------------------------ */
