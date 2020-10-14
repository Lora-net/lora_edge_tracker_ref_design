/*!
 * \file      lr1110_tracker_board.h
 *
 * \brief     Target board LR1110 driver definition
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

#ifndef __LR1110_TRACKER_BOARD_H__
#define __LR1110_TRACKER_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "smtc_hal.h"
#include "board-config.h"
#include "lis2de12.h"
#include "leds.h"
#include "lr1110.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Initializes the radio I/Os pins context
 *
 * \param [in] context Radio abstraction
 */
void lr1110_modem_board_init_io_context( void* context );

/*!
 * \brief Initializes the radio I/Os pins interface
 *
 * \param [in] context Radio abstraction
 */
void lr1110_modem_board_init_io( const void* context );

/*!
 * \brief De-initializes the radio I/Os pins interface.
 *
 * \param [in] context Radio abstraction
 *
 * \remark Useful when going in MCU low power modes
 */
void lr1110_modem_board_deinit_io( const void* context );

/*!
 * \brief De-initializes the radio I/Os pins interface for deep sleep purpose --> switch Busy and DIO in analog input.
 *
 * \param [in] context Radio abstraction
 *
 * \remark Useful when going in MCU low power modes
 */
void lr1110_modem_board_analog_deinit_io( const void* context );

/*!
 * \brief Sets the radio output power.
 *
 * \param [in] context Radio abstraction
 *
 * \param [in] tx_power_offset power Sets the RF output power offset
 */
void lr1110_modem_board_set_rf_tx_power_offset( const void* context, int8_t tx_power_offset );

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \param [in] context Radio abstraction
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t lr1110_modem_board_get_tcxo_wakeup_time( const void* context );

/*!
 * \brief Initializes the radio driver
 *
 * \param [in] context Radio abstraction
 *
 * \param [in] event Pointeur to the event callbacks
 *
 * \retval Status of the init
 */
lr1110_modem_response_code_t lr1110_modem_board_init( const void* context, lr1110_modem_event_t* event );

/*!
 * \brief Flush the modem event queue
 *
 * \param [in] context Radio abstraction
 */
lr1110_modem_response_code_t lr1110_modem_board_event_flush( const void* context );

/*!
 * \brief Read the event line value used to process the event queue
 *
 * \param [in] context Radio abstraction
 */
bool lr1110_modem_board_read_event_line( const void* context );

/*!
 * \brief turn on the LNA
 */
void lr1110_modem_board_lna_on( void );

/*!
 * \brief turn off the LNA
 */
void lr1110_modem_board_lna_off( void );

/*!
 * \brief Enable or disable the hall effect sensor
 *
 * \param [in] enable Enable or Disable the hall effect sensor
 */
void lr1110_modem_board_hall_effect_enable( bool enable );

/*!
 * \brief convert the GPS time in unix time \note assume that GPS time is right
 *
 * \param [in] context Radio abstraction
 *
 * \retval Unix time in s.
 */
uint32_t lr1110_modem_board_get_systime_from_gps( const void* context );

/*!
 * \brief notify the user is the modem is ready
 *
 * \retval Modem ready state.
 */
bool lr1110_modem_board_is_ready( void );

/*!
 * \brief set the modem is ready flag
 *
 * \param [in] ready ready state
 */
void lr1110_modem_board_set_ready( bool ready );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_TRACKER_BOARD_H__

/* --- EOF ------------------------------------------------------------------ */
