/**
 * @file      leds.h
 *
 * @brief     Leds driver definition.
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

#ifndef __LEDS_H__
#define __LEDS_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */

/*!
 * \brief LED TX MASK
 */
#define LED_TX_MASK 0x01

/*!
 * \brief LED RX MASK
 */
#define LED_RX_MASK 0x02

/*!
 * \brief LED ALL MASK
 */
#define LED_ALL_MASK 0x03

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init Leds
 */
void leds_init( void );

/*!
 * \brief Deinit Leds
 */
void leds_deinit( void );
/*!
 * \brief Select and turn on Leds
 *
 * \param [in] leds Leds MASK to turn on leds
 */
void leds_on( uint8_t leds );

/*!
 * \brief Select and turn off Leds
 *
 * \param [in] leds Leds MASK to turn off leds
 */
void leds_off( uint8_t leds );

/*!
 * \brief Select and toggle Leds
 *
 * \param [in] leds Leds MASK to turn off leds
 */
void leds_toggle( uint8_t leds );

/*!
 * \brief Select and toggle Leds
 *
 * \param [in] leds Leds MASK to turn off leds
 * \param [in] delay Blink delay
 * \param [in] nb_blink        Number of blink
 * \param [in] reset_leds     Reset leds at the beginning
 */
void leds_blink( uint8_t leds, uint32_t delay, uint8_t nb_blink, bool reset_leds );

#ifdef __cplusplus
}
#endif

#endif //__LEDS_H__
