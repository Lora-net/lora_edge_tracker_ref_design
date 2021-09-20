/**
 * @file      external_supply.h
 *
 * @brief     External supply driver definition.
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

#ifndef __EXTERNAL_SUPPLY_H__
#define __EXTERNAL_SUPPLY_H__

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
 * @brief LNA SUPPLY MASK
 */
#define LNA_SUPPLY_MASK 0x01

/*!
 * @brief 2G4 SPDT SUPPLY MASK
 */
#define SPDT_2G4_MASK 0x02

/*!
 * @brief VCC SENSORS SUPPLY MASK
 */
#define VCC_SENSORS_SUPPLY_MASK 0x04

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Init External supply
 *
 * @param [in] vcc_mask Supply MASK to turn on supply
 */
void external_supply_init( uint8_t vcc_mask );

/*!
 * @brief Deinit External supply
 *
 * @param [in] vcc_mask Supply MASK to turn off supply
 */
void external_supply_deinit( uint8_t vcc_mask );

/*!
 * @brief Turn ON the LNA
 */
void lna_on( void );

/*!
 * @brief Turn Off the LNA
 */
void lna_off( void );

/*!
 * @brief Turn ON the VCC sensors
 */
void vcc_sensors_on( void );

/*!
 * @brief Turn Off the VCC sensors
 */
void vcc_sensors_off( void );

/*!
 * @brief Turn ON the 2G4 SPDT
 */
void spdt_2g4_on( void );

/*!
 * @brief Turn OFF the 2G4 SPDT
 */
void spdt_2g4_off( void );

#ifdef __cplusplus
}
#endif

#endif  //__EXTERNAL_SUPPLY_H__

/* --- EOF ------------------------------------------------------------------ */
