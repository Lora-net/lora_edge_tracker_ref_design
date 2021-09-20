/*!
 * @file      external_supply.c
 *
 * @brief     External supply driver implementation.
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

#include "smtc_hal.h"

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
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void external_supply_init( uint8_t vcc_mask )
{
    if( vcc_mask & VCC_SENSORS_SUPPLY_MASK )
    {
        hal_gpio_init_out( VCC_SENSORS_MCU, 0 );
    }
    if( vcc_mask & LNA_SUPPLY_MASK )
    {
        hal_gpio_init_out( LNA_PON, 0 );
    }
    if( vcc_mask & SPDT_2G4_MASK )
    {
        hal_gpio_init_out( VCC_SWITCH_WIFI_BLE, 0 );
    }
}

void external_supply_deinit( uint8_t vcc_mask )
{
    if( vcc_mask & VCC_SENSORS_SUPPLY_MASK )
    {
        hal_gpio_deinit( VCC_SENSORS_MCU );
    }
    if( vcc_mask & LNA_SUPPLY_MASK )
    {
        hal_gpio_deinit( LNA_PON );
    }
    if( vcc_mask & SPDT_2G4_MASK )
    {
        hal_gpio_deinit( VCC_SWITCH_WIFI_BLE );
    }
}

void lna_on( void ) { hal_gpio_set_value( LNA_PON, 1 ); }

void lna_off( void ) { hal_gpio_set_value( LNA_PON, 0 ); }

void spdt_2g4_on( void ) { hal_gpio_set_value( VCC_SWITCH_WIFI_BLE, 1 ); }

void spdt_2g4_off( void ) { hal_gpio_set_value( VCC_SWITCH_WIFI_BLE, 0 ); }

void vcc_sensors_on( void ) { hal_gpio_set_value( VCC_SENSORS_MCU, 1 ); }

void vcc_sensors_off( void ) { hal_gpio_set_value( VCC_SENSORS_MCU, 0 ); }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
