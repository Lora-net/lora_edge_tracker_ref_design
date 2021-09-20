/*!
 * @file      leds.c
 *
 * @brief     leds driver implementation.
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
#include "leds.h"

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

void leds_init( void )
{
    hal_gpio_init_out( LED_TX, 1 );
    hal_gpio_init_out( LED_RX, 1 );
}

void leds_deinit( void )
{
    hal_gpio_deinit( LED_TX );
    hal_gpio_deinit( LED_RX );
}

void leds_on( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED1 */
        hal_gpio_set_value( LED_TX, GPIO_PIN_RESET );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED2 */
        hal_gpio_set_value( LED_RX, GPIO_PIN_RESET );
    }
}

void leds_off( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED1 */
        hal_gpio_set_value( LED_TX, GPIO_PIN_SET );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED2 */
        hal_gpio_set_value( LED_RX, GPIO_PIN_SET );
    }
}

void leds_toggle( uint8_t leds )
{
    if( leds & LED_TX_MASK )
    {
        /* LED1 */
        hal_gpio_toggle( LED_TX );
    }
    if( leds & LED_RX_MASK )
    {
        /* LED2 */
        hal_gpio_toggle( LED_RX );
    }
}

void leds_blink( uint8_t leds, uint32_t delay, uint8_t nb_blink, bool reset_leds )
{
    uint8_t i = 0;

    if( reset_leds == true )
    {
        leds_off( LED_ALL_MASK );
    }

    while( i < nb_blink )
    {
        i++;
        leds_on( leds );
        HAL_Delay( delay / 2 );
        leds_off( leds );
        HAL_Delay( delay / 2 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
