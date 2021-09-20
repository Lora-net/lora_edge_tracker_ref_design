/*!
 * @file      hall_effect.c
 *
 * @brief     Hall effect sensor driver implementation.
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
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Hardware INT IRQ callback initialization
 */
void hall_effect_irq_handler( void* obj );

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief Hall Effect interrupt flag state
 */
static bool hall_effect_irq_state = false;

/*!
 * @brief Hall Effect gpio irq definition
 */
hal_gpio_irq_t hall_effect = {
    .pin      = EFFECT_HALL_OUT,
    .callback = hall_effect_irq_handler,
    .context  = NULL,
};

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void hall_effect_init( bool irq_enable )
{
    if( irq_enable == true )
    {
        hal_gpio_init_in( hall_effect.pin, HAL_GPIO_PULL_MODE_UP, HAL_GPIO_IRQ_MODE_FALLING, &hall_effect );
    }
    else
    {
        hal_gpio_init_in( hall_effect.pin, HAL_GPIO_PULL_MODE_UP, HAL_GPIO_IRQ_MODE_OFF, NULL );
    }
}

void hall_effect_deinit( void )
{
    hal_gpio_irq_deatach( &hall_effect );
    hal_gpio_deinit( hall_effect.pin );
}

uint8_t read_hall_effect_output( void ) { return hal_gpio_get_value( hall_effect.pin ); }

bool get_hall_effect_irq_state( void ) { return hall_effect_irq_state; }

void clear_hall_effect_irq_state( void )
{
    leds_off( LED_RX_MASK );
    hall_effect_irq_state = false;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void hall_effect_irq_handler( void* context )
{
    leds_on( LED_RX_MASK );
    hall_effect_irq_state = true;
}

/* --- EOF ------------------------------------------------------------------ */
