/*!
 * @file      smtc_hal_rng.c
 *
 * @brief     Board specific package RNG API implementation
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

#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include "stm32wbxx_hal.h"
#include "smtc_hal_rng.h"
#include "smtc_hal_mcu.h"

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

static RNG_HandleTypeDef rng_handle;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

uint32_t hal_rng_get_random( void )
{
    uint32_t rand_nb = 0;

    /* Init and enable RNG */
    rng_handle.Instance = RNG;

    if( HAL_RNG_Init( &rng_handle ) != HAL_OK )
    {
        hal_mcu_panic( );
    }

    /* Wait for data ready interrupt: 42+4 RNG clock cycles */
    HAL_RNG_GenerateRandomNumber( &rng_handle, &rand_nb  );

    /* Disable RNG */
    HAL_RNG_DeInit( &rng_handle );

    return rand_nb;
}

uint32_t hal_rng_get_random_in_range( const uint32_t val_1, const uint32_t val_2 )
{
    if( val_1 <= val_2 )
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_2 - val_1 + 1 ) ) + val_1 );
    }
    else
    {
        return ( uint32_t )( ( hal_rng_get_random( ) % ( val_1 - val_2 + 1 ) ) + val_2 );
    }
}

int32_t hal_rng_get_signed_random_in_range( const int32_t val_1, const int32_t val_2 )
{
    uint32_t tmp_range = 0;  // ( val_1 <= val_2 ) ? ( val_2 - val_1 ) : ( val_1 - val_2 );

    if( val_1 <= val_2 )
    {
        tmp_range = ( val_2 - val_1 );
        return ( int32_t )( ( val_1 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
    else
    {
        tmp_range = ( val_1 - val_2 );
        return ( int32_t )( ( val_2 + hal_rng_get_random_in_range( 0, tmp_range ) ) );
    }
}

void HAL_RNG_MspInit( RNG_HandleTypeDef* hrng )
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* Select MSI output as RNG clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RNG;
    PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_MSI;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    
    /* RNG Peripheral clock enable */
    __RNG_CLK_ENABLE( );
}

void HAL_RNG_MspDeInit( RNG_HandleTypeDef* hrng )
{
    /* Enable RNG reset state */
    __RNG_FORCE_RESET( );

    /* Release RNG from reset state */
    __RNG_RELEASE_RESET( );
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
