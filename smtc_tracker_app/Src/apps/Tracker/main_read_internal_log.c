/*!
 * @file      main_read_internal_log.c
 *
 * @brief     read Internal Log implementation
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

#include "lr1110_tracker_board.h"
#include "main_tracker.h"
#include "tracker_utility.h"

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

/*!
 * @brief Tracker context structure
 */
extern tracker_ctx_t tracker_ctx;

/*!
 * @brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &tracker_ctx.modem_version );

    tracker_restore_app_ctx( );
    tracker_restore_internal_log_ctx( );

    HAL_DBG_TRACE_MSG( "Scan : \r\n\r\n" );
    leds_on( LED_TX_MASK );

    /* Send over UART scan results */
    tracker_restore_internal_log( );

    leds_on( LED_TX_MASK );

    HAL_Delay( 1000 );

    HAL_DBG_TRACE_MSG( "\r\nPress button to erase memory\r\n" );

    while( get_usr_button_irq_state( ) != true )
    {
        leds_on( LED_TX_MASK );
        HAL_Delay( 250 );
        leds_off( LED_TX_MASK );
        HAL_Delay( 250 );
    }
    clear_usr_button_irq_state( );

    HAL_DBG_TRACE_MSG( "Erase\r\n" );

    tracker_erase_internal_log( );

    HAL_DBG_TRACE_MSG( "Erase Done\r\n" );

    while( 1 )
    {
        leds_on( LED_RX_MASK );
        HAL_Delay( 1000 );
        leds_off( LED_RX_MASK );
        HAL_Delay( 1000 );
    }
}

/* --- EOF ------------------------------------------------------------------ */
