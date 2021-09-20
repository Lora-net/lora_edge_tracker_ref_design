/*!
 * @file      main_BLE_standalone.c
 *
 * @brief     main BLE standalone implementation
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
#include "tracker_utility.h"
#include "ble_thread.h"
#include "app_entry.h"

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
 * @brief  Main application entry point.
 */
int main( void )
{
    uint8_t dev_eui[8]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t join_eui[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint8_t app_key[16] = { 0x6D, 0xA2, 0x64, 0xA2, 0xA6, 0xB7, 0x09, 0xDB,
                            0xA6, 0xD5, 0x5A, 0xA6, 0x97, 0x28, 0x0E, 0x25 };

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== Tracker BLE demo application ==== ######\r\n\r\n" );

    lr1110_modem_get_dev_eui( &lr1110, dev_eui );
    lr1110_modem_get_join_eui( &lr1110, join_eui );

    tracker_init_app_ctx( dev_eui, join_eui, app_key, false );

    while( 1 )
    {
        uint8_t status;
        /* Turn on the 2G4 SPDT and set it into the right direction */
        spdt_2g4_on( );
        set_ble_antenna( );

        start_ble_thread( 5000 );

        HAL_DBG_TRACE_INFO( "###### ===== GO in low power mode ==== ######\r\n\r\n" );
        status = lr1110_modem_set_alarm_timer( &lr1110, 5 );
        HAL_DBG_TRACE_PRINTF( "wake up in %d s status %d \r\n", 5, status );

        lr1110_modem_event_process( &lr1110 );
        hal_mcu_low_power_handler( );
        HAL_DBG_TRACE_INFO( "###### ===== Exit in low power mode ==== ######\r\n\r\n" );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
