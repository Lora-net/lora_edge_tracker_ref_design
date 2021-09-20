/*!
 * @ingroup   apps_tx_continuous
 * @file      main_test_tx_continuous.c
 *
 * @brief     TX continuous test implementation
 *
 * @copyright
 * @parblock
 * Revised BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
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
 * @endparblock
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include "lr1110_tracker_board.h"

/*!
 * @addtogroup apps_tx_continuous
 * LR1110 Continuous transmission test application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * @brief TX continuous or modulated
 */
#define TX_MODULATED true

/*!
 * @brief TX single, \note only on modulated
 */
#define TX_SINGLE false

/*!
 * @brief TX continuous, \note only on modulated
 */
#define TX_CONTINUOUS true

#if( TX_CONTINUOUS == TX_SINGLE )
#error "please define TX continuous or single"
#endif

/*!
 * @brief Delay in ms between two TX single
 */
#define TX_SINGLE_INTER_DELAY 1000

/*!
 * @brief Tx Power used during test, in dBm.
 */
#define TX_POWER_USED 14

/*!
 * @brief Tx Power offset used during test
 */
#define TX_POWER_OFFSET 0

/*!
 * @brief Tx payload len \note only on modulated
 */
#define TX_PAYLOAD_LEN 51

/*!
 * @brief LoRaWAN regulatory region.
 * One of:
 * LR1110_LORAWAN_REGION_EU868
 * LR1110_LORAWAN_REGION_US915
 * LR1110_LORAWAN_REGION_AU915
 * LR1110_LORAWAN_REGION_AS923_GRP1
 * LR1110_LORAWAN_REGION_CN470
 * LR1110_LORAWAN_REGION_AS923_GRP2
 * LR1110_LORAWAN_REGION_AS923_GRP3
 * LR1110_LORAWAN_REGION_IN865
 * LR1110_LORAWAN_REGION_KR920
 * LR1110_LORAWAN_REGION_RU864
 */
#define LORAWAN_REGION_USED LR1110_LORAWAN_REGION_EU868

/*!
 * @brief Frequency used during test, \note the frequency SHALL be allow by the lorawan region,
 *       set 915MHz with EU868 region will not work
 */
#define FREQUENCY 868100000

/*!
 * @brief Spreading factor for test mode
 * One of:
 * LR1110_MODEM_TST_MODE_FSK  = 0x00,
 * LR1110_MODEM_TST_MODE_SF7  = 0x01,
 * LR1110_MODEM_TST_MODE_SF8  = 0x02,
 * LR1110_MODEM_TST_MODE_SF9  = 0x03,
 * LR1110_MODEM_TST_MODE_SF10 = 0x04,
 * LR1110_MODEM_TST_MODE_SF11 = 0x05,
 * LR1110_MODEM_TST_MODE_SF12 = 0x06,
 */
#define SPREADING_FACTOR_USED LR1110_MODEM_TST_MODE_SF7

/*!
 * @brief bandwidth for test mode
 * One of:
 * LR1110_MODEM_TST_MODE_125_KHZ = 0x00,
 * LR1110_MODEM_TST_MODE_250_KHZ = 0x01,
 * LR1110_MODEM_TST_MODE_500_KHZ = 0x02,
 */
#define BANDWIDTH_USED LR1110_MODEM_TST_MODE_125_KHZ

/*!
 * @brief bandwidth for test mode
 * One of:
 * LR1110_MODEM_TST_MODE_4_5 = 0x00,
 * LR1110_MODEM_TST_MODE_4_6 = 0x01,
 * LR1110_MODEM_TST_MODE_4_7 = 0x02,
 * LR1110_MODEM_TST_MODE_4_8 = 0x03,
 */
#define CODING_RATE_USED LR1110_MODEM_TST_MODE_4_5

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

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Main application entry point.
 */
int main( void )
{
    lr1110_modem_event_callback_t lr1110_modem_event_callback;
    lr1110_modem_version_t        modem;
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Init LR1110 modem-e event */
    lr1110_modem_event_callback.reset = lr1110_modem_reset_event;
    lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );

    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E TX continuous demo application ==== ######\r\n\r\n" );

    /* LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n\r\n", modem.bootloader );

    modem_response_code = lr1110_modem_set_region( &lr1110, LORAWAN_REGION_USED );

    modem_response_code = lr1110_modem_set_tx_power_offset( &lr1110, TX_POWER_OFFSET );

    modem_response_code = lr1110_modem_test_mode_start( &lr1110 );

    HAL_DBG_TRACE_PRINTF( "TX PARAM\r\n" );
    HAL_DBG_TRACE_PRINTF( "FREQ         : %d MHz\r\n", FREQUENCY );
    HAL_DBG_TRACE_PRINTF( "REGION       : %d\r\n", LORAWAN_REGION_USED );
    HAL_DBG_TRACE_PRINTF( "TX POWER     : %d dBm\r\n", TX_POWER_USED );
    if( TX_MODULATED )
    {
        HAL_DBG_TRACE_PRINTF( "TX           : MODULATED\r\n" );
        if( SPREADING_FACTOR_USED != LR1110_MODEM_TST_MODE_FSK )
        {
            HAL_DBG_TRACE_PRINTF( "MODULATION   : LORA\r\n" );
            HAL_DBG_TRACE_PRINTF( "SF           : %d\r\n", SPREADING_FACTOR_USED + 6 );
            HAL_DBG_TRACE_PRINTF( "CR           : 4/%d\r\n", CODING_RATE_USED + 5 );
        }
        else
        {
            HAL_DBG_TRACE_PRINTF( "MODULATION   : FSK\r\n" );
        }
        HAL_DBG_TRACE_PRINTF( "BW           : %d\r\n", BANDWIDTH_USED );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "TX           : CONTINUOUS\r\n" );
    }

#if( TX_MODULATED )
    if( TX_CONTINUOUS == true )
    {
        modem_response_code = lr1110_modem_test_tx_cont( &lr1110, FREQUENCY, TX_POWER_USED, SPREADING_FACTOR_USED,
                                                         BANDWIDTH_USED, CODING_RATE_USED, TX_PAYLOAD_LEN );
    }
    else
    {
        while( 1 )
        {
            HAL_DBG_TRACE_PRINTF( "TX\r\n" );
            modem_response_code = lr1110_modem_test_tx_single( &lr1110, FREQUENCY, TX_POWER_USED, SPREADING_FACTOR_USED,
                                                               BANDWIDTH_USED, CODING_RATE_USED, TX_PAYLOAD_LEN );
            HAL_Delay( TX_SINGLE_INTER_DELAY );
        }
    }
#else
    modem_response_code = lr1110_modem_test_tx_cw( &lr1110, FREQUENCY, TX_POWER_USED );
#endif

    while( 1 )
    {
        lr1110_modem_event_process( &lr1110 );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_tracker_board_is_ready( ) == true )
    {
        /* System reset */
        hal_mcu_reset( );
    }
    else
    {
        lr1110_tracker_board_set_ready( true );
    }
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
