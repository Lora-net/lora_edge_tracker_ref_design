/*!
 * @ingroup   apps_stream
 * @file      main_stream.c
 *
 * @brief     lr1110 Modem-E Data Streaming example application
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

/*!
 * @addtogroup apps_stream
 * LR1110 Modem-E Data Streaming example application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include "main_stream.h"
#include "lorawan_commissioning.h"
#include "lr1110_tracker_board.h"
#include "utilities.h"
#include "apps_utilities.h"

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

/*!
 * @brief ADR custom list when LORAWAN_DEFAULT_DATARATE is set to LR1110_MODEM_ADR_PROFILE_CUSTOM
 */
static uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03,
                                       0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00 };

/*!
 * @brief Device states
 */
static enum device_state_e {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_STREAM,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} device_state = DEVICE_STATE_INIT;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * Executes the network Join request
 */
static void join_network( void );

/*!
 * @brief Lorawan default init
 *
 * @param [in] region LoRaWAN region to use \ref lr1110_modem_regions_t
 * @param [in] lorawan_class LoRaWAN class to use \ref lr1110_modem_classes_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class );

/*!
 * @addtogroup lr1110_evt_callback
 * LR1110 event callbacks
 * @{
 */

/*!
 * @brief Reset event callback
 *
 * @param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*!
 * @brief Network Joined event callback
 */
static void lr1110_modem_network_joined( void );

/*!
 * @brief Join Fail event callback
 */
static void lr1110_modem_join_fail( void );

/*!
 * @brief Alarm event callback
 */
static void lr1110_modem_alarm( void );

/*!
 * @brief Stream done event callback
 */
static void lr1110_modem_stream_done( void );

/*!
 * @brief New link ADR request event callback
 */
static void lr1110_modem_new_link_adr( void );

/*!
 * @brief No event exists event callback
 */
static void lr1110_modem_no_event( void );

/*!
 * @}
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
    volatile lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_version_t                modem;
    lr1110_modem_event_callback_t         lr1110_modem_event_callback = { NULL };
    uint32_t                              pin;

    uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
    uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
    uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E Data Streaming example application ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\r\n\r\n", MAJOR_APP_VERSION, MINOR_APP_VERSION,
                          SUB_MINOR_APP_VERSION );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.reset        = lr1110_modem_reset_event;
    lr1110_modem_event_callback.alarm        = lr1110_modem_alarm;
    lr1110_modem_event_callback.joined       = lr1110_modem_network_joined;
    lr1110_modem_event_callback.join_fail    = lr1110_modem_join_fail;
    lr1110_modem_event_callback.stream_done  = lr1110_modem_stream_done;
    lr1110_modem_event_callback.new_link_adr = lr1110_modem_new_link_adr;
    lr1110_modem_event_callback.no_event     = lr1110_modem_no_event;

    if( lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n" );
    }

    /* LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E VERSION ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n", modem.bootloader );

#if( USE_PRODUCTION_KEYS == 1 )
    /* When the production keys are used, DevEUI = ChipEUI and JoinEUI is the one defined in lorawan_comissioning.h */
    modem_response_code = lr1110_modem_get_chip_eui( &lr1110, dev_eui );
#endif

    /* Basic LoRaWAN configuration */
    if( lorawan_init( LORAWAN_REGION_USED, LORAWAN_CLASS_USED ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n" );
    }

    while( 1 )
    {
        /* Process Event */
        if( lr1110.event.callback != NULL )
        {
            lr1110_modem_event_process( &lr1110 );
        }

        switch( device_state )
        {
        case DEVICE_STATE_INIT:
        {
            HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E INIT ==== ######\r\n\r\n" );

            /* Set Keys */
            modem_response_code = lr1110_modem_set_dev_eui( &lr1110, dev_eui );
            modem_response_code = lr1110_modem_set_join_eui( &lr1110, join_eui );

#if( USE_SEMTECH_JOIN_SERVER == 0 )
            modem_response_code = lr1110_modem_set_app_key( &lr1110, app_key );
#else
            modem_response_code = lr1110_modem_derive_keys( &lr1110 );
            modem_response_code = lr1110_modem_get_pin( &lr1110, &pin );
#endif

            device_state = DEVICE_STATE_JOIN;
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            /* Display used keys */
            print_lorawan_keys( dev_eui, join_eui, app_key, pin, USE_SEMTECH_JOIN_SERVER );

            join_network( );

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_STREAM:
        {
            /* Prepare and start the next stream */
            static uint8_t               payload[STREAM_APP_DATA_MAX_SIZE];
            static uint32_t              stream_cnt = 0;
            lr1110_modem_stream_status_t stream_status;
            int                          byte_count;

            /* Interleave longer and shorter messages in the stream */
            if( stream_cnt % 2 )
            {
                byte_count =
                    snprintf( ( char* ) payload, sizeof( payload ),
                              "Streaming data in a more verbose way (stream count #%.4ld)", ( long ) stream_cnt );
            }
            else
            {
                byte_count =
                    snprintf( ( char* ) payload, sizeof( payload ), "Streaming data (#%ld)", ( long ) stream_cnt );
            }

            /* Push the Payload in the FiFo stream */
            lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
            if( stream_status.free > byte_count )
            {
                HAL_DBG_TRACE_PRINTF( "Adding %d bytes in streaming FiFo\r\n", byte_count );
                lr1110_modem_send_stream_data( &lr1110, LORAWAN_STREAM_APP_PORT, payload, byte_count );
                stream_cnt++;
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "Not enough space, needed = %d bytes - free = %d bytes\r\n", byte_count,
                                      stream_status.free );
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            /* Schedule next packet transmission */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, APP_STREAM_PERIOD / 1000 );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            HAL_DBG_TRACE_PRINTF( "lr1110_modem_set_alarm_timer : %d s\r\n\r\n", APP_STREAM_PERIOD / 1000 );

            device_state = DEVICE_STATE_SLEEP;

            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            /* go in low power */
            if( lr1110_tracker_board_read_event_line( &lr1110 ) == false )
            {
                hal_mcu_low_power_handler( );
            }

            break;
        }
        default:
        {
            device_state = DEVICE_STATE_INIT;
            break;
        }
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void join_network( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    /* Starts the join procedure */
    modem_response_code = lr1110_modem_join( &lr1110 );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== JOINING CMD ERROR %d==== ######\r\n\r\n", modem_response_code );
    }
}

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

void lr1110_modem_network_joined( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== JOINED ==== ######\r\n\r\n" );

    /* Set the ADR profile once joined */
    lr1110_modem_set_adr_profile( &lr1110, LORAWAN_DEFAULT_DATARATE, adr_custom_list );

    /* Init the stream once joined */
    lr1110_modem_stream_init( &lr1110, LORAWAN_STREAM_APP_PORT, LR1110_MODEM_SERVICES_ENCRYPTION_DISABLE );
    /* Set the stream redundancy rate */
    lr1110_modem_set_stream_redundancy_rate( &lr1110, TRACKER_STREAM_REDUNDANCY_RATE );
}

void lr1110_modem_join_fail( void ) { HAL_DBG_TRACE_INFO( "###### ===== JOINED FAIL ==== ######\r\n\r\n" ); }

void lr1110_modem_alarm( void )
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== LR1110 ALARM ==== ######\r\n\r\n" );

    modem_response_code = lr1110_modem_get_status( &lr1110, &modem_status );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        modem_status_to_string( modem_status );

        if( ( modem_status & LR1110_LORAWAN_CRASH ) == LR1110_LORAWAN_CRASH )
        {
            hal_mcu_reset( );
        }
        else if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
        {
            /* Network not joined yet. Wait. */
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM )
        {
            /* Stream not done yet. Wait. */
            lr1110_modem_stream_status_t stream_status;

            lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
            HAL_DBG_TRACE_PRINTF( "Streaming ongoing %d bytes remaining %d bytes free \r\n", stream_status.pending,
                                  stream_status.free );
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
        {
            /* Ready for more stream data. */
            device_state = DEVICE_STATE_STREAM;
        }
        else
        {
            HAL_DBG_TRACE_WARNING( "Unexpected modem status %d\r\n\r\n", modem_status );
            device_state = DEVICE_STATE_CYCLE;
        }
    }
}

static void lr1110_modem_stream_done( void )
{
    static uint32_t stream_cnt = 0;

    HAL_DBG_TRACE_INFO( "###### ===== STREAM DONE %d ==== ######\r\n\r\n", stream_cnt++ );
}

static void lr1110_modem_new_link_adr( void ) { HAL_DBG_TRACE_INFO( "###### ===== NEW LINK ADR ==== ######\r\n\r\n" ); }

static void lr1110_modem_no_event( void ) { HAL_DBG_TRACE_INFO( "###### ===== NO EVENT ==== ######\r\n\r\n" ); }

lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class )
{
    lr1110_modem_dm_info_fields_t dm_info_fields;
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    modem_response_code |= lr1110_modem_set_class( &lr1110, lorawan_class );

    if( lorawan_class == LR1110_LORAWAN_CLASS_A )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : A\r\n" );
    }
    if( lorawan_class == LR1110_LORAWAN_CLASS_C )
    {
        HAL_DBG_TRACE_MSG( "CLASS       : C\r\n" );
    }

    modem_response_code |= lr1110_modem_set_region( &lr1110, region );

    switch( region )
    {
    case LR1110_LORAWAN_REGION_EU868:
    {
        HAL_DBG_TRACE_MSG( "REGION      : EU868\r\n\r\n" );
        modem_response_code |= lr1110_modem_activate_duty_cycle( &lr1110, LORAWAN_DUTYCYCLE_ON );

        modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, 0 );
        break;
    }
    case LR1110_LORAWAN_REGION_US915:
    {
        HAL_DBG_TRACE_MSG( "REGION      : US915\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AU915:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AU915\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP1:
    {
        if( LORAWAN_COUNTRY_JAPAN == 1 )
        {
            HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
            /* Activate LBT for 5ms before each transmission with a threshold at -80 dBm */
            modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -80, 5, 1250000 );
        }

        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP1\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_CN470:
    {
        HAL_DBG_TRACE_MSG( "REGION      : CN470\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP2:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP2\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP3:
    {
        HAL_DBG_TRACE_MSG( "REGION      : AS923_GRP3\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_IN865:
    {
        HAL_DBG_TRACE_MSG( "REGION      : IN865\r\n\r\n" );
        break;
    }
    case LR1110_LORAWAN_REGION_KR920:
    {
        HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
        HAL_DBG_TRACE_MSG( "REGION      : KR920\r\n\r\n" );

        /* Activate LBT for 5ms before each transmission with a threshold at -65 dBm */
        modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -65, 5, 1250000 );
        break;
    }
    case LR1110_LORAWAN_REGION_RU864:
    {
        HAL_DBG_TRACE_MSG( "REGION      : RU864\r\n\r\n" );
        break;
    }
    default:
        HAL_DBG_TRACE_ERROR( "No supported region selected\r\n\r\n" );
        break;
    }

    /* Set DM info field */
    dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_CHARGE;
    dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
    dm_info_fields.dm_info_length   = 2;

    modem_response_code |= lr1110_modem_set_dm_info_field( &lr1110, &dm_info_fields );

    modem_response_code |= lr1110_modem_set_dm_info_interval( &lr1110, LR1110_MODEM_REPORTING_INTERVAL_IN_DAY, 1 );

    return modem_response_code;
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
