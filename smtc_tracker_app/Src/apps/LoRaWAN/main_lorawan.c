/*!
 * @ingroup   apps_lorawan
 * @file      main_lorawan.c
 *
 * @brief     lr1110 Modem-E Class A/C device implementation
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
 * @addtogroup apps_lorawan
 * LR1110 Modem-E Class A/C device implementation
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include "main_lorawan.h"
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
 * @brief User application data size
 */
static uint8_t app_data_size = 0;

/*!
 * @brief User application data
 */
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * @brief Device states
 */
static enum device_state_e {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
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
 * @brief   Send an application frame on LoRaWAN port defined by LORAWAN_APP_PORT
 *
 * @param [in] tx_frame_buffer   buffer containing the LoRaWAN buffer
 * @param [in] tx_frame_buffer_size   payload len buffer
 * @param [in] confirmed   send a confirmed or unconfirmed uplink [false : unconfirmed / true : confirmed]
 *
 * @returns  [true: frame could be send, false: error]
 */
static bool send_frame( const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size,
                        const lr1110_modem_uplink_type_t confirmed );

/*!
 * @brief Parse the received downlink
 *
 * Demonstrates how a TLV-encoded command sequence received by downlink can
 *  control the state of an LED.
 * Can easily be extended to handle other commands received on the same port
 *  or another port.
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size );

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
 * @brief Tx done event callback
 *
 * @param [in] status tx done status \ref lr1110_modem_tx_done_event_t
 */
static void lr1110_modem_tx_done( lr1110_modem_tx_done_event_t status );

/*!
 * @brief Down data event callback.
 *
 * @param [in] rssi    rssi in signed value in dBm + 64
 * @param [in] snr     snr signed value in 0.25 dB steps
 * @param [in] flags   rx flags \see down_data_flag_t
 * @param [in] port    LoRaWAN port
 * @param [in] payload Received buffer pointer
 * @param [in] size    Received buffer size
 */
static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                                    const uint8_t* payload, uint8_t size );

/*!
 * @brief Stream done event callback
 *
 * @param [in] mute    modem mute status \ref lr1110_modem_mute_t
 */
static void lr1110_modem_mute( lr1110_modem_mute_t mute );

/*!
 * @brief Set conf event callback
 *
 * @param [in] tag   \ref lr1110_modem_event_setconf_tag_t
 */
static void lr1110_modem_set_conf( lr1110_modem_event_setconf_tag_t tag );

/*!
 * @brief Automatic switch from mobile to static ADR when connection timeout occurs event callback
 */
static void lr1110_modem_adr_mobile_to_static( void );

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
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_version_t        modem;
    lr1110_modem_event_callback_t lr1110_modem_event_callback = { NULL };
    uint32_t                      pin;

    uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
    uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
    uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E LoRaWAN Class A/C demo application ==== ######\r\n\r\n" );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.reset                = lr1110_modem_reset_event;
    lr1110_modem_event_callback.alarm                = lr1110_modem_alarm;
    lr1110_modem_event_callback.joined               = lr1110_modem_network_joined;
    lr1110_modem_event_callback.join_fail            = lr1110_modem_join_fail;
    lr1110_modem_event_callback.tx_done              = lr1110_modem_tx_done;
    lr1110_modem_event_callback.down_data            = lr1110_modem_down_data;
    lr1110_modem_event_callback.set_conf             = lr1110_modem_set_conf;
    lr1110_modem_event_callback.mute                 = lr1110_modem_mute;
    lr1110_modem_event_callback.adr_mobile_to_static = lr1110_modem_adr_mobile_to_static;
    lr1110_modem_event_callback.new_link_adr         = lr1110_modem_new_link_adr;
    lr1110_modem_event_callback.no_event             = lr1110_modem_no_event;

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

    /* Configure the partial low power mode */
    hal_mcu_partial_sleep_enable( APP_PARTIAL_SLEEP );

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
        case DEVICE_STATE_SEND:
        {
            /* Send LR1110 modem-e charge */
            uint32_t charge     = 0;
            modem_response_code = lr1110_modem_get_charge( &lr1110, &charge );
            if( modem_response_code != ( lr1110_modem_response_code_t ) LR1110_MODEM_HAL_STATUS_OK )
            {
                HAL_DBG_TRACE_PRINTF( "\r\n lr1110_modem_get_charge returned RC : %d \r\n", modem_response_code );
            }

            app_data_buffer[0] = charge;
            app_data_buffer[1] = charge >> 8;
            app_data_buffer[2] = charge >> 16;
            app_data_buffer[3] = charge >> 24;
            app_data_size      = 4;

            /* Send the next frame */
            send_frame( app_data_buffer, app_data_size,
                        LORAWAN_CONFIRMED_MSG_ON ? LR1110_MODEM_UPLINK_CONFIRMED : LR1110_MODEM_UPLINK_UNCONFIRMED );

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            /* Schedule next packet transmission */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, APP_TX_DUTYCYCLE / 1000 );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            HAL_DBG_TRACE_PRINTF( "lr1110_modem_set_alarm_timer : %d s\r\n\r\n", APP_TX_DUTYCYCLE / 1000 );

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

static bool send_frame( const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size,
                        const lr1110_modem_uplink_type_t tx_confirmed )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    uint8_t                      tx_max_payload;
    int32_t                      duty_cycle;

    lr1110_modem_get_duty_cycle_status( &lr1110, &duty_cycle );

    if( duty_cycle < 0 )
    {
        HAL_DBG_TRACE_INFO( "DUTY CYCLE, NEXT UPLINK AVAILABLE in %d milliseconds \r\n\r\n", duty_cycle );

        return false;
    }

    modem_response_code = lr1110_modem_get_next_tx_max_payload( &lr1110, &tx_max_payload );
    if( modem_response_code != ( lr1110_modem_response_code_t ) LR1110_MODEM_HAL_STATUS_OK )
    {
        HAL_DBG_TRACE_ERROR( "\r\n lr1110_modem_get_next_tx_max_payload RC : %d \r\n", modem_response_code );
    }

    if( tx_frame_buffer_size > tx_max_payload )
    {
        /* Send empty frame in order to flush MAC commands */
        HAL_DBG_TRACE_PRINTF( "\r\n APP DATA > MAX PAYLOAD AVAILABLE (%d bytes) \r\n", tx_max_payload );

        modem_response_code = lr1110_modem_request_tx( &lr1110, LORAWAN_APP_PORT, tx_confirmed, NULL, 0 );
    }
    else
    {
        modem_response_code =
            lr1110_modem_request_tx( &lr1110, LORAWAN_APP_PORT, tx_confirmed, tx_frame_buffer, tx_frame_buffer_size );
    }

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "LR1110 MODEM-E REQUEST TX \r\n\r\n" );

        return true;
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "LR1110 MODEM-E REQUEST TX ERROR CMD, modem_response_code : %d \r\n\r\n",
                             modem_response_code );

        return false;
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
        else if( ( ( modem_status & LR1110_LORAWAN_MUTE ) == LR1110_LORAWAN_MUTE ) ||
                 ( ( modem_status & LR1110_LORAWAN_SUSPEND ) == LR1110_LORAWAN_SUSPEND ) )
        {
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED ) ||
                 ( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM ) ||
                 ( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD ) )
        {
            device_state = DEVICE_STATE_SEND;
        }
        else if( ( modem_status & LR1110_LORAWAN_JOINING ) == LR1110_LORAWAN_JOINING )
        {
            /* Network not joined yet. Wait */
            device_state = DEVICE_STATE_CYCLE;
        }
        else
        {
            HAL_DBG_TRACE_WARNING( "Unknow modem status %d\r\n\r\n", modem_status );
            device_state = DEVICE_STATE_CYCLE;
        }
    }
}

void lr1110_modem_tx_done( lr1110_modem_tx_done_event_t status )
{
    static uint32_t uplink_cnt = 0;

    if( status != LR1110_MODEM_TX_ERROR )
    {
        HAL_DBG_TRACE_INFO( "###### ===== UPLINK FRAME %lu ==== ######\r\n\r\n", uplink_cnt++ );

        HAL_DBG_TRACE_PRINTF( "CLASS       : %c\r\n", ( LORAWAN_CLASS_USED == LR1110_LORAWAN_CLASS_A ) ? 'A' : 'C' );
        HAL_DBG_TRACE_PRINTF( "TX PORT     : %d\r\n", LORAWAN_APP_PORT );

        if( app_data_size != 0 )
        {
            HAL_DBG_TRACE_MSG( "TX DATA     : " );
            if( LORAWAN_CONFIRMED_MSG_ON )
            {
                HAL_DBG_TRACE_PRINTF( "CONFIRMED - %s\r\n", ( status == LR1110_MODEM_CONFIRMED_TX ) ? "ACK" : "NACK" );
            }
            else
            {
                HAL_DBG_TRACE_MSG( "UNCONFIRMED\r\n" );
            }
            HAL_DBG_TRACE_MSG( "TX DATA     : " );
            print_hex_buffer( app_data_buffer, app_data_size );
            HAL_DBG_TRACE_MSG( "\r\n" );
        }

        lr1110_tracker_board_led_pulse( LED_TX_MASK, true, LED_PERIOD_MS );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== TX ERROR ==== ######\r\n\r\n" );
    }
}

static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                                    const uint8_t* payload, uint8_t size )
{
    static uint32_t downlink_cnt = 0;

    HAL_DBG_TRACE_INFO( "###### ===== DOWNLINK FRAME %lu ==== ######\r\n\r\n", downlink_cnt++ );

    HAL_DBG_TRACE_PRINTF( "RX WINDOW   : %d\r\n",
                          flags & ( LR1110_MODEM_DOWN_DATA_EVENT_DNW1 | LR1110_MODEM_DOWN_DATA_EVENT_DNW2 ) );

    HAL_DBG_TRACE_PRINTF( "RX PORT     : %d\r\n", port );

    if( size != 0 )
    {
        HAL_DBG_TRACE_MSG( "RX DATA     : " );
        print_hex_buffer( payload, size );
    }

    HAL_DBG_TRACE_PRINTF( "RX RSSI     : %d\r\n", rssi );
    HAL_DBG_TRACE_PRINTF( "RX SNR      : %d\r\n\r\n", snr );

    lr1110_tracker_board_led_pulse( LED_RX_MASK, true, LED_PERIOD_MS );

    parse_downlink_frame( port, payload, size );
}

static void lr1110_modem_mute( lr1110_modem_mute_t mute )
{
    if( mute == LR1110_MODEM_UNMUTED )
    {
        HAL_DBG_TRACE_INFO( "###### ===== MODEM UNMUTED ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_INFO( "###### ===== MODEM MUTED ==== ######\r\n\r\n" );
    }
}

static void lr1110_modem_set_conf( lr1110_modem_event_setconf_tag_t tag )
{
    HAL_DBG_TRACE_INFO( "###### ===== MODEM SET CONF %02X ==== ######\r\n\r\n", tag );
}

static void lr1110_modem_adr_mobile_to_static( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== ADR HAS SWITCHED FROM MOBILE TO STATIC ==== ######\r\n\r\n" );
}

static void lr1110_modem_new_link_adr( void ) { HAL_DBG_TRACE_INFO( "###### ===== NEW LINK ADR ==== ######\r\n\r\n" ); }

static void lr1110_modem_no_event( void ) { HAL_DBG_TRACE_INFO( "###### ===== NO EVENT ==== ######\r\n\r\n" ); }

static void parse_downlink_frame( uint8_t port, const uint8_t* payload, uint8_t size )
{
    switch( port )
    {
    case APPLICATION_MSG_PORT:
    {
        uint8_t tag           = 0;
        uint8_t len           = 0;
        uint8_t payload_index = 0;

        while( payload_index < size )
        {
            tag = payload[payload_index++];
            len = payload[payload_index++];

            switch( tag )
            {
            case SET_RX_LED_CMD:
            {
                if( payload[payload_index] == 1 )
                {
                    lr1110_tracker_board_led_set( LED_RX_MASK, true );
                }
                else
                {
                    lr1110_tracker_board_led_set( LED_RX_MASK, false );
                }
                payload_index += len;
                break;
            }
            default:
                payload_index += len;
                break;
            }
        }
        break;
    }
    default:
        break;
    }
}

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
