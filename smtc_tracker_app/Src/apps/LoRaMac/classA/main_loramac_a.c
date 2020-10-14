/*!
 * \file      main_loramac_a.c
 *
 * \brief     lr1110 Modem classA device implementation
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
#include <stdio.h>
#include "lr1110_tracker_board.h"
#include "utilities.h"
#include "Commissioning.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*!
 * \brief Defines the application data transmission duty cycle. 20s, value in [ms].
 */
#define APP_TX_DUTYCYCLE 20000

/*!
 * \brief Defines the watchdog timeout application
 * when device has moved. APP_TX_DUTYCYCLE * 2, value in [ms].
 */
#define WATCHDOG_TIMEOUT APP_TX_DUTYCYCLE * 2

/*!
 * \brief Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND 1000

/*!
 * \brief Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED

/*!
 * \brief LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON false

/*!
 * \brief LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON LR1110_MODEM_DUTY_CYCLE_ENABLE

/*!
 * \brief LoRaWAN application port
 */
#define LORAWAN_APP_PORT 2

/*!
 * \brief User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE 242

/*!
 * \brief Use or not the LoRaWAN production Keys
 */
#define USE_PRODUCTION_KEYS 1

/*!
 * \brief Use or not the Semtech join server
 */
#define USE_SEMTECH_JOIN_SERVER 1

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */
 
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief Radio hardware and global parameters
 */
extern lr1110_t lr1110;

/*!
 * \brief Application port
 */
static uint8_t app_port = LORAWAN_APP_PORT;

/*!
 * \brief ADR custom list when ADR is set to custom
 */
uint8_t adr_custom_list[16] = { 0x05, 0x05, 0x05, 0x04, 0x04, 0x04, 0x03, 0x03,
                                0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x00, 0x00 };

/*!
 * \brief User application data size
 */
static uint8_t app_data_size = 1;

/*!
 * \brief User application data
 */
static uint8_t app_data_buffer[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * \brief Indicates if the node is sending confirmed or unconfirmed messages
 */
static lr1110_modem_uplink_type_t is_tx_confirmed = (lr1110_modem_uplink_type_t) LORAWAN_CONFIRMED_MSG_ON;

/*!
 * \brief Defines the application data transmission duty cycle
 */
static uint32_t tx_duty_cycle_time;

/*!
 * \brief Timer to handle the state of LED TX
 */
static timer_event_t led_tx_timer;

/*!
 * \brief Timer to handle the state of LED RX
 */
static timer_event_t led_rx_timer;

/*!
 * \brief Device states
 */
static enum edevice_state {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} device_state = DEVICE_STATE_INIT;

/*!
 * \brief Uplink counter \note only for trace/debug
 */
static uint32_t uplink_cnt = 0;

/*!
 * \brief Downlink counter \note only for trace/debug
 */
static uint32_t downlink_cnt = 0;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief Prints the provided buffer in HEX
 *
 * \param [in] buffer Buffer to be printed
 *
 * \param [in] size Buffer size to be printed
 */
static void print_hex_buffer( const uint8_t* buffer, uint8_t size );

/*!
 * \brief Prints the LoRaWAN keys
 *
 * \param [in] dev_eui Device EUI to be printed
 *
 * \param [in] join_eui Join EUI to be printed
 *
 * \param [in] app_key Application Key to be printed
 *
 * \param [in] pin pin code
 */
static void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key,
                                const uint32_t pin );

/*!
 * \brief Prints the provided buffer in HEX
 *
 * \param [in] port LoRaWAN application used
 *
 * \param [out] tx_frame_buffer buffer containing the LoRaWAN buffer
 *
 * \param [out] tx_frame_buffer_size payload len buffer
 *
 * \param [in] max_tx_buffer_size the maximum buffer len allowed by the user
 */
static void prepare_tx_frame( uint8_t port, uint8_t* tx_frame_buffer, uint8_t* tx_frame_buffer_size, uint8_t max_tx_buffer_size );

/*!
 * Executes the network Join request
 */
static void join_network( void );

/*!
 * \brief   Prepares the payload of the frame
 *
 * \param [in] port LoRaWAN application used
 *
 * \param [in] tx_frame_buffer   buffer containing the LoRaWAN buffer
 *
 * \param [in] tx_frame_buffer_size   payload len buffer
 *
 * \param [in] confirmed   send a confirmed or unconfirmed uplink [false : unconfirmed / true : confirmed]
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool send_frame( const uint8_t port, const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size, const lr1110_modem_uplink_type_t confirmed );

/*!
 * \brief Function executed on Led TX Timeout event
 */
static void on_led_tx_timer_event( void* context );

/*!
 * \brief Function executed on Led RX Timeout event
 */
static void on_led_rx_timer_event( void* context );

/*!
 * \brief Reset event callback
 *
 * \param [in] reset_count reset counter from the modem
 */
static void lr1110_modem_reset_event( uint16_t reset_count );

/*!
 * \brief Network Joined event callback
 */
static void lr1110_modem_network_joined( void );

/*!
 * \brief Join Fail event callback
 */
static void lr1110_modem_join_fail( void );

/*!
 * \brief Join Fail event callback
 */
static void lr1110_modem_alarm( void );

/*!
 * \brief Tx done event callback
 *
 * \param [in] status tx done status \ref lr1110_modem_tx_done_event_t
 */
static void lr1110_modem_tx_done( lr1110_modem_tx_done_event_t status );

/*!
 * \brief Down data event callback.
 *
 * \param [in] rssi    rssi in signed value in dBm + 64
 * \param [in] snr     snr signed value in 0.25 dB steps
 * \param [in] flags   rx flags \see down_data_flag_t
 * \param [in] port    LoRaWAN port
 * \param [in] payload Received buffer pointer
 * \param [in] size    Received buffer size
 */
static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * \brief Stream done event callback
 *
 * \param [in] mute    modem mute status \ref lr1110_modem_mute_t
 */
static void lr1110_modem_mute( lr1110_modem_mute_t mute );

/*!
 * \brief Set conf event callback
 *
 * \param [in] info_tag    modem mute status \ref lr1110_modem_mute_t
 */
static void lr1110_modem_set_conf( uint8_t info_tag );

/*!
 * \brief Stream done event callback
 */
static void lr1110_modem_stream_done( void );

/*!
 * \brief Time updated by application layer clock synchronisation event callback
 */
static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t  alc_sync_state );

/*!
 * \brief Automatic switch from mobile to static ADR when connection timeout occurs event callback
 */
static void lr1110_modem_adr_mobile_to_static( void );

/*!
 * \brief New link ADR request event callback
 */
static void lr1110_modem_new_link_adr( void );

/*!
 * \brief No event exists event callback
 */
static void lr1110_modem_no_event( void );

/*!
 * \brief lorawan default init
 */
static lr1110_modem_response_code_t lorawan_init( void );

/*!
 * \brief convert lr1110 modem status to string
 */
static void modem_status_to_string( lr1110_modem_status_t modem_status );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * \brief Main application entry point.
 */
int main( void )
{
    volatile lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_version_t                modem;
    lr1110_modem_event_t                  lr1110_modem_event;
    uint32_t                              pin;

    uint8_t dev_eui[8]  = LORAWAN_DEVICE_EUI;
    uint8_t join_eui[8] = LORAWAN_JOIN_EUI;
    uint8_t app_key[16] = LORAWAN_APP_KEY;


    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 Modem ClassA demo application ==== ######\r\n\r\n" );

    /* Init LR1110 modem event */
    lr1110_modem_event.reset                 = lr1110_modem_reset_event;
    lr1110_modem_event.alarm                 = lr1110_modem_alarm;
    lr1110_modem_event.joined                = lr1110_modem_network_joined;
    lr1110_modem_event.join_fail             = lr1110_modem_join_fail;
    lr1110_modem_event.tx_done               = lr1110_modem_tx_done;
    lr1110_modem_event.down_data             = lr1110_modem_down_data;
    lr1110_modem_event.set_conf              = lr1110_modem_set_conf;
    lr1110_modem_event.mute                  = lr1110_modem_mute;
    lr1110_modem_event.stream_done           = lr1110_modem_stream_done;
    lr1110_modem_event.time_updated_alc_sync = lr1110_modem_time_updated_alc_sync;
    lr1110_modem_event.adr_mobile_to_static  = lr1110_modem_adr_mobile_to_static;
    lr1110_modem_event.new_link_adr          = lr1110_modem_new_link_adr;
    lr1110_modem_event.no_event              = lr1110_modem_no_event;

    if( lr1110_modem_board_init( &lr1110, &lr1110_modem_event ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n" );
    }

    /* LR1110 modem version */
    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM VERSION ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", modem.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", modem.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n", modem.bootloader );

#if (USE_PRODUCTION_KEYS == 1)
    modem_response_code = lr1110_modem_get_dev_eui( &lr1110, dev_eui );
    modem_response_code = lr1110_modem_get_join_eui( &lr1110, join_eui );
#endif

    /* Basic LoRaWAN configuration */
    if(lorawan_init( ) != LR1110_MODEM_RESPONSE_CODE_OK)
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n" );
    }

    /* Init the software watchdog */
    hal_mcu_init_software_watchdog( WATCHDOG_TIMEOUT );

    /* Init Leds timer */
    timer_init( &led_tx_timer, on_led_tx_timer_event );
    timer_set_value( &led_tx_timer, 25 );
    timer_init( &led_rx_timer, on_led_rx_timer_event );
    timer_set_value( &led_rx_timer, 25 );

    while( 1 )
    {
        // Process Event
        if( lr1110.event.callback != NULL )
        {
            lr1110_modem_event_process( &lr1110 );
        }

        switch( device_state )
        {
            case DEVICE_STATE_INIT:
            {
                HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM INIT ==== ######\r\n\r\n" );

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
                print_lorawan_keys( dev_eui, join_eui, app_key, pin );

                join_network( );
            
                device_state = DEVICE_STATE_CYCLE;

                break;
            }
            case DEVICE_STATE_SEND:
            {
                prepare_tx_frame( app_port, app_data_buffer, &app_data_size, 4 );

                send_frame( app_port, app_data_buffer, app_data_size, is_tx_confirmed );
                
                device_state = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                /* Reload the software watchdog */
                hal_mcu_reset_software_watchdog( );
                
                device_state = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                tx_duty_cycle_time = ( APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND ) ) / 1000;

                // Schedule next packet transmission
                lr1110_modem_set_alarm_timer( &lr1110, tx_duty_cycle_time );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // The MCU wakes up through events
                hal_mcu_low_power_handler( );

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

void _Error_Handler( int line )
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while( 1 )
    {
        HAL_DBG_TRACE_ERROR( "%s\n", __FUNCTION__ );
    }
    /* USER CODE END Error_Handler_Debug */
}
/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_modem_response_code_t lorawan_init( void )
{
    lr1110_modem_dm_info_fields_t dm_info_fields;
    lr1110_modem_response_code_t  modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    modem_response_code |= lr1110_modem_set_class( &lr1110, LR1110_LORAWAN_CLASS_A );

#if defined( USE_REGION_EU868 )
    HAL_DBG_TRACE_MSG( "REGION      : EU868\r\n\r\n" );
    modem_response_code |= lr1110_modem_set_region( &lr1110, LR1110_LORAWAN_REGION_EU868 );

    modem_response_code |= lr1110_modem_activate_duty_cycle( &lr1110, LORAWAN_DUTYCYCLE_ON);
#endif
#if defined( USE_REGION_US915 )
    HAL_DBG_TRACE_MSG( "REGION      : US915\r\n\r\n" );
    modem_response_code |= lr1110_modem_set_region( &lr1110, LR1110_LORAWAN_REGION_US915 );
#endif

    modem_response_code |= lr1110_modem_set_adr_profile( &lr1110, LORAWAN_DEFAULT_DATARATE, adr_custom_list );

    /* Set DM info field */
    dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_CHARGE;
    dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS;
    dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
    dm_info_fields.dm_info_length   = 3;

    modem_response_code |= lr1110_modem_set_dm_info_field( &lr1110, &dm_info_fields );

    modem_response_code |= lr1110_modem_set_dm_info_interval( &lr1110, LR1110_MODEM_REPORTING_INTERVAL_IN_HOUR, 1 );

    return modem_response_code;
}

static void print_hex_buffer( const uint8_t* buffer, uint8_t size )
{
    uint8_t newline = 0;

    for( uint8_t i = 0; i < size; i++ )
    {
        if( newline != 0 )
        {
            HAL_DBG_TRACE_PRINTF( "\r\n" );
            newline = 0;
        }

        HAL_DBG_TRACE_PRINTF( "%02X ", buffer[i] );

        if( ( ( i + 1 ) % 16 ) == 0 )
        {
            newline = 1;
        }
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
}

static void print_lorawan_keys( const uint8_t* dev_eui, const uint8_t* join_eui, const uint8_t* app_key, uint32_t pin )
{
    HAL_DBG_TRACE_PRINTF( "DevEui      : %02X", dev_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", dev_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
    HAL_DBG_TRACE_PRINTF( "AppEui      : %02X", join_eui[0] );
    for( int i = 1; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", join_eui[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
#if (USE_SEMTECH_JOIN_SERVER == 1)
    HAL_DBG_TRACE_MSG( "AppKey      : Semtech join server used\r\n" );
#else
    HAL_DBG_TRACE_PRINTF( "AppKey      : %02X", app_key[0] );
    for( int i = 1; i < 16; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "-%02X", app_key[i] );
    }
    HAL_DBG_TRACE_PRINTF( "\r\n" );
#endif

    HAL_DBG_TRACE_PRINTF( "Pin         : %08X\r\n\r\n", pin );
}

static void join_network( void )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    // Starts the join procedure
    modem_response_code = lr1110_modem_join( &lr1110 );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_INFO( "###### ===== JOINING ==== ######\r\n\r\n" );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== JOINING CMD ERROR ==== ######\r\n\r\n" );
    }
}

static void prepare_tx_frame( uint8_t port, uint8_t* tx_frame_buffer, uint8_t* tx_frame_buffer_size, uint8_t max_tx_buffer_size )
{
    switch( port )
    {
    case 2:
    {
        lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
        uint32_t                     charge = 0;

        // Send LR1110 modem charge
        modem_response_code = lr1110_modem_get_charge( &lr1110, &charge );

        if( max_tx_buffer_size < 4 )
        {
            return;
        }
        *tx_frame_buffer_size = 4;

        tx_frame_buffer[0] = charge;
        tx_frame_buffer[1] = charge >> 8;
        tx_frame_buffer[2] = charge >> 16;
        tx_frame_buffer[3] = charge >> 24;
    }
    break;
    default:
        break;
    }
}

static bool send_frame( const uint8_t port, const uint8_t* tx_frame_buffer, const uint8_t tx_frame_buffer_size, const lr1110_modem_uplink_type_t tx_confirmed )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    uint8_t                      tx_max_payload;
    uint32_t                     duty_cycle;

    lr1110_modem_get_duty_cycle_status( &lr1110, &duty_cycle );
    
    if( duty_cycle == 0 )
    {
        lr1110_modem_get_next_tx_max_payload( &lr1110, &tx_max_payload );

        if( tx_frame_buffer_size > tx_max_payload )
        {
            // Send empty frame in order to flush MAC commands
            HAL_DBG_TRACE_MSG( "\r\n APP DATA > MAX PAYLOAD AVAILABLE \r\n" );

            modem_response_code = lr1110_modem_request_tx( &lr1110, port, tx_confirmed, NULL, 0 );
        }
        else
        {
            modem_response_code =
                lr1110_modem_request_tx( &lr1110, port, tx_confirmed, tx_frame_buffer, tx_frame_buffer_size );
        }

        if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
        {
            HAL_DBG_TRACE_INFO( "LR1110 MODEM REQUEST TX \r\n\r\n" );

            return true;
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "LR1110 MODEM REQUEST TX ERROR CMD, modem_response_code : %d \r\n\r\n", modem_response_code );

            return false;
        }
    }
    else
    {
        HAL_DBG_TRACE_INFO( "DUTY CYCLE, NEXT UPLINK AVAIABLE in %d milliseconds \r\n\r\n", duty_cycle );
        
        return false;
    }
}

static void on_led_tx_timer_event( void* context )
{
    timer_stop( &led_tx_timer );
    // Switch LED TX OFF
    leds_off( LED_TX_MASK );
}

static void on_led_rx_timer_event( void* context )
{
    timer_stop( &led_rx_timer );
    // Switch LED RX OFF
    leds_off( LED_RX_MASK );
}

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_modem_board_is_ready( ) == true )
    {
        // System reset
        hal_mcu_reset( );
    }
    else
    {
        lr1110_modem_board_set_ready( true );
    }
}

void lr1110_modem_network_joined( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== JOINED ==== ######\r\n\r\n" );
}

void lr1110_modem_join_fail( void ) { HAL_DBG_TRACE_INFO( "###### ===== JOINED FAIL ==== ######\r\n\r\n" ); }

static void modem_status_to_string( lr1110_modem_status_t modem_status )
{
    HAL_DBG_TRACE_MSG( "Modem status : ");
    
    if( (modem_status & LR1110_LORAWAN_BROWNOUT) == LR1110_LORAWAN_BROWNOUT )
    {
        HAL_DBG_TRACE_MSG( "BROWNOUT ");
    }
    if( (modem_status & LR1110_LORAWAN_CRASH) == LR1110_LORAWAN_CRASH )
    {
        HAL_DBG_TRACE_MSG( "CRASH ");
    }
    if( (modem_status & LR1110_LORAWAN_MUTE) == LR1110_LORAWAN_MUTE )
    {
        HAL_DBG_TRACE_MSG( "MUTE ");
    }
    if( (modem_status & LR1110_LORAWAN_JOINED) == LR1110_LORAWAN_JOINED )
    {
        HAL_DBG_TRACE_MSG( "JOINED ");
    }
    if( (modem_status & LR1110_LORAWAN_SUSPEND) == LR1110_LORAWAN_SUSPEND )
    {
        HAL_DBG_TRACE_MSG( "SUSPEND ");
    }
    if( (modem_status & LR1110_LORAWAN_UPLOAD) == LR1110_LORAWAN_UPLOAD )
    {
        HAL_DBG_TRACE_MSG( "UPLOAD ");
    }
    if( (modem_status & LR1110_LORAWAN_JOINING) == LR1110_LORAWAN_JOINING )
    {
        HAL_DBG_TRACE_MSG( "JOINING ");
    }
    if( (modem_status & LR1110_LORAWAN_STREAM) == LR1110_LORAWAN_STREAM )
    {
        HAL_DBG_TRACE_MSG( "STREAM ");
    }
    
    HAL_DBG_TRACE_MSG( "\r\n\r\n" );
}

void lr1110_modem_alarm( void )
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== LR1110 ALARM ==== ######\r\n\r\n" );

    modem_response_code = lr1110_modem_get_status( &lr1110, &modem_status );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        modem_status_to_string(modem_status);

        if( ((modem_status & LR1110_LORAWAN_BROWNOUT) == LR1110_LORAWAN_BROWNOUT ) ||
            ((modem_status & LR1110_LORAWAN_CRASH) == LR1110_LORAWAN_CRASH ))
        {
            hal_mcu_reset( );
        }
        else if( ((modem_status & LR1110_LORAWAN_MUTE) == LR1110_LORAWAN_MUTE ) ||
            ((modem_status & LR1110_LORAWAN_SUSPEND) == LR1110_LORAWAN_SUSPEND ))
        {
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ((modem_status & LR1110_LORAWAN_JOINED) == LR1110_LORAWAN_JOINED ) ||
            ((modem_status & LR1110_LORAWAN_STREAM) == LR1110_LORAWAN_STREAM ) ||
            ((modem_status & LR1110_LORAWAN_UPLOAD) == LR1110_LORAWAN_UPLOAD ))
        {
            device_state = DEVICE_STATE_SEND;
        }
        else if( (modem_status & LR1110_LORAWAN_JOINING) == LR1110_LORAWAN_JOINING )
        {
            // Network not joined yet. Wait
            device_state = DEVICE_STATE_CYCLE;
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "Unknow modem status %d\r\n\r\n",modem_status );
            device_state = DEVICE_STATE_CYCLE;
        }
    }
}

void lr1110_modem_tx_done( lr1110_modem_tx_done_event_t status )
{
    if( status != LR1110_MODEM_TX_ERROR )
    {
        HAL_DBG_TRACE_INFO( "###### ===== UPLINK FRAME %lu ==== ######\r\n\r\n", uplink_cnt++ );

        HAL_DBG_TRACE_MSG( "CLASS       : A\r\n" );
        HAL_DBG_TRACE_PRINTF( "TX PORT     : %d\r\n", app_port );

        if( app_data_size != 0 )
        {
            HAL_DBG_TRACE_MSG( "TX DATA     : " );
            if( is_tx_confirmed )
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

        leds_on( LED_TX_MASK );
        timer_start( &led_tx_timer );
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "###### ===== TX ERROR ==== ######\r\n\r\n" );
    }
}

static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload,
                                    uint8_t size )
{
    HAL_DBG_TRACE_INFO( "\r\n###### ===== DOWNLINK FRAME %lu ==== ######\r\n\r\n", downlink_cnt++ );

    HAL_DBG_TRACE_PRINTF( "RX WINDOW   : %d\r\n", (flags & 0x03) );

    HAL_DBG_TRACE_PRINTF( "RX PORT     : %d\r\n", port );

    if( size != 0 )
    {
        HAL_DBG_TRACE_MSG( "RX DATA     : " );
        print_hex_buffer( payload, size );
    }

    HAL_DBG_TRACE_PRINTF( "RX RSSI     : %d\r\n", rssi );
    HAL_DBG_TRACE_PRINTF( "RX SNR      : %d\r\n\r\n", snr );

    leds_on( LED_RX_MASK );
    timer_start( &led_rx_timer );
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

static void lr1110_modem_set_conf( uint8_t infor_tag )
{
    HAL_DBG_TRACE_INFO( "###### ===== MODEM SET CONF %02X ==== ######\r\n\r\n", infor_tag );
}

static void lr1110_modem_stream_done( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== STREAM DONE nb %d ==== ######\r\n\r\n" );
}

static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t  alc_sync_state )
{
    HAL_DBG_TRACE_INFO( "###### ===== APPLICATION LAYER CLOCK SYNC EVENT ==== ######\r\n\r\n" );
    
    if( alc_sync_state == LR1110_MODEM_ALC_SYNC_SYNCHRONIZED )
    {
        HAL_DBG_TRACE_MSG("CLOCK SYNC STATE SYNCHRONIZED\r\n");
    }
    else
    {

        HAL_DBG_TRACE_MSG("CLOCK SYNC STATE DESYNCHRONIZED\r\n");
    }
}

static void lr1110_modem_adr_mobile_to_static( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== ADR HAS SWITCHED FROM MOBILE TO STATIC ==== ######\r\n\r\n" );
}

static void lr1110_modem_new_link_adr( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== NEW LINK ADR ==== ######\r\n\r\n" );
}

static void lr1110_modem_no_event( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== NO EVENT ==== ######\r\n\r\n" );
}


/* --- EOF ------------------------------------------------------------------ */
