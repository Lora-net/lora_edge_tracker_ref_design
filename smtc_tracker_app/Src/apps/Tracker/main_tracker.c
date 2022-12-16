/*!
 * @ingroup   apps_tracker
 * @file      main_tracker.c
 *
 * @brief     lr1110 Modem-E Tracker Application implementation
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

#include "main_tracker.h"
#include "lorawan_commissioning.h"
#include "lr1110_tracker_board.h"
#include "wifi_scan.h"
#include "gnss_scan.h"
#include "tracker_utility.h"
#include "ble_thread.h"
#include "apps_utilities.h"

/*!
 * @addtogroup apps_tracker
 * LR1110 Modem-E Tracker Application
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
 * @brief Force a new tracker context in flash memory
 */
#define FORCE_NEW_TRACKER_CONTEXT 0

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

/*!
 * @brief ADR custom list when ADR is set to custom
 */
uint8_t adr_custom_list[16] = { 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
                                0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01 };

/*!
 * @brief Timer to handle the state of LED TX
 */
static timer_event_t led_tx_timer;

/*!
 * @brief Timer to handle the state of LED RX
 */
static timer_event_t led_rx_timer;

/*!
 * @brief Timer to handle the hall effect sensor management in airplane mode
 */
static timer_event_t hall_effect_sensor_airplane_timer;

/*!
 * @brief Device states
 */
static enum device_state_e {
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_COLLECT_DATA,
    DEVICE_START_BLE,
    DEVICE_STATE_SLEEP
} device_state = DEVICE_STATE_INIT;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Lorawan default init
 *
 * @param [in] region LoRaWAN region to use \ref lr1110_modem_regions_t
 * @param [in] lorawan_class LoRaWAN class to use \ref lr1110_modem_classes_t
 *
 * @returns Operation status
 */
static lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class );

/*!
 * @brief Executes the network Join request
 */
static void tracker_app_join_network( void );

/*!
 * @brief Adapt the ADR
 */
static void tracker_app_adapt_adr( void );

/*!
 * @brief   add payload of the frame
 *
 * @returns  [true : payload could be add, false : error]
 */
static bool tracker_app_add_payload_in_streaming_fifo( const uint8_t* payload, uint16_t len );

/*!
 * @brief   Check if the next scan is possible
 *
 * @returns  [true : scan is possible, false : scan is not possible]
 */
static bool tracker_app_is_next_scan_possible( void );

/*!
 * @brief   Check if the tracker is in static mode
 *
 * @returns  [true : tracker is static, false : tracker in movement]
 */
static bool tracker_app_is_tracker_in_static_mode( void );

/*!
 * @brief   return is the used region uses duty cycle or not
 *
 * @param [in] region LoRaWAN region to use \ref lr1110_modem_regions_t
 *
 * @returns  [true : uses duty cycle, false : don't use duty cycle]
 */
static bool tracker_app_is_region_use_duty_cycle( lr1110_modem_regions_t region );

/*!
 * @brief Parse the received downlink
 *
 * @param [in] port LoRaWAN port
 * @param [in] payload Payload Buffer
 * @param [in] size Payload size
 */
static void tracker_app_parse_frame( uint8_t port, const uint8_t* payload, uint8_t size );

/*!
 * @brief Update in flash context the accumulated charge if has changed
 *
 * @param [in] modem_charge the actual modem charge
 */
static void tracker_app_store_new_acculated_charge( uint32_t modem_charge );

/*!
 * @brief build results payload in TLV format and stream it
 *
 * @param [in] send_complete_sensors add all sensors value to the stream (true) or not (false)
 */
static void tracker_app_build_and_stream_payload( bool send_complete_sensors );

/*!
 * @brief reset the scan results
 */
static void tracker_app_reset_scan_results( void );

/*!
 * @brief build tracker settings payload in TLV format and stream it
 *
 * @param [in] buffer Buffer containing the tracker settings
 * @param [in] len Len of the buffer
 */
static void tracker_app_build_and_stream_tracker_settings( const uint8_t* buffer, uint8_t len );

/*!
 * @brief launch the complete scan according to the scan priority defined by the user
 *
 * @param [in] scan_piority the scan priority defined by the user \ref tracker_scan_priority_t
 *
 * @returns [true : has scan data, false : don't don't have scan data]
 */
static bool tracker_app_start_scan( const tracker_scan_priority_t scan_piority );

/*!
 * @brief Function executed on Led TX Timeout event
 */
static void on_led_tx_timer_event( void* context );

/*!
 * @brief Function executed on Led RX Timeout event
 */
static void on_led_rx_timer_event( void* context );

/*!
 * @brief Function executed on hall effect shutdown event
 */
static void on_hall_effect_shutdown_timer_event( void* context );

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
 * @param [in] tag     \ref lr1110_modem_event_setconf_tag_t
 */
static void lr1110_modem_set_conf( lr1110_modem_event_setconf_tag_t tag );

/*!
 * @brief Stream done event callback
 */
static void lr1110_modem_stream_done( void );

/*!
 * @brief Time updated by application layer clock synchronisation event callback
 */
static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state );

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

/*!
 * @brief GNSS default init
 * @param [in] gnss_settings GNSS settings used for the scan \ref gnss_settings_t
 */
static lr1110_modem_response_code_t tracker_gnss_init( const gnss_settings_t* gnss_settings );

/*!
 * @brief Get and update in flash context the gnss assistance position if has changed
 */
static void tracker_gnss_store_new_assistance_position( void );

/*!
 * @brief Return the number max of sattelites to scan according to the LoRaWAN parameters
 *
 * @returns the number max of sattelites to scan
 */
static uint8_t tracker_gnss_get_next_nb_sat( void );

/*!
 * @brief Run and call the necessary function for the GNSS scan
 *
 * @param [in] gnss_settings GNSS settings used for the scan \ref gnss_settings_t
 * @param [in] antenna GNSS antenna selection for the scan \ref antenna_t
 * @param [out] capture_result Structure containing the capture result, \ref gnss_scan_single_result_t
 */
static void tracker_gnss_run_scan( const gnss_settings_t* gnss_settings, antenna_t antenna,
                                   gnss_scan_single_result_t* capture_result );

/*!
 * @brief Run and call the necessary functions for the Wi-Fi scan
 *
 * @param [in] wifi_settings Wi-Fi settings used for the scan \ref wifi_settings_t
 * @param [out] wifi_result Wi-Fi result structure where are stored the results
 */
static void tracker_wifi_run_scan( const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result );

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
    lr1110_modem_event_callback_t lr1110_modem_event_callback;
    int32_t                       voltage_drop          = 0;
    uint32_t                      voltage_recovery_time = 0;

    uint8_t dev_eui[LORAWAN_DEVICE_EUI_LEN] = LORAWAN_DEVICE_EUI;
    uint8_t join_eui[LORAWAN_JOIN_EUI_LEN]  = LORAWAN_JOIN_EUI;
    uint8_t app_key[LORAWAN_APP_KEY_LEN]    = LORAWAN_APP_KEY;

    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    HAL_DBG_TRACE_MSG( "\r\n" );
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E Tracker demo application ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\r\n\r\n", TRACKER_MAJOR_APP_VERSION, TRACKER_MINOR_APP_VERSION,
                          TRACKER_SUB_MINOR_APP_VERSION );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.reset                 = lr1110_modem_reset_event;
    lr1110_modem_event_callback.alarm                 = lr1110_modem_alarm;
    lr1110_modem_event_callback.joined                = lr1110_modem_network_joined;
    lr1110_modem_event_callback.join_fail             = lr1110_modem_join_fail;
    lr1110_modem_event_callback.down_data             = lr1110_modem_down_data;
    lr1110_modem_event_callback.set_conf              = lr1110_modem_set_conf;
    lr1110_modem_event_callback.mute                  = lr1110_modem_mute;
    lr1110_modem_event_callback.gnss_scan_done        = lr1110_modem_gnss_scan_done;
    lr1110_modem_event_callback.wifi_scan_done        = lr1110_modem_wifi_scan_done;
    lr1110_modem_event_callback.stream_done           = lr1110_modem_stream_done;
    lr1110_modem_event_callback.time_updated_alc_sync = lr1110_modem_time_updated_alc_sync;
    lr1110_modem_event_callback.adr_mobile_to_static  = lr1110_modem_adr_mobile_to_static;
    lr1110_modem_event_callback.new_link_adr          = lr1110_modem_new_link_adr;
    lr1110_modem_event_callback.no_event              = lr1110_modem_no_event;

    if( lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LR1110 BOARD INIT FAIL ==== ######\r\n\r\n" );

        HAL_DBG_TRACE_INFO( "Something goes wrong with the LR1110 firmware, stay in BLE mode and update it\n\r" );
        tracker_ctx.has_lr1110_firmware = false;
        tracker_restore_app_ctx( );
        start_ble_thread( 0 );
    }
    else
    {
        tracker_ctx.has_lr1110_firmware = true;
    }

    /* LR1110 modem-e version */
    lr1110_modem_get_version( &lr1110, &tracker_ctx.modem_version );
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E VERSION ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "LORAWAN     : %#04X\r\n", tracker_ctx.modem_version.lorawan );
    HAL_DBG_TRACE_PRINTF( "FIRMWARE    : %#02X\r\n", tracker_ctx.modem_version.firmware );
    HAL_DBG_TRACE_PRINTF( "BOOTLOADER  : %#02X\r\n", tracker_ctx.modem_version.bootloader );

    /* Store or restore the Tracker context */
    if( ( tracker_restore_app_ctx( ) != SUCCESS ) || ( FORCE_NEW_TRACKER_CONTEXT == 1 ) )
    {
#if( USE_PRODUCTION_KEYS == 1 )
        /* When the production keys are used, DevEUI = ChipEUI and JoinEUI is the one defined in lorawan_comissioning.h
         */
        modem_response_code = lr1110_modem_get_chip_eui( &lr1110, dev_eui );
#endif
        /* Init the LoRaWAN keys set in Commissioning_tracker_ctx.h or using the production keys and init the global
         * context */
        tracker_init_app_ctx( dev_eui, join_eui, app_key, true );

        /* Init the tracker internal log context */
        tracker_init_internal_log_ctx( );
    }
    else
    {
        /* Restore the tracker internal log context */
        if( tracker_restore_internal_log_ctx( ) != SUCCESS )
        {
            tracker_init_internal_log_ctx( );
        }

        /* Set the restored LoRaWAN Keys */
        memcpy( dev_eui, tracker_ctx.dev_eui, LORAWAN_DEVICE_EUI_LEN );
        memcpy( join_eui, tracker_ctx.join_eui, LORAWAN_JOIN_EUI_LEN );
        memcpy( app_key, tracker_ctx.app_key, LORAWAN_APP_KEY_LEN );
    }

    /* LoRaWAN configuration */
    if( lorawan_init( tracker_ctx.lorawan_region, LORAWAN_CLASS_USED ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== LORAWAN INIT ERROR ==== ######\r\n\r\n" );
    }

    if( tracker_gnss_init( &tracker_ctx.gnss_settings ) != LR1110_MODEM_RESPONSE_CODE_OK )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== GNSS INIT ERROR ==== ######\r\n\r\n" );
    }

    /* Set Keys */
    modem_response_code = lr1110_modem_set_dev_eui( &lr1110, dev_eui );
    modem_response_code = lr1110_modem_set_join_eui( &lr1110, join_eui );
    /* do a derive keys to get the pin code */
    lr1110_modem_derive_keys( &lr1110 );
    lr1110_modem_get_pin( &lr1110, &tracker_ctx.lorawan_pin );
    lr1110_modem_get_chip_eui( &lr1110, tracker_ctx.chip_eui );

    /* Init the software watchdog */
    hal_mcu_init_software_watchdog( tracker_ctx.app_scan_interval * 3 );

    /* Init Leds timer */
    timer_init( &led_tx_timer, on_led_tx_timer_event );
    timer_set_value( &led_tx_timer, LED_PERIOD_MS );
    timer_init( &led_rx_timer, on_led_rx_timer_event );
    timer_set_value( &led_rx_timer, LED_PERIOD_MS );

    /* Start BLE advertisement for 30s */
    start_ble_thread( TRACKER_ADV_TIMEOUT_MS );

    /* Init tracker context volatile parameters */
    tracker_ctx.has_date                   = false;
    tracker_ctx.accelerometer_move_history = 1;
    tracker_ctx.stream_done                = true;
    tracker_ctx.voltage                    = hal_mcu_get_vref_level( );
    tracker_ctx.reset_cnt_sent             = false;
    tracker_ctx.system_sanity_check        = ( tracker_system_sanity_check_mask_t ) 0;

    /* Check if the batteries are too low, if yes switch the tracker in airplane mode, \note if this test is moved
     * elsewhere in this app the TRACKER_BOARD_MAX_VOLTAGE_RECOVERY_TIME value should be evaluate again  */
    modem_response_code = lr1110_tracker_board_measure_battery_drop( &lr1110, &voltage_drop, &voltage_recovery_time );
    if( ( voltage_recovery_time > TRACKER_BOARD_MAX_VOLTAGE_RECOVERY_TIME ) &&
        ( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK ) &&
        ( tracker_ctx.accumulated_charge > TRACKER_BOARD_BATTERY_CAPACITY_80_PERCENT ) )
    {
        HAL_DBG_TRACE_ERROR( "###### ===== BATTERIES LOW, STAY IN AIRPLANE MODE ==== ######\r\n\r\n" );
        tracker_ctx.airplane_mode = true;

        leds_blink( LED_ALL_MASK, 500, 5, true );
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

            if( tracker_ctx.use_semtech_join_server == true )
            {
                modem_response_code = lr1110_modem_derive_keys( &lr1110 );
            }
            else
            {
                modem_response_code = lr1110_modem_set_app_key( &lr1110, app_key );
            }

            device_state = DEVICE_STATE_JOIN;

            break;
        }

        case DEVICE_STATE_JOIN:
        {
            /* Display used keys */
            print_lorawan_keys( dev_eui, join_eui, app_key, tracker_ctx.lorawan_pin,
                                tracker_ctx.use_semtech_join_server );

            if( tracker_ctx.airplane_mode == false )
            {
                tracker_app_join_network( );
            }
            else
            {
                HAL_DBG_TRACE_MSG( "TRACKER IN AIRPLANE MODE\r\n\r\n" );

                /* Configure the timer managing the hall effect sensor in airplane mode */
                timer_init( &hall_effect_sensor_airplane_timer, on_hall_effect_shutdown_timer_event );
                timer_set_value( &hall_effect_sensor_airplane_timer, TRACKER_AIRPLANE_HALL_EFFECT_TIMEOUT_MS );

                /* Stop Hall Effect sensors while the tracker is static in airplane mode */
                lr1110_tracker_board_hall_effect_enable( false );

                /* Reset the accelerometer_irq1_state from the LIS2DE12 */
                is_accelerometer_detected_moved( );
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_COLLECT_DATA:
        {
            /* Create a movevment history on 8 bits and update this value only if the stream is done */
            if( tracker_ctx.stream_done == true )
            {
                tracker_ctx.accelerometer_move_history =
                    ( tracker_ctx.accelerometer_move_history << 1 ) + is_accelerometer_detected_moved( );
            }

            /* Check if scan can be launched */
            if( tracker_app_is_next_scan_possible( ) == true )
            {
                lr1110_modem_lorawan_state_t lorawan_state;
                int32_t                      duty_cycle;
                bool                         send_complete_sensors = false;

                /* Reload the software watchdog */
                hal_mcu_reset_software_watchdog( );

                /* Reset previous scan results */
                tracker_app_reset_scan_results( );

                /* Adapt the ADR following the acceleromer movement */
                tracker_app_adapt_adr( );

                /* Reset flag and counter */
                if( tracker_ctx.send_alive_frame == true )
                {
                    send_complete_sensors = true;
                }
                tracker_ctx.send_alive_frame = false;
                tracker_ctx.next_frame_ctn   = 0;

                /* Led start for user notification */
                leds_on( LED_TX_MASK );
                timer_start( &led_tx_timer );

                /* Start Hall Effect sensors while the tracker moves */
                lr1110_tracker_board_hall_effect_enable( true );

                /* Launch the scan according to the scan prioriy */
                if( tracker_app_start_scan( tracker_ctx.scan_priority ) == false )
                {
                    HAL_DBG_TRACE_MSG( "No scan results good enough, send complete sensors values\r\n" );
                    send_complete_sensors = true;
                }

                /*  SENSORS DATA */
                HAL_DBG_TRACE_INFO( "*** sensors collect ***\n\r\n\r" );

                /* Acceleration */
                acc_read_raw_data( );
                tracker_ctx.accelerometer_x = acc_get_raw_x( );
                tracker_ctx.accelerometer_y = acc_get_raw_y( );
                tracker_ctx.accelerometer_z = acc_get_raw_z( );
                HAL_DBG_TRACE_PRINTF( "Acceleration [mg]: X=%4.2f mg | Y=%4.2f mg | Z=%4.2f mg \r\n",
                                      ( double ) tracker_ctx.accelerometer_x, ( double ) tracker_ctx.accelerometer_y,
                                      ( double ) tracker_ctx.accelerometer_z );

                /* Move history */
                HAL_DBG_TRACE_PRINTF( "Move history : %d\r\n", tracker_ctx.accelerometer_move_history );

                /* Temperature */
                tracker_ctx.temperature = hal_mcu_get_temperature( ) * 100;
                HAL_DBG_TRACE_PRINTF( "Temperature : %d *C\r\n", tracker_ctx.temperature / 100 );

                /* Modem charge */
                lr1110_modem_get_charge( &lr1110, &tracker_ctx.charge );
                HAL_DBG_TRACE_PRINTF( "Charge value : %d mAh\r\n", tracker_ctx.charge );
                tracker_app_store_new_acculated_charge( tracker_ctx.charge );
                HAL_DBG_TRACE_PRINTF( "Accumulated charge value : %d mAh\r\n", tracker_ctx.accumulated_charge );

                /* Board voltage charge */
                lr1110_modem_get_lorawan_state( &lr1110, &lorawan_state );
                if( lorawan_state == LR1110_MODEM_LORAWAN_IDLE )
                {
                    tracker_ctx.voltage = hal_mcu_get_vref_level( );
                    HAL_DBG_TRACE_PRINTF( "Board voltage : %d mV\r\n", tracker_ctx.voltage );
                }
                else
                {
                    HAL_DBG_TRACE_WARNING( "TX ongoing don't collect the board voltage\r\n" );
                }

                if( tracker_ctx.internal_log_enable )
                {
                    HAL_DBG_TRACE_PRINTF( "Log results in the Internal Log memory\r\n", tracker_ctx.voltage );
                    tracker_store_internal_log( );
                    HAL_DBG_TRACE_PRINTF( "Internal Log memory space remaining: %d %%\r\n",
                                          tracker_get_remaining_memory_space( ) );
                }

                /* Build the payload and stream it if we have enough time */
                lr1110_modem_get_duty_cycle_status( &lr1110, &duty_cycle );

                if( ( tracker_app_is_region_use_duty_cycle( tracker_ctx.lorawan_region ) == false ) ||
                    ( ( duty_cycle >= TRACKER_DUTY_CYCLE_THRESHOLD ) &&
                      ( ( tracker_app_is_region_use_duty_cycle( tracker_ctx.lorawan_region ) == true ) ) ) )
                {
                    HAL_DBG_TRACE_PRINTF( "Remaining time (%d ms) to send data\r\n", duty_cycle );
                    tracker_app_build_and_stream_payload( send_complete_sensors );
                }
                else
                {
                    HAL_DBG_TRACE_WARNING(
                        "Not enough remaining time (%d ms) to send data, results are just logged (if enable)\r\n",
                        duty_cycle );
                }

                /* Send tracker setting if it has been asked by the application server */
                if( tracker_ctx.tracker_settings_payload_len > 0 )
                {
                    tracker_app_build_and_stream_tracker_settings( tracker_ctx.tracker_settings_payload,
                                                                   tracker_ctx.tracker_settings_payload_len );
                    tracker_ctx.tracker_settings_payload_len = 0;
                }

                device_state = DEVICE_STATE_SEND;
            }
            else
            {
                if( tracker_ctx.stream_done == true )
                {
                    /* Stop Hall Effect sensors while the tracker is static */
                    lr1110_tracker_board_hall_effect_enable( false );

                    if( tracker_ctx.next_frame_ctn >=
                        ( tracker_ctx.app_keep_alive_frame_interval / tracker_ctx.app_scan_interval ) )
                    {
                        HAL_DBG_TRACE_INFO( "Send a keep alive frame next time\r\n" );
                        tracker_ctx.send_alive_frame = true;
                    }
                    else
                    {
                        HAL_DBG_TRACE_PRINTF(
                            "Device is static next keep alive frame in %d sec\r\n",
                            ( ( tracker_ctx.app_keep_alive_frame_interval / tracker_ctx.app_scan_interval ) -
                              tracker_ctx.next_frame_ctn ) *
                                ( tracker_ctx.app_scan_interval / 1000 ) );
                        tracker_ctx.next_frame_ctn++;
                    }
                    device_state = DEVICE_STATE_CYCLE;
                }
                else
                {
                    device_state = DEVICE_STATE_SEND;
                }
            }

            break;
        }
        case DEVICE_STATE_SEND:
        {
            if( tracker_ctx.stream_done == false )
            {
                lr1110_modem_stream_status_t stream_status;

                /* Stream previous payload if it's not terminated */
                lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
                HAL_DBG_TRACE_PRINTF( "Streaming ongoing %d bytes remaining %d bytes free \r\n", stream_status.pending,
                                      stream_status.free );
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            /* Reload the software watchdog */
            hal_mcu_reset_software_watchdog( );

            device_state = DEVICE_STATE_SLEEP;

            /* Schedule next packet transmission */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, tracker_ctx.app_scan_interval / 1000 );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            HAL_DBG_TRACE_PRINTF( "Set alarm timer : %d s\r\n\r\n", tracker_ctx.app_scan_interval / 1000 );

            break;
        }
        case DEVICE_START_BLE:
        {
            /* Stop the running timer if in airplane mode */
            if( tracker_ctx.airplane_mode == true )
            {
                timer_stop( &hall_effect_sensor_airplane_timer );
            }

            /* Stop the LR1110 modem alarm */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, 0 );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            /* Start the BLE thread*/
            start_ble_thread( TRACKER_ADV_TIMEOUT_MS );

            if( tracker_ctx.airplane_mode == true )
            {
                /* Stop Hall Effect sensors while the tracker is static in airplane mode */
                lr1110_tracker_board_hall_effect_enable( false );

                /* Reset the accelerometer_irq1_state from the LIS2DE12 */
                is_accelerometer_detected_moved( );
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            if( ( get_hall_effect_irq_state( ) == true ) || ( get_usr_button_irq_state( ) == true ) )
            {
                clear_usr_button_irq_state( );
                clear_hall_effect_irq_state( );
                device_state = DEVICE_START_BLE;
            }
            else
            {
                /* go in low power */
                if( lr1110_tracker_board_read_event_line( &lr1110 ) == false )
                {
                    hal_mcu_low_power_handler( );
                }

                /* Wake up from static mode thanks the accelerometer ? */
                if( ( get_accelerometer_irq1_state( ) == true ) &&
                    ( tracker_app_is_tracker_in_static_mode( ) == true ) )
                {
                    /* Stop the LR1110 current modem alarm */
                    do
                    {
                        modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, 0 );
                    } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

                    device_state = DEVICE_COLLECT_DATA;
                }

                /* Wake up thanks the accelerometer and in airplane mode ? */
                if( ( tracker_ctx.airplane_mode == true ) && ( is_accelerometer_detected_moved( ) == true ) )
                {
                    /* Start Hall Effect sensors while the tracker moves */
                    lr1110_tracker_board_hall_effect_enable( true );

                    timer_reset( &hall_effect_sensor_airplane_timer );
                    HAL_DBG_TRACE_PRINTF( "Start hall effect sensor for 60s\r\n" );
                }
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

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER WI-FI FUNCTION TYPES --------------------------------------------
 */

static void tracker_wifi_run_scan( const wifi_settings_t* wifi_settings, wifi_scan_selected_result_t* wifi_result )
{
    static wifi_scan_all_results_t capture_result;

    /* Turn on the 2G4 SPDT and set it into the right direction */
    spdt_2g4_on( );
    set_wifi_antenna( );

    if( wifi_execute_scan( &lr1110, wifi_settings, &capture_result ) == WIFI_SCAN_SUCCESS )
    {
        lr1110_modem_display_wifi_scan_results( &capture_result );

        lr1110_modem_wifi_scan_select_results( &capture_result, wifi_result );

        if( wifi_result->nbr_results != 0 )
        {
            /* Update the system_sanity_check bit field */
            tracker_ctx.system_sanity_check |= TRACKER_WIFI_SCAN_SUCCESSFUL_ONCE;
        }
    }
    else
    {
        HAL_DBG_TRACE_ERROR( "Wi-Fi Scan error\n\r" );
        wifi_result->nbr_results = 0;  // reset MAC addr detected
    }

    /* Turn off the 2G4 SPDT */
    spdt_2g4_off( );
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER GNSS FUNCTION TYPES ---------------------------------------------
 */

static uint8_t tracker_gnss_get_next_nb_sat( void )
{
    uint8_t next_tx_max_payload = 0;

    /* Adapt the nb_sat value in function of the number of byte available for the next lorawan payload */
    lr1110_modem_get_next_tx_max_payload( &lr1110, &next_tx_max_payload );

    if( next_tx_max_payload <= 51 )
    {
        return 8;
    }
    else
    {
        return 12;
    }
}

static void tracker_gnss_run_scan( const gnss_settings_t* gnss_settings, antenna_t antenna,
                                   gnss_scan_single_result_t* capture_result )
{
    uint8_t gnss_status;

    gnss_status = gnss_scan_execute( &lr1110, antenna, gnss_settings, capture_result );

    if( gnss_status == GNSS_SCAN_SUCCESS )
    {
        gnss_scan_display_results( capture_result );

        if( capture_result->nb_detected_satellites != 0 )
        {
            /* Update the system_sanity_check bit field */
            tracker_ctx.system_sanity_check |= TRACKER_GNSS_SCAN_SUCCESSFUL_ONCE;
        }
    }
    else
    {
        if( gnss_status == GNSS_SCAN_NO_TIME )
        {
            HAL_DBG_TRACE_ERROR( "GNSS Scan error: No time\n\r" );
        }
        else
        {
            HAL_DBG_TRACE_ERROR( "GNSS Scan error\n\r" );
        }
    }
}

static lr1110_modem_response_code_t tracker_gnss_init( const gnss_settings_t* gnss_settings )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    modem_response_code = lr1110_modem_gnss_set_assistance_position( &lr1110, &gnss_settings->assistance_position );

    return modem_response_code;
}

static void tracker_gnss_store_new_assistance_position( void )
{
    lr1110_modem_gnss_solver_assistance_position_t assistance_position;
    float                                          latitude_dif, longitude_dif;

    lr1110_modem_gnss_read_assistance_position( &lr1110, &assistance_position );

    latitude_dif  = fabs( assistance_position.latitude - tracker_ctx.gnss_settings.assistance_position.latitude );
    longitude_dif = fabs( assistance_position.longitude - tracker_ctx.gnss_settings.assistance_position.longitude );

    /* Store the new assistance position only if the difference is greater than the conversion error */
    if( ( latitude_dif > ( float ) 0.03 ) || ( longitude_dif > ( float ) 0.03 ) )
    {
        HAL_DBG_TRACE_MSG( "New assistance position stored\r\n" );

        tracker_ctx.gnss_settings.assistance_position.latitude  = assistance_position.latitude;
        tracker_ctx.gnss_settings.assistance_position.longitude = assistance_position.longitude;

        tracker_store_app_ctx( );
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER LORAWAN FUNCTION TYPES ------------------------------------------
 */

static lr1110_modem_response_code_t lorawan_init( lr1110_modem_regions_t region, lr1110_modem_classes_t lorawan_class )
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
        
        /* limit TX power to 14 dBm */
        modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, -6 );
        
        break;
    }
    case LR1110_LORAWAN_REGION_AS923_GRP1:
    {
        if( LORAWAN_COUNTRY_JAPAN == 1 )
        {
            HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
            /* Activate LBT for 5ms before each transmission with a threshold at -80 dBm */
            modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -80, 5, 125000 );

            /* limit TX power to 9 dBm */
            modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, -7 );
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
        
        /* limit TX power to 14 dBm */
        modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, -6 );
        
        break;
    }
    case LR1110_LORAWAN_REGION_KR920:
    {
        HAL_DBG_TRACE_MSG( "LBT         : ACTIVATE LBT\r\n" );
        HAL_DBG_TRACE_MSG( "REGION      : KR920\r\n\r\n" );

        /* Activate LBT for 5ms before each transmission with a threshold at -65 dBm */
        modem_response_code |= lr1110_modem_activate_lbt( &lr1110, LR1110_MODEM_LBT_MODE_ENABLE, -65, 5, 125000 );

        /* limit TX power to 8 dBm */
        modem_response_code |= lr1110_modem_set_tx_power_offset( &lr1110, -6 );

        break;
    }
    case LR1110_LORAWAN_REGION_RU864:
    {
        HAL_DBG_TRACE_MSG( "REGION      : RU864\r\n\r\n" );
        break;
    }
    default:
        HAL_DBG_TRACE_ERROR( "No suported region selected\r\n\r\n" );
        break;
    }

    /* Set DM info field */
    dm_info_fields.dm_info_field[0] = LR1110_MODEM_DM_INFO_TYPE_CHARGE;
    dm_info_fields.dm_info_field[1] = LR1110_MODEM_DM_INFO_TYPE_GNSS_ALMANAC_STATUS;
    dm_info_fields.dm_info_field[2] = LR1110_MODEM_DM_INFO_TYPE_TEMPERATURE;
    dm_info_fields.dm_info_length   = 3;

    modem_response_code |= lr1110_modem_set_dm_info_field( &lr1110, &dm_info_fields );

    modem_response_code |= lr1110_modem_set_dm_info_interval( &lr1110, LR1110_MODEM_REPORTING_INTERVAL_IN_DAY, 1 );

    modem_response_code |= lr1110_modem_set_alc_sync_mode( &lr1110, LR1110_MODEM_ALC_SYNC_MODE_ENABLE );

    return modem_response_code;
}

/*
 * -------------------------------------------------------------------------
 * --- TRACKER APP FUNCTION TYPES ------------------------------------------
 */

static bool tracker_app_start_scan( const tracker_scan_priority_t scan_piority )
{
    bool has_scan_result = true;

    /* Activate the partial low power mode */
    hal_mcu_partial_sleep_enable( true );

    /* Timestamp scan */
    tracker_ctx.timestamp = lr1110_tracker_board_get_systime_from_gps( &lr1110 );

    switch( scan_piority )
    {
    case TRACKER_GNSS_PRIORITY:
    {
        HAL_DBG_TRACE_INFO( "*** Tracker scan GNSS priority *** \n\r\n\r" );

        /*  GNSS SCAN */
        if( tracker_ctx.gnss_settings.enabled == true )
        {
            HAL_DBG_TRACE_INFO( "*** Gnss Scan ***\n\r\n\r" );

            /* Check if a new assistance position is available */
            tracker_gnss_store_new_assistance_position( );

            if( tracker_ctx.has_date == true )
            {
                /* Get the next nb sat value */
                tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat( );

                /* Run scan on the PCB antenna if enable */
                if( ( tracker_ctx.gnss_antenna_sel & GNSS_PCB_ANTENNA ) == GNSS_PCB_ANTENNA )
                {
                    tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PCB_ANTENNA,
                                           &tracker_ctx.pcb_gnss_scan_result );
                }

                /* Run scan on the patch antenna if enable */
                if( ( tracker_ctx.gnss_antenna_sel & GNSS_PATCH_ANTENNA ) == GNSS_PATCH_ANTENNA )
                {
                    tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PATCH_ANTENNA,
                                           &tracker_ctx.patch_gnss_scan_result );
                }

                if( tracker_ctx.gnss_antenna_sel == ( GNSS_PATCH_ANTENNA | GNSS_PCB_ANTENNA ) )
                {
                    gnss_scan_determine_best_nav_message( &tracker_ctx.pcb_gnss_scan_result,
                                                          &tracker_ctx.patch_gnss_scan_result );
                }

                /* Update the last_nb_detected_satellites field */
                tracker_ctx.last_nb_detected_satellites =
                    SMTC_MAX( tracker_ctx.pcb_gnss_scan_result.nb_detected_satellites,
                              tracker_ctx.patch_gnss_scan_result.nb_detected_satellites );
            }
            else
            {
                HAL_DBG_TRACE_MSG( "Wait application layer clock synchronisation\r\n\r\n" );
            }
        }

        /*  WIFI SCAN */
        if( tracker_ctx.wifi_settings.enabled == true )
        {
            /* Proceed to a Wi-Fi scan only if no GNSS data is available */
            if( ( tracker_ctx.gnss_settings.enabled == false ) ||
                ( ( tracker_ctx.pcb_gnss_scan_result.is_valid_nav_message == false ) &&
                  ( tracker_ctx.patch_gnss_scan_result.is_valid_nav_message == false ) ) )
            {
                HAL_DBG_TRACE_INFO( "*** Wi-Fi Scan *** \n\r\n\r" );

                tracker_wifi_run_scan( &tracker_ctx.wifi_settings, &tracker_ctx.wifi_result );

                tracker_ctx.last_nb_detected_mac_address = tracker_ctx.wifi_result.nbr_results;
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "GNSS scan good enough, drop Wi-Fi scan\r\n" );
            }
        }

        break;
    }
    case TRACKER_WIFI_PRIORITY:
    {
        HAL_DBG_TRACE_INFO( "*** Tracker scan Wi-Fi priority *** \n\r\n\r" );

        /*  WIFI SCAN */
        if( tracker_ctx.wifi_settings.enabled == true )
        {
            HAL_DBG_TRACE_INFO( "*** Wi-Fi Scan *** \n\r\n\r" );

            tracker_wifi_run_scan( &tracker_ctx.wifi_settings, &tracker_ctx.wifi_result );

            tracker_ctx.last_nb_detected_mac_address = tracker_ctx.wifi_result.nbr_results;
        }

        /*  GNSS SCAN */
        if( tracker_ctx.gnss_settings.enabled == true )
        {
            /* Proceed to a GNSS scan only if no Wi-Fi data is available */
            if( ( tracker_ctx.wifi_settings.enabled == false ) || ( tracker_ctx.wifi_result.nbr_results < 2 ) )
            {
                HAL_DBG_TRACE_INFO( "*** Gnss Scan ***\n\r\n\r" );

                /* Check if a new assistance position is available */
                tracker_gnss_store_new_assistance_position( );

                if( tracker_ctx.has_date == true )
                {
                    /* Get the next nb sat value */
                    tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat( );

                    /* Run scan on the PCB antenna if enable */
                    if( ( tracker_ctx.gnss_antenna_sel & GNSS_PCB_ANTENNA ) == GNSS_PCB_ANTENNA )
                    {
                        tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PCB_ANTENNA,
                                               &tracker_ctx.pcb_gnss_scan_result );
                    }

                    /* Run scan on the patch antenna if enable */
                    if( ( tracker_ctx.gnss_antenna_sel & GNSS_PATCH_ANTENNA ) == GNSS_PATCH_ANTENNA )
                    {
                        tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PATCH_ANTENNA,
                                               &tracker_ctx.patch_gnss_scan_result );
                    }

                    if( tracker_ctx.gnss_antenna_sel == ( GNSS_PATCH_ANTENNA | GNSS_PCB_ANTENNA ) )
                    {
                        gnss_scan_determine_best_nav_message( &tracker_ctx.pcb_gnss_scan_result,
                                                              &tracker_ctx.patch_gnss_scan_result );
                    }

                    /* Update the last_nb_detected_satellites field */
                    tracker_ctx.last_nb_detected_satellites =
                        SMTC_MAX( tracker_ctx.pcb_gnss_scan_result.nb_detected_satellites,
                                  tracker_ctx.patch_gnss_scan_result.nb_detected_satellites );
                }
                else
                {
                    HAL_DBG_TRACE_MSG( "Wait application layer clock synchronisation\r\n\r\n" );
                }
            }
            else
            {
                HAL_DBG_TRACE_PRINTF( "Wi-Fi scan good enough, drop GNSS scan\r\n" );
            }
        }

        break;
    }
    case TRACKER_NO_PRIORITY:
    {
        HAL_DBG_TRACE_INFO( "*** Tracker scan no priority *** \n\r\n\r" );

        /*  WIFI SCAN */
        if( tracker_ctx.wifi_settings.enabled == true )
        {
            HAL_DBG_TRACE_INFO( "*** Wi-Fi Scan *** \n\r\n\r" );

            tracker_wifi_run_scan( &tracker_ctx.wifi_settings, &tracker_ctx.wifi_result );

            tracker_ctx.last_nb_detected_mac_address = tracker_ctx.wifi_result.nbr_results;
        }

        /*  GNSS SCAN */
        if( tracker_ctx.gnss_settings.enabled == true )
        {
            HAL_DBG_TRACE_INFO( "*** Gnss Scan ***\n\r\n\r" );

            /* Check if a new assistance position is available */
            tracker_gnss_store_new_assistance_position( );

            if( tracker_ctx.has_date == true )
            {
                /* Get the next nb sat value */
                tracker_ctx.gnss_settings.nb_sat = tracker_gnss_get_next_nb_sat( );

                /* Run scan on the PCB antenna if enable */
                if( ( tracker_ctx.gnss_antenna_sel & GNSS_PCB_ANTENNA ) == GNSS_PCB_ANTENNA )
                {
                    tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PCB_ANTENNA,
                                           &tracker_ctx.pcb_gnss_scan_result );
                }

                /* Run scan on the patch antenna if enable */
                if( ( tracker_ctx.gnss_antenna_sel & GNSS_PATCH_ANTENNA ) == GNSS_PATCH_ANTENNA )
                {
                    tracker_gnss_run_scan( &tracker_ctx.gnss_settings, GNSS_PATCH_ANTENNA,
                                           &tracker_ctx.patch_gnss_scan_result );
                }

                if( tracker_ctx.gnss_antenna_sel == ( GNSS_PATCH_ANTENNA | GNSS_PCB_ANTENNA ) )
                {
                    gnss_scan_determine_best_nav_message( &tracker_ctx.pcb_gnss_scan_result,
                                                          &tracker_ctx.patch_gnss_scan_result );
                }

                /* Update the last_nb_detected_satellites field */
                tracker_ctx.last_nb_detected_satellites =
                    SMTC_MAX( tracker_ctx.pcb_gnss_scan_result.nb_detected_satellites,
                              tracker_ctx.patch_gnss_scan_result.nb_detected_satellites );
            }
            else
            {
                HAL_DBG_TRACE_MSG( "Wait application layer clock synchronisation\r\n\r\n" );
            }
        }

        break;
    }
    default:
    {
        break;
    }
    }

    /* Deactivate the partial low power mode */
    hal_mcu_partial_sleep_enable( false );

    /* If no scan results available send sensors */
    if( ( tracker_ctx.patch_gnss_scan_result.is_valid_nav_message == false ) &&
        ( tracker_ctx.pcb_gnss_scan_result.is_valid_nav_message == false ) &&
        ( tracker_ctx.wifi_result.nbr_results < 2 ) )
    {
        has_scan_result = false;
    }

    return has_scan_result;
}

static void tracker_app_reset_scan_results( void )
{
    tracker_ctx.pcb_gnss_scan_result.nb_detected_satellites   = 0;
    tracker_ctx.pcb_gnss_scan_result.nav_message_size         = 0;
    tracker_ctx.pcb_gnss_scan_result.is_valid_nav_message     = false;
    tracker_ctx.patch_gnss_scan_result.nb_detected_satellites = 0;
    tracker_ctx.patch_gnss_scan_result.nav_message_size       = 0;
    tracker_ctx.patch_gnss_scan_result.is_valid_nav_message   = false;
    tracker_ctx.wifi_result.nbr_results                       = 0;
}

static void tracker_app_build_and_stream_payload( bool send_complete_sensors )
{
    /* BUILD THE PAYLOAD IN TLV FORMAT */
    tracker_ctx.lorawan_payload_len = 0;  // reset the payload len

    HAL_DBG_TRACE_MSG( "\r\nAdd in the FiFo stream:\r\n" );

    if( ( tracker_ctx.patch_gnss_scan_result.is_valid_nav_message == true ) &&
        ( tracker_ctx.gnss_settings.enabled == true ) )
    {
        /* Add GNSS patch scan */

        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_NAV_PATCH_TAG;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
            tracker_ctx.patch_gnss_scan_result.nav_message_size - 1;  // GNSS PATCH LEN
        memcpy( tracker_ctx.lorawan_payload + tracker_ctx.lorawan_payload_len,
                tracker_ctx.patch_gnss_scan_result.nav_message + 1,
                tracker_ctx.patch_gnss_scan_result.nav_message_size - 1 );
        tracker_ctx.lorawan_payload_len += tracker_ctx.patch_gnss_scan_result.nav_message_size - 1;

        HAL_DBG_TRACE_MSG( " - NAV message from PATCH antenna\r\n" );
    }
    if( ( tracker_ctx.pcb_gnss_scan_result.is_valid_nav_message == true ) &&
        ( tracker_ctx.gnss_settings.enabled == true ) )
    {
        /* Add GNSS PCB scan */

        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_NAV_PCB_TAG;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
            tracker_ctx.pcb_gnss_scan_result.nav_message_size - 1;  // GNSS PCB LEN
        memcpy( tracker_ctx.lorawan_payload + tracker_ctx.lorawan_payload_len,
                tracker_ctx.pcb_gnss_scan_result.nav_message + 1,
                tracker_ctx.pcb_gnss_scan_result.nav_message_size - 1 );
        tracker_ctx.lorawan_payload_len += tracker_ctx.pcb_gnss_scan_result.nav_message_size - 1;

        HAL_DBG_TRACE_MSG( " - NAV message from PCB antenna\r\n" );
    }

    if( ( tracker_ctx.wifi_result.nbr_results >= 2 ) && ( tracker_ctx.wifi_settings.enabled == true ) )
    {
        /* Add Wi-Fi scan */
        uint8_t wifi_index = 0;

        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_WIFI_SCAN_TAG;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
            tracker_ctx.wifi_result.nbr_results * TLV_WIFI_SINGLE_BEACON_LEN +
            5;  // Wi-Fi Len = nb AP * 7 bytes + version (4 bytes) + timestamp (1 byte)

        /* Add Version */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_WIFI_VERSION;

        /* Add timestamp */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.timestamp >> 24;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.timestamp >> 16;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.timestamp >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.timestamp;

        wifi_index = tracker_ctx.lorawan_payload_len;
        for( uint8_t i = 0; i < tracker_ctx.wifi_result.nbr_results; i++ )
        {
            tracker_ctx.lorawan_payload[wifi_index] = tracker_ctx.wifi_result.results[i].rssi;
            memcpy( &tracker_ctx.lorawan_payload[wifi_index + 1], tracker_ctx.wifi_result.results[i].mac_address, 6 );
            wifi_index += TLV_WIFI_SINGLE_BEACON_LEN;
        }
        tracker_ctx.lorawan_payload_len += tracker_ctx.wifi_result.nbr_results * TLV_WIFI_SINGLE_BEACON_LEN;

        HAL_DBG_TRACE_MSG( " - WiFi scan\r\n" );
    }

    /* Add sensors value */
    tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_SENSORS_TAG;
    if( send_complete_sensors )
    {
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_SENSOR_FULL_VERSION_LEN;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
            ( ( TLV_SENSOR_FULL_VERSION & 0x0F ) << 4 ) | ( tracker_ctx.accelerometer_move_history & 0x0F );

        /* Temperature */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.temperature >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.temperature;

        /* Modem-E Charge */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accumulated_charge >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.accumulated_charge;

        /* Board Voltage */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.voltage >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.voltage;
    }
    else
    {
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_SENSOR_BASIC_VERSION_LEN;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] =
            ( ( TLV_SENSOR_BASIC_VERSION & 0x0F ) << 4 ) | ( tracker_ctx.accelerometer_move_history & 0x0F );
    }

    HAL_DBG_TRACE_MSG( " - Sensors value\r\n" );

    /* send this information just once */
    if( tracker_ctx.reset_cnt_sent == false )
    {
        /* Add Reset counters value */
        tracker_ctx.reset_cnt_sent = true;

        /* Reset counter */
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_RESET_COUNTER_TAG;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = 4;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.host_reset_cnt >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.host_reset_cnt;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.modem_reset_by_itself_cnt >> 8;
        tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = tracker_ctx.modem_reset_by_itself_cnt;

        HAL_DBG_TRACE_MSG( " - Reset counters\r\n" );
    }

    /* Push the Payload in the FiFo stream */
    tracker_app_add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

    tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
}

static void tracker_app_build_and_stream_tracker_settings( const uint8_t* buffer, uint8_t len )
{
    /* Add tracker settings value */
    tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = TLV_TRACKER_SETTINGS_TAG;  // Tracker settings TAG
    tracker_ctx.lorawan_payload[tracker_ctx.lorawan_payload_len++] = len;  // Tracker settings LEN is variable

    memcpy( tracker_ctx.lorawan_payload + 2, buffer, len );
    tracker_ctx.lorawan_payload_len += len;

    HAL_DBG_TRACE_PRINTF( " - Tracker settings (%d bytes) : ", tracker_ctx.lorawan_payload_len );

    /* Push the sensor values in the FiFo stream */
    tracker_app_add_payload_in_streaming_fifo( tracker_ctx.lorawan_payload, tracker_ctx.lorawan_payload_len );

    tracker_ctx.lorawan_payload_len = 0;  // reset the payload len
}

static void tracker_app_store_new_acculated_charge( uint32_t modem_charge )
{
    static uint32_t previous_modem_charge = 0;  // Previous modem charge before read it into the LR1110 Modem-E, keep
                                                // the historic even after leave the function because of the static

    /* Store the new accumulated charge only if the modem charge has changed */
    if( modem_charge != previous_modem_charge )
    {
        tracker_ctx.accumulated_charge += modem_charge - previous_modem_charge;
        HAL_DBG_TRACE_MSG( "New acculated charge stored\r\n" );
        tracker_store_app_ctx( );

        previous_modem_charge = modem_charge;
    }
}

static void tracker_app_join_network( void )
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
        HAL_DBG_TRACE_ERROR( "###### ===== JOINING CMD ERROR ==== ######\r\n\r\n" );
    }
}

static void tracker_app_adapt_adr( void )
{
    lr1110_modem_adr_profiles_t adr_profile;

    lr1110_modem_get_adr_profile( &lr1110, &adr_profile );

    if( tracker_ctx.send_alive_frame == true )  // means device is static
    {
        if( adr_profile != LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED )
        {
            HAL_DBG_TRACE_MSG( "Set ADR to LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED\n\r\n\r" );
            lr1110_modem_set_adr_profile( &lr1110, LR1110_MODEM_ADR_PROFILE_NETWORK_SERVER_CONTROLLED, NULL );
        }
    }
    else  // means device is mobile
    {
        if( adr_profile != tracker_ctx.lorawan_adr_profile )
        {
            HAL_DBG_TRACE_PRINTF( "Set ADR to %d\n\r\n\r", tracker_ctx.lorawan_adr_profile );
            lr1110_modem_set_adr_profile( &lr1110, ( lr1110_modem_adr_profiles_t ) tracker_ctx.lorawan_adr_profile,
                                          adr_custom_list );
            if( tracker_ctx.lorawan_adr_profile == LR1110_MODEM_ADR_PROFILE_CUSTOM )
            {
                lr1110_modem_set_nb_trans( &lr1110, 2 );
            }
        }
    }
}

static bool tracker_app_add_payload_in_streaming_fifo( const uint8_t* payload, uint16_t len )
{
    lr1110_modem_stream_status_t stream_status;

    /* Push the Payload in the FiFo stream */
    lr1110_modem_stream_status( &lr1110, LORAWAN_STREAM_APP_PORT, &stream_status );
    if( stream_status.free > len )
    {
        tracker_ctx.stream_done = false;
        HAL_DBG_TRACE_PRINTF( "%d bytes added in streaming FiFo\r\n", len );
        lr1110_modem_send_stream_data( &lr1110, LORAWAN_STREAM_APP_PORT, payload, len );

        return true;
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "Not enought space, need = %d bytes - free = %d bytes\r\n", len, stream_status.free );

        return false;
    }
}

static bool tracker_app_is_next_scan_possible( void )
{
    if( ( ( ( tracker_ctx.accelerometer_move_history & TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC ) != 0 ) ||
          ( tracker_ctx.send_alive_frame == true ) || ( tracker_ctx.accelerometer_used == 0 ) ) &&
        ( tracker_ctx.stream_done == true ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool tracker_app_is_region_use_duty_cycle( lr1110_modem_regions_t region )
{
    if( ( region == LR1110_LORAWAN_REGION_EU868 ) || ( region == LR1110_LORAWAN_REGION_RU864 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool tracker_app_is_tracker_in_static_mode( void )
{
    if( ( ( tracker_ctx.accelerometer_move_history & TRACKER_SEND_ONE_MORE_SCANS_ONCE_STATIC ) == 0 ) &&
        ( tracker_ctx.accelerometer_used == 1 ) )
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void tracker_app_parse_frame( uint8_t port, const uint8_t* payload, uint8_t size )
{
    switch( port )
    {
    case GNSS_PUSH_SOLVER_MSG_PORT:
    {
        HAL_DBG_TRACE_INFO( "###### ===== GNSS PUSH SOLVER MSG ==== ######\r\n\r\n" );

        lr1110_modem_gnss_push_solver_msg( &lr1110, payload, size );

        break;
    }
    case TRACKER_REQUEST_MSG_PORT:
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
            case GET_APP_TRACKER_SETTINGS_CMD:
            {
                uint8_t settings_buffer[240];
                memcpy( settings_buffer, payload + payload_index, len );

                HAL_DBG_TRACE_INFO(
                    "###### ===== TRACKER CONFIGURATION SETTINGS PAYLOAD RECEIVED ==== ######\r\n\r\n" );

                tracker_ctx.tracker_settings_payload_len =
                    tracker_parse_cmd( settings_buffer, tracker_ctx.tracker_settings_payload, false );

                /* Store the new values here if it's asked */
                if( ( tracker_ctx.new_value_to_set ) == true )
                {
                    tracker_ctx.new_value_to_set = false;
                    tracker_store_app_ctx( );
                }

                break;
            }
            case GET_MODEM_DATE_CMD:
            {
                uint8_t        buffer[6];
                const uint32_t modem_date = lr1110_tracker_board_get_systime_from_gps( &lr1110 );

                buffer[0] = GET_MODEM_DATE_CMD;
                buffer[1] = GET_MODEM_DATE_ANSWER_LEN;
                buffer[2] = modem_date >> 24;
                buffer[3] = modem_date >> 16;
                buffer[4] = modem_date >> 8;
                buffer[5] = modem_date;

                /* Use the emergency TX to reduce the latency */
                lr1110_modem_emergency_tx( &lr1110, port, LR1110_MODEM_UPLINK_UNCONFIRMED, buffer, 6 );

                HAL_DBG_TRACE_INFO( "###### ===== SEND MODEM-E DATE IN EMERGENCY TX ==== ######\r\n\r\n" );

                break;
            }
            default:
                payload_index += len;
                break;
            }
        }
    }
    default:
        break;
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER TIMER FUNCTION TYPES ------------------------------------------
 */

static void on_led_tx_timer_event( void* context )
{
    timer_stop( &led_tx_timer );
    /* Switch LED TX OFF */
    leds_off( LED_TX_MASK );
}

static void on_led_rx_timer_event( void* context )
{
    timer_stop( &led_rx_timer );
    /* Switch LED RX OFF */
    leds_off( LED_RX_MASK );
}

static void on_hall_effect_shutdown_timer_event( void* context )
{
    /* Stop Hall Effect sensors while the tracker is static in airplane mode */
    lr1110_tracker_board_hall_effect_enable( false );
    HAL_DBG_TRACE_MSG( "\r\nShut down hall effect sensor\r\n" );
}

/*
 * -----------------------------------------------------------------------------
 * --- TRACKER MODEM-E EVENT FUNCTION TYPES ------------------------------------
 */

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM-E RESET %lu ==== ######\r\n\r\n", reset_count );

    if( lr1110_tracker_board_is_ready( ) == true )
    {
        tracker_ctx.modem_reset_by_itself_cnt++;
        tracker_store_and_reset( 1 + lr1110_tracker_board_read_event_line( &lr1110 ) );
    }
    else
    {
        lr1110_tracker_board_set_ready( true );
    }
}

static void lr1110_modem_network_joined( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== JOINED ==== ######\r\n\r\n" );

    /* Set the ADR profile once joined */
    lr1110_modem_set_adr_profile( &lr1110, ( lr1110_modem_adr_profiles_t ) tracker_ctx.lorawan_adr_profile,
                                  adr_custom_list );
    if( tracker_ctx.lorawan_adr_profile == LR1110_MODEM_ADR_PROFILE_CUSTOM )
    {
        lr1110_modem_set_nb_trans( &lr1110, 2 );
    }

    /* Init the stream once joined */
    lr1110_modem_stream_init( &lr1110, LORAWAN_STREAM_APP_PORT, LR1110_MODEM_SERVICES_ENCRYPTION_DISABLE );
    /* Set the stream redundancy rate */
    lr1110_modem_set_stream_redundancy_rate( &lr1110, TRACKER_STREAM_REDUNDANCY_RATE );
}

static void lr1110_modem_join_fail( void ) { HAL_DBG_TRACE_INFO( "###### ===== JOIN FAIL ==== ######\r\n\r\n" ); }

void lr1110_modem_alarm( void )
{
    lr1110_modem_status_t        modem_status;
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

    HAL_DBG_TRACE_INFO( "###### ===== LR1110 ALARM ==== ######\r\n\r\n" );

    modem_response_code = lr1110_modem_get_status( &lr1110, &modem_status );

    if( modem_response_code == LR1110_MODEM_RESPONSE_CODE_OK )
    {
        modem_status_to_string( modem_status );

        if( ( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED ) ||
            ( ( modem_status & LR1110_LORAWAN_STREAM ) == LR1110_LORAWAN_STREAM ) ||
            ( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD ) )
        {
            device_state = DEVICE_COLLECT_DATA;
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

static void lr1110_modem_down_data( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port,
                                    const uint8_t* payload, uint8_t size )
{
    static uint32_t downlink_cnt = 0;
    HAL_DBG_TRACE_INFO( "\r\n###### ===== DOWNLINK FRAME %lu ==== ######\r\n\r\n", downlink_cnt++ );

    HAL_DBG_TRACE_PRINTF( "RX WINDOW   : %d\r\n", flags );

    HAL_DBG_TRACE_PRINTF( "RX PORT     : %d\r\n", port );

    if( size != 0 )
    {
        HAL_DBG_TRACE_MSG( "RX DATA     : " );
        print_hex_buffer( payload, size );
    }

    HAL_DBG_TRACE_PRINTF( "RX RSSI     : %d\r\n", rssi );
    HAL_DBG_TRACE_PRINTF( "RX SNR      : %d\r\n\r\n", snr );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

    leds_on( LED_RX_MASK );
    timer_start( &led_rx_timer );

    tracker_app_parse_frame( port, payload, size );
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

static void lr1110_modem_stream_done( void )
{
    static uint32_t stream_cnt = 0;
    HAL_DBG_TRACE_INFO( "###### ===== STREAM DONE nb %d ==== ######\r\n\r\n", stream_cnt++ );

    tracker_ctx.stream_done = true;
}

static void lr1110_modem_time_updated_alc_sync( lr1110_modem_alc_sync_state_t alc_sync_state )
{
    HAL_DBG_TRACE_INFO( "###### ===== APPLICATION LAYER CLOCK SYNC EVENT ==== ######\r\n\r\n" );

    /* Update the system_sanity_check bit field */
    tracker_ctx.system_sanity_check |= TRACKER_DOWNLINK_SUCCESSFUL_ONCE;

    if( alc_sync_state == LR1110_MODEM_ALC_SYNC_SYNCHRONIZED )
    {
        HAL_DBG_TRACE_MSG( "CLOCK SYNC STATE SYNCHRONIZED\r\n\r\n" );
        /* Notify user that the date has been received */
        leds_blink( LED_RX_MASK, 100, 4, true );
        tracker_ctx.has_date = true;
    }
    else
    {
        HAL_DBG_TRACE_MSG( "CLOCK SYNC STATE DESYNCHRONIZED\r\n\r\n" );
        /* Notify user that the date has been received */
        leds_blink( LED_TX_MASK, 100, 4, true );
        tracker_ctx.has_date = false;
    }
}

static void lr1110_modem_adr_mobile_to_static( void )
{
    HAL_DBG_TRACE_INFO( "###### ===== ADR HAS SWITCHED FROM MOBILE TO STATIC ==== ######\r\n\r\n" );
}

static void lr1110_modem_new_link_adr( void ) { HAL_DBG_TRACE_INFO( "###### ===== NEW LINK ADR ==== ######\r\n\r\n" ); }

static void lr1110_modem_no_event( void ) { HAL_DBG_TRACE_INFO( "###### ===== NO EVENT ==== ######\r\n\r\n" ); }

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
