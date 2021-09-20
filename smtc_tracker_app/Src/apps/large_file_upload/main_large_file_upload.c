/*!
 * @ingroup   apps_large_file_upload
 * @file      main_large_file_upload.c
 *
 * @brief     lr1110 Modem-E Large File Upload example application
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
 * @addtogroup apps_large_file_upload
 * LR1110 Modem-E Large File Upload example application
 * @{
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdio.h>
#include "main_large_file_upload.h"
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
    DEVICE_STATE_UPLOAD,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
} device_state = DEVICE_STATE_INIT;

/*!
 * @brief Table for CRC32 computation.
 */
static const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3, 0x0EDB8832,
    0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064, 0x6AB020F2,
    0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0, 0xFD62F97A,
    0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172,
    0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3,
    0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423,
    0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB,
    0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01, 0x6B6B51F4,
    0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457, 0x65B0D9C6, 0x12B7E950,
    0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158, 0x3AB551CE, 0xA3BC0074,
    0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0,
    0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525,
    0x206F85B3, 0xB966D409, 0xCE61E49F, 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81,
    0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615,
    0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7, 0xFED41B76,
    0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5, 0xD6D6A3E8, 0xA1D1937E,
    0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B, 0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6,
    0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236,
    0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7,
    0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F,
    0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7,
    0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3, 0x166CCF45, 0xA00AE278,
    0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB, 0xAED16A4A, 0xD9D65ADC,
    0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C, 0xCABAC28A, 0x53B39330,
    0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94,
    0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D,
};

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
 * @brief  File upload completed callback prototype.
 *
 * @param [in] upload status \see lr1110_modem_upload_event_t
 */
static void lr1110_modem_upload_done( lr1110_modem_upload_event_t upload_status );

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
 * @brief Compute the CRC32 of the given data.
 *
 * @param [in] crc Initial CRC32 value.
 * @param [in] buf Pointer to the start of the buffer containing the data.
 * @param [in] size Number of data bytes.
 *
 * @return CRC32 of the given data.
 */
static uint32_t crc32( uint32_t crc, const uint8_t* buf, int size );

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
    HAL_DBG_TRACE_INFO( "###### ===== LoRa Basics Modem-E Large File Upload example application ==== ######\r\n\r\n" );
    HAL_DBG_TRACE_PRINTF( "APP VERSION : %d.%d.%d\r\n\r\n", MAJOR_APP_VERSION, MINOR_APP_VERSION,
                          SUB_MINOR_APP_VERSION );

    /* Init LR1110 modem-e event */
    memset( &lr1110_modem_event_callback, 0, sizeof( lr1110_modem_event_callback ) );
    lr1110_modem_event_callback.reset        = lr1110_modem_reset_event;
    lr1110_modem_event_callback.alarm        = lr1110_modem_alarm;
    lr1110_modem_event_callback.joined       = lr1110_modem_network_joined;
    lr1110_modem_event_callback.join_fail    = lr1110_modem_join_fail;
    lr1110_modem_event_callback.upload_done  = lr1110_modem_upload_done;
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
        case DEVICE_STATE_UPLOAD:
        {
            static bool uploaded = false;
            if( uploaded == false )
            {
                /* Prepare and start the large file upload */
                static uint8_t               payload[APP_DATA_BUFFER_SIZE];
                static int                   payload_bytes = 0;
                static char                  message[80];
                int                          byte_count;
                uint32_t                     chunk_count         = 0;
                lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;

                /* Build the full payload */
                HAL_DBG_TRACE_INFO( "###### ===== BUILDING PAYLOAD ==== ######\r\n\r\n" );
                byte_count = snprintf( message, sizeof( message ), "One file chunk (#%ld)\r\n", ( long ) chunk_count );
                while( ( byte_count > 0 ) && ( byte_count < sizeof( payload ) - payload_bytes ) )
                {
                    chunk_count++;
                    memcpy( &payload[payload_bytes], message, byte_count );
                    payload_bytes += byte_count;
                    byte_count =
                        snprintf( message, sizeof( message ), "Another file chunk (#%ld)\r\n", ( long ) chunk_count );
                }

                /* Prepare the upload. */
                modem_response_code = lr1110_modem_upload_init( &lr1110, LORAWAN_UPLOAD_APP_PORT,
                                                                LR1110_MODEM_SERVICES_ENCRYPTION_DISABLE, payload_bytes,
                                                                UPLOAD_TX_INTERVAL_S );
                if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
                {
                    HAL_DBG_TRACE_ERROR( "lr1110_modem_upload_init failed (%d)\r\n", modem_response_code );
                }
                else
                {
                    uint32_t upload_index  = 0;
                    uint32_t payload_crc32 = crc32( payload_crc32, &payload[0], payload_bytes );
                    while( payload_bytes > 0 )
                    {
                        if( payload_bytes > UPLOAD_DATA_MAX_FRAGMENT_SIZE )
                        {
                            HAL_DBG_TRACE_PRINTF( "Adding %d bytes to the payload.\r\n",
                                                  UPLOAD_DATA_MAX_FRAGMENT_SIZE );
                            modem_response_code = lr1110_modem_upload_data( &lr1110, &payload[upload_index],
                                                                            UPLOAD_DATA_MAX_FRAGMENT_SIZE );
                            upload_index += UPLOAD_DATA_MAX_FRAGMENT_SIZE;
                            payload_bytes -= UPLOAD_DATA_MAX_FRAGMENT_SIZE;
                        }
                        else
                        {
                            HAL_DBG_TRACE_PRINTF( "Adding %d bytes to the payload.\r\n", payload_bytes );
                            modem_response_code =
                                lr1110_modem_upload_data( &lr1110, &payload[upload_index], payload_bytes );
                            upload_index += payload_bytes;
                            payload_bytes = 0;
                        }

                        if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
                        {
                            HAL_DBG_TRACE_ERROR( "lr1110_modem_upload_data failed (%d)\r\n", modem_response_code );
                        }
                    }

                    /* Start the upload operation */
                    HAL_DBG_TRACE_INFO( "###### ===== STARTING LARGE FILE UPLOAD ==== ######\r\n\r\n" );
                    modem_response_code = lr1110_modem_upload_start( &lr1110, payload_crc32 );
                    if( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK )
                    {
                        HAL_DBG_TRACE_ERROR( "lr1110_modem_upload_start failed (%d)\r\n", modem_response_code );
                    }
                }

                /* Only upload once. */
                uploaded = true;
            }

            device_state = DEVICE_STATE_CYCLE;

            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            /* Schedule next packet transmission */
            do
            {
                modem_response_code = lr1110_modem_set_alarm_timer( &lr1110, UPLOAD_TX_INTERVAL_S );
            } while( modem_response_code != LR1110_MODEM_RESPONSE_CODE_OK );

            HAL_DBG_TRACE_PRINTF( "lr1110_modem_set_alarm_timer : %d s\r\n\r\n", UPLOAD_TX_INTERVAL_S );

            device_state = DEVICE_STATE_SLEEP;

            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            /* The MCU wakes up through events */
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
        else if( ( modem_status & LR1110_LORAWAN_UPLOAD ) == LR1110_LORAWAN_UPLOAD )
        {
            /* Upload not done yet. Wait. */
            device_state = DEVICE_STATE_CYCLE;
        }
        else if( ( modem_status & LR1110_LORAWAN_JOINED ) == LR1110_LORAWAN_JOINED )
        {
            /* Ready to upload more data. */
            device_state = DEVICE_STATE_UPLOAD;
        }
        else
        {
            HAL_DBG_TRACE_WARNING( "Unexpected modem status %d\r\n\r\n", modem_status );
            device_state = DEVICE_STATE_CYCLE;
        }
    }
}

static void lr1110_modem_upload_done( lr1110_modem_upload_event_t upload_status )
{
    HAL_DBG_TRACE_INFO( "###### ===== UPLOAD DONE ( status = %d ) ==== ######\r\n\r\n", upload_status );
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

uint32_t crc32( uint32_t crc, const uint8_t* buf, int size )
{
    const uint8_t* p = buf;

    crc = crc ^ ~0U;
    while( size-- > 0 )
    {
        crc = crc32_table[( crc ^ *p++ ) & 0xFF] ^ ( crc >> 8 );
    }
    return crc ^ ~0U;
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
