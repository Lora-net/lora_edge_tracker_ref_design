/*!
 * @ingroup   apps_uart_firmware_update
 * @file      main_uart_firmware_update.c
 *
 * @brief     UART-based LR1110 Firmware Update
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
#include "lr1110_modem_hal.h"
#include "leds.h"
#include "smtc_hal_options.h"
#include <stdint.h>

/*!
 * @addtogroup apps_uart_firmware_update
 * UART-based LR1110 Firmware Update
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
 * @brief Tag value for the GET_VERSION request.
 */
#define GET_VERSION_CMD 0x00

/*!
 * @brief Data length for the GET_VERSION reply (in bytes).
 */
#define GET_VERSION_ANSWER_LEN 0x0004

/*!
 * @brief Tag value for the GET_MODEM_E_VERSION request.
 */
#define GET_MODEM_E_VERSION_CMD 0x01
/*!
 * @brief Data length for the GET_MODEM_E_VERSION request (in bytes).
 */
#define GET_MODEM_E_VERSION_LEN 0x0000
/*!
 * @brief Data length for the GET_MODEM_E_VERSION reply (in bytes).
 */
#define GET_MODEM_E_VERSION_ANSWER_LEN 0x000E

/*!
 * @brief Tag value for the WRITE_LR1110_UPDATE request.
 */
#define WRITE_LR1110_UPDATE_CMD 0x02
/*!
 * @brief Data length for the WRITE_LR1110_UPDATE request (in bytes).
 *
 * Includes 2 bytes for a fragment id and up to 256 bytes of firmware data.
 */
#define WRITE_LR1110_UPDATE_LEN ( 0x0002 + 0x0100 )
/*!
 * @brief Data length for the WRITE_LR1110_UPDATE reply (in bytes).
 */
#define WRITE_LR1110_UPDATE_ANSWER_LEN 0x0002

/*!
 * @brief Number of expected 256-byte fragments for a full firmware image.
 */
#define NB_CHUNK_FIRMWARE 958

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
 * @brief Get the firmware version.
 *
 * @note Requires that the LR1110 is in bootloader mode or runs the Transceiver firmware.
 */
static void get_version( void );

/*!
 * @brief Get the Modem-E firmware version.
 *
 * @note Requires that the LR1110 runs the Modem-E firmware.
 */
static void get_modem_e_version( void );

/*!
 * @brief Get a fragment of firmware from the UART and install it.
 *
 * @param len Size in bytes of the fragment.
 *
 * @note Will automatically reboot the LR1110 in bootloader mode when handling
 *        the first fragment.
 * @note Will automatically restart and run the new firmware after copying
 *        tha last fragment.
 */
static void write_lr1110_update( uint16_t len );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/**
 * @brief Application entry point and main loop.
 */
int main( void )
{
    /* Init board */
    hal_mcu_init( );
    hal_mcu_init_periph( );

    /* Board is initialized */
    leds_blink( LED_ALL_MASK, 100, 2, true );

    while( 1 )
    {
        /* Wait and read the next command from the UART (TLV format) */
        uint8_t  buffer[3];
        uint8_t  tag = 0;
        uint16_t len = 0;
        hal_uart_rx( HAL_PRINTF_UART_ID, ( uint8_t* ) &buffer[0], 3 );
        tag = buffer[0];
        len = ( ( uint16_t ) buffer[2] << 8 ) + buffer[1];
        switch( tag )
        {
        case GET_VERSION_CMD:
            get_version( );
            break;
        case GET_MODEM_E_VERSION_CMD:
            get_modem_e_version( );
            break;
        case WRITE_LR1110_UPDATE_CMD:
            write_lr1110_update( len );
            break;
        default:
            break;
        }
    }
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

void get_version( void )
{
    lr1110_bootloader_version_t version;
    uint8_t                     buffer[3 + GET_VERSION_ANSWER_LEN];

    leds_on( LED_RX_MASK );

    memset( &version, 0, sizeof( version ) );
    lr1110_bootloader_get_version( &lr1110, &version );

    /* Prepare and send the answer. */
    buffer[0]                 = GET_VERSION_CMD;
    *( uint16_t* ) &buffer[1] = GET_VERSION_ANSWER_LEN;
    memcpy( &buffer[3], &version, GET_VERSION_ANSWER_LEN );
    hal_uart_tx( HAL_PRINTF_UART_ID, ( uint8_t* ) &buffer[0], sizeof( buffer ) );

    leds_off( LED_RX_MASK );
}

void get_modem_e_version( void )
{
    lr1110_modem_version_t version;
    uint8_t                buffer[3 + GET_MODEM_E_VERSION_ANSWER_LEN];

    leds_on( LED_RX_MASK );

    memset( &version, 0, sizeof( version ) );
    lr1110_modem_get_version( &lr1110, &version );

    /* Prepare and send the answer. */
    buffer[0]                 = GET_MODEM_E_VERSION_CMD;
    *( uint16_t* ) &buffer[1] = GET_MODEM_E_VERSION_ANSWER_LEN;
    memcpy( &buffer[3], &version, GET_MODEM_E_VERSION_ANSWER_LEN );
    hal_uart_tx( HAL_PRINTF_UART_ID, ( uint8_t* ) &buffer[0], sizeof( buffer ) );

    leds_off( LED_RX_MASK );
}

void write_lr1110_update( uint16_t len )
{
    static uint8_t  buffer[WRITE_LR1110_UPDATE_LEN];
    static uint32_t lr1110_firmware_flash_offset;
    uint16_t        fragment_id = 0;

    if( len > WRITE_LR1110_UPDATE_LEN )
    {
        len = WRITE_LR1110_UPDATE_LEN;
    }

    hal_uart_rx( HAL_PRINTF_UART_ID, &buffer[0], len );

    leds_on( LED_RX_MASK );

    fragment_id = *( uint16_t* ) &buffer[0];
    if( fragment_id == 0 )
    {
        /* First fragment */
        leds_on( LED_TX_MASK );
        /* Reset the LR1110 and make it stay in bootloader mode */
        lr1110_modem_hal_enter_dfu( &lr1110 );
        /* Erase Flash */
        lr1110_bootloader_erase_flash( &lr1110 );
        lr1110_firmware_flash_offset = 0;
    }

    /* Write the firmware fragment */
    lr1110_bootloader_write_flash_encrypted( &lr1110, lr1110_firmware_flash_offset, ( uint32_t* ) &buffer[2],
                                             ( len - 2 ) / 4 );
    lr1110_firmware_flash_offset += len - 2;

    if( fragment_id == NB_CHUNK_FIRMWARE )
    {
        /* Last fragment */
        /* Reset the LR1110 and start the new firmware */
        lr1110_bootloader_reboot( &lr1110, false );
        HAL_Delay( 1500 );
        leds_off( LED_TX_MASK );
    }

    /* Send back the fragment ID as an acknowledgment of the command */
    buffer[0]                 = WRITE_LR1110_UPDATE_CMD;
    *( uint16_t* ) &buffer[1] = WRITE_LR1110_UPDATE_ANSWER_LEN;
    *( uint16_t* ) &buffer[3] = fragment_id;
    hal_uart_tx( HAL_PRINTF_UART_ID, &buffer[0], WRITE_LR1110_UPDATE_ANSWER_LEN + 3 );

    leds_off( LED_RX_MASK );
}

/*!
 * @}
 */

/* --- EOF ------------------------------------------------------------------ */
