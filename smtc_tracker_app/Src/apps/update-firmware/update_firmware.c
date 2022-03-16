/*!
 * @file      update_firmware.c
 *
 * @brief     lr1110 firmware update implementation
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

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define READ_CHIP_EUI 0

/* download header files here : https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110 */

#if( MODEM_TO_TRX == 1 )
#include "lr1110_transceiver_0307.h"
#endif
#if( ( TRX_TO_MODEM == 1 ) || ( MODEM_TO_MODEM == 1 ) )
#include "lr1110_modem_1.1.7.h"
#endif

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
 * @brief Update the transceiver firmware to a modem firmware
 *
 * @param [in] data pointer on the buffer containing the Modem-E Firmware
 * @param [in] length Len of the buffer
 */
void lr1110_update_trx_to_modem( const uint32_t* data, const uint16_t length );

/*!
 * @brief Update the modem firmware to a transceiver firmware
 *
 * @param [in] fw_version Modem-E firmware version to update
 * @param [in] data pointer on the buffer containing the Modem-E Firmware
 * @param [in] length Len of the buffer
 */
void lr1110_update_modem_to_trx( const uint16_t fw_version, const uint32_t* data, const uint16_t length );

/*!
 * @brief Update the modem firmware to another modem firmware
 *
 * @param [in] fw_version Modem-E firmware version to update
 * @param [in] data pointer on the buffer containing the Modem-E Firmware
 * @param [in] length Len of the buffer
 */
void lr1110_update_modem_to_modem( const uint32_t fw_version, const uint32_t* data, const uint16_t length );

/*!
 * @brief Read the Chip EUI
 */
void lr1110_get_chip_eui( void );

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
    /* Target board initialization */
    hal_mcu_init( );
    hal_mcu_init_periph( );

#if( TRX_TO_MODEM )
    lr1110_update_trx_to_modem( lr1110_firmware_image, LR1110_FIRMWARE_IMAGE_SIZE );
#endif
#if( MODEM_TO_TRX )
    lr1110_update_modem_to_trx( LR1110_FIRMWARE_VERSION, lr1110_firmware_image, LR1110_FIRMWARE_IMAGE_SIZE );
#endif
#if( MODEM_TO_MODEM )
    lr1110_update_modem_to_modem( 0x010107, lr1110_firmware_image, LR1110_FIRMWARE_IMAGE_SIZE );
#endif
#if( READ_CHIP_EUI )
    lr1110_get_chip_eui( );
#endif

    while( 1 )
        ;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

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

void lr1110_update_trx_to_modem( const uint32_t* data, const uint16_t length )
{
    lr1110_bootloader_version_t   version;
    lr1110_modem_version_t        modem;
    lr1110_modem_event_callback_t lr1110_modem_event_callback;

    lr1110_bootloader_get_version( &lr1110, &version );
    HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );

    if( version.hw >= 0x21 )
    {
        HAL_DBG_TRACE_MSG( "UPDATE TO MODEM\n\r" );

        /* Switch in bootloader */
        lr1110_modem_hal_enter_dfu( &lr1110 );

        lr1110_bootloader_get_version( &lr1110, &version );
        HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );

        /* Erase Flash */
        lr1110_bootloader_erase_flash( &lr1110 );

        if( version.fw == 0 )  // 0 means dev chip
        {
            lr1110_bootloader_write_flash_full( &lr1110, 0, data, length );
        }
        else
        {
            lr1110_bootloader_write_flash_encrypted_full( &lr1110, 0, data, length );
        }

        lr1110_hal_reset( &lr1110 );
        HAL_Delay( 1500 );
        lr1110_modem_event_callback.reset = lr1110_modem_reset_event;
        lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );

        HAL_DBG_TRACE_MSG( "UPDATED\n\r" );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "DEVICE ALREADY IN MODEM \n\r" );
    }

    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LR1110 : lorawan:%#04X / firmware:%#02X bootloader:%#02X \n\r", modem.lorawan,
                          modem.firmware, modem.bootloader );
    HAL_Delay( 200 );
}

void lr1110_update_modem_to_trx( const uint16_t fw_version, const uint32_t* data, const uint16_t length )
{
    lr1110_bootloader_version_t version;

    lr1110_bootloader_get_version( &lr1110, &version );
    HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );

    if( version.fw != fw_version )
    {
        HAL_DBG_TRACE_MSG( "UPDATE TO MODEM TO TRX\n\r" );

        /* Switch in bootloader */
        lr1110_modem_hal_enter_dfu( &lr1110 );

        lr1110_bootloader_get_version( &lr1110, &version );
        HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );

        /* Erase Flash */
        lr1110_bootloader_erase_flash( &lr1110 );

        if( version.fw == 0 )  // 0 means dev chip
        {
            lr1110_bootloader_write_flash_full( &lr1110, 0, data, length );
        }
        else
        {
            lr1110_bootloader_write_flash_encrypted_full( &lr1110, 0, data, length );
        }

        lr1110_hal_reset( &lr1110 );
        HAL_Delay( 200 );
        lr1110_hal_reset( &lr1110 );

        HAL_DBG_TRACE_MSG( "UPDATED\n\r" );

        lr1110_bootloader_get_version( &lr1110, &version );
        HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", version.hw, version.type, version.fw );
        HAL_Delay( 200 );
    }
    else
    {
        HAL_DBG_TRACE_PRINTF( "DEVICE ALREADY IN TRX WITH VERSION %#04X\n\r", version.fw );
    }
}

void lr1110_update_modem_to_modem( const uint32_t fw_version, const uint32_t* data, const uint16_t length )
{
    lr1110_modem_version_t        modem;
    lr1110_modem_event_callback_t lr1110_modem_event_callback;

    lr1110_modem_event_callback.reset = lr1110_modem_reset_event;
    lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );

    lr1110_modem_get_version( &lr1110, &modem );
    HAL_DBG_TRACE_PRINTF( "LR1110 : lorawan:%#02X / firmware:%#04X / bootloader:%#03X / functionality:%#03X\n\r",
                          modem.lorawan, modem.firmware, modem.bootloader, modem.functionality );

    if( modem.functionality == 0x04 )
    {
        if( modem.firmware != fw_version )
        {
            lr1110_bootloader_version_t bootloader_version;

            HAL_DBG_TRACE_MSG( "UPDATE TO MODEM\n\r" );

            /* Switch in bootloader */
            lr1110_modem_hal_enter_dfu( &lr1110 );

            lr1110_bootloader_get_version( &lr1110, &bootloader_version );
            HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", bootloader_version.hw,
                                  bootloader_version.type, bootloader_version.fw );

            /* Erase Flash */
            lr1110_bootloader_erase_flash( &lr1110 );

            if( bootloader_version.fw == 0 )  // 0 means dev chip
            {
                lr1110_bootloader_write_flash_full( &lr1110, 0, data, length );
            }
            else
            {
                lr1110_bootloader_write_flash_encrypted_full( &lr1110, 0, data, length );
            }

            lr1110_hal_reset( &lr1110 );
            HAL_Delay( 1500 );
            lr1110_modem_event_callback.reset = lr1110_modem_reset_event;
            lr1110_tracker_board_init( &lr1110, &lr1110_modem_event_callback );

            HAL_DBG_TRACE_MSG( "UPDATED\n\r" );
        }
        else
        {
            HAL_DBG_TRACE_MSG( "MODEM ALREADY UP TO DATE\n\r" );
        }

        lr1110_modem_get_version( &lr1110, &modem );
        HAL_DBG_TRACE_PRINTF( "LR1110 : lorawan:%#02X / firmware:%#04X / bootloader:%#03X / functionality:%#03X\n\r",
                              modem.lorawan, modem.firmware, modem.bootloader, modem.functionality );
        HAL_Delay( 200 );
    }
    else
    {
        HAL_DBG_TRACE_MSG( "DEVICE IS NOT A MODEM \n\r" );
    }
}

void lr1110_get_chip_eui( void )
{
    lr1110_bootloader_version_t bootloader_version;
    uint8_t                     chip_eui[8];

    HAL_DBG_TRACE_MSG( "GET CHIP EUI\n\r" );

    /* Switch in bootloader */
    lr1110_modem_hal_enter_dfu( &lr1110 );

    lr1110_bootloader_get_version( &lr1110, &bootloader_version );
    HAL_DBG_TRACE_PRINTF( "LR1110 : hw:%#02X / type:%#02X / fw:%#04X\n\r", bootloader_version.hw,
                          bootloader_version.type, bootloader_version.fw );

    lr1110_bootloader_read_chip_eui( &lr1110, chip_eui );

    HAL_DBG_TRACE_MSG( "CHIP EUI :" );
    for( uint8_t i = 0; i < 8; i++ )
    {
        HAL_DBG_TRACE_PRINTF( "%#02X ", chip_eui[i] );
    }
    HAL_DBG_TRACE_MSG( "\n\r" );
}

static void lr1110_modem_reset_event( uint16_t reset_count )
{
    HAL_DBG_TRACE_INFO( "###### ===== LR1110 MODEM RESET %lu ==== ######\r\n\r\n", reset_count );

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

/* --- EOF ------------------------------------------------------------------ */
