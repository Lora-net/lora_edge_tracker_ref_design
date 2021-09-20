/*!
 * @file      lr1110_modem_hal.c
 *
 * @brief     Hardware Abstraction Layer (HAL) implementation for LR1110
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

#include <stdlib.h>
#include "lr1110_hal.h"
#include "lr1110_modem_hal.h"
#include "lr1110_modem_system.h"
#include "lr1110_tracker_board.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_MODEM_RESET_TIMEOUT 3000

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * @brief LR1110 modem-e reset timeout flag
 */
static bool lr1110_modem_reset_timeout = false;

/*!
 * @brief Timer to handle the scan timeout
 */
static timer_event_t lr1110_modem_reset_timeout_timer;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief Function to wait that the lr1110 transceiver busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_hal_status_t lr1110_hal_wait_on_busy( const void* context, uint32_t timeout_ms );

/*!
 * @brief Function to wait that the lr1110 modem-e busy line fall to low
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy( const void* context, uint32_t timeout_ms );

/*!
 * @brief Function to wait the that lr1110 modem-e busy line raise to high
 *
 * @param [in] context Chip implementation context
 * @param [in] timeout_ms timeout in millisec before leave the function
 *
 * @returns lr1110_hal_status_t
 */
static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy( const void* context, uint32_t timeout_ms );

/*!
 * @brief Function executed on lr1110 modem-e reset timeout event
 */
static void on_lr1110_modem_reset_timeout_event( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

/*!
 * @brief lr1110_modem_hal.h API implementation
 */

lr1110_modem_hal_status_t lr1110_modem_hal_write( const void* context, const uint8_t* command,
                                                  const uint16_t command_length, const uint8_t* data,
                                                  const uint16_t data_length )
{
    if( lr1110_modem_hal_wakeup( context ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1110_modem_hal_status_t status;

        /* NSS low */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        /* Send CMD */
        for( uint16_t i = 0; i < command_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, command[i] );
        }
        /* Send Data */
        for( uint16_t i = 0; i < data_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, data[i] );
        }
        /* Compute and send CRC */
        crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
        crc = lr1110_modem_compute_crc( crc, data, data_length );
        /* Send CRC */
        hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, crc );

        /* NSS high */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        /* Wait on busy pin up to 1000 ms */
        if( lr1110_modem_hal_wait_on_busy( context, 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        /* Send dummy byte to retrieve RC & CRC */

        /* NSS low */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        /* read RC */
        status       = ( lr1110_modem_hal_status_t ) hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );
        crc_received = hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );

        /* Compute response crc */
        crc = lr1110_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );

        /* NSS high */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        if( crc != crc_received )
        {
            /* change the response code */
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if( lr1110_modem_hal_wait_on_unbusy( context, 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_write_without_rc( const void* context, const uint8_t* command,
                                                             const uint16_t command_length, const uint8_t* data,
                                                             const uint16_t data_length )
{
    if( lr1110_modem_hal_wakeup( context ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc    = 0;
        lr1110_modem_hal_status_t status = LR1110_MODEM_HAL_STATUS_OK;

        /* NSS low */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        /* Send CMD */
        for( uint16_t i = 0; i < command_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, command[i] );
        }
        /* Send Data */
        for( uint16_t i = 0; i < data_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, data[i] );
        }
        /* Compute and send CRC */
        crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
        crc = lr1110_modem_compute_crc( crc, data, data_length );
        /* Send CRC */
        hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, crc );

        /* NSS high */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_read( const void* context, const uint8_t* command,
                                                 const uint16_t command_length, uint8_t* data,
                                                 const uint16_t data_length )
{
    if( lr1110_modem_hal_wakeup( context ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        uint8_t                   crc          = 0;
        uint8_t                   crc_received = 0;
        lr1110_modem_hal_status_t status;

        /* NSS low */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        /* Send CMD */
        for( uint16_t i = 0; i < command_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, command[i] );
        }
        /* Compute and send CRC */
        crc = lr1110_modem_compute_crc( 0xFF, command, command_length );
        /* Send CRC */
        hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, crc );

        /* NSS high */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        /* Wait on busy pin up to 1000 ms */
        if( lr1110_modem_hal_wait_on_busy( context, 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        /* Send dummy byte to retrieve RC & CRC */

        /* NSS low */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        /* read RC */
        status = ( lr1110_modem_hal_status_t ) hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );

        if( status == LR1110_MODEM_HAL_STATUS_OK )
        {
            for( uint16_t i = 0; i < data_length; i++ )
            {
                data[i] = hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );
            }
        }

        crc_received = hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );

        /* NSS high */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        /* Compute response crc */
        crc = lr1110_modem_compute_crc( 0xFF, ( uint8_t* ) &status, 1 );
        if( status == LR1110_MODEM_HAL_STATUS_OK )
        {
            crc = lr1110_modem_compute_crc( crc, data, data_length );
        }

        if( crc != crc_received )
        {
            /* change the response code */
            status = LR1110_MODEM_HAL_STATUS_BAD_FRAME;
        }

        /* Wait on busy pin up to 1000 ms */
        if( lr1110_modem_hal_wait_on_unbusy( context, 1000 ) != LR1110_MODEM_HAL_STATUS_OK )
        {
            return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
        }

        return status;
    }

    return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
}

lr1110_modem_hal_status_t lr1110_modem_hal_reset( const void* context )
{
    lr1110_tracker_board_set_ready( false );

    /* Start a reset timeout timer */
    timer_init( &lr1110_modem_reset_timeout_timer, on_lr1110_modem_reset_timeout_event );
    timer_set_value( &lr1110_modem_reset_timeout_timer, LR1110_MODEM_RESET_TIMEOUT );
    timer_start( &lr1110_modem_reset_timeout_timer );
    lr1110_modem_reset_timeout = false;

    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 0 );
    HAL_Delay( 1 );
    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 1 );

    /* wait for reset event */
    while( ( lr1110_tracker_board_is_ready( ) == false ) && ( lr1110_modem_reset_timeout == false ) )
    {
        lr1110_modem_event_process( context );
    }

    if( lr1110_modem_reset_timeout == true )
    {
        return LR1110_MODEM_HAL_STATUS_ERROR;
    }
    else
    {
        return LR1110_MODEM_HAL_STATUS_OK;
    }
}

void lr1110_modem_hal_enter_dfu( const void* context )
{
    /* Force dio0 to 0 */
    hal_gpio_init_out( ( ( lr1110_t* ) context )->busy.pin, 0 );

    /* reset the chip */
    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 0 );
    HAL_Delay( 1 );
    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 1 );

    /* wait 250ms */
    HAL_Delay( 250 );

    /* reinit dio0 */
    hal_gpio_init_in( ( ( lr1110_t* ) context )->busy.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL );
}

lr1110_modem_hal_status_t lr1110_modem_hal_wakeup( const void* context )
{
    if( lr1110_modem_hal_wait_on_busy( context, 10000 ) == LR1110_MODEM_HAL_STATUS_OK )
    {
        /* Wakeup radio */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );
    }
    else
    {
        return LR1110_MODEM_HAL_STATUS_BUSY_TIMEOUT;
    }

    /* Wait on busy pin for 1000 ms */
    return lr1110_modem_hal_wait_on_unbusy( context, 1000 );
}

/*!
 * @brief Bootstrap bootloader and SPI bootloader API implementation
 */

lr1110_hal_status_t lr1110_hal_write( const void* context, const uint8_t* command, const uint16_t command_length,
                                      const uint8_t* data, const uint16_t data_length )
{
    if( lr1110_hal_wakeup( context ) == LR1110_HAL_STATUS_OK )
    {
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );
        for( uint16_t i = 0; i < command_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, command[i] );
        }
        for( uint16_t i = 0; i < data_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, data[i] );
        }
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        return lr1110_hal_wait_on_busy( context, 5000 );
    }
    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_read( const void* context, const uint8_t* command, const uint16_t command_length,
                                     uint8_t* data, const uint16_t data_length )
{
    if( lr1110_hal_wakeup( context ) == LR1110_HAL_STATUS_OK )
    {
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        for( uint16_t i = 0; i < command_length; i++ )
        {
            hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, command[i] );
        }

        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        if( lr1110_hal_wait_on_busy( context, 5000 ) != LR1110_HAL_STATUS_OK )
        {
            return LR1110_HAL_STATUS_ERROR;
        }

        /* Send dummy byte */
        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );

        hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );

        for( uint16_t i = 0; i < data_length; i++ )
        {
            data[i] = hal_spi_in_out( ( ( lr1110_t* ) context )->spi_id, 0 );
        }

        hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

        return lr1110_hal_wait_on_busy( context, 5000 );
    }
    return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_wakeup( const void* context )
{
    /* Wakeup radio */
    hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 0 );
    hal_gpio_set_value( ( ( lr1110_t* ) context )->nss.pin, 1 );

    /* Wait on busy pin for 5000 ms */
    return lr1110_hal_wait_on_busy( context, 5000 );
}

lr1110_hal_status_t lr1110_hal_reset( const void* context )
{
    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 0 );
    HAL_Delay( 1 );
    hal_gpio_set_value( ( ( lr1110_t* ) context )->reset.pin, 1 );

    return LR1110_HAL_STATUS_OK;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static void on_lr1110_modem_reset_timeout_event( void* context ) { lr1110_modem_reset_timeout = true; }

static lr1110_hal_status_t lr1110_hal_wait_on_busy( const void* context, uint32_t timeout_ms )
{
#if 0
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 1 )
    {
        ;
    }
#else
    uint32_t start = hal_rtc_get_time_ms( );
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 1 )
    {
        if( ( int32_t )( hal_rtc_get_time_ms( ) - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_HAL_STATUS_OK;
}

static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_busy( const void* context, uint32_t timeout_ms )
{
#if 0
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 0 )
    {
        ;
    }
#else
    uint32_t start   = hal_rtc_get_time_ms( );
    uint32_t current = 0;
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 0 )
    {
        current = hal_rtc_get_time_ms( );
        if( ( int32_t )( current - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_MODEM_HAL_STATUS_OK;
}

static lr1110_modem_hal_status_t lr1110_modem_hal_wait_on_unbusy( const void* context, uint32_t timeout_ms )
{
#if 0
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 1 )
    {
        ;
    }
#else
    uint32_t start   = hal_rtc_get_time_ms( );
    uint32_t current = 0;
    while( hal_gpio_get_value( ( ( lr1110_t* ) context )->busy.pin ) == 1 )
    {
        current = hal_rtc_get_time_ms( );
        if( ( int32_t )( current - start ) > ( int32_t ) timeout_ms )
        {
            return LR1110_MODEM_HAL_STATUS_ERROR;
        }
    }
#endif
    return LR1110_MODEM_HAL_STATUS_OK;
}

/* --- EOF ------------------------------------------------------------------ */
