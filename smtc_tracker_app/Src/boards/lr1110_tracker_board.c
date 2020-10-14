/*!
 * \file      lr1110_tracker_board.c
 *
 * \brief     Target board LR1110 tracker boards driver implementation
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
#include "lr1110_modem_lorawan.h"
#include "lr1110_tracker_board.h"
#include "lr1110.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define GNSS_LEAP_SECONDS_OFFSET 18
#define GNSS_EPOCH_SECONDS 315964800  // 6/01/1980

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*!
 * \brief modem ready flag
 */
static bool modem_is_ready = false;

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * \brief initialize the TCXO
 *
 * \param [in] context Chip implementation context
 */
static lr1110_modem_response_code_t lr1110_modem_board_init_tcxo_io( const void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC VARIABLES --------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

void lr1110_modem_board_init_io_context( void* context )
{
    ( ( lr1110_t* ) context )->reset.pin      = RADIO_RESET;
    ( ( lr1110_t* ) context )->nss.pin        = RADIO_NSS;
    ( ( lr1110_t* ) context )->event.pin      = RADIO_EVENT;
    ( ( lr1110_t* ) context )->event.callback = radio_event_callback;
    ( ( lr1110_t* ) context )->event.context  = ( ( lr1110_t* ) context );
    ( ( lr1110_t* ) context )->busy.pin       = RADIO_BUSY;
    ( ( lr1110_t* ) context )->spi.pins.miso  = RADIO_MISO;
    ( ( lr1110_t* ) context )->spi.pins.mosi  = RADIO_MOSI;
    ( ( lr1110_t* ) context )->spi.pins.sclk  = RADIO_SCLK;
    ( ( lr1110_t* ) context )->spi_id         = HAL_RADIO_SPI_ID;
}

void lr1110_modem_board_init_io( const void* context )
{
    hal_gpio_init_out( ( ( lr1110_t* ) context )->reset.pin, 1 );
    hal_gpio_init_out( ( ( lr1110_t* ) context )->nss.pin, 1 );
    hal_gpio_init_in( ( ( lr1110_t* ) context )->busy.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL );
    hal_gpio_init_in( ( ( lr1110_t* ) context )->event.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_RISING,
                      &( ( lr1110_t* ) context )->event );
}

void lr1110_modem_board_deinit_io( const void* context )
{
    hal_gpio_init_out( ( ( lr1110_t* ) context )->spi.pins.mosi, 0 );
    hal_gpio_init_out( ( ( lr1110_t* ) context )->spi.pins.miso, 0 );
    hal_gpio_init_out( ( ( lr1110_t* ) context )->spi.pins.sclk, 0 );
    hal_gpio_init_out( ( ( lr1110_t* ) context )->nss.pin, 1 );
    hal_gpio_init_out( ( ( lr1110_t* ) context )->reset.pin, 1 );
    hal_gpio_init_in( ( ( lr1110_t* ) context )->busy.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_OFF, NULL );
    hal_gpio_init_in( ( ( lr1110_t* ) context )->event.pin, HAL_GPIO_PULL_MODE_NONE, HAL_GPIO_IRQ_MODE_RISING, NULL );
}

void lr1110_modem_board_analog_deinit_io( const void* context )
{
    hal_gpio_deinit( ( ( lr1110_t* ) context )->event.pin );
    hal_gpio_deinit( ( ( lr1110_t* ) context )->busy.pin );
}

void lr1110_modem_board_set_rf_tx_power_offset( const void* context, int8_t tx_power_offset )
{
    lr1110_modem_set_tx_power_offset( context, tx_power_offset );
}

uint32_t lr1110_modem_board_get_tcxo_wakeup_time( const void* context ) { return BOARD_TCXO_WAKEUP_TIME; }

lr1110_modem_response_code_t lr1110_modem_board_init( const void* context, lr1110_modem_event_t* event )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_hal_status_t modem_hal_status = LR1110_MODEM_HAL_STATUS_OK;

    radio_event_init( event );

    modem_hal_status = lr1110_modem_hal_reset( context );
    
    if( modem_hal_status != LR1110_MODEM_HAL_STATUS_OK)
    {
        /* Something goes wrong with the lr1110 modem */
        return LR1110_MODEM_RESPONSE_CODE_FAIL;
    }

    /* Initialize TCXO control */
    modem_response_code |= lr1110_modem_board_init_tcxo_io( context );

    /* Initialize RF switch control */
    lr1110_modem_system_rf_switch_cfg_t rf_switch_cfg;
    rf_switch_cfg.enable  = LR1110_MODEM_SYSTEM_RFSW0_HIGH | LR1110_MODEM_SYSTEM_RFSW1_HIGH;
    rf_switch_cfg.standby = 0;
    /* LoRa SPDT */
    rf_switch_cfg.rx = LR1110_MODEM_SYSTEM_RFSW1_HIGH;
#if USE_RFO_LP_LF
    rf_switch_cfg.tx = LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW0_HIGH;
#else
    rf_switch_cfg.tx_hp = LR1110_MODEM_SYSTEM_RFSW1_HIGH | LR1110_MODEM_SYSTEM_RFSW0_HIGH;
#endif

    modem_response_code |= lr1110_modem_system_set_dio_as_rf_switch( context, &rf_switch_cfg );

    /* Set Pa Config */
#if USE_RFO_LP_LF
    modem_response_code = lr1110_modem_set_rf_output( context, LR1110_MODEM_RADIO_PA_SEL_LP );
#else
    modem_response_code |= lr1110_modem_set_rf_output( context, LR1110_MODEM_RADIO_PA_SEL_HP );
#endif

    modem_response_code = lr1110_modem_system_set_reg_mode( context, LR1110_MODEM_SYSTEM_REG_MODE_DCDC );

    /* Set LF Clock */
    modem_response_code |= lr1110_modem_system_cfg_lfclk( context, LR1110_MODEM_SYSTEM_LFCLK_RC, true );
    modem_response_code |= lr1110_modem_system_cfg_lfclk( context, LR1110_MODEM_SYSTEM_LFCLK_EXT, true );

    return modem_response_code;
}

uint32_t lr1110_modem_board_get_systime_from_gps( const void* context )
{
    uint32_t gps_time;
    lr1110_modem_get_gps_time( context, &gps_time );
    return ( gps_time + ( GNSS_EPOCH_SECONDS - GNSS_LEAP_SECONDS_OFFSET ) );
}

void lr1110_modem_board_lna_on( void ) { lna_on( ); }

void lr1110_modem_board_lna_off( void ) { lna_off( ); }

void lr1110_modem_board_hall_effect_enable( bool enable )
{
    if(enable == true)
    {
        external_supply_init( VCC_SENSORS_SUPPLY_MASK );
        vcc_sensors_on( );
        hall_effect_init( HALL_EFFECT_IRQ_ON );
    }
    else
    {
        /* Stop Effect hall sensors while tracker is static */
        hall_effect_deinit( );
        vcc_sensors_off( );
        external_supply_deinit( VCC_SENSORS_SUPPLY_MASK );
    }
}

lr1110_modem_response_code_t lr1110_modem_board_event_flush( const void* context )
{
    lr1110_modem_response_code_t modem_response_code = LR1110_MODEM_RESPONSE_CODE_OK;
    lr1110_modem_event_fields_t  event_fields;

    do
    {
        modem_response_code = lr1110_modem_get_event( context, &event_fields );
    } while( lr1110_modem_board_read_event_line( context ) == 1 );

    return modem_response_code;
}

bool lr1110_modem_board_read_event_line( const void* context )
{
    return hal_gpio_get_value( ( ( lr1110_t* ) context )->event.pin );
}

bool lr1110_modem_board_is_ready( void ) { return modem_is_ready; }

void lr1110_modem_board_set_ready( bool ready ) { modem_is_ready = ready; }

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

static lr1110_modem_response_code_t lr1110_modem_board_init_tcxo_io( const void* context )
{
    return lr1110_modem_system_set_tcxo_mode( context, LR1110_MODEM_SYSTEM_TCXO_CTRL_1_8V,
                                              ( lr1110_modem_board_get_tcxo_wakeup_time( context ) * 1000 ) / 30.52 );
}

/* --- EOF ------------------------------------------------------------------ */
