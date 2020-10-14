/*!
 * \file      lr1110.h
 *
 * \brief     LR1110 top level definition
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

#ifndef __LR1110_H__
#define __LR1110_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include "smtc_hal.h"
#include "lr1110_bootloader.h"
#include "lr1110_modem_common.h"
#include "lr1110_modem_hal.h"
#include "lr1110_hal.h"
#include "lr1110_modem_gnss.h"
#include "lr1110_modem_lorawan.h"
#include "lr1110_modem_system.h"
#include "lr1110_modem_wifi.h"

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
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * \brief LR1110 modem callback functions
 */
typedef struct
{
    /*!
     * \brief  Reset callback prototype.
     *
     * \param [in] reset_count
     */
    void ( *reset )( uint16_t reset_count );
    /*!
     * \brief  Alarm timer expired callback prototype.
     */
    void ( *alarm )( void );
    /*!
     * \brief  Attemp to join network failed callback prototype.
     */
    void ( *joined )( void );
    /*!
     * \brief  Joined callback prototype.
     */
    void ( *join_fail )( void );
    /*!
     * \brief  Tx done callback prototype.
     *
     * \param [in] status
     */
    void ( *tx_done )( lr1110_modem_tx_done_event_t status );
    /*!
     * \brief Downlink data received callback prototype.
     *
     * \param [in] rssi    rssi in signed value in dBm + 64
     * \param [in] snr     snr signed value in 0.25 dB steps
     * \param [in] flags   rx flags \see down_data_flag_t
     * \param [in] port    LoRaWAN port
     * \param [in] payload Received buffer pointer
     * \param [in] size    Received buffer size
     */
    void ( *down_data )( int8_t rssi, int8_t snr, lr1110_modem_down_data_flag_t flags, uint8_t port, const uint8_t* payload, uint8_t size );
    /*!
     * \brief  File upload completed callback prototype.
     *
     * \param [in] session_id
     * \param [in] session_counter
     */
    void ( *upload_done )( uint8_t session_id, uint8_t session_counter );
    /*!
     * \brief  Set conf changed by DM callback prototype.
     *
     * \param [in] info_tag
     */
    void ( *set_conf )( uint8_t info_tag );
    /*!
     * \brief  Mute callback prototype.
     *
     * \param [in] mute
     */
    void ( *mute )( lr1110_modem_mute_t mute );
    /*!
     * \brief  Data stream fragments sent callback prototype.
     */
    void ( *stream_done )( void );
    /*!
     * \brief  Gnss Done Done callback prototype.
     *
     * \param [in] nav_message
     *
     * \param [in] size
     */
    void ( *gnss_scan_done )( uint8_t* nav_message, uint16_t size );
    /*!
     * \brief  Gnss Done Done callback prototype.
     *
     * \param [in] scan buffer containing the raw data coming from the scan 
     *
     * \param [in] size of the raw buffer
     */
    void ( *wifi_scan_done )( uint8_t* scan, uint16_t size );
    /*!
     * \brief  Time Updated by application layer clock synchronization callback prototype.
     *
     * \param [in] alc_sync_state \ref lr1110_modem_alc_sync_state_t
     */
    void ( *time_updated_alc_sync )( lr1110_modem_alc_sync_state_t alc_sync_state );
    /*!
     * \brief  Automatic switch from mobile to static ADR when connection timeout occurs callback prototype.
     */
    void ( *adr_mobile_to_static )( void );
    /*!
     * \brief  New link ADR request callback prototype.
     */
    void ( *new_link_adr )( void );
    /*!
     * \brief  No event exists callback prototype.
     */
    void ( *no_event )( void );
} lr1110_modem_event_t;

/*!
 * \brief Radio operating modes
 */
typedef enum lr1110_hal_operating_mode_e
{
    LR1110_HAL_OP_MODE_SLEEP = 0x00,  //! The radio is in sleep mode
    LR1110_HAL_OP_MODE_STDBY_RC,      //! The radio is in standby mode with RC oscillator
    LR1110_HAL_OP_MODE_STDBY_XOSC,    //! The radio is in standby mode with XOSC oscillator
    LR1110_HAL_OP_MODE_FS,            //! The radio is in frequency synthesis mode
    LR1110_HAL_OP_MODE_TX,            //! The radio is in transmit mode
    LR1110_HAL_OP_MODE_RX,            //! The radio is in receive single mode
    LR1110_HAL_OP_MODE_RX_C,          //! The radio is in receive continuous mode
    LR1110_HAL_OP_MODE_RX_DC,         //! The radio is in receive duty cycle mode
    LR1110_HAL_OP_MODE_CAD            //! The radio is in channel activity detection mode
} lr1110_hal_operating_mode_t;

/*!
 * Radio hardware and global parameters
 */
typedef struct lr1110_s
{
    hal_gpio_t                  reset;
    hal_gpio_t                  busy;
    hal_gpio_irq_t              event;
    hal_gpio_t                  nss;
    hal_spi_t                   spi;
    uint32_t                    spi_id;
    lr1110_hal_operating_mode_t op_mode;
} lr1110_t;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void ( *lr1110_dio_irq_handler )( void* context );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * \brief Init the LR1110 modem event callbacks
 *
 * \param [in] event lr1110 modem event callback \ref lr1110_modem_event_t
 */
void radio_event_init( lr1110_modem_event_t* event );

/*!
 * \brief Callback when event occurs
 */
void radio_event_callback( void* obj );

/*!
 * \brief Process the analysis of radio event and calls callback functions
 *        depending on event
 *
 * \param [in] context Radio abstraction
 *
 */
void lr1110_modem_event_process( const void* context );

#ifdef __cplusplus
}
#endif

#endif  // __LR1110_H__

/* --- EOF ------------------------------------------------------------------ */
